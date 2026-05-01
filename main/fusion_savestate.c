#include "fusion_savestate.h"

#include "crc8_sae_j1850.h"

#include <string.h>

/*
 * Fusion savestate protocol/format primitives — implementation.
 *
 * No FreeRTOS, no ESP-IDF: this object compiles cleanly on the host
 * for unit tests and links into the firmware via the standard ESP-IDF
 * component build (see main/CMakeLists.txt).
 */

/* ---- V2 frame helpers ---- */

bool FusionSavestate_BuildV2Frame(uint8_t addr,
                                  const uint8_t *payload,
                                  size_t payload_len,
                                  FusionV2Frame_t *out)
{
    if (out == NULL) {
        return false;
    }
    if ((addr & 0x80u) != 0u) {
        /* V2 wrapper uses 7-bit address space; bit 7 collides with the marker. */
        return false;
    }
    if (payload_len > FUSION_V2_MAX_PAYLOAD) {
        return false;
    }
    if (payload_len > 0u && payload == NULL) {
        return false;
    }

    uint8_t *b = out->bytes;
    b[0] = FUSION_V2_HEADER_MARKER;
    b[1] = addr;
    b[2] = (uint8_t)payload_len;
    if (payload_len > 0u) {
        memcpy(&b[3], payload, payload_len);
    }

    const size_t crc_in = 3u + payload_len;
    const size_t total  = crc8_sae_j1850_encode(b, crc_in, b);
    if (total != crc_in + 1u) {
        out->length = 0u;
        return false;
    }
    out->length = (uint8_t)total;
    return true;
}

bool FusionSavestate_DecodeV2Frame(const uint8_t *frame,
                                   size_t frame_len,
                                   FusionV2Decoded_t *out)
{
    if (frame == NULL || out == NULL) {
        return false;
    }
    if (frame_len < FUSION_V2_FRAME_OVERHEAD || frame_len > FUSION_V2_MAX_FRAME) {
        return false;
    }
    if (frame[0] != FUSION_V2_HEADER_MARKER) {
        return false;
    }
    const uint8_t addr = frame[1];
    if ((addr & 0x80u) != 0u) {
        return false;
    }
    const uint8_t plen = frame[2];
    if (plen > FUSION_V2_MAX_PAYLOAD) {
        return false;
    }
    if (frame_len != (size_t)(3u + plen + 1u)) {
        return false;
    }
    if (!crc8_sae_j1850_decode(frame, frame_len)) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->addr = addr;
    out->payload_len = plen;
    if (plen > 0u) {
        memcpy(out->payload, &frame[3], plen);
    }

    if (addr == kFusionAddr_StateCtl && plen >= 1u) {
        out->ctl_opcode = out->payload[0];
        switch (out->ctl_opcode) {
        case kFusionOp_AckAccepted:
        case kFusionOp_Busy:
        case kFusionOp_AckDone:
            if (plen >= 2u) {
                out->ack_for_opcode = out->payload[1];
            }
            break;
        case kFusionOp_Error:
            if (plen >= 2u) {
                out->error_code = out->payload[1];
            }
            if (plen >= 3u) {
                out->error_detail = out->payload[2];
            }
            break;
        default:
            break;
        }
    } else if (addr == kFusionAddr_StateData) {
        if (plen < FUSION_STATE_DATA_HEADER_BYTES) {
            return false;
        }
        out->data_seq = (uint16_t)out->payload[0]
                      | ((uint16_t)out->payload[1] << 8);
        out->data_len = (uint8_t)(plen - FUSION_STATE_DATA_HEADER_BYTES);
        if (out->data_len > FUSION_STATE_DATA_MAX_DATA) {
            return false;
        }
        if (out->data_len > 0u) {
            memcpy(out->data, &out->payload[2], out->data_len);
        }
    }
    return true;
}

/* ---- STATE_CTL builder helpers ---- */

static bool BuildCtl(const uint8_t *payload, size_t payload_len, FusionV2Frame_t *out)
{
    return FusionSavestate_BuildV2Frame((uint8_t)kFusionAddr_StateCtl,
                                        payload, payload_len, out);
}

bool FusionSavestate_BuildBeginSave(uint8_t flags, FusionV2Frame_t *out)
{
    const uint8_t p[2] = { (uint8_t)kFusionOp_BeginSave, flags };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildBeginLoad(uint8_t flags, FusionV2Frame_t *out)
{
    const uint8_t p[2] = { (uint8_t)kFusionOp_BeginLoad, flags };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildBeginTestRW(uint8_t flags, FusionV2Frame_t *out)
{
    const uint8_t p[2] = { (uint8_t)kFusionOp_BeginTestRW, flags };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildEndSession(FusionV2Frame_t *out)
{
    const uint8_t p[1] = { (uint8_t)kFusionOp_EndSession };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildSeek(uint8_t region, uint16_t offset, FusionV2Frame_t *out)
{
    const uint8_t p[4] = {
        (uint8_t)kFusionOp_Seek,
        region,
        (uint8_t)(offset & 0xFFu),
        (uint8_t)((offset >> 8) & 0xFFu),
    };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildReadNext(uint8_t count, FusionV2Frame_t *out)
{
    if (count == 0u || count > FUSION_STATE_DATA_MAX_DATA) {
        return false;
    }
    const uint8_t p[2] = { (uint8_t)kFusionOp_ReadNext, count };
    return BuildCtl(p, sizeof(p), out);
}

static bool BuildStreamBegin(uint8_t opcode,
                             uint8_t region,
                             uint16_t offset,
                             uint16_t length,
                             FusionV2Frame_t *out)
{
    const uint8_t p[6] = {
        opcode,
        region,
        (uint8_t)(offset & 0xFFu),
        (uint8_t)((offset >> 8) & 0xFFu),
        (uint8_t)(length & 0xFFu),
        (uint8_t)((length >> 8) & 0xFFu),
    };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildReadStreamBegin(uint8_t region,
                                          uint16_t offset,
                                          uint16_t length,
                                          FusionV2Frame_t *out)
{
    return BuildStreamBegin((uint8_t)kFusionOp_ReadStreamBegin,
                            region, offset, length, out);
}

bool FusionSavestate_BuildReadStreamContinue(uint16_t next_seq,
                                             FusionV2Frame_t *out)
{
    const uint8_t p[3] = {
        (uint8_t)kFusionOp_ReadStreamContinue,
        (uint8_t)(next_seq & 0xFFu),
        (uint8_t)((next_seq >> 8) & 0xFFu),
    };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildWriteStreamBegin(uint8_t region,
                                           uint16_t offset,
                                           uint16_t length,
                                           FusionV2Frame_t *out)
{
    return BuildStreamBegin((uint8_t)kFusionOp_WriteStreamBegin,
                            region, offset, length, out);
}

bool FusionSavestate_BuildWriteCommit(FusionV2Frame_t *out)
{
    const uint8_t p[1] = { (uint8_t)kFusionOp_WriteCommit };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildStateData(uint16_t seq,
                                    const uint8_t *data,
                                    size_t data_len,
                                    FusionV2Frame_t *out)
{
    if (data_len > FUSION_STATE_DATA_MAX_DATA) {
        return false;
    }
    if (data_len > 0u && data == NULL) {
        return false;
    }
    uint8_t buf[FUSION_STATE_DATA_HEADER_BYTES + FUSION_STATE_DATA_MAX_DATA];
    buf[0] = (uint8_t)(seq & 0xFFu);
    buf[1] = (uint8_t)((seq >> 8) & 0xFFu);
    if (data_len > 0u) {
        memcpy(&buf[2], data, data_len);
    }
    return FusionSavestate_BuildV2Frame((uint8_t)kFusionAddr_StateData,
                                        buf,
                                        FUSION_STATE_DATA_HEADER_BYTES + data_len,
                                        out);
}

bool FusionSavestate_BuildAckAccepted(uint8_t for_opcode, FusionV2Frame_t *out)
{
    const uint8_t p[2] = { (uint8_t)kFusionOp_AckAccepted, for_opcode };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildBusy(uint8_t for_opcode, FusionV2Frame_t *out)
{
    const uint8_t p[2] = { (uint8_t)kFusionOp_Busy, for_opcode };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildAckDone(uint8_t for_opcode, FusionV2Frame_t *out)
{
    const uint8_t p[2] = { (uint8_t)kFusionOp_AckDone, for_opcode };
    return BuildCtl(p, sizeof(p), out);
}

bool FusionSavestate_BuildError(uint8_t code, uint8_t detail, FusionV2Frame_t *out)
{
    const uint8_t p[3] = { (uint8_t)kFusionOp_Error, code, detail };
    return BuildCtl(p, sizeof(p), out);
}

/* ---- CRC-32/ISO-HDLC (poly 0xEDB88320 reversed, init 0xFFFFFFFF, xor-out 0xFFFFFFFF) ---- */

uint32_t FusionSavestate_Crc32Update(uint32_t prior_crc, const uint8_t *data, size_t len)
{
    uint32_t crc = prior_crc ^ 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8u; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

uint32_t FusionSavestate_Crc32(const uint8_t *data, size_t len)
{
    return FusionSavestate_Crc32Update(0u, data, len);
}

uint32_t FusionSavestate_ComputeGameIdV1(const uint8_t rom_header[FUSION_ROM_HEADER_BYTES])
{
    if (rom_header == NULL) {
        return 0u;
    }
    return FusionSavestate_Crc32(rom_header, FUSION_ROM_HEADER_BYTES);
}
