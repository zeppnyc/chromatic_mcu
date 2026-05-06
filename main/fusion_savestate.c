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

/*
 * CPU region (region 0x02) saved-slot byte offsets per FPGA mapping
 * (verified against T80.vhd, T80_Reg.vhd, GBse.vhd):
 *
 *   offset 0-1   GBSE state
 *   offset 2-9   T80_Reg CPUREGS file (RegsL/H 0-3)
 *     2: RegsL(0) = C
 *     3: RegsL(1) = E
 *     4: RegsL(2) = L
 *     5: RegsL(3) = unused (GB only uses 0-2 of L bank)
 *     6: RegsH(0) = B
 *     7: RegsH(1) = D
 *     8: RegsH(2) = H
 *     9: RegsH(3) = unused
 *   offset 10-17 T80 SS_1
 *     10-11: PC (lo, hi)
 *     12-13: address bus latch (NOT visible A)
 *   offset 18-24 T80 SS_2
 *     19: SS_2[15:8] = ACC (visible A)
 *   offset 25-32 T80 SS_3
 *     25-26: SP (lo, hi)
 *     27: SS_3[23:16] = Read_To_Reg_r + low F bits (always 0 in GB mode)
 *     28: SS_3[31:24] - bits 4-1 = F[7:4] = Z/N/H/C; bits 7-5 = Arith16_r etc
 *     31: SS_3[55:48] - bit 0 = Halt_FF, bit 4 = IntE_FF1 (saved_IFF)
 *   offset 33-39 T80 SS_4 (RegBus/Bus latches, not used by path-e)
 *
 * F register: GB mode T80 hardware-clamps F[3:0]=0 (T80.vhd line 897-899).
 * Non-byte-aligned extraction:
 *   F[7] = byte28 bit 4 (Z)
 *   F[6] = byte28 bit 3 (N)
 *   F[5] = byte28 bit 2 (H)
 *   F[4] = byte28 bit 1 (C)
 *   F[3:0] = 0
 *   => F = (byte28 << 3) & 0xF0
 */
#define PATHE_CPU_REGION_MIN_BYTES  32u
#define PATHE_CPU_OFF_C             2u
#define PATHE_CPU_OFF_E             3u
#define PATHE_CPU_OFF_L             4u
#define PATHE_CPU_OFF_B             6u
#define PATHE_CPU_OFF_D             7u
#define PATHE_CPU_OFF_H             8u
#define PATHE_CPU_OFF_PC_LO         10u
#define PATHE_CPU_OFF_PC_HI         11u
#define PATHE_CPU_OFF_ACC           19u
#define PATHE_CPU_OFF_SP_LO         25u
#define PATHE_CPU_OFF_SP_HI         26u
#define PATHE_CPU_OFF_F_BYTE        28u
#define PATHE_CPU_OFF_HALT_IFF_BYTE 31u
#define PATHE_HALT_FF_BIT           0x01u
#define PATHE_IFF_FF1_BIT           0x10u  /* SS_3[52] = bit 4 of byte 31 */

bool FusionSavestate_ExtractPathELoadInputFromCpuRegion(
    const uint8_t *cpu_region_bytes,
    size_t cpu_region_len,
    FusionPathELoadInput_t *out)
{
    if (cpu_region_bytes == NULL || out == NULL) {
        return false;
    }
    if (cpu_region_len < PATHE_CPU_REGION_MIN_BYTES) {
        return false;
    }

    out->saved_C = cpu_region_bytes[PATHE_CPU_OFF_C];
    out->saved_E = cpu_region_bytes[PATHE_CPU_OFF_E];
    out->saved_L = cpu_region_bytes[PATHE_CPU_OFF_L];
    out->saved_B = cpu_region_bytes[PATHE_CPU_OFF_B];
    out->saved_D = cpu_region_bytes[PATHE_CPU_OFF_D];
    out->saved_H = cpu_region_bytes[PATHE_CPU_OFF_H];

    out->saved_A = cpu_region_bytes[PATHE_CPU_OFF_ACC];

    /* F: extract bits Z/N/H/C from byte28 bits 4-1, F[3:0]=0 */
    const uint8_t byte28 = cpu_region_bytes[PATHE_CPU_OFF_F_BYTE];
    out->saved_F = (uint8_t)((byte28 << 3) & 0xF0u);

    out->saved_PC = (uint16_t)(cpu_region_bytes[PATHE_CPU_OFF_PC_LO] |
                               ((uint16_t)cpu_region_bytes[PATHE_CPU_OFF_PC_HI] << 8));
    out->saved_SP = (uint16_t)(cpu_region_bytes[PATHE_CPU_OFF_SP_LO] |
                               ((uint16_t)cpu_region_bytes[PATHE_CPU_OFF_SP_HI] << 8));

    const uint8_t halt_iff_byte = cpu_region_bytes[PATHE_CPU_OFF_HALT_IFF_BYTE];
    out->halted_at_save = (halt_iff_byte & PATHE_HALT_FF_BIT) != 0u;
    out->saved_IFF      = (halt_iff_byte & PATHE_IFF_FF1_BIT) != 0u;

    return true;
}

bool FusionSavestate_BuildPathELoadPayload(const FusionPathELoadInput_t *state,
                                            uint8_t out[kPathE_RegionLength])
{
    if (state == NULL || out == NULL) {
        return false;
    }

    /*
     * HALT-during-save adjustment per Gate 1 contract §4:
     *   if Halt_FF=1 at save time, saved_PC := captured_PC - 1
     * so the CPU re-executes the HALT opcode and naturally re-enters
     * HALT mode on load.
     *
     * Reject the (rare) case where halted_at_save=true and saved_PC=0
     * since PC-1 would underflow.  A real GB cannot be in HALT at
     * PC=$0000 in practice, but explicit rejection is safer than
     * letting the load proceed with PC=$FFFF (cart bus high region).
     */
    uint16_t adjusted_pc = state->saved_PC;
    if (state->halted_at_save) {
        if (state->saved_PC == 0u) {
            return false;
        }
        adjusted_pc = (uint16_t)(state->saved_PC - 1u);
    }

    out[kPathE_LoadModeActive] = 1u;       /* arm load mode */
    out[kPathE_ScratchF]       = state->saved_F;
    out[kPathE_ScratchA]       = state->saved_A;
    out[kPathE_ScratchC]       = state->saved_C;
    out[kPathE_ScratchB]       = state->saved_B;
    out[kPathE_ScratchE]       = state->saved_E;
    out[kPathE_ScratchD]       = state->saved_D;
    out[kPathE_ScratchL]       = state->saved_L;
    out[kPathE_ScratchH]       = state->saved_H;
    out[kPathE_SavedSpLo]      = (uint8_t)(state->saved_SP & 0xFFu);
    out[kPathE_SavedSpHi]      = (uint8_t)((state->saved_SP >> 8) & 0xFFu);
    out[kPathE_IffByte]        = state->saved_IFF ? FUSION_PATHE_IFF_BYTE_EI
                                                  : FUSION_PATHE_IFF_BYTE_NOP;
    out[kPathE_SavedA]         = state->saved_A;  /* re-restore A after FF50 */
    out[kPathE_SavedPcLo]      = (uint8_t)(adjusted_pc & 0xFFu);
    out[kPathE_SavedPcHi]      = (uint8_t)((adjusted_pc >> 8) & 0xFFu);
    out[kPathE_GbresetRequest] = 0u;       /* caller toggles via separate writes */

    return true;
}
