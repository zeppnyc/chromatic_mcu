#include "fusion_savestate_stage2probe.h"

#include "driver/uart.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "crc8_sae_j1850.h"
#include "fpga_common.h"
#include "fpga_rx.h"
#include "fpga_tx.h"
#include "fusion_savestate.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwrmgr.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    kStage2FrameTimeoutMs = 3000,
    kStage2PreflightDrainMs = 80,
    kStage2PostRegionMs = 10,
    kStage2FinalEndMs = 50,
    kStage2EndSessionAttempts = 4,
    kStage2OwnerAcquireMs = 2000,
    kStage2ProgressPackets = 512,
    kStage2BadPayloadDumpBytes = 24,
    kStage2CaptureRestoreMaxDelayS = 600,
};

typedef struct {
    const char *name;
    uint8_t region;
    uint16_t length;
} Stage2Region_t;

static const char *TAG = "Stage2Probe";

static const Stage2Region_t kStage2Regions[] = {
    { "R0",   0x00u, 39u    },
    { "WRAM", 0x01u, 32768u },
    { "HRAM", 0x02u, 127u   },
};

static uint8_t s_stage2_region0[39];
static uint8_t s_stage2_wram[32768];
static uint8_t s_stage2_hram[127];
static uint32_t s_stage2_capture_crc[3];
static uint32_t s_stage2_capture_packets[3];
static uint32_t s_stage2_capture_restore_delay_s = 0u;
static bool s_stage2_capture_valid = false;

static uint8_t s_uart_cache[512];
static size_t s_uart_cache_pos = 0u;
static size_t s_uart_cache_len = 0u;

static void ResetUartCache(void)
{
    s_uart_cache_pos = 0u;
    s_uart_cache_len = 0u;
}

static bool BuildStage2Begin(uint8_t opcode, FusionV2Frame_t *out)
{
    const uint8_t payload[1] = { opcode };
    return FusionSavestate_BuildV2Frame((uint8_t)kFusionAddr_StateCtl,
                                        payload,
                                        sizeof(payload),
                                        out);
}

static bool ReadByteWithDeadline(uint8_t *out, int64_t deadline_us)
{
    while (esp_timer_get_time() < deadline_us) {
        if (s_uart_cache_pos < s_uart_cache_len) {
            *out = s_uart_cache[s_uart_cache_pos++];
            return true;
        }

        s_uart_cache_pos = 0u;
        s_uart_cache_len = 0u;

        const int got = uart_read_bytes(UART_NUM_1,
                                        s_uart_cache,
                                        sizeof(s_uart_cache),
                                        pdMS_TO_TICKS(10));
        if (got > 0) {
            s_uart_cache_len = (size_t)got;
        }
    }
    return false;
}

static void PrintBytes(const uint8_t *bytes, size_t length)
{
    for (size_t i = 0u; i < length; ++i) {
        printf(" %02X", bytes[i]);
    }
}

static uint8_t ExpectedFrameCrc(const uint8_t *frame, size_t crc_input_len)
{
    uint8_t tmp[FUSION_V2_MAX_FRAME] = {0};
    if (frame == NULL || crc_input_len >= sizeof(tmp)) {
        return 0u;
    }
    memcpy(tmp, frame, crc_input_len);
    if (crc8_sae_j1850_encode(tmp, crc_input_len, tmp) != crc_input_len + 1u) {
        return 0u;
    }
    return tmp[crc_input_len];
}

static bool IsCtlResponseOpcode(uint8_t opcode)
{
    return opcode == (uint8_t)kFusionOp_AckAccepted ||
           opcode == (uint8_t)kFusionOp_Busy ||
           opcode == (uint8_t)kFusionOp_Error ||
           opcode == (uint8_t)kFusionOp_AckDone;
}

static bool TryRecoverMissingStateCtlAddr(const char *context,
                                          const uint8_t raw[3],
                                          int64_t deadline_us,
                                          FusionV2Decoded_t *decoded)
{
    if (context == NULL || raw == NULL || decoded == NULL) {
        return false;
    }
    const uint8_t payload_len = raw[1];
    const uint8_t opcode = raw[2];
    if (payload_len == 0u ||
        payload_len > FUSION_V2_MAX_PAYLOAD ||
        !IsCtlResponseOpcode(opcode)) {
        return false;
    }

    uint8_t reconstructed[FUSION_V2_MAX_FRAME] = {0};
    reconstructed[0] = FUSION_V2_HEADER_MARKER;
    reconstructed[1] = (uint8_t)kFusionAddr_StateCtl;
    reconstructed[2] = payload_len;
    reconstructed[3] = opcode;

    for (uint8_t i = 1u; i < payload_len; ++i) {
        if (!ReadByteWithDeadline(&reconstructed[3u + i], deadline_us)) {
            return false;
        }
    }
    if (!ReadByteWithDeadline(&reconstructed[3u + payload_len], deadline_us)) {
        return false;
    }

    const size_t frame_len = 3u + payload_len + 1u;
    if (!crc8_sae_j1850_decode(reconstructed, frame_len)) {
        return false;
    }

    memset(decoded, 0, sizeof(*decoded));
    decoded->addr = (uint8_t)kFusionAddr_StateCtl;
    decoded->payload_len = payload_len;
    memcpy(decoded->payload, &reconstructed[3], payload_len);
    decoded->ctl_opcode = opcode;
    if (opcode == (uint8_t)kFusionOp_AckAccepted ||
        opcode == (uint8_t)kFusionOp_Busy ||
        opcode == (uint8_t)kFusionOp_AckDone) {
        if (payload_len >= 2u) {
            decoded->ack_for_opcode = decoded->payload[1];
        }
    } else if (opcode == (uint8_t)kFusionOp_Error) {
        if (payload_len >= 2u) {
            decoded->error_code = decoded->payload[1];
        }
        if (payload_len >= 3u) {
            decoded->error_detail = decoded->payload[2];
        }
    }
    printf("Stage2Probe: recovered CTL response missing addr during %s "
           "op=0x%02X len=%u\n",
           context,
           opcode,
           (unsigned)payload_len);
    return true;
}

static bool ReadDecodedFrame(const char *context, FusionV2Decoded_t *decoded)
{
    uint8_t raw[FUSION_V2_MAX_FRAME] = {0};
    const int64_t deadline = esp_timer_get_time()
                           + ((int64_t)kStage2FrameTimeoutMs * 1000);

    while (esp_timer_get_time() < deadline) {
        uint8_t b = 0u;
        if (!ReadByteWithDeadline(&b, deadline)) {
            printf("Stage2Probe: RX timeout waiting for marker during %s\n",
                   context);
            return false;
        }
        if (b != FUSION_V2_HEADER_MARKER) {
            continue;
        }

        raw[0] = b;
        if (!ReadByteWithDeadline(&raw[1], deadline) ||
            !ReadByteWithDeadline(&raw[2], deadline)) {
            printf("Stage2Probe: RX timeout waiting for header during %s\n",
                   context);
            return false;
        }

        const uint8_t payload_len = raw[2];
        if (payload_len > FUSION_V2_MAX_PAYLOAD) {
            if (TryRecoverMissingStateCtlAddr(context,
                                              raw,
                                              deadline,
                                              decoded)) {
                return true;
            }
            const size_t avail = (s_uart_cache_pos < s_uart_cache_len)
                ? (s_uart_cache_len - s_uart_cache_pos) : 0u;
            const size_t shown = (avail > kStage2BadPayloadDumpBytes)
                ? kStage2BadPayloadDumpBytes : avail;
            printf("Stage2Probe: RX resync bad payload len %u during %s "
                   "header=%02X %02X %02X cache_peek%u=",
                   (unsigned)payload_len,
                   context,
                   raw[0],
                   raw[1],
                   raw[2],
                   (unsigned)shown);
            PrintBytes(&s_uart_cache[s_uart_cache_pos], shown);
            printf("\n");
            continue;
        }

        const size_t frame_len = 3u + payload_len + 1u;
        for (size_t i = 3u; i < frame_len; ++i) {
            if (!ReadByteWithDeadline(&raw[i], deadline)) {
                printf("Stage2Probe: RX timeout waiting for payload/crc during %s\n",
                       context);
                return false;
            }
        }

        if (!FusionSavestate_DecodeV2Frame(raw, frame_len, decoded)) {
            const uint8_t expected_crc =
                ExpectedFrameCrc(raw, frame_len - 1u);
            printf("Stage2Probe: RX resync decode/CRC failed during %s "
                   "addr=0x%02X len=%u crc_got=%02X crc_expected=%02X raw=",
                   context,
                   raw[1],
                   (unsigned)payload_len,
                   raw[frame_len - 1u],
                   expected_crc);
            PrintBytes(raw, frame_len);
            printf("\n");
            continue;
        }
        return true;
    }

    printf("Stage2Probe: RX timeout waiting for valid frame during %s\n",
           context);
    return false;
}

static bool SendFrame(const char *label,
                      const FusionV2Frame_t *frame,
                      bool quiet)
{
    if (frame == NULL || frame->length == 0u) {
        printf("Stage2Probe: %s invalid frame\n", label);
        return false;
    }

    if (!quiet) {
        printf("Stage2Probe: TX %s:", label);
        for (uint8_t i = 0u; i < frame->length; ++i) {
            printf(" %02X", frame->bytes[i]);
        }
        printf("\n");
    }

    const int written = uart_write_bytes(UART_NUM_1,
                                         (const char *)frame->bytes,
                                         frame->length);
    const esp_err_t wait_err =
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(250));
    if (written != (int)frame->length) {
        printf("Stage2Probe: TX %s failed wrote=%d expected=%u\n",
               label,
               written,
               (unsigned)frame->length);
        return false;
    }
    if (wait_err != ESP_OK) {
        printf("Stage2Probe: TX %s wait failed err=%s\n",
               label,
               esp_err_to_name(wait_err));
        return false;
    }
    return true;
}

static bool ExpectCtl(uint8_t opcode, uint8_t ack_for, const char *context)
{
    FusionV2Decoded_t d;
    if (!ReadDecodedFrame(context, &d)) {
        return false;
    }
    if (d.addr != (uint8_t)kFusionAddr_StateCtl) {
        printf("Stage2Probe: %s expected CTL addr got 0x%02X\n",
               context,
               d.addr);
        return false;
    }
    if (d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Stage2Probe: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               context,
               d.error_code,
               d.error_detail);
        return false;
    }
    if (d.ctl_opcode != opcode || d.ack_for_opcode != ack_for) {
        printf("Stage2Probe: %s unexpected CTL op=0x%02X ack_for=0x%02X\n",
               context,
               d.ctl_opcode,
               d.ack_for_opcode);
        return false;
    }
    return true;
}

static bool SendStage2BeginAndExpectAck(uint8_t opcode, const char *label)
{
    FusionV2Frame_t frame;
    if (!BuildStage2Begin(opcode, &frame)) {
        printf("Stage2Probe: build %s failed\n", label);
        return false;
    }
    if (!SendFrame(label, &frame, false)) {
        return false;
    }
    return ExpectCtl((uint8_t)kFusionOp_AckAccepted, opcode, label);
}

static bool IsEndSessionAck(const FusionV2Decoded_t *d)
{
    return d != NULL &&
           d->addr == (uint8_t)kFusionAddr_StateCtl &&
           d->ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
           d->ack_for_opcode == (uint8_t)kFusionOp_EndSession;
}

static void DrainRxForMs(uint32_t ms)
{
    uint8_t scratch[128];
    const int64_t deadline = esp_timer_get_time() + ((int64_t)ms * 1000);
    ResetUartCache();
    while (esp_timer_get_time() < deadline) {
        (void)uart_read_bytes(UART_NUM_1,
                              scratch,
                              sizeof(scratch),
                              pdMS_TO_TICKS(2));
    }
    ResetUartCache();
}

static bool SendEndSessionClear(const char *label)
{
    FusionV2Frame_t frame;
    if (!FusionSavestate_BuildEndSession(&frame)) {
        printf("Stage2Probe: build %s failed\n", label);
        return false;
    }

    for (uint8_t attempt = 0u;
         attempt < kStage2EndSessionAttempts;
         ++attempt) {
        if (attempt != 0u) {
            DrainRxForMs(kStage2PreflightDrainMs);
            ResetUartCache();
        }
        if (!SendFrame(label, &frame, false)) {
            return false;
        }
        FusionV2Decoded_t d;
        if (ReadDecodedFrame(label, &d) && IsEndSessionAck(&d)) {
            return true;
        }
    }
    printf("Stage2Probe: %s no ACK after %u attempts; FPGA may remain paused\n",
           label,
           (unsigned)kStage2EndSessionAttempts);
    return false;
}

static void PauseFpgaTraffic(void)
{
    TaskHandle_t *tx = FPGA_GetTxTaskHandle();
    TaskHandle_t *rx = FPGA_GetRxTaskHandle();
    if (tx != NULL && *tx != NULL) {
        FPGA_Tx_Pause();
    }
    if (rx != NULL && *rx != NULL) {
        FPGA_Rx_Pause();
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    uart_flush(UART_NUM_1);
    ResetUartCache();
}

static void ResumeFpgaTraffic(void)
{
    TaskHandle_t *tx = FPGA_GetTxTaskHandle();
    TaskHandle_t *rx = FPGA_GetRxTaskHandle();
    if (rx != NULL && *rx != NULL) {
        FPGA_Rx_Resume();
    }
    if (tx != NULL && *tx != NULL) {
        FPGA_Tx_Resume();
    }
}

static bool ExpectStreamAckOrFirstData(const char *context,
                                       uint8_t ack_for,
                                       bool *pending_data_valid,
                                       FusionV2Decoded_t *pending_data)
{
    FusionV2Decoded_t first;
    if (!ReadDecodedFrame(context, &first)) {
        return false;
    }
    if (first.addr == (uint8_t)kFusionAddr_StateCtl &&
        first.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Stage2Probe: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               context,
               first.error_code,
               first.error_detail);
        return false;
    }
    if (first.addr == (uint8_t)kFusionAddr_StateCtl &&
        first.ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
        first.ack_for_opcode == ack_for) {
        return true;
    }
    if (first.addr == (uint8_t)kFusionAddr_StateData) {
        *pending_data = first;
        *pending_data_valid = true;
        printf("Stage2Probe: %s DATA arrived before ACK_ACCEPTED\n", context);
        return true;
    }

    printf("Stage2Probe: %s unexpected frame addr=0x%02X op=0x%02X seq=%u\n",
           context,
           first.addr,
           first.ctl_opcode,
           (unsigned)first.data_seq);
    return false;
}

static bool ReadStage2Region(const Stage2Region_t *region,
                             uint32_t *crc_out,
                             uint32_t *packets_out,
                             uint8_t *copy_out,
                             size_t copy_len)
{
    uint32_t received = 0u;
    uint32_t packets = 0u;
    uint16_t expected_seq = 0u;
    uint32_t crc = 0u;
    const int64_t t0 = esp_timer_get_time();

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildReadStreamBegin(region->region,
                                              0u,
                                              region->length,
                                              &begin)) {
        printf("Stage2Probe: build READ_STREAM_BEGIN %s failed\n",
               region->name);
        return false;
    }

    printf("Stage2Probe: BEGIN REGION %s id=0x%02X length=%u\n",
           region->name,
           (unsigned)region->region,
           (unsigned)region->length);
    if (!SendFrame("READ_STREAM_BEGIN", &begin, false)) {
        return false;
    }

    bool pending_data_valid = false;
    FusionV2Decoded_t pending_data;
    memset(&pending_data, 0, sizeof(pending_data));
    if (!ExpectStreamAckOrFirstData("READ_STREAM_BEGIN ack",
                                    (uint8_t)kFusionOp_ReadStreamBegin,
                                    &pending_data_valid,
                                    &pending_data)) {
        return false;
    }

    while (received < region->length) {
        FusionV2Decoded_t d;
        if (pending_data_valid) {
            d = pending_data;
            pending_data_valid = false;
        } else if (!ReadDecodedFrame(region->name, &d)) {
            return false;
        }

        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_Error) {
            printf("Stage2Probe: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
                   region->name,
                   d.error_code,
                   d.error_detail);
            return false;
        }
        if (d.addr != (uint8_t)kFusionAddr_StateData) {
            printf("Stage2Probe: %s expected DATA got addr=0x%02X op=0x%02X\n",
                   region->name,
                   d.addr,
                   d.ctl_opcode);
            return false;
        }
        if (d.data_seq != expected_seq) {
            printf("Stage2Probe: %s seq mismatch got=%u expected=%u\n",
                   region->name,
                   (unsigned)d.data_seq,
                   (unsigned)expected_seq);
            return false;
        }
        if (d.data_len == 0u ||
            (uint32_t)d.data_len > ((uint32_t)region->length - received)) {
            printf("Stage2Probe: %s bad data_len=%u remaining=%u\n",
                   region->name,
                   (unsigned)d.data_len,
                   (unsigned)((uint32_t)region->length - received));
            return false;
        }

        if (copy_out != NULL) {
            if (((size_t)received + (size_t)d.data_len) > copy_len) {
                printf("Stage2Probe: %s copy overflow received=%lu len=%u copy_len=%u\n",
                       region->name,
                       (unsigned long)received,
                       (unsigned)d.data_len,
                       (unsigned)copy_len);
                return false;
            }
            memcpy(&copy_out[received], d.data, d.data_len);
        }
        crc = FusionSavestate_Crc32Update(crc, d.data, d.data_len);
        received += d.data_len;
        packets++;
        expected_seq = (uint16_t)(expected_seq + 1u);

        if ((packets % kStage2ProgressPackets) == 0u) {
            printf("Stage2Probe: REGION %s progress packets=%lu bytes=%lu\n",
                   region->name,
                   (unsigned long)packets,
                   (unsigned long)received);
        }

        if (received < region->length) {
            FusionV2Frame_t cont;
            if (!FusionSavestate_BuildReadStreamContinue(expected_seq, &cont)) {
                printf("Stage2Probe: build READ_STREAM_CONTINUE failed seq=%u\n",
                       (unsigned)expected_seq);
                return false;
            }
            if (!SendFrame("READ_STREAM_CONTINUE", &cont, true)) {
                return false;
            }
            if (!ExpectStreamAckOrFirstData("READ_STREAM_CONTINUE ack",
                                            (uint8_t)kFusionOp_ReadStreamContinue,
                                            &pending_data_valid,
                                            &pending_data)) {
                return false;
            }
        }
    }

    if (!ExpectCtl((uint8_t)kFusionOp_AckDone,
                   (uint8_t)kFusionOp_ReadStreamBegin,
                   "READ_STREAM_BEGIN done")) {
        return false;
    }

    const int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    printf("Stage2Probe: REGION %s PASS packets=%lu bytes=%lu crc32=%08lX dt_ms=%lld\n",
           region->name,
           (unsigned long)packets,
           (unsigned long)received,
           (unsigned long)crc,
           (long long)elapsed_ms);

    if (crc_out != NULL) {
        *crc_out = crc;
    }
    if (packets_out != NULL) {
        *packets_out = packets;
    }
    return true;
}

static uint8_t *CaptureBufferForRegion(uint8_t region, size_t *length_out)
{
    if (length_out != NULL) {
        *length_out = 0u;
    }
    switch (region) {
    case 0x00u:
        if (length_out != NULL) {
            *length_out = sizeof(s_stage2_region0);
        }
        return s_stage2_region0;
    case 0x01u:
        if (length_out != NULL) {
            *length_out = sizeof(s_stage2_wram);
        }
        return s_stage2_wram;
    case 0x02u:
        if (length_out != NULL) {
            *length_out = sizeof(s_stage2_hram);
        }
        return s_stage2_hram;
    default:
        return NULL;
    }
}

static bool WriteStage2Region(const Stage2Region_t *region,
                              const uint8_t *data,
                              uint32_t *packets_out)
{
    if (region == NULL || data == NULL) {
        return false;
    }

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildWriteStreamBegin(region->region,
                                               0u,
                                               region->length,
                                               &begin)) {
        printf("Stage2Probe: build WRITE_STREAM_BEGIN %s failed\n",
               region->name);
        return false;
    }

    printf("Stage2Probe: WRITE REGION %s id=0x%02X length=%u\n",
           region->name,
           (unsigned)region->region,
           (unsigned)region->length);
    if (!SendFrame("WRITE_STREAM_BEGIN", &begin, false)) {
        return false;
    }
    if (!ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                   (uint8_t)kFusionOp_WriteStreamBegin,
                   "WRITE_STREAM_BEGIN ack")) {
        return false;
    }

    uint16_t sent = 0u;
    uint16_t seq = 0u;
    while (sent < region->length) {
        const uint16_t remaining = (uint16_t)(region->length - sent);
        const uint8_t chunk = (remaining > FUSION_STATE_DATA_MAX_DATA)
            ? FUSION_STATE_DATA_MAX_DATA
            : (uint8_t)remaining;

        FusionV2Frame_t data_frame;
        if (!FusionSavestate_BuildStateData(seq,
                                            &data[sent],
                                            chunk,
                                            &data_frame)) {
            printf("Stage2Probe: build STATE_DATA %s failed seq=%u\n",
                   region->name,
                   (unsigned)seq);
            return false;
        }
        if (!SendFrame("STATE_DATA", &data_frame, true)) {
            return false;
        }

        sent = (uint16_t)(sent + chunk);
        seq = (uint16_t)(seq + 1u);

        if (((uint32_t)seq % kStage2ProgressPackets) == 0u) {
            printf("Stage2Probe: WRITE %s progress packets=%u bytes=%u\n",
                   region->name,
                   (unsigned)seq,
                   (unsigned)sent);
        }
    }

    if (!ExpectCtl((uint8_t)kFusionOp_AckDone,
                   (uint8_t)kFusionOp_WriteStreamBegin,
                   "WRITE_STREAM_BEGIN done")) {
        return false;
    }

    printf("Stage2Probe: WRITE %s PASS packets=%u bytes=%u\n",
           region->name,
           (unsigned)seq,
           (unsigned)sent);
    if (packets_out != NULL) {
        *packets_out = seq;
    }
    return true;
}

static bool WriteCommit(void)
{
    FusionV2Frame_t commit;
    if (!FusionSavestate_BuildWriteCommit(&commit)) {
        printf("Stage2Probe: build WRITE_COMMIT failed\n");
        return false;
    }
    if (!SendFrame("WRITE_COMMIT", &commit, false)) {
        return false;
    }
    return ExpectCtl((uint8_t)kFusionOp_AckDone,
                     (uint8_t)kFusionOp_WriteCommit,
                     "WRITE_COMMIT done");
}

static bool ValidateRegion0(const uint8_t data[39])
{
    if (memcmp(data, "FUSS", 4u) != 0) {
        printf("Stage2Probe: R0 magic mismatch got=%02X %02X %02X %02X\n",
               data[0],
               data[1],
               data[2],
               data[3]);
        return false;
    }
    if (data[4] != 0x01u || data[5] != 0x00u ||
        data[6] != 0x00u || data[7] != 0x00u) {
        printf("Stage2Probe: R0 info mismatch ver=%02X flags=%02X rsv=%02X%02X\n",
               data[4],
               data[5],
               data[6],
               data[7]);
        return false;
    }

    const uint16_t pc = (uint16_t)data[8] | ((uint16_t)data[9] << 8);
    const uint16_t sp = (uint16_t)data[10] | ((uint16_t)data[11] << 8);
    printf("Stage2Probe: R0 INFO magic=FUSS version=1 PC=%04X SP=%04X "
           "AF=%02X%02X BC=%02X%02X DE=%02X%02X HL=%02X%02X\n",
           pc,
           sp,
           data[12],
           data[13],
           data[15],
           data[14],
           data[17],
           data[16],
           data[19],
           data[18]);
    return true;
}

static bool EndSessionCleanup(void)
{
    printf("Stage2Probe: sending END_SESSION cleanup\n");
    DrainRxForMs(kStage2FinalEndMs);
    ResetUartCache();
    uart_flush_input(UART_NUM_1);
    return SendEndSessionClear("END_SESSION cleanup");
}

static bool RunWithUartOwner(bool (*fn)(void))
{
    bool ok = false;

    PwrMgr_IdleTimerSuspend();
    PauseFpgaTraffic();
    FPGA_Rx_ResetParser();
    ResetUartCache();
    uart_flush_input(UART_NUM_1);

    if (!FPGA_UartOwnerAcquire(pdMS_TO_TICKS(kStage2OwnerAcquireMs))) {
        printf("Stage2Probe: UART owner unavailable\n");
        ResumeFpgaTraffic();
        PwrMgr_IdleTimerResume();
        return false;
    }

    ok = fn();

    FPGA_UartOwnerRelease();
    ResumeFpgaTraffic();
    PwrMgr_IdleTimerResume();
    return ok;
}

static bool RunSaveReadbackInner(void)
{
    bool ok = false;
    uint32_t crc[3] = {0};
    uint32_t packets[3] = {0};
    uint8_t region0[39] = {0};

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        goto done;
    }
    DrainRxForMs(kStage2PreflightDrainMs);

    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginSave,
                                     "BEGIN_SAVE_STAGE2")) {
        goto done;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    for (size_t i = 0u; i < sizeof(kStage2Regions) / sizeof(kStage2Regions[0]); ++i) {
        uint8_t *region0_out = (kStage2Regions[i].region == 0x00u) ? region0 : NULL;
        if (!ReadStage2Region(&kStage2Regions[i],
                              &crc[i],
                              &packets[i],
                              region0_out,
                              sizeof(region0))) {
            goto done;
        }
        vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    }

    if (!ValidateRegion0(region0)) {
        goto done;
    }

    printf("Stage2Probe: SAVE SUMMARY total_bytes=%u "
           "r0_crc=%08lX wram_crc=%08lX hram_crc=%08lX "
           "r0_packets=%lu wram_packets=%lu hram_packets=%lu\n",
           39u + 32768u + 127u,
           (unsigned long)crc[0],
           (unsigned long)crc[1],
           (unsigned long)crc[2],
           (unsigned long)packets[0],
           (unsigned long)packets[1],
           (unsigned long)packets[2]);
    ok = true;

done:
    if (!EndSessionCleanup()) {
        printf("Stage2Probe: SAVE final cleanup failed; treating command as FAIL\n");
        ok = false;
    }
    return ok;
}

static bool RunCaptureInner(void)
{
    bool ok = false;
    uint32_t crc[3] = {0};
    uint32_t packets[3] = {0};

    s_stage2_capture_valid = false;
    memset(s_stage2_region0, 0, sizeof(s_stage2_region0));
    memset(s_stage2_wram, 0, sizeof(s_stage2_wram));
    memset(s_stage2_hram, 0, sizeof(s_stage2_hram));

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        goto done;
    }
    DrainRxForMs(kStage2PreflightDrainMs);

    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginSave,
                                     "BEGIN_SAVE_STAGE2")) {
        goto done;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    for (size_t i = 0u; i < sizeof(kStage2Regions) / sizeof(kStage2Regions[0]); ++i) {
        size_t copy_len = 0u;
        uint8_t *copy_out =
            CaptureBufferForRegion(kStage2Regions[i].region, &copy_len);
        if (!ReadStage2Region(&kStage2Regions[i],
                              &crc[i],
                              &packets[i],
                              copy_out,
                              copy_len)) {
            goto done;
        }
        vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    }

    if (!ValidateRegion0(s_stage2_region0)) {
        goto done;
    }

    memcpy(s_stage2_capture_crc, crc, sizeof(s_stage2_capture_crc));
    memcpy(s_stage2_capture_packets, packets, sizeof(s_stage2_capture_packets));
    s_stage2_capture_valid = true;

    printf("Stage2Probe: CAPTURE SUMMARY total_bytes=%u "
           "r0_crc=%08lX wram_crc=%08lX hram_crc=%08lX "
           "r0_packets=%lu wram_packets=%lu hram_packets=%lu\n",
           39u + 32768u + 127u,
           (unsigned long)crc[0],
           (unsigned long)crc[1],
           (unsigned long)crc[2],
           (unsigned long)packets[0],
           (unsigned long)packets[1],
           (unsigned long)packets[2]);
    ok = true;

done:
    if (!EndSessionCleanup()) {
        printf("Stage2Probe: CAPTURE final cleanup failed; treating command as FAIL\n");
        s_stage2_capture_valid = false;
        ok = false;
    }
    return ok;
}

static bool RunRestoreCapturedInner(void)
{
    bool ok = false;
    uint32_t packets[3] = {0};

    if (!s_stage2_capture_valid) {
        printf("Stage2Probe: no valid capture; run 'stage2probe capture' first\n");
        return false;
    }

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        goto done;
    }
    DrainRxForMs(kStage2PreflightDrainMs);

    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD_STAGE2")) {
        goto done;
    }

    for (size_t i = 0u; i < sizeof(kStage2Regions) / sizeof(kStage2Regions[0]); ++i) {
        size_t copy_len = 0u;
        const uint8_t *data =
            CaptureBufferForRegion(kStage2Regions[i].region, &copy_len);
        if (data == NULL || copy_len < kStage2Regions[i].length) {
            printf("Stage2Probe: missing capture buffer for region 0x%02X\n",
                   (unsigned)kStage2Regions[i].region);
            goto done;
        }
        if (!WriteStage2Region(&kStage2Regions[i], data, &packets[i])) {
            goto done;
        }
        vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    }

    if (!WriteCommit()) {
        goto done;
    }

    printf("Stage2Probe: RESTORE SUMMARY total_bytes=%u "
           "r0_crc=%08lX wram_crc=%08lX hram_crc=%08lX "
           "r0_packets=%lu wram_packets=%lu hram_packets=%lu "
           "write_r0_packets=%lu write_wram_packets=%lu write_hram_packets=%lu "
           "commit=yes\n",
           39u + 32768u + 127u,
           (unsigned long)s_stage2_capture_crc[0],
           (unsigned long)s_stage2_capture_crc[1],
           (unsigned long)s_stage2_capture_crc[2],
           (unsigned long)s_stage2_capture_packets[0],
           (unsigned long)s_stage2_capture_packets[1],
           (unsigned long)s_stage2_capture_packets[2],
           (unsigned long)packets[0],
           (unsigned long)packets[1],
           (unsigned long)packets[2]);
    ok = true;

done:
    if (!EndSessionCleanup()) {
        printf("Stage2Probe: RESTORE final cleanup failed; treating command as FAIL\n");
        ok = false;
    }
    return ok;
}

static bool RunCaptureRestoreInner(void)
{
    if (!RunCaptureInner()) {
        return false;
    }

    printf("Stage2Probe: READY_FOR_GAME_CHANGE delay_s=%lu\n",
           (unsigned long)s_stage2_capture_restore_delay_s);
    for (uint32_t remaining = s_stage2_capture_restore_delay_s;
         remaining > 0u;
         --remaining) {
        if ((remaining == s_stage2_capture_restore_delay_s) ||
            (remaining <= 5u) ||
            ((remaining % 10u) == 0u)) {
            printf("Stage2Probe: CHANGE_WINDOW remaining_s=%lu\n",
                   (unsigned long)remaining);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("Stage2Probe: CHANGE_WINDOW done; starting restore\n");

    return RunRestoreCapturedInner();
}

static bool RunLoadAckInner(void)
{
    bool ok = false;

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        goto done;
    }
    DrainRxForMs(kStage2PreflightDrainMs);

    ok = SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD_STAGE2");

done:
    if (!EndSessionCleanup()) {
        printf("Stage2Probe: LOAD-ACK final cleanup failed; treating command as FAIL\n");
        ok = false;
    }
    return ok;
}

static bool RunClearInner(void)
{
    DrainRxForMs(kStage2PreflightDrainMs);
    return EndSessionCleanup();
}

static bool RunSaveReadback(void)
{
    printf("Stage2Probe: START mode=save protocol=stage2-single-byte-begin\n");
    const bool ok = RunWithUartOwner(RunSaveReadbackInner);
    printf("Stage2Probe: RESULT %s mode=save\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunClear(void)
{
    printf("Stage2Probe: START mode=clear protocol=stage2-single-byte-begin end_session_only=yes\n");
    const bool ok = RunWithUartOwner(RunClearInner);
    printf("Stage2Probe: RESULT %s mode=clear\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunLoadAck(void)
{
    printf("Stage2Probe: START mode=load-ack protocol=stage2-single-byte-begin no_commit=yes\n");
    const bool ok = RunWithUartOwner(RunLoadAckInner);
    printf("Stage2Probe: RESULT %s mode=load-ack\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunCapture(void)
{
    printf("Stage2Probe: START mode=capture protocol=stage2-single-byte-begin store_in_ram=yes\n");
    const bool ok = RunWithUartOwner(RunCaptureInner);
    printf("Stage2Probe: RESULT %s mode=capture\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunRestoreCaptured(void)
{
    printf("Stage2Probe: START mode=restore protocol=stage2-single-byte-begin commit=yes\n");
    const bool ok = RunWithUartOwner(RunRestoreCapturedInner);
    printf("Stage2Probe: RESULT %s mode=restore\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunCaptureRestore(uint32_t delay_s)
{
    s_stage2_capture_restore_delay_s = delay_s;
    printf("Stage2Probe: START mode=capture-restore "
           "protocol=stage2-single-byte-begin delay_s=%lu commit=yes\n",
           (unsigned long)delay_s);
    const bool ok = RunWithUartOwner(RunCaptureRestoreInner);
    printf("Stage2Probe: RESULT %s mode=capture-restore\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

static bool ParseDelaySeconds(const char *arg, uint32_t *out)
{
    char *end = NULL;
    unsigned long value = 0u;
    if (arg == NULL || out == NULL || arg[0] == '\0') {
        return false;
    }
    value = strtoul(arg, &end, 10);
    if (end == arg || *end != '\0' ||
        value > (unsigned long)kStage2CaptureRestoreMaxDelayS) {
        return false;
    }
    *out = (uint32_t)value;
    return true;
}

static int Stage2ProbeCommand(int argc, char **argv)
{
    if (argc < 2 || strcmp(argv[1], "save") == 0) {
        return RunSaveReadback() ? 0 : 1;
    }
    if (strcmp(argv[1], "load-ack") == 0) {
        return RunLoadAck() ? 0 : 1;
    }
    if (strcmp(argv[1], "clear") == 0) {
        return RunClear() ? 0 : 1;
    }
    if (strcmp(argv[1], "capture") == 0) {
        return RunCapture() ? 0 : 1;
    }
    if (strcmp(argv[1], "restore") == 0) {
        return RunRestoreCaptured() ? 0 : 1;
    }
    if (strcmp(argv[1], "capture-restore") == 0) {
        uint32_t delay_s = 0u;
        if (argc < 3 || !ParseDelaySeconds(argv[2], &delay_s)) {
            printf("Usage: stage2probe capture-restore <0-%u seconds>\n",
                   (unsigned)kStage2CaptureRestoreMaxDelayS);
            return 1;
        }
        return RunCaptureRestore(delay_s) ? 0 : 1;
    }

    printf("Usage: stage2probe [save|load-ack|clear|capture|restore|capture-restore]\n");
    printf("  save     : BEGIN_SAVE, read R0/WRAM/HRAM, print length + CRC; no LOAD\n");
    printf("  load-ack : BEGIN_LOAD ACK smoke only; no WRITE_STREAM, no COMMIT\n");
    printf("  clear    : END_SESSION cleanup only; use after an aborted/failed probe\n");
    printf("  capture  : BEGIN_SAVE, read R0/WRAM/HRAM into MCU RAM; no LOAD\n");
    printf("  restore  : replay captured R0/WRAM/HRAM through BEGIN_LOAD, WRITE_STREAM, WRITE_COMMIT\n");
    printf("  capture-restore <seconds> : capture, wait, restore+WRITE_COMMIT in one console command\n");
    return 1;
}

void FusionSavestateStage2Probe_RegisterCommands(void)
{
    const esp_console_cmd_t command = {
        .command = "stage2probe",
        .help = "Stage 2 probe: save/clear/capture/restore three-region stream",
        .func = &Stage2ProbeCommand,
        .argtable = NULL,
    };
    const esp_err_t err = esp_console_cmd_register(&command);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Registering '%s' failed: %s",
                 command.command,
                 esp_err_to_name(err));
    }
}
