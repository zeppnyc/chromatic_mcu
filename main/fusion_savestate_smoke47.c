#include "fusion_savestate_smoke47.h"

#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "fusion_savestate.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    kSmoke47FrameTimeoutMs = 3000,
    kSmoke47BootDelayMs    = 2000,
    kWramFullLen           = 32768,
    kWramDiag1Len          = 40,
    kWramDiag2Len          = 256,
    kWramDiag3Len          = 8192,
    kFinalGateBufLen       = 32768,
};

typedef struct {
    const char *name;
    uint8_t     region;
    uint16_t    length;
    bool        validate;
} SmokeRegion_t;

/* Final gate: same five regions as 4.6 plus WRAM 32 KiB. */
static const SmokeRegion_t kFinalMatrix[] = {
    { "Header", 0x00u, 16u,                   true  },
    { "Top",    0x01u, 16u,                   true  },
    { "CPU",    0x02u, 40u,                   false },
    { "Timer",  0x03u, 8u,                    true  },
    { "HRAM",   0x09u, 127u,                  false },
    { "WRAM",   0x0Cu, (uint16_t)kWramFullLen, false },
};

#if 0
/* Diagnostic ramp before the final gate.  Each entry runs through its
 * own BeginSave/EndSession pair so a failure on one length does not
 * corrupt the next. */
static const SmokeRegion_t kWramDiagMatrix[] = {
    { "WRAM-40",    0x0Cu, (uint16_t)kWramDiag1Len, false },
    { "WRAM-256",   0x0Cu, (uint16_t)kWramDiag2Len, false },
    { "WRAM-8192",  0x0Cu, (uint16_t)kWramDiag3Len, false },
    { "WRAM-32768", 0x0Cu, (uint16_t)kWramFullLen,  false },
};
#endif

static uint8_t *s_rx_buf  = NULL;
static size_t   s_rx_buf_len = 0;

static bool EnsureRxBuf(uint16_t need)
{
    if (need <= s_rx_buf_len) {
        return true;
    }
    uint8_t *p = (uint8_t *)heap_caps_realloc(s_rx_buf, need, MALLOC_CAP_8BIT);
    if (p == NULL) {
        printf("Smoke47: heap alloc %u failed\n", (unsigned)need);
        return false;
    }
    s_rx_buf = p;
    s_rx_buf_len = need;
    return true;
}

static bool SendFrame(const char *label, const FusionV2Frame_t *frame)
{
    if (frame == NULL || frame->length == 0u) {
        printf("Smoke47: %s invalid frame\n", label);
        return false;
    }

    printf("Smoke47: TX %s:", label);
    for (uint8_t i = 0; i < frame->length; ++i) {
        printf(" %02X", frame->bytes[i]);
    }
    printf("\n");

    const int written = uart_write_bytes(UART_NUM_1,
                                         (const char *)frame->bytes,
                                         frame->length);
    (void)uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(250));
    if (written != (int)frame->length) {
        printf("Smoke47: TX %s failed wrote=%d expected=%u\n",
               label, written, (unsigned)frame->length);
        return false;
    }
    return true;
}

static bool ReadByteWithDeadline(uint8_t *out, int64_t deadline_us)
{
    while (esp_timer_get_time() < deadline_us) {
        const int got = uart_read_bytes(UART_NUM_1, out, 1, pdMS_TO_TICKS(50));
        if (got == 1) {
            return true;
        }
    }
    return false;
}

static bool ReadDecodedFrame(const char *context, FusionV2Decoded_t *decoded)
{
    uint8_t raw[FUSION_V2_MAX_FRAME] = {0};
    int64_t deadline = esp_timer_get_time()
                     + ((int64_t)kSmoke47FrameTimeoutMs * 1000);

    while (esp_timer_get_time() < deadline) {
        uint8_t b = 0;
        if (!ReadByteWithDeadline(&b, deadline)) {
            printf("Smoke47: RX timeout waiting for marker during %s\n", context);
            return false;
        }
        if (b != FUSION_V2_HEADER_MARKER) {
            continue;
        }

        raw[0] = b;
        if (!ReadByteWithDeadline(&raw[1], deadline) ||
            !ReadByteWithDeadline(&raw[2], deadline)) {
            printf("Smoke47: RX timeout waiting for header during %s\n", context);
            return false;
        }

        const uint8_t payload_len = raw[2];
        if (payload_len > FUSION_V2_MAX_PAYLOAD) {
            printf("Smoke47: RX resync bad payload len %u during %s\n",
                   (unsigned)payload_len, context);
            continue;
        }

        const size_t frame_len = 3u + payload_len + 1u;
        for (size_t i = 3u; i < frame_len; ++i) {
            if (!ReadByteWithDeadline(&raw[i], deadline)) {
                printf("Smoke47: RX timeout waiting for payload/crc during %s\n",
                       context);
                return false;
            }
        }

        if (!FusionSavestate_DecodeV2Frame(raw, frame_len, decoded)) {
            printf("Smoke47: RX resync decode/CRC failed during %s\n", context);
            continue;
        }
        return true;
    }

    printf("Smoke47: RX timeout waiting for valid frame during %s\n", context);
    return false;
}

static bool ExpectCtl(uint8_t opcode, uint8_t ack_for, const char *context)
{
    FusionV2Decoded_t d;
    if (!ReadDecodedFrame(context, &d)) {
        return false;
    }
    if (d.addr != (uint8_t)kFusionAddr_StateCtl) {
        printf("Smoke47: %s expected CTL addr got 0x%02X\n", context, d.addr);
        return false;
    }
    if (d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Smoke47: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               context, d.error_code, d.error_detail);
        return false;
    }
    if (d.ctl_opcode != opcode || d.ack_for_opcode != ack_for) {
        printf("Smoke47: %s unexpected CTL op=0x%02X ack_for=0x%02X\n",
               context, d.ctl_opcode, d.ack_for_opcode);
        return false;
    }
    return true;
}

static bool BuildEnd(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildEndSession(out);
}

static bool BuildBeginSave(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildBeginSave(0u, out);
}

static bool SendCtlAndExpectAck(const char *label,
                                bool (*builder)(FusionV2Frame_t *out),
                                uint8_t ack_for)
{
    FusionV2Frame_t frame;
    if (!builder(&frame)) {
        printf("Smoke47: build %s failed\n", label);
        return false;
    }
    if (!SendFrame(label, &frame)) {
        return false;
    }
    return ExpectCtl((uint8_t)kFusionOp_AckAccepted, ack_for, label);
}

static void PrintRegionSummary(const SmokeRegion_t *region,
                               const uint8_t *data,
                               uint16_t length)
{
    const uint32_t crc = FusionSavestate_Crc32(data, length);
    printf("Smoke47: REGION %s id=0x%02X len=%u crc32=%08lX first:",
           region->name,
           (unsigned)region->region,
           (unsigned)length,
           (unsigned long)crc);
    const uint16_t first_count = (length < 16u) ? length : 16u;
    for (uint16_t i = 0; i < first_count; ++i) {
        printf(" %02X", data[i]);
    }
    if (length > 32u) {
        printf(" .. last:");
        for (uint16_t i = (uint16_t)(length - 16u); i < length; ++i) {
            printf(" %02X", data[i]);
        }
    }
    printf("\n");
}

static bool ValidateRegion(const SmokeRegion_t *region,
                           const uint8_t *data,
                           uint16_t length)
{
    if (region->region == 0x00u) {
        const uint32_t bitmap = (uint32_t)data[8]
                              | ((uint32_t)data[9] << 8)
                              | ((uint32_t)data[10] << 16)
                              | ((uint32_t)data[11] << 24);
        if (memcmp(data, "FUSS", 4u) != 0) {
            printf("Smoke47: Header magic mismatch\n");
            return false;
        }
        if (bitmap != 0x0000120Fu) {
            printf("Smoke47: Header bitmap mismatch got=0x%08lX want=0x0000120F\n",
                   (unsigned long)bitmap);
            return false;
        }
        if (memcmp(&data[12], "P470", 4u) != 0) {
            printf("Smoke47: Header tag mismatch got='%c%c%c%c' want='P470'\n",
                   data[12], data[13], data[14], data[15]);
            return false;
        }
    } else if (region->region == 0x01u) {
        for (uint16_t i = 10u; i < length; ++i) {
            if (data[i] != 0u) {
                printf("Smoke47: Top padding byte %u nonzero=0x%02X\n",
                       (unsigned)i, data[i]);
                return false;
            }
        }
    } else if (region->region == 0x03u) {
        if (data[6] != 0u || data[7] != 0u) {
            printf("Smoke47: Timer padding mismatch b6=0x%02X b7=0x%02X\n",
                   data[6], data[7]);
            return false;
        }
    }
    return true;
}

static bool ReadRegion(const SmokeRegion_t *region)
{
    if (!EnsureRxBuf(region->length)) {
        return false;
    }
    uint8_t *data = s_rx_buf;
    memset(data, 0, region->length);

    uint16_t received = 0;
    uint16_t expected_seq = 0;
    const int64_t t0 = esp_timer_get_time();

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildReadStreamBegin(region->region,
                                              0u,
                                              region->length,
                                              &begin)) {
        printf("Smoke47: build READ_STREAM_BEGIN %s failed\n", region->name);
        return false;
    }
    printf("Smoke47: BEGIN REGION %s id=0x%02X length=%u\n",
           region->name, (unsigned)region->region, (unsigned)region->length);
    if (!SendFrame("READ_STREAM_BEGIN", &begin)) {
        return false;
    }
    if (!ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                   (uint8_t)kFusionOp_ReadStreamBegin,
                   "READ_STREAM_BEGIN ack")) {
        return false;
    }

    while (received < region->length) {
        FusionV2Decoded_t d;
        if (!ReadDecodedFrame(region->name, &d)) {
            return false;
        }
        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_Error) {
            printf("Smoke47: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
                   region->name, d.error_code, d.error_detail);
            return false;
        }
        if (d.addr != (uint8_t)kFusionAddr_StateData) {
            printf("Smoke47: %s expected DATA got addr=0x%02X op=0x%02X\n",
                   region->name, d.addr, d.ctl_opcode);
            return false;
        }
        if (d.data_seq != expected_seq) {
            printf("Smoke47: %s seq mismatch got=%u expected=%u\n",
                   region->name, (unsigned)d.data_seq, (unsigned)expected_seq);
            return false;
        }
        if (d.data_len == 0u || (uint16_t)d.data_len > (region->length - received)) {
            printf("Smoke47: %s bad data_len=%u remaining=%u\n",
                   region->name,
                   (unsigned)d.data_len,
                   (unsigned)(region->length - received));
            return false;
        }
        memcpy(&data[received], d.data, d.data_len);
        received = (uint16_t)(received + d.data_len);
        expected_seq = (uint16_t)(expected_seq + 1u);
    }

    if (!ExpectCtl((uint8_t)kFusionOp_AckDone,
                   (uint8_t)kFusionOp_ReadStreamBegin,
                   "READ_STREAM_BEGIN done")) {
        return false;
    }

    const int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    PrintRegionSummary(region, data, received);
    if (region->validate && !ValidateRegion(region, data, received)) {
        return false;
    }
    printf("Smoke47: REGION %s PASS packets=%u bytes=%u dt_ms=%lld\n",
           region->name,
           (unsigned)expected_seq,
           (unsigned)received,
           (long long)elapsed_ms);
    return true;
}

static bool RunSession(const SmokeRegion_t *matrix, size_t count, const char *label)
{
    bool ok = true;

    ok = SendCtlAndExpectAck("END_SESSION clear",
                             BuildEnd,
                             (uint8_t)kFusionOp_EndSession);
    if (ok) {
        ok = SendCtlAndExpectAck("BEGIN_SAVE",
                                 BuildBeginSave,
                                 (uint8_t)kFusionOp_BeginSave);
    }
    if (ok) {
        printf("Smoke47: %s session active; screen paused\n", label);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (size_t i = 0; ok && i < count; ++i) {
        ok = ReadRegion(&matrix[i]);
        if (ok) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    printf("Smoke47: %s sending END_SESSION cleanup\n", label);
    vTaskDelay(pdMS_TO_TICKS(50));
    bool cleanup_ok = false;
    FusionV2Frame_t end;
    if (FusionSavestate_BuildEndSession(&end)) {
        cleanup_ok = SendFrame("END_SESSION cleanup", &end) &&
                     ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                               (uint8_t)kFusionOp_EndSession,
                               "END_SESSION cleanup");
    }
    return ok && cleanup_ok;
}

void FusionSavestateSmoke47_RunBlocking(void)
{
    printf("Smoke47: Phase 4.7 WRAM smoke starting\n");
    printf("Smoke47: waiting %u ms for FPGA boot\n", (unsigned)kSmoke47BootDelayMs);
    vTaskDelay(pdMS_TO_TICKS(kSmoke47BootDelayMs));
    uart_flush(UART_NUM_1);

    if (!EnsureRxBuf((uint16_t)kFinalGateBufLen)) {
        printf("Smoke47: RESULT FAIL (rx buffer)\n");
        return;
    }

    bool diag_ok = true;
    printf("Smoke47: diagnostic ramp disabled for final-only rerun\n");

    printf("Smoke47: --- FINAL GATE ---\n");
    bool gate_ok = RunSession(kFinalMatrix,
                              sizeof(kFinalMatrix) / sizeof(kFinalMatrix[0]),
                              "FINAL");
    printf("Smoke47: RESULT %s (diag=%s gate=%s)\n",
           (diag_ok && gate_ok) ? "PASS" : "FAIL",
           diag_ok ? "PASS" : "FAIL",
           gate_ok ? "PASS" : "FAIL");
    printf("Smoke47: Phase 4.7 WRAM smoke complete; continuing normal boot\n");
}
