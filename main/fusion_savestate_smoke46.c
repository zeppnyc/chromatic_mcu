#include "fusion_savestate_smoke46.h"

#include "driver/uart.h"
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
    kSmoke46FrameTimeoutMs = 3000,
    kSmoke46BootDelayMs = 2000,
};

typedef struct {
    const char *name;
    uint8_t region;
    uint16_t length;
    bool validate;
} SmokeRegion_t;

static const SmokeRegion_t kMatrix[] = {
    { "Header", 0x00u, 16u,  true },
    { "Top",    0x01u, 16u,  true },
    { "CPU",    0x02u, 40u,  false },
    { "Timer",  0x03u, 8u,   true },
    { "HRAM",   0x09u, 127u, false },
};

static bool SendFrame(const char *label, const FusionV2Frame_t *frame)
{
    if (frame == NULL || frame->length == 0u) {
        printf("Smoke46: %s invalid frame\n", label);
        return false;
    }

    printf("Smoke46: TX %s:", label);
    for (uint8_t i = 0; i < frame->length; ++i) {
        printf(" %02X", frame->bytes[i]);
    }
    printf("\n");

    const int written = uart_write_bytes(UART_NUM_1,
                                         (const char *)frame->bytes,
                                         frame->length);
    (void)uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(250));
    if (written != (int)frame->length) {
        printf("Smoke46: TX %s failed wrote=%d expected=%u\n",
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
                     + ((int64_t)kSmoke46FrameTimeoutMs * 1000);

    uint8_t b = 0;
    do {
        if (!ReadByteWithDeadline(&b, deadline)) {
            printf("Smoke46: RX timeout waiting for marker during %s\n", context);
            return false;
        }
    } while (b != FUSION_V2_HEADER_MARKER);

    raw[0] = b;
    if (!ReadByteWithDeadline(&raw[1], deadline) ||
        !ReadByteWithDeadline(&raw[2], deadline)) {
        printf("Smoke46: RX timeout waiting for header during %s\n", context);
        return false;
    }

    const uint8_t payload_len = raw[2];
    if (payload_len > FUSION_V2_MAX_PAYLOAD) {
        printf("Smoke46: RX bad payload len %u during %s\n",
               (unsigned)payload_len, context);
        return false;
    }

    const size_t frame_len = 3u + payload_len + 1u;
    for (size_t i = 3u; i < frame_len; ++i) {
        if (!ReadByteWithDeadline(&raw[i], deadline)) {
            printf("Smoke46: RX timeout waiting for payload/crc during %s\n",
                   context);
            return false;
        }
    }

    printf("Smoke46: RX %s:", context);
    for (size_t i = 0; i < frame_len; ++i) {
        printf(" %02X", raw[i]);
    }
    printf("\n");

    if (!FusionSavestate_DecodeV2Frame(raw, frame_len, decoded)) {
        printf("Smoke46: RX decode/CRC failed during %s\n", context);
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
        printf("Smoke46: %s expected CTL addr got 0x%02X\n", context, d.addr);
        return false;
    }
    if (d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Smoke46: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               context, d.error_code, d.error_detail);
        return false;
    }
    if (d.ctl_opcode != opcode || d.ack_for_opcode != ack_for) {
        printf("Smoke46: %s unexpected CTL op=0x%02X ack_for=0x%02X\n",
               context, d.ctl_opcode, d.ack_for_opcode);
        return false;
    }
    return true;
}

static bool SendCtlAndExpectAck(const char *label,
                                bool (*builder)(FusionV2Frame_t *out),
                                uint8_t ack_for)
{
    FusionV2Frame_t frame;
    if (!builder(&frame)) {
        printf("Smoke46: build %s failed\n", label);
        return false;
    }
    if (!SendFrame(label, &frame)) {
        return false;
    }
    return ExpectCtl((uint8_t)kFusionOp_AckAccepted, ack_for, label);
}

static bool BuildEnd(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildEndSession(out);
}

static bool BuildBeginSave(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildBeginSave(0u, out);
}

static void PrintRegionSummary(const SmokeRegion_t *region,
                               const uint8_t *data,
                               uint16_t length)
{
    const uint32_t crc = FusionSavestate_Crc32(data, length);
    printf("Smoke46: REGION %s id=0x%02X len=%u crc32=%08lX first:",
           region->name,
           (unsigned)region->region,
           (unsigned)length,
           (unsigned long)crc);
    const uint16_t first_count = (length < 16u) ? length : 16u;
    for (uint16_t i = 0; i < first_count; ++i) {
        printf(" %02X", data[i]);
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
            printf("Smoke46: Header magic mismatch\n");
            return false;
        }
        if (bitmap != 0x0000020Fu) {
            printf("Smoke46: Header bitmap mismatch got=0x%08lX\n",
                   (unsigned long)bitmap);
            return false;
        }
        if (memcmp(&data[12], "P450", 4u) != 0) {
            printf("Smoke46: Header tag mismatch got='%c%c%c%c'\n",
                   data[12], data[13], data[14], data[15]);
            return false;
        }
    } else if (region->region == 0x01u) {
        for (uint16_t i = 10u; i < length; ++i) {
            if (data[i] != 0u) {
                printf("Smoke46: Top padding byte %u nonzero=0x%02X\n",
                       (unsigned)i, data[i]);
                return false;
            }
        }
    } else if (region->region == 0x03u) {
        if (data[6] != 0u || data[7] != 0u) {
            printf("Smoke46: Timer padding mismatch b6=0x%02X b7=0x%02X\n",
                   data[6], data[7]);
            return false;
        }
    }
    return true;
}

static bool ReadRegion(const SmokeRegion_t *region)
{
    uint8_t data[127] = {0};
    uint16_t received = 0;
    uint16_t expected_seq = 0;

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildReadStreamBegin(region->region,
                                              0u,
                                              region->length,
                                              &begin)) {
        printf("Smoke46: build READ_STREAM_BEGIN %s failed\n", region->name);
        return false;
    }
    printf("Smoke46: BEGIN REGION %s id=0x%02X length=%u\n",
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
            printf("Smoke46: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
                   region->name, d.error_code, d.error_detail);
            return false;
        }
        if (d.addr != (uint8_t)kFusionAddr_StateData) {
            printf("Smoke46: %s expected DATA got addr=0x%02X op=0x%02X\n",
                   region->name, d.addr, d.ctl_opcode);
            return false;
        }
        if (d.data_seq != expected_seq) {
            printf("Smoke46: %s seq mismatch got=%u expected=%u\n",
                   region->name, (unsigned)d.data_seq, (unsigned)expected_seq);
            return false;
        }
        if (d.data_len == 0u || (uint16_t)d.data_len > (region->length - received)) {
            printf("Smoke46: %s bad data_len=%u remaining=%u\n",
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

    PrintRegionSummary(region, data, received);
    if (region->validate && !ValidateRegion(region, data, received)) {
        return false;
    }
    printf("Smoke46: REGION %s PASS packets=%u\n",
           region->name, (unsigned)expected_seq);
    return true;
}

void FusionSavestateSmoke46_RunBlocking(void)
{
    bool ok = true;

    printf("Smoke46: Phase 4.6 stream-only smoke starting\n");
    printf("Smoke46: waiting %u ms for FPGA boot\n", (unsigned)kSmoke46BootDelayMs);
    vTaskDelay(pdMS_TO_TICKS(kSmoke46BootDelayMs));
    printf("Smoke46: READ_NEXT debug disabled\n");
    uart_flush(UART_NUM_1);

    ok = SendCtlAndExpectAck("END_SESSION clear",
                             BuildEnd,
                             (uint8_t)kFusionOp_EndSession);

    if (ok) {
        ok = SendCtlAndExpectAck("BEGIN_SAVE",
                                 BuildBeginSave,
                                 (uint8_t)kFusionOp_BeginSave);
    }

    if (ok) {
        printf("Smoke46: save session active; screen should be paused briefly\n");
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    for (size_t i = 0; ok && i < (sizeof(kMatrix) / sizeof(kMatrix[0])); ++i) {
        ok = ReadRegion(&kMatrix[i]);
    }

    printf("Smoke46: sending END_SESSION cleanup\n");
    FusionV2Frame_t end;
    if (FusionSavestate_BuildEndSession(&end)) {
        (void)SendFrame("END_SESSION cleanup", &end);
        (void)ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                        (uint8_t)kFusionOp_EndSession,
                        "END_SESSION cleanup");
    }

    printf("Smoke46: RESULT %s\n", ok ? "PASS" : "FAIL");
    printf("Smoke46: Phase 4.6 stream-only smoke complete; continuing normal boot\n");
}
