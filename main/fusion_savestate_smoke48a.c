#include "fusion_savestate_smoke48a.h"

#include "driver/uart.h"
#include "esp_app_desc.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "fpga_common.h"
#include "fpga_rx.h"
#include "fpga_tx.h"
#include "fusion_savestate.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwrmgr.h"
#include "savestate_storage.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    kSmoke48aFrameTimeoutMs = 3000,
    kSmoke48aPreflightDrainMs = 120,
    kSmoke48aInterRegionMs  = 10,
    kSmoke48aFinalEndMs     = 50,
    kSmoke48aRegionCount    = 6,
    kSmoke48aEndSessionAttempts = 4,
};

static const char *TAG = "Smoke48a";
static const char kPartitionLabel[] = "savestate";
static const uint32_t kExpectedBitmap = 0x0000120Fu;

typedef struct {
    const char *name;
    uint8_t     region;
    uint16_t    length;
} Smoke48aRegion_t;

typedef struct {
    bool save_captured;
    bool slot_committed;
    bool slot_reloaded;
    bool crc_valid;
    bool load_ready;
} Smoke48aResult_t;

static const Smoke48aRegion_t kRegions[kSmoke48aRegionCount] = {
    { "Header", 0x00u, 16u    },
    { "Top",    0x01u, 16u    },
    { "CPU",    0x02u, 40u    },
    { "Timer",  0x03u, 8u     },
    { "HRAM",   0x09u, 127u   },
    { "WRAM",   0x0Cu, 32768u },
};

static uint8_t *s_rx_buf = NULL;
static size_t s_rx_buf_len = 0u;
static uint8_t s_uart_cache[512];
static size_t s_uart_cache_pos = 0u;
static size_t s_uart_cache_len = 0u;

static bool BuildEnd(FusionV2Frame_t *out);
static void DrainRxForMs(uint32_t ms);

static bool EnsureRxBuf(uint16_t need)
{
    if (need <= s_rx_buf_len) {
        return true;
    }
    uint8_t *p = (uint8_t *)heap_caps_realloc(s_rx_buf, need, MALLOC_CAP_8BIT);
    if (p == NULL) {
        printf("Smoke48a: heap alloc %u failed\n", (unsigned)need);
        return false;
    }
    s_rx_buf = p;
    s_rx_buf_len = need;
    return true;
}

static void ResetUartCache(void)
{
    s_uart_cache_pos = 0u;
    s_uart_cache_len = 0u;
}

static uint32_t ExpectedTotalSize(void)
{
    uint32_t payload = 0u;
    for (size_t i = 0; i < kSmoke48aRegionCount; ++i) {
        payload += kRegions[i].length;
    }
    return (uint32_t)sizeof(FusionStateHeader_t) +
           payload +
           (uint32_t)(kSmoke48aRegionCount * sizeof(FusionRegionDirectory_t));
}

static const Smoke48aRegion_t *FindExpectedRegion(uint8_t region_id)
{
    for (size_t i = 0; i < kSmoke48aRegionCount; ++i) {
        if (kRegions[i].region == region_id) {
            return &kRegions[i];
        }
    }
    return NULL;
}

static void PrintResult(const Smoke48aResult_t *r)
{
    printf("Smoke48a: STATUS save_captured=%s slot_committed=%s "
           "slot_reloaded=%s crc_valid=%s load_ready=%s\n",
           r->save_captured ? "yes" : "no",
           r->slot_committed ? "yes" : "no",
           r->slot_reloaded ? "yes" : "no",
           r->crc_valid ? "yes" : "no",
           r->load_ready ? "yes" : "no");
    printf("Smoke48a: RESULT %s (4.8a persistence gate only)\n",
           r->load_ready ? "PASS" : "FAIL");
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

static bool ReadDecodedFrame(const char *context, FusionV2Decoded_t *decoded)
{
    uint8_t raw[FUSION_V2_MAX_FRAME] = {0};
    int64_t deadline = esp_timer_get_time()
                     + ((int64_t)kSmoke48aFrameTimeoutMs * 1000);

    while (esp_timer_get_time() < deadline) {
        uint8_t b = 0;
        if (!ReadByteWithDeadline(&b, deadline)) {
            printf("Smoke48a: RX timeout waiting for marker during %s\n", context);
            return false;
        }
        if (b != FUSION_V2_HEADER_MARKER) {
            continue;
        }

        raw[0] = b;
        if (!ReadByteWithDeadline(&raw[1], deadline) ||
            !ReadByteWithDeadline(&raw[2], deadline)) {
            printf("Smoke48a: RX timeout waiting for header during %s\n", context);
            return false;
        }

        const uint8_t payload_len = raw[2];
        if (payload_len > FUSION_V2_MAX_PAYLOAD) {
            printf("Smoke48a: RX resync bad payload len %u during %s\n",
                   (unsigned)payload_len, context);
            continue;
        }

        const size_t frame_len = 3u + payload_len + 1u;
        for (size_t i = 3u; i < frame_len; ++i) {
            if (!ReadByteWithDeadline(&raw[i], deadline)) {
                printf("Smoke48a: RX timeout waiting for payload/crc during %s\n",
                       context);
                return false;
            }
        }

        if (!FusionSavestate_DecodeV2Frame(raw, frame_len, decoded)) {
            printf("Smoke48a: RX resync decode/CRC failed during %s\n", context);
            continue;
        }
        return true;
    }

    printf("Smoke48a: RX timeout waiting for valid frame during %s\n", context);
    return false;
}

static bool SendFrame(const char *label, const FusionV2Frame_t *frame)
{
    if (frame == NULL || frame->length == 0u) {
        printf("Smoke48a: %s invalid frame\n", label);
        return false;
    }

    printf("Smoke48a: TX %s:", label);
    for (uint8_t i = 0; i < frame->length; ++i) {
        printf(" %02X", frame->bytes[i]);
    }
    printf("\n");

    const int written = uart_write_bytes(UART_NUM_1,
                                         (const char *)frame->bytes,
                                         frame->length);
    const bool is_final_cleanup = (strcmp(label, "END_SESSION cleanup") == 0);
    if (is_final_cleanup) {
        printf("Smoke48a: TX %s write returned=%d\n", label, written);
    }
    const esp_err_t wait_err = uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(250));
    if (is_final_cleanup) {
        printf("Smoke48a: TX %s wait result=%s\n",
               label,
               esp_err_to_name(wait_err));
    }
    if (written != (int)frame->length) {
        printf("Smoke48a: TX %s failed wrote=%d expected=%u\n",
               label, written, (unsigned)frame->length);
        return false;
    }
    if (wait_err != ESP_OK) {
        printf("Smoke48a: TX %s wait failed err=%s\n",
               label, esp_err_to_name(wait_err));
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
        printf("Smoke48a: %s expected CTL addr got 0x%02X\n", context, d.addr);
        return false;
    }
    if (d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Smoke48a: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               context, d.error_code, d.error_detail);
        return false;
    }
    if (d.ctl_opcode != opcode || d.ack_for_opcode != ack_for) {
        printf("Smoke48a: %s unexpected CTL op=0x%02X ack_for=0x%02X\n",
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
        printf("Smoke48a: build %s failed\n", label);
        return false;
    }
    if (!SendFrame(label, &frame)) {
        return false;
    }
    return ExpectCtl((uint8_t)kFusionOp_AckAccepted, ack_for, label);
}

static bool IsEndSessionAck(const FusionV2Decoded_t *d)
{
    return d != NULL &&
           d->addr == (uint8_t)kFusionAddr_StateCtl &&
           d->ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
           d->ack_for_opcode == (uint8_t)kFusionOp_EndSession;
}

static bool SendEndSessionClear(const char *label)
{
    FusionV2Frame_t frame;
    if (!BuildEnd(&frame)) {
        printf("Smoke48a: build %s failed\n", label);
        return false;
    }

    for (uint8_t attempt = 0u; attempt < kSmoke48aEndSessionAttempts; ++attempt) {
        if (attempt != 0u) {
            DrainRxForMs(kSmoke48aPreflightDrainMs);
            ResetUartCache();
        }
        if (!SendFrame(label, &frame)) {
            return false;
        }
        FusionV2Decoded_t d;
        if (ReadDecodedFrame(label, &d) && IsEndSessionAck(&d)) {
            return true;
        }
    }
    return false;
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

static bool BuildEnd(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildEndSession(out);
}

static bool BuildBeginSave(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildBeginSave(0u, out);
}

static bool ValidateHeaderEvidence(const uint8_t data[16],
                                   uint32_t *bitmap_out,
                                   char tag_out[4])
{
    const uint32_t bitmap = (uint32_t)data[8]
                          | ((uint32_t)data[9] << 8)
                          | ((uint32_t)data[10] << 16)
                          | ((uint32_t)data[11] << 24);
    if (memcmp(data, "FUSS", 4u) != 0) {
        printf("Smoke48a: Header magic mismatch\n");
        return false;
    }
    if (bitmap != kExpectedBitmap) {
        printf("Smoke48a: Header bitmap mismatch got=0x%08lX want=0x%08lX\n",
               (unsigned long)bitmap,
               (unsigned long)kExpectedBitmap);
        return false;
    }
    if (memcmp(&data[12], "P48C", 4u) != 0) {
        printf("Smoke48a: Header tag mismatch got='%c%c%c%c' want='P48C'\n",
               data[12], data[13], data[14], data[15]);
        return false;
    }
    if (bitmap_out != NULL) {
        *bitmap_out = bitmap;
    }
    if (tag_out != NULL) {
        memcpy(tag_out, &data[12], 4u);
    }
    return true;
}

static bool ValidateLightRegion(const Smoke48aRegion_t *region,
                                const uint8_t *data,
                                uint16_t length)
{
    if (region->region == 0x01u) {
        for (uint16_t i = 10u; i < length; ++i) {
            if (data[i] != 0u) {
                printf("Smoke48a: Top padding byte %u nonzero=0x%02X\n",
                       (unsigned)i, data[i]);
                return false;
            }
        }
    } else if (region->region == 0x03u) {
        if (data[6] != 0u || data[7] != 0u) {
            printf("Smoke48a: Timer padding mismatch b6=0x%02X b7=0x%02X\n",
                   data[6], data[7]);
            return false;
        }
    }
    return true;
}

static bool ReadRegionToSlot(FusionStorage_t *storage,
                             const Smoke48aRegion_t *region,
                             FusionRegionDirectory_t *dir,
                             uint32_t offset,
                             uint32_t *bitmap_out,
                             char tag_out[4])
{
    if (!EnsureRxBuf(region->length)) {
        return false;
    }
    uint8_t *data = s_rx_buf;
    memset(data, 0, region->length);

    uint16_t received = 0u;
    uint16_t expected_seq = 0u;
    const int64_t t0 = esp_timer_get_time();

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildReadStreamBegin(region->region,
                                              0u,
                                              region->length,
                                              &begin)) {
        printf("Smoke48a: build READ_STREAM_BEGIN %s failed\n", region->name);
        return false;
    }

    printf("Smoke48a: BEGIN REGION %s id=0x%02X length=%u\n",
           region->name, (unsigned)region->region, (unsigned)region->length);
    if (!SendFrame("READ_STREAM_BEGIN", &begin)) {
        return false;
    }

    bool pending_data_valid = false;
    FusionV2Decoded_t pending_data;
    memset(&pending_data, 0, sizeof(pending_data));
    FusionV2Decoded_t first;
    if (!ReadDecodedFrame("READ_STREAM_BEGIN ack", &first)) {
        return false;
    }
    if (first.addr == (uint8_t)kFusionAddr_StateCtl &&
        first.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Smoke48a: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               region->name, first.error_code, first.error_detail);
        return false;
    }
    if (first.addr == (uint8_t)kFusionAddr_StateCtl &&
        first.ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
        first.ack_for_opcode == (uint8_t)kFusionOp_ReadStreamBegin) {
        /* Expected path. */
    } else if (first.addr == (uint8_t)kFusionAddr_StateData &&
               first.data_seq == 0u) {
        pending_data = first;
        pending_data_valid = true;
        printf("Smoke48a: %s DATA arrived before READ_STREAM_BEGIN ACK\n",
               region->name);
    } else {
        printf("Smoke48a: READ_STREAM_BEGIN ack unexpected addr=0x%02X "
               "op=0x%02X seq=%u\n",
               first.addr, first.ctl_opcode, (unsigned)first.data_seq);
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
            printf("Smoke48a: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
                   region->name, d.error_code, d.error_detail);
            return false;
        }
        if (d.addr != (uint8_t)kFusionAddr_StateData) {
            printf("Smoke48a: %s expected DATA got addr=0x%02X op=0x%02X\n",
                   region->name, d.addr, d.ctl_opcode);
            return false;
        }
        if (d.data_seq != expected_seq) {
            printf("Smoke48a: %s seq mismatch got=%u expected=%u\n",
                   region->name, (unsigned)d.data_seq, (unsigned)expected_seq);
            return false;
        }
        if (d.data_len == 0u || (uint16_t)d.data_len > (region->length - received)) {
            printf("Smoke48a: %s bad data_len=%u remaining=%u\n",
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
    ResetUartCache();
    uart_flush_input(UART_NUM_1);

    if (region->region == 0x00u &&
        !ValidateHeaderEvidence(data, bitmap_out, tag_out)) {
        return false;
    }
    if ((region->region == 0x01u || region->region == 0x03u) &&
        !ValidateLightRegion(region, data, received)) {
        return false;
    }

    FusionStorageResult_t sr =
        FusionStorage_AppendRegionPayload(storage, data, received);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48a: storage append %s failed result=%d\n",
               region->name, (int)sr);
        return false;
    }

    const uint32_t crc = FusionSavestate_Crc32(data, received);

    dir->region_id = region->region;
    memset(dir->reserved, 0, sizeof(dir->reserved));
    dir->offset = offset;
    dir->length = region->length;
    dir->crc32 = crc;

    const int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    printf("Smoke48a: REGION %s PASS packets=%u bytes=%u crc32=%08lX dt_ms=%lld\n",
           region->name,
           (unsigned)expected_seq,
           (unsigned)received,
           (unsigned long)crc,
           (long long)elapsed_ms);
    return true;
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

static bool EndSessionCleanup(void)
{
    printf("Smoke48a: sending END_SESSION cleanup\n");
    FusionV2Frame_t frame;
    if (!BuildEnd(&frame)) {
        printf("Smoke48a: build END_SESSION cleanup failed\n");
        return false;
    }
    DrainRxForMs(kSmoke48aFinalEndMs);
    ResetUartCache();
    uart_flush_input(UART_NUM_1);
    if (!SendFrame("END_SESSION cleanup", &frame)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    printf("Smoke48a: END_SESSION cleanup TX complete; ACK wait skipped\n");
    return true;
}

static bool ReadAndCrc(FusionStorage_t *storage,
                       uint32_t slot,
                       uint32_t offset,
                       uint32_t length,
                       uint32_t *crc_out)
{
    uint32_t crc = 0u;
    uint8_t buf[256];
    uint32_t pos = 0u;
    PwrMgr_IdleTimerPet();
    while (pos < length) {
        const uint32_t chunk = ((length - pos) > sizeof(buf))
            ? (uint32_t)sizeof(buf)
            : (length - pos);
        FusionStorageResult_t sr =
            FusionStorage_ReadSlotBytes(storage, slot, offset + pos, buf, chunk);
        if (sr != kFusionStorage_Ok) {
            printf("Smoke48a: storage read offset=%lu len=%lu failed result=%d\n",
                   (unsigned long)(offset + pos),
                   (unsigned long)chunk,
                   (int)sr);
            return false;
        }
        crc = FusionSavestate_Crc32Update(crc, buf, chunk);
        pos += chunk;
        PwrMgr_IdleTimerPet();
    }
    *crc_out = crc;
    return true;
}

static bool ValidateSlot(FusionStorage_t *storage, Smoke48aResult_t *result)
{
    uint32_t slot = 0u;
    uint32_t generation = 0u;
    FusionStorageResult_t sr =
        FusionStorage_FindNewestValidSlot(storage, &slot, &generation);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48a: no valid slot result=%d\n", (int)sr);
        return false;
    }

    FusionStateHeader_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    sr = FusionStorage_ReadSlotHeader(storage, slot, &hdr);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48a: read slot header failed result=%d\n", (int)sr);
        return false;
    }
    result->slot_reloaded = true;

    if (hdr.magic != FUSION_SAVESTATE_MAGIC ||
        hdr.format_version != FUSION_SAVESTATE_FORMAT_VERSION ||
        hdr.header_size != sizeof(FusionStateHeader_t) ||
        hdr.commit_state != (uint8_t)kFusionCommit_Valid) {
        printf("Smoke48a: bad slot header magic=%08lX version=%u header=%u state=0x%02X\n",
               (unsigned long)hdr.magic,
               (unsigned)hdr.format_version,
               (unsigned)hdr.header_size,
               hdr.commit_state);
        return false;
    }
    if (hdr.total_size != ExpectedTotalSize() ||
        hdr.region_count != kSmoke48aRegionCount ||
        hdr.region_bitmap != kExpectedBitmap ||
        memcmp(hdr.savestate_tag, "P48C", 4u) != 0) {
        printf("Smoke48a: metadata mismatch total=%lu/%lu regions=%lu/%u "
               "bitmap=0x%08lX tag='%c%c%c%c'\n",
               (unsigned long)hdr.total_size,
               (unsigned long)ExpectedTotalSize(),
               (unsigned long)hdr.region_count,
               (unsigned)kSmoke48aRegionCount,
               (unsigned long)hdr.region_bitmap,
               hdr.savestate_tag[0], hdr.savestate_tag[1],
               hdr.savestate_tag[2], hdr.savestate_tag[3]);
        return false;
    }

    uint32_t payload_crc = 0u;
    if (!ReadAndCrc(storage,
                    slot,
                    sizeof(FusionStateHeader_t),
                    hdr.total_size - (uint32_t)sizeof(FusionStateHeader_t),
                    &payload_crc)) {
        return false;
    }
    if (payload_crc != hdr.payload_crc32) {
        printf("Smoke48a: overall CRC mismatch got=%08lX want=%08lX\n",
               (unsigned long)payload_crc,
               (unsigned long)hdr.payload_crc32);
        return false;
    }

    FusionRegionDirectory_t dirs[kSmoke48aRegionCount];
    sr = FusionStorage_ReadSlotBytes(storage,
                                     slot,
                                     hdr.region_table_offset,
                                     (uint8_t *)dirs,
                                     sizeof(dirs));
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48a: read region directory failed result=%d\n", (int)sr);
        return false;
    }

    uint32_t expected_offset = (uint32_t)sizeof(FusionStateHeader_t);
    for (size_t i = 0; i < kSmoke48aRegionCount; ++i) {
        const Smoke48aRegion_t *expected = FindExpectedRegion(dirs[i].region_id);
        if (expected == NULL || expected->region != kRegions[i].region ||
            dirs[i].offset != expected_offset ||
            dirs[i].length != expected->length) {
            printf("Smoke48a: directory mismatch idx=%u id=0x%02X offset=%lu length=%lu\n",
                   (unsigned)i,
                   dirs[i].region_id,
                   (unsigned long)dirs[i].offset,
                   (unsigned long)dirs[i].length);
            return false;
        }
        uint32_t region_crc = 0u;
        if (!ReadAndCrc(storage, slot, dirs[i].offset, dirs[i].length, &region_crc)) {
            return false;
        }
        if (region_crc != dirs[i].crc32) {
            printf("Smoke48a: region 0x%02X CRC mismatch got=%08lX want=%08lX\n",
                   dirs[i].region_id,
                   (unsigned long)region_crc,
                   (unsigned long)dirs[i].crc32);
            return false;
        }
        expected_offset += dirs[i].length;
    }

    uint8_t captured_header[16] = {0};
    sr = FusionStorage_ReadSlotBytes(storage,
                                     slot,
                                     dirs[0].offset,
                                     captured_header,
                                     sizeof(captured_header));
    if (sr != kFusionStorage_Ok ||
        !ValidateHeaderEvidence(captured_header, NULL, NULL)) {
        return false;
    }

    result->crc_valid = true;
    result->load_ready = true;
    printf("Smoke48a: slot valid generation=%lu total=%lu payload_crc=%08lX\n",
           (unsigned long)generation,
           (unsigned long)hdr.total_size,
           (unsigned long)hdr.payload_crc32);
    return true;
}

static bool InitStorage(FusionStorage_t *storage)
{
    FusionStorageResult_t sr = FusionStorage_InitFromPartition(storage, kPartitionLabel);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48a: init partition '%s' failed result=%d\n",
               kPartitionLabel, (int)sr);
        return false;
    }
    return true;
}

bool FusionSavestate_QuickLoadSlot0(void)
{
    PwrMgr_IdleTimerPet();

    Smoke48aResult_t result = {0};
    FusionStorage_t storage;
    if (!InitStorage(&storage)) {
        PrintResult(&result);
        return false;
    }
    const bool ok = ValidateSlot(&storage, &result);
    PrintResult(&result);
    return ok;
}

bool FusionSavestate_QuickSaveSlot0(void)
{
    Smoke48aResult_t result = {0};
    FusionStorage_t storage;
    if (!InitStorage(&storage)) {
        PrintResult(&result);
        return false;
    }

    uint32_t prior_slot = 0u;
    uint32_t prior_generation = 0u;
    uint32_t next_generation = 1u;
    if (FusionStorage_FindNewestValidSlot(&storage, &prior_slot, &prior_generation) ==
        kFusionStorage_Ok) {
        next_generation = prior_generation + 1u;
    }

    FusionRegionDirectory_t dirs[kSmoke48aRegionCount];
    memset(dirs, 0, sizeof(dirs));
    uint32_t next_offset = (uint32_t)sizeof(FusionStateHeader_t);
    uint32_t region_bitmap = 0u;
    char savestate_tag[4] = {0};
    bool ok = true;
    bool session_started = false;

    PwrMgr_IdleTimerSuspend();
    PauseFpgaTraffic();
    FPGA_Rx_ResetParser();
    ResetUartCache();
    if (!FPGA_UartOwnerAcquire(pdMS_TO_TICKS(2000))) {
        printf("Smoke48a: UART owner unavailable\n");
        ResumeFpgaTraffic();
        PwrMgr_IdleTimerResume();
        PrintResult(&result);
        return false;
    }

    do {
        DrainRxForMs(kSmoke48aPreflightDrainMs);
        FPGA_Rx_ResetParser();
        ResetUartCache();
        ok = SendEndSessionClear("END_SESSION clear");
        if (!ok) {
            break;
        }
        DrainRxForMs(kSmoke48aPreflightDrainMs);
        FPGA_Rx_ResetParser();
        ResetUartCache();
        ok = SendCtlAndExpectAck("BEGIN_SAVE",
                                 BuildBeginSave,
                                 (uint8_t)kFusionOp_BeginSave);
        if (!ok) {
            break;
        }
        session_started = true;
        printf("Smoke48a: save session active\n");
        vTaskDelay(pdMS_TO_TICKS(50));

        FusionStorageResult_t sr = FusionStorage_BeginInactiveSlotWrite(&storage);
        if (sr != kFusionStorage_Ok) {
            printf("Smoke48a: begin slot write failed result=%d\n", (int)sr);
            ok = false;
            break;
        }

        for (size_t i = 0; i < kSmoke48aRegionCount; ++i) {
            ok = ReadRegionToSlot(&storage,
                                  &kRegions[i],
                                  &dirs[i],
                                  next_offset,
                                  &region_bitmap,
                                  savestate_tag);
            if (!ok) {
                break;
            }
            next_offset += kRegions[i].length;
            DrainRxForMs(kSmoke48aInterRegionMs);
        }
    } while (false);

    bool cleanup_ok = true;
    if (session_started) {
        cleanup_ok = EndSessionCleanup();
    }

    if (!ok || !cleanup_ok) {
        (void)FusionStorage_AbortSlotWrite(&storage);
        ResumeFpgaTraffic();
        FPGA_UartOwnerRelease();
        PwrMgr_IdleTimerResume();
        PrintResult(&result);
        return false;
    }
    result.save_captured = true;

    FusionStateHeader_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.game_id_algorithm = (uint8_t)FUSION_GAME_ID_ALGORITHM_V1;
    memcpy(hdr.fpga_version, "P48C", 4u);
    const esp_app_desc_t *desc = esp_app_get_description();
    if (desc != NULL) {
        memcpy(hdr.mcu_version, desc->version,
               strlen(desc->version) < sizeof(hdr.mcu_version)
                   ? strlen(desc->version)
                   : sizeof(hdr.mcu_version));
    }
    hdr.commit_generation = next_generation;
    hdr.region_bitmap = region_bitmap;
    memcpy(hdr.savestate_tag, savestate_tag, sizeof(hdr.savestate_tag));

    printf("Smoke48a: staging slot header\n");
    FusionStorageResult_t sr =
        FusionStorage_StageHeader(&storage, &hdr, dirs, kSmoke48aRegionCount);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48a: stage header failed result=%d\n", (int)sr);
        (void)FusionStorage_AbortSlotWrite(&storage);
        ResumeFpgaTraffic();
        FPGA_UartOwnerRelease();
        PwrMgr_IdleTimerResume();
        PrintResult(&result);
        return false;
    }
    printf("Smoke48a: committing slot\n");
    sr = FusionStorage_VerifyAndCommit(&storage);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48a: verify/commit failed result=%d\n", (int)sr);
        (void)FusionStorage_AbortSlotWrite(&storage);
        ResumeFpgaTraffic();
        FPGA_UartOwnerRelease();
        PwrMgr_IdleTimerResume();
        PrintResult(&result);
        return false;
    }
    result.slot_committed = true;

    printf("Smoke48a: validating committed slot\n");
    (void)ValidateSlot(&storage, &result);
    ResumeFpgaTraffic();
    FPGA_UartOwnerRelease();
    PwrMgr_IdleTimerResume();
    PrintResult(&result);
    return result.load_ready;
}

static int Smoke48aCommand(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: smoke48a save|validate|reboot\n");
        return 1;
    }
    if (strcmp(argv[1], "save") == 0) {
        return FusionSavestate_QuickSaveSlot0() ? 0 : 1;
    }
    if (strcmp(argv[1], "validate") == 0) {
        return FusionSavestate_QuickLoadSlot0() ? 0 : 1;
    }
    if (strcmp(argv[1], "reboot") == 0) {
        printf("Smoke48a: rebooting now; run 'smoke48a validate' after boot\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return 0;
    }
    printf("Usage: smoke48a save|validate|reboot\n");
    return 1;
}

void FusionSavestateSmoke48a_RegisterCommands(void)
{
    const esp_console_cmd_t command = {
        .command = "smoke48a",
        .help = "Phase 4.8a savestate persistence smoke: save|validate|reboot",
        .func = &Smoke48aCommand,
        .argtable = NULL,
    };

    esp_err_t err = esp_console_cmd_register(&command);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Registering '%s' failed: %s",
                 command.command, esp_err_to_name(err));
    }
}
