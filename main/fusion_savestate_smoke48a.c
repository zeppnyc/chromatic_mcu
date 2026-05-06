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
#include "fusion_savestate_pathe.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "pwrmgr.h"
#include "savestate_storage.h"
#include "savestate_toast.h"
#include "save_slot.h"

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
static const uint32_t kExpectedBitmap = 0x0000320Fu;

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
static bool InitStorage(FusionStorage_t *storage);

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
    printf("Smoke48a: RESULT %s\n",
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

    const bool quiet_data = (strcmp(label, "STATE_DATA") == 0);
    if (!quiet_data) {
        printf("Smoke48a: TX %s:", label);
        for (uint8_t i = 0; i < frame->length; ++i) {
            printf(" %02X", frame->bytes[i]);
        }
        printf("\n");
    }

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

static bool BuildBeginTestRW(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildBeginTestRW(0u, out);
}

static bool BuildBeginLoad(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildBeginLoad(0u, out);
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
    if (memcmp(&data[12], "P48E", 4u) != 0) {
        printf("Smoke48a: Header tag mismatch got='%c%c%c%c' want='P48E'\n",
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
    DrainRxForMs(kSmoke48aFinalEndMs);
    ResetUartCache();
    uart_flush_input(UART_NUM_1);
    return SendEndSessionClear("END_SESSION cleanup");
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
    const bool tag_ok = (memcmp(hdr.savestate_tag, "P48E", 4u) == 0);
    if (hdr.total_size != ExpectedTotalSize() ||
        hdr.region_count != kSmoke48aRegionCount ||
        hdr.region_bitmap != kExpectedBitmap ||
        !tag_ok) {
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

static const FusionRegionDirectory_t *FindDir(const FusionRegionDirectory_t *dirs,
                                              uint32_t count,
                                              uint8_t region)
{
    if (dirs == NULL) {
        return NULL;
    }
    for (uint32_t i = 0u; i < count; ++i) {
        if (dirs[i].region_id == region) {
            return &dirs[i];
        }
    }
    return NULL;
}

static bool ReadRegionToBuffer(uint8_t region,
                               uint16_t offset,
                               uint16_t length,
                               uint8_t *dest,
                               const char *label)
{
    if (dest == NULL || length == 0u) {
        return false;
    }
    uint16_t received = 0u;
    uint16_t expected_seq = 0u;

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildReadStreamBegin(region, offset, length, &begin)) {
        printf("Smoke48a: build READ_STREAM_BEGIN %s failed\n", label);
        return false;
    }
    if (!SendFrame("READ_STREAM_BEGIN", &begin)) {
        return false;
    }
    if (!ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                   (uint8_t)kFusionOp_ReadStreamBegin,
                   "READ_STREAM_BEGIN ack")) {
        return false;
    }

    while (received < length) {
        FusionV2Decoded_t d;
        if (!ReadDecodedFrame(label, &d)) {
            return false;
        }
        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_Error) {
            printf("Smoke48a: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
                   label, d.error_code, d.error_detail);
            return false;
        }
        if (d.addr != (uint8_t)kFusionAddr_StateData ||
            d.data_seq != expected_seq ||
            d.data_len == 0u ||
            (uint16_t)d.data_len > (length - received)) {
            printf("Smoke48a: %s read mismatch addr=0x%02X seq=%u/%u len=%u rem=%u\n",
                   label,
                   d.addr,
                   (unsigned)d.data_seq,
                   (unsigned)expected_seq,
                   (unsigned)d.data_len,
                   (unsigned)(length - received));
            return false;
        }
        memcpy(&dest[received], d.data, d.data_len);
        received = (uint16_t)(received + d.data_len);
        expected_seq = (uint16_t)(expected_seq + 1u);
    }

    return ExpectCtl((uint8_t)kFusionOp_AckDone,
                     (uint8_t)kFusionOp_ReadStreamBegin,
                     "READ_STREAM_BEGIN done");
}

static bool ReadCurrentCartHeader(uint8_t rom_header[FUSION_ROM_HEADER_BYTES],
                                  uint32_t *game_id_out)
{
    if (rom_header == NULL || game_id_out == NULL) {
        return false;
    }
    memset(rom_header, 0, FUSION_ROM_HEADER_BYTES);
    if (!ReadRegionToBuffer((uint8_t)kFusionRegion_RomHeader,
                            0u,
                            FUSION_ROM_HEADER_BYTES,
                            rom_header,
                            "CartHeader(current)")) {
        return false;
    }
    *game_id_out = FusionSavestate_ComputeGameIdV1(rom_header);
    if (*game_id_out == 0u) {
        printf("Smoke48a: current cart game-id hash is zero; refusing\n");
        return false;
    }
    return true;
}

static bool VerifyCurrentFpgaHeader(void)
{
    uint8_t header[16] = {0};
    uint32_t bitmap = 0u;
    char tag[4] = {0};
    if (!ReadRegionToBuffer((uint8_t)kFusionRegion_Header,
                            0u,
                            sizeof(header),
                            header,
                            "Header(current)")) {
        return false;
    }
    if (!ValidateHeaderEvidence(header, &bitmap, tag)) {
        return false;
    }
    printf("Smoke48a: current FPGA capability bitmap=0x%08lX tag='%c%c%c%c'\n",
           (unsigned long)bitmap, tag[0], tag[1], tag[2], tag[3]);
    return true;
}

static bool VerifyCurrentGameId(const FusionStateHeader_t *slot_header,
                                SavestateToastResult_t *toast_result)
{
    if (slot_header == NULL || toast_result == NULL) {
        return false;
    }
    if (slot_header->game_id_algorithm != (uint8_t)FUSION_GAME_ID_ALGORITHM_V1 ||
        slot_header->game_id_hash == 0u) {
        printf("Smoke48a/PathE: slot has no valid game-id evidence algo=%u hash=%08lX\n",
               (unsigned)slot_header->game_id_algorithm,
               (unsigned long)slot_header->game_id_hash);
        *toast_result = kSavestateToast_Failed;
        return false;
    }

    uint8_t rom_header[FUSION_ROM_HEADER_BYTES];
    uint32_t current_hash = 0u;
    if (!ReadCurrentCartHeader(rom_header, &current_hash)) {
        *toast_result = kSavestateToast_Failed;
        return false;
    }
    if (current_hash != slot_header->game_id_hash) {
        printf("Smoke48a/PathE: wrong game current=%08lX saved=%08lX\n",
               (unsigned long)current_hash,
               (unsigned long)slot_header->game_id_hash);
        *toast_result = kSavestateToast_WrongGame;
        return false;
    }
    printf("Smoke48a/PathE: game-id verified hash=%08lX\n",
           (unsigned long)current_hash);
    return true;
}

static bool WriteSlotStream(FusionStorage_t *storage,
                            uint32_t slot,
                            const FusionRegionDirectory_t *dir,
                            uint16_t region_offset,
                            uint16_t length,
                            const char *label)
{
    if (storage == NULL || dir == NULL || length == 0u) {
        return false;
    }
    if ((uint32_t)region_offset + length > dir->length) {
        printf("Smoke48a: %s scalar slice out of range off=%u len=%u dir_len=%lu\n",
               label,
               (unsigned)region_offset,
               (unsigned)length,
               (unsigned long)dir->length);
        return false;
    }

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildWriteStreamBegin(dir->region_id,
                                               region_offset,
                                               length,
                                               &begin)) {
        printf("Smoke48a: build WRITE_STREAM_BEGIN %s failed\n", label);
        return false;
    }
    if (!SendFrame("WRITE_STREAM_BEGIN", &begin)) {
        return false;
    }
    /* Tolerate transient bad-payload BEGIN ACK; rely on final ACK_DONE.
     * Mirrors smoke48c.ExpectWriteBeginOrProceed fallback. */
    (void)ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                    (uint8_t)kFusionOp_WriteStreamBegin,
                    "WRITE_STREAM_BEGIN ack (lenient)");

    uint16_t sent = 0u;
    uint16_t seq = 0u;
    uint8_t chunk_buf[FUSION_STATE_DATA_MAX_DATA];
    while (sent < length) {
        const uint16_t remaining = (uint16_t)(length - sent);
        const uint8_t chunk = (remaining > FUSION_STATE_DATA_MAX_DATA)
            ? FUSION_STATE_DATA_MAX_DATA
            : (uint8_t)remaining;
        FusionStorageResult_t sr =
            FusionStorage_ReadSlotBytes(storage,
                                        slot,
                                        dir->offset + region_offset + sent,
                                        chunk_buf,
                                        chunk);
        if (sr != kFusionStorage_Ok) {
            printf("Smoke48a: %s storage read failed result=%d\n",
                   label, (int)sr);
            return false;
        }
        FusionV2Frame_t data;
        if (!FusionSavestate_BuildStateData(seq, chunk_buf, chunk, &data)) {
            printf("Smoke48a: build STATE_DATA %s failed\n", label);
            return false;
        }
        if (!SendFrame("STATE_DATA", &data)) {
            return false;
        }
        sent = (uint16_t)(sent + chunk);
        seq = (uint16_t)(seq + 1u);
    }

    if (!ExpectCtl((uint8_t)kFusionOp_AckDone,
                   (uint8_t)kFusionOp_WriteStreamBegin,
                   "WRITE_STREAM_BEGIN done")) {
        return false;
    }
    printf("Smoke48a: WRITE %s PASS bytes=%u packets=%u\n",
           label, (unsigned)length, (unsigned)seq);
    /* 50ms inter-write delay: bridge str_state transition + tx pipeline drain. */
    vTaskDelay(pdMS_TO_TICKS(50));
    return true;
}

static bool WriteCommit(void)
{
    FusionV2Frame_t commit;
    if (!FusionSavestate_BuildWriteCommit(&commit)) {
        printf("Smoke48a: build WRITE_COMMIT failed\n");
        return false;
    }
    if (!SendFrame("WRITE_COMMIT", &commit)) {
        return false;
    }
    return ExpectCtl((uint8_t)kFusionOp_AckDone,
                     (uint8_t)kFusionOp_WriteCommit,
                     "WRITE_COMMIT done");
}

static bool VerifyReadbackCrc(const FusionRegionDirectory_t *dir,
                              const char *label)
{
    if (dir == NULL || !EnsureRxBuf((uint16_t)dir->length)) {
        return false;
    }
    uint8_t *data = s_rx_buf;
    memset(data, 0, dir->length);
    if (!ReadRegionToBuffer(dir->region_id,
                            0u,
                            (uint16_t)dir->length,
                            data,
                            label)) {
        return false;
    }
    const uint32_t crc = FusionSavestate_Crc32(data, dir->length);
    if (crc != dir->crc32) {
        printf("Smoke48a: READBACK %s CRC mismatch got=%08lX want=%08lX\n",
               label, (unsigned long)crc, (unsigned long)dir->crc32);
        return false;
    }
    printf("Smoke48a: READBACK %s PASS crc32=%08lX\n",
           label, (unsigned long)crc);
    return true;
}

/* Write `length` bytes from memory `data` to FPGA region:offset.
 * Generalization of WriteSlotStream that does not require a slot — used
 * by path-e load to push the freshly-built region 0xFE payload + the
 * gbreset toggle bytes.
 */
static bool WriteRawToRegion(uint8_t region_id,
                              uint16_t region_offset,
                              const uint8_t *data,
                              uint16_t length,
                              const char *label)
{
    if (data == NULL || length == 0u) {
        return false;
    }
    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildWriteStreamBegin(region_id,
                                                region_offset,
                                                length,
                                                &begin)) {
        printf("Smoke48a: build WRITE_STREAM_BEGIN %s failed\n", label);
        return false;
    }
    if (!SendFrame("WRITE_STREAM_BEGIN", &begin)) {
        return false;
    }
    /* Tolerate transient bad-payload BEGIN ACK; rely on final ACK_DONE.
     * Mirrors smoke48c.ExpectWriteBeginOrProceed fallback. */
    (void)ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                    (uint8_t)kFusionOp_WriteStreamBegin,
                    "WRITE_STREAM_BEGIN ack (lenient)");
    uint16_t sent = 0u;
    uint16_t seq = 0u;
    while (sent < length) {
        const uint16_t remaining = (uint16_t)(length - sent);
        const uint8_t chunk = (remaining > FUSION_STATE_DATA_MAX_DATA)
            ? FUSION_STATE_DATA_MAX_DATA
            : (uint8_t)remaining;
        FusionV2Frame_t frame;
        if (!FusionSavestate_BuildStateData(seq, &data[sent], chunk, &frame)) {
            printf("Smoke48a: build STATE_DATA %s seq=%u failed\n",
                   label, (unsigned)seq);
            return false;
        }
        if (!SendFrame("STATE_DATA", &frame)) {
            return false;
        }
        sent = (uint16_t)(sent + chunk);
        seq = (uint16_t)(seq + 1u);
    }
    if (!ExpectCtl((uint8_t)kFusionOp_AckDone,
                   (uint8_t)kFusionOp_WriteStreamBegin,
                   "WRITE_STREAM_BEGIN done")) {
        return false;
    }
    printf("Smoke48a: WRITE %s PASS bytes=%u packets=%u\n",
           label, (unsigned)length, (unsigned)seq);
    /* 50ms inter-write delay: bridge str_state transition + tx pipeline drain. */
    vTaskDelay(pdMS_TO_TICKS(50));
    return true;
}

/*
 * Path-e load orchestration helper.  Path-e specifically does NOT write
 * region 0x02 (CPU regs) — the FPGA stub restores CPU state via POPs
 * during cold-boot.  Writing CPU region would set T80 SS_1[15:0] = saved_PC
 * which then becomes the cold-reset PC, defeating the stub mechanism.
 *
 * Sequence:
 *   1. Read CPU region from slot, extract path-e load input
 *   2. Build region 0xFE payload via FusionSavestate_BuildPathELoadPayload
 *   3. Write Top/Timer/HRAM/WRAM (skip CPU)
 *   4. Write region 0xFE bytes 0-14 (load_mode_active=1 + scratch + saved_*)
 *   5. Write region 0xFE byte 15 = 1 (gbreset assert)
 *   6. vTaskDelay
 *   7. Write region 0xFE byte 15 = 0 (gbreset release)
 *   - CPU now executes stub from $0000, restores via POPs, jumps to saved_PC
 */
static bool LoadSlotToFpga_PathE(FusionStorage_t *storage,
                                  uint32_t slot,
                                  const FusionRegionDirectory_t dirs[kSmoke48aRegionCount])
{
    const FusionRegionDirectory_t *top   = FindDir(dirs, kSmoke48aRegionCount, 0x01u);
    const FusionRegionDirectory_t *cpu   = FindDir(dirs, kSmoke48aRegionCount, 0x02u);
    const FusionRegionDirectory_t *timer = FindDir(dirs, kSmoke48aRegionCount, 0x03u);
    const FusionRegionDirectory_t *hram  = FindDir(dirs, kSmoke48aRegionCount, 0x09u);
    const FusionRegionDirectory_t *wram  = FindDir(dirs, kSmoke48aRegionCount, 0x0Cu);
    if (top == NULL || cpu == NULL || timer == NULL || hram == NULL || wram == NULL) {
        printf("Smoke48a/PathE: load missing required region directory\n");
        return false;
    }

    /* 1. Extract path-e load input from saved CPU region (40 bytes). */
    uint8_t cpu_bytes[40] = {0};
    if (cpu->length < sizeof(cpu_bytes)) {
        printf("Smoke48a/PathE: CPU region too short %lu < %u\n",
               (unsigned long)cpu->length, (unsigned)sizeof(cpu_bytes));
        return false;
    }
    FusionStorageResult_t sr = FusionStorage_ReadSlotBytes(storage,
                                                            slot,
                                                            cpu->offset,
                                                            cpu_bytes,
                                                            sizeof(cpu_bytes));
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48a/PathE: storage read CPU bytes failed result=%d\n", (int)sr);
        return false;
    }
    FusionPathELoadInput_t input;
    if (!FusionSavestate_ExtractPathELoadInputFromCpuRegion(cpu_bytes,
                                                              sizeof(cpu_bytes),
                                                              &input)) {
        printf("Smoke48a/PathE: extract path-e input failed\n");
        return false;
    }

    /* 2. Build path-e region 0xFE payload (16 bytes; byte 15 stays 0). */
    uint8_t pathe_payload[kPathE_RegionLength];
    if (!FusionSavestate_BuildPathELoadPayload(&input, pathe_payload)) {
        printf("Smoke48a/PathE: build payload failed (HALT@PC=0?)\n");
        return false;
    }
    printf("Smoke48a/PathE: payload built A=%02X F=%02X BC=%02X%02X DE=%02X%02X "
           "HL=%02X%02X SP=%04X PC=%04X IFF=%d HALT=%d\n",
           input.saved_A, input.saved_F,
           input.saved_B, input.saved_C,
           input.saved_D, input.saved_E,
           input.saved_H, input.saved_L,
           input.saved_SP, input.saved_PC,
           (int)input.saved_IFF, (int)input.halted_at_save);

    /* 3. Write Top, Timer, HRAM, WRAM.  Skip CPU region (path-e architecture). */
    if (!WriteSlotStream(storage, slot, top,   0u, 8u,  "Top[0]") ||
        !WriteSlotStream(storage, slot, top,   8u, 8u,  "Top[1]") ||
        !WriteSlotStream(storage, slot, timer, 0u, 8u,  "Timer") ||
        !WriteSlotStream(storage, slot, hram,  0u, (uint16_t)hram->length, "HRAM") ||
        !WriteSlotStream(storage, slot, wram,  0u, (uint16_t)wram->length, "WRAM")) {
        return false;
    }

    /* 4. Write region 0xFE bytes 0-14 (15 bytes including load_mode_active=1). */
    if (!WriteRawToRegion((uint8_t)kFusionRegion_PathELoad,
                          0u,
                          pathe_payload,
                          (uint16_t)kPathE_GbresetRequest,  /* 15 bytes */
                          "PathE[0..14]")) {
        return false;
    }

    /* 5. Assert gbreset via region 0xFE byte 15 = 1. */
    const uint8_t gbreset_assert = 0x01u;
    if (!WriteRawToRegion((uint8_t)kFusionRegion_PathELoad,
                          (uint16_t)kPathE_GbresetRequest,
                          &gbreset_assert,
                          1u,
                          "PathE gbreset=1")) {
        return false;
    }

    /* 6. Hold gbreset for FUSION_PATHE_GBRESET_HOLD_MS to let reset propagate. */
    vTaskDelay(pdMS_TO_TICKS(FUSION_PATHE_GBRESET_HOLD_MS));

    /* 7. Release gbreset via region 0xFE byte 15 = 0.  CPU now cold-boots
     *    from $0000 = stub bytes (load_mode_active=1 from step 4). */
    const uint8_t gbreset_release = 0x00u;
    if (!WriteRawToRegion((uint8_t)kFusionRegion_PathELoad,
                          (uint16_t)kPathE_GbresetRequest,
                          &gbreset_release,
                          1u,
                          "PathE gbreset=0")) {
        return false;
    }

    printf("Smoke48a/PathE: load orchestration complete\n");
    return true;
}

bool FusionSavestate_QuickLoadSlot0_PathE(void)
{
    PwrMgr_IdleTimerPet();

    Smoke48aResult_t result = {0};
    SavestateToastResult_t toast_result = kSavestateToast_Failed;
    bool ok = false;
    bool cleanup_ok = true;
    bool traffic_paused = false;
    bool owner_acquired = false;
    bool preflight_session_started = false;
    bool load_session_started = false;

    FusionStorage_t storage;
    if (!InitStorage(&storage)) {
        printf("Smoke48a/PathE: storage init failed\n");
        goto done;
    }

    ok = ValidateSlot(&storage, &result);
    if (!ok) {
        goto done;
    }

    uint32_t slot = 0u;
    uint32_t generation = 0u;
    FusionStorageResult_t sr =
        FusionStorage_FindNewestValidSlot(&storage, &slot, &generation);
    if (sr != kFusionStorage_Ok) {
        ok = false;
        goto done;
    }

    FusionStateHeader_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    sr = FusionStorage_ReadSlotHeader(&storage, slot, &hdr);
    if (sr != kFusionStorage_Ok) {
        ok = false;
        goto done;
    }
    FusionRegionDirectory_t dirs[kSmoke48aRegionCount];
    sr = FusionStorage_ReadSlotBytes(&storage,
                                     slot,
                                     hdr.region_table_offset,
                                     (uint8_t *)dirs,
                                     sizeof(dirs));
    if (sr != kFusionStorage_Ok) {
        ok = false;
        goto done;
    }

    PwrMgr_IdleTimerSuspend();
    PauseFpgaTraffic();
    traffic_paused = true;
    FPGA_Rx_ResetParser();
    ResetUartCache();
    if (!FPGA_UartOwnerAcquire(pdMS_TO_TICKS(2000))) {
        printf("Smoke48a/PathE: UART owner unavailable\n");
        ok = false;
        goto cleanup;
    }
    owner_acquired = true;

    do {
        DrainRxForMs(kSmoke48aPreflightDrainMs);
        FPGA_Rx_ResetParser();
        ResetUartCache();
        ok = SendEndSessionClear("END_SESSION clear");
        if (!ok) break;

        DrainRxForMs(kSmoke48aPreflightDrainMs);
        FPGA_Rx_ResetParser();
        ResetUartCache();
        ok = SendCtlAndExpectAck("BEGIN_TEST_RW",
                                 BuildBeginTestRW,
                                 (uint8_t)kFusionOp_BeginTestRW);
        if (!ok) break;
        preflight_session_started = true;

        ok = VerifyCurrentFpgaHeader() && VerifyCurrentGameId(&hdr, &toast_result);
        if (!ok) break;

        cleanup_ok = EndSessionCleanup();
        preflight_session_started = false;
        if (!cleanup_ok) {
            ok = false;
            break;
        }

        DrainRxForMs(kSmoke48aPreflightDrainMs);
        FPGA_Rx_ResetParser();
        ResetUartCache();
        ok = SendCtlAndExpectAck("BEGIN_LOAD",
                                 BuildBeginLoad,
                                 (uint8_t)kFusionOp_BeginLoad);
        if (!ok) break;

        load_session_started = true;
        printf("Smoke48a/PathE: load session active slot=%lu generation=%lu\n",
               (unsigned long)slot, (unsigned long)generation);

        ok = LoadSlotToFpga_PathE(&storage, slot, dirs);
    } while (false);

cleanup:
    if (load_session_started && !ok) {
        const uint8_t load_mode_clear = 0u;
        (void)WriteRawToRegion((uint8_t)kFusionRegion_PathELoad,
                               (uint16_t)kPathE_LoadModeActive,
                               &load_mode_clear,
                               1u,
                               "PathE load_mode=0 cleanup");
    }
    if (preflight_session_started || load_session_started) {
        cleanup_ok = EndSessionCleanup();
    }

    if (traffic_paused) {
        ResumeFpgaTraffic();
    }
    if (owner_acquired) {
        FPGA_UartOwnerRelease();
    }
    if (traffic_paused) {
        PwrMgr_IdleTimerResume();
    }

done:
    ok = ok && cleanup_ok;
    if (ok) {
        toast_result = kSavestateToast_Loaded;
    }
    SavestateToast_Show(toast_result, SAVESTATE_TOAST_DEFAULT_MS);
    return ok;
}

static bool LoadSlotToFpga(FusionStorage_t *storage,
                           uint32_t slot,
                           const FusionRegionDirectory_t dirs[kSmoke48aRegionCount])
{
    const FusionRegionDirectory_t *top   = FindDir(dirs, kSmoke48aRegionCount, 0x01u);
    const FusionRegionDirectory_t *cpu   = FindDir(dirs, kSmoke48aRegionCount, 0x02u);
    const FusionRegionDirectory_t *timer = FindDir(dirs, kSmoke48aRegionCount, 0x03u);
    const FusionRegionDirectory_t *hram  = FindDir(dirs, kSmoke48aRegionCount, 0x09u);
    const FusionRegionDirectory_t *wram  = FindDir(dirs, kSmoke48aRegionCount, 0x0Cu);
    if (top == NULL || cpu == NULL || timer == NULL || hram == NULL || wram == NULL) {
        printf("Smoke48a: load missing required region directory\n");
        return false;
    }

    uint8_t header[16] = {0};
    if (!ReadRegionToBuffer(0x00u, 0u, sizeof(header), header, "Header(current)")) {
        return false;
    }
    uint32_t bitmap = 0u;
    char tag[4] = {0};
    if (!ValidateHeaderEvidence(header, &bitmap, tag) ||
        memcmp(tag, "P48E", 4u) != 0) {
        printf("Smoke48a: current FPGA is not P48E load-capable tag='%c%c%c%c'\n",
               tag[0], tag[1], tag[2], tag[3]);
        return false;
    }

    if (!WriteSlotStream(storage, slot, top,   0u, 8u, "Top[0]") ||
        !WriteSlotStream(storage, slot, top,   8u, 8u, "Top[1]") ||
        !WriteSlotStream(storage, slot, cpu,   0u, 2u, "CPU_GBSE") ||
        !WriteSlotStream(storage, slot, cpu,   2u, 8u, "CPU_CPUREGS") ||
        !WriteSlotStream(storage, slot, cpu,  10u, 8u, "CPU_T80_1") ||
        !WriteSlotStream(storage, slot, cpu,  18u, 7u, "CPU_T80_2") ||
        !WriteSlotStream(storage, slot, cpu,  25u, 8u, "CPU_T80_3") ||
        !WriteSlotStream(storage, slot, cpu,  33u, 7u, "CPU_T80_4") ||
        !WriteSlotStream(storage, slot, timer, 0u, 8u, "Timer") ||
        !WriteSlotStream(storage, slot, hram,  0u, (uint16_t)hram->length, "HRAM") ||
        !WriteSlotStream(storage, slot, wram,  0u, (uint16_t)wram->length, "WRAM")) {
        return false;
    }

    if (!WriteCommit()) {
        return false;
    }

    return VerifyReadbackCrc(top, "Top") &&
           VerifyReadbackCrc(cpu, "CPU") &&
           VerifyReadbackCrc(timer, "Timer") &&
           VerifyReadbackCrc(hram, "HRAM") &&
           VerifyReadbackCrc(wram, "WRAM");
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

    bool ok = ValidateSlot(&storage, &result);
    if (!ok) {
        PrintResult(&result);
        return false;
    }

    uint32_t slot = 0u;
    uint32_t generation = 0u;
    FusionStorageResult_t sr =
        FusionStorage_FindNewestValidSlot(&storage, &slot, &generation);
    if (sr != kFusionStorage_Ok) {
        PrintResult(&result);
        return false;
    }

    FusionStateHeader_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    sr = FusionStorage_ReadSlotHeader(&storage, slot, &hdr);
    if (sr != kFusionStorage_Ok) {
        PrintResult(&result);
        return false;
    }
    FusionRegionDirectory_t dirs[kSmoke48aRegionCount];
    sr = FusionStorage_ReadSlotBytes(&storage,
                                     slot,
                                     hdr.region_table_offset,
                                     (uint8_t *)dirs,
                                     sizeof(dirs));
    if (sr != kFusionStorage_Ok) {
        PrintResult(&result);
        return false;
    }

    bool session_started = false;
    PwrMgr_IdleTimerSuspend();
    PauseFpgaTraffic();
    FPGA_Rx_ResetParser();
    ResetUartCache();
    if (!FPGA_UartOwnerAcquire(pdMS_TO_TICKS(2000))) {
        printf("Smoke48a: UART owner unavailable for load\n");
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
        ok = SendCtlAndExpectAck("BEGIN_TEST_RW",
                                 BuildBeginTestRW,
                                 (uint8_t)kFusionOp_BeginTestRW);
        if (!ok) {
            break;
        }
        session_started = true;
        printf("Smoke48a: load mixed session active slot=%lu generation=%lu\n",
               (unsigned long)slot,
               (unsigned long)generation);
        ok = LoadSlotToFpga(&storage, slot, dirs);
    } while (false);

    bool cleanup_ok = true;
    if (session_started) {
        cleanup_ok = EndSessionCleanup();
    }

    ResumeFpgaTraffic();
    FPGA_UartOwnerRelease();
    PwrMgr_IdleTimerResume();

    result.load_ready = ok && cleanup_ok;
    PrintResult(&result);
    return result.load_ready;
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
    uint8_t current_rom_header[FUSION_ROM_HEADER_BYTES];
    uint32_t current_game_id = 0u;
    uint32_t staged_slot = UINT32_MAX;
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

        ok = VerifyCurrentFpgaHeader() &&
             ReadCurrentCartHeader(current_rom_header, &current_game_id);
        if (!ok) {
            break;
        }

        FusionStorageResult_t sr = FusionStorage_BeginInactiveSlotWrite(&storage);
        if (sr != kFusionStorage_Ok) {
            printf("Smoke48a: begin slot write failed result=%d\n", (int)sr);
            ok = false;
            break;
        }
        if (storage.active_slot < 0) {
            printf("Smoke48a: begin slot write returned no active slot\n");
            ok = false;
            break;
        }
        staged_slot = (uint32_t)storage.active_slot;

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
    hdr.game_id_hash = current_game_id;
    memcpy(hdr.fpga_version, savestate_tag, sizeof(savestate_tag));
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
    if (!ValidateSlot(&storage, &result)) {
        printf("Smoke48a: post-commit validation failed; invalidating slot %lu\n",
               (unsigned long)staged_slot);
        if (staged_slot != UINT32_MAX) {
            (void)FusionStorage_InvalidateSlot(&storage, staged_slot);
        }
        result.slot_committed = false;
        ResumeFpgaTraffic();
        FPGA_UartOwnerRelease();
        PwrMgr_IdleTimerResume();
        PrintResult(&result);
        SavestateToast_Show(kSavestateToast_Failed, SAVESTATE_TOAST_DEFAULT_MS);
        return false;
    }
    ResumeFpgaTraffic();
    FPGA_UartOwnerRelease();
    PwrMgr_IdleTimerResume();
    PrintResult(&result);

    /*
     * Update OSD Save Slot menu entry with new slot's game-id hash so
     * users see what's saved.  Display short hex of game_id_hash; full
     * GB title support deferred per lock §4.8d.
     */
    if (result.slot_committed) {
        FusionStorage_t s;
        if (FusionStorage_InitFromPartition(&s, "savestate") == kFusionStorage_Ok) {
            uint32_t s_slot, s_gen;
            if (FusionStorage_FindNewestValidSlot(&s, &s_slot, &s_gen) ==
                kFusionStorage_Ok) {
                FusionStateHeader_t hdr;
                if (FusionStorage_ReadSlotHeader(&s, s_slot, &hdr) ==
                    kFusionStorage_Ok) {
                    char text[24];
                    snprintf(text, sizeof(text), "%08lX",
                             (unsigned long)hdr.game_id_hash);
                    SaveSlot_SetText(text);
                }
            }
        }
    }

    SavestateToast_Show(result.slot_committed ? kSavestateToast_Saved
                                              : kSavestateToast_Failed,
                        SAVESTATE_TOAST_DEFAULT_MS);
    return result.load_ready;
}

static int Smoke48aCommand(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: smoke48a save|validate|load|reboot\n");
        return 1;
    }
    if (strcmp(argv[1], "save") == 0) {
        return FusionSavestate_QuickSaveSlot0() ? 0 : 1;
    }
    if (strcmp(argv[1], "validate") == 0) {
        Smoke48aResult_t result = {0};
        FusionStorage_t storage;
        if (!InitStorage(&storage)) {
            PrintResult(&result);
            return 1;
        }
        return ValidateSlot(&storage, &result) ? 0 : 1;
    }
    if (strcmp(argv[1], "load") == 0) {
        return FusionSavestate_QuickLoadSlot0_PathE() ? 0 : 1;
    }
    if (strcmp(argv[1], "reboot") == 0) {
        printf("Smoke48a: rebooting now; run 'smoke48a validate' after boot\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return 0;
    }
    printf("Usage: smoke48a save|validate|load|reboot\n");
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

/* ===== Path-e (4.8d) chord-triggered dispatcher ===== */

enum {
    kChordQueueLen      = 4,        /* allow a small backlog */
    kChordTaskStack     = 16384,    /* save/load are stack-heavy */
    kChordTaskPriority  = 5,
};

static QueueHandle_t s_chord_queue = NULL;
static StaticQueue_t s_chord_queue_storage;
static uint8_t       s_chord_queue_buf[kChordQueueLen * sizeof(ButtonChordEvent_t)];

static void ChordDispatcherTask(void *arg)
{
    (void)arg;
    for (;;) {
        ButtonChordEvent_t evt = kButtonChord_None;
        if (xQueueReceive(s_chord_queue, &evt, portMAX_DELAY) == pdPASS) {
            switch (evt) {
                case kButtonChord_SaveRequested:
                    ESP_LOGI(TAG, "chord: SaveRequested -> QuickSaveSlot0");
                    (void)FusionSavestate_QuickSaveSlot0();
                    break;
                case kButtonChord_LoadRequested:
                    ESP_LOGI(TAG, "chord: LoadRequested -> QuickLoadSlot0_PathE");
                    (void)FusionSavestate_QuickLoadSlot0_PathE();
                    break;
                default:
                    break;
            }
        }
    }
}

void FusionSavestate_StartChordDispatcher(void)
{
    if (s_chord_queue != NULL) {
        return;  /* already started */
    }
    s_chord_queue = xQueueCreateStatic(kChordQueueLen,
                                       sizeof(ButtonChordEvent_t),
                                       s_chord_queue_buf,
                                       &s_chord_queue_storage);
    if (s_chord_queue == NULL) {
        ESP_LOGE(TAG, "chord queue create failed");
        return;
    }
    BaseType_t r = xTaskCreate(ChordDispatcherTask,
                               "ss_chord",
                               kChordTaskStack,
                               NULL,
                               kChordTaskPriority,
                               NULL);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "chord task create failed: %d", (int)r);
    }
}

void FusionSavestate_PostChordRequest(ButtonChordEvent_t event)
{
    if (s_chord_queue == NULL || event == kButtonChord_None) {
        return;
    }
    /* Non-blocking send: if dispatcher is busy, drop the event silently
     * (prevents button-mash from queuing multiple saves). */
    (void) xQueueSend(s_chord_queue, &event, 0);
}
