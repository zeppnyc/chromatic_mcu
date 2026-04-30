#include "fusion_savestate_transport_probe.h"

#include "driver/uart.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "fpga_common.h"
#include "fpga_rx.h"
#include "fpga_tx.h"
#include "fusion_savestate.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    kProbeFrameTimeoutMs = 3000,
    kProbeInterRegionMs  = 10,
    kProbeFinalEndMs     = 50,
    kProbeBootDelayMs    = 2000,
    kProbeRegionCount    = 6,
    kProbeRxCacheBytes   = 512,
    kProbeOldRxBufBytes  = 1024,
    kProbeCaptureStackBytes = 12288,
    kProbeCapturePriority = configMAX_PRIORITIES - 2,
    kProbeOwnerAcquireMs = 1000,
    kProbePreflightDrainMs = 60,
    kProbeCleanupAckMs = 500,
    kProbePostCleanupDrainMs = 60,
    kProbeBoundaryDrainMs = 10,
    kProbeBoundaryRawDrainMs = 2,
};

static const char *TAG = "SvProbe";
static const uint32_t kExpectedBitmap = 0x0000120Fu;

typedef struct {
    const char *name;
    uint8_t region;
    uint16_t length;
    bool validate;
} ProbeRegion_t;

typedef enum {
    kProbePhase_NextRegion = 0,
    kProbePhase_WaitingAckAccepted,
    kProbePhase_ReadingData,
    kProbePhase_WaitingAckDone,
    kProbePhase_BoundaryDrain,
} ProbePhase_t;

typedef struct {
    const char *type;
    uint8_t addr;
    uint8_t opcode;
    uint16_t seq;
    uint8_t data_len;
    uint8_t payload_len;
    bool valid;
} ProbeFrameSummary_t;

typedef struct {
    const ProbeRegion_t *previous_region;
    const ProbeRegion_t *current_region;
    ProbePhase_t phase;
    uint32_t boundary_drain_bytes;
    uint32_t boundary_stale_frames;
    uint32_t packets;
    uint32_t bytes;
    uint16_t expected_seq;
    bool ack_accepted;
    bool ack_done;
    bool stream_done_seen;
    ProbeFrameSummary_t last_frame;
} ProbeRegionStats_t;

typedef struct {
    uint8_t buf[kProbeRxCacheBytes];
    size_t pos;
    size_t len;
    uint32_t skipped_non_marker;
    uint32_t bad_payload_len;
    uint32_t bad_crc;
} ProbeReader_t;

typedef struct {
    const char *mode_name;
    bool reduced_logging;
    bool boot_mode;
    bool capture_task;
} ProbeConfig_t;

typedef struct {
    bool tx_paused;
    bool rx_paused;
    bool lvgl_paused;
} ProbePauseState_t;

typedef struct {
    bool owner_acquired;
    bool cleanup_ack;
    bool begin_save_ack;
    uint32_t initial_drain_bytes;
    uint32_t post_cleanup_drain_bytes;
    uint32_t stale_frames_during_cleanup;
    uint32_t skipped_non_marker;
    uint32_t bad_payload_len;
    uint32_t bad_crc;
    ProbePauseState_t pause;
} ProbePreflight_t;

typedef struct {
    ProbeConfig_t cfg;
    TaskHandle_t caller;
    bool ok;
} ProbeTaskRequest_t;

typedef struct {
    const ProbeRegion_t *region;
    uint16_t expected_seq;
    uint16_t got_seq;
    uint32_t bytes_received;
    uint32_t packets_received;
    int64_t elapsed_ms;
    uint8_t frame_addr;
    uint8_t frame_op;
    uint8_t frame_payload_len;
    size_t uart_buffered;
    size_t heap_free;
    size_t heap_min_free;
    uint32_t skipped_non_marker;
    uint32_t bad_payload_len;
    uint32_t bad_crc;
    bool has_got_seq;
    bool stale_data_while_ack;
    bool stale_ack_done_while_ack;
    bool final_end_ack;
    const char *reason;
} ProbeFailure_t;

static const ProbeRegion_t kRegions[kProbeRegionCount] = {
    { "Header", 0x00u, 16u,    true  },
    { "Top",    0x01u, 16u,    true  },
    { "CPU",    0x02u, 40u,    false },
    { "Timer",  0x03u, 8u,     true  },
    { "HRAM",   0x09u, 127u,   false },
    { "WRAM",   0x0Cu, 32768u, false },
};

static uint8_t *s_rx_buf = NULL;
static size_t s_rx_buf_len = 0u;
static bool s_lvgl_suspended = false;

extern TaskHandle_t FusionApp_GetLvglTimerTaskHandle(void);

static bool ReadDecodedFrameWithTimeout(ProbeReader_t *reader,
                                        const char *context,
                                        uint32_t timeout_ms,
                                        bool print_timeout,
                                        FusionV2Decoded_t *decoded);
static uint32_t DrainUartRxForMs(uint32_t duration_ms);

static bool EnsureRxBuf(uint16_t need)
{
    if (need <= s_rx_buf_len) {
        return true;
    }
    uint8_t *p = (uint8_t *)heap_caps_realloc(s_rx_buf, need, MALLOC_CAP_8BIT);
    if (p == NULL) {
        printf("SvProbe: heap alloc %u failed\n", (unsigned)need);
        return false;
    }
    s_rx_buf = p;
    s_rx_buf_len = need;
    return true;
}

static void ReaderReset(ProbeReader_t *reader)
{
    memset(reader, 0, sizeof(*reader));
}

static bool ReaderByte(ProbeReader_t *reader, uint8_t *out, int64_t deadline_us)
{
    while (esp_timer_get_time() < deadline_us) {
        if (reader->pos < reader->len) {
            *out = reader->buf[reader->pos++];
            return true;
        }
        reader->pos = 0u;
        reader->len = 0u;

        const int got = uart_read_bytes(UART_NUM_1,
                                        reader->buf,
                                        sizeof(reader->buf),
                                        pdMS_TO_TICKS(10));
        if (got > 0) {
            reader->len = (size_t)got;
        }
    }
    return false;
}

static void SnapshotRuntimeStats(ProbeFailure_t *failure)
{
    size_t buffered = 0u;
    (void)uart_get_buffered_data_len(UART_NUM_1, &buffered);
    failure->uart_buffered = buffered;
    failure->heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    failure->heap_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
}

static const char *PhaseName(ProbePhase_t phase)
{
    switch (phase) {
        case kProbePhase_WaitingAckAccepted: return "waiting_ack_accepted";
        case kProbePhase_ReadingData: return "reading_data";
        case kProbePhase_WaitingAckDone: return "waiting_ack_done";
        case kProbePhase_BoundaryDrain: return "boundary_drain";
        case kProbePhase_NextRegion:
        default: return "next_region";
    }
}

static const char *FrameTypeName(const FusionV2Decoded_t *frame)
{
    if (frame == NULL) {
        return "none";
    }
    if (frame->addr == (uint8_t)kFusionAddr_StateCtl) {
        return "CTL";
    }
    if (frame->addr == (uint8_t)kFusionAddr_StateData) {
        return "DATA";
    }
    return "OTHER";
}

static void CaptureFrameSummary(ProbeFrameSummary_t *summary,
                                const FusionV2Decoded_t *frame)
{
    memset(summary, 0, sizeof(*summary));
    summary->type = FrameTypeName(frame);
    if (frame == NULL) {
        return;
    }
    summary->addr = frame->addr;
    summary->opcode = frame->ctl_opcode;
    summary->seq = frame->data_seq;
    summary->data_len = frame->data_len;
    summary->payload_len = frame->payload_len;
    summary->valid = true;
}

static void PrintRegionSummary(const ProbeRegionStats_t *stats,
                               const char *status,
                               int64_t elapsed_ms)
{
    printf("SvProbe: REGION_SUMMARY %s status=%s id=0x%02X length=%u "
           "packets=%lu bytes=%lu elapsed_ms=%lld ack_accepted=%s "
           "ack_done=%s boundary_stale_frames=%lu boundary_drain_bytes=%lu\n",
           stats->current_region != NULL ? stats->current_region->name : "(none)",
           status,
           stats->current_region != NULL ? (unsigned)stats->current_region->region : 0u,
           stats->current_region != NULL ? (unsigned)stats->current_region->length : 0u,
           (unsigned long)stats->packets,
           (unsigned long)stats->bytes,
           (long long)elapsed_ms,
           stats->ack_accepted ? "yes" : "no",
           stats->ack_done ? "yes" : "no",
           (unsigned long)stats->boundary_stale_frames,
           (unsigned long)stats->boundary_drain_bytes);
}

static void PrintRegionFailureContext(const ProbeRegionStats_t *stats,
                                      const char *expected_frame,
                                      const FusionV2Decoded_t *actual,
                                      const ProbeReader_t *reader)
{
    ProbeFrameSummary_t actual_summary;
    if (actual != NULL) {
        CaptureFrameSummary(&actual_summary, actual);
    } else if (stats->last_frame.valid) {
        actual_summary = stats->last_frame;
    } else {
        CaptureFrameSummary(&actual_summary, NULL);
    }
    printf("SvProbe: REGION_FAIL phase=%s expected=%s actual_type=%s "
           "actual_addr=0x%02X actual_opcode=0x%02X actual_seq=%u "
           "actual_data_len=%u actual_payload_len=%u expected_seq=%u "
           "previous_region=%s current_region=%s boundary_stale_frames=%lu "
           "boundary_drain_bytes=%lu bad_len=%lu bad_crc=%lu resync_non_marker=%lu\n",
           PhaseName(stats->phase),
           expected_frame,
           actual_summary.type,
           actual_summary.addr,
           actual_summary.opcode,
           (unsigned)actual_summary.seq,
           (unsigned)actual_summary.data_len,
           (unsigned)actual_summary.payload_len,
           (unsigned)stats->expected_seq,
           stats->previous_region != NULL ? stats->previous_region->name : "(none)",
           stats->current_region != NULL ? stats->current_region->name : "(none)",
           (unsigned long)stats->boundary_stale_frames,
           (unsigned long)stats->boundary_drain_bytes,
           (unsigned long)reader->bad_payload_len,
           (unsigned long)reader->bad_crc,
           (unsigned long)reader->skipped_non_marker);
}

static void PrintFailure(const ProbeFailure_t *f)
{
    printf("SvProbe: FIRST_FAIL reason=%s region=%s id=0x%02X "
           "bytes=%lu packets=%lu elapsed_ms=%lld\n",
           f->reason,
           f->region != NULL ? f->region->name : "(none)",
           f->region != NULL ? (unsigned)f->region->region : 0u,
           (unsigned long)f->bytes_received,
           (unsigned long)f->packets_received,
           (long long)f->elapsed_ms);
    printf("SvProbe: FIRST_FAIL seq expected=%u got=%s%u frame_addr=0x%02X "
           "frame_op=0x%02X payload_len=%u\n",
           (unsigned)f->expected_seq,
           f->has_got_seq ? "" : "n/a/",
           f->has_got_seq ? (unsigned)f->got_seq : 0u,
           f->frame_addr,
           f->frame_op,
           f->frame_payload_len);
    printf("SvProbe: FIRST_FAIL uart_buffered=%u heap_free=%u heap_min=%u "
           "resync_non_marker=%lu bad_len=%lu bad_crc=%lu\n",
           (unsigned)f->uart_buffered,
           (unsigned)f->heap_free,
           (unsigned)f->heap_min_free,
           (unsigned long)f->skipped_non_marker,
           (unsigned long)f->bad_payload_len,
           (unsigned long)f->bad_crc);
    printf("SvProbe: FIRST_FAIL stale_data_while_ack=%s "
           "stale_ack_done_while_ack=%s final_end_ack=%s\n",
           f->stale_data_while_ack ? "yes" : "no",
           f->stale_ack_done_while_ack ? "yes" : "no",
           f->final_end_ack ? "yes" : "no");
}

static bool Fail(ProbeFailure_t *failure,
                 ProbeReader_t *reader,
                 const ProbeRegion_t *region,
                 const char *reason,
                 const FusionV2Decoded_t *frame,
                 uint16_t expected_seq,
                 uint32_t bytes_received,
                 uint32_t packets_received,
                 int64_t t0)
{
    memset(failure, 0, sizeof(*failure));
    failure->region = region;
    failure->reason = reason;
    failure->expected_seq = expected_seq;
    failure->bytes_received = bytes_received;
    failure->packets_received = packets_received;
    failure->elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    failure->skipped_non_marker = reader->skipped_non_marker;
    failure->bad_payload_len = reader->bad_payload_len;
    failure->bad_crc = reader->bad_crc;
    failure->stale_data_while_ack =
        (strcmp(reason, "stale_DATA_while_expecting_ACK") == 0);
    failure->stale_ack_done_while_ack =
        (strcmp(reason, "stale_ACK_DONE_while_expecting_ACK") == 0);
    if (frame != NULL) {
        failure->frame_addr = frame->addr;
        failure->frame_op = frame->ctl_opcode;
        failure->frame_payload_len = frame->payload_len;
        if (frame->addr == (uint8_t)kFusionAddr_StateData) {
            failure->got_seq = frame->data_seq;
            failure->has_got_seq = true;
        }
    }
    SnapshotRuntimeStats(failure);
    PrintFailure(failure);
    return false;
}

static bool ReadDecodedFrame(ProbeReader_t *reader,
                             const char *context,
                             FusionV2Decoded_t *decoded)
{
    return ReadDecodedFrameWithTimeout(reader,
                                       context,
                                       kProbeFrameTimeoutMs,
                                       true,
                                       decoded);
}

static bool ReadDecodedFrameWithTimeout(ProbeReader_t *reader,
                                        const char *context,
                                        uint32_t timeout_ms,
                                        bool print_timeout,
                                        FusionV2Decoded_t *decoded)
{
    uint8_t raw[FUSION_V2_MAX_FRAME] = {0};
    const int64_t deadline = esp_timer_get_time()
                           + ((int64_t)timeout_ms * 1000);

    while (esp_timer_get_time() < deadline) {
        uint8_t b = 0u;
        if (!ReaderByte(reader, &b, deadline)) {
            if (print_timeout) {
                printf("SvProbe: RX timeout waiting for marker during %s\n", context);
            }
            return false;
        }
        if (b != FUSION_V2_HEADER_MARKER) {
            reader->skipped_non_marker++;
            continue;
        }

        raw[0] = b;
        if (!ReaderByte(reader, &raw[1], deadline) ||
            !ReaderByte(reader, &raw[2], deadline)) {
            if (print_timeout) {
                printf("SvProbe: RX timeout waiting for header during %s\n", context);
            }
            return false;
        }

        const uint8_t payload_len = raw[2];
        if (payload_len > FUSION_V2_MAX_PAYLOAD) {
            reader->bad_payload_len++;
            printf("SvProbe: RX resync bad payload len %u during %s\n",
                   (unsigned)payload_len, context);
            continue;
        }

        const size_t frame_len = 3u + payload_len + 1u;
        for (size_t i = 3u; i < frame_len; ++i) {
            if (!ReaderByte(reader, &raw[i], deadline)) {
                if (print_timeout) {
                    printf("SvProbe: RX timeout waiting for payload/crc during %s\n",
                           context);
                }
                return false;
            }
        }

        if (!FusionSavestate_DecodeV2Frame(raw, frame_len, decoded)) {
            reader->bad_crc++;
            printf("SvProbe: RX resync decode/CRC failed during %s\n", context);
            continue;
        }
        return true;
    }

    if (print_timeout) {
        printf("SvProbe: RX timeout waiting for valid frame during %s\n", context);
    }
    return false;
}

static bool SendFrame(const ProbeConfig_t *cfg,
                      const char *label,
                      const FusionV2Frame_t *frame)
{
    if (frame == NULL || frame->length == 0u) {
        printf("SvProbe: %s invalid frame\n", label);
        return false;
    }
    if (!cfg->reduced_logging) {
        printf("SvProbe: TX %s:", label);
        for (uint8_t i = 0; i < frame->length; ++i) {
            printf(" %02X", frame->bytes[i]);
        }
        printf("\n");
    }

    const int written = uart_write_bytes(UART_NUM_1,
                                         (const char *)frame->bytes,
                                         frame->length);
    (void)uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(250));
    if (written != (int)frame->length) {
        printf("SvProbe: TX %s failed wrote=%d expected=%u\n",
               label, written, (unsigned)frame->length);
        return false;
    }
    return true;
}

static bool ExpectCtl(const ProbeConfig_t *cfg,
                      ProbeReader_t *reader,
                      ProbeFailure_t *failure,
                      const ProbeRegion_t *region,
                      uint8_t opcode,
                      uint8_t ack_for,
                      const char *context,
                      uint16_t expected_seq,
                      uint32_t bytes_received,
                      uint32_t packets_received,
                      int64_t t0)
{
    FusionV2Decoded_t d;
    if (!ReadDecodedFrame(reader, context, &d)) {
        return Fail(failure, reader, region, "timeout_or_unrecoverable_resync",
                    NULL, expected_seq, bytes_received, packets_received, t0);
    }
    if (d.addr == (uint8_t)kFusionAddr_StateData) {
        return Fail(failure, reader, region, "stale_DATA_while_expecting_ACK",
                    &d, expected_seq, bytes_received, packets_received, t0);
    }
    if (d.addr != (uint8_t)kFusionAddr_StateCtl) {
        return Fail(failure, reader, region, "unexpected_addr_while_expecting_ACK",
                    &d, expected_seq, bytes_received, packets_received, t0);
    }
    if (d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        return Fail(failure, reader, region, "FPGA_ERROR",
                    &d, expected_seq, bytes_received, packets_received, t0);
    }
    if (opcode == (uint8_t)kFusionOp_AckAccepted &&
        d.ctl_opcode == (uint8_t)kFusionOp_AckDone) {
        return Fail(failure, reader, region, "stale_ACK_DONE_while_expecting_ACK",
                    &d, expected_seq, bytes_received, packets_received, t0);
    }
    if (d.ctl_opcode != opcode || d.ack_for_opcode != ack_for) {
        return Fail(failure, reader, region, "unexpected_CTL",
                    &d, expected_seq, bytes_received, packets_received, t0);
    }
    (void)cfg;
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

static bool SendCtlAndExpectAck(const ProbeConfig_t *cfg,
                                ProbeReader_t *reader,
                                ProbeFailure_t *failure,
                                const char *label,
                                bool (*builder)(FusionV2Frame_t *out),
                                uint8_t ack_for,
                                bool final_end)
{
    const int64_t t0 = esp_timer_get_time();
    FusionV2Frame_t frame;
    if (!builder(&frame)) {
        printf("SvProbe: build %s failed\n", label);
        return false;
    }
    if (!SendFrame(cfg, label, &frame)) {
        return false;
    }
    const bool ok = ExpectCtl(cfg, reader, failure, NULL,
                              (uint8_t)kFusionOp_AckAccepted,
                              ack_for, label, 0u, 0u, 0u, t0);
    if (final_end) {
        failure->final_end_ack = ok;
    }
    return ok;
}

static bool ValidateHeader(const uint8_t *data)
{
    const uint32_t bitmap = (uint32_t)data[8]
                          | ((uint32_t)data[9] << 8)
                          | ((uint32_t)data[10] << 16)
                          | ((uint32_t)data[11] << 24);
    if (memcmp(data, "FUSS", 4u) != 0) {
        printf("SvProbe: Header magic mismatch\n");
        return false;
    }
    if (bitmap != kExpectedBitmap) {
        printf("SvProbe: Header bitmap mismatch got=0x%08lX want=0x%08lX\n",
               (unsigned long)bitmap,
               (unsigned long)kExpectedBitmap);
        return false;
    }
    if (memcmp(&data[12], "P470", 4u) != 0) {
        printf("SvProbe: Header tag mismatch got='%c%c%c%c' want='P470'\n",
               data[12], data[13], data[14], data[15]);
        return false;
    }
    return true;
}

static bool ValidateRegionLight(const ProbeRegion_t *region,
                                const uint8_t *data,
                                uint16_t length)
{
    if (region->region == 0x00u) {
        return ValidateHeader(data);
    }
    if (region->region == 0x01u) {
        for (uint16_t i = 10u; i < length; ++i) {
            if (data[i] != 0u) {
                printf("SvProbe: Top padding byte %u nonzero=0x%02X\n",
                       (unsigned)i, data[i]);
                return false;
            }
        }
    } else if (region->region == 0x03u) {
        if (data[6] != 0u || data[7] != 0u) {
            printf("SvProbe: Timer padding mismatch b6=0x%02X b7=0x%02X\n",
                   data[6], data[7]);
            return false;
        }
    }
    return true;
}

static void PrintBytes(const char *label, const uint8_t *data, uint16_t count)
{
    printf(" %s:", label);
    for (uint16_t i = 0; i < count; ++i) {
        printf(" %02X", data[i]);
    }
}

static bool ExpectReadStreamAckAccepted(ProbeReader_t *reader,
                                        ProbeFailure_t *failure,
                                        ProbeRegionStats_t *stats,
                                        int64_t t0)
{
    const uint32_t bad_len0 = reader->bad_payload_len;
    const uint32_t bad_crc0 = reader->bad_crc;
    const int64_t deadline = esp_timer_get_time()
                           + ((int64_t)kProbeFrameTimeoutMs * 1000);

    while (esp_timer_get_time() < deadline) {
        FusionV2Decoded_t d;
        const int64_t now = esp_timer_get_time();
        const uint32_t remain_ms =
            (uint32_t)((deadline > now) ? ((deadline - now + 999) / 1000) : 0);
        if (remain_ms == 0u ||
            !ReadDecodedFrameWithTimeout(reader,
                                         "READ_STREAM_BEGIN ack",
                                         remain_ms,
                                         true,
                                         &d)) {
            PrintRegionSummary(stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
            PrintRegionFailureContext(stats, "ACK_ACCEPTED(ReadStreamBegin)",
                                      NULL, reader);
            return Fail(failure, reader, stats->current_region,
                        "timeout_or_unrecoverable_resync",
                        NULL, stats->expected_seq, stats->bytes,
                        stats->packets, t0);
        }
        CaptureFrameSummary(&stats->last_frame, &d);

        if (reader->bad_payload_len != bad_len0 || reader->bad_crc != bad_crc0) {
            PrintRegionSummary(stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
            PrintRegionFailureContext(stats, "ACK_ACCEPTED(ReadStreamBegin)",
                                      &d, reader);
            return Fail(failure, reader, stats->current_region,
                        reader->bad_payload_len != bad_len0 ?
                            "bad_len_during_ack_accepted" :
                            "bad_crc_during_ack_accepted",
                        &d, stats->expected_seq, stats->bytes,
                        stats->packets, t0);
        }

        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
            d.ack_for_opcode == (uint8_t)kFusionOp_ReadStreamBegin) {
            stats->ack_accepted = true;
            if (stats->boundary_stale_frames != 0u) {
                PrintRegionSummary(stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
                PrintRegionFailureContext(stats, "clean ACK_ACCEPTED(ReadStreamBegin)",
                                          &d, reader);
                return Fail(failure, reader, stats->current_region,
                            "stale_frame_before_ack_accepted",
                            &d, stats->expected_seq, stats->bytes,
                            stats->packets, t0);
            }
            return true;
        }

        stats->boundary_stale_frames++;
        if (d.addr == (uint8_t)kFusionAddr_StateData ||
            (d.addr == (uint8_t)kFusionAddr_StateCtl &&
             d.ctl_opcode == (uint8_t)kFusionOp_AckDone)) {
            continue;
        }

        PrintRegionSummary(stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
        PrintRegionFailureContext(stats, "ACK_ACCEPTED(ReadStreamBegin)",
                                  &d, reader);
        return Fail(failure, reader, stats->current_region,
                    "unexpected_frame_while_expecting_ack_accepted",
                    &d, stats->expected_seq, stats->bytes, stats->packets, t0);
    }

    PrintRegionSummary(stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
    PrintRegionFailureContext(stats, "ACK_ACCEPTED(ReadStreamBegin)",
                              NULL, reader);
    return Fail(failure, reader, stats->current_region,
                "timeout_waiting_ack_accepted",
                NULL, stats->expected_seq, stats->bytes, stats->packets, t0);
}

static bool DrainRegionBoundary(ProbeReader_t *reader,
                                ProbeFailure_t *failure,
                                ProbeRegionStats_t *stats,
                                int64_t t0)
{
    stats->phase = kProbePhase_BoundaryDrain;
    const uint32_t bad_len0 = reader->bad_payload_len;
    const uint32_t bad_crc0 = reader->bad_crc;
    const int64_t deadline = esp_timer_get_time()
                           + ((int64_t)kProbeBoundaryDrainMs * 1000);

    while (esp_timer_get_time() < deadline) {
        FusionV2Decoded_t d;
        const int64_t now = esp_timer_get_time();
        const uint32_t remain_ms =
            (uint32_t)((deadline > now) ? ((deadline - now + 999) / 1000) : 0);
        if (remain_ms == 0u ||
            !ReadDecodedFrameWithTimeout(reader,
                                         "boundary_drain",
                                         remain_ms,
                                         false,
                                         &d)) {
            break;
        }

        CaptureFrameSummary(&stats->last_frame, &d);
        stats->boundary_stale_frames++;
        stats->boundary_drain_bytes += (uint32_t)d.payload_len + 4u;
    }

    if (reader->bad_payload_len != bad_len0 || reader->bad_crc != bad_crc0 ||
        stats->boundary_stale_frames != 0u) {
        stats->boundary_drain_bytes += DrainUartRxForMs(kProbeBoundaryRawDrainMs);
        PrintRegionSummary(stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
        PrintRegionFailureContext(stats, "quiet boundary before next region",
                                  NULL, reader);
        const bool failed = Fail(failure, reader, stats->current_region,
                    stats->boundary_stale_frames != 0u ?
                        "boundary_stale_frame" :
                        (reader->bad_payload_len != bad_len0 ?
                            "bad_len_during_boundary_drain" :
                            "bad_crc_during_boundary_drain"),
                    NULL, stats->expected_seq, stats->bytes,
                    stats->packets, t0);
        ReaderReset(reader);
        return failed;
    }

    stats->boundary_drain_bytes += DrainUartRxForMs(kProbeBoundaryRawDrainMs);
    ReaderReset(reader);
    return true;
}

static bool ReadRegion(const ProbeConfig_t *cfg,
                       ProbeReader_t *reader,
                       ProbeFailure_t *failure,
                       const ProbeRegion_t *previous_region,
                       const ProbeRegion_t *region)
{
    if (!EnsureRxBuf(region->length)) {
        return false;
    }
    uint8_t *data = s_rx_buf;
    memset(data, 0, region->length);

    uint32_t received = 0u;
    uint32_t packets = 0u;
    uint16_t expected_seq = 0u;
    const int64_t t0 = esp_timer_get_time();
    ProbeRegionStats_t stats = {
        .previous_region = previous_region,
        .current_region = region,
        .phase = kProbePhase_NextRegion,
    };

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildReadStreamBegin(region->region, 0u,
                                              region->length, &begin)) {
        printf("SvProbe: build READ_STREAM_BEGIN %s failed\n", region->name);
        return false;
    }

    printf("SvProbe: BEGIN region=%s id=0x%02X length=%u\n",
           region->name, (unsigned)region->region, (unsigned)region->length);
    if (!SendFrame(cfg, "READ_STREAM_BEGIN", &begin)) {
        return false;
    }
    stats.phase = kProbePhase_WaitingAckAccepted;
    if (!ExpectReadStreamAckAccepted(reader, failure, &stats, t0)) {
        return false;
    }

    stats.phase = kProbePhase_ReadingData;
    while (received < region->length) {
        FusionV2Decoded_t d;
        if (!ReadDecodedFrame(reader, region->name, &d)) {
            stats.expected_seq = expected_seq;
            stats.bytes = received;
            stats.packets = packets;
            PrintRegionSummary(&stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
            PrintRegionFailureContext(&stats, "STATE_DATA", NULL, reader);
            return Fail(failure, reader, region,
                        "timeout_or_unrecoverable_resync",
                        NULL, expected_seq, received, packets, t0);
        }
        CaptureFrameSummary(&stats.last_frame, &d);
        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_Error) {
            stats.expected_seq = expected_seq;
            stats.bytes = received;
            stats.packets = packets;
            PrintRegionSummary(&stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
            PrintRegionFailureContext(&stats, "STATE_DATA", &d, reader);
            return Fail(failure, reader, region, "FPGA_ERROR",
                        &d, expected_seq, received, packets, t0);
        }
        if (d.addr != (uint8_t)kFusionAddr_StateData) {
            stats.expected_seq = expected_seq;
            stats.bytes = received;
            stats.packets = packets;
            PrintRegionSummary(&stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
            PrintRegionFailureContext(&stats, "STATE_DATA", &d, reader);
            return Fail(failure, reader, region, "unexpected_frame_in_stream",
                        &d, expected_seq, received, packets, t0);
        }
        if (d.data_seq != expected_seq) {
            stats.expected_seq = expected_seq;
            stats.bytes = received;
            stats.packets = packets;
            PrintRegionSummary(&stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
            PrintRegionFailureContext(&stats, "STATE_DATA seq", &d, reader);
            return Fail(failure, reader, region, "seq_mismatch",
                        &d, expected_seq, received, packets, t0);
        }
        if (d.data_len == 0u ||
            (uint32_t)d.data_len > ((uint32_t)region->length - received)) {
            stats.expected_seq = expected_seq;
            stats.bytes = received;
            stats.packets = packets;
            PrintRegionSummary(&stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
            PrintRegionFailureContext(&stats, "STATE_DATA len", &d, reader);
            return Fail(failure, reader, region, "bad_data_len",
                        &d, expected_seq, received, packets, t0);
        }

        memcpy(&data[received], d.data, d.data_len);
        received += d.data_len;
        packets++;
        expected_seq++;
        stats.bytes = received;
        stats.packets = packets;
        stats.expected_seq = expected_seq;
    }
    stats.stream_done_seen = true;

    stats.phase = kProbePhase_WaitingAckDone;
    FusionV2Decoded_t done;
    if (!ReadDecodedFrame(reader, "READ_STREAM_BEGIN done", &done)) {
        PrintRegionSummary(&stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
        PrintRegionFailureContext(&stats, "ACK_DONE(ReadStreamBegin)", NULL, reader);
        return Fail(failure, reader, region, "timeout_or_unrecoverable_resync",
                    NULL, expected_seq, received, packets, t0);
    }
    CaptureFrameSummary(&stats.last_frame, &done);
    if (done.addr != (uint8_t)kFusionAddr_StateCtl ||
        done.ctl_opcode != (uint8_t)kFusionOp_AckDone ||
        done.ack_for_opcode != (uint8_t)kFusionOp_ReadStreamBegin) {
        PrintRegionSummary(&stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
        PrintRegionFailureContext(&stats, "ACK_DONE(ReadStreamBegin)", &done, reader);
        return Fail(failure, reader, region,
                    done.addr == (uint8_t)kFusionAddr_StateData ?
                        "stale_DATA_while_expecting_ACK_DONE" :
                        "unexpected_frame_while_expecting_ACK_DONE",
                    &done, expected_seq, received, packets, t0);
    }
    stats.ack_done = true;

    if (region->validate && !ValidateRegionLight(region, data, region->length)) {
        PrintRegionSummary(&stats, "FAIL", (esp_timer_get_time() - t0) / 1000);
        PrintRegionFailureContext(&stats, "region validation", NULL, reader);
        return Fail(failure, reader, region, "region_validation_failed",
                    NULL, expected_seq, received, packets, t0);
    }

    if (!DrainRegionBoundary(reader, failure, &stats, t0)) {
        return false;
    }

    const int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    const uint32_t crc = FusionSavestate_Crc32(data, region->length);
    PrintRegionSummary(&stats, "PASS", elapsed_ms);
    printf("SvProbe: REGION %s PASS id=0x%02X packets=%lu bytes=%lu "
           "dt_ms=%lld crc32=%08lX",
           region->name,
           (unsigned)region->region,
           (unsigned long)packets,
           (unsigned long)received,
           (long long)elapsed_ms,
           (unsigned long)crc);
    if (!cfg->reduced_logging) {
        const uint16_t first = region->length < 16u ? region->length : 16u;
        PrintBytes("first", data, first);
        if (region->length > 32u) {
            PrintBytes("last", &data[region->length - 16u], 16u);
        }
    }
    printf("\n");
    return true;
}

static ProbePauseState_t PauseNormalUartTasks(void)
{
    TaskHandle_t *tx = FPGA_GetTxTaskHandle();
    TaskHandle_t *rx = FPGA_GetRxTaskHandle();
    ProbePauseState_t pause = {0};

    TaskHandle_t lvgl = FusionApp_GetLvglTimerTaskHandle();
    if (lvgl != NULL && lvgl != xTaskGetCurrentTaskHandle()) {
        vTaskSuspend(lvgl);
        pause.lvgl_paused = true;
        s_lvgl_suspended = true;
    } else {
        s_lvgl_suspended = false;
    }
    if (tx != NULL && *tx != NULL) {
        FPGA_Tx_Pause();
        pause.tx_paused = true;
    }
    if (rx != NULL && *rx != NULL) {
        FPGA_Rx_Pause();
        pause.rx_paused = true;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    printf("SvProbe: PAUSE tx=%s rx=%s lvgl=%s\n",
           pause.tx_paused ? "yes" : "no",
           pause.rx_paused ? "yes" : "no",
           pause.lvgl_paused ? "yes" : "no");
    return pause;
}

static void ResumeNormalUartTasks(void)
{
    TaskHandle_t *tx = FPGA_GetTxTaskHandle();
    TaskHandle_t *rx = FPGA_GetRxTaskHandle();
    if (rx != NULL && *rx != NULL) {
        FPGA_Rx_Resume();
    }
    if (tx != NULL && *tx != NULL) {
        FPGA_Tx_Resume();
    }
    if (s_lvgl_suspended) {
        TaskHandle_t lvgl = FusionApp_GetLvglTimerTaskHandle();
        if (lvgl != NULL) {
            vTaskResume(lvgl);
        }
        s_lvgl_suspended = false;
    }
}

static uint32_t DrainUartRxForMs(uint32_t duration_ms)
{
    uint8_t tmp[128];
    uint32_t total = 0u;
    const int64_t deadline = esp_timer_get_time() + ((int64_t)duration_ms * 1000);
    while (esp_timer_get_time() < deadline) {
        const int got = uart_read_bytes(UART_NUM_1,
                                        tmp,
                                        sizeof(tmp),
                                        pdMS_TO_TICKS(2));
        if (got > 0) {
            total += (uint32_t)got;
        }
    }
    return total;
}

static bool WaitPreflightCleanupAck(ProbeReader_t *reader,
                                    ProbePreflight_t *preflight)
{
    const uint32_t bad_len0 = reader->bad_payload_len;
    const uint32_t bad_crc0 = reader->bad_crc;
    const uint32_t skipped0 = reader->skipped_non_marker;
    const int64_t deadline = esp_timer_get_time()
                           + ((int64_t)kProbeCleanupAckMs * 1000);

    while (esp_timer_get_time() < deadline) {
        FusionV2Decoded_t d;
        const int64_t now = esp_timer_get_time();
        const uint32_t remain_ms =
            (uint32_t)((deadline > now) ? ((deadline - now + 999) / 1000) : 0);
        if (remain_ms == 0u ||
            !ReadDecodedFrameWithTimeout(reader,
                                         "preflight END_SESSION cleanup",
                                         remain_ms,
                                         false,
                                         &d)) {
            break;
        }

        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
            d.ack_for_opcode == (uint8_t)kFusionOp_EndSession) {
            preflight->cleanup_ack = true;
            break;
        }

        preflight->stale_frames_during_cleanup++;
    }

    preflight->bad_payload_len = reader->bad_payload_len - bad_len0;
    preflight->bad_crc = reader->bad_crc - bad_crc0;
    preflight->skipped_non_marker = reader->skipped_non_marker - skipped0;
    return preflight->cleanup_ack &&
           preflight->bad_payload_len == 0u &&
           preflight->bad_crc == 0u;
}

static void PrintPreflight(const ProbePreflight_t *preflight)
{
    printf("SvProbe: PREFLIGHT owner_acquired=%s normal_tasks_paused=%s "
           "initial_drain_bytes=%lu cleanup_ack=%s "
           "stale_frames_during_cleanup=%lu bad_len=%lu bad_crc=%lu "
           "resync_non_marker=%lu post_cleanup_drain_bytes=%lu "
           "begin_save_ack=%s\n",
           preflight->owner_acquired ? "yes" : "no",
           (preflight->pause.tx_paused && preflight->pause.rx_paused) ? "yes" : "no",
           (unsigned long)preflight->initial_drain_bytes,
           preflight->cleanup_ack ? "yes" : "no",
           (unsigned long)preflight->stale_frames_during_cleanup,
           (unsigned long)preflight->bad_payload_len,
           (unsigned long)preflight->bad_crc,
           (unsigned long)preflight->skipped_non_marker,
           (unsigned long)preflight->post_cleanup_drain_bytes,
           preflight->begin_save_ack ? "yes" : "no");
}

static bool RunPreflight(const ProbeConfig_t *cfg,
                         ProbeReader_t *reader,
                         ProbeFailure_t *failure,
                         ProbePreflight_t *preflight)
{
    memset(preflight, 0, sizeof(*preflight));
    printf("SvProbe: preflight_start\n");

    preflight->owner_acquired =
        FPGA_UartOwnerAcquire(pdMS_TO_TICKS(kProbeOwnerAcquireMs));
    if (!preflight->owner_acquired) {
        PrintPreflight(preflight);
        return Fail(failure, reader, NULL, "state_port_owner_unavailable",
                    NULL, 0u, 0u, 0u, esp_timer_get_time());
    }

    if (!cfg->boot_mode) {
        preflight->pause = PauseNormalUartTasks();
    }
    FPGA_Rx_ResetParser();
    ReaderReset(reader);
    preflight->initial_drain_bytes = DrainUartRxForMs(kProbePreflightDrainMs);
    ReaderReset(reader);

    FusionV2Frame_t end;
    if (!BuildEnd(&end) || !SendFrame(cfg, "PREFLIGHT END_SESSION", &end)) {
        PrintPreflight(preflight);
        return false;
    }

    const bool cleanup_ok = WaitPreflightCleanupAck(reader, preflight);
    preflight->post_cleanup_drain_bytes =
        DrainUartRxForMs(kProbePostCleanupDrainMs);
    ReaderReset(reader);

    if (!cleanup_ok) {
        PrintPreflight(preflight);
        return Fail(failure, reader, NULL, "preflight_cleanup_failed",
                    NULL, 0u, 0u, 0u, esp_timer_get_time());
    }

    FusionV2Frame_t begin;
    if (!BuildBeginSave(&begin) || !SendFrame(cfg, "BEGIN_SAVE", &begin)) {
        PrintPreflight(preflight);
        return false;
    }
    const int64_t t0 = esp_timer_get_time();
    preflight->begin_save_ack =
        ExpectCtl(cfg, reader, failure, NULL,
                  (uint8_t)kFusionOp_AckAccepted,
                  (uint8_t)kFusionOp_BeginSave,
                  "BEGIN_SAVE", 0u, 0u, 0u, t0);
    PrintPreflight(preflight);
    return preflight->begin_save_ack;
}

static bool RunProbe(const ProbeConfig_t *cfg)
{
    ProbeReader_t reader;
    ProbeFailure_t failure;
    ProbePreflight_t preflight;
    ReaderReset(&reader);
    memset(&failure, 0, sizeof(failure));
    memset(&preflight, 0, sizeof(preflight));

    printf("SvProbe: START mode=%s reduced_logging=%s flash_writes=no "
           "rx_driver_old=%u rx_driver_new=%u local_rx_cache=%u "
           "capture_task=%s task_prio=%u core=%d\n",
           cfg->mode_name,
           cfg->reduced_logging ? "yes" : "no",
           (unsigned)kProbeOldRxBufBytes,
           (unsigned)kFPGA_RxConsts_BufferSize,
           (unsigned)kProbeRxCacheBytes,
           cfg->capture_task ? "yes" : "no",
           (unsigned)uxTaskPriorityGet(NULL),
           xPortGetCoreID());

    bool ok = RunPreflight(cfg, &reader, &failure, &preflight);
    if (ok) {
        printf("SvProbe: save session active\n");
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (size_t i = 0; ok && i < kProbeRegionCount; ++i) {
        const ProbeRegion_t *previous = (i == 0u) ? NULL : &kRegions[i - 1u];
        ok = ReadRegion(cfg, &reader, &failure, previous, &kRegions[i]);
        if (ok) {
            vTaskDelay(pdMS_TO_TICKS(kProbeInterRegionMs));
        }
    }

    bool cleanup_ok = false;
    if (preflight.owner_acquired) {
        printf("SvProbe: sending END_SESSION cleanup\n");
        vTaskDelay(pdMS_TO_TICKS(kProbeFinalEndMs));
        cleanup_ok =
            SendCtlAndExpectAck(cfg, &reader, &failure,
                                "END_SESSION cleanup",
                                BuildEnd,
                                (uint8_t)kFusionOp_EndSession,
                                true);
        ok = ok && cleanup_ok;
    }

    if (!cfg->boot_mode) {
        ResumeNormalUartTasks();
    }
    if (preflight.owner_acquired) {
        FPGA_UartOwnerRelease();
    }

    printf("SvProbe: RESULT %s mode=%s cleanup_ack=%s resync_non_marker=%lu "
           "bad_len=%lu bad_crc=%lu heap_free=%u heap_min=%u\n",
           ok ? "PASS" : "FAIL",
           cfg->mode_name,
           cleanup_ok ? "yes" : "no",
           (unsigned long)reader.skipped_non_marker,
           (unsigned long)reader.bad_payload_len,
           (unsigned long)reader.bad_crc,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
    return ok;
}

bool FusionSavestateTransportProbe_RunRepl(bool reduced_logging)
{
    const ProbeConfig_t cfg = {
        .mode_name = reduced_logging ? "repl-quiet" : "repl",
        .reduced_logging = reduced_logging,
        .boot_mode = false,
        .capture_task = false,
    };
    return RunProbe(&cfg);
}

static void ProbeCaptureTask(void *arg)
{
    ProbeTaskRequest_t *request = (ProbeTaskRequest_t *)arg;
    request->ok = RunProbe(&request->cfg);
    xTaskNotifyGive(request->caller);
    vTaskDelete(NULL);
}

static bool RunCaptureTask(const char *mode_name, bool reduced_logging)
{
    ProbeTaskRequest_t request = {
        .cfg = {
            .mode_name = mode_name,
            .reduced_logging = reduced_logging,
            .boot_mode = false,
            .capture_task = true,
        },
        .caller = xTaskGetCurrentTaskHandle(),
        .ok = false,
    };

    BaseType_t created = xTaskCreatePinnedToCore(ProbeCaptureTask,
                                                 "sv_capture",
                                                 kProbeCaptureStackBytes,
                                                 &request,
                                                 kProbeCapturePriority,
                                                 NULL,
                                                 0);
    if (created != pdPASS) {
        printf("SvProbe: failed to create capture task\n");
        return false;
    }
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return request.ok;
}

void FusionSavestateTransportProbe_RunBootProbeOnce(void)
{
    printf("SvProbe: boot probe waiting %u ms for FPGA\n",
           (unsigned)kProbeBootDelayMs);
    vTaskDelay(pdMS_TO_TICKS(kProbeBootDelayMs));
    const ProbeConfig_t cfg = {
        .mode_name = "boot-final-only",
        .reduced_logging = true,
        .boot_mode = true,
        .capture_task = false,
    };
    (void)RunProbe(&cfg);
}

static int ProbeCommand(int argc, char **argv)
{
    const char *mode_name = "repl";
    bool reduced_logging = false;
    if (argc >= 2) {
        if (strcmp(argv[1], "menu") == 0) {
            mode_name = "menu";
            reduced_logging = true;
        } else if (strcmp(argv[1], "game") == 0) {
            mode_name = "game";
            reduced_logging = true;
        } else if (strcmp(argv[1], "quiet") == 0 || strcmp(argv[1], "reduced") == 0) {
            mode_name = "repl-quiet";
            reduced_logging = true;
        } else if (strcmp(argv[1], "repl") != 0) {
            printf("Usage: svprobe [menu|game|repl|quiet]\n");
            return 1;
        }
    }
    return RunCaptureTask(mode_name, reduced_logging) ? 0 : 1;
}

void FusionSavestateTransportProbe_RegisterCommands(void)
{
    const esp_console_cmd_t command = {
        .command = "svprobe",
        .help = "Phase 4.8a read-transport probe: svprobe [menu|game|repl|quiet]",
        .func = &ProbeCommand,
        .argtable = NULL,
    };
    esp_err_t err = esp_console_cmd_register(&command);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Registering '%s' failed: %s",
                 command.command, esp_err_to_name(err));
    }
}
