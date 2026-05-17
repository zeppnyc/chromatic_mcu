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
    /* Batch 1: ms to wait after BEGIN_LOAD ACK before issuing the first
       WRITE_STREAM_BEGIN.  Matches the BEGIN_SAVE 2 ms settle in spirit:
       gives session_pause time to propagate from gClk to hclk so the
       loader/WRAM port-A muxes are quiet before host writes start. */
    kStage2BeginLoadSettleMs = 3,
    /* Batch 1: abort probe parameters.  abort-read captures N DATA packets
       then injects END_SESSION instead of READ_STREAM_CONTINUE; abort-load
       sends N STATE_DATA packets then END_SESSION instead of WRITE_COMMIT.
       Numbers are deliberately small so the abort happens inside an
       active stream, not after it has naturally finished. */
    kStage2AbortReadPackets = 5,
    kStage2AbortLoadPackets = 2,
    kStage2AbortLoadProcessMs = 5,
    kStage2AbortDrainBudgetMs = 800,
    kStage2AckLoadLoopMaxCount = 10000,
    kStage2RxRawRingBytes = 2048,
    kStage2RxRawTailDefaultBytes = 128,
    kStage2RxRawTailMaxBytes = 512,
    kStage2R0Bytes = 65,
    kStage2R0InfoBytes = 8,
    kStage2R0CpuBytes = 42,
    kStage2R0TopBytes = 8,
    kStage2R0TimerBytes = 7,
    kStage2TotalBytes = kStage2R0Bytes + 32768 + 127,
};

typedef struct {
    const char *name;
    uint8_t region;
    uint16_t length;
} Stage2Region_t;

static const char *TAG = "Stage2Probe";

static const Stage2Region_t kStage2Regions[] = {
    { "R0",   0x00u, kStage2R0Bytes },
    { "WRAM", 0x01u, 32768u },
    { "HRAM", 0x02u, 127u   },
};

static const Stage2Region_t kStage2LoaderDiagRegion = {
    "LDIAG", 0x7Eu, 8u
};

static uint8_t s_stage2_region0[kStage2R0Bytes];
static uint8_t s_stage2_wram[32768];
static uint8_t s_stage2_hram[127];
static uint32_t s_stage2_capture_crc[3];
static uint32_t s_stage2_capture_packets[3];
static uint32_t s_stage2_capture_restore_delay_s = 0u;
static uint32_t s_stage2_ack_load_loop_count = 0u;
static uint32_t s_stage2_ack_after_capture_delay_s = 0u;
static bool s_stage2_capture_valid = false;

static uint8_t s_uart_cache[512];
static size_t s_uart_cache_pos = 0u;
static size_t s_uart_cache_len = 0u;

static uint8_t s_stage2_rx_raw_ring[kStage2RxRawRingBytes];
static uint32_t s_stage2_rx_raw_chunk_seq_ring[kStage2RxRawRingBytes];
static uint16_t s_stage2_rx_raw_chunk_off_ring[kStage2RxRawRingBytes];
static int64_t s_stage2_rx_raw_t_us_ring[kStage2RxRawRingBytes];
static uint32_t s_stage2_rx_raw_write = 0u;
static uint32_t s_stage2_rx_raw_total = 0u;
static uint32_t s_stage2_rx_raw_chunk_seq = 0u;

/* Batch 1: cleanup-failure latch.  Set whenever END_SESSION cleanup runs
   out of retries; refuses further non-clear commands so the host does not
   start a fresh long smoke on an unresponsive bridge.  Only cleared by a
   successful 'stage2probe clear' or by an MCU/board reset (statics
   re-initialise on boot). */
static bool s_stage2_fpga_stuck = false;

static void ResetUartCache(void)
{
    s_uart_cache_pos = 0u;
    s_uart_cache_len = 0u;
}

static void Stage2RawTapReset(void)
{
    s_stage2_rx_raw_write = 0u;
    s_stage2_rx_raw_total = 0u;
    s_stage2_rx_raw_chunk_seq = 0u;
    memset(s_stage2_rx_raw_ring, 0, sizeof(s_stage2_rx_raw_ring));
    memset(s_stage2_rx_raw_chunk_seq_ring, 0, sizeof(s_stage2_rx_raw_chunk_seq_ring));
    memset(s_stage2_rx_raw_chunk_off_ring, 0, sizeof(s_stage2_rx_raw_chunk_off_ring));
    memset(s_stage2_rx_raw_t_us_ring, 0, sizeof(s_stage2_rx_raw_t_us_ring));
}

static void Stage2RawTapRecord(const uint8_t *bytes, size_t length, int64_t t_us)
{
    if (bytes == NULL || length == 0u) {
        return;
    }
    const uint32_t seq = ++s_stage2_rx_raw_chunk_seq;
    for (size_t i = 0u; i < length; ++i) {
        const uint32_t pos = s_stage2_rx_raw_write;
        s_stage2_rx_raw_ring[pos] = bytes[i];
        s_stage2_rx_raw_chunk_seq_ring[pos] = seq;
        s_stage2_rx_raw_chunk_off_ring[pos] = (uint16_t)i;
        s_stage2_rx_raw_t_us_ring[pos] = t_us;
        s_stage2_rx_raw_write = (s_stage2_rx_raw_write + 1u)
                              % (uint32_t)kStage2RxRawRingBytes;
        if (s_stage2_rx_raw_total < (uint32_t)kStage2RxRawRingBytes) {
            ++s_stage2_rx_raw_total;
        }
    }
}

static void PrintBytes(const uint8_t *bytes, size_t length);

static void Stage2RawTapDumpTail(const char *context, size_t requested)
{
    size_t tail_len = requested;
    if (tail_len == 0u) {
        tail_len = kStage2RxRawTailDefaultBytes;
    }
    if (tail_len > kStage2RxRawTailMaxBytes) {
        tail_len = kStage2RxRawTailMaxBytes;
    }
    if (tail_len > s_stage2_rx_raw_total) {
        tail_len = s_stage2_rx_raw_total;
    }

    const uint32_t start = (s_stage2_rx_raw_write
                         + (uint32_t)kStage2RxRawRingBytes
                         - (uint32_t)tail_len)
                         % (uint32_t)kStage2RxRawRingBytes;
    printf("Stage2Probe: RX_RAW_TAIL context=%s total=%lu write=%lu "
           "tail_len=%u bytes=",
           context != NULL ? context : "",
           (unsigned long)s_stage2_rx_raw_total,
           (unsigned long)s_stage2_rx_raw_write,
           (unsigned)tail_len);
    for (size_t i = 0u; i < tail_len; ++i) {
        const uint32_t pos = (start + (uint32_t)i)
                           % (uint32_t)kStage2RxRawRingBytes;
        printf(" %02X", s_stage2_rx_raw_ring[pos]);
    }
    printf("\n");

    if (tail_len == 0u) {
        return;
    }

    size_t chunk_start_i = 0u;
    while (chunk_start_i < tail_len) {
        const uint32_t chunk_start_pos =
            (start + (uint32_t)chunk_start_i)
            % (uint32_t)kStage2RxRawRingBytes;
        const uint32_t seq = s_stage2_rx_raw_chunk_seq_ring[chunk_start_pos];
        const int64_t t_us = s_stage2_rx_raw_t_us_ring[chunk_start_pos];
        size_t chunk_len = 1u;
        while ((chunk_start_i + chunk_len) < tail_len) {
            const uint32_t prev_pos =
                (start + (uint32_t)chunk_start_i + (uint32_t)chunk_len - 1u)
                % (uint32_t)kStage2RxRawRingBytes;
            const uint32_t next_pos =
                (start + (uint32_t)chunk_start_i + (uint32_t)chunk_len)
                % (uint32_t)kStage2RxRawRingBytes;
            if (s_stage2_rx_raw_chunk_seq_ring[next_pos] != seq ||
                s_stage2_rx_raw_chunk_off_ring[next_pos] !=
                    (uint16_t)(s_stage2_rx_raw_chunk_off_ring[prev_pos] + 1u)) {
                break;
            }
            ++chunk_len;
        }

        printf("Stage2Probe: RX_RAW_CHUNK_TAIL context=%s seq=%lu "
               "t_us=%lld start_off=%u len=%u bytes=",
               context != NULL ? context : "",
               (unsigned long)seq,
               (long long)t_us,
               (unsigned)s_stage2_rx_raw_chunk_off_ring[chunk_start_pos],
               (unsigned)chunk_len);
        for (size_t i = 0u; i < chunk_len; ++i) {
            const uint32_t pos =
                (start + (uint32_t)chunk_start_i + (uint32_t)i)
                % (uint32_t)kStage2RxRawRingBytes;
            printf(" %02X", s_stage2_rx_raw_ring[pos]);
        }
        printf("\n");
        chunk_start_i += chunk_len;
    }
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
            Stage2RawTapRecord(s_uart_cache,
                               (size_t)got,
                               esp_timer_get_time());
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
    const uint8_t received_crc = reconstructed[3u + payload_len];
    uint8_t no_addr_frame[FUSION_V2_MAX_FRAME] = {0};
    uint8_t full_frame[FUSION_V2_MAX_FRAME] = {0};
    const size_t no_addr_frame_len = 2u + (size_t)payload_len + 1u;
    no_addr_frame[0] = raw[0];
    no_addr_frame[1] = raw[1];
    memcpy(&no_addr_frame[2], &reconstructed[3], payload_len);
    no_addr_frame[2u + payload_len] = received_crc;
    memcpy(full_frame, reconstructed, frame_len);
    const uint8_t no_addr_expected_crc =
        ExpectedFrameCrc(no_addr_frame, no_addr_frame_len - 1u);
    const uint8_t full_expected_crc =
        ExpectedFrameCrc(full_frame, frame_len - 1u);
    const bool no_addr_crc_match = (received_crc == no_addr_expected_crc);
    const bool full_crc_match = (received_crc == full_expected_crc);
    const char *classification = "neither_crc_match";
    if (no_addr_crc_match) {
        classification = "noaddr_crc_match";
    } else if (full_crc_match) {
        classification = "full_crc_match";
    }

    printf("Stage2Probe: ACK_RAW_DIAG context=%s raw_prefix=%02X %02X %02X "
           "payload_or_ack_for=%02X received_crc=%02X noaddr_frame=",
           context,
           raw[0],
           raw[1],
           raw[2],
           reconstructed[4],
           received_crc);
    PrintBytes(no_addr_frame, no_addr_frame_len);
    printf(" full_frame=");
    PrintBytes(full_frame, frame_len);
    printf(" expected_crc_noaddr=%02X expected_crc_full=%02X "
           "ACK_RAW_CLASS=%s\n",
           no_addr_expected_crc,
           full_expected_crc,
           classification);
    Stage2RawTapDumpTail(context, kStage2RxRawTailDefaultBytes);

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
            Stage2RawTapDumpTail(context, kStage2RxRawTailDefaultBytes);
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
            Stage2RawTapDumpTail(context, kStage2RxRawTailDefaultBytes);
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
            Stage2RawTapDumpTail(context, kStage2RxRawTailDefaultBytes);
            continue;
        }

        const size_t frame_len = 3u + payload_len + 1u;
        for (size_t i = 3u; i < frame_len; ++i) {
            if (!ReadByteWithDeadline(&raw[i], deadline)) {
                printf("Stage2Probe: RX timeout waiting for payload/crc during %s\n",
                       context);
                Stage2RawTapDumpTail(context, kStage2RxRawTailDefaultBytes);
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
    Stage2RawTapDumpTail(context, kStage2RxRawTailDefaultBytes);
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

static bool WriteStage2RegionImpl(const Stage2Region_t *region,
                                  const uint8_t *data,
                                  uint32_t *packets_out,
                                  uint8_t max_chunk)
{
    if (region == NULL || data == NULL || max_chunk == 0u ||
        max_chunk > FUSION_STATE_DATA_MAX_DATA) {
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

    printf("Stage2Probe: WRITE REGION %s id=0x%02X length=%u max_chunk=%u\n",
           region->name,
           (unsigned)region->region,
           (unsigned)region->length,
           (unsigned)max_chunk);
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
        const uint8_t chunk = (remaining > (uint16_t)max_chunk)
            ? max_chunk
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

    printf("Stage2Probe: WRITE %s PASS packets=%u bytes=%u max_chunk=%u\n",
           region->name,
           (unsigned)seq,
           (unsigned)sent,
           (unsigned)max_chunk);
    if (packets_out != NULL) {
        *packets_out = seq;
    }
    return true;
}

static bool WriteStage2Region(const Stage2Region_t *region,
                              const uint8_t *data,
                              uint32_t *packets_out)
{
    return WriteStage2RegionImpl(region, data, packets_out,
                                 FUSION_STATE_DATA_MAX_DATA);
}

/* Bytewise variant: forces exactly one state byte per STATE_DATA packet for
   every region.  Purpose is the host-only foundation path documented in the
   2026-05-17 byte-pulse root-cause artifact -- gives the accepted bridge
   one fresh state_byte_valid low/high transition per state byte, without
   touching FPGA RTL or PnR.  Slower than the default 8-byte packetization;
   restore latency scales with byte count. */
static bool WriteStage2RegionBytewise(const Stage2Region_t *region,
                                      const uint8_t *data,
                                      uint32_t *packets_out)
{
    return WriteStage2RegionImpl(region, data, packets_out, 1u);
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

/* R0 v2 INFO flags byte (offset 5) bit 0 = HOLD_AFTER_APPLY. */
#define kStage2R0FlagHoldAfterApply 0x01u

typedef struct {
    uint32_t t_ms;
    uint8_t data[8];
    bool valid;
} Stage2TraceDiagSample_t;

static uint16_t Stage2R0Pc(const uint8_t data[kStage2R0Bytes])
{
    return (uint16_t)data[18] | ((uint16_t)data[19] << 8);
}

static uint16_t Stage2DiagCpuBufPc(const uint8_t data[8])
{
    return (uint16_t)data[2] | ((uint16_t)data[3] << 8);
}

static uint16_t Stage2DiagLoaderPc(const uint8_t data[8])
{
    return (uint16_t)data[4] | ((uint16_t)data[5] << 8);
}

static const char *Stage2LoaderStateName(uint8_t state)
{
    switch (state & 0x0Fu) {
    case 0x00u: return "LD_IDLE";
    case 0x01u: return "LD_WAIT_PAUSE";
    case 0x02u: return "LD_WAIT_BOUNDARY";
    case 0x03u: return "LD_WALK_HEADER";
    case 0x04u: return "LD_STREAM_WRAM";
    case 0x05u: return "LD_STREAM_HRAM";
    case 0x06u: return "LD_WAIT_APPLY";
    case 0x07u: return "LD_FORCE_IDLE";
    case 0x08u: return "LD_DONE";
    case 0x09u: return "LD_HOLD_APPLY";
    default: return "LD_UNKNOWN";
    }
}

static bool Stage2StateIsStream(uint8_t state)
{
    const uint8_t s = (uint8_t)(state & 0x0Fu);
    return s == 0x04u || s == 0x05u;
}

static void PrintTraceDiag(uint32_t t_ms, const uint8_t data[8])
{
    /* Accepted bitstream's 0x7E region only populates bytes 0..5; bytes 6/7
       are reserved/undefined.  Print them as NA so a degraded trace cannot
       be misread as having info_flags / progress evidence. */
    printf("TRACE_DIAG t_ms=%lu flags=0x%02X state=%02X/%s "
           "cpu_buf_pc=%04X loader_pc=%04X info_flags=NA progress=NA\n",
           (unsigned long)t_ms,
           data[0],
           data[1],
           Stage2LoaderStateName(data[1]),
           Stage2DiagCpuBufPc(data),
           Stage2DiagLoaderPc(data));
}

/* Degraded byte0..5 classifier.  The accepted 0x7E region only exposes
   bytes 0..5 reliably, so this classifier intentionally does NOT consult
   data[6] / data[7].  Categories follow the 2026-05-16 LOAD contract
   trace spec:

     CONTRACT_MISSING_REGION
     COMMIT_TOO_EARLY
     STREAM_STUCK
     R0_PC_NOT_LATCHED_OR_TOO_EARLY
     HOLD_NOT_REACHED
     INCONCLUSIVE

   BASELINE_FAIL is emitted by the outer command when capture itself fails
   (FPGA already non-responsive before restore starts) and is not produced
   here. */
static const char *ClassifyHoldContractTrace(
    const Stage2TraceDiagSample_t *samples,
    size_t sample_count,
    bool host_r0_ok,
    bool host_sent_wram_complete,
    bool host_sent_hram_complete,
    uint16_t expected_pc)
{
    bool any_valid = false;
    bool later_hold_or_done = false;
    bool all_diag_pc_zero = true;
    bool any_cpu_pc_correct = false;
    bool any_hold_apply = false;
    bool any_wait_force_or_done = false;
    bool stream_state_unchanged = true;
    bool stream_cpu_pc_unchanged = true;
    bool stream_loader_pc_unchanged = true;
    bool all_in_stream = true;
    uint8_t first_state = 0xFFu;
    uint8_t last_state = 0xFFu;
    uint16_t first_cpu_pc = 0u;
    uint16_t first_loader_pc = 0u;

    for (size_t i = 0u; i < sample_count; ++i) {
        if (!samples[i].valid) {
            continue;
        }

        const uint8_t state = (uint8_t)(samples[i].data[1] & 0x0Fu);
        const uint16_t cpu_pc = Stage2DiagCpuBufPc(samples[i].data);
        const uint16_t loader_pc = Stage2DiagLoaderPc(samples[i].data);

        if (!any_valid) {
            first_state = state;
            first_cpu_pc = cpu_pc;
            first_loader_pc = loader_pc;
        } else {
            if (state != first_state) {
                stream_state_unchanged = false;
            }
            if (cpu_pc != first_cpu_pc) {
                stream_cpu_pc_unchanged = false;
            }
            if (loader_pc != first_loader_pc) {
                stream_loader_pc_unchanged = false;
            }
        }
        any_valid = true;
        last_state = state;

        if (!Stage2StateIsStream(state)) {
            all_in_stream = false;
        }
        if (i > 0u && (state == 0x09u || state == 0x08u)) {
            later_hold_or_done = true;
        }
        if (cpu_pc != 0u || loader_pc != 0u) {
            all_diag_pc_zero = false;
        }
        if (cpu_pc == expected_pc) {
            any_cpu_pc_correct = true;
        }
        if (state == 0x09u) {
            any_hold_apply = true;
        }
        if (state == 0x06u || state == 0x07u || state == 0x08u) {
            any_wait_force_or_done = true;
        }
    }

    (void)host_r0_ok; /* reserved for future use */

    if (!any_valid) {
        return "INCONCLUSIVE";
    }
    /* CONTRACT_MISSING_REGION: host did not actually send the region whose
       stream state we are now stuck in. */
    if ((!host_sent_wram_complete && last_state == 0x04u) ||
        (!host_sent_hram_complete && last_state == 0x05u)) {
        return "CONTRACT_MISSING_REGION";
    }
    /* COMMIT_TOO_EARLY: first sample was in stream, but later samples reached
       hold/done -- loader was still chewing when commit returned. */
    if (Stage2StateIsStream(first_state) && later_hold_or_done) {
        return "COMMIT_TOO_EARLY";
    }
    /* STREAM_STUCK: every sample in stream and no observed change in
       state/cpu_buf_pc/loader_pc across the window. */
    if (host_sent_wram_complete && host_sent_hram_complete &&
        all_in_stream &&
        stream_state_unchanged &&
        stream_cpu_pc_unchanged &&
        stream_loader_pc_unchanged) {
        return "STREAM_STUCK";
    }
    /* R0_PC_NOT_LATCHED_OR_TOO_EARLY: host R0 carries a non-zero expected PC,
       every observed cpu_buf_pc / loader_pc is 0000, and we never reached
       any apply / hold / done state.  Either R0 PC bytes are not being
       latched into the loader, or we sampled too early. */
    if (expected_pc != 0u && all_diag_pc_zero &&
        !any_wait_force_or_done && !any_hold_apply) {
        return "R0_PC_NOT_LATCHED_OR_TOO_EARLY";
    }
    /* HOLD_NOT_REACHED: loader advanced past stream into wait/force/done but
       never entered LD_HOLD_APPLY. */
    if (host_sent_wram_complete && host_sent_hram_complete &&
        any_cpu_pc_correct &&
        any_wait_force_or_done && !any_hold_apply) {
        return "HOLD_NOT_REACHED";
    }
    return "INCONCLUSIVE";
}

static bool ValidateRegion0(const uint8_t data[kStage2R0Bytes])
{
    if (memcmp(data, "FUSS", 4u) != 0) {
        printf("Stage2Probe: R0 magic mismatch got=%02X %02X %02X %02X\n",
               data[0],
               data[1],
               data[2],
               data[3]);
        return false;
    }
    if (data[4] != 0x02u || data[5] != 0x00u ||
        data[6] != 0x00u || data[7] != 0x00u) {
        printf("Stage2Probe: R0 info mismatch ver=%02X flags=%02X rsv=%02X%02X\n",
               data[4],
               data[5],
               data[6],
               data[7]);
        return false;
    }

    const uint16_t gbse = (uint16_t)data[8] | ((uint16_t)data[9] << 8);
    const uint16_t pc = (uint16_t)data[18] | ((uint16_t)data[19] << 8);
    const uint16_t sp = (uint16_t)data[34] | ((uint16_t)data[35] << 8);
    printf("Stage2Probe: R0 RAW bytes=");
    PrintBytes(data, kStage2R0Bytes);
    printf("\n");
    printf("Stage2Probe: R0 INFO magic=FUSS version=2 PC=%04X SP=%04X "
           "AF=%02X%02X BC=%02X%02X DE=%02X%02X HL=%02X%02X\n",
           pc,
           sp,
           data[17],
           data[13],
           data[14],
           data[10],
           data[15],
           data[11],
           data[16],
           data[12]);
    printf("Stage2Probe: R0 CPU_FULL gbse=%03X "
           "cpuregs_crc=%08lX t80_1_crc=%08lX t80_2_crc=%08lX "
           "t80_3_crc=%08lX t80_4_crc=%08lX\n",
           (unsigned)(gbse & 0x0FFFu),
           (unsigned long)FusionSavestate_Crc32(&data[10], 8u),
           (unsigned long)FusionSavestate_Crc32(&data[18], 8u),
           (unsigned long)FusionSavestate_Crc32(&data[26], 8u),
           (unsigned long)FusionSavestate_Crc32(&data[34], 8u),
           (unsigned long)FusionSavestate_Crc32(&data[42], 8u));
    return true;
}

static void Stage2ProbeMarkStuck(const char *context)
{
    s_stage2_fpga_stuck = true;
    printf("Stage2Probe: FPGA_STUCK_NEEDS_RESET context=%s "
           "-- END_SESSION cleanup failed; reset the board or run "
           "'stage2probe clear' (must PASS) before further probes\n",
           context != NULL ? context : "");
}

static bool Stage2ProbeRefuseIfStuck(const char *context)
{
    if (s_stage2_fpga_stuck) {
        printf("Stage2Probe: FPGA_STUCK_NEEDS_RESET context=%s "
               "-- previous cleanup failed; run 'stage2probe clear' "
               "(must PASS) or reset the board first\n",
               context != NULL ? context : "");
        return true;
    }
    return false;
}

static bool EndSessionCleanup(void)
{
    printf("Stage2Probe: sending END_SESSION cleanup\n");
    DrainRxForMs(kStage2FinalEndMs);
    ResetUartCache();
    uart_flush_input(UART_NUM_1);
    if (SendEndSessionClear("END_SESSION cleanup")) {
        return true;
    }
    Stage2ProbeMarkStuck("END_SESSION cleanup");
    return false;
}

static bool RunWithUartOwner(bool (*fn)(void))
{
    bool ok = false;

    /* Batch 1 v2: PwrMgr_IdleTimerSuspend() is sticky for the whole
       stage2probe session.  We never call PwrMgr_IdleTimerResume() in
       this file because the idle timer's expiry (kIdleTime_ms = 100 ms,
       pwrmgr.c) triggers PwrMgr_TriggerLightSleep -> esp_light_sleep_start
       and the only registered wake source is PIN_NUM_UART_FROM_FPGA.
       The console UART is NOT a wake source, so any console command sent
       after a previous stage2probe command's PwrMgr_IdleTimerResume()
       would have its bytes sit unread in the UART0 RX FIFO and the runner
       would time out with no echo.  Observed on 2026-05-15 12:11 and
       12:53 hardware runs: stage2probe clear PASS, second clear TIMEOUT,
       MCU console unresponsive.  Cost: light sleep is disabled until the
       next board reset.  This is acceptable for hardware validation
       builds and gets restored on reboot. */
    PwrMgr_IdleTimerSuspend();
    PauseFpgaTraffic();
    FPGA_Rx_ResetParser();
    ResetUartCache();
    Stage2RawTapReset();
    uart_flush_input(UART_NUM_1);

    if (!FPGA_UartOwnerAcquire(pdMS_TO_TICKS(kStage2OwnerAcquireMs))) {
        printf("Stage2Probe: UART owner unavailable\n");
        ResumeFpgaTraffic();
        /* idle timer intentionally left suspended -- see top comment */
        return false;
    }

    ok = fn();

    /* UART owner can always be released: the next 'stage2probe clear'
       still needs to acquire it to issue an END_SESSION recovery. */
    FPGA_UartOwnerRelease();

    /* Batch 1: gate FPGA-traffic resume on the stuck latch.  If
       EndSessionCleanup() failed inside fn() and Stage2ProbeMarkStuck()
       set s_stage2_fpga_stuck, the FPGA bridge is still
       session_pause-asserted and unresponsive.  Resuming the routine
       fpga_tx/fpga_rx tasks in that state would let unrelated
       MCU<->FPGA chatter run against a paused bridge and make the next
       failure harder to classify.  Keep them paused until
       'stage2probe clear' returns PASS, or until the board is reset.
       Idle timer is unconditionally kept suspended -- see top comment. */
    if (s_stage2_fpga_stuck) {
        printf("Stage2Probe: keeping FPGA traffic paused "
               "(FPGA_STUCK_NEEDS_RESET); run 'stage2probe clear' until "
               "PASS or reset the board to recover\n");
    } else {
        ResumeFpgaTraffic();
    }
    return ok;
}

static bool RunSaveReadbackInner(void)
{
    bool ok = false;
    uint32_t crc[3] = {0};
    uint32_t packets[3] = {0};
    uint8_t region0[kStage2R0Bytes] = {0};

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
           kStage2TotalBytes,
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
           kStage2TotalBytes,
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
    /* Batch 1: let session_pause settle on hclk before the first
       WRITE_STREAM_BEGIN so the bridge/loader port-A muxes are quiet. */
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

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
           kStage2TotalBytes,
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

/* Bytewise full-subset restore: identical to RunRestoreCapturedInner except
   every STATE_DATA packet carries exactly one state byte for all three
   regions (R0 v2 + WRAM + HRAM).  This is the host-only foundation path
   from the 2026-05-17 byte-pulse root-cause artifact applied to the full
   required subset, with no FPGA RTL change.  WRITE_COMMIT + END_SESSION
   semantics are unchanged from the default restore path. */
static bool RunRestoreCapturedBytewiseFullInner(void)
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
                                     "BEGIN_LOAD_STAGE2_BYTEWISE_FULL")) {
        goto done;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    for (size_t i = 0u; i < sizeof(kStage2Regions) / sizeof(kStage2Regions[0]); ++i) {
        size_t copy_len = 0u;
        const uint8_t *data =
            CaptureBufferForRegion(kStage2Regions[i].region, &copy_len);
        if (data == NULL || copy_len < kStage2Regions[i].length) {
            printf("Stage2Probe: missing capture buffer for region 0x%02X\n",
                   (unsigned)kStage2Regions[i].region);
            goto done;
        }
        if (!WriteStage2RegionBytewise(&kStage2Regions[i], data, &packets[i])) {
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
           "commit=yes mode=bytewise-full\n",
           kStage2TotalBytes,
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

static bool RunCaptureRestoreBytewiseFullInner(void)
{
    if (!RunCaptureInner()) {
        return false;
    }

    printf("Stage2Probe: READY_FOR_GAME_CHANGE delay_s=%lu mode=bytewise-full\n",
           (unsigned long)s_stage2_capture_restore_delay_s);
    for (uint32_t remaining = s_stage2_capture_restore_delay_s;
         remaining > 0u;
         --remaining) {
        if ((remaining == s_stage2_capture_restore_delay_s) ||
            (remaining <= 5u) ||
            ((remaining % 10u) == 0u)) {
            printf("Stage2Probe: CHANGE_WINDOW remaining_s=%lu mode=bytewise-full\n",
                   (unsigned long)remaining);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("Stage2Probe: CHANGE_WINDOW done; starting restore mode=bytewise-full\n");

    return RunRestoreCapturedBytewiseFullInner();
}

static bool RunRestoreCapturedHoldContractTraceInner(void)
{
    /* Spec: 0/5/20/100/500/1000 ms required, 2000 ms optional but bounded.
       Total bounded wait is ~2 s after WRITE_COMMIT_RETURN. */
    static const uint32_t kTraceMs[] = {
        0u, 5u, 20u, 100u, 500u, 1000u, 2000u
    };
    bool ok = false;
    bool classification_printed = false;
    uint32_t sent_bytes[3] = {0};
    Stage2TraceDiagSample_t samples[
        sizeof(kTraceMs) / sizeof(kTraceMs[0])
    ];
    uint8_t r0_with_hold[kStage2R0Bytes];
    uint32_t diag_crc = 0u;
    uint32_t diag_packets = 0u;
    int64_t commit_send_us = 0;
    int64_t commit_return_us = 0;

    memset(samples, 0, sizeof(samples));
    memset(r0_with_hold, 0, sizeof(r0_with_hold));

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
                                     "BEGIN_LOAD_STAGE2_HOLD_CONTRACT_TRACE")) {
        goto done;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    {
        size_t copy_len = 0u;
        const uint8_t *src =
            CaptureBufferForRegion(kStage2Regions[0].region, &copy_len);
        if (src == NULL || copy_len < kStage2R0Bytes) {
            printf("Stage2Probe: missing capture buffer for region 0x00\n");
            goto done;
        }
        memcpy(r0_with_hold, src, kStage2R0Bytes);
        r0_with_hold[5] = (uint8_t)(r0_with_hold[5] |
                                    kStage2R0FlagHoldAfterApply);
    }

    if (!WriteStage2Region(&kStage2Regions[0], r0_with_hold, NULL)) {
        goto done;
    }
    sent_bytes[0] = kStage2Regions[0].length;
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));

    for (size_t i = 1u; i < sizeof(kStage2Regions) / sizeof(kStage2Regions[0]); ++i) {
        size_t copy_len = 0u;
        const uint8_t *data =
            CaptureBufferForRegion(kStage2Regions[i].region, &copy_len);
        if (data == NULL || copy_len < kStage2Regions[i].length) {
            printf("Stage2Probe: missing capture buffer for region 0x%02X\n",
                   (unsigned)kStage2Regions[i].region);
            goto done;
        }
        if (!WriteStage2Region(&kStage2Regions[i], data, NULL)) {
            goto done;
        }
        sent_bytes[i] = kStage2Regions[i].length;
        vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    }

    printf("HOST_CONTRACT captured_pc=%04X r0_length=%u r0_version=%u "
           "r0_flags=0x%02X hold_bit=%u t80_1_pc_bytes=%02X %02X "
           "expected_loader_pc=%04X send_wram=yes wram_length=%u "
           "wram_sent_bytes=%lu send_hram=yes hram_length=%u "
           "hram_sent_bytes=%lu\n",
           Stage2R0Pc(s_stage2_region0),
           (unsigned)kStage2R0Bytes,
           (unsigned)r0_with_hold[4],
           r0_with_hold[5],
           (unsigned)((r0_with_hold[5] & kStage2R0FlagHoldAfterApply) ? 1u : 0u),
           r0_with_hold[18],
           r0_with_hold[19],
           Stage2R0Pc(r0_with_hold),
           (unsigned)kStage2Regions[1].length,
           (unsigned long)sent_bytes[1],
           (unsigned)kStage2Regions[2].length,
           (unsigned long)sent_bytes[2]);

    commit_send_us = esp_timer_get_time();
    if (!WriteCommit()) {
        goto done;
    }
    commit_return_us = esp_timer_get_time();
    /* WriteCommit() returns when the FPGA acks WRITE_COMMIT on the transport
       channel.  That ack confirms commit packet receipt; it does NOT mean
       loader_apply finished or that the CPU has been released from pause.
       The 0x7E trace below is the only signal for loader/apply progress. */
    printf("WRITE_COMMIT_RETURN t_us=%lld elapsed_ms=%lld ack=ACK_DONE "
           "scope=transport_commit_receipt loader_apply_done=UNKNOWN\n",
           (long long)commit_return_us,
           (long long)((commit_return_us - commit_send_us) / 1000));

    if (!SendEndSessionClear("END_SESSION load-close trace")) {
        Stage2ProbeMarkStuck("END_SESSION load-close trace");
        goto done;
    }

    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginSave,
                                     "BEGIN_SAVE_CONTRACT_TRACE")) {
        goto done;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    for (size_t i = 0u; i < sizeof(kTraceMs) / sizeof(kTraceMs[0]); ++i) {
        const int64_t target_us = commit_return_us +
                                  ((int64_t)kTraceMs[i] * 1000);
        const int64_t now_us = esp_timer_get_time();
        if (now_us < target_us) {
            const uint32_t wait_ms =
                (uint32_t)((target_us - now_us + 999) / 1000);
            if (wait_ms > 0u) {
                vTaskDelay(pdMS_TO_TICKS(wait_ms));
            }
        }

        samples[i].t_ms = kTraceMs[i];
        memset(samples[i].data, 0, sizeof(samples[i].data));
        if (!ReadStage2Region(&kStage2LoaderDiagRegion,
                              &diag_crc,
                              &diag_packets,
                              samples[i].data,
                              sizeof(samples[i].data))) {
            printf("Stage2Probe: CONTRACT_TRACE loader diagnostic read failed "
                   "t_ms=%lu\n",
                   (unsigned long)kTraceMs[i]);
            goto done;
        }
        samples[i].valid = true;
        PrintTraceDiag(kTraceMs[i], samples[i].data);
    }

    {
        const bool host_r0_ok =
            memcmp(s_stage2_region0, "FUSS", 4u) == 0 &&
            s_stage2_region0[4] == 0x02u;
        const char *classification = ClassifyHoldContractTrace(
            samples,
            sizeof(samples) / sizeof(samples[0]),
            host_r0_ok,
            sent_bytes[1] == kStage2Regions[1].length,
            sent_bytes[2] == kStage2Regions[2].length,
            Stage2R0Pc(r0_with_hold));
        printf("FINAL_CLASSIFICATION %s\n", classification);
        classification_printed = true;
    }

    ok = true;

done:
    if (!classification_printed) {
        printf("FINAL_CLASSIFICATION INCONCLUSIVE\n");
    }
    if (!EndSessionCleanup()) {
        printf("Stage2Probe: CONTRACT_TRACE final cleanup failed; treating command as FAIL\n");
        ok = false;
    }
    return ok;
}

static bool RunCaptureRestoreHoldContractTraceInner(void)
{
    if (!RunCaptureInner()) {
        /* Capture itself failed: FPGA was non-responsive before the restore
           ever started, which is also how an already-broken baseline looks
           from the MCU side.  Emit BASELINE_FAIL so the trace classifier
           does not silently fall through to INCONCLUSIVE. */
        printf("FINAL_CLASSIFICATION BASELINE_FAIL\n");
        return false;
    }

    printf("Stage2Probe: READY_FOR_GAME_CHANGE delay_s=%lu mode=hold-contract-trace\n",
           (unsigned long)s_stage2_capture_restore_delay_s);
    for (uint32_t remaining = s_stage2_capture_restore_delay_s;
         remaining > 0u;
         --remaining) {
        if ((remaining == s_stage2_capture_restore_delay_s) ||
            (remaining <= 5u) ||
            ((remaining % 10u) == 0u)) {
            printf("Stage2Probe: CHANGE_WINDOW remaining_s=%lu mode=hold-contract-trace\n",
                   (unsigned long)remaining);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("Stage2Probe: CHANGE_WINDOW done; starting restore mode=hold-contract-trace\n");

    return RunRestoreCapturedHoldContractTraceInner();
}

/* ============================================================================
 *  LOAD progress ladder (2026-05-16)
 *
 *  Single-pass diagnostic that asks: when the host delivers N bytes through
 *  BEGIN_LOAD + WRITE_STREAM, how many bytes does the FPGA loader actually
 *  count?  Each case emits a single LADDER_CASE line that pins
 *  state/byte_idx/wram_idx/hram_idx/flags to whatever the loader exposes
 *  through the loader-local 0x7E byte-serial diagnostic (bytes 0..5).  The
 *  command is non-product; it does not claim PASS for restored gameplay.
 *
 *  The loader FSM only resets its byte_idx / wram_idx / hram_idx counters
 *  when it is in LD_IDLE on the next session_pause_rising.  Partial cases
 *  intentionally leave the loader mid-stream, so between cases we drive the
 *  loader back to LD_IDLE through a full LOAD+COMMIT cycle made of zero
 *  bytes (R0 zero bytes have no FUSS magic -> info_nonzero==0 ->
 *  loader_apply stays low through LD_FORCE_IDLE, so the architectural CPU
 *  FFs do NOT receive the loader's packed reset-init words; only the loader
 *  FSM walks LD_WAIT_APPLY -> LD_FORCE_IDLE -> LD_DONE -> LD_IDLE).
 * ========================================================================== */

enum {
    kLadderResetWramBytes = 32768u,
    kLadderResetHramBytes = 127u,
    /* Polling schedule for COMMIT_HOLD (ms from WRITE_COMMIT return). */
    kLadderHoldPolls = 7u,
    kLadderHoldClampMaxMs = 5000u,
};

typedef enum {
    kLadderVerdict_PASS = 0,
    kLadderVerdict_FAIL,
    kLadderVerdict_NOT_RUN
} LadderVerdict_t;

typedef enum {
    kLadderBranch_NONE = 0,
    kLadderBranch_R0_HEADER_PATH,
    kLadderBranch_WRAM_STREAM_BYTE_PUMP_OR_LOADER_COUNT,
    kLadderBranch_WRAM_FINAL_BOUNDARY,
    kLadderBranch_HRAM_STREAM_BYTE_PUMP_OR_LOADER_COUNT,
    kLadderBranch_HRAM_FINAL_BOUNDARY,
    kLadderBranch_COMMIT_APPLY_PULSE,
    kLadderBranch_INCONCLUSIVE,
    kLadderBranch_BLOCKED_BY_BUILD_OR_PNR,
    kLadderBranch_BLOCKED_BY_HARDWARE,
} LadderBranch_t;

typedef struct {
    const char *name;
    /* Per-case byte counts the host SHOULD send within this case's
       BEGIN_LOAD session.  r0_len is always 65 for cases that drive a
       fresh LOAD; commit_with_hold means write R0 with hold_bit set
       at byte 5 bit 0 and trail with WRITE_COMMIT. */
    uint16_t wram_len;
    uint16_t hram_len;
    bool commit_with_hold;
    /* Expected loader state token + numeric value. */
    const char *expect_state_name;
    uint8_t expect_state;
    uint8_t expect_byte_idx;
    uint16_t expect_wram_idx;
    uint8_t expect_hram_idx;
} LadderCase_t;

typedef struct {
    LadderVerdict_t verdict;
    const char *reason;
    uint8_t state;
    uint8_t byte_idx;
    uint16_t wram_idx;
    uint8_t hram_idx;
    uint8_t flags;
    bool any_apply_or_hold; /* COMMIT_HOLD only */
} LadderResult_t;

static const LadderCase_t kLadderCases[] = {
    /* name           wram     hram   hold   expect_state                                  state byte_idx wram_idx        hram_idx */
    { "R0_ONLY",      0u,      0u,    false, "LD_STREAM_WRAM",                              0x04u, 64u,    0u,             0u   },
    { "WRAM_1",       1u,      0u,    false, "LD_STREAM_WRAM",                              0x04u, 64u,    1u,             0u   },
    { "WRAM_8",       8u,      0u,    false, "LD_STREAM_WRAM",                              0x04u, 64u,    8u,             0u   },
    { "WRAM_4096",    4096u,   0u,    false, "LD_STREAM_WRAM",                              0x04u, 64u,    4096u,          0u   },
    { "WRAM_32760",   32760u,  0u,    false, "LD_STREAM_WRAM",                              0x04u, 64u,    32760u,         0u   },
    { "WRAM_32767",   32767u,  0u,    false, "LD_STREAM_WRAM",                              0x04u, 64u,    32767u,         0u   },
    { "WRAM_32768",   32768u,  0u,    false, "LD_STREAM_HRAM",                              0x05u, 64u,    32767u,         0u   },
    { "HRAM_1",       32768u,  1u,    false, "LD_STREAM_HRAM",                              0x05u, 64u,    32767u,         1u   },
    { "HRAM_127",     32768u,  127u,  false, "LD_WAIT_APPLY",                               0x06u, 64u,    32767u,         126u },
    { "COMMIT_HOLD",  32768u,  127u,  true,  "LD_FORCE_IDLE_OR_LD_HOLD_APPLY_OR_TRANSITION", 0x09u, 64u,   32767u,         126u },
};
enum { kLadderCaseCount = (uint8_t)(sizeof(kLadderCases) / sizeof(kLadderCases[0])) };

static const uint8_t kLadderZeroChunk[FUSION_STATE_DATA_MAX_DATA] = {0};

static bool LadderSendStreamWireOrder(uint8_t region_id,
                                      uint16_t length,
                                      const uint8_t *data_or_null,
                                      const char *label_begin,
                                      const char *label_data,
                                      bool reverse_wire_chunk);

static bool LadderSendStream(uint8_t region_id,
                             uint16_t length,
                             const uint8_t *data_or_null,
                             const char *label_begin,
                             const char *label_data)
{
    return LadderSendStreamWireOrder(region_id,
                                     length,
                                     data_or_null,
                                     label_begin,
                                     label_data,
                                     false);
}

static bool LadderSendStreamWireOrder(uint8_t region_id,
                                      uint16_t length,
                                      const uint8_t *data_or_null,
                                      const char *label_begin,
                                      const char *label_data,
                                      bool reverse_wire_chunk)
{
    if (length == 0u) {
        return true;
    }
    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildWriteStreamBegin(region_id, 0u, length, &begin)) {
        printf("Stage2Probe: build %s failed\n", label_begin);
        return false;
    }
    if (!SendFrame(label_begin, &begin, false)) {
        return false;
    }
    if (!ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                   (uint8_t)kFusionOp_WriteStreamBegin,
                   label_begin)) {
        return false;
    }
    uint16_t sent = 0u;
    uint16_t seq = 0u;
    while (sent < length) {
        const uint16_t remaining = (uint16_t)(length - sent);
        const uint8_t chunk =
            (remaining > FUSION_STATE_DATA_MAX_DATA)
                ? FUSION_STATE_DATA_MAX_DATA
                : (uint8_t)remaining;
        FusionV2Frame_t data_frame;
        const uint8_t *src =
            (data_or_null != NULL) ? &data_or_null[sent] : kLadderZeroChunk;
        uint8_t reversed[FUSION_STATE_DATA_MAX_DATA];
        if (reverse_wire_chunk) {
            for (uint8_t i = 0u; i < chunk; ++i) {
                reversed[i] = src[(uint8_t)(chunk - 1u - i)];
            }
            src = reversed;
        }
        if (!FusionSavestate_BuildStateData(seq, src, chunk, &data_frame)) {
            printf("Stage2Probe: build %s seq=%u failed\n",
                   label_data, (unsigned)seq);
            return false;
        }
        if (!SendFrame(label_data, &data_frame, true)) {
            return false;
        }
        sent = (uint16_t)(sent + chunk);
        seq = (uint16_t)(seq + 1u);
    }
    return ExpectCtl((uint8_t)kFusionOp_AckDone,
                     (uint8_t)kFusionOp_WriteStreamBegin,
                     label_begin);
}

static bool LadderSendR0Zero(uint16_t length, bool set_hold_bit)
{
    if (length == 0u) {
        return true;
    }
    if (length > kStage2R0Bytes) {
        return false;
    }
    uint8_t r0_buf[kStage2R0Bytes];
    memset(r0_buf, 0, sizeof(r0_buf));
    if (set_hold_bit && length > 5u) {
        r0_buf[5] = kStage2R0FlagHoldAfterApply;
    }
    return LadderSendStream(kStage2Regions[0].region,
                            length,
                            r0_buf,
                            set_hold_bit ? "WRITE_STREAM_BEGIN ladder R0 (hold)"
                                         : "WRITE_STREAM_BEGIN ladder R0",
                            "STATE_DATA ladder R0");
}

static bool LadderResetLoader(const char *context)
{
    /* Full zero LOAD+COMMIT cycle.  Drives the loader FSM back to LD_IDLE
       regardless of which mid-stream state the prior case left it in.
       The CPU is NOT clobbered because info_buf carries no FUSS magic. */
    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendEndSessionClear("END_SESSION ladder reset preflight")) {
        printf("Stage2Probe: ladder reset preflight failed (%s)\n", context);
        Stage2ProbeMarkStuck("ladder reset preflight");
        return false;
    }
    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD ladder reset")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));
    if (!LadderSendR0Zero(kStage2R0Bytes, false)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    if (!LadderSendStream(kStage2Regions[1].region,
                          kLadderResetWramBytes,
                          NULL,
                          "WRITE_STREAM_BEGIN ladder reset WRAM",
                          "STATE_DATA ladder reset WRAM")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    if (!LadderSendStream(kStage2Regions[2].region,
                          kLadderResetHramBytes,
                          NULL,
                          "WRITE_STREAM_BEGIN ladder reset HRAM",
                          "STATE_DATA ladder reset HRAM")) {
        return false;
    }
    if (!WriteCommit()) {
        return false;
    }
    if (!SendEndSessionClear("END_SESSION ladder reset close")) {
        Stage2ProbeMarkStuck("ladder reset close");
        return false;
    }
    DrainRxForMs(kStage2PreflightDrainMs);
    return true;
}

static bool LadderReadDiagOnce(uint8_t out[8])
{
    uint32_t crc = 0u;
    uint32_t packets = 0u;
    memset(out, 0, 8);
    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginSave,
                                     "BEGIN_SAVE ladder diag")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    if (!ReadStage2Region(&kStage2LoaderDiagRegion,
                          &crc,
                          &packets,
                          out,
                          8u)) {
        return false;
    }
    if (!SendEndSessionClear("END_SESSION ladder diag close")) {
        Stage2ProbeMarkStuck("ladder diag close");
        return false;
    }
    return true;
}

static void LadderPrintCaseLine(const LadderCase_t *c,
                                const LadderResult_t *r,
                                bool include_apply_token)
{
    const char *verdict_token =
        (r->verdict == kLadderVerdict_PASS)    ? "PASS"
        : (r->verdict == kLadderVerdict_FAIL)  ? "FAIL"
                                               : "NOT_RUN";
    /* got_state name uses the loader-local state nibble.  For NOT_RUN cases
       we still want a parseable line with explicit n/a markers. */
    char got_state[32];
    if (r->verdict == kLadderVerdict_NOT_RUN) {
        snprintf(got_state, sizeof(got_state), "NA/NA");
    } else {
        snprintf(got_state, sizeof(got_state), "%02X/%s",
                 r->state, Stage2LoaderStateName(r->state));
    }
    if (include_apply_token && c->commit_with_hold) {
        printf("LADDER_CASE name=%s host_wram_sent=%u host_hram_sent=%u "
               "expect_state=%s got_state=%s byte_idx=%u wram_idx=%u "
               "hram_idx=%u flags=0x%02X any_apply_or_hold=%u "
               "verdict=%s reason=%s\n",
               c->name,
               (unsigned)c->wram_len,
               (unsigned)c->hram_len,
               c->expect_state_name,
               got_state,
               (unsigned)r->byte_idx,
               (unsigned)r->wram_idx,
               (unsigned)r->hram_idx,
               r->flags,
               (unsigned)(r->any_apply_or_hold ? 1u : 0u),
               verdict_token,
               r->reason);
    } else {
        printf("LADDER_CASE name=%s host_wram_sent=%u host_hram_sent=%u "
               "expect_state=%s got_state=%s byte_idx=%u wram_idx=%u "
               "hram_idx=%u flags=0x%02X verdict=%s reason=%s\n",
               c->name,
               (unsigned)c->wram_len,
               (unsigned)c->hram_len,
               c->expect_state_name,
               got_state,
               (unsigned)r->byte_idx,
               (unsigned)r->wram_idx,
               (unsigned)r->hram_idx,
               r->flags,
               verdict_token,
               r->reason);
    }
}

static void LadderDecodeDiag(const uint8_t diag[8], LadderResult_t *r)
{
    r->flags    = diag[0];
    r->state    = (uint8_t)(diag[1] & 0x0Fu);
    r->byte_idx = (uint8_t)(diag[2] & 0x7Fu);
    r->wram_idx = (uint16_t)diag[3] | (((uint16_t)(diag[4] & 0x7Fu)) << 8);
    r->hram_idx = (uint8_t)(diag[5] & 0x7Fu);
}

static bool LadderRunStandardCase(const LadderCase_t *c, LadderResult_t *r)
{
    /* Standard (non-COMMIT_HOLD) ladder case.  Opens a fresh BEGIN_LOAD,
       writes the partial payload, closes the LOAD session, then reads diag
       through a BEGIN_SAVE -> 0x7E -> END_SESSION mini-cycle. */
    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD ladder case")) {
        r->reason = "BEGIN_LOAD_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    if (!LadderSendR0Zero(kStage2R0Bytes, false)) {
        r->reason = "R0_WRITE_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));

    if (c->wram_len > 0u) {
        if (!LadderSendStream(kStage2Regions[1].region,
                              c->wram_len,
                              NULL,
                              "WRITE_STREAM_BEGIN ladder WRAM",
                              "STATE_DATA ladder WRAM")) {
            r->reason = "WRAM_WRITE_FAIL";
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    }
    if (c->hram_len > 0u) {
        if (!LadderSendStream(kStage2Regions[2].region,
                              c->hram_len,
                              NULL,
                              "WRITE_STREAM_BEGIN ladder HRAM",
                              "STATE_DATA ladder HRAM")) {
            r->reason = "HRAM_WRITE_FAIL";
            return false;
        }
    }

    if (!SendEndSessionClear("END_SESSION ladder LOAD close")) {
        Stage2ProbeMarkStuck("ladder LOAD close");
        r->reason = "LOAD_END_SESSION_FAIL";
        return false;
    }

    uint8_t diag[8] = {0};
    if (!LadderReadDiagOnce(diag)) {
        r->reason = "DIAG_READ_FAIL";
        return false;
    }
    LadderDecodeDiag(diag, r);
    return true;
}

static bool LadderRunCommitHoldCase(const LadderCase_t *c, LadderResult_t *r)
{
    /* COMMIT_HOLD: full payload with R0 hold-bit set, WRITE_COMMIT, then
       multi-sample read across one BEGIN_SAVE session (single SAVE session
       holds session_pause high for the whole polling window, so the loader
       only sees the LOAD-close falling edge until SAVE-close). */
    static const uint32_t kPollMs[kLadderHoldPolls] = {
        0u, 5u, 20u, 100u, 500u, 1000u, 2000u
    };
    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD ladder commit-hold")) {
        r->reason = "BEGIN_LOAD_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    if (!LadderSendR0Zero(kStage2R0Bytes, true)) {
        r->reason = "R0_WRITE_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    if (!LadderSendStream(kStage2Regions[1].region,
                          c->wram_len,
                          NULL,
                          "WRITE_STREAM_BEGIN ladder commit-hold WRAM",
                          "STATE_DATA ladder commit-hold WRAM")) {
        r->reason = "WRAM_WRITE_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    if (!LadderSendStream(kStage2Regions[2].region,
                          c->hram_len,
                          NULL,
                          "WRITE_STREAM_BEGIN ladder commit-hold HRAM",
                          "STATE_DATA ladder commit-hold HRAM")) {
        r->reason = "HRAM_WRITE_FAIL";
        return false;
    }
    const int64_t commit_send_us = esp_timer_get_time();
    if (!WriteCommit()) {
        r->reason = "WRITE_COMMIT_FAIL";
        return false;
    }
    const int64_t commit_return_us = esp_timer_get_time();
    printf("LADDER_COMMIT_HOLD_COMMIT_RETURN t_us=%lld elapsed_ms=%lld\n",
           (long long)commit_return_us,
           (long long)((commit_return_us - commit_send_us) / 1000));

    if (!SendEndSessionClear("END_SESSION ladder commit-hold LOAD close")) {
        Stage2ProbeMarkStuck("ladder commit-hold LOAD close");
        r->reason = "LOAD_END_SESSION_FAIL";
        return false;
    }

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginSave,
                                     "BEGIN_SAVE ladder commit-hold")) {
        r->reason = "BEGIN_SAVE_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t last_diag[8] = {0};
    bool any_apply_or_hold = false;
    for (uint8_t i = 0u; i < kLadderHoldPolls; ++i) {
        const int64_t target_us = commit_return_us +
                                  ((int64_t)kPollMs[i] * 1000);
        const int64_t now_us = esp_timer_get_time();
        if (now_us < target_us) {
            uint32_t wait_ms = (uint32_t)((target_us - now_us + 999) / 1000);
            if (wait_ms > kLadderHoldClampMaxMs) {
                wait_ms = kLadderHoldClampMaxMs;
            }
            if (wait_ms > 0u) {
                vTaskDelay(pdMS_TO_TICKS(wait_ms));
            }
        }
        uint32_t crc = 0u;
        uint32_t packets = 0u;
        memset(last_diag, 0, sizeof(last_diag));
        if (!ReadStage2Region(&kStage2LoaderDiagRegion,
                              &crc,
                              &packets,
                              last_diag,
                              sizeof(last_diag))) {
            r->reason = "DIAG_READ_FAIL";
            (void)SendEndSessionClear("END_SESSION ladder commit-hold SAVE close (after diag fail)");
            return false;
        }
        const uint8_t flags = last_diag[0];
        const uint8_t state = (uint8_t)(last_diag[1] & 0x0Fu);
        if ((flags & 0x03u) != 0u ||
            state == 0x06u || state == 0x07u || state == 0x08u ||
            state == 0x09u) {
            any_apply_or_hold = true;
        }
        printf("LADDER_COMMIT_HOLD_POLL t_ms=%lu flags=0x%02X state=%02X/%s "
               "byte_idx=%u wram_idx=%u hram_idx=%u\n",
               (unsigned long)kPollMs[i],
               flags,
               state,
               Stage2LoaderStateName(state),
               (unsigned)(last_diag[2] & 0x7Fu),
               (unsigned)((uint16_t)last_diag[3] |
                          (((uint16_t)(last_diag[4] & 0x7Fu)) << 8)),
               (unsigned)(last_diag[5] & 0x7Fu));
    }

    if (!SendEndSessionClear("END_SESSION ladder commit-hold SAVE close")) {
        Stage2ProbeMarkStuck("ladder commit-hold SAVE close");
        r->reason = "SAVE_END_SESSION_FAIL";
        return false;
    }

    LadderDecodeDiag(last_diag, r);
    r->any_apply_or_hold = any_apply_or_hold;
    return true;
}

static void LadderJudge(const LadderCase_t *c, LadderResult_t *r)
{
    /* Apply per-case PASS/FAIL rules.  r already carries the decoded diag
       (state/byte_idx/wram_idx/hram_idx/flags) when LadderRun*Case returned
       true. */
    if (r->verdict == kLadderVerdict_NOT_RUN) {
        return;
    }
    if (c->commit_with_hold) {
        if (r->any_apply_or_hold) {
            r->verdict = kLadderVerdict_PASS;
            r->reason = "ANY_APPLY_OR_HOLD_OR_DONE";
        } else {
            r->verdict = kLadderVerdict_FAIL;
            r->reason = "NO_APPLY_OR_HOLD_TRANSITION";
        }
        return;
    }
    if (r->byte_idx != c->expect_byte_idx) {
        r->verdict = kLadderVerdict_FAIL;
        r->reason = "BYTE_IDX_MISMATCH";
        return;
    }
    if (r->state != c->expect_state) {
        r->verdict = kLadderVerdict_FAIL;
        r->reason = "STATE_MISMATCH";
        return;
    }
    if (r->wram_idx != c->expect_wram_idx) {
        r->verdict = kLadderVerdict_FAIL;
        r->reason = "WRAM_IDX_MISMATCH";
        return;
    }
    if (r->hram_idx != c->expect_hram_idx) {
        r->verdict = kLadderVerdict_FAIL;
        r->reason = "HRAM_IDX_MISMATCH";
        return;
    }
    r->verdict = kLadderVerdict_PASS;
    r->reason = "OK";
}

static const char *LadderBranchToken(LadderBranch_t b)
{
    switch (b) {
    case kLadderBranch_NONE:                                    return "NONE";
    case kLadderBranch_R0_HEADER_PATH:                          return "R0_HEADER_PATH";
    case kLadderBranch_WRAM_STREAM_BYTE_PUMP_OR_LOADER_COUNT:   return "WRAM_STREAM_BYTE_PUMP_OR_LOADER_COUNT";
    case kLadderBranch_WRAM_FINAL_BOUNDARY:                     return "WRAM_FINAL_BOUNDARY";
    case kLadderBranch_HRAM_STREAM_BYTE_PUMP_OR_LOADER_COUNT:   return "HRAM_STREAM_BYTE_PUMP_OR_LOADER_COUNT";
    case kLadderBranch_HRAM_FINAL_BOUNDARY:                     return "HRAM_FINAL_BOUNDARY";
    case kLadderBranch_COMMIT_APPLY_PULSE:                      return "COMMIT_APPLY_PULSE";
    case kLadderBranch_INCONCLUSIVE:                            return "INCONCLUSIVE";
    case kLadderBranch_BLOCKED_BY_BUILD_OR_PNR:                 return "BLOCKED_BY_BUILD_OR_PNR";
    case kLadderBranch_BLOCKED_BY_HARDWARE:                     return "BLOCKED_BY_HARDWARE";
    }
    return "INCONCLUSIVE";
}

static LadderBranch_t LadderClassifyBranch(const LadderResult_t results[kLadderCaseCount],
                                           int *first_fail_idx_out)
{
    int first_fail_idx = -1;
    for (uint8_t i = 0u; i < kLadderCaseCount; ++i) {
        if (results[i].verdict == kLadderVerdict_FAIL) {
            first_fail_idx = (int)i;
            break;
        }
    }
    if (first_fail_idx_out != NULL) {
        *first_fail_idx_out = first_fail_idx;
    }
    if (first_fail_idx < 0) {
        return kLadderBranch_NONE;
    }
    const char *name = kLadderCases[first_fail_idx].name;
    if (strcmp(name, "R0_ONLY") == 0) {
        return kLadderBranch_R0_HEADER_PATH;
    }
    if (strcmp(name, "WRAM_1") == 0 ||
        strcmp(name, "WRAM_8") == 0 ||
        strcmp(name, "WRAM_4096") == 0 ||
        strcmp(name, "WRAM_32760") == 0 ||
        strcmp(name, "WRAM_32767") == 0) {
        return kLadderBranch_WRAM_STREAM_BYTE_PUMP_OR_LOADER_COUNT;
    }
    if (strcmp(name, "WRAM_32768") == 0) {
        return kLadderBranch_WRAM_FINAL_BOUNDARY;
    }
    if (strcmp(name, "HRAM_1") == 0) {
        return kLadderBranch_HRAM_STREAM_BYTE_PUMP_OR_LOADER_COUNT;
    }
    if (strcmp(name, "HRAM_127") == 0) {
        return kLadderBranch_HRAM_FINAL_BOUNDARY;
    }
    if (strcmp(name, "COMMIT_HOLD") == 0) {
        return kLadderBranch_COMMIT_APPLY_PULSE;
    }
    return kLadderBranch_INCONCLUSIVE;
}

static bool RunLoadProgressLadderInner(void)
{
    LadderResult_t results[kLadderCaseCount];
    memset(results, 0, sizeof(results));
    for (uint8_t i = 0u; i < kLadderCaseCount; ++i) {
        results[i].verdict = kLadderVerdict_NOT_RUN;
        results[i].reason = "NOT_RUN";
    }

    bool blocked_by_fatal = false;
    bool any_case_ok = false;

    /* Initial preflight: clean state.  An END_SESSION here is also our
       chance to detect a board that is already non-responsive (in which
       case the entire ladder is marked NOT_RUN). */
    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        for (uint8_t i = 0u; i < kLadderCaseCount; ++i) {
            results[i].verdict = kLadderVerdict_NOT_RUN;
            results[i].reason = "NOT_RUN_BLOCKED_BY_FATAL_FAIL";
        }
        printf("LADDER_FATAL preflight_clear=FAIL\n");
        blocked_by_fatal = true;
        goto report;
    }

    for (uint8_t i = 0u; i < kLadderCaseCount; ++i) {
        const LadderCase_t *c = &kLadderCases[i];
        LadderResult_t *r = &results[i];

        /* Per-case loader reset so wram_idx / hram_idx / state are
           authoritative for this case, not contaminated by the prior
           partial-stream case.  If reset fails, mark this case + all
           remaining as NOT_RUN_BLOCKED_BY_FATAL_FAIL and stop. */
        if (!LadderResetLoader(c->name)) {
            r->verdict = kLadderVerdict_NOT_RUN;
            r->reason = "NOT_RUN_BLOCKED_BY_FATAL_FAIL";
            LadderPrintCaseLine(c, r, false);
            for (uint8_t j = (uint8_t)(i + 1u); j < kLadderCaseCount; ++j) {
                results[j].verdict = kLadderVerdict_NOT_RUN;
                results[j].reason = "NOT_RUN_BLOCKED_BY_FATAL_FAIL";
                LadderPrintCaseLine(&kLadderCases[j], &results[j], false);
            }
            blocked_by_fatal = true;
            goto report;
        }

        bool case_ran = false;
        if (c->commit_with_hold) {
            case_ran = LadderRunCommitHoldCase(c, r);
        } else {
            case_ran = LadderRunStandardCase(c, r);
        }
        if (!case_ran) {
            r->verdict = kLadderVerdict_FAIL;
            if (r->reason == NULL) {
                r->reason = "TRANSPORT_FAIL";
            }
        } else {
            r->verdict = kLadderVerdict_PASS; /* tentative; LadderJudge() refines */
            LadderJudge(c, r);
            any_case_ok = true;
        }
        LadderPrintCaseLine(c, r, true);

        /* Tear down: send a final END_SESSION to make sure nothing is
           still pending before the next case's reset. */
        DrainRxForMs(kStage2PreflightDrainMs);
        if (!SendEndSessionClear("END_SESSION ladder case teardown")) {
            Stage2ProbeMarkStuck("ladder case teardown");
            for (uint8_t j = (uint8_t)(i + 1u); j < kLadderCaseCount; ++j) {
                results[j].verdict = kLadderVerdict_NOT_RUN;
                results[j].reason = "NOT_RUN_BLOCKED_BY_FATAL_FAIL";
                LadderPrintCaseLine(&kLadderCases[j], &results[j], false);
            }
            blocked_by_fatal = true;
            goto report;
        }
    }

    /* Final clean: walk the loader one last time to LD_IDLE for tidy
       hand-off to the next command in the host script. */
    (void)LadderResetLoader("final cleanup");

report:
    {
        int first_fail_idx = -1;
        LadderBranch_t branch = LadderClassifyBranch(results, &first_fail_idx);
        const char *first_fail_name =
            (first_fail_idx < 0) ? "NONE" : kLadderCases[first_fail_idx].name;
        const char *first_fail_reason =
            (first_fail_idx < 0) ? "NONE"
                                 : (results[first_fail_idx].reason != NULL
                                        ? results[first_fail_idx].reason
                                        : "UNSPECIFIED");
        if (blocked_by_fatal && first_fail_idx < 0) {
            first_fail_reason = "NOT_RUN_BLOCKED_BY_FATAL_FAIL";
        }
        printf("LADDER_FIRST_FAIL name=%s reason=%s\n",
               first_fail_name,
               first_fail_reason);
        printf("LADDER_PRIMARY_BRANCH %s\n", LadderBranchToken(branch));
        printf("PRODUCT_VERDICT FAIL\n");
    }
    (void)any_case_ok;

    if (!EndSessionCleanup()) {
        printf("Stage2Probe: load-progress-ladder final cleanup failed; "
               "treating command as FAIL\n");
        return false;
    }
    return true;
}

static bool RunLoadProgressLadder(void)
{
    if (Stage2ProbeRefuseIfStuck("load-progress-ladder")) {
        return false;
    }
    printf("Stage2Probe: START mode=load-progress-ladder "
           "protocol=stage2-single-byte-begin cases=%u "
           "reset_between_cases=zero_load_commit\n",
           (unsigned)kLadderCaseCount);
    const bool ok = RunWithUartOwner(RunLoadProgressLadderInner);
    printf("Stage2Probe: RESULT %s mode=load-progress-ladder\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

/* ============================================================================
 *  LOAD state ladder, byte0..5 map (2026-05-16)
 *
 *  Uses the accepted/flashed 0x7E loader diagnostic map (byte0..5 only):
 *
 *      byte0 = flags (bit0 loader_apply, bit1 state==LD_HOLD_APPLY,
 *                     bit2 hold_fall_seen, bit3 session_pause)
 *      byte1 = loader state low nibble
 *      byte2 = cpu_buf_pc[7:0]
 *      byte3 = cpu_buf_pc[15:8]
 *      byte4 = loader_t80_1_data[7:0]
 *      byte5 = loader_t80_1_data[15:8]
 *      byte6/7 = reserved zero / unavailable
 *
 *  The command runs four cases, each from a fresh loader-IDLE state, with a
 *  deterministic R0 v2 marker payload whose PC field is A55A.  It localizes
 *  where LOAD first fails using the *accepted* bitstream only: no FPGA RTL
 *  change, no PnR, no FPGA flash.
 *
 *  Branch tokens:
 *      HEADER_NOT_LATCHED
 *      WRAM_FINAL_BOUNDARY_OR_STREAM_PUMP
 *      HRAM_FINAL_BOUNDARY_OR_STREAM_PUMP
 *      COMMIT_APPLY_PULSE
 *      NONE
 *      INCONCLUSIVE
 *      BLOCKED_BY_HARDWARE
 * ========================================================================== */

enum {
    kByte05MarkerPc = 0xA55Au,
    kByte05PollCount = 7u,
};

typedef enum {
    kByte05Verdict_PASS = 0,
    kByte05Verdict_FAIL,
    kByte05Verdict_NOT_RUN,
} Byte05Verdict_t;

typedef enum {
    kByte05Branch_NONE = 0,
    kByte05Branch_HEADER_NOT_LATCHED,
    kByte05Branch_WRAM_FINAL_BOUNDARY_OR_STREAM_PUMP,
    kByte05Branch_HRAM_FINAL_BOUNDARY_OR_STREAM_PUMP,
    kByte05Branch_COMMIT_APPLY_PULSE,
    kByte05Branch_INCONCLUSIVE,
    kByte05Branch_BLOCKED_BY_HARDWARE,
} Byte05Branch_t;

typedef struct {
    const char *name;
    bool send_wram_full;
    bool send_hram_full;
    bool do_commit;
    bool hold_bit;
    bool is_poll;
    uint8_t expect_state;            /* primary expected loader_state nibble */
    const char *expect_state_name;   /* human-readable token for printout */
} Byte05Case_t;

typedef struct {
    Byte05Verdict_t verdict;
    const char *reason;
    bool diag_valid;
    uint8_t state;       /* loader state low nibble, from last sample */
    uint8_t flags;
    uint16_t cpu_buf_pc;
    uint16_t loader_pc;
    bool any_apply_or_hold; /* COMMIT_HOLD only */
} Byte05Result_t;

static const Byte05Case_t kByte05Cases[] = {
    /* name                 wram   hram   commit hold   poll   expect_state expect_state_name */
    { "R0_MARKER_ONLY",     false, false, false, false, false, 0x04u, "LD_STREAM_WRAM"  },
    { "WRAM_FULL_BOUNDARY", true,  false, false, false, false, 0x05u, "LD_STREAM_HRAM"  },
    { "HRAM_FULL_BOUNDARY", true,  true,  false, false, false, 0x06u, "LD_WAIT_APPLY"   },
    { "COMMIT_HOLD",        true,  true,  true,  true,  true,  0x09u, "LD_FORCE_IDLE_OR_LD_HOLD_APPLY_OR_LD_DONE_OR_APPLY_HOLD_FLAGS" },
};
enum { kByte05CaseCount = (uint8_t)(sizeof(kByte05Cases) / sizeof(kByte05Cases[0])) };

static void Byte05BuildMarkerR0(uint8_t r0[kStage2R0Bytes], bool hold_on)
{
    memset(r0, 0, kStage2R0Bytes);
    r0[0] = 'F';
    r0[1] = 'U';
    r0[2] = 'S';
    r0[3] = 'S';
    r0[4] = 0x02u;
    r0[5] = hold_on ? kStage2R0FlagHoldAfterApply : 0x00u;
    /* r0[6]/r0[7] reserved zero */
    /* PC field at bytes 18..19 (little-endian), inside the 42-byte CPU
       section that starts at r0[8].  Marker PC = A55A. */
    r0[18] = (uint8_t)(kByte05MarkerPc & 0xFFu);
    r0[19] = (uint8_t)((kByte05MarkerPc >> 8) & 0xFFu);
}

static bool Byte05SendMarkerR0(bool hold_on)
{
    uint8_t r0[kStage2R0Bytes];
    Byte05BuildMarkerR0(r0, hold_on);
    return LadderSendStream(kStage2Regions[0].region,
                            kStage2R0Bytes,
                            r0,
                            hold_on ? "WRITE_STREAM_BEGIN byte05 R0 marker (hold)"
                                    : "WRITE_STREAM_BEGIN byte05 R0 marker",
                            "STATE_DATA byte05 R0 marker");
}

static bool Byte05SendMarkerR0ReversedWireOrder(void)
{
    uint8_t r0[kStage2R0Bytes];
    Byte05BuildMarkerR0(r0, false);
    return LadderSendStreamWireOrder(kStage2Regions[0].region,
                                     kStage2R0Bytes,
                                     r0,
                                     "WRITE_STREAM_BEGIN byte05 R0 marker reversed-wire",
                                     "STATE_DATA byte05 R0 marker reversed-wire",
                                     true);
}

static void Byte05BuildOffsetPatternR0(uint8_t r0[kStage2R0Bytes])
{
    for (uint8_t i = 0u; i < kStage2R0Bytes; ++i) {
        r0[i] = (uint8_t)(0x80u + i);
    }
}

static bool Byte05SendOffsetPatternR0(void)
{
    uint8_t r0[kStage2R0Bytes];
    Byte05BuildOffsetPatternR0(r0);
    return LadderSendStream(kStage2Regions[0].region,
                            kStage2R0Bytes,
                            r0,
                            "WRITE_STREAM_BEGIN byte05 R0 offset-pattern",
                            "STATE_DATA byte05 R0 offset-pattern");
}

/* Bytewise R0 sender: identical R0 payload bytes as Byte05SendOffsetPatternR0,
   but forces each STATE_DATA packet to carry exactly one state byte instead of
   the usual up-to-FUSION_STATE_DATA_MAX_DATA bytes.  Purpose is to give the
   FPGA bridge byte pump a fresh packet boundary (and therefore a fresh
   state_byte_valid rising edge on the hclk side) per state byte, without
   changing FPGA RTL.  Send order is byte 0 first, then byte 1, ... so chunk
   ordering inside a STATE_DATA payload is irrelevant. */
static bool Byte05SendOffsetPatternR0Bytewise(void)
{
    uint8_t r0[kStage2R0Bytes];
    Byte05BuildOffsetPatternR0(r0);

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildWriteStreamBegin(kStage2Regions[0].region,
                                               0u,
                                               kStage2R0Bytes,
                                               &begin)) {
        printf("Stage2Probe: build WRITE_STREAM_BEGIN bytewise R0 failed\n");
        return false;
    }
    if (!SendFrame("WRITE_STREAM_BEGIN bytewise R0 offset-pattern",
                   &begin,
                   false)) {
        return false;
    }
    if (!ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                   (uint8_t)kFusionOp_WriteStreamBegin,
                   "WRITE_STREAM_BEGIN bytewise R0 offset-pattern")) {
        return false;
    }
    for (uint16_t i = 0u; i < kStage2R0Bytes; ++i) {
        FusionV2Frame_t data_frame;
        if (!FusionSavestate_BuildStateData((uint16_t)i,
                                            &r0[i],
                                            1u,
                                            &data_frame)) {
            printf("Stage2Probe: build STATE_DATA bytewise R0 seq=%u failed\n",
                   (unsigned)i);
            return false;
        }
        if (!SendFrame("STATE_DATA bytewise R0 offset-pattern",
                       &data_frame,
                       true)) {
            return false;
        }
    }
    return ExpectCtl((uint8_t)kFusionOp_AckDone,
                     (uint8_t)kFusionOp_WriteStreamBegin,
                     "WRITE_STREAM_BEGIN bytewise R0 offset-pattern");
}

static const char *Byte05BranchToken(Byte05Branch_t b)
{
    switch (b) {
    case kByte05Branch_NONE:                                  return "NONE";
    case kByte05Branch_HEADER_NOT_LATCHED:                    return "HEADER_NOT_LATCHED";
    case kByte05Branch_WRAM_FINAL_BOUNDARY_OR_STREAM_PUMP:    return "WRAM_FINAL_BOUNDARY_OR_STREAM_PUMP";
    case kByte05Branch_HRAM_FINAL_BOUNDARY_OR_STREAM_PUMP:    return "HRAM_FINAL_BOUNDARY_OR_STREAM_PUMP";
    case kByte05Branch_COMMIT_APPLY_PULSE:                    return "COMMIT_APPLY_PULSE";
    case kByte05Branch_INCONCLUSIVE:                          return "INCONCLUSIVE";
    case kByte05Branch_BLOCKED_BY_HARDWARE:                   return "BLOCKED_BY_HARDWARE";
    }
    return "INCONCLUSIVE";
}

static void Byte05PrintCaseLine(const Byte05Case_t *c, const Byte05Result_t *r)
{
    const char *verdict_token =
        (r->verdict == kByte05Verdict_PASS)    ? "PASS"
        : (r->verdict == kByte05Verdict_FAIL)  ? "FAIL"
                                               : "NOT_RUN";
    char got_state[32];
    if (!r->diag_valid) {
        snprintf(got_state, sizeof(got_state), "NA/NA");
    } else {
        snprintf(got_state, sizeof(got_state), "%02X/%s",
                 r->state, Stage2LoaderStateName(r->state));
    }
    printf("BYTE05_CASE name=%s marker_pc=%04X expect_state=%s got_state=%s "
           "cpu_buf_pc=%04X loader_pc=%04X flags=0x%02X verdict=%s reason=%s\n",
           c->name,
           (unsigned)kByte05MarkerPc,
           c->expect_state_name,
           got_state,
           r->cpu_buf_pc,
           r->loader_pc,
           r->flags,
           verdict_token,
           r->reason != NULL ? r->reason : "UNSPECIFIED");
}

static void Byte05DecodeDiag(const uint8_t diag[8], Byte05Result_t *r)
{
    r->flags = diag[0];
    r->state = (uint8_t)(diag[1] & 0x0Fu);
    r->cpu_buf_pc = (uint16_t)diag[2] | ((uint16_t)diag[3] << 8);
    r->loader_pc = (uint16_t)diag[4] | ((uint16_t)diag[5] << 8);
    r->diag_valid = true;
}

static bool Byte05RunStandardCase(const Byte05Case_t *c, Byte05Result_t *r)
{
    /* Standard (non-COMMIT_HOLD) case.  Opens a fresh BEGIN_LOAD on top of a
       reset loader, writes the marker R0 plus any required WRAM/HRAM bytes,
       closes the LOAD session, then reads 0x7E through a single
       BEGIN_SAVE -> read region 0x7E -> END_SESSION mini-cycle. */
    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD byte05 case")) {
        r->reason = "BEGIN_LOAD_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    if (!Byte05SendMarkerR0(false)) {
        r->reason = "R0_WRITE_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));

    if (c->send_wram_full) {
        if (!LadderSendStream(kStage2Regions[1].region,
                              kStage2Regions[1].length,
                              NULL,
                              "WRITE_STREAM_BEGIN byte05 WRAM",
                              "STATE_DATA byte05 WRAM")) {
            r->reason = "WRAM_WRITE_FAIL";
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    }
    if (c->send_hram_full) {
        if (!LadderSendStream(kStage2Regions[2].region,
                              kStage2Regions[2].length,
                              NULL,
                              "WRITE_STREAM_BEGIN byte05 HRAM",
                              "STATE_DATA byte05 HRAM")) {
            r->reason = "HRAM_WRITE_FAIL";
            return false;
        }
    }

    if (!SendEndSessionClear("END_SESSION byte05 LOAD close")) {
        Stage2ProbeMarkStuck("byte05 LOAD close");
        r->reason = "LOAD_END_SESSION_FAIL";
        return false;
    }

    uint8_t diag[8] = {0};
    if (!LadderReadDiagOnce(diag)) {
        r->reason = "DIAG_READ_FAIL";
        return false;
    }
    Byte05DecodeDiag(diag, r);
    return true;
}

static bool Byte05RunCommitHoldCase(const Byte05Case_t *c, Byte05Result_t *r)
{
    static const uint32_t kPollMs[kByte05PollCount] = {
        0u, 5u, 20u, 100u, 500u, 1000u, 2000u
    };
    (void)c;

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD byte05 commit-hold")) {
        r->reason = "BEGIN_LOAD_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    if (!Byte05SendMarkerR0(true)) {
        r->reason = "R0_WRITE_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));

    if (!LadderSendStream(kStage2Regions[1].region,
                          kStage2Regions[1].length,
                          NULL,
                          "WRITE_STREAM_BEGIN byte05 commit-hold WRAM",
                          "STATE_DATA byte05 commit-hold WRAM")) {
        r->reason = "WRAM_WRITE_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2PostRegionMs));
    if (!LadderSendStream(kStage2Regions[2].region,
                          kStage2Regions[2].length,
                          NULL,
                          "WRITE_STREAM_BEGIN byte05 commit-hold HRAM",
                          "STATE_DATA byte05 commit-hold HRAM")) {
        r->reason = "HRAM_WRITE_FAIL";
        return false;
    }

    const int64_t commit_send_us = esp_timer_get_time();
    if (!WriteCommit()) {
        r->reason = "WRITE_COMMIT_FAIL";
        return false;
    }
    const int64_t commit_return_us = esp_timer_get_time();
    printf("BYTE05_COMMIT_RETURN t_us=%lld elapsed_ms=%lld\n",
           (long long)commit_return_us,
           (long long)((commit_return_us - commit_send_us) / 1000));

    if (!SendEndSessionClear("END_SESSION byte05 commit-hold LOAD close")) {
        Stage2ProbeMarkStuck("byte05 commit-hold LOAD close");
        r->reason = "LOAD_END_SESSION_FAIL";
        return false;
    }

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginSave,
                                     "BEGIN_SAVE byte05 commit-hold")) {
        r->reason = "BEGIN_SAVE_FAIL";
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t last_diag[8] = {0};
    bool any_apply_or_hold = false;
    for (uint8_t i = 0u; i < kByte05PollCount; ++i) {
        const int64_t target_us = commit_return_us +
                                  ((int64_t)kPollMs[i] * 1000);
        const int64_t now_us = esp_timer_get_time();
        if (now_us < target_us) {
            uint32_t wait_ms = (uint32_t)((target_us - now_us + 999) / 1000);
            if (wait_ms > kLadderHoldClampMaxMs) {
                wait_ms = kLadderHoldClampMaxMs;
            }
            if (wait_ms > 0u) {
                vTaskDelay(pdMS_TO_TICKS(wait_ms));
            }
        }
        uint32_t crc = 0u;
        uint32_t packets = 0u;
        memset(last_diag, 0, sizeof(last_diag));
        if (!ReadStage2Region(&kStage2LoaderDiagRegion,
                              &crc,
                              &packets,
                              last_diag,
                              sizeof(last_diag))) {
            r->reason = "DIAG_READ_FAIL";
            (void)SendEndSessionClear("END_SESSION byte05 commit-hold SAVE close (diag fail)");
            return false;
        }
        const uint8_t flags = last_diag[0];
        const uint8_t state = (uint8_t)(last_diag[1] & 0x0Fu);
        const uint16_t cpu_pc = (uint16_t)last_diag[2] |
                                ((uint16_t)last_diag[3] << 8);
        const uint16_t loader_pc = (uint16_t)last_diag[4] |
                                   ((uint16_t)last_diag[5] << 8);
        if ((flags & 0x03u) != 0u ||
            state == 0x06u || state == 0x07u || state == 0x08u ||
            state == 0x09u) {
            any_apply_or_hold = true;
        }
        printf("BYTE05_POLL case=COMMIT_HOLD t_ms=%lu flags=0x%02X "
               "state=%02X/%s cpu_buf_pc=%04X loader_pc=%04X\n",
               (unsigned long)kPollMs[i],
               flags,
               state,
               Stage2LoaderStateName(state),
               cpu_pc,
               loader_pc);
    }

    if (!SendEndSessionClear("END_SESSION byte05 commit-hold SAVE close")) {
        Stage2ProbeMarkStuck("byte05 commit-hold SAVE close");
        r->reason = "SAVE_END_SESSION_FAIL";
        return false;
    }

    Byte05DecodeDiag(last_diag, r);
    r->any_apply_or_hold = any_apply_or_hold;
    return true;
}

static void Byte05Judge(const Byte05Case_t *c, Byte05Result_t *r)
{
    if (r->verdict == kByte05Verdict_NOT_RUN) {
        return;
    }
    if (!r->diag_valid) {
        r->verdict = kByte05Verdict_FAIL;
        if (r->reason == NULL) {
            r->reason = "NO_DIAG";
        }
        return;
    }
    if (c->is_poll) {
        if (r->any_apply_or_hold) {
            r->verdict = kByte05Verdict_PASS;
            r->reason = "APPLY_OR_HOLD_OR_DONE_SEEN";
        } else {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "NO_APPLY_OR_HOLD_OR_DONE";
        }
        return;
    }
    /* Non-poll cases: judge state + cpu_buf_pc. */
    if (strcmp(c->name, "R0_MARKER_ONLY") == 0) {
        if (r->cpu_buf_pc != (uint16_t)kByte05MarkerPc) {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "HEADER_NOT_LATCHED";
            return;
        }
        if (r->state != c->expect_state) {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "STATE_MISMATCH";
            return;
        }
        r->verdict = kByte05Verdict_PASS;
        r->reason = "OK";
        return;
    }
    if (strcmp(c->name, "WRAM_FULL_BOUNDARY") == 0) {
        if (r->state == 0x04u) {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "STILL_STREAM_WRAM";
            return;
        }
        if (r->state != c->expect_state) {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "STATE_MISMATCH";
            return;
        }
        if (r->cpu_buf_pc != (uint16_t)kByte05MarkerPc) {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "CPU_BUF_PC_MISMATCH";
            return;
        }
        r->verdict = kByte05Verdict_PASS;
        r->reason = "OK";
        return;
    }
    if (strcmp(c->name, "HRAM_FULL_BOUNDARY") == 0) {
        if (r->state == 0x05u) {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "STILL_STREAM_HRAM";
            return;
        }
        if (r->state != c->expect_state) {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "STATE_MISMATCH";
            return;
        }
        if (r->cpu_buf_pc != (uint16_t)kByte05MarkerPc) {
            r->verdict = kByte05Verdict_FAIL;
            r->reason = "CPU_BUF_PC_MISMATCH";
            return;
        }
        r->verdict = kByte05Verdict_PASS;
        r->reason = "OK";
        return;
    }
    r->verdict = kByte05Verdict_FAIL;
    r->reason = "UNHANDLED_CASE";
}

static Byte05Branch_t Byte05ClassifyBranch(const Byte05Result_t results[],
                                           int *first_fail_idx_out)
{
    int first_fail_idx = -1;
    for (uint8_t i = 0u; i < kByte05CaseCount; ++i) {
        if (results[i].verdict == kByte05Verdict_FAIL) {
            first_fail_idx = (int)i;
            break;
        }
    }
    if (first_fail_idx_out != NULL) {
        *first_fail_idx_out = first_fail_idx;
    }
    if (first_fail_idx < 0) {
        return kByte05Branch_NONE;
    }
    const char *name = kByte05Cases[first_fail_idx].name;
    if (strcmp(name, "R0_MARKER_ONLY") == 0) {
        return kByte05Branch_HEADER_NOT_LATCHED;
    }
    if (strcmp(name, "WRAM_FULL_BOUNDARY") == 0) {
        return kByte05Branch_WRAM_FINAL_BOUNDARY_OR_STREAM_PUMP;
    }
    if (strcmp(name, "HRAM_FULL_BOUNDARY") == 0) {
        return kByte05Branch_HRAM_FINAL_BOUNDARY_OR_STREAM_PUMP;
    }
    if (strcmp(name, "COMMIT_HOLD") == 0) {
        return kByte05Branch_COMMIT_APPLY_PULSE;
    }
    return kByte05Branch_INCONCLUSIVE;
}

static bool RunLoadStateLadderByte05Inner(void)
{
    Byte05Result_t results[kByte05CaseCount];
    memset(results, 0, sizeof(results));
    for (uint8_t i = 0u; i < kByte05CaseCount; ++i) {
        results[i].verdict = kByte05Verdict_NOT_RUN;
        results[i].reason = "NOT_RUN";
    }

    bool blocked_by_hw = false;
    bool stop_on_header = false;

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        for (uint8_t i = 0u; i < kByte05CaseCount; ++i) {
            results[i].verdict = kByte05Verdict_NOT_RUN;
            results[i].reason = "NOT_RUN_BLOCKED_BY_HARDWARE";
        }
        printf("BYTE05_FATAL preflight_clear=FAIL\n");
        blocked_by_hw = true;
        goto report;
    }

    for (uint8_t i = 0u; i < kByte05CaseCount; ++i) {
        const Byte05Case_t *c = &kByte05Cases[i];
        Byte05Result_t *r = &results[i];

        if (stop_on_header) {
            r->verdict = kByte05Verdict_NOT_RUN;
            r->reason = "NOT_RUN_AFTER_HEADER_NOT_LATCHED";
            Byte05PrintCaseLine(c, r);
            continue;
        }

        /* Fresh reset/clear cycle before every case so byte_idx/wram_idx/
           hram_idx/state are authoritative for this case alone.  Zero R0
           carries no FUSS magic, so info_nonzero==0 and the CPU FFs are
           not clobbered while the loader FSM walks back to LD_IDLE. */
        if (!LadderResetLoader(c->name)) {
            r->verdict = kByte05Verdict_NOT_RUN;
            r->reason = "NOT_RUN_BLOCKED_BY_HARDWARE";
            Byte05PrintCaseLine(c, r);
            for (uint8_t j = (uint8_t)(i + 1u); j < kByte05CaseCount; ++j) {
                results[j].verdict = kByte05Verdict_NOT_RUN;
                results[j].reason = "NOT_RUN_BLOCKED_BY_HARDWARE";
                Byte05PrintCaseLine(&kByte05Cases[j], &results[j]);
            }
            blocked_by_hw = true;
            goto report;
        }

        bool case_ran = false;
        if (c->is_poll) {
            case_ran = Byte05RunCommitHoldCase(c, r);
        } else {
            case_ran = Byte05RunStandardCase(c, r);
        }
        if (!case_ran) {
            r->verdict = kByte05Verdict_FAIL;
            if (r->reason == NULL) {
                r->reason = "TRANSPORT_FAIL";
            }
            blocked_by_hw = true;
            Byte05PrintCaseLine(c, r);
            for (uint8_t j = (uint8_t)(i + 1u); j < kByte05CaseCount; ++j) {
                results[j].verdict = kByte05Verdict_NOT_RUN;
                results[j].reason = "NOT_RUN_BLOCKED_BY_HARDWARE";
                Byte05PrintCaseLine(&kByte05Cases[j], &results[j]);
            }
            goto report;
        }

        r->verdict = kByte05Verdict_PASS; /* tentative; Byte05Judge() refines */
        Byte05Judge(c, r);

        if (r->verdict == kByte05Verdict_FAIL &&
            strcmp(c->name, "R0_MARKER_ONLY") == 0 &&
            r->reason != NULL &&
            strcmp(r->reason, "HEADER_NOT_LATCHED") == 0) {
            /* "if cpu_buf_pc is not A55A, classify HEADER_NOT_LATCHED and
               stop further cases unless cleanup proves it is safe to
               continue."  Per spec we stop. */
            stop_on_header = true;
        }

        Byte05PrintCaseLine(c, r);

        DrainRxForMs(kStage2PreflightDrainMs);
        if (!SendEndSessionClear("END_SESSION byte05 case teardown")) {
            Stage2ProbeMarkStuck("byte05 case teardown");
            blocked_by_hw = true;
            for (uint8_t j = (uint8_t)(i + 1u); j < kByte05CaseCount; ++j) {
                results[j].verdict = kByte05Verdict_NOT_RUN;
                results[j].reason = "NOT_RUN_BLOCKED_BY_HARDWARE";
                Byte05PrintCaseLine(&kByte05Cases[j], &results[j]);
            }
            goto report;
        }
    }

    (void)LadderResetLoader("byte05 final cleanup");

report:
    {
        int first_fail_idx = -1;
        Byte05Branch_t branch = Byte05ClassifyBranch(results, &first_fail_idx);
        const char *first_fail_name =
            (first_fail_idx < 0) ? "NONE"
                                 : kByte05Cases[first_fail_idx].name;
        const char *first_fail_reason =
            (first_fail_idx < 0)
                ? "NONE"
                : (results[first_fail_idx].reason != NULL
                       ? results[first_fail_idx].reason
                       : "UNSPECIFIED");
        if (blocked_by_hw) {
            branch = kByte05Branch_BLOCKED_BY_HARDWARE;
            if (first_fail_idx < 0) {
                first_fail_reason = "NOT_RUN_BLOCKED_BY_HARDWARE";
            }
        }
        printf("BYTE05_FIRST_FAIL name=%s reason=%s\n",
               first_fail_name,
               first_fail_reason);
        printf("BYTE05_PRIMARY_BRANCH %s\n", Byte05BranchToken(branch));
        printf("PRODUCT_VERDICT FAIL\n");
    }

    if (!EndSessionCleanup()) {
        printf("Stage2Probe: load-state-ladder-byte05 final cleanup failed; "
               "treating command as FAIL\n");
        return false;
    }
    return true;
}

static bool RunLoadStateLadderByte05(void)
{
    if (Stage2ProbeRefuseIfStuck("load-state-ladder-byte05")) {
        return false;
    }
    printf("Stage2Probe: START mode=load-state-ladder-byte05 "
           "protocol=stage2-single-byte-begin cases=%u "
           "marker_pc=%04X reset_between_cases=zero_load_commit "
           "diag_map=accepted_byte05\n",
           (unsigned)kByte05CaseCount,
           (unsigned)kByte05MarkerPc);
    const bool ok = RunWithUartOwner(RunLoadStateLadderByte05Inner);
    printf("Stage2Probe: RESULT %s mode=load-state-ladder-byte05\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunLoadR0WireOrderByte05Inner(void)
{
    uint8_t diag[8];
    Byte05Result_t r;
    memset(&r, 0, sizeof(r));
    r.verdict = kByte05Verdict_FAIL;
    r.reason = "UNSPECIFIED";

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        printf("BYTE05_WIRE_ORDER mode=reversed_chunks marker_pc=%04X "
               "verdict=FAIL reason=PRE_FLIGHT_END_SESSION_FAIL\n",
               (unsigned)kByte05MarkerPc);
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    if (!LadderResetLoader("byte05 wire-order preflight")) {
        printf("BYTE05_WIRE_ORDER mode=reversed_chunks marker_pc=%04X "
               "verdict=FAIL reason=PRE_FLIGHT_LOADER_RESET_FAIL\n",
               (unsigned)kByte05MarkerPc);
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD byte05 wire-order case")) {
        printf("BYTE05_WIRE_ORDER mode=reversed_chunks marker_pc=%04X "
               "verdict=FAIL reason=BEGIN_LOAD_FAIL\n",
               (unsigned)kByte05MarkerPc);
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }
    if (!Byte05SendMarkerR0ReversedWireOrder()) {
        printf("BYTE05_WIRE_ORDER mode=reversed_chunks marker_pc=%04X "
               "verdict=FAIL reason=R0_STREAM_FAIL\n",
               (unsigned)kByte05MarkerPc);
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }
    if (!SendEndSessionClear("END_SESSION byte05 wire-order LOAD close")) {
        Stage2ProbeMarkStuck("byte05 wire-order load close");
        printf("BYTE05_WIRE_ORDER mode=reversed_chunks marker_pc=%04X "
               "verdict=FAIL reason=END_SESSION_FAIL\n",
               (unsigned)kByte05MarkerPc);
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    if (!LadderReadDiagOnce(diag)) {
        printf("BYTE05_WIRE_ORDER mode=reversed_chunks marker_pc=%04X "
               "verdict=FAIL reason=DIAG_READ_FAIL\n",
               (unsigned)kByte05MarkerPc);
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    Byte05DecodeDiag(diag, &r);
    const bool marker_latched = (r.cpu_buf_pc == (uint16_t)kByte05MarkerPc);
    const bool state_after_header = (r.state == 0x04u);
    const char *reason =
        marker_latched
            ? "REVERSED_WIRE_CHUNK_LATCHED_MARKER"
            : "REVERSED_WIRE_CHUNK_DID_NOT_LATCH_MARKER";
    const char *verdict =
        (marker_latched && state_after_header) ? "PASS" : "FAIL";

    printf("BYTE05_WIRE_ORDER mode=reversed_chunks marker_pc=%04X "
           "expect_state=LD_STREAM_WRAM got_state=%02X/%s "
           "cpu_buf_pc=%04X loader_pc=%04X flags=0x%02X verdict=%s reason=%s\n",
           (unsigned)kByte05MarkerPc,
           (unsigned)r.state,
           Stage2LoaderStateName(r.state),
           (unsigned)r.cpu_buf_pc,
           (unsigned)r.loader_pc,
           (unsigned)r.flags,
           verdict,
           reason);
    printf("BYTE05_PRIMARY_BRANCH %s\n",
           marker_latched
               ? "STATE_DATA_PAYLOAD_BYTE_ORDER_REVERSED"
               : "HEADER_NOT_LATCHED_AFTER_REVERSED_WIRE_ORDER");
    printf("PRODUCT_VERDICT FAIL\n");

    (void)LadderResetLoader("byte05 wire-order final cleanup");
    return true;
}

static bool RunLoadR0WireOrderByte05(void)
{
    if (Stage2ProbeRefuseIfStuck("load-r0-wire-order-byte05")) {
        return false;
    }
    printf("Stage2Probe: START mode=load-r0-wire-order-byte05 "
           "protocol=stage2-single-byte-begin marker_pc=%04X "
           "diag_map=accepted_byte05 wire_order=reversed_chunks "
           "fpga_unchanged=yes\n",
           (unsigned)kByte05MarkerPc);
    const bool ok = RunWithUartOwner(RunLoadR0WireOrderByte05Inner);
    printf("Stage2Probe: RESULT %s mode=load-r0-wire-order-byte05\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunLoadR0OffsetPatternByte05Inner(void)
{
    uint8_t diag[8];
    Byte05Result_t r;
    memset(&r, 0, sizeof(r));

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        printf("BYTE05_PATTERN verdict=FAIL reason=PRE_FLIGHT_END_SESSION_FAIL\n");
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD byte05 offset-pattern case")) {
        printf("BYTE05_PATTERN verdict=FAIL reason=BEGIN_LOAD_FAIL\n");
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    if (!Byte05SendOffsetPatternR0()) {
        printf("BYTE05_PATTERN verdict=FAIL reason=R0_STREAM_FAIL\n");
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }
    if (!SendEndSessionClear("END_SESSION byte05 offset-pattern LOAD close")) {
        Stage2ProbeMarkStuck("byte05 offset-pattern load close");
        printf("BYTE05_PATTERN verdict=FAIL reason=END_SESSION_FAIL\n");
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    if (!LadderReadDiagOnce(diag)) {
        printf("BYTE05_PATTERN verdict=FAIL reason=DIAG_READ_FAIL\n");
        printf("BYTE05_PRIMARY_BRANCH BLOCKED_BY_HARDWARE\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    Byte05DecodeDiag(diag, &r);
    printf("BYTE05_PATTERN mode=offset_value_0x80_plus_index "
           "expect_normal_pc=9392 expect_reversed_chunk_pc=9495 "
           "got_state=%02X/%s cpu_buf_pc=%04X loader_pc=%04X "
           "flags=0x%02X raw_diag=%02X_%02X_%02X_%02X_%02X_%02X_%02X_%02X\n",
           (unsigned)r.state,
           Stage2LoaderStateName(r.state),
           (unsigned)r.cpu_buf_pc,
           (unsigned)r.loader_pc,
           (unsigned)r.flags,
           diag[0], diag[1], diag[2], diag[3],
           diag[4], diag[5], diag[6], diag[7]);
    if (r.cpu_buf_pc == 0x9392u) {
        printf("BYTE05_PRIMARY_BRANCH STATE_DATA_PAYLOAD_ORDER_NORMAL\n");
    } else if (r.cpu_buf_pc == 0x9495u) {
        printf("BYTE05_PRIMARY_BRANCH STATE_DATA_PAYLOAD_ORDER_REVERSED_CHUNK\n");
    } else if (r.cpu_buf_pc == 0x0000u) {
        printf("BYTE05_PRIMARY_BRANCH HEADER_PAYLOAD_NOT_REACHING_DIAG_BYTES\n");
    } else {
        printf("BYTE05_PRIMARY_BRANCH HEADER_PAYLOAD_OFFSET_MAP_UNKNOWN\n");
    }
    printf("BYTE05_POSTCONDITION loader_left_waiting_for_wram=yes "
           "requires_board_reset_before_next_load_probe=yes\n");
    printf("PRODUCT_VERDICT FAIL\n");
    return true;
}

static bool RunLoadR0OffsetPatternByte05(void)
{
    if (Stage2ProbeRefuseIfStuck("load-r0-offset-pattern-byte05")) {
        return false;
    }
    printf("Stage2Probe: START mode=load-r0-offset-pattern-byte05 "
           "protocol=stage2-single-byte-begin diag_map=accepted_byte05 "
           "pattern=byte_value_0x80_plus_offset fpga_unchanged=yes "
           "no_wram=yes no_hram=yes no_commit=yes no_loader_reset=yes "
           "one_shot_requires_idle_loader=yes\n");
    const bool ok = RunWithUartOwner(RunLoadR0OffsetPatternByte05Inner);
    printf("Stage2Probe: RESULT %s mode=load-r0-offset-pattern-byte05\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

/* ============================================================================
 *  BYTEWISE R0 offset-pattern probe (2026-05-17)
 *
 *  Same R0 payload, same diag readback, same boundaries as
 *  load-r0-offset-pattern-byte05.  ONLY difference is host packetization:
 *  each STATE_DATA packet carries exactly one state byte.
 *
 *  Hypothesis: the accepted FPGA bridge holds state_byte_valid high
 *  continuously across one DATA packet, so the hclk-side rising-edge
 *  detector sees one byte pulse per DATA packet -- not one per state byte.
 *  If that is the only thing wrong, sending 1 state byte per DATA packet
 *  should let the loader walk the header in order and latch cpu_buf_pc.
 *
 *  PASS branch (BYTEWISE_R0_PASS_NORMAL):
 *    cpu_buf_pc == 9392 means bytes consumed in order.  Byte-pulse root
 *    cause is confirmed.  This is foundation PASS, NOT product PASS.
 *
 *  PASS-with-order-bug branch (BYTEWISE_R0_PASS_REVERSED_OR_ORDER_BUG):
 *    cpu_buf_pc == 9495 means the loader did consume one byte per
 *    packet, but byte order inside the header is still wrong (e.g. the
 *    secondary rx_data -> pump_buf byte-mapping bug in the accepted
 *    bridge).  Byte-pulse delivery is good but byte ordering still
 *    needs work; continue in LOAD byte/header layer, not PnR.
 *
 *  FAIL branches:
 *    BYTEWISE_R0_STILL_HEADER_NOT_LATCHED -- still in LD_WALK_HEADER with
 *      cpu_buf_pc=0000; the byte-pulse explanation is incomplete and the
 *      next step stays in LOAD session/header/packet framing.
 *    BYTEWISE_R0_OTHER_ALIGNMENT_FAIL -- some other shape; LOAD header
 *      mapping is still unclear and product transport is not yet fixed.
 *
 *  Product verdict is FAIL on every branch.  This probe ONLY validates
 *  the byte-pulse foundation; full product PASS requires the full
 *  capture/restore + visual + audio gates.
 * ========================================================================== */

static bool RunLoadR0OffsetPatternBytewiseByte05Inner(void)
{
    uint8_t diag[8];
    Byte05Result_t r;
    memset(&r, 0, sizeof(r));

    printf("BYTEWISE_R0_BEGIN\n");
    printf("BYTEWISE_R0_EXPECT expect_normal_pc=9392 expect_reversed_chunk_pc=9495\n");

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        printf("BYTEWISE_R0_RESULT state=NA/NA cpu_buf_pc=0000 loader_pc=0000 "
               "flags=0x00 raw_diag=00_00_00_00_00_00_00_00\n");
        printf("BYTEWISE_R0_CLASSIFICATION BLOCKED_BY_HARDWARE_END_SESSION_FAIL\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD bytewise R0 offset-pattern")) {
        printf("BYTEWISE_R0_RESULT state=NA/NA cpu_buf_pc=0000 loader_pc=0000 "
               "flags=0x00 raw_diag=00_00_00_00_00_00_00_00\n");
        printf("BYTEWISE_R0_CLASSIFICATION BLOCKED_BY_HARDWARE_BEGIN_LOAD_FAIL\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    if (!Byte05SendOffsetPatternR0Bytewise()) {
        printf("BYTEWISE_R0_RESULT state=NA/NA cpu_buf_pc=0000 loader_pc=0000 "
               "flags=0x00 raw_diag=00_00_00_00_00_00_00_00\n");
        printf("BYTEWISE_R0_CLASSIFICATION BLOCKED_BY_HARDWARE_R0_STREAM_FAIL\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }
    if (!SendEndSessionClear("END_SESSION bytewise R0 offset-pattern LOAD close")) {
        Stage2ProbeMarkStuck("bytewise R0 offset-pattern load close");
        printf("BYTEWISE_R0_RESULT state=NA/NA cpu_buf_pc=0000 loader_pc=0000 "
               "flags=0x00 raw_diag=00_00_00_00_00_00_00_00\n");
        printf("BYTEWISE_R0_CLASSIFICATION BLOCKED_BY_HARDWARE_END_SESSION_FAIL\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    if (!LadderReadDiagOnce(diag)) {
        printf("BYTEWISE_R0_RESULT state=NA/NA cpu_buf_pc=0000 loader_pc=0000 "
               "flags=0x00 raw_diag=00_00_00_00_00_00_00_00\n");
        printf("BYTEWISE_R0_CLASSIFICATION BLOCKED_BY_HARDWARE_DIAG_READ_FAIL\n");
        printf("PRODUCT_VERDICT FAIL\n");
        return false;
    }

    Byte05DecodeDiag(diag, &r);
    printf("BYTEWISE_R0_RESULT state=%02X/%s cpu_buf_pc=%04X loader_pc=%04X "
           "flags=0x%02X raw_diag=%02X_%02X_%02X_%02X_%02X_%02X_%02X_%02X\n",
           (unsigned)r.state,
           Stage2LoaderStateName(r.state),
           (unsigned)r.cpu_buf_pc,
           (unsigned)r.loader_pc,
           (unsigned)r.flags,
           diag[0], diag[1], diag[2], diag[3],
           diag[4], diag[5], diag[6], diag[7]);

    if (r.cpu_buf_pc == 0x9392u) {
        printf("BYTEWISE_R0_CLASSIFICATION BYTEWISE_R0_PASS_NORMAL\n");
    } else if (r.cpu_buf_pc == 0x9495u) {
        printf("BYTEWISE_R0_CLASSIFICATION BYTEWISE_R0_PASS_REVERSED_OR_ORDER_BUG\n");
    } else if (r.state == 0x03u && r.cpu_buf_pc == 0x0000u) {
        printf("BYTEWISE_R0_CLASSIFICATION BYTEWISE_R0_STILL_HEADER_NOT_LATCHED\n");
    } else {
        printf("BYTEWISE_R0_CLASSIFICATION BYTEWISE_R0_OTHER_ALIGNMENT_FAIL\n");
    }
    printf("PRODUCT_VERDICT FAIL\n");
    return true;
}

static bool RunLoadR0OffsetPatternBytewiseByte05(void)
{
    if (Stage2ProbeRefuseIfStuck("load-r0-offset-pattern-bytewise-byte05")) {
        return false;
    }
    printf("Stage2Probe: START mode=load-r0-offset-pattern-bytewise-byte05 "
           "protocol=stage2-single-byte-begin diag_map=accepted_byte05 "
           "pattern=byte_value_0x80_plus_offset fpga_unchanged=yes "
           "no_wram=yes no_hram=yes no_commit=yes no_loader_reset=yes "
           "packetization=one_state_byte_per_data_packet "
           "data_packets_per_r0=%u\n",
           (unsigned)kStage2R0Bytes);
    const bool ok = RunWithUartOwner(RunLoadR0OffsetPatternBytewiseByte05Inner);
    printf("Stage2Probe: RESULT %s mode=load-r0-offset-pattern-bytewise-byte05\n",
           ok ? "PASS" : "FAIL");
    return ok;
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

static bool RunBeginLoadWriteR0AckOnly(const char *mode, uint32_t round)
{
    FusionV2Frame_t begin;

    printf("Stage2Probe: ACK-ONLY round=%lu mode=%s begin_load\n",
           (unsigned long)round,
           mode);
    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD_STAGE2")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    if (!FusionSavestate_BuildWriteStreamBegin(kStage2Regions[0].region,
                                               0u,
                                               kStage2Regions[0].length,
                                               &begin)) {
        printf("Stage2Probe: build WRITE_STREAM_BEGIN %s failed\n",
               kStage2Regions[0].name);
        return false;
    }

    printf("Stage2Probe: ACK-ONLY round=%lu mode=%s WRITE_STREAM_BEGIN "
           "%s id=0x%02X length=%u no_data=yes no_commit=yes\n",
           (unsigned long)round,
           mode,
           kStage2Regions[0].name,
           (unsigned)kStage2Regions[0].region,
           (unsigned)kStage2Regions[0].length);
    if (!SendFrame("WRITE_STREAM_BEGIN", &begin, false)) {
        return false;
    }
    return ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                     (uint8_t)kFusionOp_WriteStreamBegin,
                     "WRITE_STREAM_BEGIN ack");
}

static bool RunAckLoadLoopInner(void)
{
    bool ok = true;

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        return false;
    }
    DrainRxForMs(kStage2PreflightDrainMs);

    for (uint32_t i = 0u; i < s_stage2_ack_load_loop_count; ++i) {
        const uint32_t round = i + 1u;
        printf("Stage2Probe: ACK-LOAD-LOOP round=%lu/%lu\n",
               (unsigned long)round,
               (unsigned long)s_stage2_ack_load_loop_count);
        ok = RunBeginLoadWriteR0AckOnly("ack-load-loop", round);
        if (!EndSessionCleanup()) {
            printf("Stage2Probe: ACK-LOAD-LOOP cleanup failed round=%lu; "
                   "treating command as FAIL\n",
                   (unsigned long)round);
            return false;
        }
        DrainRxForMs(kStage2PreflightDrainMs);
        if (!ok) {
            printf("Stage2Probe: ACK-LOAD-LOOP stopping after failed ACK "
                   "round=%lu\n",
                   (unsigned long)round);
            return false;
        }
    }

    return true;
}

static bool RunAckAfterCaptureInner(void)
{
    bool ok = false;

    if (!RunCaptureInner()) {
        return false;
    }

    printf("Stage2Probe: ACK-AFTER-CAPTURE wait_s=%lu\n",
           (unsigned long)s_stage2_ack_after_capture_delay_s);
    for (uint32_t remaining = s_stage2_ack_after_capture_delay_s;
         remaining > 0u;
         --remaining) {
        if ((remaining == s_stage2_ack_after_capture_delay_s) ||
            (remaining <= 5u) ||
            ((remaining % 10u) == 0u)) {
            printf("Stage2Probe: ACK-AFTER-CAPTURE remaining_s=%lu\n",
                   (unsigned long)remaining);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("Stage2Probe: ACK-AFTER-CAPTURE starting load ACK-only probe\n");

    DrainRxForMs(kStage2PreflightDrainMs);
    ok = RunBeginLoadWriteR0AckOnly("ack-after-capture", 1u);
    if (!EndSessionCleanup()) {
        printf("Stage2Probe: ACK-AFTER-CAPTURE final cleanup failed; "
               "treating command as FAIL\n");
        ok = false;
    }
    return ok;
}

static bool DrainUntilEndAck(const char *context)
{
    const int64_t deadline = esp_timer_get_time()
                           + ((int64_t)kStage2AbortDrainBudgetMs * 1000);
    while (esp_timer_get_time() < deadline) {
        FusionV2Decoded_t d;
        if (!ReadDecodedFrame(context, &d)) {
            break;
        }
        if (IsEndSessionAck(&d)) {
            printf("Stage2Probe: %s END_SESSION ACK received mid-stream\n",
                   context);
            return true;
        }
        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_Error) {
            printf("Stage2Probe: %s FPGA ERROR while draining "
                   "code=0x%02X detail=0x%02X\n",
                   context, d.error_code, d.error_detail);
            return false;
        }
        if (d.addr == (uint8_t)kFusionAddr_StateData) {
            printf("Stage2Probe: %s discarding stale DATA seq=%u len=%u\n",
                   context, (unsigned)d.data_seq, (unsigned)d.data_len);
            continue;
        }
        printf("Stage2Probe: %s discarding unexpected frame "
               "addr=0x%02X op=0x%02X ack_for=0x%02X\n",
               context, d.addr, d.ctl_opcode, d.ack_for_opcode);
    }
    printf("Stage2Probe: %s did NOT receive END_SESSION ACK within %u ms\n",
           context, (unsigned)kStage2AbortDrainBudgetMs);
    return false;
}

static bool RunAbortReadInner(void)
{
    bool ok = false;
    const Stage2Region_t *region = &kStage2Regions[1]; /* WRAM */
    uint32_t received = 0u;
    uint32_t packets = 0u;
    uint16_t expected_seq = 0u;
    bool pending_data_valid = false;
    FusionV2Decoded_t pending_data;
    memset(&pending_data, 0, sizeof(pending_data));

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

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildReadStreamBegin(region->region,
                                              0u,
                                              region->length,
                                              &begin)) {
        printf("Stage2Probe: build READ_STREAM_BEGIN %s failed\n",
               region->name);
        goto done;
    }
    printf("Stage2Probe: abort-read BEGIN REGION %s id=0x%02X "
           "length=%u abort_after=%u\n",
           region->name,
           (unsigned)region->region,
           (unsigned)region->length,
           (unsigned)kStage2AbortReadPackets);
    if (!SendFrame("READ_STREAM_BEGIN", &begin, false)) {
        goto done;
    }
    if (!ExpectStreamAckOrFirstData("READ_STREAM_BEGIN ack",
                                    (uint8_t)kFusionOp_ReadStreamBegin,
                                    &pending_data_valid,
                                    &pending_data)) {
        goto done;
    }

    while (packets < (uint32_t)kStage2AbortReadPackets) {
        FusionV2Decoded_t d;
        if (pending_data_valid) {
            d = pending_data;
            pending_data_valid = false;
        } else if (!ReadDecodedFrame("abort-read DATA", &d)) {
            goto done;
        }
        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_Error) {
            printf("Stage2Probe: abort-read FPGA ERROR code=0x%02X detail=0x%02X\n",
                   d.error_code, d.error_detail);
            goto done;
        }
        if (d.addr != (uint8_t)kFusionAddr_StateData) {
            printf("Stage2Probe: abort-read expected DATA got addr=0x%02X op=0x%02X\n",
                   d.addr, d.ctl_opcode);
            goto done;
        }
        if (d.data_seq != expected_seq) {
            printf("Stage2Probe: abort-read seq mismatch got=%u expected=%u\n",
                   (unsigned)d.data_seq, (unsigned)expected_seq);
            goto done;
        }
        received += d.data_len;
        packets++;
        expected_seq = (uint16_t)(expected_seq + 1u);

        if (packets < (uint32_t)kStage2AbortReadPackets) {
            FusionV2Frame_t cont;
            if (!FusionSavestate_BuildReadStreamContinue(expected_seq,
                                                         &cont)) {
                printf("Stage2Probe: build READ_STREAM_CONTINUE failed seq=%u\n",
                       (unsigned)expected_seq);
                goto done;
            }
            if (!SendFrame("READ_STREAM_CONTINUE", &cont, true)) {
                goto done;
            }
            if (!ExpectStreamAckOrFirstData("READ_STREAM_CONTINUE ack",
                                            (uint8_t)kFusionOp_ReadStreamContinue,
                                            &pending_data_valid,
                                            &pending_data)) {
                goto done;
            }
        }
    }

    printf("Stage2Probe: abort-read got packets=%u bytes=%u; "
           "injecting END_SESSION mid-stream\n",
           (unsigned)packets, (unsigned)received);

    {
        FusionV2Frame_t end_frame;
        if (!FusionSavestate_BuildEndSession(&end_frame)) {
            printf("Stage2Probe: build END_SESSION (abort) failed\n");
            goto done;
        }
        if (!SendFrame("END_SESSION (abort-read)", &end_frame, false)) {
            goto done;
        }
    }

    if (!DrainUntilEndAck("abort-read")) {
        goto done;
    }

    ok = true;

done:
    if (!EndSessionCleanup()) {
        printf("Stage2Probe: ABORT-READ final cleanup failed; "
               "treating command as FAIL\n");
        ok = false;
    }
    return ok;
}

static bool RunAbortLoadInner(void)
{
    bool ok = false;
    const Stage2Region_t *region = &kStage2Regions[0]; /* R0 */
    static const uint8_t kAbortPad[8] = {
        0x46u, 0x55u, 0x53u, 0x53u, 0x01u, 0x00u, 0x00u, 0x00u
    };

    DrainRxForMs(kStage2PreflightDrainMs);
    if (!EndSessionCleanup()) {
        goto done;
    }
    DrainRxForMs(kStage2PreflightDrainMs);

    if (!SendStage2BeginAndExpectAck((uint8_t)kFusionOp_BeginLoad,
                                     "BEGIN_LOAD_STAGE2")) {
        goto done;
    }
    vTaskDelay(pdMS_TO_TICKS(kStage2BeginLoadSettleMs));

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildWriteStreamBegin(region->region,
                                               0u,
                                               region->length,
                                               &begin)) {
        printf("Stage2Probe: build WRITE_STREAM_BEGIN %s failed\n",
               region->name);
        goto done;
    }
    printf("Stage2Probe: abort-load WRITE REGION %s id=0x%02X "
           "length=%u abort_after=%u packets\n",
           region->name,
           (unsigned)region->region,
           (unsigned)region->length,
           (unsigned)kStage2AbortLoadPackets);
    if (!SendFrame("WRITE_STREAM_BEGIN", &begin, false)) {
        goto done;
    }
    if (!ExpectCtl((uint8_t)kFusionOp_AckAccepted,
                   (uint8_t)kFusionOp_WriteStreamBegin,
                   "WRITE_STREAM_BEGIN ack")) {
        goto done;
    }

    {
        uint16_t seq = 0u;
        for (uint8_t i = 0u;
             i < (uint8_t)kStage2AbortLoadPackets;
             ++i) {
            FusionV2Frame_t data_frame;
            if (!FusionSavestate_BuildStateData(seq,
                                                kAbortPad,
                                                sizeof(kAbortPad),
                                                &data_frame)) {
                printf("Stage2Probe: build STATE_DATA failed seq=%u\n",
                       (unsigned)seq);
                goto done;
            }
            if (!SendFrame("STATE_DATA", &data_frame, true)) {
                goto done;
            }
            seq = (uint16_t)(seq + 1u);
        }
        printf("Stage2Probe: abort-load sent %u DATA packets; "
               "injecting END_SESSION mid-stream\n",
               (unsigned)kStage2AbortLoadPackets);
    }

    vTaskDelay(pdMS_TO_TICKS(kStage2AbortLoadProcessMs));

    {
        FusionV2Frame_t end_frame;
        if (!FusionSavestate_BuildEndSession(&end_frame)) {
            printf("Stage2Probe: build END_SESSION (abort) failed\n");
            goto done;
        }
        if (!SendFrame("END_SESSION (abort-load)", &end_frame, false)) {
            goto done;
        }
    }

    if (!DrainUntilEndAck("abort-load")) {
        goto done;
    }

    ok = true;

done:
    if (!EndSessionCleanup()) {
        printf("Stage2Probe: ABORT-LOAD final cleanup failed; "
               "treating command as FAIL\n");
        ok = false;
    }
    return ok;
}

static bool RunClearInner(void)
{
    DrainRxForMs(kStage2PreflightDrainMs);
    if (EndSessionCleanup()) {
        s_stage2_fpga_stuck = false;
        return true;
    }
    return false;
}

static bool RunSaveReadback(void)
{
    if (Stage2ProbeRefuseIfStuck("save")) {
        return false;
    }
    printf("Stage2Probe: START mode=save protocol=stage2-single-byte-begin\n");
    const bool ok = RunWithUartOwner(RunSaveReadbackInner);
    printf("Stage2Probe: RESULT %s mode=save\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunClear(void)
{
    /* Intentionally NOT guarded by Stage2ProbeRefuseIfStuck -- clear is the
       recovery command and must always run.  Successful clear is what
       releases the stuck latch. */
    printf("Stage2Probe: START mode=clear protocol=stage2-single-byte-begin end_session_only=yes\n");
    const bool ok = RunWithUartOwner(RunClearInner);
    printf("Stage2Probe: RESULT %s mode=clear\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunLoadAck(void)
{
    if (Stage2ProbeRefuseIfStuck("load-ack")) {
        return false;
    }
    printf("Stage2Probe: START mode=load-ack protocol=stage2-single-byte-begin no_commit=yes\n");
    const bool ok = RunWithUartOwner(RunLoadAckInner);
    printf("Stage2Probe: RESULT %s mode=load-ack\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunAckLoadLoop(uint32_t count)
{
    if (Stage2ProbeRefuseIfStuck("ack-load-loop")) {
        return false;
    }
    s_stage2_ack_load_loop_count = count;
    printf("Stage2Probe: START mode=ack-load-loop "
           "protocol=stage2-single-byte-begin count=%lu "
           "r0_length=%u no_data=yes no_commit=yes cleanup_each_round=yes\n",
           (unsigned long)count,
           (unsigned)kStage2Regions[0].length);
    const bool ok = RunWithUartOwner(RunAckLoadLoopInner);
    printf("Stage2Probe: RESULT %s mode=ack-load-loop\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunAckAfterCapture(uint32_t delay_s)
{
    if (Stage2ProbeRefuseIfStuck("ack-after-capture")) {
        return false;
    }
    s_stage2_ack_after_capture_delay_s = delay_s;
    printf("Stage2Probe: START mode=ack-after-capture "
           "protocol=stage2-single-byte-begin delay_s=%lu "
           "save_regions=R0_WRAM_HRAM load_probe=WRITE_STREAM_BEGIN_R0 "
           "no_data=yes no_commit=yes\n",
           (unsigned long)delay_s);
    const bool ok = RunWithUartOwner(RunAckAfterCaptureInner);
    printf("Stage2Probe: RESULT %s mode=ack-after-capture\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunCapture(void)
{
    if (Stage2ProbeRefuseIfStuck("capture")) {
        return false;
    }
    printf("Stage2Probe: START mode=capture protocol=stage2-single-byte-begin store_in_ram=yes\n");
    const bool ok = RunWithUartOwner(RunCaptureInner);
    printf("Stage2Probe: RESULT %s mode=capture\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunRestoreCaptured(void)
{
    if (Stage2ProbeRefuseIfStuck("restore")) {
        return false;
    }
    printf("Stage2Probe: START mode=restore protocol=stage2-single-byte-begin commit=yes\n");
    const bool ok = RunWithUartOwner(RunRestoreCapturedInner);
    printf("Stage2Probe: RESULT %s mode=restore\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunCaptureRestore(uint32_t delay_s)
{
    if (Stage2ProbeRefuseIfStuck("capture-restore")) {
        return false;
    }
    s_stage2_capture_restore_delay_s = delay_s;
    printf("Stage2Probe: START mode=capture-restore "
           "protocol=stage2-single-byte-begin delay_s=%lu commit=yes\n",
           (unsigned long)delay_s);
    const bool ok = RunWithUartOwner(RunCaptureRestoreInner);
    printf("Stage2Probe: RESULT %s mode=capture-restore\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunCaptureRestoreBytewiseFull(uint32_t delay_s)
{
    if (Stage2ProbeRefuseIfStuck("capture-restore-bytewise-full")) {
        return false;
    }
    s_stage2_capture_restore_delay_s = delay_s;
    printf("Stage2Probe: START mode=capture-restore-bytewise-full "
           "protocol=stage2-single-byte-begin delay_s=%lu commit=yes "
           "max_chunk=1\n",
           (unsigned long)delay_s);
    const bool ok = RunWithUartOwner(RunCaptureRestoreBytewiseFullInner);
    printf("Stage2Probe: RESULT %s mode=capture-restore-bytewise-full\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunCaptureRestoreHoldContractTrace(uint32_t delay_s)
{
    if (Stage2ProbeRefuseIfStuck("capture-restore-hold-contract-trace")) {
        return false;
    }
    s_stage2_capture_restore_delay_s = delay_s;
    printf("Stage2Probe: START mode=capture-restore-hold-contract-trace "
           "protocol=stage2-single-byte-begin delay_s=%lu commit=hold "
           "diag_region=0x7E diag_bytes=0..5 byte67=NA "
           "poll_ms=0,5,20,100,500,1000,2000\n",
           (unsigned long)delay_s);
    const bool ok = RunWithUartOwner(RunCaptureRestoreHoldContractTraceInner);
    printf("Stage2Probe: RESULT %s mode=capture-restore-hold-contract-trace\n",
           ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunAbortRead(void)
{
    if (Stage2ProbeRefuseIfStuck("abort-read")) {
        return false;
    }
    printf("Stage2Probe: START mode=abort-read protocol=stage2-single-byte-begin "
           "end_session_during=read_stream\n");
    const bool ok = RunWithUartOwner(RunAbortReadInner);
    printf("Stage2Probe: RESULT %s mode=abort-read\n", ok ? "PASS" : "FAIL");
    return ok;
}

static bool RunAbortLoad(void)
{
    if (Stage2ProbeRefuseIfStuck("abort-load")) {
        return false;
    }
    printf("Stage2Probe: START mode=abort-load protocol=stage2-single-byte-begin "
           "end_session_during=write_stream\n");
    const bool ok = RunWithUartOwner(RunAbortLoadInner);
    printf("Stage2Probe: RESULT %s mode=abort-load\n", ok ? "PASS" : "FAIL");
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

static bool ParseLoopCount(const char *arg, uint32_t *out)
{
    char *end = NULL;
    unsigned long value = 0u;
    if (arg == NULL || out == NULL || arg[0] == '\0') {
        return false;
    }
    value = strtoul(arg, &end, 10);
    if (end == arg || *end != '\0' || value == 0u ||
        value > (unsigned long)kStage2AckLoadLoopMaxCount) {
        return false;
    }
    *out = (uint32_t)value;
    return true;
}

static bool ParseRawTailLength(const char *arg, uint32_t *out)
{
    char *end = NULL;
    unsigned long value = 0u;
    if (arg == NULL || out == NULL || arg[0] == '\0') {
        return false;
    }
    value = strtoul(arg, &end, 10);
    if (end == arg || *end != '\0' ||
        value > (unsigned long)kStage2RxRawTailMaxBytes) {
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
    if (strcmp(argv[1], "ack-load-loop") == 0) {
        uint32_t count = 0u;
        if (argc < 3 || !ParseLoopCount(argv[2], &count)) {
            printf("Usage: stage2probe ack-load-loop <1-%u>\n",
                   (unsigned)kStage2AckLoadLoopMaxCount);
            return 1;
        }
        return RunAckLoadLoop(count) ? 0 : 1;
    }
    if (strcmp(argv[1], "ack-after-capture") == 0) {
        uint32_t delay_s = 0u;
        if (argc < 3 || !ParseDelaySeconds(argv[2], &delay_s)) {
            printf("Usage: stage2probe ack-after-capture <0-%u seconds>\n",
                   (unsigned)kStage2CaptureRestoreMaxDelayS);
            return 1;
        }
        return RunAckAfterCapture(delay_s) ? 0 : 1;
    }
    if (strcmp(argv[1], "raw-tail") == 0) {
        uint32_t length = kStage2RxRawTailDefaultBytes;
        if (argc >= 3 && !ParseRawTailLength(argv[2], &length)) {
            printf("Usage: stage2probe raw-tail [0-%u]\n",
                   (unsigned)kStage2RxRawTailMaxBytes);
            return 1;
        }
        Stage2RawTapDumpTail("manual", length);
        return 0;
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
    if (strcmp(argv[1], "capture-restore-bytewise-full") == 0) {
        uint32_t delay_s = 0u;
        if (argc < 3 || !ParseDelaySeconds(argv[2], &delay_s)) {
            printf("Usage: stage2probe capture-restore-bytewise-full <0-%u seconds>\n",
                   (unsigned)kStage2CaptureRestoreMaxDelayS);
            return 1;
        }
        return RunCaptureRestoreBytewiseFull(delay_s) ? 0 : 1;
    }
    if (strcmp(argv[1], "capture-restore-hold-contract-trace") == 0) {
        uint32_t delay_s = 0u;
        if (argc < 3 || !ParseDelaySeconds(argv[2], &delay_s)) {
            printf("Usage: stage2probe capture-restore-hold-contract-trace <0-%u seconds>\n",
                   (unsigned)kStage2CaptureRestoreMaxDelayS);
            return 1;
        }
        return RunCaptureRestoreHoldContractTrace(delay_s) ? 0 : 1;
    }
    if (strcmp(argv[1], "abort-read") == 0) {
        return RunAbortRead() ? 0 : 1;
    }
    if (strcmp(argv[1], "abort-load") == 0) {
        return RunAbortLoad() ? 0 : 1;
    }
    if (strcmp(argv[1], "load-progress-ladder") == 0) {
        return RunLoadProgressLadder() ? 0 : 1;
    }
    if (strcmp(argv[1], "load-state-ladder-byte05") == 0) {
        return RunLoadStateLadderByte05() ? 0 : 1;
    }
    if (strcmp(argv[1], "load-r0-wire-order-byte05") == 0) {
        return RunLoadR0WireOrderByte05() ? 0 : 1;
    }
    if (strcmp(argv[1], "load-r0-offset-pattern-byte05") == 0) {
        return RunLoadR0OffsetPatternByte05() ? 0 : 1;
    }
    if (strcmp(argv[1], "load-r0-offset-pattern-bytewise-byte05") == 0) {
        return RunLoadR0OffsetPatternBytewiseByte05() ? 0 : 1;
    }

    printf("Usage: stage2probe [save|load-ack|ack-load-loop|ack-after-capture|raw-tail|clear|capture|restore|capture-restore|capture-restore-bytewise-full|capture-restore-hold-contract-trace|abort-read|abort-load|load-progress-ladder|load-state-ladder-byte05|load-r0-wire-order-byte05|load-r0-offset-pattern-byte05|load-r0-offset-pattern-bytewise-byte05]\n");
    printf("  save        : BEGIN_SAVE, read R0/WRAM/HRAM, print length + CRC; no LOAD\n");
    printf("  load-ack    : BEGIN_LOAD ACK smoke only; no WRITE_STREAM, no COMMIT\n");
    printf("  ack-load-loop <n> : repeat BEGIN_LOAD + WRITE_STREAM_BEGIN R0 ACK-only; no DATA, no COMMIT\n");
    printf("  ack-after-capture <seconds> : capture R0/WRAM/HRAM, wait, then WRITE_STREAM_BEGIN R0 ACK-only; no DATA, no COMMIT\n");
    printf("  raw-tail [n] : dump recent MCU UART RX raw bytes; no UART owner, no FPGA command\n");
    printf("  clear       : END_SESSION cleanup only; clears MCU stuck latch on PASS\n");
    printf("  capture     : BEGIN_SAVE, read R0/WRAM/HRAM into MCU RAM; no LOAD\n");
    printf("  restore     : replay captured R0/WRAM/HRAM through BEGIN_LOAD, WRITE_STREAM, WRITE_COMMIT\n");
    printf("  capture-restore <seconds> : capture, wait, restore+WRITE_COMMIT in one console command\n");
    printf("  capture-restore-bytewise-full <seconds> : capture, wait, restore+WRITE_COMMIT with all regions (R0/WRAM/HRAM) sent at one state byte per STATE_DATA packet; host-only foundation path from 2026-05-17 byte-pulse root cause; no FPGA RTL change\n");
    printf("  capture-restore-hold-contract-trace <seconds> : capture, wait, restore with HOLD flag, poll region 0x7E, print final classification\n");
    printf("  abort-read  : BEGIN_SAVE + READ_STREAM_BEGIN WRAM, send END_SESSION mid-stream; PASS on END ACK + released game\n");
    printf("  abort-load  : BEGIN_LOAD + WRITE_STREAM_BEGIN R0, send END_SESSION mid-stream; PASS on END ACK + released game\n");
    printf("  load-progress-ladder : ladder of partial LOAD streams; print one LADDER_CASE line per case, then LADDER_FIRST_FAIL / LADDER_PRIMARY_BRANCH / PRODUCT_VERDICT FAIL\n");
    printf("  load-state-ladder-byte05 : four-case state ladder using accepted 0x7E byte05 map (marker R0 PC=A55A); print one BYTE05_CASE per case (plus BYTE05_POLL for COMMIT_HOLD), then BYTE05_FIRST_FAIL / BYTE05_PRIMARY_BRANCH / PRODUCT_VERDICT FAIL\n");
    printf("  load-r0-wire-order-byte05 : MCU-only proof probe for accepted FPGA byte order; sends R0 marker with reversed 8-byte STATE_DATA chunks and reads accepted 0x7E byte05 map\n");
    printf("  load-r0-offset-pattern-byte05 : one-shot non-WRAM R0 pattern probe; sends only R0 bytes, no WRAM/HRAM/commit/reset, then reads accepted 0x7E byte05 map\n");
    printf("  load-r0-offset-pattern-bytewise-byte05 : same R0 offset pattern, but each STATE_DATA packet carries exactly one state byte; tests whether bridge byte-pulse foundation can be unblocked without FPGA RTL change\n");
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
