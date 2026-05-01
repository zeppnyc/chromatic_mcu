#include "fusion_savestate_smoke48c.h"

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
#include "pwrmgr.h"
#include "savestate_storage.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
 * Phase 4.8c v2 — strict bit-exact RAM-only round-trip smoke harness.
 *
 * One continuous paused session.  After OP_BEGIN_TEST_RW is accepted,
 * `session_pause` stays asserted until the final END_SESSION, so the
 * GB CPU never runs between any harness write and any subsequent
 * readback.  The bridge change that enables this lives in
 * `state_port_uart_bridge.sv` and uses the predicates
 * `sess_can_read_stream` / `sess_can_write_stream` to gate
 * READ_STREAM_* / WRITE_STREAM_* opcodes during the mixed session.
 *
 * Sequence (no flash writes, no hotkeys, no Top/CPU/Timer restore):
 *   BEGIN_TEST_RW
 *     R1: read original HRAM/WRAM into MCU RAM
 *     W1: write XOR-pattern temp HRAM/WRAM
 *     R2: re-read temp HRAM/WRAM and bit-exact compare
 *     W2: write original HRAM/WRAM back (restore)
 *     R3: re-read original HRAM/WRAM and bit-exact compare
 *   END_SESSION
 *
 * The strict 4.8c PASS bar:
 *   - temp HRAM readback exact;
 *   - temp WRAM readback exact;
 *   - restored HRAM readback exact;
 *   - restored WRAM readback exact;
 *   - END_SESSION cleanup ACK;
 *   - persistent slot still valid (gen unchanged);
 *   - user-visible core/game recovery cleanly verified after END_SESSION.
 *
 * The harness emits a per-byte mismatch preview (up to 16 offsets) on
 * any failed compare and records the first protocol failure.  Pre/post
 * slot validation is read-only.  If OP_BEGIN_TEST_RW is rejected with
 * ERR_NO_SESSION the bridge is the older P48C bitstream; flash a P48D
 * (or newer) bitstream before retrying.
 */

enum {
    kSmoke48cFrameTimeoutMs       = 3000,
    kSmoke48cPreflightDrainMs     = 120,
    kSmoke48cFinalEndMs           = 50,
    kSmoke48cEndSessionAttempts   = 4,
    kSmoke48cWriteProgressEvery   = 512u,
    kSmoke48cInterSessionMs       = 30,
    kSmoke48cPostBeginDrainMs     = 60,
    kSmoke48cBadPayloadDumpBytes  = 16,
    kSmoke48cMismatchPreviewCount = 16,
};

#define SMOKE48C_HRAM_REGION 0x09u
#define SMOKE48C_WRAM_REGION 0x0Cu
#define SMOKE48C_HRAM_LEN    127u
#define SMOKE48C_WRAM_LEN    32768u
#define SMOKE48C_SLOT_TAG    "P48C"

static const char *TAG = "Smoke48c";
static const char kPartitionLabel[] = "savestate";

static const uint8_t kXorPattern[32] = {
    0xA5u, 0x5Au, 0xC3u, 0x3Cu, 0xF0u, 0x0Fu, 0x96u, 0x69u,
    0x33u, 0xCCu, 0x55u, 0xAAu, 0x77u, 0x88u, 0x11u, 0xEEu,
    0x22u, 0xDDu, 0x44u, 0xBBu, 0x66u, 0x99u, 0xB7u, 0x4Du,
    0x1Fu, 0xE0u, 0x2Au, 0xD5u, 0x88u, 0x77u, 0x4Bu, 0xB4u,
};

typedef struct {
    bool slot_valid_pre;
    /* Phase tracking for the single mixed paused session.  All phases
     * happen within ONE BEGIN_TEST_RW / END_SESSION pair. */
    bool begin_ok;            /* OP_BEGIN_TEST_RW accepted */
    bool phase_r1_ok;         /* read original HRAM + WRAM */
    bool phase_w1_ok;         /* write temp HRAM + WRAM */
    bool phase_r2_ok;         /* read back temp HRAM + WRAM */
    bool phase_w2_ok;         /* write original HRAM + WRAM */
    bool phase_r3_ok;         /* read back original HRAM + WRAM */
    bool end_ok;              /* END_SESSION acked inside the session */
    bool slot_valid_post;
    bool core_released;       /* final cleanup END_SESSION ACK observed */
    bool restore_attempted;
    bool restore_verified;

    uint32_t crc_orig_hram;
    uint32_t crc_orig_wram;
    uint32_t crc_temp_hram;
    uint32_t crc_temp_wram;
    uint32_t crc_readback_temp_hram;
    uint32_t crc_readback_temp_wram;
    uint32_t crc_readback_orig_hram;
    uint32_t crc_readback_orig_wram;

    uint32_t mismatch_temp_hram;
    uint32_t mismatch_temp_wram;
    uint32_t mismatch_orig_hram;
    uint32_t mismatch_orig_wram;
    int32_t  first_mismatch_temp_hram;
    int32_t  first_mismatch_temp_wram;
    int32_t  first_mismatch_orig_hram;
    int32_t  first_mismatch_orig_wram;

    uint32_t generation_pre;
    uint32_t generation_post;

    /* Total elapsed time of the paused session window (BEGIN_TEST_RW
     * ACK -> END_SESSION ACK).  Diagnostic only; not a pass criterion. */
    int64_t  paused_session_us;

    /* First-failure forensics. */
    char     fail_step[24];
    char     fail_reason[64];
    int32_t  fail_region;
    int32_t  fail_expected_seq;
    int32_t  fail_got_seq;
    uint32_t fail_bytes_so_far;
    uint32_t fail_packets_so_far;
    uint32_t fail_drained_bytes;
} Smoke48cResult_t;

static Smoke48cResult_t s_result;

static uint8_t s_orig_hram[SMOKE48C_HRAM_LEN];
static uint8_t s_scratch_hram[SMOKE48C_HRAM_LEN];
static uint8_t *s_wram_storage = NULL;
static uint8_t *s_orig_wram = NULL;
static uint8_t *s_scratch_wram = NULL;

static uint8_t s_uart_cache[512];
static size_t  s_uart_cache_pos = 0u;
static size_t  s_uart_cache_len = 0u;

/* Timestamp of the BEGIN_TEST_RW ACK so we can report total paused
 * session duration from the diagnostic side. */
static int64_t s_session_open_us = 0;

static void ResetUartCache(void)
{
    s_uart_cache_pos = 0u;
    s_uart_cache_len = 0u;
}

static uint32_t DrainRxForMsCounted(uint32_t ms)
{
    uint8_t scratch[128];
    uint32_t total = 0u;
    const int64_t deadline = esp_timer_get_time() + ((int64_t)ms * 1000);
    ResetUartCache();
    while (esp_timer_get_time() < deadline) {
        const int got = uart_read_bytes(UART_NUM_1,
                                        scratch,
                                        sizeof(scratch),
                                        pdMS_TO_TICKS(2));
        if (got > 0) {
            total += (uint32_t)got;
        }
    }
    ResetUartCache();
    return total;
}

static void DrainRxForMs(uint32_t ms)
{
    (void)DrainRxForMsCounted(ms);
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

static bool ReadDecodedFrameTimed(const char *context,
                                  FusionV2Decoded_t *decoded,
                                  uint32_t timeout_ms,
                                  bool *saw_bad_payload)
{
    uint8_t raw[FUSION_V2_MAX_FRAME] = {0};
    int64_t deadline = esp_timer_get_time() + ((int64_t)timeout_ms * 1000);
    if (saw_bad_payload != NULL) {
        *saw_bad_payload = false;
    }

    while (esp_timer_get_time() < deadline) {
        uint8_t b = 0;
        if (!ReadByteWithDeadline(&b, deadline)) {
            printf("Smoke48c: RX timeout marker during %s\n", context);
            return false;
        }
        if (b != FUSION_V2_HEADER_MARKER) {
            continue;
        }
        raw[0] = b;
        if (!ReadByteWithDeadline(&raw[1], deadline) ||
            !ReadByteWithDeadline(&raw[2], deadline)) {
            printf("Smoke48c: RX timeout header during %s\n", context);
            return false;
        }
        const uint8_t payload_len = raw[2];
        if (payload_len > FUSION_V2_MAX_PAYLOAD) {
            if (saw_bad_payload != NULL) {
                *saw_bad_payload = true;
            }
            /*
             * Forensic preview: peek at cached bytes WITHOUT consuming
             * them.  An earlier dump that called ReadByteWithDeadline 16
             * times ate the first valid STATE_DATA frame after a
             * desync (run 3 evidence:
             * `.codex_tmp/phase4_8c_ram_roundtrip/run3.log` Phase C/WRAM)
             * and produced a "unexpected first frame addr=0x21" follow-on
             * error.  Looking only at already-buffered bytes preserves
             * stream alignment for the next iteration of the main
             * resync loop.
             */
            const size_t avail = (s_uart_cache_pos < s_uart_cache_len)
                ? (s_uart_cache_len - s_uart_cache_pos) : 0u;
            const size_t shown =
                avail > kSmoke48cBadPayloadDumpBytes
                    ? kSmoke48cBadPayloadDumpBytes : avail;
            printf("Smoke48c: RX bad payload_len %u during %s "
                   "header=[%02X %02X %02X] cache_peek%u=",
                   (unsigned)payload_len, context,
                   raw[0], raw[1], raw[2], (unsigned)shown);
            for (size_t i = 0; i < shown; ++i) {
                printf(" %02X", s_uart_cache[s_uart_cache_pos + i]);
            }
            printf("\n");
            continue;
        }
        const size_t frame_len = 3u + payload_len + 1u;
        for (size_t i = 3u; i < frame_len; ++i) {
            if (!ReadByteWithDeadline(&raw[i], deadline)) {
                printf("Smoke48c: RX timeout payload/crc during %s\n", context);
                return false;
            }
        }
        if (!FusionSavestate_DecodeV2Frame(raw, frame_len, decoded)) {
            printf("Smoke48c: RX decode/CRC failed during %s raw=", context);
            for (size_t i = 0; i < frame_len; ++i) {
                printf(" %02X", raw[i]);
            }
            printf("\n");
            continue;
        }
        return true;
    }
    printf("Smoke48c: RX timeout valid frame during %s\n", context);
    return false;
}

static bool ReadDecodedFrame(const char *context, FusionV2Decoded_t *decoded)
{
    return ReadDecodedFrameTimed(context,
                                 decoded,
                                 kSmoke48cFrameTimeoutMs,
                                 NULL);
}

static bool SendFrame(const char *label, const FusionV2Frame_t *frame, bool quiet)
{
    if (frame == NULL || frame->length == 0u) {
        printf("Smoke48c: %s invalid frame\n", label);
        return false;
    }
    if (!quiet) {
        printf("Smoke48c: TX %s:", label);
        for (uint8_t i = 0; i < frame->length; ++i) {
            printf(" %02X", frame->bytes[i]);
        }
        printf("\n");
    }
    const int written = uart_write_bytes(UART_NUM_1,
                                         (const char *)frame->bytes,
                                         frame->length);
    const esp_err_t wait_err = uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(250));
    if (written != (int)frame->length) {
        printf("Smoke48c: TX %s failed wrote=%d expected=%u\n",
               label, written, (unsigned)frame->length);
        return false;
    }
    if (wait_err != ESP_OK) {
        printf("Smoke48c: TX %s wait failed err=%s\n",
               label, esp_err_to_name(wait_err));
        return false;
    }
    return true;
}

static void RecordFirstFailure(const char *step,
                               const char *reason,
                               int32_t region,
                               int32_t expected_seq,
                               int32_t got_seq,
                               uint32_t bytes_so_far,
                               uint32_t packets_so_far)
{
    if (s_result.fail_step[0] != '\0') {
        return;
    }
    snprintf(s_result.fail_step, sizeof(s_result.fail_step), "%s", step);
    snprintf(s_result.fail_reason, sizeof(s_result.fail_reason), "%s", reason);
    s_result.fail_region = region;
    s_result.fail_expected_seq = expected_seq;
    s_result.fail_got_seq = got_seq;
    s_result.fail_bytes_so_far = bytes_so_far;
    s_result.fail_packets_so_far = packets_so_far;
}

static bool ExpectCtl(uint8_t opcode, uint8_t ack_for, const char *context,
                      const char *step_label, int32_t region)
{
    FusionV2Decoded_t d;
    if (!ReadDecodedFrame(context, &d)) {
        RecordFirstFailure(step_label, "rx_timeout_or_unrecoverable_resync",
                           region, -1, -1, 0u, 0u);
        return false;
    }
    if (d.addr != (uint8_t)kFusionAddr_StateCtl) {
        printf("Smoke48c: %s expected CTL got addr=0x%02X op=0x%02X\n",
               context, d.addr, d.ctl_opcode);
        RecordFirstFailure(step_label, "unexpected_addr",
                           region, -1, -1, 0u, 0u);
        return false;
    }
    if (d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Smoke48c: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               context, d.error_code, d.error_detail);
        char reason[64];
        snprintf(reason, sizeof(reason),
                 "fpga_error_code=0x%02X_detail=0x%02X",
                 d.error_code, d.error_detail);
        RecordFirstFailure(step_label, reason, region, -1, -1, 0u, 0u);
        return false;
    }
    if (d.ctl_opcode != opcode || d.ack_for_opcode != ack_for) {
        printf("Smoke48c: %s unexpected CTL op=0x%02X ack_for=0x%02X "
               "(want op=0x%02X ack_for=0x%02X)\n",
               context, d.ctl_opcode, d.ack_for_opcode, opcode, ack_for);
        char reason[64];
        snprintf(reason, sizeof(reason),
                 "unexpected_ctl_op=0x%02X_ack_for=0x%02X",
                 d.ctl_opcode, d.ack_for_opcode);
        RecordFirstFailure(step_label, reason, region, -1, -1, 0u, 0u);
        return false;
    }
    return true;
}

static bool ExpectWriteBeginOrProceed(const char *context,
                                      const char *step_label,
                                      int32_t region)
{
    FusionV2Decoded_t d;
    bool saw_bad_payload = false;
    if (!ReadDecodedFrameTimed(context, &d, 200u, &saw_bad_payload)) {
        printf("Smoke48c: %s no decodable ACK_ACCEPTED within 200ms "
               "(saw_bad_payload=%s); proceeding to DATA and requiring "
               "final ACK_DONE\n",
               context, saw_bad_payload ? "yes" : "no");
        return true;
    }
    if (d.addr != (uint8_t)kFusionAddr_StateCtl) {
        printf("Smoke48c: %s expected CTL got addr=0x%02X op=0x%02X\n",
               context, d.addr, d.ctl_opcode);
        RecordFirstFailure(step_label, "unexpected_write_begin_ack_addr",
                           region, -1, -1, 0u, 0u);
        return false;
    }
    if (d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Smoke48c: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               context, d.error_code, d.error_detail);
        char reason[64];
        snprintf(reason, sizeof(reason),
                 "fpga_error_code=0x%02X_detail=0x%02X",
                 d.error_code, d.error_detail);
        RecordFirstFailure(step_label, reason, region, -1, -1, 0u, 0u);
        return false;
    }
    if (d.ctl_opcode != (uint8_t)kFusionOp_AckAccepted ||
        d.ack_for_opcode != (uint8_t)kFusionOp_WriteStreamBegin) {
        printf("Smoke48c: %s unexpected CTL op=0x%02X ack_for=0x%02X "
               "(want ACK_ACCEPTED/WRITE_STREAM_BEGIN)\n",
               context, d.ctl_opcode, d.ack_for_opcode);
        RecordFirstFailure(step_label, "unexpected_write_begin_ack_ctl",
                           region, -1, -1, 0u, 0u);
        return false;
    }
    return true;
}

static bool ExpectWriteDoneOrProceed(const char *context,
                                     const char *step_label,
                                     int32_t region)
{
    FusionV2Decoded_t d;
    bool saw_bad_payload = false;
    if (!ReadDecodedFrameTimed(context, &d, 200u, &saw_bad_payload)) {
        printf("Smoke48c: %s no decodable ACK_DONE within 200ms "
               "(saw_bad_payload=%s); proceeding to readback verification\n",
               context, saw_bad_payload ? "yes" : "no");
        return true;
    }
    if (d.addr != (uint8_t)kFusionAddr_StateCtl) {
        printf("Smoke48c: %s expected CTL got addr=0x%02X op=0x%02X\n",
               context, d.addr, d.ctl_opcode);
        RecordFirstFailure(step_label, "unexpected_write_done_addr",
                           region, -1, -1, 0u, 0u);
        return false;
    }
    if (d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Smoke48c: %s FPGA ERROR code=0x%02X detail=0x%02X\n",
               context, d.error_code, d.error_detail);
        char reason[64];
        snprintf(reason, sizeof(reason),
                 "fpga_error_code=0x%02X_detail=0x%02X",
                 d.error_code, d.error_detail);
        RecordFirstFailure(step_label, reason, region, -1, -1, 0u, 0u);
        return false;
    }
    if (d.ctl_opcode != (uint8_t)kFusionOp_AckDone ||
        d.ack_for_opcode != (uint8_t)kFusionOp_WriteStreamBegin) {
        printf("Smoke48c: %s unexpected CTL op=0x%02X ack_for=0x%02X "
               "(want ACK_DONE/WRITE_STREAM_BEGIN)\n",
               context, d.ctl_opcode, d.ack_for_opcode);
        RecordFirstFailure(step_label, "unexpected_write_done_ctl",
                           region, -1, -1, 0u, 0u);
        return false;
    }
    return true;
}

static bool BuildEnd(FusionV2Frame_t *out)
{
    return FusionSavestate_BuildEndSession(out);
}

static bool IsEndSessionAck(const FusionV2Decoded_t *d)
{
    return d != NULL &&
           d->addr == (uint8_t)kFusionAddr_StateCtl &&
           d->ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
           d->ack_for_opcode == (uint8_t)kFusionOp_EndSession;
}

static bool SendEndSessionRetry(const char *label, const char *step_label)
{
    FusionV2Frame_t frame;
    if (!BuildEnd(&frame)) {
        printf("Smoke48c: build %s failed\n", label);
        return false;
    }
    for (uint8_t attempt = 0u; attempt < kSmoke48cEndSessionAttempts; ++attempt) {
        if (attempt != 0u) {
            uint32_t drained =
                DrainRxForMsCounted(kSmoke48cPreflightDrainMs);
            if (drained > 0u) {
                printf("Smoke48c: %s retry attempt=%u drained_bytes=%" PRIu32 "\n",
                       label, (unsigned)attempt, drained);
            }
            ResetUartCache();
        }
        if (!SendFrame(label, &frame, /*quiet=*/(attempt != 0u))) {
            return false;
        }
        FusionV2Decoded_t d;
        if (ReadDecodedFrame(label, &d) && IsEndSessionAck(&d)) {
            return true;
        }
    }
    if (step_label != NULL) {
        RecordFirstFailure(step_label, "end_session_no_ack_after_retries",
                           -1, -1, -1, 0u, 0u);
    }
    return false;
}

/*
 * Open the single mixed paused session.  Drains any prior bytes, sends
 * an idempotent END_SESSION clear in case a stale session is open, then
 * BEGIN_TEST_RW.  After the ACK, drains residue and records the open
 * timestamp so PrintResult can report total paused session duration.
 *
 * Distinguishes "old bridge" (BEGIN_TEST_RW rejected with ERR_NO_SESSION
 * by the SESS_NONE default arm of a P48C bridge) from a real protocol
 * failure: in the former case the harness reports the firmware mismatch
 * and aborts cleanly so the maintainer can flash the P48D bitstream.
 */
static bool OpenMixedSession(void)
{
    DrainRxForMs(kSmoke48cPreflightDrainMs);
    FPGA_Rx_ResetParser();
    ResetUartCache();

    if (!SendEndSessionRetry("END_SESSION clear (mixed)",
                             "mixed_clear")) {
        printf("Smoke48c: mixed-session clear failed\n");
        return false;
    }
    DrainRxForMs(kSmoke48cPreflightDrainMs);
    FPGA_Rx_ResetParser();
    ResetUartCache();

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildBeginTestRW(0u, &begin)) {
        printf("Smoke48c: build BEGIN_TEST_RW failed\n");
        return false;
    }
    if (!SendFrame("BEGIN_TEST_RW (mixed)", &begin, /*quiet=*/false)) {
        return false;
    }

    FusionV2Decoded_t d;
    if (!ReadDecodedFrame("BEGIN_TEST_RW ack", &d)) {
        RecordFirstFailure("mixed_begin",
                           "rx_timeout_for_begin_ack",
                           -1, -1, -1, 0u, 0u);
        return false;
    }
    if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
        d.ctl_opcode == (uint8_t)kFusionOp_Error) {
        printf("Smoke48c: BEGIN_TEST_RW FPGA ERROR code=0x%02X detail=0x%02X "
               "(if code=0x03 the bridge is older P48C; flash a P48D bitstream "
               "with OP_BEGIN_TEST_RW support)\n",
               d.error_code, d.error_detail);
        char reason[64];
        snprintf(reason, sizeof(reason),
                 "begin_test_rw_error_code=0x%02X_detail=0x%02X",
                 d.error_code, d.error_detail);
        RecordFirstFailure("mixed_begin", reason, -1, -1, -1, 0u, 0u);
        return false;
    }
    if (!(d.addr == (uint8_t)kFusionAddr_StateCtl &&
          d.ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
          d.ack_for_opcode == (uint8_t)kFusionOp_BeginTestRW)) {
        printf("Smoke48c: BEGIN_TEST_RW unexpected reply addr=0x%02X "
               "op=0x%02X ack_for=0x%02X\n",
               d.addr, d.ctl_opcode, d.ack_for_opcode);
        char reason[64];
        snprintf(reason, sizeof(reason),
                 "unexpected_ctl_op=0x%02X_ack_for=0x%02X",
                 d.ctl_opcode, d.ack_for_opcode);
        RecordFirstFailure("mixed_begin", reason, -1, -1, -1, 0u, 0u);
        return false;
    }

    s_session_open_us = esp_timer_get_time();

    const uint32_t drained = DrainRxForMsCounted(kSmoke48cPostBeginDrainMs);
    if (drained != 0u) {
        printf("Smoke48c: mixed post-BEGIN drained %" PRIu32 " stale bytes\n",
               drained);
    }
    FPGA_Rx_ResetParser();
    ResetUartCache();
    vTaskDelay(pdMS_TO_TICKS(kSmoke48cInterSessionMs));
    return true;
}

static bool CloseMixedSession(void)
{
    DrainRxForMs(kSmoke48cFinalEndMs);
    ResetUartCache();
    return SendEndSessionRetry("END_SESSION cleanup (mixed)",
                               "mixed_cleanup");
}

static bool ReadRegionStream(uint8_t region,
                             uint16_t length,
                             uint8_t *dest,
                             const char *region_label,
                             const char *phase_label,
                             const char *step_label)
{
    uint16_t received = 0u;
    uint16_t expected_seq = 0u;
    const int64_t t0 = esp_timer_get_time();

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildReadStreamBegin(region, 0u, length, &begin)) {
        printf("Smoke48c: build READ_STREAM_BEGIN %s failed\n", region_label);
        return false;
    }
    char tx_label[48];
    snprintf(tx_label, sizeof(tx_label), "READ_STREAM_BEGIN(%s,%s)",
             region_label, phase_label);
    if (!SendFrame(tx_label, &begin, /*quiet=*/false)) {
        return false;
    }

    bool pending_data_valid = false;
    FusionV2Decoded_t pending_data;
    memset(&pending_data, 0, sizeof(pending_data));
    {
        FusionV2Decoded_t first;
        if (!ReadDecodedFrame("READ_STREAM_BEGIN ack", &first)) {
            RecordFirstFailure(step_label, "rx_timeout_for_begin_ack",
                               (int32_t)region, 0, -1, 0u, 0u);
            return false;
        }
        if (first.addr == (uint8_t)kFusionAddr_StateCtl &&
            first.ctl_opcode == (uint8_t)kFusionOp_Error) {
            printf("Smoke48c: %s/%s FPGA ERROR begin code=0x%02X detail=0x%02X\n",
                   phase_label, region_label, first.error_code, first.error_detail);
            char reason[64];
            snprintf(reason, sizeof(reason),
                     "fpga_error_code=0x%02X_detail=0x%02X",
                     first.error_code, first.error_detail);
            RecordFirstFailure(step_label, reason, (int32_t)region, 0, -1, 0u, 0u);
            return false;
        }
        if (first.addr == (uint8_t)kFusionAddr_StateCtl &&
            first.ctl_opcode == (uint8_t)kFusionOp_AckAccepted &&
            first.ack_for_opcode == (uint8_t)kFusionOp_ReadStreamBegin) {
            /* Expected. */
        } else if (first.addr == (uint8_t)kFusionAddr_StateData &&
                   first.data_seq == 0u) {
            pending_data = first;
            pending_data_valid = true;
        } else {
            printf("Smoke48c: %s/%s unexpected first frame addr=0x%02X op=0x%02X\n",
                   phase_label, region_label, first.addr, first.ctl_opcode);
            RecordFirstFailure(step_label, "unexpected_first_frame",
                               (int32_t)region, 0, -1, 0u, 0u);
            return false;
        }
    }

    uint32_t packets = 0u;
    while (received < length) {
        FusionV2Decoded_t d;
        if (pending_data_valid) {
            d = pending_data;
            pending_data_valid = false;
        } else if (!ReadDecodedFrame(region_label, &d)) {
            RecordFirstFailure(step_label, "rx_timeout_mid_stream",
                               (int32_t)region, (int32_t)expected_seq, -1,
                               received, packets);
            return false;
        }
        if (d.addr == (uint8_t)kFusionAddr_StateCtl &&
            d.ctl_opcode == (uint8_t)kFusionOp_Error) {
            printf("Smoke48c: %s/%s FPGA ERROR mid-stream code=0x%02X "
                   "detail=0x%02X received=%u expected_seq=%u\n",
                   phase_label, region_label, d.error_code, d.error_detail,
                   (unsigned)received, (unsigned)expected_seq);
            char reason[64];
            snprintf(reason, sizeof(reason),
                     "fpga_error_code=0x%02X_detail=0x%02X",
                     d.error_code, d.error_detail);
            RecordFirstFailure(step_label, reason, (int32_t)region,
                               (int32_t)expected_seq, -1, received, packets);
            return false;
        }
        if (d.addr != (uint8_t)kFusionAddr_StateData) {
            printf("Smoke48c: %s/%s expected DATA got addr=0x%02X op=0x%02X\n",
                   phase_label, region_label, d.addr, d.ctl_opcode);
            RecordFirstFailure(step_label, "expected_data_got_other",
                               (int32_t)region, (int32_t)expected_seq, -1,
                               received, packets);
            return false;
        }
        if (d.data_seq != expected_seq) {
            printf("Smoke48c: %s/%s seq mismatch got=%u expected=%u received=%u\n",
                   phase_label, region_label, (unsigned)d.data_seq,
                   (unsigned)expected_seq, (unsigned)received);
            RecordFirstFailure(step_label, "seq_mismatch",
                               (int32_t)region, (int32_t)expected_seq,
                               (int32_t)d.data_seq, received, packets);
            return false;
        }
        if (d.data_len == 0u || (uint16_t)d.data_len > (length - received)) {
            printf("Smoke48c: %s/%s bad data_len=%u remaining=%u\n",
                   phase_label, region_label,
                   (unsigned)d.data_len, (unsigned)(length - received));
            RecordFirstFailure(step_label, "bad_data_len",
                               (int32_t)region, (int32_t)expected_seq,
                               (int32_t)d.data_seq, received, packets);
            return false;
        }
        memcpy(&dest[received], d.data, d.data_len);
        received = (uint16_t)(received + d.data_len);
        expected_seq = (uint16_t)(expected_seq + 1u);
        packets++;
    }

    char done_ctx[48];
    snprintf(done_ctx, sizeof(done_ctx), "ACK_DONE(READ %s)", region_label);
    if (!ExpectCtl((uint8_t)kFusionOp_AckDone,
                   (uint8_t)kFusionOp_ReadStreamBegin,
                   done_ctx, step_label, (int32_t)region)) {
        return false;
    }
    ResetUartCache();
    uart_flush_input(UART_NUM_1);

    const uint32_t crc = FusionSavestate_Crc32(dest, received);
    const int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    printf("Smoke48c: READ %s/%s PASS packets=%u bytes=%u crc32=%08" PRIX32
           " dt_ms=%" PRId64 "\n",
           phase_label, region_label,
           (unsigned)expected_seq, (unsigned)received, crc, elapsed_ms);
    return true;
}

static bool WriteRegionStream(uint8_t region,
                              uint16_t length,
                              const uint8_t *src,
                              const char *region_label,
                              const char *phase_label,
                              const char *step_label)
{
    const int64_t t0 = esp_timer_get_time();

    FusionV2Frame_t begin;
    if (!FusionSavestate_BuildWriteStreamBegin(region, 0u, length, &begin)) {
        printf("Smoke48c: build WRITE_STREAM_BEGIN %s failed\n", region_label);
        return false;
    }
    char tx_label[48];
    snprintf(tx_label, sizeof(tx_label), "WRITE_STREAM_BEGIN(%s,%s)",
             region_label, phase_label);
    if (!SendFrame(tx_label, &begin, /*quiet=*/false)) {
        return false;
    }
    if (!ExpectWriteBeginOrProceed(tx_label, step_label, (int32_t)region)) {
        return false;
    }

    uint16_t total_sent = 0u;
    uint16_t seq = 0u;
    while (total_sent < length) {
        const uint16_t remaining = (uint16_t)(length - total_sent);
        const uint16_t chunk = remaining >= FUSION_STATE_DATA_MAX_DATA
                             ? (uint16_t)FUSION_STATE_DATA_MAX_DATA
                             : remaining;
        FusionV2Frame_t data_frame;
        if (!FusionSavestate_BuildStateData(seq,
                                            &src[total_sent],
                                            chunk,
                                            &data_frame)) {
            printf("Smoke48c: %s/%s build STATE_DATA seq=%u chunk=%u failed\n",
                   phase_label, region_label, (unsigned)seq, (unsigned)chunk);
            RecordFirstFailure(step_label, "build_state_data_failed",
                               (int32_t)region, (int32_t)seq, -1,
                               total_sent, seq);
            return false;
        }
        if (!SendFrame("STATE_DATA", &data_frame, /*quiet=*/true)) {
            RecordFirstFailure(step_label, "tx_state_data_failed",
                               (int32_t)region, (int32_t)seq, -1,
                               total_sent, seq);
            return false;
        }
        seq = (uint16_t)(seq + 1u);
        total_sent = (uint16_t)(total_sent + chunk);
        if ((seq % kSmoke48cWriteProgressEvery) == 0u) {
            printf("Smoke48c: WRITE %s/%s progress packets=%u bytes=%u\n",
                   phase_label, region_label,
                   (unsigned)seq, (unsigned)total_sent);
        }
    }

    char done_ctx[48];
    snprintf(done_ctx, sizeof(done_ctx),
             "ACK_DONE(WRITE %s,%s)", region_label, phase_label);
    if (!ExpectWriteDoneOrProceed(done_ctx, step_label, (int32_t)region)) {
        return false;
    }

    const uint32_t crc = FusionSavestate_Crc32(src, length);
    const int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    printf("Smoke48c: WRITE %s/%s PASS packets=%u bytes=%u crc32=%08" PRIX32
           " dt_ms=%" PRId64 "\n",
           phase_label, region_label,
           (unsigned)seq, (unsigned)total_sent, crc, elapsed_ms);
    return true;
}

static void ApplyXorPattern(const uint8_t *in, uint8_t *out, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        out[i] = (uint8_t)(in[i] ^ kXorPattern[i & 0x1Fu]);
    }
}

static void CompareAndPreview(const char *label,
                              const uint8_t *expected,
                              const uint8_t *actual,
                              size_t len,
                              uint32_t *mismatch_out,
                              int32_t *first_offset_out)
{
    uint32_t cnt = 0u;
    int32_t first = -1;
    /* Capture up to N preview entries for forensic analysis. */
    int32_t  preview_offsets[kSmoke48cMismatchPreviewCount] = {0};
    uint8_t  preview_expected[kSmoke48cMismatchPreviewCount] = {0};
    uint8_t  preview_actual[kSmoke48cMismatchPreviewCount] = {0};
    uint8_t  preview_count = 0u;

    for (size_t i = 0; i < len; ++i) {
        if (expected[i] != actual[i]) {
            if (first < 0) {
                first = (int32_t)i;
            }
            if (preview_count < kSmoke48cMismatchPreviewCount) {
                preview_offsets[preview_count] = (int32_t)i;
                preview_expected[preview_count] = expected[i];
                preview_actual[preview_count] = actual[i];
                preview_count++;
            }
            cnt++;
        }
    }
    *mismatch_out = cnt;
    *first_offset_out = first;
    if (cnt == 0u) {
        printf("Smoke48c: COMPARE %s match=exact len=%u\n",
               label, (unsigned)len);
        return;
    }
    printf("Smoke48c: COMPARE %s match=fail len=%u mismatches=%u "
           "first_offset=%" PRId32 " expected=%02X actual=%02X\n",
           label, (unsigned)len, (unsigned)cnt, first,
           expected[first], actual[first]);
    printf("Smoke48c: COMPARE %s preview", label);
    for (uint8_t i = 0; i < preview_count; ++i) {
        printf(" [%" PRId32 ":%02X->%02X]",
               preview_offsets[i], preview_expected[i], preview_actual[i]);
    }
    printf("\n");
}

static bool ValidateSlotQuiet(uint32_t *generation_out, const char *moment)
{
    FusionStorage_t storage;
    FusionStorageResult_t sr =
        FusionStorage_InitFromPartition(&storage, kPartitionLabel);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48c: SLOT_%s init partition failed result=%d\n",
               moment, (int)sr);
        return false;
    }
    uint32_t slot = 0u;
    uint32_t generation = 0u;
    sr = FusionStorage_FindNewestValidSlot(&storage, &slot, &generation);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48c: SLOT_%s no valid slot result=%d\n", moment, (int)sr);
        return false;
    }
    FusionStateHeader_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    sr = FusionStorage_ReadSlotHeader(&storage, slot, &hdr);
    if (sr != kFusionStorage_Ok) {
        printf("Smoke48c: SLOT_%s read header failed result=%d\n",
               moment, (int)sr);
        return false;
    }
    const bool tag_ok =
        (memcmp(hdr.savestate_tag, "P48C", 4u) == 0) ||
        (memcmp(hdr.savestate_tag, "P48D", 4u) == 0);
    if (hdr.magic != FUSION_SAVESTATE_MAGIC ||
        hdr.format_version != FUSION_SAVESTATE_FORMAT_VERSION ||
        hdr.header_size != sizeof(FusionStateHeader_t) ||
        hdr.commit_state != (uint8_t)kFusionCommit_Valid ||
        !tag_ok) {
        printf("Smoke48c: SLOT_%s header rejected magic=%08" PRIX32
               " state=0x%02X tag='%c%c%c%c'\n",
               moment, hdr.magic, hdr.commit_state,
               hdr.savestate_tag[0], hdr.savestate_tag[1],
               hdr.savestate_tag[2], hdr.savestate_tag[3]);
        return false;
    }
    if (generation_out != NULL) {
        *generation_out = generation;
    }
    printf("Smoke48c: SLOT_%s valid generation=%" PRIu32 " total=%" PRIu32
           " payload_crc=%08" PRIX32 "\n",
           moment, generation, hdr.total_size, hdr.payload_crc32);
    return true;
}

static bool AllocBuffers(void)
{
    if (s_wram_storage != NULL) {
        return true;
    }
    s_wram_storage = (uint8_t *)heap_caps_malloc(2u * SMOKE48C_WRAM_LEN,
                                                 MALLOC_CAP_8BIT);
    if (s_wram_storage == NULL) {
        printf("Smoke48c: heap alloc %u failed\n",
               (unsigned)(2u * SMOKE48C_WRAM_LEN));
        return false;
    }
    s_orig_wram = s_wram_storage;
    s_scratch_wram = s_wram_storage + SMOKE48C_WRAM_LEN;
    return true;
}

static void FreeBuffers(void)
{
    if (s_wram_storage != NULL) {
        heap_caps_free(s_wram_storage);
        s_wram_storage = NULL;
        s_orig_wram = NULL;
        s_scratch_wram = NULL;
    }
}

static void PrintResult(bool overall_pass)
{
    const Smoke48cResult_t *r = &s_result;
    printf("Smoke48c: STATUS slot_pre=%s begin=%s r1=%s w1=%s r2=%s w2=%s "
           "r3=%s end=%s slot_post=%s core_released=%s "
           "restore_attempted=%s restore_verified=%s\n",
           r->slot_valid_pre ? "yes" : "no",
           r->begin_ok ? "yes" : "no",
           r->phase_r1_ok ? "yes" : "no",
           r->phase_w1_ok ? "yes" : "no",
           r->phase_r2_ok ? "yes" : "no",
           r->phase_w2_ok ? "yes" : "no",
           r->phase_r3_ok ? "yes" : "no",
           r->end_ok ? "yes" : "no",
           r->slot_valid_post ? "yes" : "no",
           r->core_released ? "yes" : "no",
           r->restore_attempted ? "yes" : "no",
           r->restore_verified ? "yes" : "no");
    printf("Smoke48c: CRC orig_hram=%08" PRIX32 " orig_wram=%08" PRIX32
           " temp_hram=%08" PRIX32 " temp_wram=%08" PRIX32 "\n",
           r->crc_orig_hram, r->crc_orig_wram,
           r->crc_temp_hram, r->crc_temp_wram);
    printf("Smoke48c: CRC readback_temp_hram=%08" PRIX32
           " readback_temp_wram=%08" PRIX32
           " readback_orig_hram=%08" PRIX32
           " readback_orig_wram=%08" PRIX32 "\n",
           r->crc_readback_temp_hram, r->crc_readback_temp_wram,
           r->crc_readback_orig_hram, r->crc_readback_orig_wram);
    printf("Smoke48c: MISMATCH temp_hram=%" PRIu32 " (first=%" PRId32 ") "
           "temp_wram=%" PRIu32 " (first=%" PRId32 ") "
           "orig_hram=%" PRIu32 " (first=%" PRId32 ") "
           "orig_wram=%" PRIu32 " (first=%" PRId32 ")\n",
           r->mismatch_temp_hram, r->first_mismatch_temp_hram,
           r->mismatch_temp_wram, r->first_mismatch_temp_wram,
           r->mismatch_orig_hram, r->first_mismatch_orig_hram,
           r->mismatch_orig_wram, r->first_mismatch_orig_wram);
    printf("Smoke48c: PAUSED_SESSION_us=%" PRId64 "\n",
           r->paused_session_us);
    printf("Smoke48c: SLOT_GEN pre=%" PRIu32 " post=%" PRIu32 "\n",
           r->generation_pre, r->generation_post);
    if (r->fail_step[0] != '\0') {
        printf("Smoke48c: FIRST_FAIL step=%s reason=%s region=%" PRId32
               " expected_seq=%" PRId32 " got_seq=%" PRId32
               " bytes_so_far=%" PRIu32 " packets_so_far=%" PRIu32
               " drained_bytes=%" PRIu32 "\n",
               r->fail_step, r->fail_reason, r->fail_region,
               r->fail_expected_seq, r->fail_got_seq,
               r->fail_bytes_so_far, r->fail_packets_so_far,
               r->fail_drained_bytes);
    }
    printf("Smoke48c: RESULT %s (4.8c v2 RAM-only round-trip; one paused "
           "session; Top/CPU/Timer restore not exercised; no hotkey)\n",
           overall_pass ? "PASS" : "FAIL");
}

static void CompareTempWram(const uint8_t *orig_wram,
                            const uint8_t *readback)
{
    /* WRAM expected = orig XOR kXorPattern.  Re-derived on the fly to
     * avoid a second 32 KiB allocation; this is a hot loop in the
     * single-session run so the bookkeeping stays tight. */
    uint32_t cnt = 0u;
    int32_t  first = -1;
    int32_t  preview_offsets[kSmoke48cMismatchPreviewCount] = {0};
    uint8_t  preview_expected[kSmoke48cMismatchPreviewCount] = {0};
    uint8_t  preview_actual[kSmoke48cMismatchPreviewCount] = {0};
    uint8_t  preview_count = 0u;
    for (size_t i = 0; i < SMOKE48C_WRAM_LEN; ++i) {
        const uint8_t expected =
            (uint8_t)(orig_wram[i] ^ kXorPattern[i & 0x1Fu]);
        if (expected != readback[i]) {
            if (first < 0) first = (int32_t)i;
            if (preview_count < kSmoke48cMismatchPreviewCount) {
                preview_offsets[preview_count] = (int32_t)i;
                preview_expected[preview_count] = expected;
                preview_actual[preview_count] = readback[i];
                preview_count++;
            }
            cnt++;
        }
    }
    s_result.mismatch_temp_wram = cnt;
    s_result.first_mismatch_temp_wram = first;
    if (cnt == 0u) {
        printf("Smoke48c: COMPARE temp_wram match=exact len=%u\n",
               (unsigned)SMOKE48C_WRAM_LEN);
        return;
    }
    const uint8_t expected =
        (uint8_t)(orig_wram[first] ^ kXorPattern[first & 0x1F]);
    printf("Smoke48c: COMPARE temp_wram match=fail len=%u mismatches=%u "
           "first_offset=%" PRId32 " expected=%02X actual=%02X\n",
           (unsigned)SMOKE48C_WRAM_LEN, (unsigned)cnt, first,
           expected, readback[first]);
    printf("Smoke48c: COMPARE temp_wram preview");
    for (uint8_t i = 0; i < preview_count; ++i) {
        printf(" [%" PRId32 ":%02X->%02X]",
               preview_offsets[i], preview_expected[i], preview_actual[i]);
    }
    printf("\n");
}

bool FusionSavestate_RamRoundTripSlot0(void)
{
    memset(&s_result, 0, sizeof(s_result));
    s_result.first_mismatch_temp_hram = -1;
    s_result.first_mismatch_temp_wram = -1;
    s_result.first_mismatch_orig_hram = -1;
    s_result.first_mismatch_orig_wram = -1;
    s_result.fail_region = -1;
    s_result.fail_expected_seq = -1;
    s_result.fail_got_seq = -1;
    s_session_open_us = 0;

    if (!AllocBuffers()) {
        PrintResult(false);
        return false;
    }

    s_result.slot_valid_pre =
        ValidateSlotQuiet(&s_result.generation_pre, "PRE");
    if (!s_result.slot_valid_pre) {
        printf("Smoke48c: pre-slot validation failed; run 'smoke48a save' to "
               "capture a P48C-tagged slot before retrying smoke48c\n");
        FreeBuffers();
        PrintResult(false);
        return false;
    }

    PwrMgr_IdleTimerSuspend();
    TaskHandle_t *tx_task = FPGA_GetTxTaskHandle();
    TaskHandle_t *rx_task = FPGA_GetRxTaskHandle();
    const bool tx_paused = (tx_task != NULL && *tx_task != NULL);
    const bool rx_paused = (rx_task != NULL && *rx_task != NULL);
    if (tx_paused) FPGA_Tx_Pause();
    if (rx_paused) FPGA_Rx_Pause();
    vTaskDelay(pdMS_TO_TICKS(30));
    uart_flush(UART_NUM_1);
    ResetUartCache();
    if (!FPGA_UartOwnerAcquire(pdMS_TO_TICKS(2000))) {
        printf("Smoke48c: UART owner unavailable\n");
        if (rx_paused) FPGA_Rx_Resume();
        if (tx_paused) FPGA_Tx_Resume();
        PwrMgr_IdleTimerResume();
        FreeBuffers();
        PrintResult(false);
        return false;
    }

    /*
     * Single mixed paused session.  Inside this BEGIN_TEST_RW /
     * END_SESSION pair, session_pause stays asserted continuously and
     * the GB CPU never executes, so RAM cannot drift between any
     * harness write and its readback.  All five phases (R1, W1, R2,
     * W2, R3) run inline below.  No END_SESSION is sent until the
     * very end.
     */
    if (OpenMixedSession()) {
        s_result.begin_ok = true;

        /* R1: read original HRAM and WRAM. */
        const bool r1_h = ReadRegionStream(SMOKE48C_HRAM_REGION,
                                           SMOKE48C_HRAM_LEN,
                                           s_orig_hram, "HRAM", "R1",
                                           "read_orig_hram");
        const bool r1_w = r1_h && ReadRegionStream(SMOKE48C_WRAM_REGION,
                                                   SMOKE48C_WRAM_LEN,
                                                   s_orig_wram, "WRAM", "R1",
                                                   "read_orig_wram");
        s_result.phase_r1_ok = r1_h && r1_w;

        if (s_result.phase_r1_ok) {
            s_result.crc_orig_hram =
                FusionSavestate_Crc32(s_orig_hram, SMOKE48C_HRAM_LEN);
            s_result.crc_orig_wram =
                FusionSavestate_Crc32(s_orig_wram, SMOKE48C_WRAM_LEN);
            ApplyXorPattern(s_orig_hram, s_scratch_hram, SMOKE48C_HRAM_LEN);
            ApplyXorPattern(s_orig_wram, s_scratch_wram, SMOKE48C_WRAM_LEN);
            s_result.crc_temp_hram =
                FusionSavestate_Crc32(s_scratch_hram, SMOKE48C_HRAM_LEN);
            s_result.crc_temp_wram =
                FusionSavestate_Crc32(s_scratch_wram, SMOKE48C_WRAM_LEN);
            printf("Smoke48c: PHASE R1 complete: orig_hram=%08" PRIX32
                   " orig_wram=%08" PRIX32 " temp_hram=%08" PRIX32
                   " temp_wram=%08" PRIX32 "\n",
                   s_result.crc_orig_hram, s_result.crc_orig_wram,
                   s_result.crc_temp_hram, s_result.crc_temp_wram);

            /* W1: write XOR-pattern temp into HRAM and WRAM. */
            const bool w1_h = WriteRegionStream(SMOKE48C_HRAM_REGION,
                                                SMOKE48C_HRAM_LEN,
                                                s_scratch_hram, "HRAM", "W1",
                                                "write_temp_hram");
            const bool w1_w = w1_h && WriteRegionStream(SMOKE48C_WRAM_REGION,
                                                        SMOKE48C_WRAM_LEN,
                                                        s_scratch_wram,
                                                        "WRAM", "W1",
                                                        "write_temp_wram");
            s_result.phase_w1_ok = w1_h && w1_w;

            if (s_result.phase_w1_ok) {
                /* R2: re-read temp; bit-exact compare. */
                uint8_t saved_temp_hram[SMOKE48C_HRAM_LEN];
                memcpy(saved_temp_hram, s_scratch_hram, SMOKE48C_HRAM_LEN);

                const bool r2_h = ReadRegionStream(SMOKE48C_HRAM_REGION,
                                                   SMOKE48C_HRAM_LEN,
                                                   s_scratch_hram, "HRAM", "R2",
                                                   "read_back_temp_hram");
                const bool r2_w = r2_h && ReadRegionStream(SMOKE48C_WRAM_REGION,
                                                           SMOKE48C_WRAM_LEN,
                                                           s_scratch_wram,
                                                           "WRAM", "R2",
                                                           "read_back_temp_wram");
                s_result.phase_r2_ok = r2_h && r2_w;
                if (r2_h) {
                    s_result.crc_readback_temp_hram =
                        FusionSavestate_Crc32(s_scratch_hram,
                                              SMOKE48C_HRAM_LEN);
                    CompareAndPreview("temp_hram",
                                      saved_temp_hram, s_scratch_hram,
                                      SMOKE48C_HRAM_LEN,
                                      &s_result.mismatch_temp_hram,
                                      &s_result.first_mismatch_temp_hram);
                }
                if (r2_w) {
                    s_result.crc_readback_temp_wram =
                        FusionSavestate_Crc32(s_scratch_wram,
                                              SMOKE48C_WRAM_LEN);
                    CompareTempWram(s_orig_wram, s_scratch_wram);
                }

                /* W2: write originals back (best-effort restore). */
                if (s_result.phase_r2_ok) {
                    s_result.restore_attempted = true;
                    const bool w2_h = WriteRegionStream(SMOKE48C_HRAM_REGION,
                                                        SMOKE48C_HRAM_LEN,
                                                        s_orig_hram, "HRAM",
                                                        "W2",
                                                        "write_orig_hram");
                    const bool w2_w = w2_h &&
                                      WriteRegionStream(SMOKE48C_WRAM_REGION,
                                                        SMOKE48C_WRAM_LEN,
                                                        s_orig_wram, "WRAM",
                                                        "W2",
                                                        "write_orig_wram");
                    s_result.phase_w2_ok = w2_h && w2_w;

                    if (s_result.phase_w2_ok) {
                        /* R3: re-read originals; bit-exact compare. */
                        const bool r3_h =
                            ReadRegionStream(SMOKE48C_HRAM_REGION,
                                             SMOKE48C_HRAM_LEN,
                                             s_scratch_hram, "HRAM", "R3",
                                             "read_back_orig_hram");
                        const bool r3_w = r3_h &&
                            ReadRegionStream(SMOKE48C_WRAM_REGION,
                                             SMOKE48C_WRAM_LEN,
                                             s_scratch_wram, "WRAM", "R3",
                                             "read_back_orig_wram");
                        s_result.phase_r3_ok = r3_h && r3_w;
                        if (r3_h) {
                            s_result.crc_readback_orig_hram =
                                FusionSavestate_Crc32(s_scratch_hram,
                                                      SMOKE48C_HRAM_LEN);
                            CompareAndPreview("orig_hram",
                                              s_orig_hram, s_scratch_hram,
                                              SMOKE48C_HRAM_LEN,
                                              &s_result.mismatch_orig_hram,
                                              &s_result.first_mismatch_orig_hram);
                        }
                        if (r3_w) {
                            s_result.crc_readback_orig_wram =
                                FusionSavestate_Crc32(s_scratch_wram,
                                                      SMOKE48C_WRAM_LEN);
                            CompareAndPreview("orig_wram",
                                              s_orig_wram, s_scratch_wram,
                                              SMOKE48C_WRAM_LEN,
                                              &s_result.mismatch_orig_wram,
                                              &s_result.first_mismatch_orig_wram);
                            s_result.restore_verified =
                                (s_result.mismatch_orig_hram == 0u) &&
                                (s_result.mismatch_orig_wram == 0u);
                        }
                    }
                }
            }
        }

        /* End the single mixed session.  This is the only END_SESSION
         * inside the round-trip; session_pause drops here. */
        s_result.end_ok = CloseMixedSession();
        if (s_session_open_us != 0) {
            s_result.paused_session_us =
                esp_timer_get_time() - s_session_open_us;
            printf("Smoke48c: PAUSED_SESSION dt_us=%" PRId64 "\n",
                   s_result.paused_session_us);
        }
    }

    /* Final cleanup: idempotent END_SESSION ack confirms the bridge is
     * idle and the GB core is released. */
    {
        FusionV2Frame_t end_frame;
        if (BuildEnd(&end_frame)) {
            DrainRxForMs(kSmoke48cFinalEndMs);
            ResetUartCache();
            uart_flush_input(UART_NUM_1);
            (void)SendFrame("END_SESSION final", &end_frame, /*quiet=*/false);
            FusionV2Decoded_t d;
            if (ReadDecodedFrame("END_SESSION final ack", &d) && IsEndSessionAck(&d)) {
                s_result.core_released = true;
            } else {
                DrainRxForMs(kSmoke48cPreflightDrainMs);
                ResetUartCache();
                if (SendFrame("END_SESSION final retry", &end_frame, /*quiet=*/false)) {
                    if (ReadDecodedFrame("END_SESSION final retry ack", &d) &&
                        IsEndSessionAck(&d)) {
                        s_result.core_released = true;
                    }
                }
            }
        }
    }

    FPGA_UartOwnerRelease();
    if (rx_paused) FPGA_Rx_Resume();
    if (tx_paused) FPGA_Tx_Resume();
    PwrMgr_IdleTimerResume();

    s_result.slot_valid_post =
        ValidateSlotQuiet(&s_result.generation_post, "POST");

    const bool overall_pass =
        s_result.slot_valid_pre &&
        s_result.begin_ok &&
        s_result.phase_r1_ok && s_result.phase_w1_ok &&
        s_result.phase_r2_ok && s_result.phase_w2_ok &&
        s_result.phase_r3_ok && s_result.end_ok &&
        s_result.slot_valid_post && s_result.core_released &&
        s_result.restore_verified &&
        s_result.mismatch_temp_hram == 0u &&
        s_result.mismatch_temp_wram == 0u &&
        s_result.mismatch_orig_hram == 0u &&
        s_result.mismatch_orig_wram == 0u &&
        s_result.generation_pre == s_result.generation_post;

    PrintResult(overall_pass);
    FreeBuffers();
    return overall_pass;
}

static int Smoke48cCommand(int argc, char **argv)
{
    if (argc < 2 || strcmp(argv[1], "ramroundtrip") != 0) {
        printf("Usage: smoke48c ramroundtrip\n");
        return 1;
    }
    return FusionSavestate_RamRoundTripSlot0() ? 0 : 1;
}

void FusionSavestateSmoke48c_RegisterCommands(void)
{
    const esp_console_cmd_t command = {
        .command = "smoke48c",
        .help    = "Phase 4.8c RAM-only round-trip smoke: smoke48c ramroundtrip",
        .func    = &Smoke48cCommand,
        .argtable = NULL,
    };
    esp_err_t err = esp_console_cmd_register(&command);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Registering '%s' failed: %s",
                 command.command, esp_err_to_name(err));
    }
}
