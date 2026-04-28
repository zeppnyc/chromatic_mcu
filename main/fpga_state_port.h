#pragma once

#include "fusion_savestate.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * MCU-side state-port session/stream state machine.
 *
 * Owns the rules from the architecture doc:
 *   - one outstanding stream
 *   - per-stream 16-bit sequence counter starting at 0
 *   - wrong seq, wrong len, ERROR, or unexpected packet -> abort
 *   - after abort, late ACK_DONE / STATE_DATA from the failed generation
 *     are dropped, not applied
 *
 * Pure C: the transport (UART tx) and clock are injected via callbacks
 * so the same state machine compiles against ESP-IDF on device or against
 * a host mock for unit tests.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    kFusionStatePortRes_Ok = 0,
    kFusionStatePortRes_BadArg,
    kFusionStatePortRes_NotIdle,
    kFusionStatePortRes_NoSession,
    kFusionStatePortRes_StreamNotActive,
    kFusionStatePortRes_StreamWrongMode,
    kFusionStatePortRes_DataTooLong,
    kFusionStatePortRes_StreamExhausted,
    kFusionStatePortRes_Aborted,
    kFusionStatePortRes_LateIgnored,
    kFusionStatePortRes_Unexpected,
    kFusionStatePortRes_TxFailed,
    kFusionStatePortRes_TimedOut,
} FusionStatePortResult_t;

typedef enum {
    kFusionSession_Idle = 0,
    kFusionSession_OpenSave,
    kFusionSession_OpenLoad,
    kFusionSession_Aborted,
} FusionSessionState_t;

typedef enum {
    kFusionStream_None = 0,
    kFusionStream_Read,   /* MCU receives DATA from FPGA */
    kFusionStream_Write,  /* MCU sends DATA to FPGA */
} FusionStreamMode_t;

/*
 * Transport injection. tx_frame is invoked with a fully built V2 frame
 * (header...crc) and must move all bytes to the FPGA in order.
 *
 * now_us is optional; if NULL, the session watchdog is inactive.
 */
typedef struct {
    void   *user;
    bool  (*tx_frame)(void *user, const uint8_t *frame, size_t len);
    uint64_t (*now_us)(void *user);
} FusionStatePortTransport_t;

/*
 * Session/stream state. Plain struct; allocate wherever convenient.
 * stream_generation is bumped on every begin/abort so callers can fence
 * late responses against the right epoch.
 */
typedef struct {
    FusionStatePortTransport_t transport;

    FusionSessionState_t session_state;
    FusionStreamMode_t   stream_mode;
    bool                 stream_accepted;   /* true after ACK_ACCEPTED for current stream */
    uint8_t              pending_op;        /* opcode for which an ACK is expected, 0 if none */
    uint32_t             stream_generation; /* increments per stream begin and per abort */

    uint8_t  region;
    uint16_t offset;          /* current cursor relative to region start */
    uint16_t remaining;       /* bytes remaining in active stream */
    uint16_t expected_seq;

    uint8_t  last_error_code;
    uint8_t  last_error_detail;

    uint64_t deadline_us;
    uint64_t session_timeout_us;
    bool     abort_pending;

    /* Counters useful for tests and telemetry. */
    uint32_t stat_data_packets;
    uint32_t stat_late_ignored;
    uint32_t stat_aborts;
    uint32_t stat_errors_received;
    uint32_t stat_busy_received;
} FusionStatePort_t;

/* Default session watchdogs from the architecture doc. */
#define FUSION_STATE_PORT_TIMEOUT_115K_US     (30ull * 1000ull * 1000ull)
#define FUSION_STATE_PORT_TIMEOUT_1M_US       (5ull * 1000ull * 1000ull)

void FusionStatePort_Init(FusionStatePort_t *p,
                          const FusionStatePortTransport_t *transport);
void FusionStatePort_Reset(FusionStatePort_t *p);

void FusionStatePort_SetSessionTimeoutUs(FusionStatePort_t *p, uint64_t timeout_us);

/* Session control. */
FusionStatePortResult_t FusionStatePort_BeginSave(FusionStatePort_t *p, uint8_t flags);
FusionStatePortResult_t FusionStatePort_BeginLoad(FusionStatePort_t *p, uint8_t flags);
FusionStatePortResult_t FusionStatePort_EndSession(FusionStatePort_t *p);

/* Stream control. length is region-local bytes; offset is region-local. */
FusionStatePortResult_t FusionStatePort_BeginReadStream(FusionStatePort_t *p,
                                                        uint8_t region,
                                                        uint16_t offset,
                                                        uint16_t length);
FusionStatePortResult_t FusionStatePort_BeginWriteStream(FusionStatePort_t *p,
                                                         uint8_t region,
                                                         uint16_t offset,
                                                         uint16_t length);

/*
 * Push the next chunk of write-stream payload. data_len must be <= 8.
 * On success the frame is sent and expected_seq advances. Caller is
 * responsible for honoring abort_pending between calls.
 */
FusionStatePortResult_t FusionStatePort_PushWriteData(FusionStatePort_t *p,
                                                      const uint8_t *data,
                                                      size_t data_len);

/* Deliver a fully decoded frame from the RX side. */
FusionStatePortResult_t FusionStatePort_OnRxFrame(FusionStatePort_t *p,
                                                  const FusionV2Decoded_t *decoded);

/*
 * Async abort hook. Safe to call from RX task or watchdog. The state
 * machine sends ERROR(LocalAbort) to the FPGA, transitions to Aborted,
 * bumps the stream generation, and clears pending_op so subsequent late
 * responses are dropped.
 */
void FusionStatePort_RequestAbort(FusionStatePort_t *p, uint8_t code, uint8_t detail);

bool FusionStatePort_IsAbortPending(const FusionStatePort_t *p);

/* Watchdog tick. If now_us was provided and deadline elapsed, force an abort. */
void FusionStatePort_TickWatchdog(FusionStatePort_t *p);

#ifdef __cplusplus
}
#endif
