#include "fpga_state_port.h"

#include <string.h>

/*
 * MCU-side state-port session/stream state machine — implementation.
 *
 * Pure C; no FreeRTOS or ESP-IDF symbols. The transport layer is
 * injected via callbacks so the same object compiles under both the
 * firmware build and the host self-test.
 */

static bool TxFrame(FusionStatePort_t *p, const FusionV2Frame_t *f)
{
    if (p->transport.tx_frame == NULL) {
        return false;
    }
    return p->transport.tx_frame(p->transport.user, f->bytes, f->length);
}

static void RefreshDeadline(FusionStatePort_t *p)
{
    if (p->transport.now_us != NULL && p->session_timeout_us > 0u) {
        p->deadline_us = p->transport.now_us(p->transport.user) + p->session_timeout_us;
    } else {
        p->deadline_us = 0u;
    }
}

static void ClearStream(FusionStatePort_t *p)
{
    p->stream_mode      = kFusionStream_None;
    p->stream_accepted  = false;
    p->pending_op       = 0u;
    p->region           = 0u;
    p->offset           = 0u;
    p->remaining        = 0u;
    p->expected_seq     = 0u;
}

static void EnterAborted(FusionStatePort_t *p, uint8_t code, uint8_t detail)
{
    if (p->session_state != kFusionSession_Aborted) {
        p->stat_aborts++;
    }
    p->session_state    = kFusionSession_Aborted;
    p->abort_pending    = true;
    p->last_error_code  = code;
    p->last_error_detail = detail;
    p->stream_generation++;
    ClearStream(p);
}

void FusionStatePort_Init(FusionStatePort_t *p,
                          const FusionStatePortTransport_t *transport)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    if (transport != NULL) {
        p->transport = *transport;
    }
    p->session_timeout_us = FUSION_STATE_PORT_TIMEOUT_115K_US;
}

void FusionStatePort_Reset(FusionStatePort_t *p)
{
    if (p == NULL) {
        return;
    }
    const FusionStatePortTransport_t t = p->transport;
    const uint64_t timeout = p->session_timeout_us;
    memset(p, 0, sizeof(*p));
    p->transport = t;
    p->session_timeout_us = timeout;
}

void FusionStatePort_SetSessionTimeoutUs(FusionStatePort_t *p, uint64_t timeout_us)
{
    if (p == NULL) {
        return;
    }
    p->session_timeout_us = timeout_us;
}

bool FusionStatePort_IsAbortPending(const FusionStatePort_t *p)
{
    return (p != NULL) && p->abort_pending;
}

void FusionStatePort_RequestAbort(FusionStatePort_t *p, uint8_t code, uint8_t detail)
{
    if (p == NULL) {
        return;
    }
    if (p->session_state == kFusionSession_Idle) {
        return;
    }
    /*
     * Send ERROR(LocalAbort) to FPGA so its ABORTED state engages even if
     * MCU-side bytes already in the UART FIFO continue to leave the chip.
     * Any tx failure is non-fatal here: the caller is already aborting.
     */
    FusionV2Frame_t f;
    if (FusionSavestate_BuildError(code, detail, &f)) {
        (void)TxFrame(p, &f);
    }
    EnterAborted(p, code, detail);
}

void FusionStatePort_TickWatchdog(FusionStatePort_t *p)
{
    if (p == NULL || p->session_state == kFusionSession_Idle) {
        return;
    }
    if (p->transport.now_us == NULL || p->deadline_us == 0u) {
        return;
    }
    const uint64_t now = p->transport.now_us(p->transport.user);
    if (now >= p->deadline_us) {
        FusionStatePort_RequestAbort(p,
                                     (uint8_t)kFusionErr_Timeout,
                                     (uint8_t)kFusionErr_LocalAbort);
    }
}

/* ---- Session control ---- */

static FusionStatePortResult_t BeginSession(FusionStatePort_t *p,
                                            FusionSessionState_t target,
                                            uint8_t opcode,
                                            uint8_t flags)
{
    if (p == NULL) {
        return kFusionStatePortRes_BadArg;
    }
    if (p->session_state != kFusionSession_Idle) {
        return kFusionStatePortRes_NotIdle;
    }
    FusionV2Frame_t f;
    bool built = (opcode == (uint8_t)kFusionOp_BeginSave)
        ? FusionSavestate_BuildBeginSave(flags, &f)
        : FusionSavestate_BuildBeginLoad(flags, &f);
    if (!built) {
        return kFusionStatePortRes_BadArg;
    }
    if (!TxFrame(p, &f)) {
        return kFusionStatePortRes_TxFailed;
    }
    p->session_state    = target;
    p->abort_pending    = false;
    p->stat_aborts      = p->stat_aborts; /* untouched */
    p->pending_op       = opcode;
    p->stream_generation++;
    RefreshDeadline(p);
    return kFusionStatePortRes_Ok;
}

FusionStatePortResult_t FusionStatePort_BeginSave(FusionStatePort_t *p, uint8_t flags)
{
    return BeginSession(p, kFusionSession_OpenSave,
                        (uint8_t)kFusionOp_BeginSave, flags);
}

FusionStatePortResult_t FusionStatePort_BeginLoad(FusionStatePort_t *p, uint8_t flags)
{
    return BeginSession(p, kFusionSession_OpenLoad,
                        (uint8_t)kFusionOp_BeginLoad, flags);
}

FusionStatePortResult_t FusionStatePort_EndSession(FusionStatePort_t *p)
{
    if (p == NULL) {
        return kFusionStatePortRes_BadArg;
    }
    if (p->session_state == kFusionSession_Idle) {
        return kFusionStatePortRes_NoSession;
    }
    FusionV2Frame_t f;
    if (FusionSavestate_BuildEndSession(&f)) {
        (void)TxFrame(p, &f);
    }
    p->session_state    = kFusionSession_Idle;
    p->abort_pending    = false;
    p->deadline_us      = 0u;
    p->stream_generation++;
    ClearStream(p);
    return kFusionStatePortRes_Ok;
}

/* ---- Stream control ---- */

static FusionStatePortResult_t BeginStream(FusionStatePort_t *p,
                                           FusionStreamMode_t mode,
                                           uint8_t opcode,
                                           uint8_t region,
                                           uint16_t offset,
                                           uint16_t length)
{
    if (p == NULL) {
        return kFusionStatePortRes_BadArg;
    }
    if (p->session_state != kFusionSession_OpenSave
     && p->session_state != kFusionSession_OpenLoad) {
        return kFusionStatePortRes_NoSession;
    }
    if (p->stream_mode != kFusionStream_None) {
        return kFusionStatePortRes_StreamNotActive; /* stream already in flight */
    }
    if (length == 0u) {
        return kFusionStatePortRes_BadArg;
    }
    FusionV2Frame_t f;
    bool built = (mode == kFusionStream_Read)
        ? FusionSavestate_BuildReadStreamBegin(region, offset, length, &f)
        : FusionSavestate_BuildWriteStreamBegin(region, offset, length, &f);
    if (!built) {
        return kFusionStatePortRes_BadArg;
    }
    if (!TxFrame(p, &f)) {
        return kFusionStatePortRes_TxFailed;
    }

    p->stream_mode      = mode;
    p->stream_accepted  = false;
    p->pending_op       = opcode;
    p->region           = region;
    p->offset           = offset;
    p->remaining        = length;
    p->expected_seq     = 0u;
    p->stream_generation++;
    RefreshDeadline(p);
    return kFusionStatePortRes_Ok;
}

FusionStatePortResult_t FusionStatePort_BeginReadStream(FusionStatePort_t *p,
                                                        uint8_t region,
                                                        uint16_t offset,
                                                        uint16_t length)
{
    return BeginStream(p, kFusionStream_Read,
                       (uint8_t)kFusionOp_ReadStreamBegin,
                       region, offset, length);
}

FusionStatePortResult_t FusionStatePort_BeginWriteStream(FusionStatePort_t *p,
                                                         uint8_t region,
                                                         uint16_t offset,
                                                         uint16_t length)
{
    return BeginStream(p, kFusionStream_Write,
                       (uint8_t)kFusionOp_WriteStreamBegin,
                       region, offset, length);
}

FusionStatePortResult_t FusionStatePort_PushWriteData(FusionStatePort_t *p,
                                                      const uint8_t *data,
                                                      size_t data_len)
{
    if (p == NULL) {
        return kFusionStatePortRes_BadArg;
    }
    if (p->stream_mode != kFusionStream_Write) {
        return kFusionStatePortRes_StreamWrongMode;
    }
    if (!p->stream_accepted) {
        /*
         * Allow MCU to enqueue first DATA before ACK_ACCEPTED arrives only
         * if the firmware decides to pipeline; for now require accept first
         * to keep abort fences clean.
         */
        return kFusionStatePortRes_StreamNotActive;
    }
    if (p->abort_pending || p->session_state == kFusionSession_Aborted) {
        return kFusionStatePortRes_Aborted;
    }
    if (data_len == 0u || data_len > FUSION_STATE_DATA_MAX_DATA) {
        return kFusionStatePortRes_DataTooLong;
    }
    if (data_len > p->remaining) {
        FusionStatePort_RequestAbort(p,
                                     (uint8_t)kFusionErr_LenMismatch,
                                     (uint8_t)kFusionErr_LocalAbort);
        return kFusionStatePortRes_StreamExhausted;
    }

    FusionV2Frame_t f;
    if (!FusionSavestate_BuildStateData(p->expected_seq, data, data_len, &f)) {
        return kFusionStatePortRes_BadArg;
    }
    if (!TxFrame(p, &f)) {
        return kFusionStatePortRes_TxFailed;
    }
    p->expected_seq      = (uint16_t)(p->expected_seq + 1u);
    p->offset            = (uint16_t)(p->offset + data_len);
    p->remaining         = (uint16_t)(p->remaining - data_len);
    p->stat_data_packets++;
    RefreshDeadline(p);
    return kFusionStatePortRes_Ok;
}

/* ---- RX delivery ---- */

static FusionStatePortResult_t HandleStateData(FusionStatePort_t *p,
                                               const FusionV2Decoded_t *d)
{
    if (p->stream_mode != kFusionStream_Read) {
        /* DATA can only arrive on a Read stream. Drop without aborting if
         * the session isn't expecting it (e.g. crossed wire on a Write). */
        p->stat_late_ignored++;
        return kFusionStatePortRes_LateIgnored;
    }
    if (!p->stream_accepted) {
        /* Doc allows FPGA to begin emitting DATA only after ACK_ACCEPTED.
         * Treat early DATA as protocol violation. */
        FusionStatePort_RequestAbort(p,
                                     (uint8_t)kFusionErr_BadPayload,
                                     (uint8_t)kFusionOp_AckAccepted);
        return kFusionStatePortRes_Aborted;
    }
    if (d->data_seq != p->expected_seq) {
        FusionStatePort_RequestAbort(p,
                                     (uint8_t)kFusionErr_SeqMismatch,
                                     (uint8_t)(d->data_seq & 0xFFu));
        return kFusionStatePortRes_Aborted;
    }
    if (d->data_len == 0u || d->data_len > p->remaining) {
        FusionStatePort_RequestAbort(p,
                                     (uint8_t)kFusionErr_LenMismatch,
                                     d->data_len);
        return kFusionStatePortRes_Aborted;
    }
    p->expected_seq  = (uint16_t)(p->expected_seq + 1u);
    p->offset        = (uint16_t)(p->offset + d->data_len);
    p->remaining     = (uint16_t)(p->remaining - d->data_len);
    p->stat_data_packets++;
    RefreshDeadline(p);
    return kFusionStatePortRes_Ok;
}

FusionStatePortResult_t FusionStatePort_OnRxFrame(FusionStatePort_t *p,
                                                  const FusionV2Decoded_t *d)
{
    if (p == NULL || d == NULL) {
        return kFusionStatePortRes_BadArg;
    }
    if (p->session_state == kFusionSession_Idle) {
        p->stat_late_ignored++;
        return kFusionStatePortRes_LateIgnored;
    }
    if (p->session_state == kFusionSession_Aborted) {
        /* In Aborted state we keep the session alive only long enough for
         * the caller to send END_SESSION. Drop everything else.
         *
         * Per architecture doc:
         *   "MCU clears its RX queue after abort and ignores any late
         *    ACK_DONE from the failed stream generation."
         */
        p->stat_late_ignored++;
        return kFusionStatePortRes_LateIgnored;
    }
    if (d->addr == (uint8_t)kFusionAddr_StateData) {
        return HandleStateData(p, d);
    }
    if (d->addr != (uint8_t)kFusionAddr_StateCtl) {
        /* STATE_EVENT (0x22) is reserved for Phase 2+; ignore for now. */
        return kFusionStatePortRes_LateIgnored;
    }

    switch (d->ctl_opcode) {
    case kFusionOp_AckAccepted:
        if (d->ack_for_opcode != p->pending_op) {
            p->stat_late_ignored++;
            return kFusionStatePortRes_LateIgnored;
        }
        if (p->stream_mode != kFusionStream_None) {
            p->stream_accepted = true;
        }
        RefreshDeadline(p);
        return kFusionStatePortRes_Ok;

    case kFusionOp_Busy:
        if (d->ack_for_opcode != p->pending_op) {
            p->stat_late_ignored++;
            return kFusionStatePortRes_LateIgnored;
        }
        p->stat_busy_received++;
        RefreshDeadline(p);
        return kFusionStatePortRes_Ok;

    case kFusionOp_AckDone:
        if (d->ack_for_opcode != p->pending_op) {
            p->stat_late_ignored++;
            return kFusionStatePortRes_LateIgnored;
        }
        if (p->stream_mode != kFusionStream_None) {
            ClearStream(p);
        } else {
            p->pending_op = 0u;
        }
        RefreshDeadline(p);
        return kFusionStatePortRes_Ok;

    case kFusionOp_Error:
        p->stat_errors_received++;
        EnterAborted(p, d->error_code, d->error_detail);
        return kFusionStatePortRes_Aborted;

    default:
        /* Unknown CTL opcode is unexpected in MCU's RX direction. Abort
         * loudly so the engineer sees a real protocol mismatch. */
        EnterAborted(p,
                     (uint8_t)kFusionErr_BadOpcode,
                     d->ctl_opcode);
        return kFusionStatePortRes_Aborted;
    }
}
