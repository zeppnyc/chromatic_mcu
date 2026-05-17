/*
 * Host self-test for the Fusion savestate Phase 1 MCU protocol skeleton.
 *
 * Compiles to a stand-alone binary on the host. Does not require ESP-IDF
 * or FreeRTOS. Links directly against:
 *   - main/fusion_savestate.c
 *   - main/fpga_state_port.c
 *   - main/savestate_storage.c
 *   - components/crc/crc8_sae_j1850.c
 *
 * Tests each requirement called out in the Phase 1 task brief:
 *   - STATE_DATA encode/decode roundtrip with seq=0 and 8 bytes
 *   - Seq mismatch abort
 *   - ERROR response abort
 *   - Late ACK_DONE ignored after abort/generation mismatch
 *   - game_id_v1 determinism
 *   - Storage header size/field sanity, two-phase commit lifecycle
 */

#include "fusion_savestate.h"
#include "fpga_state_port.h"
#include "savestate_storage.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail_count = 0;
static int g_pass_count = 0;

#define EXPECT(cond) do {                                              \
    if (cond) {                                                        \
        g_pass_count++;                                                \
    } else {                                                           \
        g_fail_count++;                                                \
        fprintf(stderr, "  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
    }                                                                  \
} while (0)

#define EXPECT_EQ_U(a, b) do {                                                       \
    const uint64_t _a = (uint64_t)(a);                                               \
    const uint64_t _b = (uint64_t)(b);                                               \
    if (_a == _b) {                                                                  \
        g_pass_count++;                                                              \
    } else {                                                                         \
        g_fail_count++;                                                              \
        fprintf(stderr, "  FAIL %s:%d  %s == %s  (got 0x%llx vs 0x%llx)\n",          \
                __FILE__, __LINE__, #a, #b,                                          \
                (unsigned long long)_a, (unsigned long long)_b);                     \
    }                                                                                \
} while (0)

/* --- Mock transport that records sent frames. --- */

typedef struct {
    uint8_t  frames[64][FUSION_V2_MAX_FRAME];
    uint8_t  lengths[64];
    uint32_t count;
    bool     fail_next;
    uint64_t now;
} MockTx_t;

static bool MockTxFrame(void *user, const uint8_t *frame, size_t len)
{
    MockTx_t *m = (MockTx_t *)user;
    if (m->fail_next) {
        m->fail_next = false;
        return false;
    }
    if (m->count >= 64u || len > FUSION_V2_MAX_FRAME) {
        return false;
    }
    memcpy(m->frames[m->count], frame, len);
    m->lengths[m->count] = (uint8_t)len;
    m->count++;
    return true;
}

static uint64_t MockNowUs(void *user)
{
    MockTx_t *m = (MockTx_t *)user;
    return m->now;
}

static void MockReset(MockTx_t *m)
{
    memset(m, 0, sizeof(*m));
}

/* --- Tests --- */

static void test_v2_frame_state_data_roundtrip(void)
{
    fprintf(stderr, "[test] STATE_DATA encode/decode roundtrip seq=0, 8 bytes\n");
    const uint8_t data[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78 };
    FusionV2Frame_t frame;
    EXPECT(FusionSavestate_BuildStateData(0u, data, 8u, &frame));
    /* Wire layout: 0x8F | 0x21 | 0x0A | seq_lo | seq_hi | 8 data bytes | crc */
    EXPECT_EQ_U(frame.length, 14u);
    EXPECT_EQ_U(frame.bytes[0], FUSION_V2_HEADER_MARKER);
    EXPECT_EQ_U(frame.bytes[1], (uint8_t)kFusionAddr_StateData);
    EXPECT_EQ_U(frame.bytes[2], 10u);
    EXPECT_EQ_U(frame.bytes[3], 0u); /* seq_lo */
    EXPECT_EQ_U(frame.bytes[4], 0u); /* seq_hi */
    EXPECT_EQ_U(frame.bytes[5], 0xDE);
    EXPECT_EQ_U(frame.bytes[12], 0x78);

    FusionV2Decoded_t decoded;
    EXPECT(FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &decoded));
    EXPECT_EQ_U(decoded.addr, (uint8_t)kFusionAddr_StateData);
    EXPECT_EQ_U(decoded.payload_len, 10u);
    EXPECT_EQ_U(decoded.data_seq, 0u);
    EXPECT_EQ_U(decoded.data_len, 8u);
    EXPECT_EQ_U(memcmp(decoded.data, data, 8), 0);
}

static void test_v2_frame_state_data_seq_nonzero(void)
{
    fprintf(stderr, "[test] STATE_DATA encode/decode seq=0xBEEF\n");
    const uint8_t data[3] = { 0x11, 0x22, 0x33 };
    FusionV2Frame_t frame;
    EXPECT(FusionSavestate_BuildStateData(0xBEEFu, data, 3u, &frame));

    FusionV2Decoded_t decoded;
    EXPECT(FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &decoded));
    EXPECT_EQ_U(decoded.data_seq, 0xBEEFu);
    EXPECT_EQ_U(decoded.data_len, 3u);
    EXPECT_EQ_U(memcmp(decoded.data, data, 3), 0);
}

static void test_v2_frame_decode_rejects_bad_crc(void)
{
    fprintf(stderr, "[test] DecodeV2Frame rejects corrupted CRC\n");
    FusionV2Frame_t frame;
    EXPECT(FusionSavestate_BuildBeginSave(0u, &frame));
    frame.bytes[frame.length - 1u] ^= 0xA5;
    FusionV2Decoded_t decoded;
    EXPECT(!FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &decoded));
}

static void test_v2_frame_decode_rejects_bad_marker(void)
{
    fprintf(stderr, "[test] DecodeV2Frame rejects bad marker / oversize / 7-bit addr\n");
    FusionV2Frame_t frame;
    EXPECT(FusionSavestate_BuildEndSession(&frame));
    frame.bytes[0] = 0x8A; /* V1 marker */
    FusionV2Decoded_t decoded;
    EXPECT(!FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &decoded));

    /* Address with bit 7 set must be rejected by builder. */
    EXPECT(!FusionSavestate_BuildV2Frame(0x80, NULL, 0, &frame));

    /* Payload length cap. */
    uint8_t big[FUSION_V2_MAX_PAYLOAD + 1] = {0};
    EXPECT(!FusionSavestate_BuildV2Frame(0x20, big, sizeof(big), &frame));
}

static void test_ctl_builders(void)
{
    fprintf(stderr, "[test] CTL builders produce decodable frames\n");
    FusionV2Frame_t frame;
    FusionV2Decoded_t d;

    EXPECT(FusionSavestate_BuildBeginSave(0x07u, &frame));
    EXPECT(FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &d));
    EXPECT_EQ_U(d.ctl_opcode, (uint8_t)kFusionOp_BeginSave);
    EXPECT_EQ_U(d.payload[1], 0x07u);

    EXPECT(FusionSavestate_BuildSeek(kFusionRegion_Wram, 0xCAFEu, &frame));
    EXPECT(FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &d));
    EXPECT_EQ_U(d.ctl_opcode, (uint8_t)kFusionOp_Seek);
    EXPECT_EQ_U(d.payload[1], (uint8_t)kFusionRegion_Wram);
    EXPECT_EQ_U(d.payload[2], 0xFEu);
    EXPECT_EQ_U(d.payload[3], 0xCAu);

    EXPECT(FusionSavestate_BuildReadStreamBegin(kFusionRegion_Hram, 0x0011u, 0x007Fu, &frame));
    EXPECT(FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &d));
    EXPECT_EQ_U(d.ctl_opcode, (uint8_t)kFusionOp_ReadStreamBegin);
    EXPECT_EQ_U(d.payload[1], (uint8_t)kFusionRegion_Hram);
    EXPECT_EQ_U(d.payload[2], 0x11u);
    EXPECT_EQ_U(d.payload[3], 0x00u);
    EXPECT_EQ_U(d.payload[4], 0x7Fu);
    EXPECT_EQ_U(d.payload[5], 0x00u);

    EXPECT(FusionSavestate_BuildReadStreamContinue(0x1234u, &frame));
    EXPECT(FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &d));
    EXPECT_EQ_U(d.ctl_opcode, (uint8_t)kFusionOp_ReadStreamContinue);
    EXPECT_EQ_U(d.payload[1], 0x34u);
    EXPECT_EQ_U(d.payload[2], 0x12u);

    EXPECT(FusionSavestate_BuildAckAccepted(kFusionOp_BeginLoad, &frame));
    EXPECT(FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &d));
    EXPECT_EQ_U(d.ctl_opcode, (uint8_t)kFusionOp_AckAccepted);
    EXPECT_EQ_U(d.ack_for_opcode, (uint8_t)kFusionOp_BeginLoad);

    EXPECT(FusionSavestate_BuildError(kFusionErr_SeqMismatch, 0x99u, &frame));
    EXPECT(FusionSavestate_DecodeV2Frame(frame.bytes, frame.length, &d));
    EXPECT_EQ_U(d.error_code, (uint8_t)kFusionErr_SeqMismatch);
    EXPECT_EQ_U(d.error_detail, 0x99u);
}

static void test_state_port_seq_mismatch_aborts(void)
{
    fprintf(stderr, "[test] state-port seq mismatch -> aborted, ERROR sent\n");
    MockTx_t mock;
    MockReset(&mock);

    FusionStatePortTransport_t tx = { .user = &mock, .tx_frame = MockTxFrame, .now_us = MockNowUs };
    FusionStatePort_t port;
    FusionStatePort_Init(&port, &tx);
    FusionStatePort_SetSessionTimeoutUs(&port, 0u); /* disable watchdog for this test */

    EXPECT_EQ_U(FusionStatePort_BeginSave(&port, 0u), kFusionStatePortRes_Ok);

    /* FPGA accepts session. */
    FusionV2Frame_t f;
    FusionV2Decoded_t d;
    EXPECT(FusionSavestate_BuildAckAccepted(kFusionOp_BeginSave, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Ok);

    EXPECT_EQ_U(FusionStatePort_BeginReadStream(&port, kFusionRegion_Hram, 0u, 16u),
                kFusionStatePortRes_Ok);
    EXPECT(FusionSavestate_BuildAckAccepted(kFusionOp_ReadStreamBegin, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Ok);

    /* expected_seq is 0; FPGA "sends" seq=1 instead. */
    const uint8_t bad[8] = {0};
    EXPECT(FusionSavestate_BuildStateData(1u, bad, 8u, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Aborted);

    EXPECT_EQ_U(port.session_state, (uint64_t)kFusionSession_Aborted);
    EXPECT(port.abort_pending);
    EXPECT_EQ_U(port.last_error_code, (uint8_t)kFusionErr_SeqMismatch);

    /* The state machine must have transmitted an ERROR frame to FPGA. */
    EXPECT(mock.count >= 3u);
    /* Last frame should be ERROR with code = SeqMismatch. */
    const uint8_t *last = mock.frames[mock.count - 1u];
    EXPECT_EQ_U(last[1], (uint8_t)kFusionAddr_StateCtl);
    EXPECT_EQ_U(last[3], (uint8_t)kFusionOp_Error);
    EXPECT_EQ_U(last[4], (uint8_t)kFusionErr_SeqMismatch);
}

static void test_state_port_error_response_aborts(void)
{
    fprintf(stderr, "[test] state-port ERROR response -> aborted\n");
    MockTx_t mock;
    MockReset(&mock);
    FusionStatePortTransport_t tx = { .user = &mock, .tx_frame = MockTxFrame, .now_us = MockNowUs };
    FusionStatePort_t port;
    FusionStatePort_Init(&port, &tx);
    FusionStatePort_SetSessionTimeoutUs(&port, 0u);

    EXPECT_EQ_U(FusionStatePort_BeginLoad(&port, 0u), kFusionStatePortRes_Ok);

    FusionV2Frame_t f;
    FusionV2Decoded_t d;
    EXPECT(FusionSavestate_BuildError(kFusionErr_NoSession, 0xAAu, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Aborted);

    EXPECT_EQ_U(port.session_state, (uint64_t)kFusionSession_Aborted);
    EXPECT_EQ_U(port.last_error_code, (uint8_t)kFusionErr_NoSession);
    EXPECT_EQ_U(port.last_error_detail, 0xAAu);
    EXPECT(port.stat_aborts >= 1u);
}

static void test_state_port_late_ack_ignored(void)
{
    fprintf(stderr, "[test] state-port late ACK_DONE after abort -> ignored\n");
    MockTx_t mock;
    MockReset(&mock);
    FusionStatePortTransport_t tx = { .user = &mock, .tx_frame = MockTxFrame, .now_us = MockNowUs };
    FusionStatePort_t port;
    FusionStatePort_Init(&port, &tx);
    FusionStatePort_SetSessionTimeoutUs(&port, 0u);

    EXPECT_EQ_U(FusionStatePort_BeginSave(&port, 0u), kFusionStatePortRes_Ok);

    FusionV2Frame_t f;
    FusionV2Decoded_t d;
    EXPECT(FusionSavestate_BuildAckAccepted(kFusionOp_BeginSave, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Ok);

    EXPECT_EQ_U(FusionStatePort_BeginReadStream(&port, kFusionRegion_Wram, 0u, 32u),
                kFusionStatePortRes_Ok);

    const uint32_t gen_before_abort = port.stream_generation;
    FusionStatePort_RequestAbort(&port,
                                 (uint8_t)kFusionErr_LocalAbort,
                                 (uint8_t)kFusionErr_LocalAbort);
    EXPECT(port.session_state == kFusionSession_Aborted);
    EXPECT(port.stream_generation > gen_before_abort);
    EXPECT_EQ_U(port.pending_op, 0u);

    const uint32_t late_before = port.stat_late_ignored;

    /* A late ACK_DONE for the prior stream must be ignored. */
    EXPECT(FusionSavestate_BuildAckDone(kFusionOp_ReadStreamBegin, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_LateIgnored);

    /* A late STATE_DATA must also be ignored, not applied. */
    const uint8_t junk[4] = {1,2,3,4};
    EXPECT(FusionSavestate_BuildStateData(0u, junk, 4u, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_LateIgnored);

    EXPECT(port.stat_late_ignored >= late_before + 2u);
}

static void test_state_port_ack_done_completes_stream(void)
{
    fprintf(stderr, "[test] state-port ACK_DONE for stream begin clears stream\n");
    MockTx_t mock;
    MockReset(&mock);
    FusionStatePortTransport_t tx = { .user = &mock, .tx_frame = MockTxFrame, .now_us = MockNowUs };
    FusionStatePort_t port;
    FusionStatePort_Init(&port, &tx);
    FusionStatePort_SetSessionTimeoutUs(&port, 0u);

    EXPECT_EQ_U(FusionStatePort_BeginSave(&port, 0u), kFusionStatePortRes_Ok);
    FusionV2Frame_t f;
    FusionV2Decoded_t d;
    EXPECT(FusionSavestate_BuildAckAccepted(kFusionOp_BeginSave, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Ok);

    EXPECT_EQ_U(FusionStatePort_BeginReadStream(&port, kFusionRegion_PaletteRam, 0u, 8u),
                kFusionStatePortRes_Ok);
    EXPECT(FusionSavestate_BuildAckAccepted(kFusionOp_ReadStreamBegin, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Ok);

    /* One DATA frame with seq=0, 8 bytes -> remaining hits 0. */
    const uint8_t data[8] = {0,1,2,3,4,5,6,7};
    EXPECT(FusionSavestate_BuildStateData(0u, data, 8u, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Ok);
    EXPECT_EQ_U(port.remaining, 0u);

    /* ACK_DONE for the stream begin clears stream and leaves session open. */
    EXPECT(FusionSavestate_BuildAckDone(kFusionOp_ReadStreamBegin, &f));
    EXPECT(FusionSavestate_DecodeV2Frame(f.bytes, f.length, &d));
    EXPECT_EQ_U(FusionStatePort_OnRxFrame(&port, &d), kFusionStatePortRes_Ok);
    EXPECT_EQ_U(port.stream_mode, (uint64_t)kFusionStream_None);
    EXPECT_EQ_U(port.session_state, (uint64_t)kFusionSession_OpenSave);

    EXPECT_EQ_U(FusionStatePort_EndSession(&port), kFusionStatePortRes_Ok);
    EXPECT_EQ_U(port.session_state, (uint64_t)kFusionSession_Idle);
}

static void test_state_port_watchdog_aborts(void)
{
    fprintf(stderr, "[test] state-port watchdog tick -> abort on deadline\n");
    MockTx_t mock;
    MockReset(&mock);
    FusionStatePortTransport_t tx = { .user = &mock, .tx_frame = MockTxFrame, .now_us = MockNowUs };
    FusionStatePort_t port;
    FusionStatePort_Init(&port, &tx);
    FusionStatePort_SetSessionTimeoutUs(&port, 1000u);
    mock.now = 0u;

    EXPECT_EQ_U(FusionStatePort_BeginSave(&port, 0u), kFusionStatePortRes_Ok);
    /* Not yet expired. */
    mock.now = 500u;
    FusionStatePort_TickWatchdog(&port);
    EXPECT_EQ_U(port.session_state, (uint64_t)kFusionSession_OpenSave);

    /* Deadline exceeded -> abort. */
    mock.now = 2000u;
    FusionStatePort_TickWatchdog(&port);
    EXPECT_EQ_U(port.session_state, (uint64_t)kFusionSession_Aborted);
    EXPECT_EQ_U(port.last_error_code, (uint8_t)kFusionErr_Timeout);
}

static void test_game_id_v1_deterministic(void)
{
    fprintf(stderr, "[test] game_id_v1 deterministic and distinguishes inputs\n");
    uint8_t header_a[FUSION_ROM_HEADER_BYTES];
    uint8_t header_b[FUSION_ROM_HEADER_BYTES];
    for (size_t i = 0; i < FUSION_ROM_HEADER_BYTES; ++i) {
        header_a[i] = (uint8_t)(i * 3u + 1u);
        header_b[i] = header_a[i];
    }
    const uint32_t a1 = FusionSavestate_ComputeGameIdV1(header_a);
    const uint32_t a2 = FusionSavestate_ComputeGameIdV1(header_a);
    EXPECT_EQ_U(a1, a2);

    header_b[0x14] ^= 0x01u;
    const uint32_t b1 = FusionSavestate_ComputeGameIdV1(header_b);
    EXPECT(a1 != b1);

    /* CRC32("123456789") == 0xCBF43926 (well-known CRC-32/ISO-HDLC test vector). */
    static const uint8_t kref[] = {'1','2','3','4','5','6','7','8','9'};
    EXPECT_EQ_U(FusionSavestate_Crc32(kref, sizeof(kref)), 0xCBF43926u);
}

static void test_storage_header_layout(void)
{
    fprintf(stderr, "[test] FusionStateHeader / RegionDirectory size and field sanity\n");
    EXPECT_EQ_U(sizeof(FusionStateHeader_t), 64u);
    EXPECT_EQ_U(sizeof(FusionRegionDirectory_t), 16u);
    /* Magic must round-trip as 'F','U','S','S' little-endian. */
    EXPECT_EQ_U(FUSION_SAVESTATE_MAGIC, 0x53535546u);
    const uint8_t *m = (const uint8_t *)&(uint32_t){FUSION_SAVESTATE_MAGIC};
    EXPECT_EQ_U(m[0], (uint8_t)'F');
    EXPECT_EQ_U(m[1], (uint8_t)'U');
    EXPECT_EQ_U(m[2], (uint8_t)'S');
    EXPECT_EQ_U(m[3], (uint8_t)'S');
}

static void test_storage_two_phase_commit(void)
{
    fprintf(stderr, "[test] storage two-phase commit lifecycle (mock)\n");
    enum { kSlots = 2, kSlotBytes = 1024 };
    uint8_t backing[kSlots * kSlotBytes];
    FusionStorageSlot_t slots[kSlots];
    FusionStorage_t s;
    EXPECT_EQ_U(FusionStorage_InitMock(&s, slots, kSlots, backing, sizeof(backing)),
                kFusionStorage_Ok);

    /* Stage slot 0. */
    EXPECT_EQ_U(FusionStorage_BeginInactiveSlotWrite(&s), kFusionStorage_Ok);
    const uint8_t hram[] = {1,2,3,4,5,6,7,8};
    EXPECT_EQ_U(FusionStorage_AppendRegionPayload(&s, hram, sizeof(hram)),
                kFusionStorage_Ok);

    FusionRegionDirectory_t dir = {
        .region_id = (uint8_t)kFusionRegion_Hram,
        .reserved  = {0},
        .offset    = sizeof(FusionStateHeader_t),
        .length    = sizeof(hram),
        .crc32     = FusionSavestate_Crc32(hram, sizeof(hram)),
    };
    FusionStateHeader_t hdr = {0};
    hdr.game_id_hash      = 0xDEADBEEFu;
    hdr.game_id_algorithm = (uint8_t)FUSION_GAME_ID_ALGORITHM_V1;
    memcpy(hdr.fpga_version, "abcdef12", 8);
    memcpy(hdr.mcu_version,  "12345678", 8);
    hdr.commit_generation = 42u;
    hdr.region_bitmap = 0x00000200u;
    memcpy(hdr.savestate_tag, "P470", 4);
    EXPECT_EQ_U(FusionStorage_StageHeader(&s, &hdr, &dir, 1u), kFusionStorage_Ok);

    /* Before commit, no slot is "newest valid". */
    uint32_t newest = 0u;
    uint32_t newest_gen = 0u;
    EXPECT_EQ_U(FusionStorage_FindNewestValidSlot(&s, &newest, &newest_gen),
                kFusionStorage_NoValidSlot);

    /* Commit and verify atomically. */
    EXPECT_EQ_U(FusionStorage_VerifyAndCommit(&s), kFusionStorage_Ok);
    EXPECT_EQ_U(FusionStorage_FindNewestValidSlot(&s, &newest, &newest_gen),
                kFusionStorage_Ok);
    EXPECT_EQ_U(newest_gen, 42u);

    /* Stage slot 1 with newer generation, leave it in-progress (simulate power loss
     * between StageHeader and VerifyAndCommit). */
    EXPECT_EQ_U(FusionStorage_BeginInactiveSlotWrite(&s), kFusionStorage_Ok);
    EXPECT_EQ_U(FusionStorage_AppendRegionPayload(&s, hram, sizeof(hram)),
                kFusionStorage_Ok);
    FusionStateHeader_t hdr2 = hdr;
    hdr2.commit_generation = 100u;
    EXPECT_EQ_U(FusionStorage_StageHeader(&s, &hdr2, &dir, 1u), kFusionStorage_Ok);
    /* No commit. Find newest valid -> still slot 0 / generation 42. */
    EXPECT_EQ_U(FusionStorage_FindNewestValidSlot(&s, &newest, &newest_gen),
                kFusionStorage_Ok);
    EXPECT_EQ_U(newest_gen, 42u);
    /* Tampered (Writing) header must not be selected as Valid. */
    FusionStateHeader_t reread;
    EXPECT_EQ_U(FusionStorage_ReadSlotHeader(&s, 1u, &reread), kFusionStorage_Ok);
    EXPECT_EQ_U(reread.commit_state, (uint8_t)kFusionCommit_Writing);

    /* Now finish the commit. */
    EXPECT_EQ_U(FusionStorage_VerifyAndCommit(&s), kFusionStorage_Ok);
    EXPECT_EQ_U(FusionStorage_FindNewestValidSlot(&s, &newest, &newest_gen),
                kFusionStorage_Ok);
    EXPECT_EQ_U(newest_gen, 100u);

    EXPECT_EQ_U(FusionStorage_InvalidateSlot(&s, newest), kFusionStorage_Ok);
    EXPECT_EQ_U(FusionStorage_FindNewestValidSlot(&s, &newest, &newest_gen),
                kFusionStorage_Ok);
    EXPECT_EQ_U(newest_gen, 42u);
    EXPECT_EQ_U(FusionStorage_InvalidateSlot(&s, newest), kFusionStorage_Ok);
    EXPECT_EQ_U(FusionStorage_FindNewestValidSlot(&s, &newest, &newest_gen),
                kFusionStorage_NoValidSlot);
}

static void test_storage_partition_init_deferred(void)
{
    fprintf(stderr, "[test] partition backend reports NotImplemented\n");
    FusionStorage_t s;
    EXPECT_EQ_U(FusionStorage_InitFromPartition(&s, "savestate"),
                kFusionStorage_NotImplemented);
}

static void test_pathe_load_payload_basic(void)
{
    fprintf(stderr, "[test] path-e load payload basic IME=1\n");
    FusionPathELoadInput_t s = {
        .saved_F = 0xA0u, .saved_A = 0x7Fu,
        .saved_C = 0x34u, .saved_B = 0x12u,
        .saved_E = 0x78u, .saved_D = 0x56u,
        .saved_L = 0xBCu, .saved_H = 0x9Au,
        .saved_SP = 0xDEADu,
        .saved_PC = 0x0150u,
        .saved_IFF = true,
        .halted_at_save = false,
    };
    uint8_t out[kPathE_RegionLength];
    EXPECT(FusionSavestate_BuildPathELoadPayload(&s, out));
    EXPECT_EQ_U(out[kPathE_LoadModeActive], 1u);
    EXPECT_EQ_U(out[kPathE_ScratchF], 0xA0u);
    EXPECT_EQ_U(out[kPathE_ScratchA], 0x7Fu);
    EXPECT_EQ_U(out[kPathE_ScratchC], 0x34u);
    EXPECT_EQ_U(out[kPathE_ScratchB], 0x12u);
    EXPECT_EQ_U(out[kPathE_ScratchE], 0x78u);
    EXPECT_EQ_U(out[kPathE_ScratchD], 0x56u);
    EXPECT_EQ_U(out[kPathE_ScratchL], 0xBCu);
    EXPECT_EQ_U(out[kPathE_ScratchH], 0x9Au);
    EXPECT_EQ_U(out[kPathE_SavedSpLo], 0xADu);
    EXPECT_EQ_U(out[kPathE_SavedSpHi], 0xDEu);
    EXPECT_EQ_U(out[kPathE_IffByte], FUSION_PATHE_IFF_BYTE_EI);  /* 0xFB EI */
    EXPECT_EQ_U(out[kPathE_SavedA], 0x7Fu);
    EXPECT_EQ_U(out[kPathE_SavedPcLo], 0x50u);
    EXPECT_EQ_U(out[kPathE_SavedPcHi], 0x01u);
    EXPECT_EQ_U(out[kPathE_GbresetRequest], 0u);
}

static void test_pathe_load_payload_ime0(void)
{
    fprintf(stderr, "[test] path-e load payload IME=0 selects NOP\n");
    FusionPathELoadInput_t s = {
        .saved_F = 0x00u, .saved_A = 0x00u,
        .saved_SP = 0xFFFEu, .saved_PC = 0x0150u,
        .saved_IFF = false, .halted_at_save = false,
    };
    uint8_t out[kPathE_RegionLength];
    EXPECT(FusionSavestate_BuildPathELoadPayload(&s, out));
    EXPECT_EQ_U(out[kPathE_IffByte], FUSION_PATHE_IFF_BYTE_NOP);  /* 0x00 NOP */
}

static void test_pathe_load_payload_halt_pc_minus1(void)
{
    fprintf(stderr, "[test] path-e load payload HALT applies PC-1\n");
    FusionPathELoadInput_t s = {
        .saved_PC = 0x0234u,         /* CPU was at instruction after HALT */
        .saved_IFF = true, .halted_at_save = true,
    };
    uint8_t out[kPathE_RegionLength];
    EXPECT(FusionSavestate_BuildPathELoadPayload(&s, out));
    /* Adjusted to 0x0233 = HALT opcode address. */
    EXPECT_EQ_U(out[kPathE_SavedPcLo], 0x33u);
    EXPECT_EQ_U(out[kPathE_SavedPcHi], 0x02u);
}

static void test_pathe_load_payload_rejects_pc0_halt(void)
{
    fprintf(stderr, "[test] path-e load payload rejects HALT at PC=0\n");
    FusionPathELoadInput_t s = {
        .saved_PC = 0x0000u,
        .halted_at_save = true,
    };
    uint8_t out[kPathE_RegionLength];
    EXPECT(!FusionSavestate_BuildPathELoadPayload(&s, out));
}

static void test_pathe_load_payload_null_inputs(void)
{
    fprintf(stderr, "[test] path-e load payload rejects null inputs\n");
    FusionPathELoadInput_t s = {0};
    uint8_t out[kPathE_RegionLength];
    EXPECT(!FusionSavestate_BuildPathELoadPayload(NULL, out));
    EXPECT(!FusionSavestate_BuildPathELoadPayload(&s, NULL));
}

static void test_pathe_extract_from_cpu_region(void)
{
    fprintf(stderr, "[test] path-e extract input from saved CPU region bytes\n");
    /* Construct a synthetic 40-byte CPU region with known field values. */
    uint8_t cpu[40] = {0};
    /* GBSE bytes 0-1 (don't care for this test) */
    cpu[2] = 0x34u;  /* C */
    cpu[3] = 0x78u;  /* E */
    cpu[4] = 0xBCu;  /* L */
    cpu[6] = 0x12u;  /* B */
    cpu[7] = 0x56u;  /* D */
    cpu[8] = 0x9Au;  /* H */
    cpu[10] = 0x50u; /* PC lo */
    cpu[11] = 0x01u; /* PC hi */
    cpu[19] = 0x7Fu; /* ACC = visible A */
    cpu[25] = 0xADu; /* SP lo */
    cpu[26] = 0xDEu; /* SP hi */
    /*
     * F = 0xA0 means Z=1, N=0, H=1, C=0; encoded in byte 28:
     *   F[7]=Z=1 -> bit 4 of byte28 = 1
     *   F[6]=N=0 -> bit 3 of byte28 = 0
     *   F[5]=H=1 -> bit 2 of byte28 = 1
     *   F[4]=C=0 -> bit 1 of byte28 = 0
     * byte28 lower 5 bits = 0b10100 = 0x14
     */
    cpu[28] = 0x14u;
    cpu[31] = 0x10u;  /* IFF=1 (bit 4), Halt=0 (bit 0) */

    FusionPathELoadInput_t state = {0};
    EXPECT(FusionSavestate_ExtractPathELoadInputFromCpuRegion(cpu, sizeof(cpu), &state));
    EXPECT_EQ_U(state.saved_C, 0x34u);
    EXPECT_EQ_U(state.saved_E, 0x78u);
    EXPECT_EQ_U(state.saved_L, 0xBCu);
    EXPECT_EQ_U(state.saved_B, 0x12u);
    EXPECT_EQ_U(state.saved_D, 0x56u);
    EXPECT_EQ_U(state.saved_H, 0x9Au);
    EXPECT_EQ_U(state.saved_A, 0x7Fu);
    EXPECT_EQ_U(state.saved_F, 0xA0u);
    EXPECT_EQ_U(state.saved_PC, 0x0150u);
    EXPECT_EQ_U(state.saved_SP, 0xDEADu);
    EXPECT(state.saved_IFF);
    EXPECT(!state.halted_at_save);

    /* Halt scenario: byte 31 bit 0 = 1, IFF still 1 */
    cpu[31] = 0x11u;
    EXPECT(FusionSavestate_ExtractPathELoadInputFromCpuRegion(cpu, sizeof(cpu), &state));
    EXPECT(state.halted_at_save);
    EXPECT(state.saved_IFF);
}

static void test_pathe_extract_rejects_short_input(void)
{
    fprintf(stderr, "[test] path-e extract rejects short input\n");
    uint8_t cpu[16] = {0};
    FusionPathELoadInput_t state = {0};
    EXPECT(!FusionSavestate_ExtractPathELoadInputFromCpuRegion(cpu, sizeof(cpu), &state));
    EXPECT(!FusionSavestate_ExtractPathELoadInputFromCpuRegion(NULL, 40, &state));
}

static void test_cart_feature_classification(void)
{
    fprintf(stderr, "[test] cart feature classification gates supported classes\n");

    FusionCartFeatureInfo_t info = FusionSavestate_ClassifyCartType(0x00u);
    EXPECT_EQ_U(info.support, kFusionCartSupport_Supported);
    EXPECT_EQ_U(info.cart_class, kFusionCartClass_RomOnly);
    EXPECT(FusionSavestate_CartTypeSupported(0x00u));

    info = FusionSavestate_ClassifyCartType(0x03u);
    EXPECT_EQ_U(info.support, kFusionCartSupport_Supported);
    EXPECT_EQ_U(info.cart_class, kFusionCartClass_Mbc1);

    info = FusionSavestate_ClassifyCartType(0x13u);
    EXPECT_EQ_U(info.support, kFusionCartSupport_Supported);
    EXPECT_EQ_U(info.cart_class, kFusionCartClass_Mbc3NoRtc);

    info = FusionSavestate_ClassifyCartType(0x1Bu);
    EXPECT_EQ_U(info.support, kFusionCartSupport_Supported);
    EXPECT_EQ_U(info.cart_class, kFusionCartClass_Mbc5NoSpecial);

    EXPECT_EQ_U(FusionSavestate_ClassifyCartType(0x10u).support,
                kFusionCartSupport_UnsupportedRtc);
    EXPECT_EQ_U(FusionSavestate_ClassifyCartType(0x1Eu).support,
                kFusionCartSupport_UnsupportedRumble);
    EXPECT_EQ_U(FusionSavestate_ClassifyCartType(0xFCu).support,
                kFusionCartSupport_UnsupportedCamera);
    EXPECT_EQ_U(FusionSavestate_ClassifyCartType(0x22u).support,
                kFusionCartSupport_UnsupportedAccelerometer);
    EXPECT_EQ_U(FusionSavestate_ClassifyCartType(0xFEu).support,
                kFusionCartSupport_UnsupportedIrSpecial);
    EXPECT(!FusionSavestate_CartTypeSupported(0x20u));
    EXPECT(strcmp(FusionSavestate_CartSupportReason(kFusionCartSupport_Supported),
                  "supported") == 0);
}

/* --- Full per-header-type matrix for the 2026-05-17 cart feature gate. --- */

typedef struct {
    uint8_t              cart_type;
    FusionCartClass_t    cart_class;
    FusionCartSupport_t  support;
    uint32_t             feature_flags;     /* exact required bitset */
    bool                 supported;
    bool                 requires_cart_state;
    const char          *class_name;
    const char          *support_token;
} CartCase_t;

static void test_cart_feature_classification_matrix(void)
{
    fprintf(stderr, "[test] cart feature classification full per-header matrix\n");

    static const CartCase_t cases[] = {
        /* Supported classes. */
        { 0x00u, kFusionCartClass_RomOnly,       kFusionCartSupport_Supported,
          FUSION_CART_FLAG_ROM_ONLY,
          true, false, "ROM_ONLY", "ALLOW_SUPPORTED" },
        { 0x01u, kFusionCartClass_Mbc1,          kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC1,
          true, true,  "MBC1",     "ALLOW_SUPPORTED" },
        { 0x02u, kFusionCartClass_Mbc1,          kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC1,
          true, true,  "MBC1",     "ALLOW_SUPPORTED" },
        { 0x03u, kFusionCartClass_Mbc1,          kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC1,
          true, true,  "MBC1",     "ALLOW_SUPPORTED" },
        { 0x11u, kFusionCartClass_Mbc3NoRtc,     kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC3,
          true, true,  "MBC3",     "ALLOW_SUPPORTED" },
        { 0x12u, kFusionCartClass_Mbc3NoRtc,     kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC3,
          true, true,  "MBC3",     "ALLOW_SUPPORTED" },
        { 0x13u, kFusionCartClass_Mbc3NoRtc,     kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC3,
          true, true,  "MBC3",     "ALLOW_SUPPORTED" },
        { 0x19u, kFusionCartClass_Mbc5NoSpecial, kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC5,
          true, true,  "MBC5",     "ALLOW_SUPPORTED" },
        { 0x1Au, kFusionCartClass_Mbc5NoSpecial, kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC5,
          true, true,  "MBC5",     "ALLOW_SUPPORTED" },
        { 0x1Bu, kFusionCartClass_Mbc5NoSpecial, kFusionCartSupport_Supported,
          FUSION_CART_FLAG_MBC5,
          true, true,  "MBC5",     "ALLOW_SUPPORTED" },

        /* Refused classes. */
        { 0x0Fu, kFusionCartClass_Mbc3WithRtc,    kFusionCartSupport_UnsupportedRtc,
          FUSION_CART_FLAG_MBC3 | FUSION_CART_FLAG_RTC,
          false, false, "MBC3_RTC",     "REJECT_RTC" },
        { 0x10u, kFusionCartClass_Mbc3WithRtc,    kFusionCartSupport_UnsupportedRtc,
          FUSION_CART_FLAG_MBC3 | FUSION_CART_FLAG_RTC,
          false, false, "MBC3_RTC",     "REJECT_RTC" },
        { 0x1Cu, kFusionCartClass_Mbc5WithRumble, kFusionCartSupport_UnsupportedRumble,
          FUSION_CART_FLAG_MBC5 | FUSION_CART_FLAG_RUMBLE,
          false, false, "MBC5_RUMBLE",  "REJECT_RUMBLE" },
        { 0x1Du, kFusionCartClass_Mbc5WithRumble, kFusionCartSupport_UnsupportedRumble,
          FUSION_CART_FLAG_MBC5 | FUSION_CART_FLAG_RUMBLE,
          false, false, "MBC5_RUMBLE",  "REJECT_RUMBLE" },
        { 0x1Eu, kFusionCartClass_Mbc5WithRumble, kFusionCartSupport_UnsupportedRumble,
          FUSION_CART_FLAG_MBC5 | FUSION_CART_FLAG_RUMBLE,
          false, false, "MBC5_RUMBLE",  "REJECT_RUMBLE" },
        { 0x20u, kFusionCartClass_Mbc6,           kFusionCartSupport_UnsupportedMbc6,
          FUSION_CART_FLAG_MBC6,
          false, false, "MBC6",         "REJECT_MBC6" },
        { 0x22u, kFusionCartClass_Mbc7Accel,      kFusionCartSupport_UnsupportedAccelerometer,
          FUSION_CART_FLAG_MBC7 | FUSION_CART_FLAG_ACCELEROMETER,
          false, false, "MBC7_ACCEL",   "REJECT_ACCELEROMETER" },
        { 0xFCu, kFusionCartClass_PocketCamera,   kFusionCartSupport_UnsupportedCamera,
          FUSION_CART_FLAG_CAMERA,
          false, false, "POCKET_CAMERA","REJECT_CAMERA" },
        { 0xFEu, kFusionCartClass_HuC3,           kFusionCartSupport_UnsupportedIrSpecial,
          FUSION_CART_FLAG_HUC | FUSION_CART_FLAG_IR_SPECIAL,
          false, false, "HUC3",         "REJECT_IR_SPECIAL" },
        { 0xFFu, kFusionCartClass_HuC1IrBattery,  kFusionCartSupport_UnsupportedIrSpecial,
          FUSION_CART_FLAG_HUC | FUSION_CART_FLAG_IR_SPECIAL,
          false, false, "HUC1_IR_BATTERY", "REJECT_IR_SPECIAL" },

        /* A representative sample of unclassified types. */
        { 0x04u, kFusionCartClass_Unsupported,    kFusionCartSupport_UnsupportedMapper,
          FUSION_CART_FLAG_UNKNOWN_MAPPER,
          false, false, "UNKNOWN_MAPPER", "REJECT_UNKNOWN_MAPPER" },
        { 0x05u, kFusionCartClass_Unsupported,    kFusionCartSupport_UnsupportedMapper,
          FUSION_CART_FLAG_UNKNOWN_MAPPER,
          false, false, "UNKNOWN_MAPPER", "REJECT_UNKNOWN_MAPPER" },
        { 0x21u, kFusionCartClass_Unsupported,    kFusionCartSupport_UnsupportedMapper,
          FUSION_CART_FLAG_UNKNOWN_MAPPER,
          false, false, "UNKNOWN_MAPPER", "REJECT_UNKNOWN_MAPPER" },
        { 0xFDu, kFusionCartClass_Unsupported,    kFusionCartSupport_UnsupportedMapper,
          FUSION_CART_FLAG_UNKNOWN_MAPPER,
          false, false, "UNKNOWN_MAPPER", "REJECT_UNKNOWN_MAPPER" },
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const CartCase_t *c = &cases[i];
        FusionCartFeatureInfo_t info =
            FusionSavestate_ClassifyCartType(c->cart_type);

        EXPECT_EQ_U(info.cart_type,          c->cart_type);
        EXPECT_EQ_U(info.cart_class,         c->cart_class);
        EXPECT_EQ_U(info.support,            c->support);
        EXPECT_EQ_U(info.feature_flags,      c->feature_flags);
        EXPECT_EQ_U((uint8_t)info.supported, (uint8_t)c->supported);
        EXPECT_EQ_U((uint8_t)info.requires_cart_state,
                    (uint8_t)c->requires_cart_state);
        EXPECT_EQ_U((uint8_t)FusionSavestate_CartTypeSupported(c->cart_type),
                    (uint8_t)c->supported);

        const char *name  = FusionSavestate_CartClassName(info.cart_class);
        const char *token = FusionSavestate_CartSupportToken(info.support);
        EXPECT(name  != NULL && strcmp(name,  c->class_name)    == 0);
        EXPECT(token != NULL && strcmp(token, c->support_token) == 0);

        FusionCartLoadGate_t gate =
            FusionSavestate_EvaluateCartLoadGate(c->cart_type);
        EXPECT_EQ_U((uint8_t)gate.allow_load, (uint8_t)c->supported);
        EXPECT_EQ_U(gate.info.cart_class, c->cart_class);
        EXPECT_EQ_U(gate.info.support,    c->support);
        EXPECT(gate.reject_token != NULL &&
               strcmp(gate.reject_token, c->support_token) == 0);
        EXPECT(gate.message != NULL);
        /* The human-readable reason must agree with the support enum text. */
        EXPECT(gate.message ==
               FusionSavestate_CartSupportReason(gate.info.support));
    }
}

/*
 * Distinguishability checks: the spec calls out two MBC pairs that must NOT be
 * collapsed.  Each pair shares the parent mapper feature but differs only in
 * RTC/rumble presence, so the classifier must keep them on different support
 * tracks and different class names.
 */
static void test_cart_feature_distinguishes_mbc3_rtc_vs_no_rtc(void)
{
    fprintf(stderr, "[test] cart classifier distinguishes MBC3 RTC vs MBC3 no-RTC\n");

    FusionCartFeatureInfo_t no_rtc = FusionSavestate_ClassifyCartType(0x13u);
    FusionCartFeatureInfo_t rtc    = FusionSavestate_ClassifyCartType(0x10u);

    EXPECT(no_rtc.supported);
    EXPECT(!rtc.supported);
    EXPECT_EQ_U(no_rtc.cart_class, kFusionCartClass_Mbc3NoRtc);
    EXPECT_EQ_U(rtc.cart_class,    kFusionCartClass_Mbc3WithRtc);
    /* Both must carry the MBC3 family flag. */
    EXPECT_EQ_U(no_rtc.feature_flags & FUSION_CART_FLAG_MBC3, FUSION_CART_FLAG_MBC3);
    EXPECT_EQ_U(rtc.feature_flags    & FUSION_CART_FLAG_MBC3, FUSION_CART_FLAG_MBC3);
    /* RTC bit must be set only on the RTC variant. */
    EXPECT_EQ_U(no_rtc.feature_flags & FUSION_CART_FLAG_RTC, 0u);
    EXPECT_EQ_U(rtc.feature_flags    & FUSION_CART_FLAG_RTC, FUSION_CART_FLAG_RTC);

    EXPECT(strcmp(FusionSavestate_CartSupportToken(no_rtc.support),
                  "ALLOW_SUPPORTED") == 0);
    EXPECT(strcmp(FusionSavestate_CartSupportToken(rtc.support),
                  "REJECT_RTC") == 0);
}

static void test_cart_feature_distinguishes_mbc5_rumble_vs_no_rumble(void)
{
    fprintf(stderr, "[test] cart classifier distinguishes MBC5 rumble vs MBC5 no-rumble\n");

    FusionCartFeatureInfo_t no_rumble = FusionSavestate_ClassifyCartType(0x1Bu);
    FusionCartFeatureInfo_t rumble    = FusionSavestate_ClassifyCartType(0x1Eu);

    EXPECT(no_rumble.supported);
    EXPECT(!rumble.supported);
    EXPECT_EQ_U(no_rumble.cart_class, kFusionCartClass_Mbc5NoSpecial);
    EXPECT_EQ_U(rumble.cart_class,    kFusionCartClass_Mbc5WithRumble);
    EXPECT_EQ_U(no_rumble.feature_flags & FUSION_CART_FLAG_MBC5,    FUSION_CART_FLAG_MBC5);
    EXPECT_EQ_U(rumble.feature_flags    & FUSION_CART_FLAG_MBC5,    FUSION_CART_FLAG_MBC5);
    EXPECT_EQ_U(no_rumble.feature_flags & FUSION_CART_FLAG_RUMBLE,  0u);
    EXPECT_EQ_U(rumble.feature_flags    & FUSION_CART_FLAG_RUMBLE,  FUSION_CART_FLAG_RUMBLE);

    EXPECT(strcmp(FusionSavestate_CartSupportToken(no_rumble.support),
                  "ALLOW_SUPPORTED") == 0);
    EXPECT(strcmp(FusionSavestate_CartSupportToken(rumble.support),
                  "REJECT_RUMBLE") == 0);
}

static void test_cart_load_gate_refuses_all_unsupported(void)
{
    fprintf(stderr, "[test] cart load gate refuses every unsupported header type\n");

    static const uint8_t refused[] = {
        0x0Fu, 0x10u,                  /* MBC3 RTC */
        0x1Cu, 0x1Du, 0x1Eu,           /* MBC5 rumble */
        0x20u,                         /* MBC6 */
        0x22u,                         /* MBC7 accelerometer */
        0xFCu,                         /* Pocket Camera */
        0xFEu, 0xFFu,                  /* HuC3 / HuC1+IR+Battery */
        /* Sample of unclassified types. */
        0x04u, 0x05u, 0x06u, 0x07u,
        0x08u, 0x09u, 0x0Au, 0x0Bu,
        0x0Cu, 0x0Du, 0x0Eu, 0x14u,
        0x15u, 0x16u, 0x17u, 0x18u,
        0x1Fu, 0x21u, 0x23u, 0xFDu,
        0xA0u, 0xCCu,
    };

    for (size_t i = 0; i < sizeof(refused) / sizeof(refused[0]); ++i) {
        const uint8_t t = refused[i];
        FusionCartLoadGate_t gate = FusionSavestate_EvaluateCartLoadGate(t);
        EXPECT(!gate.allow_load);
        EXPECT(gate.info.supported == false);
        EXPECT(gate.info.requires_cart_state == false);
        EXPECT(gate.reject_token != NULL);
        /* No refused entry may use the ALLOW token. */
        EXPECT(strcmp(gate.reject_token, "ALLOW_SUPPORTED") != 0);
        /* No refused entry may carry the ROM_ONLY family flag. */
        EXPECT_EQ_U(gate.info.feature_flags & FUSION_CART_FLAG_ROM_ONLY, 0u);
    }
}

static void test_cart_load_gate_allows_supported_subset(void)
{
    fprintf(stderr, "[test] cart load gate allows every supported header type\n");

    static const uint8_t allowed[] = {
        0x00u,
        0x01u, 0x02u, 0x03u,
        0x11u, 0x12u, 0x13u,
        0x19u, 0x1Au, 0x1Bu,
    };

    for (size_t i = 0; i < sizeof(allowed) / sizeof(allowed[0]); ++i) {
        const uint8_t t = allowed[i];
        FusionCartLoadGate_t gate = FusionSavestate_EvaluateCartLoadGate(t);
        EXPECT(gate.allow_load);
        EXPECT(gate.info.supported);
        EXPECT(gate.reject_token != NULL &&
               strcmp(gate.reject_token, "ALLOW_SUPPORTED") == 0);
        /* ROM_ONLY does not need cart/MBC state; every other supported class does. */
        if (t == 0x00u) {
            EXPECT(!gate.info.requires_cart_state);
        } else {
            EXPECT(gate.info.requires_cart_state);
        }
    }
}

/* PathE product package gate selftests live in fusion_pathe_pkg_selftest.c.
 * It folds its pass/fail counts into the values passed in. */
void RunFusionPathePkgSelftests(int *pass_out, int *fail_out);

int main(void)
{
    fprintf(stderr, "Fusion savestate Phase 1 host self-test\n");

    test_v2_frame_state_data_roundtrip();
    test_v2_frame_state_data_seq_nonzero();
    test_v2_frame_decode_rejects_bad_crc();
    test_v2_frame_decode_rejects_bad_marker();
    test_ctl_builders();
    test_state_port_seq_mismatch_aborts();
    test_state_port_error_response_aborts();
    test_state_port_late_ack_ignored();
    test_state_port_ack_done_completes_stream();
    test_state_port_watchdog_aborts();
    test_game_id_v1_deterministic();
    test_storage_header_layout();
    test_storage_two_phase_commit();
    test_storage_partition_init_deferred();
    test_pathe_load_payload_basic();
    test_pathe_load_payload_ime0();
    test_pathe_load_payload_halt_pc_minus1();
    test_pathe_load_payload_rejects_pc0_halt();
    test_pathe_load_payload_null_inputs();
    test_pathe_extract_from_cpu_region();
    test_pathe_extract_rejects_short_input();
    test_cart_feature_classification();
    test_cart_feature_classification_matrix();
    test_cart_feature_distinguishes_mbc3_rtc_vs_no_rtc();
    test_cart_feature_distinguishes_mbc5_rumble_vs_no_rumble();
    test_cart_load_gate_refuses_all_unsupported();
    test_cart_load_gate_allows_supported_subset();

    /* PathE product package gate (preparatory; no FPGA dependency). */
    RunFusionPathePkgSelftests(&g_pass_count, &g_fail_count);

    fprintf(stderr, "\n%d passed, %d failed\n", g_pass_count, g_fail_count);
    return g_fail_count == 0 ? 0 : 1;
}
