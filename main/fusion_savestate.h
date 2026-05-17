#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Fusion savestate protocol/format primitives.
 *
 * This header defines the wire format and on-flash format used by the
 * MCU-owned save/load architecture documented in
 * project-wiki/50_decisions/fusion-savestate-mcu-architecture.md.
 *
 * Pure C: no FreeRTOS, no ESP-IDF. Everything declared here is
 * compilable on the host for unit tests.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Magic 'FUSS' interpreted little-endian. */
#define FUSION_SAVESTATE_MAGIC          0x53535546u
#define FUSION_SAVESTATE_FORMAT_VERSION 1u
#define FUSION_GAME_ID_ALGORITHM_V1     1u
#define FUSION_VERSION_FIELD_LEN        8u
#define FUSION_ROM_HEADER_BYTES         0x50u /* ROM header bytes 0x100-0x14F */

/* V2 packet framing constants — must match fpga_common.h. */
#define FUSION_V2_HEADER_MARKER  0x8Fu
#define FUSION_V2_MAX_PAYLOAD    10u
#define FUSION_V2_FRAME_OVERHEAD 4u   /* header + addr + len + crc */
#define FUSION_V2_MAX_FRAME      (FUSION_V2_FRAME_OVERHEAD + FUSION_V2_MAX_PAYLOAD)

/* STATE_DATA payload layout: seq16 (2 bytes) + up to 8 data bytes. */
#define FUSION_STATE_DATA_HEADER_BYTES 2u
#define FUSION_STATE_DATA_MAX_DATA     8u

/* V2 addresses reserved for state-port. */
typedef enum {
    kFusionAddr_StateCtl   = 0x20u,
    kFusionAddr_StateData  = 0x21u,
    kFusionAddr_StateEvent = 0x22u, /* Phase 2+ placeholder; payload schema deferred */
} FusionAddr_t;

/* STATE_CTL opcodes — see architecture doc table. */
typedef enum {
    /* MCU -> FPGA */
    kFusionOp_BeginSave        = 0x01u,
    kFusionOp_BeginLoad        = 0x02u,
    kFusionOp_EndSession       = 0x03u,
    kFusionOp_Seek             = 0x04u,
    kFusionOp_ReadNext         = 0x05u,
    kFusionOp_ReadStreamBegin  = 0x06u,
    kFusionOp_WriteStreamBegin = 0x07u,
    kFusionOp_WriteCommit      = 0x08u,
    kFusionOp_ReadStreamContinue = 0x09u,
    /*
     * Phase 4.8c v2: mixed paused session.  The bridge keeps
     * session_pause asserted from BEGIN_TEST_RW until END_SESSION and
     * accepts both READ_STREAM_* and WRITE_STREAM_* in the same
     * session.  Used only by the strict bit-exact RAM round-trip smoke
     * harness; OP_BEGIN_SAVE / OP_BEGIN_LOAD semantics are unchanged.
     */
    kFusionOp_BeginTestRW      = 0x0Au,
    /* FPGA -> MCU */
    kFusionOp_AckAccepted      = 0x10u,
    kFusionOp_Busy             = 0x11u,
    kFusionOp_Error            = 0x12u,
    kFusionOp_AckDone          = 0x13u,
} FusionOpcode_t;

/* Error codes carried in ERROR(opcode=0x12, code, detail). */
typedef enum {
    kFusionErr_None             = 0x00u,
    kFusionErr_BadOpcode        = 0x01u,
    kFusionErr_BadPayload       = 0x02u,
    kFusionErr_NoSession        = 0x03u,
    kFusionErr_StreamExists     = 0x04u,
    kFusionErr_NoStream         = 0x05u,
    kFusionErr_SeqMismatch      = 0x06u,
    kFusionErr_LenMismatch      = 0x07u,
    kFusionErr_OffsetOutOfRange = 0x08u,
    kFusionErr_InvalidRegion    = 0x09u,
    kFusionErr_Aborted          = 0x0Au,
    kFusionErr_Timeout          = 0x0Bu,
    kFusionErr_Internal         = 0x0Cu,
    kFusionErr_LocalAbort       = 0x0Du, /* MCU-local: aborted from this side */
} FusionErrorCode_t;

/* Region IDs (architecture doc Phase 4 plan — names, not adapter availability).
 * Region 0xFE is retained only for historical PathE-stub compatibility; the
 * current load path writes required regions 0x01/0x02/0x03/0x09/0x0C.
 */
typedef enum {
    kFusionRegion_Header     = 0x00u,
    kFusionRegion_CoreScalar = 0x01u,
    kFusionRegion_CpuRegs    = 0x02u,
    kFusionRegion_Timer      = 0x03u,
    kFusionRegion_Hdma       = 0x04u,
    kFusionRegion_PpuRegs    = 0x05u,
    kFusionRegion_PaletteRam = 0x06u,
    kFusionRegion_ApuRegs    = 0x07u,
    kFusionRegion_WaveRam    = 0x08u,
    kFusionRegion_Hram       = 0x09u,
    kFusionRegion_Oam        = 0x0Au,
    kFusionRegion_Vram       = 0x0Bu,
    kFusionRegion_Wram       = 0x0Cu,
    kFusionRegion_RomHeader  = 0x0Du,
    kFusionRegion_CartShadow = 0x0Eu,
    kFusionRegion_CartRam    = 0x0Fu,

    /*
     * Legacy PathE-stub load mode region. 16 bytes:
     *   idx 0:    load_mode_active (data[0])
     *   idx 1-8:  scratch[0-7] = saved_F, saved_A, saved_C, saved_B,
     *                            saved_E, saved_D, saved_L, saved_H
     *   idx 9-10: saved_SP lo / hi
     *   idx 11:   iff_byte (FB if saved_IFF=1, else 00)
     *   idx 12:   saved_a (cart-override re-restore after FF50 clobber)
     *   idx 13-14: saved_PC lo / hi
     *   idx 15:   gbreset_request (data[0]=1 hold reset, =0 release)
     *
     * Read at the same indices (FPGA generates pathe_byte combinationally).
     */
    kFusionRegion_PathELoad  = 0xFEu,
} FusionRegion_t;

/* Cartridge feature support is based on ROM header byte 0x0147. */
typedef enum {
    kFusionCartSupport_Supported = 0,
    kFusionCartSupport_UnsupportedRtc,
    kFusionCartSupport_UnsupportedRumble,
    kFusionCartSupport_UnsupportedCamera,
    kFusionCartSupport_UnsupportedAccelerometer,
    kFusionCartSupport_UnsupportedIrSpecial,
    kFusionCartSupport_UnsupportedMapper,
    kFusionCartSupport_UnsupportedMbc6,
} FusionCartSupport_t;

typedef enum {
    kFusionCartClass_RomOnly = 0,
    kFusionCartClass_Mbc1,
    kFusionCartClass_Mbc3NoRtc,
    kFusionCartClass_Mbc5NoSpecial,
    kFusionCartClass_Mbc3WithRtc,
    kFusionCartClass_Mbc5WithRumble,
    kFusionCartClass_Mbc6,
    kFusionCartClass_Mbc7Accel,
    kFusionCartClass_PocketCamera,
    kFusionCartClass_HuC3,
    kFusionCartClass_HuC1IrBattery,
    kFusionCartClass_Unsupported,
} FusionCartClass_t;

#define FUSION_CART_FLAG_ROM_ONLY       (1u << 0)
#define FUSION_CART_FLAG_MBC1           (1u << 1)
#define FUSION_CART_FLAG_MBC3           (1u << 2)
#define FUSION_CART_FLAG_MBC5           (1u << 3)
#define FUSION_CART_FLAG_RTC            (1u << 4)
#define FUSION_CART_FLAG_RUMBLE         (1u << 5)
#define FUSION_CART_FLAG_CAMERA         (1u << 6)
#define FUSION_CART_FLAG_ACCELEROMETER  (1u << 7)
#define FUSION_CART_FLAG_IR_SPECIAL     (1u << 8)
#define FUSION_CART_FLAG_UNKNOWN_MAPPER (1u << 9)
#define FUSION_CART_FLAG_MBC6           (1u << 10)
#define FUSION_CART_FLAG_MBC7           (1u << 11)
#define FUSION_CART_FLAG_HUC            (1u << 12)

typedef struct {
    uint8_t cart_type;
    FusionCartClass_t cart_class;
    FusionCartSupport_t support;
    uint32_t feature_flags;
    /*
     * True when a Batch B cart/MBC state region is required to restore this
     * class.  Always false today because Batch B FPGA RTL is not implemented;
     * supported MBC classes set this true to drive future load-gate logic that
     * will refuse to claim PASS once Batch B regions become required.
     */
    bool requires_cart_state;
    /* Convenience: support == kFusionCartSupport_Supported. */
    bool supported;
} FusionCartFeatureInfo_t;

/*
 * Load-gate decision for a ROM header cartridge type byte.  This is the
 * MCU/host-side "should we even attempt to load" check.  Today it has no
 * FPGA-side enforcement counterpart: it only encodes refuse/warn policy so the
 * host can fail fast on unsupported cartridges instead of pushing a state
 * stream into a build that cannot apply it.
 *
 * reject_token is a stable machine-parseable string suitable for log scraping;
 * see FusionSavestate_CartSupportToken().  message is the human-readable form.
 */
typedef struct {
    bool allow_load;
    FusionCartFeatureInfo_t info;
    const char *reject_token;
    const char *message;
} FusionCartLoadGate_t;

/* Legacy PathE-stub region 0xFE byte indices. */
typedef enum {
    kPathE_LoadModeActive = 0u,
    kPathE_ScratchF       = 1u,
    kPathE_ScratchA       = 2u,
    kPathE_ScratchC       = 3u,
    kPathE_ScratchB       = 4u,
    kPathE_ScratchE       = 5u,
    kPathE_ScratchD       = 6u,
    kPathE_ScratchL       = 7u,
    kPathE_ScratchH       = 8u,
    kPathE_SavedSpLo      = 9u,
    kPathE_SavedSpHi      = 10u,
    kPathE_IffByte        = 11u,
    kPathE_SavedA         = 12u,
    kPathE_SavedPcLo      = 13u,
    kPathE_SavedPcHi      = 14u,
    kPathE_GbresetRequest = 15u,
    kPathE_RegionLength   = 16u,
} FusionPathERegionIdx_t;

/* Legacy PathE-stub cart override IFF byte values. */
#define FUSION_PATHE_IFF_BYTE_EI    0xFBu  /* EI opcode if saved_IFF=1 */
#define FUSION_PATHE_IFF_BYTE_NOP   0x00u  /* NOP opcode if saved_IFF=0 */

/* Slot commit-state byte values (atomic single-byte transition). */
typedef enum {
    kFusionCommit_Erased  = 0xFFu,
    kFusionCommit_Writing = 0xA1u,
    kFusionCommit_Valid   = 0x5Cu,
    kFusionCommit_Invalid = 0x00u,
} FusionCommitState_t;

/*
 * On-flash header for one savestate slot.
 * Field layout follows the architecture doc Storage Format section.
 * All multibyte fields are little-endian.
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;                  /* FUSION_SAVESTATE_MAGIC */
    uint16_t format_version;
    uint16_t header_size;            /* sizeof(FusionStateHeader_t) */
    uint32_t total_size;             /* slot bytes consumed including this header */
    uint32_t game_id_hash;           /* per game_id_algorithm */
    uint8_t  game_id_algorithm;      /* FUSION_GAME_ID_ALGORITHM_V1 */
    uint8_t  reserved0[3];
    char     fpga_version[FUSION_VERSION_FIELD_LEN]; /* git short hash, NUL-padded */
    char     mcu_version[FUSION_VERSION_FIELD_LEN];  /* git short hash, NUL-padded */
    uint32_t region_table_offset;    /* relative to start of slot */
    uint32_t region_count;
    uint32_t region_bitmap;          /* capability bitmap captured from region 0 header */
    char     savestate_tag[4];       /* capability tag captured from region 0 header */
    uint32_t payload_crc32;          /* CRC32 over payload + region table */
    uint32_t commit_generation;      /* monotonic per write */
    uint8_t  commit_state;           /* FusionCommitState_t */
    uint8_t  reserved1[3];
} FusionStateHeader_t;

typedef struct __attribute__((packed)) {
    uint8_t  region_id;
    uint8_t  reserved[3];
    uint32_t offset;   /* relative to slot start */
    uint32_t length;
    uint32_t crc32;
} FusionRegionDirectory_t;

/* Built/decoded V2 frame container. */
typedef struct {
    uint8_t bytes[FUSION_V2_MAX_FRAME];
    uint8_t length;
} FusionV2Frame_t;

typedef struct {
    uint8_t  addr;
    uint8_t  payload[FUSION_V2_MAX_PAYLOAD];
    uint8_t  payload_len;

    /* Convenience parse for STATE_CTL. */
    uint8_t  ctl_opcode;       /* first payload byte if addr == STATE_CTL, else 0 */
    uint8_t  ack_for_opcode;   /* second byte for ACK_ACCEPTED/BUSY/ACK_DONE */
    uint8_t  error_code;       /* ERROR only */
    uint8_t  error_detail;     /* ERROR only */

    /* Convenience parse for STATE_DATA. */
    uint16_t data_seq;
    uint8_t  data_len;
    uint8_t  data[FUSION_STATE_DATA_MAX_DATA];
} FusionV2Decoded_t;

/* ---- Generic V2 frame helpers. ---- */

bool FusionSavestate_BuildV2Frame(uint8_t addr,
                                  const uint8_t *payload,
                                  size_t payload_len,
                                  FusionV2Frame_t *out);

bool FusionSavestate_DecodeV2Frame(const uint8_t *frame,
                                   size_t frame_len,
                                   FusionV2Decoded_t *out);

/* ---- STATE_CTL builders. ---- */

bool FusionSavestate_BuildBeginSave(uint8_t flags, FusionV2Frame_t *out);
bool FusionSavestate_BuildBeginLoad(uint8_t flags, FusionV2Frame_t *out);
bool FusionSavestate_BuildBeginTestRW(uint8_t flags, FusionV2Frame_t *out);
bool FusionSavestate_BuildEndSession(FusionV2Frame_t *out);
bool FusionSavestate_BuildSeek(uint8_t region, uint16_t offset, FusionV2Frame_t *out);
bool FusionSavestate_BuildReadNext(uint8_t count, FusionV2Frame_t *out);
bool FusionSavestate_BuildReadStreamBegin(uint8_t region,
                                          uint16_t offset,
                                          uint16_t length,
                                          FusionV2Frame_t *out);
bool FusionSavestate_BuildReadStreamContinue(uint16_t next_seq,
                                             FusionV2Frame_t *out);
bool FusionSavestate_BuildWriteStreamBegin(uint8_t region,
                                           uint16_t offset,
                                           uint16_t length,
                                           FusionV2Frame_t *out);
bool FusionSavestate_BuildWriteCommit(FusionV2Frame_t *out);

/* ---- STATE_DATA builder. data_len must be <= FUSION_STATE_DATA_MAX_DATA. ---- */

bool FusionSavestate_BuildStateData(uint16_t seq,
                                    const uint8_t *data,
                                    size_t data_len,
                                    FusionV2Frame_t *out);

/* ---- FPGA-side response builders (used by mock test harness). ---- */

bool FusionSavestate_BuildAckAccepted(uint8_t for_opcode, FusionV2Frame_t *out);
bool FusionSavestate_BuildBusy(uint8_t for_opcode, FusionV2Frame_t *out);
bool FusionSavestate_BuildAckDone(uint8_t for_opcode, FusionV2Frame_t *out);
bool FusionSavestate_BuildError(uint8_t code, uint8_t detail, FusionV2Frame_t *out);

/* ---- Legacy PathE-stub load state assembly. ---- */

/*
 * Captured CPU/HALT/IFF state at save time.  This is the input to the
 * legacy PathE-stub payload builder.  saved_PC must be the captured CPU PC
 * BEFORE any HALT-bug adjustment; the builder applies PC-1 if halted_at_save.
 *
 * Capture sources (Gate 1 contract §4):
 *   saved_A/F/B/C/D/E/H/L/SP/PC : T80 register file via state-port region 0x02
 *   halted_at_save              : T80 SS_3_BACK[48] = Halt_FF, exposed in
 *                                 region 0x02 idx 31 byte LSB
 *   saved_IFF                   : T80 SS_3_BACK[52] = IntE_FF1, exposed in
 *                                 region 0x02 idx 31 byte bit 4
 */
typedef struct {
    uint8_t  saved_F;
    uint8_t  saved_A;
    uint8_t  saved_C;
    uint8_t  saved_B;
    uint8_t  saved_E;
    uint8_t  saved_D;
    uint8_t  saved_L;
    uint8_t  saved_H;
    uint16_t saved_SP;
    uint16_t saved_PC;        /* unadjusted; builder applies PC-1 if halted */
    bool     saved_IFF;       /* IME state at save */
    bool     halted_at_save;  /* Halt_FF=1 at save */
} FusionPathELoadInput_t;

/*
 * Build the 16-byte legacy PathE-stub region 0xFE payload from captured save state.
 * out[0..14] are written to FPGA before gbreset is toggled (load_mode_active
 * is byte 0; setting it before gbreset arms the load mode).
 * out[15] is set to 0 by this builder (caller toggles gbreset separately
 * via two single-byte writes to offset 15).
 *
 * Return false on null input or if both halted_at_save=true and saved_PC=0
 * (PC-1 would underflow into MSB cart-bus space, considered invalid).
 */
bool FusionSavestate_BuildPathELoadPayload(const FusionPathELoadInput_t *state,
                                            uint8_t out[kPathE_RegionLength]);

/*
 * Extract legacy PathE-stub load input from saved CPU region (region 0x02) bytes.
 *
 * Layout per smoke48a convention (verified with FPGA's GBse.vhd / T80.vhd
 * mappings):
 *   region 0x02 byte 10..17 = T80 SS_1[0..63] (PC/A/TmpAddr/IR/...)
 *     - bytes 10-11: PC[7:0], PC[15:8]
 *     - bytes 12: A (visible ACC value matches SS_2[15:8] but SS_1 PC drives stub)
 *   region 0x02 byte 18..24 = T80 SS_2[0..55] (DO/ACC/Ap/Fp/I/R/MCycles)
 *     - byte 19: ACC (= visible A)
 *   region 0x02 byte 25..32 = T80 SS_3[0..63] (SP/F/IFF/MCycle/TState/Halt_FF/...)
 *     - bytes 25-26: SP[7:0], SP[15:8]
 *     - byte 27: SS_3[28:21] = F register
 *     - byte 31 (= "10101" = SS_3[55:48]): bit 0 = Halt_FF (SS_3[48]),
 *                                          bit 4 = IntE_FF1 = saved_IFF (SS_3[52])
 *
 * For BC/DE/HL: the T80_Reg register file maps to region 0x02 bytes 2-9.
 *   bytes 2-3: B,C (or C,B?) — exact bit mapping per T80_Reg ext_cpuregs port
 *
 * NOTE on byte 19 vs byte 12: SS_1 byte 12 is "A" but in T80.vhd that's
 * the 16-bit address bus (A[7:0]) — NOT the visible accumulator.  The
 * visible accumulator is ACC at SS_2[15:8] = byte 19.
 *
 * cpu_region_bytes must point to at least 40 bytes of saved CPU region.
 * Returns false on null inputs or if length < required offset.
 */
bool FusionSavestate_ExtractPathELoadInputFromCpuRegion(
    const uint8_t *cpu_region_bytes,
    size_t cpu_region_len,
    FusionPathELoadInput_t *out);

/* ---- CRC-32/ISO-HDLC (zlib/PNG) helpers and game ID v1. ---- */

uint32_t FusionSavestate_Crc32(const uint8_t *data, size_t len);
uint32_t FusionSavestate_Crc32Update(uint32_t prior_crc, const uint8_t *data, size_t len);

/*
 * game_id_v1 = CRC-32/ISO-HDLC over the 0x50 bytes of the GB ROM header
 * region 0x0100-0x014F. Caller must pass exactly that 80-byte slice.
 */
uint32_t FusionSavestate_ComputeGameIdV1(const uint8_t rom_header_0x100_to_0x14F[FUSION_ROM_HEADER_BYTES]);

/* Classify ROM header cartridge type 0x0147 for product save/load gating. */
FusionCartFeatureInfo_t FusionSavestate_ClassifyCartType(uint8_t cart_type);
bool FusionSavestate_CartTypeSupported(uint8_t cart_type);
const char *FusionSavestate_CartSupportReason(FusionCartSupport_t support);

/* Stable machine-parseable token for the support enum (e.g. "ALLOW_SUPPORTED",
 * "REJECT_RTC").  Suitable for log/console scraping; never localised. */
const char *FusionSavestate_CartSupportToken(FusionCartSupport_t support);

/* Stable machine-parseable name for the class enum (e.g. "ROM_ONLY", "MBC3"). */
const char *FusionSavestate_CartClassName(FusionCartClass_t cls);

/*
 * Evaluate the host-side load gate for a ROM header cartridge type.  Returns
 * allow_load=true only when the cartridge class is in the implemented
 * supported set.  Caller must refuse to push a load stream if allow_load is
 * false.  This does not contact the FPGA and does not assume Batch B/C/D
 * regions exist; it is preparatory routing only.
 */
FusionCartLoadGate_t FusionSavestate_EvaluateCartLoadGate(uint8_t cart_type);

#ifdef __cplusplus
}
#endif
