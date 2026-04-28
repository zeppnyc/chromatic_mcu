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

/* Region IDs (architecture doc Phase 4 plan — names, not adapter availability). */
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
} FusionRegion_t;

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
bool FusionSavestate_BuildEndSession(FusionV2Frame_t *out);
bool FusionSavestate_BuildSeek(uint8_t region, uint16_t offset, FusionV2Frame_t *out);
bool FusionSavestate_BuildReadNext(uint8_t count, FusionV2Frame_t *out);
bool FusionSavestate_BuildReadStreamBegin(uint8_t region,
                                          uint16_t offset,
                                          uint16_t length,
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

/* ---- CRC-32/ISO-HDLC (zlib/PNG) helpers and game ID v1. ---- */

uint32_t FusionSavestate_Crc32(const uint8_t *data, size_t len);
uint32_t FusionSavestate_Crc32Update(uint32_t prior_crc, const uint8_t *data, size_t len);

/*
 * game_id_v1 = CRC-32/ISO-HDLC over the 0x50 bytes of the GB ROM header
 * region 0x0100-0x014F. Caller must pass exactly that 80-byte slice.
 */
uint32_t FusionSavestate_ComputeGameIdV1(const uint8_t rom_header_0x100_to_0x14F[FUSION_ROM_HEADER_BYTES]);

#ifdef __cplusplus
}
#endif
