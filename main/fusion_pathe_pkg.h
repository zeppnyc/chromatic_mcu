#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Fusion PathE product state package: payload contract + load refusal gate.
 *
 * Scope: host/MCU side parser and validator for the versioned product state
 * package described in docs/PATHE_PRODUCT_STATE_DESIGN.md.  This module owns
 * the wire-format envelope (magic / version / required+optional region bitmap
 * / per-region table / CRCs) and the reason-coded refusal logic that prevents
 * load from running with bad, missing, wrong-version, or future-unsupported
 * required regions.
 *
 * This module does NOT implement any new FPGA state region, drive any FPGA
 * loader path, or change the Stage2 wire transport.  It is preparatory glue
 * that future Batch A/B/C/D work will register against by extending the
 * supported-region masks here.
 *
 * Pure C: no FreeRTOS, no ESP-IDF.  Compiles on host for unit tests.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Magic and current package version ---- */

/* 'F','U','S','S' interpreted little-endian. */
#define FUSION_PATHE_PKG_MAGIC            0x53535546u
#define FUSION_PATHE_PKG_VERSION_CURRENT  2u

/* Header byte layout (little-endian throughout):
 *   off  size  field
 *    0    4    magic
 *    4    2    package_version
 *    6    2    header_length
 *    8    4    package_flags
 *   12    4    required_region_bitmap
 *   16    4    optional_region_bitmap
 *   20    4    cart_feature_flags
 *   24    4    game_id_crc32
 *   28    2    region_count
 *   30    4    header_crc32
 * Total: 34 bytes.
 */
#define FUSION_PATHE_PKG_HEADER_SIZE      34u
#define FUSION_PATHE_PKG_HEADER_CRC_OFF   30u

/* Region table entry layout (little-endian throughout):
 *   off  size  field
 *    0    1    region_id
 *    1    1    region_version
 *    2    2    flags
 *    4    4    offset       (package-relative)
 *    8    4    length
 *   12    4    crc32
 * Total: 16 bytes.
 */
#define FUSION_PATHE_PKG_ENTRY_SIZE       16u

/* Region IDs.  These match the product Region Map in
 * docs/PATHE_PRODUCT_STATE_DESIGN.md and are stable across versions.
 */
#define FUSION_PATHE_REGION_ID_MANIFEST       0x00u
#define FUSION_PATHE_REGION_ID_CPU_SCALAR     0x10u
#define FUSION_PATHE_REGION_ID_CART_MBC       0x20u
#define FUSION_PATHE_REGION_ID_WRAM           0x30u
#define FUSION_PATHE_REGION_ID_HRAM           0x31u
#define FUSION_PATHE_REGION_ID_VRAM           0x40u
#define FUSION_PATHE_REGION_ID_OAM            0x41u
#define FUSION_PATHE_REGION_ID_VIDEO_REGS     0x42u
#define FUSION_PATHE_REGION_ID_APU            0x50u

/* Known exact payload lengths for regions that have a fixed product contract. */
#define FUSION_PATHE_REGION_LEN_MANIFEST      65u
#define FUSION_PATHE_REGION_LEN_CPU_SCALAR    42u
#define FUSION_PATHE_REGION_LEN_WRAM          32768u
#define FUSION_PATHE_REGION_LEN_HRAM          127u
#define FUSION_PATHE_REGION_LEN_VRAM          16384u
#define FUSION_PATHE_REGION_LEN_OAM           160u

/* Region entry flag bits. */
#define FUSION_PATHE_REGION_FLAG_REQUIRED         (1u << 0)
#define FUSION_PATHE_REGION_FLAG_OPTIONAL_META    (1u << 1)
#define FUSION_PATHE_REGION_FLAG_UNSUPPORTED_META (1u << 2)
#define FUSION_PATHE_REGION_FLAG_COMPRESSED       (1u << 3)  /* future */

/*
 * Slot encoding inside required_region_bitmap / optional_region_bitmap /
 * supported masks.  Bit N of a 32-bit bitmap means the region with that slot
 * id participates.  Region id (0x00..0x50) is mapped to slot id (0..8) via
 * FusionPathePkg_RegionSlot().
 */
typedef enum {
    kFusionPathePkgSlot_PackageManifest = 0,
    kFusionPathePkgSlot_CpuScalarFull   = 1,
    kFusionPathePkgSlot_CartMbcScalar   = 2,
    kFusionPathePkgSlot_Wram            = 3,
    kFusionPathePkgSlot_Hram            = 4,
    kFusionPathePkgSlot_Vram            = 5,
    kFusionPathePkgSlot_Oam             = 6,
    kFusionPathePkgSlot_VideoRegs       = 7,
    kFusionPathePkgSlot_Apu             = 8,
    kFusionPathePkgSlot_Count           = 9,
} FusionPathePkgSlot_t;

#define FUSION_PATHE_PKG_SLOT_INVALID 0xFFu

/*
 * Set of region slots supported by the CURRENT FPGA build's load path.
 * Today (2026-05-17): the Stage2 wire region 0 carries the package manifest
 * payload inline (CPU scalar + TOP + TIMER), and WRAM/HRAM are wired
 * through the validated port-A pause-window / wrapper RAM access paths.
 *
 * Slot 1 (CPU_SCALAR_FULL) is reserved for a future Batch A region split
 * and is intentionally NOT in this mask: a package whose required_region
 * bitmap demands slot 1 must be refused on the current build until the
 * separate region is implemented.
 *
 * Cart/MBC/VRAM/OAM/VideoRegs/APU slots (2/5/6/7/8) belong to Batch B/C/D
 * and are also intentionally not in this mask.
 */
#define FUSION_PATHE_PKG_SUPPORTED_REQUIRED_MASK_CURRENT_BUILD \
    ((1u << kFusionPathePkgSlot_PackageManifest) | \
     (1u << kFusionPathePkgSlot_Wram) | \
     (1u << kFusionPathePkgSlot_Hram))

/*
 * Set of slots that the current build understands as OPTIONAL metadata.
 * Optional regions outside this mask are still allowed (they are saved /
 * displayed as metadata) and never cause load refusal.  This mask exists
 * so callers can flag "unsupported optional metadata seen" for UI/logging.
 */
#define FUSION_PATHE_PKG_SUPPORTED_OPTIONAL_MASK_CURRENT_BUILD \
    ((1u << kFusionPathePkgSlot_PackageManifest) | \
     (1u << kFusionPathePkgSlot_Wram) | \
     (1u << kFusionPathePkgSlot_Hram))

/* ---- Reason codes ---- */

typedef enum {
    kFusionPathePkg_Ok                     = 0,
    kFusionPathePkg_BadMagic               = 1,
    kFusionPathePkg_BadVersion             = 2,
    kFusionPathePkg_BadHeaderCrc           = 3,
    kFusionPathePkg_RegionMissing          = 4,
    kFusionPathePkg_RegionBadVersion       = 5,
    kFusionPathePkg_RegionBadCrc           = 6,
    kFusionPathePkg_RegionUnsupported      = 7,
    kFusionPathePkg_RegionRangeInvalid     = 8,
    kFusionPathePkg_RegionOverlap          = 9,
    kFusionPathePkg_RegionTableInvalid     = 10, /* table itself out of bounds */
    kFusionPathePkg_TruncatedHeader        = 11, /* bytes shorter than header */
    kFusionPathePkg_DuplicateRegion        = 12, /* same slot listed twice as required */
    kFusionPathePkg_RegionBadLength        = 13, /* fixed-length required region has wrong length */
} FusionPathePkgStatus_t;

/* ---- Parsed header view (host-side, native types) ---- */

typedef struct {
    uint32_t magic;
    uint16_t package_version;
    uint16_t header_length;
    uint32_t package_flags;
    uint32_t required_region_bitmap;
    uint32_t optional_region_bitmap;
    uint32_t cart_feature_flags;
    uint32_t game_id_crc32;
    uint16_t region_count;
    uint32_t header_crc32;
} FusionPathePkgHeader_t;

typedef struct {
    uint8_t  region_id;
    uint8_t  region_version;
    uint16_t flags;
    uint32_t offset;
    uint32_t length;
    uint32_t crc32;
} FusionPathePkgEntry_t;

/* ---- Validation result ---- */

typedef struct {
    FusionPathePkgStatus_t status;
    /* Index into the region table for region-specific reasons.
     * Set to UINT16_MAX (0xFFFFu) when not region-specific. */
    uint16_t offending_entry_index;
    /* region_id copied from the offending entry, 0 when not specific. */
    uint8_t  offending_region_id;
    /* Slot id of the offending entry, FUSION_PATHE_PKG_SLOT_INVALID when n/a. */
    uint8_t  offending_slot;
} FusionPathePkgValidation_t;

#define FUSION_PATHE_PKG_NO_ENTRY 0xFFFFu

/* ---- Public helpers ---- */

const char *FusionPathePkg_ReasonString(FusionPathePkgStatus_t status);

uint8_t FusionPathePkg_RegionSlot(uint8_t region_id);
uint8_t FusionPathePkg_SlotRegionId(uint8_t slot);
uint8_t FusionPathePkg_ExpectedRegionVersion(uint8_t slot);
bool FusionPathePkg_ExpectedRegionLength(uint8_t slot, uint32_t *out_length);

/*
 * Parse just the header (first FUSION_PATHE_PKG_HEADER_SIZE bytes) into a
 * native struct.  Does not validate magic, version, or CRC; that is what
 * FusionPathePkg_Validate() is for.  Returns false only on null inputs or
 * len < FUSION_PATHE_PKG_HEADER_SIZE.
 */
bool FusionPathePkg_ParseHeader(const uint8_t *bytes,
                                size_t len,
                                FusionPathePkgHeader_t *out_hdr);

/*
 * Parse the region table entry at byte offset
 *   header_length + entry_index * FUSION_PATHE_PKG_ENTRY_SIZE
 * Returns false if the entry would extend beyond len.
 */
bool FusionPathePkg_ParseEntry(const uint8_t *bytes,
                               size_t len,
                               uint16_t header_length,
                               uint16_t entry_index,
                               FusionPathePkgEntry_t *out_entry);

/*
 * Compute the header CRC for `bytes` treating header_crc32 field as zero.
 * `len` must be >= header_length.
 */
uint32_t FusionPathePkg_ComputeHeaderCrc(const uint8_t *bytes,
                                         uint16_t header_length);

/*
 * Full validation pass.  Returns a reason code plus context.
 *
 * - bytes/len describe the entire on-flash package.
 * - supported_required_mask: bit set per slot that the CURRENT build can
 *   actually load (refer to FUSION_PATHE_PKG_SUPPORTED_REQUIRED_MASK_*).
 * - supported_optional_mask: bit set per slot understood as metadata; unset
 *   slots in the optional bitmap are still allowed (they never cause
 *   refusal), but the caller may want to surface them as
 *   "unsupported optional metadata".
 *
 * On kFusionPathePkg_Ok every required region in the package is present,
 * version-matched, CRC-clean, range-clean, overlap-free, and supported by
 * the current build.
 *
 * On any non-Ok status the validation must be treated as a load refusal:
 * the caller MUST NOT proceed to apply the package contents.
 */
FusionPathePkgValidation_t FusionPathePkg_Validate(
    const uint8_t *bytes,
    size_t len,
    uint32_t supported_required_mask,
    uint32_t supported_optional_mask);

#ifdef __cplusplus
}
#endif
