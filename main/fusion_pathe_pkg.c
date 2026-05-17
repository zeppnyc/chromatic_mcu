#include "fusion_pathe_pkg.h"

#include "fusion_savestate.h"

#include <string.h>

/* ---- Little-endian byte readers ---- */

static uint16_t ReadU16Le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t ReadU32Le(const uint8_t *p)
{
    return ((uint32_t)p[0])
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* ---- Slot mapping ---- */

uint8_t FusionPathePkg_RegionSlot(uint8_t region_id)
{
    switch (region_id) {
    case FUSION_PATHE_REGION_ID_MANIFEST:   return kFusionPathePkgSlot_PackageManifest;
    case FUSION_PATHE_REGION_ID_CPU_SCALAR: return kFusionPathePkgSlot_CpuScalarFull;
    case FUSION_PATHE_REGION_ID_CART_MBC:   return kFusionPathePkgSlot_CartMbcScalar;
    case FUSION_PATHE_REGION_ID_WRAM:       return kFusionPathePkgSlot_Wram;
    case FUSION_PATHE_REGION_ID_HRAM:       return kFusionPathePkgSlot_Hram;
    case FUSION_PATHE_REGION_ID_VRAM:       return kFusionPathePkgSlot_Vram;
    case FUSION_PATHE_REGION_ID_OAM:        return kFusionPathePkgSlot_Oam;
    case FUSION_PATHE_REGION_ID_VIDEO_REGS: return kFusionPathePkgSlot_VideoRegs;
    case FUSION_PATHE_REGION_ID_APU:        return kFusionPathePkgSlot_Apu;
    default:                                return FUSION_PATHE_PKG_SLOT_INVALID;
    }
}

uint8_t FusionPathePkg_SlotRegionId(uint8_t slot)
{
    switch (slot) {
    case kFusionPathePkgSlot_PackageManifest: return FUSION_PATHE_REGION_ID_MANIFEST;
    case kFusionPathePkgSlot_CpuScalarFull:   return FUSION_PATHE_REGION_ID_CPU_SCALAR;
    case kFusionPathePkgSlot_CartMbcScalar:   return FUSION_PATHE_REGION_ID_CART_MBC;
    case kFusionPathePkgSlot_Wram:            return FUSION_PATHE_REGION_ID_WRAM;
    case kFusionPathePkgSlot_Hram:            return FUSION_PATHE_REGION_ID_HRAM;
    case kFusionPathePkgSlot_Vram:            return FUSION_PATHE_REGION_ID_VRAM;
    case kFusionPathePkgSlot_Oam:             return FUSION_PATHE_REGION_ID_OAM;
    case kFusionPathePkgSlot_VideoRegs:       return FUSION_PATHE_REGION_ID_VIDEO_REGS;
    case kFusionPathePkgSlot_Apu:             return FUSION_PATHE_REGION_ID_APU;
    default:                                  return 0u;
    }
}

uint8_t FusionPathePkg_ExpectedRegionVersion(uint8_t slot)
{
    /* Per-slot expected payload version.  Required regions whose entry
     * region_version != expected version are refused as
     * kFusionPathePkg_RegionBadVersion.  Mismatched optional regions are
     * tolerated as metadata.
     */
    switch (slot) {
    case kFusionPathePkgSlot_PackageManifest: return 2u;
    case kFusionPathePkgSlot_CpuScalarFull:   return 1u;
    case kFusionPathePkgSlot_CartMbcScalar:   return 1u;
    case kFusionPathePkgSlot_Wram:            return 1u;
    case kFusionPathePkgSlot_Hram:            return 1u;
    case kFusionPathePkgSlot_Vram:            return 1u;
    case kFusionPathePkgSlot_Oam:             return 1u;
    case kFusionPathePkgSlot_VideoRegs:       return 1u;
    case kFusionPathePkgSlot_Apu:             return 1u;
    default:                                  return 0u;
    }
}

bool FusionPathePkg_ExpectedRegionLength(uint8_t slot, uint32_t *out_length)
{
    if (out_length == NULL) {
        return false;
    }

    switch (slot) {
    case kFusionPathePkgSlot_PackageManifest:
        *out_length = FUSION_PATHE_REGION_LEN_MANIFEST;
        return true;
    case kFusionPathePkgSlot_CpuScalarFull:
        *out_length = FUSION_PATHE_REGION_LEN_CPU_SCALAR;
        return true;
    case kFusionPathePkgSlot_Wram:
        *out_length = FUSION_PATHE_REGION_LEN_WRAM;
        return true;
    case kFusionPathePkgSlot_Hram:
        *out_length = FUSION_PATHE_REGION_LEN_HRAM;
        return true;
    case kFusionPathePkgSlot_Vram:
        *out_length = FUSION_PATHE_REGION_LEN_VRAM;
        return true;
    case kFusionPathePkgSlot_Oam:
        *out_length = FUSION_PATHE_REGION_LEN_OAM;
        return true;
    default:
        return false;
    }
}

const char *FusionPathePkg_ReasonString(FusionPathePkgStatus_t status)
{
    switch (status) {
    case kFusionPathePkg_Ok:                 return "PKG_OK";
    case kFusionPathePkg_BadMagic:           return "PKG_BAD_MAGIC";
    case kFusionPathePkg_BadVersion:         return "PKG_BAD_VERSION";
    case kFusionPathePkg_BadHeaderCrc:       return "PKG_BAD_HEADER_CRC";
    case kFusionPathePkg_RegionMissing:      return "PKG_REGION_MISSING";
    case kFusionPathePkg_RegionBadVersion:   return "PKG_REGION_BAD_VERSION";
    case kFusionPathePkg_RegionBadCrc:       return "PKG_REGION_BAD_CRC";
    case kFusionPathePkg_RegionUnsupported:  return "PKG_REGION_UNSUPPORTED";
    case kFusionPathePkg_RegionRangeInvalid: return "PKG_REGION_RANGE_INVALID";
    case kFusionPathePkg_RegionOverlap:      return "PKG_REGION_OVERLAP";
    case kFusionPathePkg_RegionTableInvalid: return "PKG_REGION_TABLE_INVALID";
    case kFusionPathePkg_TruncatedHeader:    return "PKG_TRUNCATED_HEADER";
    case kFusionPathePkg_DuplicateRegion:    return "PKG_DUPLICATE_REGION";
    case kFusionPathePkg_RegionBadLength:    return "PKG_REGION_BAD_LENGTH";
    default:                                 return "PKG_UNKNOWN";
    }
}

/* ---- Header / entry parsing ---- */

bool FusionPathePkg_ParseHeader(const uint8_t *bytes,
                                size_t len,
                                FusionPathePkgHeader_t *out_hdr)
{
    if (bytes == NULL || out_hdr == NULL) {
        return false;
    }
    if (len < FUSION_PATHE_PKG_HEADER_SIZE) {
        return false;
    }
    out_hdr->magic                  = ReadU32Le(&bytes[0]);
    out_hdr->package_version        = ReadU16Le(&bytes[4]);
    out_hdr->header_length          = ReadU16Le(&bytes[6]);
    out_hdr->package_flags          = ReadU32Le(&bytes[8]);
    out_hdr->required_region_bitmap = ReadU32Le(&bytes[12]);
    out_hdr->optional_region_bitmap = ReadU32Le(&bytes[16]);
    out_hdr->cart_feature_flags     = ReadU32Le(&bytes[20]);
    out_hdr->game_id_crc32          = ReadU32Le(&bytes[24]);
    out_hdr->region_count           = ReadU16Le(&bytes[28]);
    out_hdr->header_crc32           = ReadU32Le(&bytes[30]);
    return true;
}

bool FusionPathePkg_ParseEntry(const uint8_t *bytes,
                               size_t len,
                               uint16_t header_length,
                               uint16_t entry_index,
                               FusionPathePkgEntry_t *out_entry)
{
    if (bytes == NULL || out_entry == NULL) {
        return false;
    }
    const size_t base = (size_t)header_length
                      + (size_t)entry_index * FUSION_PATHE_PKG_ENTRY_SIZE;
    if (base + FUSION_PATHE_PKG_ENTRY_SIZE > len) {
        return false;
    }
    const uint8_t *p = &bytes[base];
    out_entry->region_id      = p[0];
    out_entry->region_version = p[1];
    out_entry->flags          = ReadU16Le(&p[2]);
    out_entry->offset         = ReadU32Le(&p[4]);
    out_entry->length         = ReadU32Le(&p[8]);
    out_entry->crc32          = ReadU32Le(&p[12]);
    return true;
}

uint32_t FusionPathePkg_ComputeHeaderCrc(const uint8_t *bytes,
                                         uint16_t header_length)
{
    if (bytes == NULL || header_length < FUSION_PATHE_PKG_HEADER_SIZE) {
        return 0u;
    }
    /* CRC is computed over the header bytes with the 4-byte header_crc32
     * field treated as zero.  Computed in two halves to avoid allocating
     * a temporary buffer.
     */
    uint32_t crc = FusionSavestate_Crc32Update(0u, bytes, FUSION_PATHE_PKG_HEADER_CRC_OFF);
    static const uint8_t zero4[4] = {0u, 0u, 0u, 0u};
    crc = FusionSavestate_Crc32Update(crc, zero4, sizeof(zero4));
    const uint16_t tail_off = (uint16_t)(FUSION_PATHE_PKG_HEADER_CRC_OFF + 4u);
    if (header_length > tail_off) {
        crc = FusionSavestate_Crc32Update(crc,
                                          &bytes[tail_off],
                                          (size_t)(header_length - tail_off));
    }
    return crc;
}

/* ---- Helpers used by Validate ---- */

static FusionPathePkgValidation_t MakeResult(FusionPathePkgStatus_t status)
{
    FusionPathePkgValidation_t r;
    r.status = status;
    r.offending_entry_index = FUSION_PATHE_PKG_NO_ENTRY;
    r.offending_region_id = 0u;
    r.offending_slot = FUSION_PATHE_PKG_SLOT_INVALID;
    return r;
}

static FusionPathePkgValidation_t MakeEntryResult(FusionPathePkgStatus_t status,
                                                  uint16_t entry_index,
                                                  uint8_t region_id,
                                                  uint8_t slot)
{
    FusionPathePkgValidation_t r;
    r.status = status;
    r.offending_entry_index = entry_index;
    r.offending_region_id = region_id;
    r.offending_slot = slot;
    return r;
}

/* ---- Full validation ---- */

FusionPathePkgValidation_t FusionPathePkg_Validate(
    const uint8_t *bytes,
    size_t len,
    uint32_t supported_required_mask,
    uint32_t supported_optional_mask)
{
    (void)supported_optional_mask;  /* allowed-but-not-enforced; reserved for caller UI */

    if (bytes == NULL) {
        return MakeResult(kFusionPathePkg_TruncatedHeader);
    }
    if (len < FUSION_PATHE_PKG_HEADER_SIZE) {
        return MakeResult(kFusionPathePkg_TruncatedHeader);
    }

    FusionPathePkgHeader_t hdr;
    if (!FusionPathePkg_ParseHeader(bytes, len, &hdr)) {
        return MakeResult(kFusionPathePkg_TruncatedHeader);
    }

    /* Magic. */
    if (hdr.magic != FUSION_PATHE_PKG_MAGIC) {
        return MakeResult(kFusionPathePkg_BadMagic);
    }

    /* Version. */
    if (hdr.package_version != FUSION_PATHE_PKG_VERSION_CURRENT) {
        return MakeResult(kFusionPathePkg_BadVersion);
    }

    /* Header length sanity: must be at least the documented size and must
     * not exceed the package length.
     */
    if (hdr.header_length < FUSION_PATHE_PKG_HEADER_SIZE) {
        return MakeResult(kFusionPathePkg_TruncatedHeader);
    }
    if ((size_t)hdr.header_length > len) {
        return MakeResult(kFusionPathePkg_TruncatedHeader);
    }

    /* Header CRC. */
    const uint32_t header_crc = FusionPathePkg_ComputeHeaderCrc(bytes, hdr.header_length);
    if (header_crc != hdr.header_crc32) {
        return MakeResult(kFusionPathePkg_BadHeaderCrc);
    }

    /* Region table fits inside the package. */
    const size_t table_bytes = (size_t)hdr.region_count * FUSION_PATHE_PKG_ENTRY_SIZE;
    const size_t table_end   = (size_t)hdr.header_length + table_bytes;
    if (table_end < (size_t)hdr.header_length) {
        return MakeResult(kFusionPathePkg_RegionTableInvalid); /* overflow */
    }
    if (table_end > len) {
        return MakeResult(kFusionPathePkg_RegionTableInvalid);
    }

    /*
     * Walk entries.  For each entry, parse / range-check / CRC-check.
     * Track which slots have been seen and which were marked required so
     * we can spot duplicates and check required-coverage.
     */
    uint32_t required_in_pkg_mask = 0u;
    uint32_t seen_required_mask   = 0u;
    /* Indexed by slot. */
    uint8_t  required_versions[kFusionPathePkgSlot_Count];
    memset(required_versions, 0u, sizeof(required_versions));

    for (uint16_t i = 0; i < hdr.region_count; ++i) {
        FusionPathePkgEntry_t e;
        if (!FusionPathePkg_ParseEntry(bytes, len, hdr.header_length, i, &e)) {
            return MakeResult(kFusionPathePkg_RegionTableInvalid);
        }
        const uint8_t slot = FusionPathePkg_RegionSlot(e.region_id);

        /* Range checks. */
        const size_t off_end = (size_t)e.offset + (size_t)e.length;
        if (off_end < (size_t)e.offset) {
            return MakeEntryResult(kFusionPathePkg_RegionRangeInvalid, i, e.region_id, slot);
        }
        if ((size_t)e.offset < table_end) {
            return MakeEntryResult(kFusionPathePkg_RegionRangeInvalid, i, e.region_id, slot);
        }
        if (off_end > len) {
            return MakeEntryResult(kFusionPathePkg_RegionRangeInvalid, i, e.region_id, slot);
        }

        /* Overlap check against earlier entries. */
        for (uint16_t j = 0; j < i; ++j) {
            FusionPathePkgEntry_t prev;
            if (!FusionPathePkg_ParseEntry(bytes, len, hdr.header_length, j, &prev)) {
                /* Should not happen — we already walked to entry i. */
                return MakeResult(kFusionPathePkg_RegionTableInvalid);
            }
            const size_t prev_end = (size_t)prev.offset + (size_t)prev.length;
            const bool no_overlap = (off_end <= (size_t)prev.offset) || (prev_end <= (size_t)e.offset);
            if (!no_overlap) {
                return MakeEntryResult(kFusionPathePkg_RegionOverlap, i, e.region_id, slot);
            }
        }

        /* Payload CRC. */
        const uint32_t payload_crc =
            FusionSavestate_Crc32(&bytes[e.offset], (size_t)e.length);
        if (payload_crc != e.crc32) {
            return MakeEntryResult(kFusionPathePkg_RegionBadCrc, i, e.region_id, slot);
        }

        /* Required-region accounting (entry flag OR header bitmap). */
        const bool flag_required = (e.flags & FUSION_PATHE_REGION_FLAG_REQUIRED) != 0u;
        const bool header_required =
            (slot != FUSION_PATHE_PKG_SLOT_INVALID) &&
            ((hdr.required_region_bitmap & (1u << slot)) != 0u);
        const bool is_required = flag_required || header_required;

        if (is_required) {
            if (slot == FUSION_PATHE_PKG_SLOT_INVALID) {
                /* Unknown region id required by entry flag — current build
                 * cannot provide it. */
                return MakeEntryResult(kFusionPathePkg_RegionUnsupported,
                                       i, e.region_id, slot);
            }
            const uint32_t bit = (1u << slot);
            if ((seen_required_mask & bit) != 0u) {
                return MakeEntryResult(kFusionPathePkg_DuplicateRegion,
                                       i, e.region_id, slot);
            }
            seen_required_mask |= bit;
            required_in_pkg_mask |= bit;
            required_versions[slot] = e.region_version;

            /* Required region version must match expected. */
            const uint8_t expected = FusionPathePkg_ExpectedRegionVersion(slot);
            if (e.region_version != expected) {
                return MakeEntryResult(kFusionPathePkg_RegionBadVersion,
                                       i, e.region_id, slot);
            }
            /* Required region must be supported by the current build. */
            if ((supported_required_mask & bit) == 0u) {
                return MakeEntryResult(kFusionPathePkg_RegionUnsupported,
                                       i, e.region_id, slot);
            }
            uint32_t expected_length = 0u;
            if (FusionPathePkg_ExpectedRegionLength(slot, &expected_length) &&
                e.length != expected_length) {
                return MakeEntryResult(kFusionPathePkg_RegionBadLength,
                                       i, e.region_id, slot);
            }
        }
        /* Non-required entries (optional / metadata) are accepted as long as
         * their range/CRC are clean.  They never cause refusal. */
    }

    /*
     * Required-bitmap completeness: every bit set in the header's
     * required_region_bitmap must be matched by an entry we just walked.
     * (If the package declared a slot required in the bitmap but no entry
     * is present, that slot's bit is missing from seen_required_mask.)
     */
    const uint32_t hdr_required = hdr.required_region_bitmap;
    if ((hdr_required & seen_required_mask) != hdr_required) {
        /* Find the first missing required slot to report it. */
        const uint32_t missing = hdr_required & ~seen_required_mask;
        uint8_t miss_slot = FUSION_PATHE_PKG_SLOT_INVALID;
        for (uint8_t s = 0; s < kFusionPathePkgSlot_Count; ++s) {
            if ((missing & (1u << s)) != 0u) {
                miss_slot = s;
                break;
            }
        }
        const uint8_t miss_region_id =
            (miss_slot == FUSION_PATHE_PKG_SLOT_INVALID)
                ? 0u
                : FusionPathePkg_SlotRegionId(miss_slot);
        /* If the missing required slot is also outside the build's
         * supported-required mask, classify as Unsupported (more specific
         * than Missing).  This matches the spec rule
         * "当前 build 不支持 required region: 拒绝".
         */
        if (miss_slot != FUSION_PATHE_PKG_SLOT_INVALID &&
            (supported_required_mask & (1u << miss_slot)) == 0u) {
            return MakeEntryResult(kFusionPathePkg_RegionUnsupported,
                                   FUSION_PATHE_PKG_NO_ENTRY,
                                   miss_region_id, miss_slot);
        }
        return MakeEntryResult(kFusionPathePkg_RegionMissing,
                               FUSION_PATHE_PKG_NO_ENTRY,
                               miss_region_id, miss_slot);
    }

    /*
     * Required-bitmap support: bits set in the header bitmap must all be
     * inside supported_required_mask.  Catches the case where the package
     * itself declares a future region required AND provides an entry for it
     * (so the per-entry unsupported check above also fires); this is the
     * belt-and-suspenders check for the no-entry edge already handled.
     */
    const uint32_t unsupported_required = hdr_required & ~supported_required_mask;
    if (unsupported_required != 0u) {
        uint8_t miss_slot = FUSION_PATHE_PKG_SLOT_INVALID;
        for (uint8_t s = 0; s < kFusionPathePkgSlot_Count; ++s) {
            if ((unsupported_required & (1u << s)) != 0u) {
                miss_slot = s;
                break;
            }
        }
        const uint8_t miss_region_id =
            (miss_slot == FUSION_PATHE_PKG_SLOT_INVALID)
                ? 0u
                : FusionPathePkg_SlotRegionId(miss_slot);
        return MakeEntryResult(kFusionPathePkg_RegionUnsupported,
                               FUSION_PATHE_PKG_NO_ENTRY,
                               miss_region_id, miss_slot);
    }

    (void)required_in_pkg_mask;
    return MakeResult(kFusionPathePkg_Ok);
}
