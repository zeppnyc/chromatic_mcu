/*
 * Host self-test for the PathE product state package gate.
 *
 * Builds against main/fusion_pathe_pkg.c + main/fusion_savestate.c (for the
 * shared CRC-32 helper).  Covers every reason code declared in
 * fusion_pathe_pkg.h via small in-memory packages mutated from a "valid
 * minimal Batch A package" baseline.
 *
 * This file deliberately exposes only one entry point,
 *   RunFusionPathePkgSelftests(int *pass_out, int *fail_out),
 * which the existing fusion_savestate_selftest.c main() calls so the
 * combined run still prints a single "N passed, M failed" line.
 */

#include "fusion_pathe_pkg.h"
#include "fusion_savestate.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int s_pass = 0;
static int s_fail = 0;

#define PKG_EXPECT(cond) do {                                          \
    if (cond) {                                                        \
        s_pass++;                                                      \
    } else {                                                           \
        s_fail++;                                                      \
        fprintf(stderr, "  FAIL %s:%d  %s\n",                          \
                __FILE__, __LINE__, #cond);                            \
    }                                                                  \
} while (0)

#define PKG_EXPECT_EQ_U(a, b) do {                                                  \
    const uint64_t _a = (uint64_t)(a);                                              \
    const uint64_t _b = (uint64_t)(b);                                              \
    if (_a == _b) {                                                                 \
        s_pass++;                                                                   \
    } else {                                                                        \
        s_fail++;                                                                   \
        fprintf(stderr, "  FAIL %s:%d  %s == %s  (got 0x%llx vs 0x%llx)\n",         \
                __FILE__, __LINE__, #a, #b,                                         \
                (unsigned long long)_a, (unsigned long long)_b);                    \
    }                                                                               \
} while (0)

/* ---- Little-endian write helpers ---- */

static void WriteU16Le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void WriteU32Le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* ---- Package builder ----
 *
 * We carry a fixed-size scratch buffer.  The package layout used by every
 * test (unless explicitly mutated):
 *
 *   header          34 B at offset 0
 *   region table    16 B * N  at offset 34
 *   payload bytes   contiguous after table
 *
 * For "valid minimal Batch A package" the three required regions are
 *   slot 0  manifest  (region id 0x00)  65 bytes
 *   slot 3  WRAM      (region id 0x30)  32768 bytes
 *   slot 4  HRAM      (region id 0x31)  127 bytes
 */

#define BUILDER_REGION_CAP 6

typedef struct {
    uint8_t  region_id;
    uint8_t  region_version;
    uint16_t flags;
    uint32_t length;
} BuilderRegion_t;

typedef struct {
    /* Header fields. */
    uint32_t magic;
    uint16_t package_version;
    uint32_t package_flags;
    uint32_t required_region_bitmap;
    uint32_t optional_region_bitmap;
    uint32_t cart_feature_flags;
    uint32_t game_id_crc32;

    /* Regions to emit. */
    BuilderRegion_t regions[BUILDER_REGION_CAP];
    uint16_t        region_count;
} Builder_t;

static void BuilderInit(Builder_t *b)
{
    memset(b, 0, sizeof(*b));
    b->magic                  = FUSION_PATHE_PKG_MAGIC;
    b->package_version        = FUSION_PATHE_PKG_VERSION_CURRENT;
    /* By default the bitmap declares manifest, WRAM, HRAM required. */
    b->required_region_bitmap =
        (1u << kFusionPathePkgSlot_PackageManifest) |
        (1u << kFusionPathePkgSlot_Wram) |
        (1u << kFusionPathePkgSlot_Hram);
    b->game_id_crc32          = 0xDEADBEEFu;
}

static void BuilderAddRegion(Builder_t *b,
                             uint8_t region_id,
                             uint8_t version,
                             uint16_t flags,
                             uint32_t length)
{
    if (b->region_count >= BUILDER_REGION_CAP) {
        return;
    }
    BuilderRegion_t *r = &b->regions[b->region_count++];
    r->region_id      = region_id;
    r->region_version = version;
    r->flags          = flags;
    r->length         = length;
}

static void BuilderAddBatchARequiredRegions(Builder_t *b)
{
    /* Manifest: 65 bytes per design doc Stage2 wire region 0 layout. */
    BuilderAddRegion(b,
                     FUSION_PATHE_REGION_ID_MANIFEST,
                     /*version*/ 2u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED,
                     FUSION_PATHE_REGION_LEN_MANIFEST);
    /* WRAM: exact fixed product length. */
    BuilderAddRegion(b,
                     FUSION_PATHE_REGION_ID_WRAM,
                     /*version*/ 1u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED,
                     FUSION_PATHE_REGION_LEN_WRAM);
    /* HRAM: exact fixed product length. */
    BuilderAddRegion(b,
                     FUSION_PATHE_REGION_ID_HRAM,
                     /*version*/ 1u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED,
                     FUSION_PATHE_REGION_LEN_HRAM);
}

#define BUILDER_OUT_CAP 34000u

/*
 * Emit a fully-formed package into out_bytes; returns number of bytes
 * written, or 0 on capacity overflow.  Computes payload offsets and CRC32s.
 */
static size_t BuilderEmit(const Builder_t *b, uint8_t *out_bytes, size_t out_cap)
{
    const size_t header_size = FUSION_PATHE_PKG_HEADER_SIZE;
    const size_t table_size  = (size_t)b->region_count * FUSION_PATHE_PKG_ENTRY_SIZE;
    size_t cursor = header_size + table_size;
    if (cursor > out_cap) {
        return 0u;
    }

    memset(out_bytes, 0, out_cap);

    /* Layout payloads after table. */
    uint32_t entry_offsets[BUILDER_REGION_CAP];
    for (uint16_t i = 0; i < b->region_count; ++i) {
        const uint32_t len = b->regions[i].length;
        if (cursor + len > out_cap) {
            return 0u;
        }
        entry_offsets[i] = (uint32_t)cursor;
        for (uint32_t j = 0; j < len; ++j) {
            out_bytes[cursor + j] =
                (uint8_t)((0xC0u + (uint32_t)b->regions[i].region_id + j) & 0xFFu);
        }
        cursor += len;
    }
    const size_t total = cursor;

    /* Header. */
    WriteU32Le(&out_bytes[0],  b->magic);
    WriteU16Le(&out_bytes[4],  b->package_version);
    WriteU16Le(&out_bytes[6],  (uint16_t)header_size);
    WriteU32Le(&out_bytes[8],  b->package_flags);
    WriteU32Le(&out_bytes[12], b->required_region_bitmap);
    WriteU32Le(&out_bytes[16], b->optional_region_bitmap);
    WriteU32Le(&out_bytes[20], b->cart_feature_flags);
    WriteU32Le(&out_bytes[24], b->game_id_crc32);
    WriteU16Le(&out_bytes[28], b->region_count);
    /* header_crc32 written below. */

    /* Region table entries. */
    for (uint16_t i = 0; i < b->region_count; ++i) {
        const size_t base = header_size + (size_t)i * FUSION_PATHE_PKG_ENTRY_SIZE;
        const uint32_t off = entry_offsets[i];
        const uint32_t len = b->regions[i].length;
        const uint32_t crc = FusionSavestate_Crc32(&out_bytes[off], (size_t)len);
        out_bytes[base + 0] = b->regions[i].region_id;
        out_bytes[base + 1] = b->regions[i].region_version;
        WriteU16Le(&out_bytes[base + 2],  b->regions[i].flags);
        WriteU32Le(&out_bytes[base + 4],  off);
        WriteU32Le(&out_bytes[base + 8],  len);
        WriteU32Le(&out_bytes[base + 12], crc);
    }

    /* Header CRC last. */
    const uint32_t hdr_crc = FusionPathePkg_ComputeHeaderCrc(out_bytes, (uint16_t)header_size);
    WriteU32Le(&out_bytes[FUSION_PATHE_PKG_HEADER_CRC_OFF], hdr_crc);
    return total;
}

/* Recompute header CRC after caller mutates header bytes (but not the CRC). */
static void ResealHeaderCrc(uint8_t *bytes, uint16_t header_length)
{
    const uint32_t hdr_crc = FusionPathePkg_ComputeHeaderCrc(bytes, header_length);
    WriteU32Le(&bytes[FUSION_PATHE_PKG_HEADER_CRC_OFF], hdr_crc);
}

/* Recompute one entry's payload CRC after caller mutates entry length/offset
 * or payload contents.
 */
static void ResealEntryCrc(uint8_t *bytes,
                           uint16_t header_length,
                           uint16_t entry_index)
{
    const size_t base = (size_t)header_length
                      + (size_t)entry_index * FUSION_PATHE_PKG_ENTRY_SIZE;
    const uint32_t off = (uint32_t)(((uint32_t)bytes[base + 4])
                          | ((uint32_t)bytes[base + 5] << 8)
                          | ((uint32_t)bytes[base + 6] << 16)
                          | ((uint32_t)bytes[base + 7] << 24));
    const uint32_t len = (uint32_t)(((uint32_t)bytes[base + 8])
                          | ((uint32_t)bytes[base + 9] << 8)
                          | ((uint32_t)bytes[base + 10] << 16)
                          | ((uint32_t)bytes[base + 11] << 24));
    const uint32_t crc = FusionSavestate_Crc32(&bytes[off], (size_t)len);
    WriteU32Le(&bytes[base + 12], crc);
}

/* ---- Individual tests ---- */

static const uint32_t kCurrentRequiredMask =
    FUSION_PATHE_PKG_SUPPORTED_REQUIRED_MASK_CURRENT_BUILD;
static const uint32_t kCurrentOptionalMask =
    FUSION_PATHE_PKG_SUPPORTED_OPTIONAL_MASK_CURRENT_BUILD;

static void test_reason_string_table(void)
{
    fprintf(stderr, "[test] pathe pkg: reason-code strings\n");
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_Ok), "PKG_OK") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_BadMagic), "PKG_BAD_MAGIC") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_BadVersion), "PKG_BAD_VERSION") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_BadHeaderCrc), "PKG_BAD_HEADER_CRC") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_RegionMissing), "PKG_REGION_MISSING") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_RegionBadVersion), "PKG_REGION_BAD_VERSION") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_RegionBadCrc), "PKG_REGION_BAD_CRC") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_RegionUnsupported), "PKG_REGION_UNSUPPORTED") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_RegionRangeInvalid), "PKG_REGION_RANGE_INVALID") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_RegionOverlap), "PKG_REGION_OVERLAP") == 0);
    PKG_EXPECT(strcmp(FusionPathePkg_ReasonString(kFusionPathePkg_RegionBadLength), "PKG_REGION_BAD_LENGTH") == 0);
}

static void test_slot_mapping_roundtrip(void)
{
    fprintf(stderr, "[test] pathe pkg: slot <-> region_id mapping\n");
    PKG_EXPECT_EQ_U(FusionPathePkg_RegionSlot(FUSION_PATHE_REGION_ID_MANIFEST),
                    kFusionPathePkgSlot_PackageManifest);
    PKG_EXPECT_EQ_U(FusionPathePkg_RegionSlot(FUSION_PATHE_REGION_ID_WRAM),
                    kFusionPathePkgSlot_Wram);
    PKG_EXPECT_EQ_U(FusionPathePkg_RegionSlot(FUSION_PATHE_REGION_ID_HRAM),
                    kFusionPathePkgSlot_Hram);
    PKG_EXPECT_EQ_U(FusionPathePkg_RegionSlot(FUSION_PATHE_REGION_ID_APU),
                    kFusionPathePkgSlot_Apu);
    PKG_EXPECT_EQ_U(FusionPathePkg_RegionSlot(0x55u),
                    FUSION_PATHE_PKG_SLOT_INVALID);
    PKG_EXPECT_EQ_U(FusionPathePkg_SlotRegionId(kFusionPathePkgSlot_Apu),
                    FUSION_PATHE_REGION_ID_APU);
    PKG_EXPECT_EQ_U(FusionPathePkg_ExpectedRegionVersion(kFusionPathePkgSlot_PackageManifest), 2u);
    PKG_EXPECT_EQ_U(FusionPathePkg_ExpectedRegionVersion(kFusionPathePkgSlot_Wram), 1u);
    uint32_t len = 0u;
    PKG_EXPECT(FusionPathePkg_ExpectedRegionLength(kFusionPathePkgSlot_PackageManifest, &len));
    PKG_EXPECT_EQ_U(len, FUSION_PATHE_REGION_LEN_MANIFEST);
    PKG_EXPECT(FusionPathePkg_ExpectedRegionLength(kFusionPathePkgSlot_Wram, &len));
    PKG_EXPECT_EQ_U(len, FUSION_PATHE_REGION_LEN_WRAM);
    PKG_EXPECT(FusionPathePkg_ExpectedRegionLength(kFusionPathePkgSlot_Hram, &len));
    PKG_EXPECT_EQ_U(len, FUSION_PATHE_REGION_LEN_HRAM);
}

static void test_valid_minimal_batch_a_package_loads_ok(void)
{
    fprintf(stderr, "[test] pathe pkg: valid minimal Batch A package -> PKG_OK\n");
    Builder_t b;
    BuilderInit(&b);
    BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);

    /* Sanity-check parsed header. */
    FusionPathePkgHeader_t hdr;
    PKG_EXPECT(FusionPathePkg_ParseHeader(buf, pkg_len, &hdr));
    PKG_EXPECT_EQ_U(hdr.magic, FUSION_PATHE_PKG_MAGIC);
    PKG_EXPECT_EQ_U(hdr.package_version, FUSION_PATHE_PKG_VERSION_CURRENT);
    PKG_EXPECT_EQ_U(hdr.header_length, FUSION_PATHE_PKG_HEADER_SIZE);
    PKG_EXPECT_EQ_U(hdr.region_count, 3u);
    const uint32_t expect_required =
        (1u << kFusionPathePkgSlot_PackageManifest) |
        (1u << kFusionPathePkgSlot_Wram) |
        (1u << kFusionPathePkgSlot_Hram);
    PKG_EXPECT_EQ_U(hdr.required_region_bitmap, expect_required);

    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_Ok);
    PKG_EXPECT_EQ_U(v.offending_entry_index, FUSION_PATHE_PKG_NO_ENTRY);
}

static void test_bad_magic_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: bad magic -> PKG_BAD_MAGIC\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    /* Corrupt magic + reseal header CRC so we are sure magic is the FIRST
     * failure surfaced rather than CRC. */
    WriteU32Le(&buf[0], 0xBADBADBAu);
    ResealHeaderCrc(buf, FUSION_PATHE_PKG_HEADER_SIZE);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_BadMagic);
}

static void test_bad_package_version_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: bad package_version -> PKG_BAD_VERSION\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    WriteU16Le(&buf[4], (uint16_t)(FUSION_PATHE_PKG_VERSION_CURRENT + 1u));
    ResealHeaderCrc(buf, FUSION_PATHE_PKG_HEADER_SIZE);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_BadVersion);
}

static void test_bad_header_crc_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: bad header CRC -> PKG_BAD_HEADER_CRC\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    /* Flip a CRC byte. */
    buf[FUSION_PATHE_PKG_HEADER_CRC_OFF] ^= 0xFFu;
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_BadHeaderCrc);
}

static void test_missing_required_region_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: required WRAM missing -> PKG_REGION_MISSING\n");
    /* Build a package that declares manifest+WRAM+HRAM required but omits
     * the WRAM entry (only manifest+HRAM entries).
     */
    Builder_t b; BuilderInit(&b);
    BuilderAddRegion(&b, FUSION_PATHE_REGION_ID_MANIFEST, 2u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED, 65u);
    BuilderAddRegion(&b, FUSION_PATHE_REGION_ID_HRAM, 1u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED,
                     FUSION_PATHE_REGION_LEN_HRAM);
    /* Bitmap still demands WRAM. */
    b.required_region_bitmap =
        (1u << kFusionPathePkgSlot_PackageManifest) |
        (1u << kFusionPathePkgSlot_Wram) |
        (1u << kFusionPathePkgSlot_Hram);

    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionMissing);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_Wram);
    PKG_EXPECT_EQ_U(v.offending_region_id, FUSION_PATHE_REGION_ID_WRAM);
}

static void test_required_region_bad_version_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: required region bad version -> PKG_REGION_BAD_VERSION\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    /* Bump manifest region_version to a value we don't claim to support. */
    b.regions[0].region_version = 99u;
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionBadVersion);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_PackageManifest);
}

static void test_required_region_bad_crc_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: required region bad CRC -> PKG_REGION_BAD_CRC\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);

    /* Find WRAM entry payload offset and flip a payload byte without
     * resealing the entry CRC.  Manifest is entry index 0; WRAM is entry 1.
     */
    const size_t entry_base = FUSION_PATHE_PKG_HEADER_SIZE
                            + 1u * FUSION_PATHE_PKG_ENTRY_SIZE;
    const uint32_t off = (uint32_t)(((uint32_t)buf[entry_base + 4])
                         | ((uint32_t)buf[entry_base + 5] << 8)
                         | ((uint32_t)buf[entry_base + 6] << 16)
                         | ((uint32_t)buf[entry_base + 7] << 24));
    buf[off] ^= 0x55u;

    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionBadCrc);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_Wram);
}

static void test_required_region_bad_length_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: required region bad length -> PKG_REGION_BAD_LENGTH\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);

    /* Shrink WRAM by one byte while resealing its payload CRC.  This makes the
     * package range/CRC clean but semantically wrong. */
    const size_t entry_base = FUSION_PATHE_PKG_HEADER_SIZE
                            + 1u * FUSION_PATHE_PKG_ENTRY_SIZE;
    WriteU32Le(&buf[entry_base + 8], FUSION_PATHE_REGION_LEN_WRAM - 1u);
    ResealEntryCrc(buf, FUSION_PATHE_PKG_HEADER_SIZE, 1u);

    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionBadLength);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_Wram);
    PKG_EXPECT_EQ_U(v.offending_region_id, FUSION_PATHE_REGION_ID_WRAM);
}

static void test_unsupported_future_required_region_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: future APU declared required -> PKG_REGION_UNSUPPORTED\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    /* Add an APU entry marked required AND demand it in the header bitmap. */
    BuilderAddRegion(&b, FUSION_PATHE_REGION_ID_APU, 1u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED, 32u);
    b.required_region_bitmap |= (1u << kFusionPathePkgSlot_Apu);

    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionUnsupported);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_Apu);
}

static void test_unsupported_future_required_via_bitmap_only(void)
{
    fprintf(stderr,
            "[test] pathe pkg: future cart slot required by bitmap (no entry) -> PKG_REGION_UNSUPPORTED\n");
    /* No cart entry, but bitmap demands it.  The missing-required path
     * classifies this as Unsupported because the slot is not in the build's
     * supported_required_mask. */
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    b.required_region_bitmap |= (1u << kFusionPathePkgSlot_CartMbcScalar);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionUnsupported);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_CartMbcScalar);
}

static void test_optional_unsupported_metadata_is_allowed(void)
{
    fprintf(stderr,
            "[test] pathe pkg: optional unsupported APU metadata -> PKG_OK\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    /* Add APU entry with optional metadata flag; bitmap stays at Batch A
     * defaults, optional bit set for APU. */
    BuilderAddRegion(&b, FUSION_PATHE_REGION_ID_APU, 1u,
                     FUSION_PATHE_REGION_FLAG_OPTIONAL_META |
                     FUSION_PATHE_REGION_FLAG_UNSUPPORTED_META,
                     24u);
    b.optional_region_bitmap |= (1u << kFusionPathePkgSlot_Apu);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_Ok);
}

static void test_malformed_offset_length_is_refused(void)
{
    fprintf(stderr,
            "[test] pathe pkg: entry length runs past package -> PKG_REGION_RANGE_INVALID\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);

    /* Inflate HRAM (entry index 2) length so payload runs past package end. */
    const size_t entry_base = FUSION_PATHE_PKG_HEADER_SIZE
                            + 2u * FUSION_PATHE_PKG_ENTRY_SIZE;
    WriteU32Le(&buf[entry_base + 8], (uint32_t)(pkg_len + 100u));
    /* Recompute HRAM payload CRC over the (impossibly large) range that
     * would be addressed; but we cannot actually read past buf, so do NOT
     * reseal that entry CRC -- the validator must reject by RANGE before
     * touching CRC.  Header CRC also is independent of entry contents.
     */
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionRangeInvalid);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_Hram);
}

static void test_overlap_between_entries_is_refused(void)
{
    fprintf(stderr,
            "[test] pathe pkg: two entries overlap -> PKG_REGION_OVERLAP\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);

    /* Force HRAM (entry 2) to point inside WRAM payload (entry 1). */
    const size_t wram_base = FUSION_PATHE_PKG_HEADER_SIZE
                           + 1u * FUSION_PATHE_PKG_ENTRY_SIZE;
    const uint32_t wram_off = (uint32_t)(((uint32_t)buf[wram_base + 4])
                              | ((uint32_t)buf[wram_base + 5] << 8)
                              | ((uint32_t)buf[wram_base + 6] << 16)
                              | ((uint32_t)buf[wram_base + 7] << 24));

    const size_t hram_base = FUSION_PATHE_PKG_HEADER_SIZE
                           + 2u * FUSION_PATHE_PKG_ENTRY_SIZE;
    WriteU32Le(&buf[hram_base + 4], wram_off + 8u);  /* overlaps WRAM */
    WriteU32Le(&buf[hram_base + 8], 16u);
    /* Reseal HRAM CRC over the overlapping bytes so we are sure the
     * overlap fires before CRC. */
    ResealEntryCrc(buf, FUSION_PATHE_PKG_HEADER_SIZE, 2u);

    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionOverlap);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_Hram);
}

static void test_payload_offset_overlaps_table_is_refused(void)
{
    fprintf(stderr,
            "[test] pathe pkg: entry offset inside header/table -> PKG_REGION_RANGE_INVALID\n");
    Builder_t b; BuilderInit(&b); BuilderAddBatchARequiredRegions(&b);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);

    /* Force manifest entry's offset to point at the very beginning of the
     * region table -- inside the structural area. */
    const size_t entry_base = FUSION_PATHE_PKG_HEADER_SIZE
                            + 0u * FUSION_PATHE_PKG_ENTRY_SIZE;
    WriteU32Le(&buf[entry_base + 4], (uint32_t)FUSION_PATHE_PKG_HEADER_SIZE);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_RegionRangeInvalid);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_PackageManifest);
}

static void test_truncated_buffer_is_refused(void)
{
    fprintf(stderr, "[test] pathe pkg: truncated buffer -> PKG_TRUNCATED_HEADER\n");
    uint8_t buf[FUSION_PATHE_PKG_HEADER_SIZE - 1];
    memset(buf, 0, sizeof(buf));
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, sizeof(buf), kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_TruncatedHeader);

    v = FusionPathePkg_Validate(NULL, 0u, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_TruncatedHeader);
}

static void test_duplicate_required_region_is_refused(void)
{
    fprintf(stderr,
            "[test] pathe pkg: duplicate required slot -> PKG_DUPLICATE_REGION\n");
    Builder_t b; BuilderInit(&b);
    /* Two manifest entries, both flagged required. */
    BuilderAddRegion(&b, FUSION_PATHE_REGION_ID_MANIFEST, 2u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED,
                     FUSION_PATHE_REGION_LEN_MANIFEST);
    BuilderAddRegion(&b, FUSION_PATHE_REGION_ID_MANIFEST, 2u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED,
                     FUSION_PATHE_REGION_LEN_MANIFEST);
    BuilderAddRegion(&b, FUSION_PATHE_REGION_ID_WRAM, 1u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED,
                     FUSION_PATHE_REGION_LEN_WRAM);
    BuilderAddRegion(&b, FUSION_PATHE_REGION_ID_HRAM, 1u,
                     FUSION_PATHE_REGION_FLAG_REQUIRED,
                     FUSION_PATHE_REGION_LEN_HRAM);
    uint8_t buf[BUILDER_OUT_CAP];
    const size_t pkg_len = BuilderEmit(&b, buf, sizeof(buf));
    PKG_EXPECT(pkg_len > 0u);
    FusionPathePkgValidation_t v = FusionPathePkg_Validate(
        buf, pkg_len, kCurrentRequiredMask, kCurrentOptionalMask);
    PKG_EXPECT_EQ_U(v.status, kFusionPathePkg_DuplicateRegion);
    PKG_EXPECT_EQ_U(v.offending_slot, kFusionPathePkgSlot_PackageManifest);
}

/* ---- Runner ---- */

void RunFusionPathePkgSelftests(int *pass_out, int *fail_out);

void RunFusionPathePkgSelftests(int *pass_out, int *fail_out)
{
    s_pass = 0;
    s_fail = 0;

    test_reason_string_table();
    test_slot_mapping_roundtrip();
    test_valid_minimal_batch_a_package_loads_ok();
    test_bad_magic_is_refused();
    test_bad_package_version_is_refused();
    test_bad_header_crc_is_refused();
    test_missing_required_region_is_refused();
    test_required_region_bad_version_is_refused();
    test_required_region_bad_crc_is_refused();
    test_required_region_bad_length_is_refused();
    test_unsupported_future_required_region_is_refused();
    test_unsupported_future_required_via_bitmap_only();
    test_optional_unsupported_metadata_is_allowed();
    test_malformed_offset_length_is_refused();
    test_overlap_between_entries_is_refused();
    test_payload_offset_overlaps_table_is_refused();
    test_truncated_buffer_is_refused();
    test_duplicate_required_region_is_refused();

    if (pass_out != NULL) {
        *pass_out += s_pass;
    }
    if (fail_out != NULL) {
        *fail_out += s_fail;
    }
}
