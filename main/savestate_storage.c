#include "savestate_storage.h"

#include <string.h>

/*
 * Savestate storage layer — implementation.
 *
 * Phase 1 only ships the mock backend. The partition backend's
 * entry point is wired up but returns NotImplemented; Phase 1.5/5
 * will replace it with esp_partition_* calls and a custom partition
 * table entry. No real flash writes happen in this commit.
 */

static FusionStorageResult_t WriteMock(FusionStorage_t *s,
                                       uint32_t offset,
                                       const uint8_t *data,
                                       uint32_t length)
{
    if (s == NULL || s->kind != kFusionStorageBackend_Mock) {
        return kFusionStorage_NotInitialized;
    }
    if (offset + length > s->mock_size) {
        return kFusionStorage_OutOfSpace;
    }
    memcpy(&s->mock_backing[offset], data, length);
    return kFusionStorage_Ok;
}

static FusionStorageResult_t ReadMock(const FusionStorage_t *s,
                                      uint32_t offset,
                                      uint8_t *data,
                                      uint32_t length)
{
    if (s == NULL || s->kind != kFusionStorageBackend_Mock) {
        return kFusionStorage_NotInitialized;
    }
    if (offset + length > s->mock_size) {
        return kFusionStorage_OutOfSpace;
    }
    memcpy(data, &s->mock_backing[offset], length);
    return kFusionStorage_Ok;
}

FusionStorageResult_t FusionStorage_InitMock(FusionStorage_t *s,
                                             FusionStorageSlot_t *slots,
                                             uint32_t slot_count,
                                             uint8_t *backing,
                                             uint32_t backing_size)
{
    if (s == NULL || slots == NULL || backing == NULL) {
        return kFusionStorage_BadArg;
    }
    if (slot_count == 0u) {
        return kFusionStorage_BadArg;
    }
    if (backing_size < slot_count * sizeof(FusionStateHeader_t)) {
        return kFusionStorage_OutOfSpace;
    }

    memset(s, 0, sizeof(*s));
    s->kind         = kFusionStorageBackend_Mock;
    s->mock_backing = backing;
    s->mock_size    = backing_size;
    s->slot_count   = slot_count;
    s->slot_size    = backing_size / slot_count;
    s->slots        = slots;
    s->active_slot  = -1;
    s->initialized  = true;

    /* Pre-erase all slots so commit_state reads as Erased. */
    memset(backing, 0xFFu, backing_size);
    for (uint32_t i = 0; i < slot_count; ++i) {
        s->slots[i].slot_offset       = i * s->slot_size;
        s->slots[i].slot_size         = s->slot_size;
        s->slots[i].header_offset     = i * s->slot_size;
        s->slots[i].payload_offset    = i * s->slot_size + (uint32_t)sizeof(FusionStateHeader_t);
        s->slots[i].bytes_used        = (uint32_t)sizeof(FusionStateHeader_t);
        s->slots[i].write_in_progress = false;
    }
    return kFusionStorage_Ok;
}

FusionStorageResult_t FusionStorage_InitFromPartition(FusionStorage_t *s,
                                                      const char *partition_label)
{
    (void)s;
    (void)partition_label;
    /*
     * Phase 1: deliberately not implemented. Wiring this up to
     * esp_partition_find_first / esp_partition_erase_range / write +
     * adding a custom partition CSV is the Phase 1.5/5 task.
     */
    return kFusionStorage_NotImplemented;
}

static int32_t FindInactiveSlot(const FusionStorage_t *s)
{
    if (s->slot_count == 0u) {
        return -1;
    }
    /*
     * Pick the slot with the lowest commit_generation among slots whose
     * commit_state is not Valid. If everything is Valid, pick the oldest
     * generation (the one we want to overwrite). This mirrors the
     * "newest-valid wins" load-time rule.
     */
    int32_t pick = -1;
    uint32_t pick_gen = 0u;
    for (uint32_t i = 0; i < s->slot_count; ++i) {
        FusionStateHeader_t h;
        if (ReadMock(s, s->slots[i].header_offset, (uint8_t *)&h, sizeof(h)) != kFusionStorage_Ok) {
            continue;
        }
        if (h.magic != FUSION_SAVESTATE_MAGIC || h.commit_state != (uint8_t)kFusionCommit_Valid) {
            return (int32_t)i;
        }
        if (pick < 0 || h.commit_generation < pick_gen) {
            pick     = (int32_t)i;
            pick_gen = h.commit_generation;
        }
    }
    return pick;
}

FusionStorageResult_t FusionStorage_BeginInactiveSlotWrite(FusionStorage_t *s)
{
    if (s == NULL || !s->initialized) {
        return kFusionStorage_NotInitialized;
    }
    if (s->active_slot >= 0) {
        return kFusionStorage_WrongState;
    }
    const int32_t idx = FindInactiveSlot(s);
    if (idx < 0) {
        return kFusionStorage_OutOfSpace;
    }
    /* Erase the slot and mark in-progress. */
    if (s->kind == kFusionStorageBackend_Mock) {
        memset(&s->mock_backing[s->slots[idx].slot_offset], 0xFFu, s->slots[idx].slot_size);
    }
    s->slots[idx].bytes_used        = (uint32_t)sizeof(FusionStateHeader_t);
    s->slots[idx].write_in_progress = true;
    s->active_slot                  = idx;
    return kFusionStorage_Ok;
}

FusionStorageResult_t FusionStorage_AppendRegionPayload(FusionStorage_t *s,
                                                        const uint8_t *data,
                                                        uint32_t length)
{
    if (s == NULL || !s->initialized || data == NULL) {
        return kFusionStorage_BadArg;
    }
    if (s->active_slot < 0) {
        return kFusionStorage_NoActiveSlot;
    }
    FusionStorageSlot_t *slot = &s->slots[s->active_slot];
    if (slot->bytes_used + length > slot->slot_size) {
        return kFusionStorage_OutOfSpace;
    }
    const uint32_t offset = slot->slot_offset + slot->bytes_used;
    const FusionStorageResult_t r = WriteMock(s, offset, data, length);
    if (r != kFusionStorage_Ok) {
        return r;
    }
    slot->bytes_used += length;
    return kFusionStorage_Ok;
}

FusionStorageResult_t FusionStorage_StageHeader(FusionStorage_t *s,
                                                FusionStateHeader_t *header,
                                                const FusionRegionDirectory_t *regions,
                                                uint32_t region_count)
{
    if (s == NULL || header == NULL) {
        return kFusionStorage_BadArg;
    }
    if (s->active_slot < 0) {
        return kFusionStorage_NoActiveSlot;
    }
    if (region_count > 0u && regions == NULL) {
        return kFusionStorage_BadArg;
    }

    FusionStorageSlot_t *slot = &s->slots[s->active_slot];

    /* Write region directory at end of payload. */
    const uint32_t directory_bytes = region_count * (uint32_t)sizeof(FusionRegionDirectory_t);
    const uint32_t directory_offset_in_slot = slot->bytes_used;
    if (directory_offset_in_slot + directory_bytes > slot->slot_size) {
        return kFusionStorage_OutOfSpace;
    }
    if (region_count > 0u) {
        const FusionStorageResult_t r = WriteMock(s,
                                                  slot->slot_offset + directory_offset_in_slot,
                                                  (const uint8_t *)regions,
                                                  directory_bytes);
        if (r != kFusionStorage_Ok) {
            return r;
        }
        slot->bytes_used += directory_bytes;
    }

    /* Compute payload+directory CRC for the header. */
    const uint32_t payload_start_in_slot = (uint32_t)sizeof(FusionStateHeader_t);
    const uint32_t payload_bytes = slot->bytes_used - payload_start_in_slot;
    uint32_t crc = 0u;
    if (payload_bytes > 0u && s->kind == kFusionStorageBackend_Mock) {
        crc = FusionSavestate_Crc32(
            &s->mock_backing[slot->slot_offset + payload_start_in_slot],
            payload_bytes);
    }

    header->magic               = FUSION_SAVESTATE_MAGIC;
    header->format_version      = (uint16_t)FUSION_SAVESTATE_FORMAT_VERSION;
    header->header_size         = (uint16_t)sizeof(FusionStateHeader_t);
    header->total_size          = slot->bytes_used;
    header->region_table_offset = directory_offset_in_slot;
    header->region_count        = region_count;
    header->payload_crc32       = crc;
    header->commit_state        = (uint8_t)kFusionCommit_Writing;

    return WriteMock(s,
                     slot->header_offset,
                     (const uint8_t *)header,
                     (uint32_t)sizeof(*header));
}

FusionStorageResult_t FusionStorage_VerifyAndCommit(FusionStorage_t *s)
{
    if (s == NULL || !s->initialized) {
        return kFusionStorage_NotInitialized;
    }
    if (s->active_slot < 0) {
        return kFusionStorage_NoActiveSlot;
    }
    FusionStorageSlot_t *slot = &s->slots[s->active_slot];

    FusionStateHeader_t hdr;
    FusionStorageResult_t r = ReadMock(s, slot->header_offset, (uint8_t *)&hdr, sizeof(hdr));
    if (r != kFusionStorage_Ok) {
        return r;
    }
    if (hdr.magic != FUSION_SAVESTATE_MAGIC) {
        return kFusionStorage_BadHeader;
    }
    if (hdr.commit_state != (uint8_t)kFusionCommit_Writing) {
        return kFusionStorage_WrongState;
    }

    const uint32_t payload_start = (uint32_t)sizeof(FusionStateHeader_t);
    const uint32_t payload_bytes = slot->bytes_used - payload_start;
    uint32_t crc = 0u;
    if (payload_bytes > 0u && s->kind == kFusionStorageBackend_Mock) {
        crc = FusionSavestate_Crc32(
            &s->mock_backing[slot->slot_offset + payload_start],
            payload_bytes);
    }
    if (crc != hdr.payload_crc32) {
        return kFusionStorage_BadCrc;
    }

    /* Atomic single-byte flip from Writing -> Valid. */
    hdr.commit_state = (uint8_t)kFusionCommit_Valid;
    r = WriteMock(s, slot->header_offset, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr));
    if (r != kFusionStorage_Ok) {
        return r;
    }
    slot->write_in_progress = false;
    s->active_slot = -1;
    return kFusionStorage_Ok;
}

FusionStorageResult_t FusionStorage_AbortSlotWrite(FusionStorage_t *s)
{
    if (s == NULL || !s->initialized) {
        return kFusionStorage_NotInitialized;
    }
    if (s->active_slot < 0) {
        return kFusionStorage_NoActiveSlot;
    }
    FusionStorageSlot_t *slot = &s->slots[s->active_slot];
    FusionStateHeader_t hdr;
    if (ReadMock(s, slot->header_offset, (uint8_t *)&hdr, sizeof(hdr)) == kFusionStorage_Ok) {
        if (hdr.magic == FUSION_SAVESTATE_MAGIC) {
            hdr.commit_state = (uint8_t)kFusionCommit_Invalid;
            (void)WriteMock(s, slot->header_offset, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr));
        }
    }
    slot->write_in_progress = false;
    s->active_slot = -1;
    return kFusionStorage_Ok;
}

FusionStorageResult_t FusionStorage_FindNewestValidSlot(FusionStorage_t *s,
                                                        uint32_t *slot_index_out,
                                                        uint32_t *generation_out)
{
    if (s == NULL || !s->initialized) {
        return kFusionStorage_NotInitialized;
    }
    int32_t best = -1;
    uint32_t best_gen = 0u;
    for (uint32_t i = 0; i < s->slot_count; ++i) {
        FusionStateHeader_t h;
        if (ReadMock(s, s->slots[i].header_offset, (uint8_t *)&h, sizeof(h)) != kFusionStorage_Ok) {
            continue;
        }
        if (h.magic != FUSION_SAVESTATE_MAGIC) {
            continue;
        }
        if (h.commit_state != (uint8_t)kFusionCommit_Valid) {
            continue;
        }
        if (best < 0 || h.commit_generation > best_gen) {
            best = (int32_t)i;
            best_gen = h.commit_generation;
        }
    }
    if (best < 0) {
        return kFusionStorage_NoValidSlot;
    }
    if (slot_index_out != NULL) {
        *slot_index_out = (uint32_t)best;
    }
    if (generation_out != NULL) {
        *generation_out = best_gen;
    }
    return kFusionStorage_Ok;
}

FusionStorageResult_t FusionStorage_ReadSlotHeader(FusionStorage_t *s,
                                                   uint32_t slot_index,
                                                   FusionStateHeader_t *header_out)
{
    if (s == NULL || !s->initialized || header_out == NULL) {
        return kFusionStorage_BadArg;
    }
    if (slot_index >= s->slot_count) {
        return kFusionStorage_BadArg;
    }
    return ReadMock(s,
                    s->slots[slot_index].header_offset,
                    (uint8_t *)header_out,
                    (uint32_t)sizeof(*header_out));
}
