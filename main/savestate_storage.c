#include "savestate_storage.h"

#if defined(ESP_PLATFORM)
#include "esp_partition.h"
#endif

#include <string.h>

/*
 * Savestate storage layer — implementation.
 *
 * The mock backend is used by host tests.  ESP-IDF builds can also use a
 * single custom data partition for persistent smoke-test slots.
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

static FusionStorageResult_t WriteStorage(FusionStorage_t *s,
                                          uint32_t offset,
                                          const uint8_t *data,
                                          uint32_t length)
{
    if (s == NULL || data == NULL) {
        return kFusionStorage_BadArg;
    }
    if (s->kind == kFusionStorageBackend_Mock) {
        return WriteMock(s, offset, data, length);
    }
#if defined(ESP_PLATFORM)
    if (s->kind == kFusionStorageBackend_Partition) {
        const esp_partition_t *partition = (const esp_partition_t *)s->partition;
        if (partition == NULL) {
            return kFusionStorage_NotInitialized;
        }
        if (offset + length > partition->size) {
            return kFusionStorage_OutOfSpace;
        }
        return (esp_partition_write(partition, offset, data, length) == ESP_OK)
            ? kFusionStorage_Ok
            : kFusionStorage_WrongState;
    }
#endif
    return kFusionStorage_NotInitialized;
}

static FusionStorageResult_t ReadStorage(const FusionStorage_t *s,
                                         uint32_t offset,
                                         uint8_t *data,
                                         uint32_t length)
{
    if (s == NULL || data == NULL) {
        return kFusionStorage_BadArg;
    }
    if (s->kind == kFusionStorageBackend_Mock) {
        return ReadMock(s, offset, data, length);
    }
#if defined(ESP_PLATFORM)
    if (s->kind == kFusionStorageBackend_Partition) {
        const esp_partition_t *partition = (const esp_partition_t *)s->partition;
        if (partition == NULL) {
            return kFusionStorage_NotInitialized;
        }
        if (offset + length > partition->size) {
            return kFusionStorage_OutOfSpace;
        }
        return (esp_partition_read(partition, offset, data, length) == ESP_OK)
            ? kFusionStorage_Ok
            : kFusionStorage_WrongState;
    }
#endif
    return kFusionStorage_NotInitialized;
}

static FusionStorageResult_t EraseStorage(FusionStorage_t *s,
                                          uint32_t offset,
                                          uint32_t length)
{
    if (s == NULL) {
        return kFusionStorage_BadArg;
    }
    if (s->kind == kFusionStorageBackend_Mock) {
        if (offset + length > s->mock_size) {
            return kFusionStorage_OutOfSpace;
        }
        memset(&s->mock_backing[offset], 0xFFu, length);
        return kFusionStorage_Ok;
    }
#if defined(ESP_PLATFORM)
    if (s->kind == kFusionStorageBackend_Partition) {
        const esp_partition_t *partition = (const esp_partition_t *)s->partition;
        if (partition == NULL) {
            return kFusionStorage_NotInitialized;
        }
        if (offset + length > partition->size) {
            return kFusionStorage_OutOfSpace;
        }
        return (esp_partition_erase_range(partition, offset, length) == ESP_OK)
            ? kFusionStorage_Ok
            : kFusionStorage_WrongState;
    }
#endif
    return kFusionStorage_NotInitialized;
}

static FusionStorageResult_t ComputeCrc(FusionStorage_t *s,
                                        uint32_t offset,
                                        uint32_t length,
                                        uint32_t *crc_out)
{
    if (s == NULL || crc_out == NULL) {
        return kFusionStorage_BadArg;
    }
    if (length == 0u) {
        *crc_out = 0u;
        return kFusionStorage_Ok;
    }
    if (s->kind == kFusionStorageBackend_Mock) {
        if (offset + length > s->mock_size) {
            return kFusionStorage_OutOfSpace;
        }
        *crc_out = FusionSavestate_Crc32(&s->mock_backing[offset], length);
        return kFusionStorage_Ok;
    }

    uint32_t crc = 0u;
    uint8_t buf[256];
    uint32_t pos = 0u;
    while (pos < length) {
        const uint32_t chunk = ((length - pos) > sizeof(buf))
            ? (uint32_t)sizeof(buf)
            : (length - pos);
        FusionStorageResult_t r = ReadStorage(s, offset + pos, buf, chunk);
        if (r != kFusionStorage_Ok) {
            return r;
        }
        crc = FusionSavestate_Crc32Update(crc, buf, chunk);
        pos += chunk;
    }
    *crc_out = crc;
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
#if defined(ESP_PLATFORM)
    if (s == NULL || partition_label == NULL) {
        return kFusionStorage_BadArg;
    }

    const esp_partition_t *partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                 ESP_PARTITION_SUBTYPE_ANY,
                                 partition_label);
    if (partition == NULL) {
        return kFusionStorage_NotImplemented;
    }
    if (partition->size < sizeof(FusionStateHeader_t)) {
        return kFusionStorage_OutOfSpace;
    }

    memset(s, 0, sizeof(*s));
    s->kind        = kFusionStorageBackend_Partition;
    s->partition   = partition;
    s->slot_count  = 1u;
    s->slot_size   = partition->size;
    s->slots       = &s->partition_slot;
    s->active_slot = -1;
    s->initialized = true;

    s->partition_slot.slot_offset       = 0u;
    s->partition_slot.slot_size         = partition->size;
    s->partition_slot.header_offset     = 0u;
    s->partition_slot.payload_offset    = (uint32_t)sizeof(FusionStateHeader_t);
    s->partition_slot.bytes_used        = (uint32_t)sizeof(FusionStateHeader_t);
    s->partition_slot.write_in_progress = false;
    return kFusionStorage_Ok;
#else
    (void)s;
    (void)partition_label;
    return kFusionStorage_NotImplemented;
#endif
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
        if (ReadStorage(s, s->slots[i].header_offset, (uint8_t *)&h, sizeof(h)) != kFusionStorage_Ok) {
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
    FusionStorageResult_t r = EraseStorage(s, s->slots[idx].slot_offset, s->slots[idx].slot_size);
    if (r != kFusionStorage_Ok) {
        return r;
    }
    s->slots[idx].bytes_used        = (uint32_t)sizeof(FusionStateHeader_t);
    s->slots[idx].write_in_progress = true;
    s->active_slot                  = idx;
    s->has_staged_header            = false;
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
    const FusionStorageResult_t r = WriteStorage(s, offset, data, length);
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
        const FusionStorageResult_t r = WriteStorage(s,
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
    FusionStorageResult_t r = ComputeCrc(s,
                                         slot->slot_offset + payload_start_in_slot,
                                         payload_bytes,
                                         &crc);
    if (r != kFusionStorage_Ok) {
        return r;
    }

    header->magic               = FUSION_SAVESTATE_MAGIC;
    header->format_version      = (uint16_t)FUSION_SAVESTATE_FORMAT_VERSION;
    header->header_size         = (uint16_t)sizeof(FusionStateHeader_t);
    header->total_size          = slot->bytes_used;
    header->region_table_offset = directory_offset_in_slot;
    header->region_count        = region_count;
    header->payload_crc32       = crc;
    header->commit_state        = (uint8_t)kFusionCommit_Writing;

    if (s->kind == kFusionStorageBackend_Partition) {
        s->staged_header = *header;
        s->has_staged_header = true;
        return kFusionStorage_Ok;
    }

    r = WriteStorage(s,
                     slot->header_offset,
                     (const uint8_t *)header,
                     (uint32_t)sizeof(*header));
    if (r == kFusionStorage_Ok) {
        s->staged_header = *header;
        s->has_staged_header = true;
    }
    return r;
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
    FusionStorageResult_t r = kFusionStorage_Ok;
    if (s->kind == kFusionStorageBackend_Partition) {
        if (!s->has_staged_header) {
            return kFusionStorage_WrongState;
        }
        hdr = s->staged_header;
    } else {
        r = ReadStorage(s, slot->header_offset, (uint8_t *)&hdr, sizeof(hdr));
        if (r != kFusionStorage_Ok) {
            return r;
        }
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
    r = ComputeCrc(s, slot->slot_offset + payload_start, payload_bytes, &crc);
    if (r != kFusionStorage_Ok) {
        return r;
    }
    if (crc != hdr.payload_crc32) {
        return kFusionStorage_BadCrc;
    }

    /*
     * Mock keeps the original two-step byte flip. Partition writes the header
     * once, after payload+directory verify, so erased/partial headers never
     * validate after power loss.
     */
    hdr.commit_state = (uint8_t)kFusionCommit_Valid;
    r = WriteStorage(s, slot->header_offset, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr));
    if (r != kFusionStorage_Ok) {
        return r;
    }
    slot->write_in_progress = false;
    s->active_slot = -1;
    s->has_staged_header = false;
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
    if (ReadStorage(s, slot->header_offset, (uint8_t *)&hdr, sizeof(hdr)) == kFusionStorage_Ok) {
        if (hdr.magic == FUSION_SAVESTATE_MAGIC) {
            hdr.commit_state = (uint8_t)kFusionCommit_Invalid;
            (void)WriteStorage(s, slot->header_offset, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr));
        }
    }
    slot->write_in_progress = false;
    s->active_slot = -1;
    s->has_staged_header = false;
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
        if (ReadStorage(s, s->slots[i].header_offset, (uint8_t *)&h, sizeof(h)) != kFusionStorage_Ok) {
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
    return ReadStorage(s,
                       s->slots[slot_index].header_offset,
                       (uint8_t *)header_out,
                       (uint32_t)sizeof(*header_out));
}

FusionStorageResult_t FusionStorage_ReadSlotBytes(FusionStorage_t *s,
                                                  uint32_t slot_index,
                                                  uint32_t offset,
                                                  uint8_t *data,
                                                  uint32_t length)
{
    if (s == NULL || !s->initialized || data == NULL) {
        return kFusionStorage_BadArg;
    }
    if (slot_index >= s->slot_count) {
        return kFusionStorage_BadArg;
    }
    if (offset + length > s->slots[slot_index].slot_size) {
        return kFusionStorage_OutOfSpace;
    }
    return ReadStorage(s,
                       s->slots[slot_index].slot_offset + offset,
                       data,
                       length);
}
