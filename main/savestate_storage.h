#pragma once

#include "fusion_savestate.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Savestate storage abstraction: header struct, region directory,
 * and the two-phase commit API shape required by the architecture
 * doc.
 *
 * Phase 1 ships:
 *   - The struct layout (above, in fusion_savestate.h).
 *   - A clean API surface for slot lifecycle.
 *   - A pure in-memory mock backend used by host self-tests.
 *
 * Real ESP-IDF custom partition integration is deferred to Phase 1.5
 * /Phase 5; the API shape here is what that integration will fill in.
 * No partition table is added in this commit.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    kFusionStorage_Ok = 0,
    kFusionStorage_BadArg,
    kFusionStorage_NotInitialized,
    kFusionStorage_NotImplemented,    /* real-flash backend not yet wired */
    kFusionStorage_OutOfSpace,
    kFusionStorage_NoActiveSlot,
    kFusionStorage_NoValidSlot,
    kFusionStorage_BadHeader,
    kFusionStorage_BadCrc,
    kFusionStorage_WrongState,
} FusionStorageResult_t;

typedef enum {
    kFusionStorageBackend_None = 0,
    kFusionStorageBackend_Mock,       /* in-RAM, used by host tests */
    kFusionStorageBackend_Partition,  /* ESP-IDF custom data partition (deferred) */
} FusionStorageBackendKind_t;

typedef struct {
    uint32_t slot_offset;             /* offset into backing store */
    uint32_t slot_size;
    uint32_t header_offset;           /* relative to backing store */
    uint32_t payload_offset;          /* where region payload begins */
    uint32_t bytes_used;
    bool     write_in_progress;
} FusionStorageSlot_t;

typedef struct {
    FusionStorageBackendKind_t kind;
    uint8_t  *mock_backing;           /* mock backend only; otherwise NULL */
    uint32_t  mock_size;

    uint32_t  slot_count;
    uint32_t  slot_size;

    FusionStorageSlot_t *slots;       /* length == slot_count */
    int32_t   active_slot;            /* slot being staged, -1 if none */
    bool      initialized;
} FusionStorage_t;

#define FUSION_STORAGE_DEFAULT_SLOTS 2u

/*
 * Initialize a mock backend over caller-provided RAM. Backing memory is
 * partitioned into N equal slots; each slot fits a header + region
 * payload + region directory.
 */
FusionStorageResult_t FusionStorage_InitMock(FusionStorage_t *s,
                                             FusionStorageSlot_t *slots,
                                             uint32_t slot_count,
                                             uint8_t *backing,
                                             uint32_t backing_size);

/*
 * Real-partition init shape — wired up in Phase 1.5/5. Always returns
 * kFusionStorage_NotImplemented in Phase 1 so callers can fall back to
 * the mock without conditional compilation.
 */
FusionStorageResult_t FusionStorage_InitFromPartition(FusionStorage_t *s,
                                                      const char *partition_label);

/*
 * Reserve the inactive slot for a new write. Marks the slot in-progress
 * and clears the staged header byte (commit_state = Erased).
 */
FusionStorageResult_t FusionStorage_BeginInactiveSlotWrite(FusionStorage_t *s);

/* Append region payload bytes into the active staged slot. */
FusionStorageResult_t FusionStorage_AppendRegionPayload(FusionStorage_t *s,
                                                        const uint8_t *data,
                                                        uint32_t length);

/*
 * Stage the on-disk header in 'Writing' state, plus the region directory
 * entries that the caller has built. CRC32 over payload+directory is
 * recomputed by the storage layer for consistency with later
 * VerifyAndCommit.
 */
FusionStorageResult_t FusionStorage_StageHeader(FusionStorage_t *s,
                                                FusionStateHeader_t *header,
                                                const FusionRegionDirectory_t *regions,
                                                uint32_t region_count);

/*
 * Re-read the staged header + payload, recompute CRC32, and atomically
 * flip commit_state from 'Writing' to 'Valid' if it matches.
 */
FusionStorageResult_t FusionStorage_VerifyAndCommit(FusionStorage_t *s);

/* Mark the staged slot 'Invalid' and release the in-progress lock. */
FusionStorageResult_t FusionStorage_AbortSlotWrite(FusionStorage_t *s);

/*
 * Pick the slot with the highest commit_generation among slots whose
 * commit_state == Valid. Returns NoValidSlot if none.
 */
FusionStorageResult_t FusionStorage_FindNewestValidSlot(FusionStorage_t *s,
                                                        uint32_t *slot_index_out,
                                                        uint32_t *generation_out);

/* Read the header for a given slot. */
FusionStorageResult_t FusionStorage_ReadSlotHeader(FusionStorage_t *s,
                                                   uint32_t slot_index,
                                                   FusionStateHeader_t *header_out);

#ifdef __cplusplus
}
#endif
