#pragma once

#include "fusion_savestate.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Historical PathE compatibility API.
 *
 * The retired PathE-stub sequence wrote 0xFE and toggled gbreset so a boot
 * stub could jump to saved_PC.  The current path does not use that sequence.
 * It delegates to FusionSavestate_QuickLoadSlot0(), which writes the Design
 * Lock required regions, including CPU 0x02, and then sends WRITE_COMMIT.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    kFusionPathELoad_Ok = 0,
    kFusionPathELoad_BadArg,
    kFusionPathELoad_StorageFailed,
    kFusionPathELoad_HeaderInvalid,
    kFusionPathELoad_GameIdMismatch,
    kFusionPathELoad_StatePortFailed,
    kFusionPathELoad_ExtractFailed,
    kFusionPathELoad_GbresetFailed,
} FusionPathELoadResult_t;

/*
 * Apply a slot load sequence to the FPGA.
 *
 * The product UI currently exposes one logical load slot.  This wrapper
 * accepts only slot 0; non-zero slot IDs return kFusionPathELoad_BadArg.
 * The implementation delegates to the smoke48a local-restore load path.
 */
FusionPathELoadResult_t FusionPathE_QuickLoadFromSlot(uint32_t slot);

#ifdef __cplusplus
}
#endif
