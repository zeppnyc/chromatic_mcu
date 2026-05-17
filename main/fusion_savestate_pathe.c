/*
 * Historical PathE compatibility wrapper.
 *
 * The active load path is the Design Lock local-restore route implemented by
 * FusionSavestate_QuickLoadSlot0(): write Top/CPU/Timer/HRAM/WRAM, then
 * WRITE_COMMIT.  This file keeps the older single-slot UI wrapper name alive
 * while preventing callers from using the retired 0xFE cold-boot stub route.
 */

#include "fusion_savestate_pathe.h"

#include "fusion_savestate_smoke48a.h"

FusionPathELoadResult_t FusionPathE_QuickLoadFromSlot(uint32_t slot)
{
    if (slot != 0u) {
        return kFusionPathELoad_BadArg;
    }
    const bool ok = FusionSavestate_QuickLoadSlot0_PathE();
    return ok ? kFusionPathELoad_Ok : kFusionPathELoad_StatePortFailed;
}
