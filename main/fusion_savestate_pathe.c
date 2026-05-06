/*
 * Path-e (4.8d) MCU-side load orchestration.  See fusion_savestate_pathe.h
 * for the full sequence specification.
 *
 * IMPLEMENTATION STATUS (2026-05-04, Gate 3b complete):
 *   - Pure-C protocol layer (region 0xFE enum, payload builder, extract
 *     helper) is fully implemented in fusion_savestate.h/c with host
 *     unit tests (193/193 passing).
 *   - The actual ESP-IDF orchestration is implemented as
 *     `FusionSavestate_QuickLoadSlot0_PathE()` in fusion_savestate_smoke48a.c
 *     (line ~1103).  It runs the full 11-step sequence:
 *       InitStorage -> ValidateSlot -> FindNewestValidSlot ->
 *       ReadSlotHeader/dirs -> PauseFpgaTraffic + UartOwnerAcquire ->
 *       DrainRx + EndSession clear -> BEGIN_LOAD ->
 *       LoadSlotToFpga_PathE (writes regions 0x01/0x03/0x09/0x0C/0xFE
 *       including gbreset assert/release) -> EndSessionCleanup ->
 *       ResumeFpgaTraffic + UartOwnerRelease.
 *   - Chord dispatcher (smoke48a.c:1593) routes button chord
 *     `LoadRequested` to QuickLoadSlot0_PathE; toast UI shows result.
 *
 * The decision to fold orchestration into smoke48a.c (instead of duplicating
 * its UART owner / region streaming / preflight drain helpers here) was
 * recommendation (b) from the original design: minimal change, reuse proven
 * smoke48a helpers.  All callers should use
 * `FusionSavestate_QuickLoadSlot0_PathE()` directly.
 *
 * The function below (`FusionPathE_QuickLoadFromSlot`) is a thin wrapper
 * preserved for the single-slot UI signature documented in this module's
 * header.  Only slot 0 is accepted; non-zero slot IDs fail closed instead
 * of silently loading a different saved state.
 *
 * Gate 3c hardware bring-up uses this orchestration unchanged.
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
