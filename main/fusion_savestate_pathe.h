#pragma once

#include "fusion_savestate.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Path-e (4.8d) MCU-side load orchestration.
 *
 * This module wraps the path-e specific load sequence on top of the
 * generic state-port API.  The sequence (per Gate 1 contract §8 + Gate 3a
 * FPGA RTL):
 *
 *   1. BEGIN_LOAD (pause GB core, enter load session)
 *   2. WRITE region 0x01 (Top)        -- IO regs / IF / IE / KEY1 / etc
 *   3. WRITE region 0x03 (Timer)
 *   4. WRITE region 0x09 (HRAM)
 *   5. WRITE region 0x0C (WRAM)
 *   6. WRITE region 0xFE bytes 0-14   -- path-e load state (load_mode_active=1
 *                                        + scratch + saved_SP + iff_byte +
 *                                        saved_a + saved_PC)
 *   7. WRITE region 0xFE byte 15 = 1  -- assert gbreset
 *   8. delay >= a few hclk cycles     -- let FPGA reset propagate
 *   9. WRITE region 0xFE byte 15 = 0  -- release gbreset.  CPU now cold-boots
 *                                        from $0000 = stub bytes (since
 *                                        load_mode_active=1 from step 6).
 *   10. END_SESSION                   -- release UART / clear pause
 *
 * After step 10:
 *   - CPU executes stub at $0000-$00FD (DI / LD SP / POP×4 / LD SP imm /
 *     JP $00FA / FF50 tail).
 *   - FF50 write disables boot ROM; CPU fetch falls through to $00FE.
 *   - FPGA cart override returns 6 bytes (FB/00 + 3E saved_A + C3 saved_PC).
 *   - CPU lands at saved_PC with all registers (and IFF) restored.
 *
 * Region 0x02 (CPU regs) is INTENTIONALLY NOT written in path-e mode.  The
 * stub restores AF/BC/DE/HL/SP via POPs from scratch.  Writing region 0x02
 * during path-e load would corrupt the cold-reset PC=0 assumption that
 * brings the stub up.
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
 * Delay between gbreset assert and release.  GB core needs at least a few
 * hclk cycles for reset to propagate through T80 / GBse / video / timer
 * / etc.  hclk is 16.777 MHz so 1 ms is ~16800 cycles, which is plenty
 * of margin.  Real product can tune lower if needed.
 */
#define FUSION_PATHE_GBRESET_HOLD_MS  10u

/*
 * Apply a slot's path-e load sequence to the FPGA.
 *
 * STATUS (2026-05-04, Gate 3b complete):
 *   This is a thin wrapper that delegates to
 *   `FusionSavestate_QuickLoadSlot0_PathE()` in fusion_savestate_smoke48a.c
 *   (the actual orchestration is implemented there to reuse smoke48a's
 *   UART owner / region streaming / preflight drain helpers).
 *
 *   The product UI currently exposes one logical load slot.  This wrapper
 *   accepts only slot 0; non-zero slot IDs return kFusionPathELoad_BadArg
 *   instead of silently loading another state.
 *
 *   The richer 8-value FusionPathELoadResult_t enum is collapsed to
 *   Ok/StatePortFailed in the wrapper because smoke48a returns only bool.
 *   To preserve granular error info, callers should use
 *   `FusionSavestate_QuickLoadSlot0_PathE()` directly and inspect the
 *   smoke48a logs for failure detail.
 *
 * Returns kFusionPathELoad_Ok on success, kFusionPathELoad_BadArg for a
 * non-zero slot, or kFusionPathELoad_StatePortFailed on orchestration failure.
 *
 * Caller does NOT need to hold the UART owner — smoke48a acquires/releases
 * it internally.  Same for FPGA traffic pause and toast UI feedback.
 *
 * Game-id verification (per lock §4.8d wrong-game refusal) is performed in
 * smoke48a before any path-e region 0xFE writes.
 */
FusionPathELoadResult_t FusionPathE_QuickLoadFromSlot(uint32_t slot);

#ifdef __cplusplus
}
#endif
