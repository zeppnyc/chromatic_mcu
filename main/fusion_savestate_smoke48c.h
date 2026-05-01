#pragma once

#include <stdbool.h>

/*
 * Phase 4.8c MCU RAM-only round-trip smoke harness.
 *
 * Manual console command for hardware bring-up only:
 *   smoke48c ramroundtrip
 *
 * Sequence (no flash writes, no hotkeys, no Top/CPU/Timer restore):
 *   A: BEGIN_SAVE  -> read original HRAM/WRAM into MCU RAM
 *   B: BEGIN_LOAD  -> write XOR-pattern temp HRAM/WRAM
 *   C: BEGIN_SAVE  -> read back, compare against temp (non-noop write proof)
 *   D: BEGIN_LOAD  -> write original HRAM/WRAM back (restore)
 *   E: BEGIN_SAVE  -> read back, compare against original (restore proof)
 *
 * Pre/post validates the existing persistent slot but never erases or
 * rewrites it. The persistent slot is expected to remain valid across
 * the entire round trip.
 *
 * 4.8c does not bind hotkeys, does not restore Top/CPU/Timer, and does
 * not claim full Phase 4.8 pass. See
 * project-wiki/50_decisions/fusion-savestate-phase4-8-acceptance-test.md.
 */

bool FusionSavestate_RamRoundTripSlot0(void);
void FusionSavestateSmoke48c_RegisterCommands(void);
