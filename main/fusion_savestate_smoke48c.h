#pragma once

#include <stdbool.h>

/*
 * Phase 4.8c v2 MCU RAM-only round-trip smoke harness.
 *
 * Manual console command for hardware bring-up only:
 *   smoke48c ramroundtrip
 *
 * Single mixed paused session.  Inside one BEGIN_TEST_RW /
 * END_SESSION pair, session_pause stays asserted continuously, so
 * the GB CPU never runs between any harness write and its readback:
 *
 *   BEGIN_TEST_RW
 *     R1: read original HRAM/WRAM into MCU RAM
 *     W1: write XOR-pattern temp HRAM/WRAM
 *     R2: re-read temp HRAM/WRAM; bit-exact compare
 *     W2: write original HRAM/WRAM back (restore)
 *     R3: re-read original HRAM/WRAM; bit-exact compare
 *   END_SESSION
 *
 * Pre/post validates the existing persistent slot but never erases or
 * rewrites it.  The persistent slot is expected to remain valid across
 * the entire round trip.
 *
 * Requires a P48D (or newer) FPGA bitstream that accepts
 * OP_BEGIN_TEST_RW.  An older P48C bridge rejects the BEGIN with
 * ERR_NO_SESSION; the harness reports that mismatch and exits cleanly.
 *
 * 4.8c does not bind hotkeys, does not restore Top/CPU/Timer, and does
 * not claim full Phase 4.8 pass.  See
 * project-wiki/50_decisions/fusion-savestate-phase4-8-acceptance-test.md.
 */

bool FusionSavestate_RamRoundTripSlot0(void);
void FusionSavestateSmoke48c_RegisterCommands(void);
