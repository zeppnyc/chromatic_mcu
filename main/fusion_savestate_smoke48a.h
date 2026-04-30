#pragma once

#include <stdbool.h>

/*
 * Phase 4.8a MCU persistence smoke harness.
 *
 * Stable MCU entry points for the future hotkey layer:
 *   - FusionSavestate_QuickSaveSlot0()
 *   - FusionSavestate_QuickLoadSlot0()
 *
 * 4.8a deliberately does not choose or hardwire the final Chromatic button
 * chord. Registers a manual console command for validation only:
 *   smoke48a save      - capture the Phase 4.7 read matrix and commit slot 0
 *   smoke48a validate  - call QuickLoadSlot0 and report load-ready
 *   smoke48a reboot    - software reboot helper for persistence smoke tests
 *
 * 4.8d hotkey integration constraints:
 *   - Save and load triggers must be distinct.
 *   - Do not conflict with power/sleep/reset/menu/OSD ownership.
 *   - Debounce and prevent repeated accidental triggers.
 *   - Load needs a mis-trigger guard such as long hold, chord, or confirmation.
 *   - Button scanning stays separate from protocol/storage logic.
 *   - Final button chord is deferred until input ownership is reviewed.
 */

bool FusionSavestate_QuickSaveSlot0(void);
bool FusionSavestate_QuickLoadSlot0(void);
void FusionSavestateSmoke48a_RegisterCommands(void);
