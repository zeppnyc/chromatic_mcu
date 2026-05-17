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

/*
 * Historical compatibility entry point.  It now dispatches to
 * FusionSavestate_QuickLoadSlot0(), which uses the Design Lock local
 * restore path: Top/CPU/Timer/HRAM/WRAM writes followed by WRITE_COMMIT.
 */
bool FusionSavestate_QuickLoadSlot0_PathE(void);

void FusionSavestateSmoke48a_RegisterCommands(void);

/* ----- Chord-triggered dispatcher ----- */

#include "button.h"

/*
 * Initialize the chord dispatcher task and queue.  Call once during MCU
 * boot (in main.c, after Button_RegisterCommands).  The task consumes
 * save/load chord events and runs the corresponding QuickSave/QuickLoad
 * synchronously.  Save/load are slow (UART transfer of WRAM), so this
 * decouples them from the FPGA RX task.
 */
void FusionSavestate_StartChordDispatcher(void);

/*
 * Post a chord event to the dispatcher.  Safe to call from FPGA RX task.
 * Drops the event silently if the queue is full (prevents button spam
 * from queuing many save attempts).
 */
void FusionSavestate_PostChordRequest(ButtonChordEvent_t event);
