#pragma once

#include <stdbool.h>
#include <stdint.h>

// Bit definitions follow the scheme sent to us by the FPGA
typedef enum Button {
    kButton_Start,
    kButton_Select,
    kButton_B,
    kButton_A,
    kButton_Up,
    kButton_Right,
    kButton_Left,
    kButton_Down,
    kButton_MenuEnAlt,
    kButton_MenuEn,
    kNumButtons,
    kButton_None,
} Button_t;

typedef enum ButtonState {
    kButtonState_None,
    kButtonState_Pressed,
    kNumButtonStates,
} ButtonState_t;

typedef enum ButtonBits {
    kButtonBits_None   = 0,
    kButtonBits_B      = (1 << kButton_B),
    kButtonBits_A      = (1 << kButton_A),
    kButtonBits_Down   = (1 << kButton_Down),
    kButtonBits_Right  = (1 << kButton_Right),
    kButtonBits_Left   = (1 << kButton_Left),
    kButtonBits_Up     = (1 << kButton_Up),
    kButtonBits_MenuEn = (1 << kButton_MenuEn),
    kButtonBits_MenuEnAlt = (1 << kButton_MenuEnAlt),
} ButtonBits_t;

typedef void (*fnOnButtonPokeCb_t)(void);

void Button_Update(const uint16_t NewButtons);
ButtonState_t Button_GetState(const Button_t b);
const char* Button_GetNameStr(const Button_t b);
const char* Button_GetStateStr(const ButtonState_t s);
void Button_ResetAll(void);
void Button_RegisterOnButtonPokeCb(fnOnButtonPokeCb_t Handler);
uint16_t Button_GetPokedInputs(void);
void Button_RegisterCommands(void);

/*
 * Path-e (4.8d) chord/hold detection.
 *
 * Per project-wiki/50_decisions/fusion-savestate-phase4-design-lock.md
 * §4.8d (line 464-469):
 *   - hold MENU + DOWN for save
 *   - hold MENU + UP for load
 *   - same hold threshold for both actions
 *   - consume the recognized chord until all keys are released so the
 *     chord does not also open OSD or pass through to the game/menu
 *
 * Implementation: tick-driven chord state machine.  Caller invokes
 * ButtonChord_Update with the current button bitmap on each FPGA button
 * frame (cadence ~hclk/N from system_monitor).  When the chord has been
 * held for >= ButtonChord_HoldThresholdTicks consecutive ticks, the
 * state machine fires the corresponding event (latched until polled).
 *
 * "Consume until all released" rule: after firing, the state machine
 * stays in Consumed until NewButtons==0 (or only non-chord bits set);
 * subsequent re-entries to the same chord are rejected until a full
 * release.  This prevents the chord from also being interpreted as a
 * normal MenuEn press (which would open OSD).
 */
typedef enum ButtonChordEvent {
    kButtonChord_None = 0,
    kButtonChord_SaveRequested,  /* MENU + DOWN held */
    kButtonChord_LoadRequested,  /* MENU + UP held */
} ButtonChordEvent_t;

/*
 * Number of consecutive Update ticks the chord must remain held before
 * firing.  Existing button.c module declares kMinHold_ticks = 10 but
 * never uses it; we reuse the same threshold for chord detection.
 * Caller may override at startup via ButtonChord_SetHoldThreshold.
 */
#define BUTTON_CHORD_DEFAULT_HOLD_TICKS  10u

void ButtonChord_Reset(void);
void ButtonChord_SetHoldThreshold(uint16_t ticks);

/*
 * Update with current button bitmap.  Returns kButtonChord_None if
 * nothing latches THIS tick; returns one-shot Save/LoadRequested when
 * the chord crosses the hold threshold.  After firing, the chord is
 * "consumed" — must call ButtonChord_Update with NewButtons clear of
 * MENU bits before another Save/Load can fire.
 */
ButtonChordEvent_t ButtonChord_Update(uint16_t NewButtons);

/*
 * Same as ButtonChord_Update but takes an explicit "tick count" for
 * unit testing.  Each tick increments the hold counter when the chord
 * matches.  ButtonChord_Update internally calls this with tick=1.
 */
ButtonChordEvent_t ButtonChord_UpdateWithTick(uint16_t NewButtons,
                                              uint16_t tick_increment);
