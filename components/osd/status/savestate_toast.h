#pragma once

#include "osd_shared.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Path-e (4.8d) save/load result toast.
 *
 * Displays a transient text label for a bounded duration, then auto-hides.
 * Used by the path-e save/load handler to give the user immediate feedback:
 *   - SAVED  : after successful quick-save
 *   - LOADED : after successful quick-load
 *   - FAILED : on slot corruption / wrong-game / state-port error
 *
 * Per project-wiki/50_decisions/fusion-savestate-phase4-design-lock.md
 * line 470-471: "transient result text: SAVED, LOADED, or FAILED".
 *
 * Visibility coordinates with the main OSD: when this toast is showing,
 * SetVisibilityState(true) is called so the widget actually draws.  Caller
 * is responsible for not stacking multiple toasts; current call replaces
 * any in-flight toast.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum SavestateToastResult {
    kSavestateToast_Saved = 0,
    kSavestateToast_Loaded,
    kSavestateToast_Failed,
    kSavestateToast_WrongGame,
} SavestateToastResult_t;

#define SAVESTATE_TOAST_DEFAULT_MS  1500u

OSD_Result_t SavestateToast_Initialize(OSD_Widget_t *pWidget);

/*
 * Show the toast with the given result and auto-hide after duration_ms.
 * Pass duration_ms=0 to use SAVESTATE_TOAST_DEFAULT_MS.  Replaces any
 * in-flight toast.  Thread-safe via mutex.
 */
void SavestateToast_Show(SavestateToastResult_t result, uint32_t duration_ms);

/* Hide immediately (e.g., on user input that should clear the toast). */
void SavestateToast_Hide(void);

/* True if a toast is currently visible (within its duration window). */
bool SavestateToast_IsActive(void);

#ifdef __cplusplus
}
#endif
