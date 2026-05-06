#pragma once

#include "osd_shared.h"

#include <stdbool.h>

/*
 * Path-e (4.8d) Save Slot menu entry for System tab.
 *
 * Per project-wiki/50_decisions/fusion-savestate-phase4-design-lock.md
 * line 472-474: "read-only System menu status item after Firmware and
 * Player: Save Slot, showing EMPTY or the saved game name when the
 * slot can identify it."
 *
 * This widget is presentation-only.  Slot state is pushed in by main
 * (via SaveSlot_SetText) so this component does not pull in storage /
 * fusion_savestate dependencies.
 */

#ifdef __cplusplus
extern "C" {
#endif

OSD_Result_t SaveSlot_Init(OSD_Widget_t *pWidget);
OSD_Result_t SaveSlot_Draw(void *arg);
OSD_Result_t SaveSlot_OnTransition(void *arg);

/*
 * Update displayed slot status text.  Caller (main) may use:
 *   - "EMPTY"   on boot if no valid slot, or after slot invalidation
 *   - "<HASH>"  short hex of game_id_hash for known slot
 *   - GB title  if a separate title lookup becomes available
 *
 * Text is copied; safe to call from non-OSD task.  Max length 23 chars
 * (truncated if longer).
 */
void SaveSlot_SetText(const char *text);

#ifdef __cplusplus
}
#endif
