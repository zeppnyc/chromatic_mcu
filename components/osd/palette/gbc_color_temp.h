#pragma once

#include "osd_shared.h"
#include "settings.h"

typedef uint8_t GBCColorTempLevel_t;

OSD_Result_t GBCColorTemp_Draw(void* arg);
OSD_Result_t GBCColorTemp_OnButton(const Button_t Button, const ButtonState_t State, void* arg);
OSD_Result_t GBCColorTemp_OnTransition(void* arg);

GBCColorTempLevel_t GBCColorTemp_GetLevel(void);
void GBCColorTemp_RegisterOnUpdateCb(fnOnUpdateCb_t fnOnUpdate);

// Sync level from external runtime sources (e.g. FPGA hotkeys) and persist it.
void GBCColorTemp_Update(GBCColorTempLevel_t level);

// Helper for restoring persisted value without forcing a draw.
OSD_Result_t GBCColorTemp_ApplySetting(const SettingValue_t* pValue);
