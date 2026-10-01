#pragma once

#include "osd_shared.h"

OSD_Result_t HotKeys_DrawBrightness(void* arg);
OSD_Result_t HotKeys_DrawColorTemp(void* arg);
OSD_Result_t HotKeys_DrawResetEmulation(void* arg);
OSD_Result_t HotKeys_OnButton(const Button_t Button, const ButtonState_t State, void *arg);
OSD_Result_t HotKeys_OnTransition(void* arg);
