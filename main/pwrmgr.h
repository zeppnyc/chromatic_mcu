#pragma once

#include <stdbool.h>

void PwrMgr_Task(void *arg);
void PwrMgr_TriggerLightSleep(void);
void PwrMgr_IdleTimerPet(void);
void PwrMgr_IdleTimerSuspend(void);
void PwrMgr_IdleTimerResume(void);
bool PwrMgr_IsLPMActive(void);
void PwrMgr_SetLPM(const bool Active);
