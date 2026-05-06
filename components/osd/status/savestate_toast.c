#include "savestate_toast.h"

#include "osd.h"
#include "osd_shared.h"
#include "lvgl.h"
#include "mutex.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

enum {
    kToastOriginX_px = 60,
    kToastOriginY_px = 64,  /* roughly screen center vertically */
};

typedef struct ToastState {
    SavestateToastResult_t result;
    uint64_t hide_at_us;
    bool     active;
    /* Cached LVGL object so we don't recreate every Draw cycle. */
    lv_obj_t *pLabel;
} ToastState_t;

__attribute__((unused)) static const char *TAG = "ss_toast";

static ToastState_t _Ctx = {
    .result     = kSavestateToast_Saved,
    .hide_at_us = 0u,
    .active     = false,
    .pLabel     = NULL,
};

static const char *ToastText(SavestateToastResult_t r)
{
    switch (r) {
        case kSavestateToast_Saved:  return "SAVED";
        case kSavestateToast_Loaded: return "LOADED";
        case kSavestateToast_Failed: return "FAILED";
        case kSavestateToast_WrongGame: return "WRONG GAME";
        default:                     return "";
    }
}

static OSD_Result_t SavestateToast_Draw(void *arg)
{
    if (arg == NULL) {
        return kOSD_Result_Err_NullDataPtr;
    }
    lv_obj_t *const pScreen = (lv_obj_t *const)arg;

    bool active_local = false;
    SavestateToastResult_t result_local = kSavestateToast_Saved;

    if (Mutex_Take(kMutexKey_OSD) == kMutexResult_Ok) {
        const uint64_t now_us = (uint64_t)esp_timer_get_time();
        if (_Ctx.active && now_us >= _Ctx.hide_at_us) {
            _Ctx.active = false;
        }
        active_local = _Ctx.active;
        result_local = _Ctx.result;
        (void) Mutex_Give(kMutexKey_OSD);
    }

    if (active_local) {
        if (_Ctx.pLabel == NULL) {
            _Ctx.pLabel = lv_label_create(pScreen);
            lv_obj_add_style(_Ctx.pLabel, OSD_GetStyleTextWhite_L(), 0);
            lv_obj_set_align(_Ctx.pLabel, LV_ALIGN_TOP_LEFT);
            lv_obj_set_pos(_Ctx.pLabel, kToastOriginX_px, kToastOriginY_px);
        }
        lv_label_set_text(_Ctx.pLabel, ToastText(result_local));
    } else {
        if (_Ctx.pLabel != NULL) {
            lv_obj_del(_Ctx.pLabel);
            _Ctx.pLabel = NULL;
        }
    }

    return kOSD_Result_Ok;
}

static OSD_Result_t SavestateToast_OnTransition(void *arg)
{
    (void)arg;
    /* No transition state to clear/setup beyond what Draw handles. */
    return kOSD_Result_Ok;
}

OSD_Result_t SavestateToast_Initialize(OSD_Widget_t *pWidget)
{
    if (pWidget == NULL) {
        return kOSD_Result_Err_NullDataPtr;
    }
    pWidget->Name = "SavestateToast";
    pWidget->fnDraw = SavestateToast_Draw;
    pWidget->fnOnTransition = SavestateToast_OnTransition;
    pWidget->fnOnButton = NULL;
    return kOSD_Result_Ok;
}

void SavestateToast_Show(SavestateToastResult_t result, uint32_t duration_ms)
{
    if (duration_ms == 0u) {
        duration_ms = SAVESTATE_TOAST_DEFAULT_MS;
    }
    if (Mutex_Take(kMutexKey_OSD) == kMutexResult_Ok) {
        _Ctx.result = result;
        _Ctx.hide_at_us = (uint64_t)esp_timer_get_time() +
                          (uint64_t)duration_ms * 1000ull;
        _Ctx.active = true;
        (void) Mutex_Give(kMutexKey_OSD);
    }
    /* Coordinate with main OSD: ensure visible while toast active. */
    OSD_SetVisiblityState(true);
}

void SavestateToast_Hide(void)
{
    if (Mutex_Take(kMutexKey_OSD) == kMutexResult_Ok) {
        _Ctx.active = false;
        (void) Mutex_Give(kMutexKey_OSD);
    }
}

bool SavestateToast_IsActive(void)
{
    bool ret = false;
    if (Mutex_Take(kMutexKey_OSD) == kMutexResult_Ok) {
        const uint64_t now_us = (uint64_t)esp_timer_get_time();
        ret = _Ctx.active && now_us < _Ctx.hide_at_us;
        (void) Mutex_Give(kMutexKey_OSD);
    }
    return ret;
}
