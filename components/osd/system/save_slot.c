#include "save_slot.h"

#include "osd_shared.h"
#include "lvgl.h"
#include "esp_log.h"
#include "mutex.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

enum {
    kHeaderOffsetX_px = 81,
    kHeaderOffsetY_px = 38,
    kValueOffsetX_px  = 81,
    kValueOffsetY_px  = 60,
    kSaveSlotTextLen  = 24,
};

static const char *TAG = "save_slot";

typedef struct SaveSlotCtx {
    lv_obj_t *pHeader;
    lv_obj_t *pValue;
    char value_text[kSaveSlotTextLen];
    bool dirty;
} SaveSlotCtx_t;

static SaveSlotCtx_t _Ctx = {
    .pHeader    = NULL,
    .pValue     = NULL,
    .value_text = "EMPTY",
    .dirty      = true,
};

OSD_Result_t SaveSlot_OnTransition(void *arg)
{
    (void)arg;
    /* Mark dirty so next Draw refreshes the label.  No storage I/O here. */
    if (Mutex_Take(kMutexKey_OSD) == kMutexResult_Ok) {
        _Ctx.dirty = true;
        (void) Mutex_Give(kMutexKey_OSD);
    }
    return kOSD_Result_Ok;
}

OSD_Result_t SaveSlot_Draw(void *arg)
{
    if (arg == NULL) {
        return kOSD_Result_Err_NullDataPtr;
    }
    lv_obj_t *const pScreen = (lv_obj_t *const)arg;

    if (_Ctx.pHeader == NULL) {
        _Ctx.pHeader = lv_label_create(pScreen);
        lv_label_set_text(_Ctx.pHeader, "SAVE SLOT");
        lv_obj_align(_Ctx.pHeader, LV_ALIGN_TOP_LEFT,
                     kHeaderOffsetX_px, kHeaderOffsetY_px);
        lv_obj_add_style(_Ctx.pHeader, OSD_GetStyleTextGrey(), 0);
    }

    if (_Ctx.pValue == NULL) {
        _Ctx.pValue = lv_label_create(pScreen);
        lv_obj_align(_Ctx.pValue, LV_ALIGN_TOP_LEFT,
                     kValueOffsetX_px, kValueOffsetY_px);
        lv_obj_add_style(_Ctx.pValue, OSD_GetStyleTextWhite(), 0);
    }

    bool needs_refresh = false;
    char text_local[kSaveSlotTextLen];
    if (Mutex_Take(kMutexKey_OSD) == kMutexResult_Ok) {
        needs_refresh = _Ctx.dirty;
        memcpy(text_local, _Ctx.value_text, sizeof(text_local));
        _Ctx.dirty = false;
        (void) Mutex_Give(kMutexKey_OSD);
    } else {
        memcpy(text_local, _Ctx.value_text, sizeof(text_local));
    }

    if (needs_refresh) {
        lv_label_set_text(_Ctx.pValue, text_local);
    }
    return kOSD_Result_Ok;
}

OSD_Result_t SaveSlot_Init(OSD_Widget_t *pWidget)
{
    if (pWidget == NULL) {
        return kOSD_Result_Err_NullDataPtr;
    }
    pWidget->Name = "SAVE SLOT";
    pWidget->fnDraw = SaveSlot_Draw;
    pWidget->fnOnTransition = SaveSlot_OnTransition;
    pWidget->fnOnButton = NULL;  /* read-only */
    ESP_LOGI(TAG, "Save Slot widget initialized");
    return kOSD_Result_Ok;
}

void SaveSlot_SetText(const char *text)
{
    if (text == NULL) {
        return;
    }
    if (Mutex_Take(kMutexKey_OSD) == kMutexResult_Ok) {
        strncpy(_Ctx.value_text, text, sizeof(_Ctx.value_text) - 1);
        _Ctx.value_text[sizeof(_Ctx.value_text) - 1] = '\0';
        _Ctx.dirty = true;
        (void) Mutex_Give(kMutexKey_OSD);
    }
}
