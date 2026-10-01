#include "gbc_color_temp.h"

#include "line.h"
#include "lvgl.h"
#include "settings.h"
#include "esp_log.h"

#include <string.h>

enum {
    // Numbered from neutral (0) upward through warmer steps.
    kGBCColorTemp_MinLevel = 0,
    kGBCColorTemp_MaxLevel = 5,

    kBarOffsetX_px = 85,
    kBarOffsetY_px = 60,
    kBarWidth_px   = 8,
    kBarGap_px     = 2,
    kBarHeight_px  = 26,
    kHintGap_px    = 1,
};

// Neutral at 0, then five warmer steps.
static const uint32_t kTempColors[kGBCColorTemp_MaxLevel + 1] = {
    0xCCCCCC, // neutral
    0xF7D6C2, // warm 1
    0xF2C2A3, // warm 2
    0xEEAE85, // warm 3
    0xE99966, // warm 4
    0xE5854A, // warm 5 (hottest)
};

static const char* kTempLabels[kGBCColorTemp_MaxLevel + 1] = {
    "NEUTRAL",
    "WARM 1",
    "WARM 2",
    "WARM 3",
    "WARM 4",
    "WARM 5",
};

static const char *TAG = "GBCColorTemp";

typedef struct GBCColorTempCtx {
    lv_obj_t* pLabel;
    lv_obj_t* pLabelA;
    lv_obj_t* pLabelB;
    Line_t Bars[kGBCColorTemp_MaxLevel + 1];
    GBCColorTempLevel_t Level;
    GBCColorTempLevel_t PrevDrawnLevel;
    fnOnUpdateCb_t fnOnUpdateCb;
    bool HasPrevDrawnLevel;
} GBCColorTempCtx_t;

static GBCColorTempCtx_t _Ctx = {
    // Default until persist_storage_init() applies kSettingKey_GBCColorTemp.
    .Level = kGBCColorTemp_MinLevel,
};

static GBCColorTempLevel_t NormalizeStoredLevel(uint32_t raw)
{
    return (GBCColorTempLevel_t)MIN(raw, kGBCColorTemp_MaxLevel);
}

static void SaveToSettings(GBCColorTempLevel_t level);
static void DrawBars(lv_obj_t* pScreen, bool recalc);

OSD_Result_t GBCColorTemp_Draw(void* arg)
{
    if (arg == NULL)
    {
        return kOSD_Result_Err_NullDataPtr;
    }

    lv_obj_t* const pScreen = (lv_obj_t *const)arg;

    if (_Ctx.pLabel == NULL)
    {
        _Ctx.pLabel = lv_label_create(pScreen);
        lv_obj_add_style(_Ctx.pLabel, OSD_GetStyleTextWhite(), 0);
    }

    if (_Ctx.pLabelB == NULL)
    {
        _Ctx.pLabelB = lv_label_create(pScreen);
        lv_obj_add_style(_Ctx.pLabelB, OSD_GetStyleTextWhite(), 0);
        lv_label_set_text_static(_Ctx.pLabelB, "B");
    }

    if (_Ctx.pLabelA == NULL)
    {
        _Ctx.pLabelA = lv_label_create(pScreen);
        lv_obj_add_style(_Ctx.pLabelA, OSD_GetStyleTextWhite(), 0);
        lv_label_set_text_static(_Ctx.pLabelA, "A");
    }

    lv_label_set_text_static(_Ctx.pLabel, kTempLabels[_Ctx.Level]);

    const int32_t barHalfWidth_px = kBarWidth_px / 2;
    const int32_t barsLeftEdgeX_px = kBarOffsetX_px - barHalfWidth_px;
    const int32_t barsRightEdgeX_px = kBarOffsetX_px +
        (kGBCColorTemp_MaxLevel * (kBarWidth_px + kBarGap_px)) +
        barHalfWidth_px;
    const int32_t barsCenterX_px = (barsLeftEdgeX_px + barsRightEdgeX_px) / 2;
    const int32_t barsCenterY_px = kBarOffsetY_px + (kBarHeight_px / 2);
    lv_obj_update_layout(_Ctx.pLabel);
    lv_obj_update_layout(_Ctx.pLabelB);
    lv_obj_update_layout(_Ctx.pLabelA);
    lv_obj_set_pos(
        _Ctx.pLabel,
        barsCenterX_px - ((int32_t)lv_obj_get_width(_Ctx.pLabel) / 2),
        kBarOffsetY_px - 10
    );

    lv_obj_set_pos(
        _Ctx.pLabelB,
        barsLeftEdgeX_px - kHintGap_px - (int32_t)lv_obj_get_width(_Ctx.pLabelB),
        barsCenterY_px - ((int32_t)lv_obj_get_height(_Ctx.pLabelB) / 2)
    );
    lv_obj_set_pos(
        _Ctx.pLabelA,
        barsRightEdgeX_px + kHintGap_px,
        barsCenterY_px - ((int32_t)lv_obj_get_height(_Ctx.pLabelA) / 2)
    );

    const bool recalc = (!_Ctx.HasPrevDrawnLevel || (_Ctx.PrevDrawnLevel != _Ctx.Level));
    _Ctx.PrevDrawnLevel = _Ctx.Level;
    _Ctx.HasPrevDrawnLevel = true;

    DrawBars(pScreen, recalc);
    return kOSD_Result_Ok;
}

OSD_Result_t GBCColorTemp_OnButton(const Button_t Button, const ButtonState_t State, void* arg)
{
    (void)arg;

    if (State != kButtonState_Pressed)
    {
        return kOSD_Result_Ok;
    }

    switch (Button)
    {
        case kButton_A:
            if (_Ctx.Level < kGBCColorTemp_MaxLevel)
            {
                _Ctx.Level++;
                SaveToSettings(_Ctx.Level);
                if (_Ctx.fnOnUpdateCb != NULL)
                {
                    _Ctx.fnOnUpdateCb();
                }
            }
            break;
        case kButton_B:
            if (_Ctx.Level > kGBCColorTemp_MinLevel)
            {
                _Ctx.Level--;
                SaveToSettings(_Ctx.Level);
                if (_Ctx.fnOnUpdateCb != NULL)
                {
                    _Ctx.fnOnUpdateCb();
                }
            }
            break;
        default:
            break;
    }

    return kOSD_Result_Ok;
}

OSD_Result_t GBCColorTemp_OnTransition(void* arg)
{
    (void)arg;

    if (_Ctx.pLabel != NULL)
    {
        lv_obj_del(_Ctx.pLabel);
        _Ctx.pLabel = NULL;
    }

    if (_Ctx.pLabelA != NULL)
    {
        lv_obj_del(_Ctx.pLabelA);
        _Ctx.pLabelA = NULL;
    }

    if (_Ctx.pLabelB != NULL)
    {
        lv_obj_del(_Ctx.pLabelB);
        _Ctx.pLabelB = NULL;
    }

    for (size_t i = 0; i < ARRAY_SIZE(_Ctx.Bars); i++)
    {
        if (_Ctx.Bars[i].pObj != NULL)
        {
            lv_obj_del(_Ctx.Bars[i].pObj);
            _Ctx.Bars[i].pObj = NULL;
        }
    }

    _Ctx.HasPrevDrawnLevel = false;

    return kOSD_Result_Ok;
}

GBCColorTempLevel_t GBCColorTemp_GetLevel(void)
{
    return _Ctx.Level;
}

void GBCColorTemp_RegisterOnUpdateCb(fnOnUpdateCb_t fnOnUpdate)
{
    _Ctx.fnOnUpdateCb = fnOnUpdate;
}

void GBCColorTemp_Update(GBCColorTempLevel_t level)
{
    const GBCColorTempLevel_t normalized = NormalizeStoredLevel(level);

    if (_Ctx.Level == normalized)
    {
        return;
    }

    _Ctx.Level = normalized;
    SaveToSettings(_Ctx.Level);
    // This path is fed by FPGA status messages. Persist it locally, but do not
    // trigger the update callback here to avoid immediately echoing the same
    // value back to the FPGA over UART.
}

OSD_Result_t GBCColorTemp_ApplySetting(const SettingValue_t* pValue)
{
    if (pValue == NULL)
    {
        return kOSD_Result_Err_NullDataPtr;
    }

    if (pValue->eType != kSettingDataType_U8)
    {
        return kOSD_Result_Err_UnexpectedSettingDataType;
    }

    _Ctx.Level = NormalizeStoredLevel(pValue->U8);
    return kOSD_Result_Ok;
}

static void DrawBars(lv_obj_t* pScreen, bool recalc)
{
    for (size_t i = 0; i < ARRAY_SIZE(_Ctx.Bars); i++)
    {
        if (recalc && (_Ctx.Bars[i].pObj != NULL))
        {
            lv_obj_del(_Ctx.Bars[i].pObj);
            _Ctx.Bars[i].pObj = NULL;
        }

        if (_Ctx.Bars[i].pObj == NULL)
        {
            _Ctx.Bars[i].pObj = lv_line_create(pScreen);
            if (_Ctx.Bars[i].pObj == NULL)
            {
                ESP_LOGE(TAG, "Failed to create color temp bar object %u", (unsigned)i);
                return;
            }
        }

        const lv_point_t pts[kLineConst_NumPoints] = {
            { .x = kBarOffsetX_px + (int32_t)i * (kBarWidth_px + kBarGap_px), .y = kBarOffsetY_px },
            { .x = kBarOffsetX_px + (int32_t)i * (kBarWidth_px + kBarGap_px), .y = kBarOffsetY_px + kBarHeight_px },
        };
        memcpy(_Ctx.Bars[i].points, pts, sizeof(pts));

        lv_line_set_points(_Ctx.Bars[i].pObj, _Ctx.Bars[i].points, kLineConst_NumPoints);
        lv_obj_set_style_line_width(_Ctx.Bars[i].pObj, kBarWidth_px, LV_PART_MAIN);
        lv_obj_set_style_line_color(_Ctx.Bars[i].pObj, lv_color_hex(kTempColors[i]), LV_PART_MAIN);

        if (i > _Ctx.Level)
        {
            lv_obj_set_style_opa(_Ctx.Bars[i].pObj, LV_OPA_40, LV_PART_MAIN);
        }
        else
        {
            lv_obj_set_style_opa(_Ctx.Bars[i].pObj, LV_OPA_COVER, LV_PART_MAIN);
        }
    }
}

static void SaveToSettings(GBCColorTempLevel_t level)
{
    (void)Settings_Update(kSettingKey_GBCColorTemp, level);
}
