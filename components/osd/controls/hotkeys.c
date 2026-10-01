#include "hotkeys.h"

#include "line.h"
#include "esp_log.h"
#include "osd_shared.h"
#include "style.h"
#include "lvgl.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum HKID {
    kHKID_Brightness,
    kHKID_ColorTemp,
    kHKID_ResetEmuCore,
    kNumHotKeys
} HKID_t;

typedef enum HKFlashMode {
    kHKFlash_DPadLR,
    kHKFlash_DPadUD,
    kHKFlash_ResetButtons,
} HKFlashMode_t;

typedef struct HKEntry {
    const char* pName;
    const char* pDescr;
    HKFlashMode_t eFlashMode;
} HKEntry_t;

static HKEntry_t gHotKeys[kNumHotKeys] = {
    [kHKID_Brightness] = {
        .pName = "BRIGHTNESS",
        .pDescr = "MENU + L/R ON DPAD",
        .eFlashMode = kHKFlash_DPadLR,
    },
    [kHKID_ColorTemp] = {
        .pName = "PALETTE / COLOR TEMP",
        .pDescr = "MENU + U/D ON DPAD",
        .eFlashMode = kHKFlash_DPadUD,
    },
    [kHKID_ResetEmuCore] = {
        .pName = "RESET EMULATION",
        .pDescr = "MENU + B + A + START + SELECT",
        .eFlashMode = kHKFlash_ResetButtons,
    }
};

static const HKEntry_t gPaletteHotKeyEntry = {
    .pName = "CYCLE PALETTE (GB)",
    .pDescr = "MENU + U/D ON DPAD",
    .eFlashMode = kHKFlash_DPadUD,
};

static const HKEntry_t gColorTempHotKeyEntry = {
    .pName = "ADJUST COLOR TEMP (GBC)",
    .pDescr = "MENU + U/D ON DPAD",
    .eFlashMode = kHKFlash_DPadUD,
};

enum {
    kTitleX_px = 36,
    kTitleY_px = 52,
    kTitleW_px = 20,

    kTextX_px = 77,
    kTextY_px = 48,
    kTextGapY_px = 2,
    kMaxTextWidth_px = 63,

    // Draw/update cadence for this tab.
    kBlinkUpdateTick_ms = 30,
    // Time between visibility toggles (full on+off blink cycle is 2x this value).
    kBlinkTogglePeriod_ms = 240,
    kBlinkToggle_ticks = (kBlinkTogglePeriod_ms / kBlinkUpdateTick_ms),

    // Global offset for the reset-combo button masks (A/B/Start/Select).
    kResetBtnMaskOffsetX_px = 0,
    kResetBtnMaskOffsetY_px = 0,
    kResetABMaskOffsetX_px = 0,
    kResetABMaskOffsetY_px = 0,
    kResetStartMaskOffsetX_px = 1,
    kResetStartMaskOffsetY_px = -1,
    kResetSelectMaskOffsetX_px = 0,
    kResetSelectMaskOffsetY_px = 0,

    kColor_White = 0xFFFFFF,
};

enum {
    kFlashSegWidth1_px = 1,
    kFlashSegWidth3_px = 3,
};

// Menu button flash geometry.
enum {
    kMenuBtnFlashX_px = 64,
    kMenuBtnFlashYStart_px = 67,
    kMenuBtnFlashYEnd_px = 70,
};

// D-pad flash geometry.
enum {
    kDPadLeftFlashXStart_px = 33,
    kDPadLeftFlashXEnd_px = 36,
    kDPadLeftFlashY_px = 84,

    kDPadRightFlashXStart_px = 39,
    kDPadRightFlashXEnd_px = 42,
    kDPadRightFlashY_px = 84,

    kDPadUpFlashX_px = 37,
    kDPadUpFlashYStart_px = 80,
    kDPadUpFlashYEnd_px = 83,

    kDPadDownFlashX_px = 37,
    kDPadDownFlashYStart_px = 86,
    kDPadDownFlashYEnd_px = 89,
};

// A button mask rows.
enum {
    kABtnMaskRow0XStart_px = 58,
    kABtnMaskRow0Y_px = 81,
    kABtnMaskRow0XEnd_px = 60,
    kABtnMaskRow1XStart_px = 57,
    kABtnMaskRow1Y_px = 82,
    kABtnMaskRow1XEnd_px = 61,
    kABtnMaskRow2XStart_px = 57,
    kABtnMaskRow2Y_px = 83,
    kABtnMaskRow2XEnd_px = 61,
    kABtnMaskRow3XStart_px = 58,
    kABtnMaskRow3Y_px = 84,
    kABtnMaskRow3XEnd_px = 60,
};

// B button mask rows.
enum {
    kBBtnMaskRow0XStart_px = 52,
    kBBtnMaskRow0Y_px = 84,
    kBBtnMaskRow0XEnd_px = 54,
    kBBtnMaskRow1XStart_px = 51,
    kBBtnMaskRow1Y_px = 85,
    kBBtnMaskRow1XEnd_px = 55,
    kBBtnMaskRow2XStart_px = 51,
    kBBtnMaskRow2Y_px = 86,
    kBBtnMaskRow2XEnd_px = 55,
    kBBtnMaskRow3XStart_px = 52,
    kBBtnMaskRow3Y_px = 87,
    kBBtnMaskRow3XEnd_px = 54,
};

// Start button mask rows.
enum {
    kStartBtnMaskRow0XStart_px = 50,
    kStartBtnMaskRow0Y_px = 94,
    kStartBtnMaskRow0XEnd_px = 52,
    kStartBtnMaskRow1XStart_px = 48,
    kStartBtnMaskRow1Y_px = 95,
    kStartBtnMaskRow1XEnd_px = 51,
    kStartBtnMaskRow2XStart_px = 47,
    kStartBtnMaskRow2Y_px = 96,
    kStartBtnMaskRow2XEnd_px = 49,
};

// Select button mask rows.
enum {
    kSelectBtnMaskRow0XStart_px = 44,
    kSelectBtnMaskRow0Y_px = 93,
    kSelectBtnMaskRow0XEnd_px = 46,
    kSelectBtnMaskRow1XStart_px = 42,
    kSelectBtnMaskRow1Y_px = 94,
    kSelectBtnMaskRow1XEnd_px = 45,
    kSelectBtnMaskRow2XStart_px = 41,
    kSelectBtnMaskRow2Y_px = 95,
    kSelectBtnMaskRow2XEnd_px = 43,
};

static inline lv_point_t HK_ResetMaskPoint(int16_t x, int16_t y, int16_t offsetX, int16_t offsetY)
{
    return (lv_point_t){
        .x = x + kResetBtnMaskOffsetX_px + offsetX,
        .y = y + kResetBtnMaskOffsetY_px + offsetY,
    };
}

static inline void HK_SetResetMaskPoints(
    Line_t *const pSeg,
    int16_t x1,
    int16_t y1,
    int16_t x2,
    int16_t y2,
    int16_t offsetX,
    int16_t offsetY)
{
    pSeg->points[kLineConst_PointStartIdx] = HK_ResetMaskPoint(x1, y1, offsetX, offsetY);
    pSeg->points[kLineConst_PointEndIdx] = HK_ResetMaskPoint(x2, y2, offsetX, offsetY);
}

static inline void HK_SetResetABPoints(Line_t *const pSeg, int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    HK_SetResetMaskPoints(pSeg, x1, y1, x2, y2, kResetABMaskOffsetX_px, kResetABMaskOffsetY_px);
}

static inline void HK_SetResetStartPoints(Line_t *const pSeg, int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    HK_SetResetMaskPoints(pSeg, x1, y1, x2, y2, kResetStartMaskOffsetX_px, kResetStartMaskOffsetY_px);
}

static inline void HK_SetResetSelectPoints(Line_t *const pSeg, int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    HK_SetResetMaskPoints(pSeg, x1, y1, x2, y2, kResetSelectMaskOffsetX_px, kResetSelectMaskOffsetY_px);
}

typedef enum HKFlashSeg {
    kHKFlashSeg_Menu,
    kHKFlashSeg_Left,
    kHKFlashSeg_Right,
    kHKFlashSeg_Up,
    kHKFlashSeg_Down,
    kHKFlashSeg_A0,
    kHKFlashSeg_A1,
    kHKFlashSeg_A2,
    kHKFlashSeg_A3,
    kHKFlashSeg_B0,
    kHKFlashSeg_B1,
    kHKFlashSeg_B2,
    kHKFlashSeg_B3,
    kHKFlashSeg_Start0,
    kHKFlashSeg_Start1,
    kHKFlashSeg_Start2,
    kHKFlashSeg_Select0,
    kHKFlashSeg_Select1,
    kHKFlashSeg_Select2,
    kNumHKFlashSegs,
} HKFlashSegID_t;

typedef struct HKCtx {
    lv_obj_t* pMenuTitleTextObj;
    lv_obj_t* pNameTextObj;
    lv_obj_t* pDescrTextObj;
    bool FlashVisible;
    uint32_t BlinkCounter;
} HKCtx_t;

static HKCtx_t _Ctx = {
    .FlashVisible = true,
};

static void HotKeys_InitFlashSegments(void);
static void HotKeys_EnsureFlashSegmentsCreated(lv_obj_t *pScreen);
static void HotKeys_HideAllFlashSegments(void);
static void HotKeys_SetFlashSegmentVisible(HKFlashSegID_t eSegID, bool visible);
static void HotKeys_InitABMaskSegments(void);
static void HotKeys_InitBMaskSegments(void);
static void HotKeys_InitStartMaskSegments(void);
static void HotKeys_InitSelectMaskSegments(void);
static Line_t gFlashSegs[kNumHKFlashSegs] = {
    // Menu button (image-space x=56, y=60..62, + menu origin offset {+8,+7})
    [kHKFlashSeg_Menu] =   { .points = { {kMenuBtnFlashX_px, kMenuBtnFlashYStart_px}, {kMenuBtnFlashX_px, kMenuBtnFlashYEnd_px} }, .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_Left] =  { .points = { {kDPadLeftFlashXStart_px, kDPadLeftFlashY_px}, {kDPadLeftFlashXEnd_px, kDPadLeftFlashY_px} }, .width = kFlashSegWidth3_px, .color = kColor_White },
    [kHKFlashSeg_Right] = { .points = { {kDPadRightFlashXStart_px, kDPadRightFlashY_px}, {kDPadRightFlashXEnd_px, kDPadRightFlashY_px} }, .width = kFlashSegWidth3_px, .color = kColor_White },
    [kHKFlashSeg_Up] =    { .points = { {kDPadUpFlashX_px, kDPadUpFlashYStart_px}, {kDPadUpFlashX_px, kDPadUpFlashYEnd_px} }, .width = kFlashSegWidth3_px, .color = kColor_White },
    [kHKFlashSeg_Down] =  { .points = { {kDPadDownFlashX_px, kDPadDownFlashYStart_px}, {kDPadDownFlashX_px, kDPadDownFlashYEnd_px} }, .width = kFlashSegWidth3_px, .color = kColor_White },

    // A button (menu_controls image-space + menu origin offset {+8,+7})
    [kHKFlashSeg_A0] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_A1] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_A2] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_A3] = { .width = kFlashSegWidth1_px, .color = kColor_White },

    // B button (menu_controls image-space + menu origin offset {+8,+7})
    [kHKFlashSeg_B0] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_B1] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_B2] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_B3] = { .width = kFlashSegWidth1_px, .color = kColor_White },

    // Start button (image-space adjusted to keep gap from select, then +{8,+7})
    [kHKFlashSeg_Start0] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_Start1] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_Start2] = { .width = kFlashSegWidth1_px, .color = kColor_White },

    // Select button (menu_controls image-space + menu origin offset {+8,+7})
    [kHKFlashSeg_Select0] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_Select1] = { .width = kFlashSegWidth1_px, .color = kColor_White },
    [kHKFlashSeg_Select2] = { .width = kFlashSegWidth1_px, .color = kColor_White },
};

static const char* TAG = "HK";
static void HotKeys_DeleteFlashSegments(void);
static void HotKeys_DrawFlashSegments(lv_obj_t *pScreen, HKFlashMode_t eFlashMode);
static OSD_Result_t HotKeys_DrawForID(void* arg, HKID_t eID);
static const HKEntry_t* HotKeys_GetEntry(HKID_t eID);

OSD_Result_t HotKeys_DrawBrightness(void* arg)
{
    return HotKeys_DrawForID(arg, kHKID_Brightness);
}

OSD_Result_t HotKeys_DrawColorTemp(void* arg)
{
    return HotKeys_DrawForID(arg, kHKID_ColorTemp);
}

OSD_Result_t HotKeys_DrawResetEmulation(void* arg)
{
    return HotKeys_DrawForID(arg, kHKID_ResetEmuCore);
}

static OSD_Result_t HotKeys_DrawForID(void* arg, HKID_t eID)
{
    if (arg == NULL)
    {
        return kOSD_Result_Err_NullDataPtr;
    }

    lv_obj_t *const pScreen = (lv_obj_t *const) arg;
    HotKeys_InitFlashSegments();

    if (_Ctx.pMenuTitleTextObj == NULL)
    {
        _Ctx.pMenuTitleTextObj = lv_label_create(pScreen);
        lv_obj_align(_Ctx.pMenuTitleTextObj, LV_ALIGN_TOP_LEFT, kTitleX_px, kTitleY_px);
        lv_label_set_long_mode(_Ctx.pMenuTitleTextObj, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(_Ctx.pMenuTitleTextObj, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(_Ctx.pMenuTitleTextObj, kTitleW_px);
        lv_obj_add_style(_Ctx.pMenuTitleTextObj, OSD_GetStyleTextBlack(), 0);
        lv_label_set_text_static(_Ctx.pMenuTitleTextObj, "HOT KEYS");
    }

    if (_Ctx.pNameTextObj == NULL)
    {
        _Ctx.pNameTextObj = lv_label_create(pScreen);
        lv_label_set_long_mode(_Ctx.pNameTextObj, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(_Ctx.pNameTextObj, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_width(_Ctx.pNameTextObj, kMaxTextWidth_px);
        lv_obj_add_style(_Ctx.pNameTextObj, OSD_GetStyleTextWhite(), 0);
    }

    if (_Ctx.pDescrTextObj == NULL)
    {
        _Ctx.pDescrTextObj = lv_label_create(pScreen);
        lv_label_set_long_mode(_Ctx.pDescrTextObj, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(_Ctx.pDescrTextObj, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_width(_Ctx.pDescrTextObj, kMaxTextWidth_px);
        lv_obj_add_style(_Ctx.pDescrTextObj, OSD_GetStyleTextGrey(), 0);
    }

    const HKEntry_t *const pHK = HotKeys_GetEntry(eID);
    lv_label_set_text_static(_Ctx.pNameTextObj, pHK->pName);
    lv_obj_set_pos(_Ctx.pNameTextObj, kTextX_px, kTextY_px);
    lv_obj_update_layout(_Ctx.pNameTextObj);

    const int32_t name_h = (int32_t)lv_obj_get_height(_Ctx.pNameTextObj);
    lv_label_set_text_static(_Ctx.pDescrTextObj, pHK->pDescr);
    lv_obj_set_pos(_Ctx.pDescrTextObj, kTextX_px, kTextY_px + name_h + kTextGapY_px);

    if (++_Ctx.BlinkCounter >= kBlinkToggle_ticks)
    {
        _Ctx.BlinkCounter = 0;
        _Ctx.FlashVisible = !_Ctx.FlashVisible;
    }

    if (_Ctx.FlashVisible)
    {
        HotKeys_DrawFlashSegments(pScreen, pHK->eFlashMode);
    }
    else
    {
        HotKeys_HideAllFlashSegments();
    }

    return kOSD_Result_Ok;
}

static const HKEntry_t* HotKeys_GetEntry(HKID_t eID)
{
    if (eID == kHKID_ColorTemp)
    {
        // Match palette tab context: GB mode => palette control, GBC mode => color temp control.
        return Style_IsGBCMode() ? &gColorTempHotKeyEntry : &gPaletteHotKeyEntry;
    }

    return &gHotKeys[eID];
}

static void HotKeys_InitFlashSegments(void)
{
    static bool sInit = false;

    if (sInit)
    {
        return;
    }

    HotKeys_InitABMaskSegments();
    HotKeys_InitBMaskSegments();
    HotKeys_InitStartMaskSegments();
    HotKeys_InitSelectMaskSegments();

    sInit = true;
}

static void HotKeys_InitABMaskSegments(void)
{
    HK_SetResetABPoints(&gFlashSegs[kHKFlashSeg_A0], kABtnMaskRow0XStart_px, kABtnMaskRow0Y_px, kABtnMaskRow0XEnd_px, kABtnMaskRow0Y_px);
    HK_SetResetABPoints(&gFlashSegs[kHKFlashSeg_A1], kABtnMaskRow1XStart_px, kABtnMaskRow1Y_px, kABtnMaskRow1XEnd_px, kABtnMaskRow1Y_px);
    HK_SetResetABPoints(&gFlashSegs[kHKFlashSeg_A2], kABtnMaskRow2XStart_px, kABtnMaskRow2Y_px, kABtnMaskRow2XEnd_px, kABtnMaskRow2Y_px);
    HK_SetResetABPoints(&gFlashSegs[kHKFlashSeg_A3], kABtnMaskRow3XStart_px, kABtnMaskRow3Y_px, kABtnMaskRow3XEnd_px, kABtnMaskRow3Y_px);
}

static void HotKeys_InitBMaskSegments(void)
{
    HK_SetResetABPoints(&gFlashSegs[kHKFlashSeg_B0], kBBtnMaskRow0XStart_px, kBBtnMaskRow0Y_px, kBBtnMaskRow0XEnd_px, kBBtnMaskRow0Y_px);
    HK_SetResetABPoints(&gFlashSegs[kHKFlashSeg_B1], kBBtnMaskRow1XStart_px, kBBtnMaskRow1Y_px, kBBtnMaskRow1XEnd_px, kBBtnMaskRow1Y_px);
    HK_SetResetABPoints(&gFlashSegs[kHKFlashSeg_B2], kBBtnMaskRow2XStart_px, kBBtnMaskRow2Y_px, kBBtnMaskRow2XEnd_px, kBBtnMaskRow2Y_px);
    HK_SetResetABPoints(&gFlashSegs[kHKFlashSeg_B3], kBBtnMaskRow3XStart_px, kBBtnMaskRow3Y_px, kBBtnMaskRow3XEnd_px, kBBtnMaskRow3Y_px);
}

static void HotKeys_InitStartMaskSegments(void)
{
    HK_SetResetStartPoints(&gFlashSegs[kHKFlashSeg_Start0], kStartBtnMaskRow0XStart_px, kStartBtnMaskRow0Y_px, kStartBtnMaskRow0XEnd_px, kStartBtnMaskRow0Y_px);
    HK_SetResetStartPoints(&gFlashSegs[kHKFlashSeg_Start1], kStartBtnMaskRow1XStart_px, kStartBtnMaskRow1Y_px, kStartBtnMaskRow1XEnd_px, kStartBtnMaskRow1Y_px);
    HK_SetResetStartPoints(&gFlashSegs[kHKFlashSeg_Start2], kStartBtnMaskRow2XStart_px, kStartBtnMaskRow2Y_px, kStartBtnMaskRow2XEnd_px, kStartBtnMaskRow2Y_px);
}

static void HotKeys_InitSelectMaskSegments(void)
{
    HK_SetResetSelectPoints(&gFlashSegs[kHKFlashSeg_Select0], kSelectBtnMaskRow0XStart_px, kSelectBtnMaskRow0Y_px, kSelectBtnMaskRow0XEnd_px, kSelectBtnMaskRow0Y_px);
    HK_SetResetSelectPoints(&gFlashSegs[kHKFlashSeg_Select1], kSelectBtnMaskRow1XStart_px, kSelectBtnMaskRow1Y_px, kSelectBtnMaskRow1XEnd_px, kSelectBtnMaskRow1Y_px);
    HK_SetResetSelectPoints(&gFlashSegs[kHKFlashSeg_Select2], kSelectBtnMaskRow2XStart_px, kSelectBtnMaskRow2Y_px, kSelectBtnMaskRow2XEnd_px, kSelectBtnMaskRow2Y_px);
}

static void HotKeys_EnsureFlashSegmentsCreated(lv_obj_t *pScreen)
{
    if (pScreen == NULL)
    {
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(gFlashSegs); i++)
    {
        if (gFlashSegs[i].pObj == NULL)
        {
            gFlashSegs[i].pObj = lv_line_create(pScreen);
            if (gFlashSegs[i].pObj == NULL)
            {
                ESP_LOGE(TAG, "Failed to create flash segment %u", (unsigned)i);
                continue;
            }
            lv_obj_remove_style_all(gFlashSegs[i].pObj);
            lv_obj_set_style_line_width(gFlashSegs[i].pObj, gFlashSegs[i].width, LV_PART_MAIN);
            lv_obj_set_style_line_color(gFlashSegs[i].pObj, lv_color_hex(gFlashSegs[i].color), LV_PART_MAIN);
            lv_line_set_points(gFlashSegs[i].pObj, gFlashSegs[i].points, kLineConst_NumPoints);
            lv_obj_set_style_line_rounded(gFlashSegs[i].pObj, false, LV_PART_MAIN);
            lv_obj_add_flag(gFlashSegs[i].pObj, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void HotKeys_HideAllFlashSegments(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(gFlashSegs); i++)
    {
        HotKeys_SetFlashSegmentVisible((HKFlashSegID_t)i, false);
    }
}

static void HotKeys_SetFlashSegmentVisible(HKFlashSegID_t eSegID, bool visible)
{
    if ((unsigned)eSegID >= ARRAY_SIZE(gFlashSegs))
    {
        return;
    }

    lv_obj_t *const pObj = gFlashSegs[eSegID].pObj;
    if (pObj == NULL)
    {
        return;
    }

    if (visible)
    {
        lv_obj_clear_flag(pObj, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(pObj, LV_OBJ_FLAG_HIDDEN);
    }
}

OSD_Result_t HotKeys_OnButton(const Button_t Button, const ButtonState_t State, void *arg)
{
    (void)Button;
    (void)State;
    (void)arg;

    return kOSD_Result_Ok;
}

OSD_Result_t HotKeys_OnTransition(void* arg)
{
    (void)arg;

    if (_Ctx.pNameTextObj != NULL)
    {
        lv_obj_del(_Ctx.pNameTextObj);
        _Ctx.pNameTextObj = NULL;
    }

    if (_Ctx.pDescrTextObj != NULL)
    {
        lv_obj_del(_Ctx.pDescrTextObj);
        _Ctx.pDescrTextObj = NULL;
    }

    if (_Ctx.pMenuTitleTextObj != NULL)
    {
        lv_obj_del(_Ctx.pMenuTitleTextObj);
        _Ctx.pMenuTitleTextObj = NULL;
    }

    _Ctx.BlinkCounter = 0;
    _Ctx.FlashVisible = true;
    HotKeys_DeleteFlashSegments();

    return kOSD_Result_Ok;
}

static void HotKeys_DeleteFlashSegments(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(gFlashSegs); i++)
    {
        if (gFlashSegs[i].pObj != NULL)
        {
            lv_obj_del(gFlashSegs[i].pObj);
            gFlashSegs[i].pObj = NULL;
        }
    }
}

static void HotKeys_DrawFlashSegments(lv_obj_t *pScreen, HKFlashMode_t eFlashMode)
{
    bool drawSeg[kNumHKFlashSegs] = {0};
    HotKeys_EnsureFlashSegmentsCreated(pScreen);

    drawSeg[kHKFlashSeg_Menu] = true;

    switch (eFlashMode)
    {
        case kHKFlash_DPadLR:
            drawSeg[kHKFlashSeg_Left] = true;
            drawSeg[kHKFlashSeg_Right] = true;
            break;
        case kHKFlash_DPadUD:
            drawSeg[kHKFlashSeg_Up] = true;
            drawSeg[kHKFlashSeg_Down] = true;
            break;
        case kHKFlash_ResetButtons:
            drawSeg[kHKFlashSeg_A0] = true;
            drawSeg[kHKFlashSeg_A1] = true;
            drawSeg[kHKFlashSeg_A2] = true;
            drawSeg[kHKFlashSeg_A3] = true;
            drawSeg[kHKFlashSeg_B0] = true;
            drawSeg[kHKFlashSeg_B1] = true;
            drawSeg[kHKFlashSeg_B2] = true;
            drawSeg[kHKFlashSeg_B3] = true;
            drawSeg[kHKFlashSeg_Start0] = true;
            drawSeg[kHKFlashSeg_Start1] = true;
            drawSeg[kHKFlashSeg_Start2] = true;
            drawSeg[kHKFlashSeg_Select0] = true;
            drawSeg[kHKFlashSeg_Select1] = true;
            drawSeg[kHKFlashSeg_Select2] = true;
            break;
        default:
            break;
    }

    for (size_t i = 0; i < ARRAY_SIZE(gFlashSegs); i++)
    {
        HotKeys_SetFlashSegmentVisible((HKFlashSegID_t)i, drawSeg[i]);
    }
}
