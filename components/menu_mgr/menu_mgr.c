#include "menu_mgr.h"

#include "osd_shared.h"
#include "lvgl.h"
#include "esp_log.h"
#include "button.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct MenuMgrCtx
{
    TabID_t eCurTab;
    MenuTab_t* pMenus[kNumTabIDs];
} MenuMgrCtx_t;

const lv_point_t _MenuOrigin_px = {
    .x = 8,
    .y = 7,
};

static MenuMgrCtx_t _Ctx;
static const char* TAG = "MenuMgr";
static OSD_Result_t MenuMgr_OnButton(const Button_t Button, const ButtonState_t State, void *arg);
static OSD_Result_t MenuMgr_Draw(void* arg);
static OSD_Result_t MenuMgr_OnTransition(void *arg);
static void MenuMgr_NextTab(void);
static void MenuMgr_PrevTab(void);
static bool MenuMgr_IsTabActive(TabID_t eID);
static TabID_t MenuMgr_FindNextActiveTab(TabID_t eFrom, bool forward);

OSD_Result_t MenuMgr_Initialize(OSD_Widget_t* const pWidget, lv_obj_t *const pScreen)
{
    if (pScreen == NULL)
    {
        return kOSD_Result_Err_NullDataPtr;
    }

    if (sys_dnode_is_linked(&pWidget->Node))
    {
        return kOSD_Result_Err_MenuAlreadyInit;
    }

    pWidget->fnDraw = MenuMgr_Draw;
    pWidget->fnOnButton = MenuMgr_OnButton;
    pWidget->fnOnTransition = MenuMgr_OnTransition;

    _Ctx.eCurTab = kTabID_First;

    return kOSD_Result_Ok;
}

OSD_Result_t MenuMgr_AddTab(TabID_t eID, MenuTab_t *const pTab)
{
    if ((unsigned)eID >= kNumTabIDs)
    {
        return kOSD_Result_Err_InvalidTabID;
    }

    if ((pTab == NULL) || (pTab->pImageDesc == NULL))
    {
        return kOSD_Result_Err_NullDataPtr;
    }

    if ((_Ctx.pMenus[eID] != NULL) || (pTab->pImgObj != NULL))
    {
        return kOSD_Result_Err_MenuTabAlreadyInit;
    }

    _Ctx.pMenus[eID] = pTab;

    return kOSD_Result_Ok;
}

static OSD_Result_t MenuMgr_Draw(void* arg)
{
    if (arg == NULL)
    {
        return kOSD_Result_Err_NullDataPtr;
    }

    lv_obj_t *const pScreen = (lv_obj_t*)arg;

    const TabID_t eID = _Ctx.eCurTab;

    if ((unsigned)eID >= kNumTabIDs)
    {
        return kOSD_Result_Err_InvalidTabID;
    }

    MenuTab_t *const pTab = _Ctx.pMenus[eID];
    if ((pTab == NULL) || (pTab->pImageDesc == NULL))
    {
        return kOSD_Result_Err_NullDataPtr;
    }

    if (pTab->pImgObj == NULL)
    {
        pTab->pImgObj = lv_img_create(pScreen);
        lv_obj_align(pTab->pImgObj , LV_ALIGN_TOP_LEFT, _MenuOrigin_px.x, _MenuOrigin_px.y);
    }

    lv_img_set_src(pTab->pImgObj, pTab->pImageDesc);

    if (pTab->Widget.fnDraw != NULL)
    {
        Tab_DrawCtx_t Ctx = { pTab->Menu, pScreen, pTab->Accent };
        pTab->Widget.fnDraw(&Ctx);
    }

    return kOSD_Result_Ok;
}

static OSD_Result_t MenuMgr_OnButton(const Button_t Button, const ButtonState_t State, void *arg)
{
    (void)arg;

    const TabID_t eID = _Ctx.eCurTab;

    if ((unsigned)eID >= kNumTabIDs)
    {
        return kOSD_Result_Err_InvalidTabID;
    }

    MenuTab_t *const pTab = _Ctx.pMenus[eID];
    if (pTab == NULL)
    {
        ESP_LOGE(TAG, "no tab in %s", __func__ );
        return kOSD_Result_Err_NullDataPtr;
    }

    switch (Button)
    {
        // Fall-through
        case kButton_B:
        case kButton_A:
        case kButton_Down:
        case kButton_Up:
            if (pTab->Widget.fnOnButton != NULL)
            {
                const OSD_Result_t eResult = pTab->Widget.fnOnButton(Button, State, pTab->Menu);

                if (eResult != kOSD_Result_Ok)
                {
                    ESP_LOGE(TAG, "%s OnButton call failed with %d", pTab->Widget.Name, eResult);
                }
            }
            break;
        case kButton_Left:
            if (State == kButtonState_Pressed)
            {
                MenuMgr_PrevTab();
            }
            break;
        case kButton_Right:
            if (State == kButtonState_Pressed)
            {
                MenuMgr_NextTab();
            }
            break;
        default:
            break;
    }

    return kOSD_Result_Ok;
}

static void MenuMgr_NextTab(void)
{
    const TabID_t eNextID = MenuMgr_FindNextActiveTab(_Ctx.eCurTab, true);

    if (eNextID == _Ctx.eCurTab)
    {
        return;
    }

    // Clean up the old tab data
    (void)MenuMgr_OnTransition(NULL);
    _Ctx.eCurTab = eNextID;
}

static void MenuMgr_PrevTab(void)
{
    const TabID_t ePrevID = MenuMgr_FindNextActiveTab(_Ctx.eCurTab, false);

    if (ePrevID == _Ctx.eCurTab)
    {
        return;
    }

    // Clean up the old tab data
    (void)MenuMgr_OnTransition(NULL);
    _Ctx.eCurTab = ePrevID;
}

static bool MenuMgr_IsTabActive(TabID_t eID)
{
    if ((unsigned)eID >= kNumTabIDs)
    {
        return false;
    }

    MenuTab_t *const pTab = _Ctx.pMenus[eID];
    if (pTab == NULL)
    {
        return false;
    }

    if (pTab->fnIsActive == NULL)
    {
        return true;
    }

    return pTab->fnIsActive();
}

static TabID_t MenuMgr_FindNextActiveTab(TabID_t eFrom, bool forward)
{
    TabID_t eCandidate = eFrom;

    for (size_t i = 0; i < kNumTabIDs; i++)
    {
        if (forward)
        {
            eCandidate = (eCandidate + 1 >= kNumTabIDs) ? kTabID_First : eCandidate + 1;
        }
        else
        {
            eCandidate = ((signed)eCandidate - 1 < 0) ? kTabID_Last : eCandidate - 1;
        }

        if (MenuMgr_IsTabActive(eCandidate))
        {
            return eCandidate;
        }
    }

    return eFrom;
}

static OSD_Result_t MenuMgr_OnTransition(void *arg)
{
    (void) arg;
    OSD_Result_t eResult = kOSD_Result_Ok;

    MenuTab_t *const pTab = _Ctx.pMenus[_Ctx.eCurTab];

    if (pTab->Widget.fnOnTransition != NULL)
    {
        eResult = pTab->Widget.fnOnTransition(pTab->Menu);
        if (eResult != kOSD_Result_Ok)
        {
            ESP_LOGE(TAG, "%s OnTransition failed with %d", pTab->Widget.Name, eResult);
        }
    }

    if (pTab->pImgObj != NULL)
    {
        lv_obj_del(pTab->pImgObj);
        pTab->pImgObj = NULL;
    }

    return eResult;
}
