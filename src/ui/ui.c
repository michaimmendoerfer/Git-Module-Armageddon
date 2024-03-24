// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.2.0
// Project name: SquareLine_Project_Jeepify_Large_PDC

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_ScrMenu
void ui_ScrMenu_screen_init(void);
void ui_event_ScrMenu(lv_event_t * e);
lv_obj_t * ui_ScrMenu;
void ui_event_BtnMenu1(lv_event_t * e);
lv_obj_t * ui_BtnMenu1;
void ui_event_BtnMenu2(lv_event_t * e);
lv_obj_t * ui_BtnMenu2;
void ui_event_BtnMenu3(lv_event_t * e);
lv_obj_t * ui_BtnMenu3;
void ui_event_BtnMenu4(lv_event_t * e);
lv_obj_t * ui_BtnMenu4;
void ui_event_BtnMenu5(lv_event_t * e);
lv_obj_t * ui_BtnMenu5;
void ui_event_BtnMenu6(lv_event_t * e);
lv_obj_t * ui_BtnMenu6;
lv_obj_t * ui_LblMenuJeepify;
lv_obj_t * ui_LblMenuType;


// SCREEN: ui_ScrSettings
void ui_ScrSettings_screen_init(void);
void ui_event_ScrSettings(lv_event_t * e);
lv_obj_t * ui_ScrSettings;
void ui_event_SwSettingsDemo(lv_event_t * e);
lv_obj_t * ui_SwSettingsDemo;
void ui_event_SwSettingsSleep(lv_event_t * e);
lv_obj_t * ui_SwSettingsSleep;
void ui_event_SwSettingsDebug(lv_event_t * e);
lv_obj_t * ui_SwSettingsDebug;
void ui_event_SwSettingsPair(lv_event_t * e);
lv_obj_t * ui_SwSettingsPair;
lv_obj_t * ui_Label1;
lv_obj_t * ui_Label2;
lv_obj_t * ui_Label3;
lv_obj_t * ui_Label4;
lv_obj_t * ui_Label5;
lv_obj_t * ui_LblSettingName;
lv_obj_t * ui_LblSettingType;
void ui_event_ImgSettingsReset(lv_event_t * e);
lv_obj_t * ui_ImgSettingsReset;
lv_obj_t * ui_Label9;


// SCREEN: ui_ScrGaugeSingle
void ui_ScrGaugeSingle_screen_init(void);
void ui_event_ScrGaugeSingle(lv_event_t * e);
lv_obj_t * ui_ScrGaugeSingle;
lv_obj_t * ui_ImgGaugeSingleGauge;
lv_obj_t * ui_LblGaugeSingleValueDescription;
lv_obj_t * ui_LblGaugeSingleValue;


// SCREEN: ui_ScrSwitch
void ui_ScrSwitch_screen_init(void);
void ui_event_ScrSwitch(lv_event_t * e);
lv_obj_t * ui_ScrSwitch;
lv_obj_t * ui_SwitchButton1;
lv_obj_t * ui_SwitchButton2;
lv_obj_t * ui_SwitchButton3;
lv_obj_t * ui_SwitchButton4;
lv_obj_t * ui_ContainerSwitchValues;
lv_obj_t * ui_LblGaugeSingleValueDescription1;
lv_obj_t * ui_LblGaugeSingleValueDescription2;
lv_obj_t * ui_LblGaugeSingleValueDescription3;
lv_obj_t * ui_LblGaugeSingleValueDescription4;
lv_obj_t * ui_LblGaugeSingleValue1;
lv_obj_t * ui_LblGaugeSingleValue2;
lv_obj_t * ui_LblGaugeSingleValue3;
lv_obj_t * ui_LblGaugeSingleValue4;
lv_obj_t * ui_LblSwitchPage;


// SCREEN: ui_ScrChangeName
void ui_ScrChangeName_screen_init(void);
void ui_event_ScrChangeName(lv_event_t * e);
lv_obj_t * ui_ScrChangeName;
void ui_event_Keyboard1(lv_event_t * e);
lv_obj_t * ui_Keyboard1;
lv_obj_t * ui_TxtAreaChangeName;


// SCREEN: ui_ScreenCalib
void ui_ScreenCalib_screen_init(void);
void ui_event_ScreenCalib(lv_event_t * e);
lv_obj_t * ui_ScreenCalib;
lv_obj_t * ui_Label7;
void ui_event_LblCalibVoltage(lv_event_t * e);
lv_obj_t * ui_LblCalibVoltage;
lv_obj_t * ui_Label6;
void ui_event_LblCalibCurrent(lv_event_t * e);
lv_obj_t * ui_LblCalibCurrent;


// SCREEN: ui_ScrNumKey
void ui_ScrNumKey_screen_init(void);
void ui_event_ScrNumKey(lv_event_t * e);
lv_obj_t * ui_ScrNumKey;
void ui_event_Keyboard2(lv_event_t * e);
lv_obj_t * ui_Keyboard2;
lv_obj_t * ui_Label10;
lv_obj_t * ui_TxtAreaScrNumKeyVoltage;
void ui_event____initial_actions0(lv_event_t * e);
lv_obj_t * ui____initial_actions0;
const lv_img_dsc_t * ui_imgset_25820555[1] = {&ui_img_551600910};
const lv_img_dsc_t * ui_imgset_toggle_vertical_on_[1] = {&ui_img_toggle_vertical_on_75_png};
const lv_img_dsc_t * ui_imgset_toggle_vertical_off_[1] = {&ui_img_toggle_vertical_off_75_png};

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_ScrMenu(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        Ui_Menu_Loaded(e);
    }
}
void ui_event_BtnMenu1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_ScrSettings, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrSettings_screen_init);
    }
}
void ui_event_BtnMenu2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_ScrGaugeSingle, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrGaugeSingle_screen_init);
    }
}
void ui_event_BtnMenu3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_ScrSwitch, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrSwitch_screen_init);
    }
}
void ui_event_BtnMenu4(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        Ui_MenuBtn4_Click(e);
    }
}
void ui_event_BtnMenu5(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_ScrChangeName, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrChangeName_screen_init);
    }
}
void ui_event_BtnMenu6(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_ScreenCalib, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScreenCalib_screen_init);
    }
}
void ui_event_ScrSettings(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_ScrMenu, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_ScrMenu_screen_init);
    }
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        Ui_Settings_Loaded(e);
    }
}
void ui_event_SwSettingsDemo(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        Ui_SettingBtn1_Click(e);
    }
}
void ui_event_SwSettingsSleep(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        Ui_SettingBtn2_Click(e);
    }
}
void ui_event_SwSettingsDebug(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        Ui_SettingBtn3_Click(e);
    }
}
void ui_event_SwSettingsPair(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        Ui_SettingBtn4_Click(e);
    }
}
void ui_event_ImgSettingsReset(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        Ui_Settings_Reset(e);
    }
}
void ui_event_ScrGaugeSingle(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_ScrMenu, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrMenu_screen_init);
    }
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        Ui_GaugeSingle_Loaded(e);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_GaugeSingle_Next(e);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_GaugeSingle_Prev(e);
    }
    if(event_code == LV_EVENT_SCREEN_UNLOAD_START) {
        Ui_GaugeSingle_Leave(e);
    }
}
void ui_event_ScrSwitch(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_ScrMenu, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_ScrMenu_screen_init);
    }
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        _ui_label_set_property(ui_comp_get_child(ui_SwitchButton2, UI_COMP_SWITCHBUTTON_LBLSWITCHID), _UI_LABEL_PROPERTY_TEXT,
                               "2");
        _ui_label_set_property(ui_comp_get_child(ui_SwitchButton3, UI_COMP_SWITCHBUTTON_LBLSWITCHID), _UI_LABEL_PROPERTY_TEXT,
                               "3");
        _ui_label_set_property(ui_comp_get_child(ui_SwitchButton4, UI_COMP_SWITCHBUTTON_LBLSWITCHID), _UI_LABEL_PROPERTY_TEXT,
                               "4");
        _ui_label_set_property(ui_comp_get_child(ui_SwitchButton1, UI_COMP_SWITCHBUTTON_LBLSWITCHID), _UI_LABEL_PROPERTY_TEXT,
                               "1");
        Ui_Switch_Loaded(e);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_Switch_Next(e);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        UI_Switch_Prev(e);
    }
    if(event_code == LV_EVENT_SCREEN_UNLOAD_START) {
        Ui_Switch_Leave(e);
    }
}
void ui_event_ScrChangeName(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_ScrMenu, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrMenu_screen_init);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_ChangeName_Next(e);
    }
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
        lv_indev_wait_release(lv_indev_get_act());
        Ui_ChangeName_Prev(e);
    }
    if(event_code == LV_EVENT_SCREEN_LOADED) {
        Ui_ChangeName_Loaded(e);
    }
}
void ui_event_Keyboard1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_READY) {
        Ui_ChangeName_Ready(e);
    }
}
void ui_event_ScreenCalib(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP) {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_ScrMenu, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrMenu_screen_init);
    }
    if(event_code == LV_EVENT_CLICKED) {
        ui_Calib_Loaded(e);
    }
    if(event_code == LV_EVENT_SCREEN_UNLOAD_START) {
        Ui_Calib_Leave(e);
    }
}
void ui_event_LblCalibVoltage(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_ScrNumKey, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrNumKey_screen_init);
    }
}
void ui_event_LblCalibCurrent(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        ui_Calib_Current_Click(e);
    }
}
void ui_event_ScrNumKey(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_ScrMenu, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrMenu_screen_init);
    }
}
void ui_event_Keyboard2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_READY) {
        Ui_NumKey_Ready(e);
    }
}
void ui_event____initial_actions0(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOAD_START) {
        Ui_Init_Custom(e);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    LV_EVENT_GET_COMP_CHILD = lv_event_register_id();

    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_ScrMenu_screen_init();
    ui_ScrSettings_screen_init();
    ui_ScrGaugeSingle_screen_init();
    ui_ScrSwitch_screen_init();
    ui_ScrChangeName_screen_init();
    ui_ScreenCalib_screen_init();
    ui_ScrNumKey_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_obj_add_event_cb(ui____initial_actions0, ui_event____initial_actions0, LV_EVENT_ALL, NULL);

    lv_disp_load_scr(ui____initial_actions0);
    lv_disp_load_scr(ui_ScrMenu);
}
