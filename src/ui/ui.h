// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.2.0
// Project name: SquareLine_Project_Jeepify_Large_PDC

#ifndef _SQUARELINE_PROJECT_JEEPIFY_LARGE_PDC_UI_H
#define _SQUARELINE_PROJECT_JEEPIFY_LARGE_PDC_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "components/ui_comp.h"
#include "components/ui_comp_hook.h"
#include "ui_events.h"
// SCREEN: ui_ScrMenu
void ui_ScrMenu_screen_init(void);
void ui_event_ScrMenu(lv_event_t * e);
extern lv_obj_t * ui_ScrMenu;
void ui_event_BtnMenu1(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu1;
void ui_event_BtnMenu2(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu2;
void ui_event_BtnMenu3(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu3;
void ui_event_BtnMenu4(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu4;
void ui_event_BtnMenu5(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu5;
void ui_event_BtnMenu6(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu6;
extern lv_obj_t * ui_Label8;
extern lv_obj_t * ui_LblMenuJeepify;
extern lv_obj_t * ui_LblMenuType;
// SCREEN: ui_ScrSettings
void ui_ScrSettings_screen_init(void);
void ui_event_ScrSettings(lv_event_t * e);
extern lv_obj_t * ui_ScrSettings;
void ui_event_SwSettingsDemo(lv_event_t * e);
extern lv_obj_t * ui_SwSettingsDemo;
void ui_event_SwSettingsSleep(lv_event_t * e);
extern lv_obj_t * ui_SwSettingsSleep;
void ui_event_SwSettingsDebug(lv_event_t * e);
extern lv_obj_t * ui_SwSettingsDebug;
void ui_event_SwSettingsPair(lv_event_t * e);
extern lv_obj_t * ui_SwSettingsPair;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_Label2;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_Label4;
extern lv_obj_t * ui_Label5;
extern lv_obj_t * ui_LblSettingName;
extern lv_obj_t * ui_LblSettingType;
void ui_event_ImgSettingsReset(lv_event_t * e);
extern lv_obj_t * ui_ImgSettingsReset;
extern lv_obj_t * ui_Label9;
// SCREEN: ui_ScrGaugeSingle
void ui_ScrGaugeSingle_screen_init(void);
void ui_event_ScrGaugeSingle(lv_event_t * e);
extern lv_obj_t * ui_ScrGaugeSingle;
extern lv_obj_t * ui_ImgGaugeSingleGauge;
extern lv_obj_t * ui_LblGaugeSingleValueDescription;
extern lv_obj_t * ui_LblGaugeSingleValue;
// SCREEN: ui_ScrSwitch
void ui_ScrSwitch_screen_init(void);
void ui_event_ScrSwitch(lv_event_t * e);
extern lv_obj_t * ui_ScrSwitch;
extern lv_obj_t * ui_SwitchButton1;
extern lv_obj_t * ui_SwitchButton2;
extern lv_obj_t * ui_SwitchButton3;
extern lv_obj_t * ui_SwitchButton4;
extern lv_obj_t * ui_ContainerSwitchValues;
extern lv_obj_t * ui_LblGaugeSingleValueDescription1;
extern lv_obj_t * ui_LblGaugeSingleValueDescription2;
extern lv_obj_t * ui_LblGaugeSingleValueDescription3;
extern lv_obj_t * ui_LblGaugeSingleValueDescription4;
extern lv_obj_t * ui_LblGaugeSingleValue1;
extern lv_obj_t * ui_LblGaugeSingleValue2;
extern lv_obj_t * ui_LblGaugeSingleValue3;
extern lv_obj_t * ui_LblGaugeSingleValue4;
extern lv_obj_t * ui_LblSwitchPage;
// SCREEN: ui_ScrChangeName
void ui_ScrChangeName_screen_init(void);
void ui_event_ScrChangeName(lv_event_t * e);
extern lv_obj_t * ui_ScrChangeName;
void ui_event_Keyboard1(lv_event_t * e);
extern lv_obj_t * ui_Keyboard1;
extern lv_obj_t * ui_TxtAreaChangeName;
// SCREEN: ui_ScreenCalib
void ui_ScreenCalib_screen_init(void);
void ui_event_ScreenCalib(lv_event_t * e);
extern lv_obj_t * ui_ScreenCalib;
extern lv_obj_t * ui_Label7;
void ui_event_LblCalibVoltage(lv_event_t * e);
extern lv_obj_t * ui_LblCalibVoltage;
extern lv_obj_t * ui_Label6;
void ui_event_LblCalibCurrent(lv_event_t * e);
extern lv_obj_t * ui_LblCalibCurrent;
// SCREEN: ui_ScrNumKey
void ui_ScrNumKey_screen_init(void);
void ui_event_ScrNumKey(lv_event_t * e);
extern lv_obj_t * ui_ScrNumKey;
void ui_event_Keyboard2(lv_event_t * e);
extern lv_obj_t * ui_Keyboard2;
extern lv_obj_t * ui_Label10;
extern lv_obj_t * ui_TxtAreaScrNumKeyVoltage;
// SCREEN: ui_ScreenGaugeMulti
void ui_ScreenGaugeMulti_screen_init(void);
void ui_event_ScreenGaugeMulti(lv_event_t * e);
extern lv_obj_t * ui_ScreenGaugeMulti;
extern lv_obj_t * ui_LblGaugeMultiPeer1;
extern lv_obj_t * ui_LblGaugeMultiPeer2;
extern lv_obj_t * ui_LblGaugeMultiPeer3;
extern lv_obj_t * ui_LblGaugeMultiPeer4;
extern lv_obj_t * ui_LblGaugeMultiPeer5;
extern lv_obj_t * ui_LblGaugeMultiValue1;
extern lv_obj_t * ui_LblGaugeMultiValue2;
extern lv_obj_t * ui_LblGaugeMultiValue3;
extern lv_obj_t * ui_LblGaugeMultiValue4;
extern lv_obj_t * ui_LblGaugeMultiValue5;
extern lv_obj_t * ui_Image1;
extern lv_obj_t * ui_Image2;
extern lv_obj_t * ui_Image4;
extern lv_obj_t * ui_Image5;
extern lv_obj_t * ui_Image6;
void ui_event____initial_actions0(lv_event_t * e);
extern lv_obj_t * ui____initial_actions0;

LV_IMG_DECLARE(ui_img_bg_grill_png);    // assets\bg_grill.png
LV_IMG_DECLARE(ui_img_icon_settings_png);    // assets\icon_settings.png
LV_IMG_DECLARE(ui_img_icon_gauge_png);    // assets\icon_gauge.png
LV_IMG_DECLARE(ui_img_icon_switch_png);    // assets\icon_switch.png
LV_IMG_DECLARE(ui_img_icon_pair_off_png);    // assets\icon_pair_off.png
LV_IMG_DECLARE(ui_img_icon_pair_png);    // assets\icon_pair.png
LV_IMG_DECLARE(ui_img_icon_keyboard_png);    // assets\icon_keyboard.png
LV_IMG_DECLARE(ui_img_1825836894);    // assets\bg_grill-small.png
LV_IMG_DECLARE(ui_img_551600910);    // assets\Btn-small-on-100.png
LV_IMG_DECLARE(ui_img_1404430020);    // assets\voltmeter-alt.png
LV_IMG_DECLARE(ui_img_toggle_vertical_off_75_png);    // assets\Toggle_vertical_off_75.png
LV_IMG_DECLARE(ui_img_toggle_vertical_on_75_png);    // assets\Toggle_vertical_on_75.png
LV_IMG_DECLARE(ui_img_1844714932);    // assets\voltmeter-alt-klein.png



void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
