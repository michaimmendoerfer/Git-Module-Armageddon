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
#include "ui_events.h"
// SCREEN: ui_ScrMenu
void ui_ScrMenu_screen_init(void);
extern lv_obj_t * ui_ScrMenu;
void ui_event_BtnMenu1(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu1;
void ui_event_BtnMenu2(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu2;
void ui_event_BtnMenu3(lv_event_t * e);
extern lv_obj_t * ui_BtnMenu3;
// SCREEN: ui_ScrSettings
void ui_ScrSettings_screen_init(void);
void ui_event_ScrSettings(lv_event_t * e);
extern lv_obj_t * ui_ScrSettings;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_Switch1;
extern lv_obj_t * ui_Switch2;
extern lv_obj_t * ui_Switch3;
extern lv_obj_t * ui_Switch4;
extern lv_obj_t * ui_Label2;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_Label4;
extern lv_obj_t * ui_Label5;
// SCREEN: ui_ScrGauge
void ui_ScrGauge_screen_init(void);
extern lv_obj_t * ui_ScrGauge;
extern lv_obj_t * ui_Image2;
// SCREEN: ui_ScrSwitch
void ui_ScrSwitch_screen_init(void);
extern lv_obj_t * ui_ScrSwitch;
extern lv_obj_t * ui_Image7;
extern lv_obj_t * ui_Image8;
extern lv_obj_t * ui_Image9;
extern lv_obj_t * ui_Image10;
extern lv_obj_t * ui____initial_actions0;

LV_IMG_DECLARE(ui_img_bg_grill_png);    // assets\bg_grill.png
LV_IMG_DECLARE(ui_img_icon_settings_png);    // assets\icon_settings.png
LV_IMG_DECLARE(ui_img_icon_gauge_png);    // assets\icon_gauge.png
LV_IMG_DECLARE(ui_img_icon_switch_png);    // assets\icon_switch.png
LV_IMG_DECLARE(ui_img_1825836894);    // assets\bg_grill-small.png
LV_IMG_DECLARE(ui_img_1404430020);    // assets\voltmeter-alt.png
LV_IMG_DECLARE(ui_img_551600910);    // assets\Btn-small-on-100.png



void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
