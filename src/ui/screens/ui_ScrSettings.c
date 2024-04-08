// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.2.0
// Project name: SquareLine_Project_Jeepify_Large_PDC

#include "../ui.h"

void ui_ScrSettings_screen_init(void)
{
    ui_ScrSettings = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScrSettings, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_ScrSettings, &ui_img_1825836894, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SwSettingsDemo = lv_switch_create(ui_ScrSettings);
    lv_obj_set_width(ui_SwSettingsDemo, 50);
    lv_obj_set_height(ui_SwSettingsDemo, 25);
    lv_obj_set_x(ui_SwSettingsDemo, -170);
    lv_obj_set_y(ui_SwSettingsDemo, -80);
    lv_obj_set_align(ui_SwSettingsDemo, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_SwSettingsDemo, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SwSettingsDemo, 255, LV_PART_MAIN | LV_STATE_DEFAULT);


    ui_SwSettingsSleep = lv_switch_create(ui_ScrSettings);
    lv_obj_set_width(ui_SwSettingsSleep, 50);
    lv_obj_set_height(ui_SwSettingsSleep, 25);
    lv_obj_set_x(ui_SwSettingsSleep, -170);
    lv_obj_set_y(ui_SwSettingsSleep, -40);
    lv_obj_set_align(ui_SwSettingsSleep, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_SwSettingsSleep, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SwSettingsSleep, 255, LV_PART_MAIN | LV_STATE_DEFAULT);


    ui_SwSettingsDebug = lv_switch_create(ui_ScrSettings);
    lv_obj_set_width(ui_SwSettingsDebug, 50);
    lv_obj_set_height(ui_SwSettingsDebug, 25);
    lv_obj_set_x(ui_SwSettingsDebug, -170);
    lv_obj_set_y(ui_SwSettingsDebug, 0);
    lv_obj_set_align(ui_SwSettingsDebug, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_SwSettingsDebug, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SwSettingsDebug, 255, LV_PART_MAIN | LV_STATE_DEFAULT);


    ui_SwSettingsPair = lv_switch_create(ui_ScrSettings);
    lv_obj_set_width(ui_SwSettingsPair, 50);
    lv_obj_set_height(ui_SwSettingsPair, 25);
    lv_obj_set_x(ui_SwSettingsPair, -170);
    lv_obj_set_y(ui_SwSettingsPair, 40);
    lv_obj_set_align(ui_SwSettingsPair, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_SwSettingsPair, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SwSettingsPair, 255, LV_PART_MAIN | LV_STATE_DEFAULT);


    ui_Label1 = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, 0);
    lv_obj_set_y(ui_Label1, -140);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "Settings");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xDBDBDB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label2 = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_Label2, 100);
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, -76);
    lv_obj_set_y(ui_Label2, -78);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "Demo");
    lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label3 = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_Label3, 100);
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, -76);
    lv_obj_set_y(ui_Label3, -40);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Sleep");
    lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label3, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label4 = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_Label4, 100);
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label4, -75);
    lv_obj_set_y(ui_Label4, 0);
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label4, "Debug");
    lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label4, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label5 = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_Label5, 164);
    lv_obj_set_height(ui_Label5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label5, -42);
    lv_obj_set_y(ui_Label5, 40);
    lv_obj_set_align(ui_Label5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label5, "Brightness");
    lv_obj_set_style_text_color(ui_Label5, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label5, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LblSettingName = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_LblSettingName, 236);
    lv_obj_set_height(ui_LblSettingName, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblSettingName, 10);
    lv_obj_set_y(ui_LblSettingName, 260);
    lv_label_set_text(ui_LblSettingName, "ESP32 - LVGL8 - #3.51");
    lv_obj_set_style_text_color(ui_LblSettingName, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblSettingName, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LblSettingName, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LblSettingType = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_LblSettingType, 290);
    lv_obj_set_height(ui_LblSettingType, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblSettingType, 10);
    lv_obj_set_y(ui_LblSettingType, 280);
    lv_label_set_text(ui_LblSettingType, "4-Way-Switch with Votage-Sensor");
    lv_obj_set_style_text_color(ui_LblSettingType, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblSettingType, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LblSettingType, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button4 = lv_btn_create(ui_ScrSettings);
    lv_obj_set_width(ui_Button4, 116);
    lv_obj_set_height(ui_Button4, 107);
    lv_obj_set_x(ui_Button4, 182);
    lv_obj_set_y(ui_Button4, -108);
    lv_obj_set_align(ui_Button4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_Button4, &ui_img_551600910, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_opa(ui_Button4, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label9 = lv_label_create(ui_Button4);
    lv_obj_set_width(ui_Label9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label9, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label9, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label9, "Reset");
    lv_obj_set_style_text_color(ui_Label9, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SlSettingsBrightness = lv_slider_create(ui_ScrSettings);
    lv_obj_set_width(ui_SlSettingsBrightness, 191);
    lv_obj_set_height(ui_SlSettingsBrightness, 10);
    lv_obj_set_x(ui_SlSettingsBrightness, 116);
    lv_obj_set_y(ui_SlSettingsBrightness, -12);
    lv_obj_set_align(ui_SlSettingsBrightness, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_SlSettingsBrightness, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SlSettingsBrightness, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_SlSettingsBrightness, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SlSettingsBrightness, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_SlSettingsBrightness, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SlSettingsBrightness, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_Label8 = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_Label8, 164);
    lv_obj_set_height(ui_Label8, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label8, 94);
    lv_obj_set_y(ui_Label8, -40);
    lv_obj_set_align(ui_Label8, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label8, "Brightness:");
    lv_obj_set_style_text_color(ui_Label8, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label8, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SlSettingsBrightnessValue = lv_label_create(ui_ScrSettings);
    lv_obj_set_width(ui_SlSettingsBrightnessValue, 46);
    lv_obj_set_height(ui_SlSettingsBrightnessValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SlSettingsBrightnessValue, 189);
    lv_obj_set_y(ui_SlSettingsBrightnessValue, -40);
    lv_obj_set_align(ui_SlSettingsBrightnessValue, LV_ALIGN_CENTER);
    lv_label_set_text(ui_SlSettingsBrightnessValue, "100");
    lv_obj_set_style_text_color(ui_SlSettingsBrightnessValue, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_SlSettingsBrightnessValue, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_SlSettingsBrightnessValue, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_SlSettingsBrightnessValue, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_SwSettingsDemo, ui_event_SwSettingsDemo, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwSettingsSleep, ui_event_SwSettingsSleep, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwSettingsDebug, ui_event_SwSettingsDebug, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwSettingsPair, ui_event_SwSettingsPair, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button4, ui_event_Button4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SlSettingsBrightness, ui_event_SlSettingsBrightness, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ScrSettings, ui_event_ScrSettings, LV_EVENT_ALL, NULL);

}
