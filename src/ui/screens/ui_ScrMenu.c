// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.2.0
// Project name: SquareLine_Project_Jeepify_Large_PDC

#include "../ui.h"

void ui_ScrMenu_screen_init(void)
{
    ui_ScrMenu = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScrMenu, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_ScrMenu, &ui_img_bg_grill_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnMenu1 = lv_btn_create(ui_ScrMenu);
    lv_obj_set_width(ui_BtnMenu1, 50);
    lv_obj_set_height(ui_BtnMenu1, 50);
    lv_obj_set_x(ui_BtnMenu1, -194);
    lv_obj_set_y(ui_BtnMenu1, -121);
    lv_obj_set_align(ui_BtnMenu1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu1, &ui_img_icon_settings_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu1, lv_color_hex(0xC1851A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_BtnMenu1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_BtnMenu1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_BtnMenu1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_BtnMenu1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnMenu2 = lv_btn_create(ui_ScrMenu);
    lv_obj_set_width(ui_BtnMenu2, 50);
    lv_obj_set_height(ui_BtnMenu2, 50);
    lv_obj_set_x(ui_BtnMenu2, -113);
    lv_obj_set_y(ui_BtnMenu2, -120);
    lv_obj_set_align(ui_BtnMenu2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu2, &ui_img_icon_gauge_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu2, lv_color_hex(0xC1851A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu2, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_BtnMenu2, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_BtnMenu2, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_BtnMenu2, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_BtnMenu2, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnMenu3 = lv_btn_create(ui_ScrMenu);
    lv_obj_set_width(ui_BtnMenu3, 50);
    lv_obj_set_height(ui_BtnMenu3, 50);
    lv_obj_set_x(ui_BtnMenu3, -35);
    lv_obj_set_y(ui_BtnMenu3, -120);
    lv_obj_set_align(ui_BtnMenu3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu3, &ui_img_icon_switch_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu3, lv_color_hex(0xC1851A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu3, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_BtnMenu3, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_BtnMenu3, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_BtnMenu3, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_BtnMenu3, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnMenu4 = lv_btn_create(ui_ScrMenu);
    lv_obj_set_width(ui_BtnMenu4, 50);
    lv_obj_set_height(ui_BtnMenu4, 50);
    lv_obj_set_x(ui_BtnMenu4, 45);
    lv_obj_set_y(ui_BtnMenu4, -121);
    lv_obj_set_align(ui_BtnMenu4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu4, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu4, &ui_img_icon_pair_off_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu4, lv_color_hex(0xC1851A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu4, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_BtnMenu4, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_BtnMenu4, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_BtnMenu4, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_BtnMenu4, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnMenu4, lv_color_hex(0x8B6729), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_BtnMenu4, 255, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_img_src(ui_BtnMenu4, &ui_img_icon_pair_png, LV_PART_MAIN | LV_STATE_CHECKED);

    ui_BtnMenu5 = lv_btn_create(ui_ScrMenu);
    lv_obj_set_width(ui_BtnMenu5, 50);
    lv_obj_set_height(ui_BtnMenu5, 50);
    lv_obj_set_x(ui_BtnMenu5, 121);
    lv_obj_set_y(ui_BtnMenu5, -119);
    lv_obj_set_align(ui_BtnMenu5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu5, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu5, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu5, &ui_img_icon_keyboard_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu5, lv_color_hex(0xC1851A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu5, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_BtnMenu5, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_BtnMenu5, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_BtnMenu5, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_BtnMenu5, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnMenu5, lv_color_hex(0x8B6729), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_BtnMenu5, 255, LV_PART_MAIN | LV_STATE_CHECKED);

    ui_BtnMenu6 = lv_btn_create(ui_ScrMenu);
    lv_obj_set_width(ui_BtnMenu6, 50);
    lv_obj_set_height(ui_BtnMenu6, 50);
    lv_obj_set_x(ui_BtnMenu6, 198);
    lv_obj_set_y(ui_BtnMenu6, -121);
    lv_obj_set_align(ui_BtnMenu6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu6, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu6, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu6, &ui_img_icon_calib_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu6, lv_color_hex(0xC1851A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu6, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu6, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_BtnMenu6, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_BtnMenu6, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_BtnMenu6, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_BtnMenu6, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnMenu6, lv_color_hex(0x8B6729), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_BtnMenu6, 255, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_img_src(ui_BtnMenu6, &ui_img_icon_pair_png, LV_PART_MAIN | LV_STATE_CHECKED);

    ui_LblMenuJeepify = lv_label_create(ui_ScrMenu);
    lv_obj_set_width(ui_LblMenuJeepify, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LblMenuJeepify, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblMenuJeepify, 15);
    lv_obj_set_y(ui_LblMenuJeepify, 261);
    lv_label_set_text(ui_LblMenuJeepify, "Jeepify");
    lv_obj_set_style_text_color(ui_LblMenuJeepify, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblMenuJeepify, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LblMenuJeepify, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LblMenuType = lv_label_create(ui_ScrMenu);
    lv_obj_set_width(ui_LblMenuType, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LblMenuType, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LblMenuType, 14);
    lv_obj_set_y(ui_LblMenuType, 286);
    lv_label_set_text(ui_LblMenuType, "4-Switch PDC");
    lv_obj_set_style_text_color(ui_LblMenuType, lv_color_hex(0x825E02), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LblMenuType, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_LblMenuType, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LblMenuType, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BtnMenu8 = lv_btn_create(ui_ScrMenu);
    lv_obj_set_width(ui_BtnMenu8, 50);
    lv_obj_set_height(ui_BtnMenu8, 50);
    lv_obj_set_x(ui_BtnMenu8, 198);
    lv_obj_set_y(ui_BtnMenu8, -121);
    lv_obj_set_align(ui_BtnMenu8, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_BtnMenu8, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnMenu8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BtnMenu8, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnMenu8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BtnMenu8, &ui_img_icon_calib_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BtnMenu8, lv_color_hex(0xC1851A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BtnMenu8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BtnMenu8, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_BtnMenu8, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_BtnMenu8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_BtnMenu8, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_BtnMenu8, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_BtnMenu8, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_BtnMenu8, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnMenu8, lv_color_hex(0x8B6729), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_BtnMenu8, 255, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_img_src(ui_BtnMenu8, &ui_img_icon_pair_png, LV_PART_MAIN | LV_STATE_CHECKED);

    lv_obj_add_event_cb(ui_BtnMenu1, ui_event_BtnMenu1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu2, ui_event_BtnMenu2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu3, ui_event_BtnMenu3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu4, ui_event_BtnMenu4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu5, ui_event_BtnMenu5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu6, ui_event_BtnMenu6, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnMenu8, ui_event_BtnMenu8, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ScrMenu, ui_event_ScrMenu, LV_EVENT_ALL, NULL);

}
