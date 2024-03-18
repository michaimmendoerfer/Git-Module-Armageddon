SET(SOURCES screens/ui_ScrMenu.c
    screens/ui_ScrSettings.c
    screens/ui_ScrGauge.c
    screens/ui_ScrSwitch.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.cpp
    images/ui_img_bg_grill_png.c
    images/ui_img_icon_settings_png.c
    images/ui_img_icon_gauge_png.c
    images/ui_img_icon_switch_png.c
    images/ui_img_1825836894.c
    images/ui_img_551600910.c)

add_library(ui ${SOURCES})
