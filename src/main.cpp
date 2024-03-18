#include <Arduino.h>

#include <esp32_smartdisplay.h>
#include <ui/ui.h>

void setup()
{
#ifdef ARDUINO_USB_CDC_ON_BOOT
    delay(5000);
#endif
    Serial.begin(115200);

    smartdisplay_init();

    __attribute__((unused)) auto disp = lv_disp_get_default();
    lv_disp_set_rotation(disp, LV_DISP_ROT_90);

    ui_init();
}

ulong next_millis;

void loop()
{


#ifdef BOARD_HAS_RGB_LED
        auto const rgb = (millis() / 2000) % 8;
        //smartdisplay_led_set_rgb(rgb & 0x01, rgb & 0x02, rgb & 0x04);
#endif



    lv_timer_handler();
}