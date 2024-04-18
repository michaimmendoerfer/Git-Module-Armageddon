#ifndef MODULE_DEFINITIONS_H
#define MODULE_DEFINITIONS_H

// Module Definition
//#define C3_MINI
//#define C3_MINI_PORT
//#define C3_MINI_PORT_ADS
//#define ESP8266_PORT
#define ESP8266_ADS
//#define ESP8266_4WAY_INTEGRATED
//#define ESP32_DISPLAY_480
//#define DISPLAY_C3_ROUND

// IC-Things
#ifdef C3_MINI
    //#define ESP32_MODULE_1S_1V
    //#define ESP32_MODULE_2S_1V
    
    #define BOOT_BUTTON 9
    #define LED_PIN     8
    #define LED_OFF     HIGH
    #define LED_ON      LOW
#endif

#ifdef C3_MINI_PORT
    #define ESP32_MODULE_4S_1V_PORT
    //#define ESP32_MODULE_4A_1V_ADS

    #define PORT_USED   1
    #define BOOT_BUTTON 9
    #define LED_PIN     8
    #define LED_OFF     HIGH
    #define LED_ON      LOW
    #define SDA_PIN     6
    #define SCL_PIN     7
#endif

#ifdef C3_MINI_ADS
    //#define ESP32_MODULE_4A_1V_ADS

    #define ADS_USED    1
    #define BOOT_BUTTON 9
    #define LED_PIN     8
    #define LED_OFF     HIGH
    #define LED_ON      LOW
    #define SDA_PIN     6
    #define SCL_PIN     7
#endif

#ifdef C3_MINI_PORT_ADS
    #define ESP32_MODULE_4S_4A_1V_PORT
    
    #define PORT_USED   1
    #define ADS_USED    1
    #define BOOT_BUTTON 9
    #define LED_PIN     8
    #define LED_OFF     HIGH
    #define LED_ON      LOW
    #define SDA_PIN     6
    #define SCL_PIN     7
#endif

#ifdef ESP8266_PORT
    #define MRD_USED    1
    #define PORT_USED   1
    #define LED_PIN     LED_BUILTIN
    #define LED_OFF     LOW
    #define LED_ON      HIGH
    #define SDA_PIN     14
    #define SCL_PIN     12
#endif

// Jeep Battery hinten
#ifdef ESP8266_ADS
    #define ESP8266_MODULE_4A_1V_ADS
    #define MRD_USED    1
    #define ADS_USED   1
    #define LED_PIN     LED_BUILTIN
    #define LED_OFF     LOW
    #define LED_ON      HIGH
    #define SDA_PIN     14
    #define SCL_PIN     12
#endif

// fertiges 4-Kanal ESP8266
#ifdef ESP8266_4WAY_INTEGRATED
    #define ESP8266_MODULE_4S_INTEGRATED
    #define MRD_USED    1
    #define LED_PIN     LED_BUILTIN
    #define LED_OFF     LOW
    #define LED_ON      HIGH
    #define SDA_PIN     14
    #define SCL_PIN     12

    #define BOOT_BUTTON 4 
#endif


#endif
