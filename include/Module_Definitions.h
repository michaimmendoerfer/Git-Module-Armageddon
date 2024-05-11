#ifndef MODULE_DEFINITIONS_H
#define MODULE_DEFINITIONS_H

#define MODULE_JL_BATTERY_SENSOR      // (tut)
//#define MODULE_TERMINATOR_PRO
//#define MODULE_DOUBLEDRAGON
//#define MODULE_LONELYDRAGON
//#define MODULE_4WAY_INTEGRATED_8266 // (tut)

// Module Definition
//#define C3_MINI
//#define C3_MINI_PORT
//#define C3_MINI_PORT_ADS
//#define ESP8266_PORT
//#define ESP8266_ADS
//#define ESP32_DISPLAY_480
//#define DISPLAY_C3_ROUND

// JL-Battery-Sensor hinten (tut)
// 4 AMP-Sensors
// 1 VOLT-Sensor
#ifdef MODULE_JL_BATTERY_SENSOR
    #define ESP8266_ADS
    #define ESP8266_MODULE_4A_1V_ADS
    #define MRD_USED        1
    #define ADS_USED        1
    //#define PAIRING_BUTTON 4
    #define LED_PIN         LED_BUILTIN
    #define LED_OFF         HIGH
    #define LED_ON          LOW
    #define SDA_PIN         14
    #define SCL_PIN         12
#endif

// fertiges 4-Kanal mit ESP8266 (tut)
// 4 Switches
#ifdef MODULE_4WAY_INTEGRATED_8266
    #define ESP8266_MODULE_4S_INTEGRATED
    #define MRD_USED        1
    #define PAIRING_BUTTON  4 
    #define LED_PIN         LED_BUILTIN
    #define LED_OFF         LOW
    #define LED_ON          HIGH
    #define SDA_PIN         14
    #define SCL_PIN         12
#endif

// Jeepify-Terminator-Pro
// 4 Switches
// 4 AMP-Sensors
// 1 VOLT-Sensor
#ifdef MODULE_TERMINATOR_PRO
    #define ESP32_MODULE_4S_4A_1V_ADS_PORT
    #define PORT_USED       1
    #define ADS_USED        1
    #define PAIRING_BUTTON 9
    #define LED_PIN         8
    #define LED_OFF         HIGH
    #define LED_ON          LOW
    #define SDA_PIN         6
    #define SCL_PIN         7
#endif

#ifdef MODULE_DOUBLEDRAGON
    #define ESP32_MODULE_2S_PORT
    #define PORT_USED       1
    #define PAIRING_BUTTON  9
    #define LED_PIN         8
    #define LED_OFF         HIGH
    #define LED_ON          LOW
    #define SDA_PIN         6
    #define SCL_PIN         7
#endif

#ifdef MODULE_LONELYDRAGON
    #define ESP32_MODULE_2S_NOPORT
    #define PAIRING_BUTTON  9
    #define LED_PIN         8
    #define LED_OFF         HIGH
    #define LED_ON          LOW
    #define SDA_PIN         6
    #define SCL_PIN         7
#endif

// IC-Things
#ifdef C3_MINI
    //#define ESP32_MODULE_1S_1V
    //#define ESP32_MODULE_2S_1V
    
    #define PAIRING_BUTTON  9
    #define LED_PIN         8
    #define LED_OFF         HIGH
    #define LED_ON          LOW
#endif

#ifdef C3_MINI_PORT
    #define ESP32_MODULE_4S_1V_PORT
    //#define ESP32_MODULE_4A_1V_ADS

    #define PORT_USED       1
    #define PAIRING_BUTTON  9
    #define LED_PIN         8
    #define LED_OFF         HIGH
    #define LED_ON          LOW
    #define SDA_PIN         6
    #define SCL_PIN         7
#endif

#ifdef C3_MINI_ADS
    //#define ESP32_MODULE_4A_1V_ADS

    #define ADS_USED        1
    #define PAIRING_BUTTON  9
    #define LED_PIN         8
    #define LED_OFF         HIGH
    #define LED_ON          LOW
    #define SDA_PIN         6
    #define SCL_PIN         7
#endif

#ifdef C3_MINI_PORT_ADS
    #define ESP32_MODULE_4S_4A_1V_PORT
    
    #define PORT_USED       1
    #define ADS_USED        1
    #define PAIRING_BUTTON  9
    #define LED_PIN         8
    #define LED_OFF         HIGH
    #define LED_ON          LOW
    #define SDA_PIN         6
    #define SCL_PIN         7
#endif

#ifdef ESP8266_PORT
    #define MRD_USED        1
    #define PORT_USED       1
    #define PAIRING_BUTTON  4
    #define LED_PIN         LED_BUILTIN
    #define LED_OFF         LOW
    #define LED_ON          HIGH
    #define SDA_PIN         14
    #define SCL_PIN         12
#endif

#endif
