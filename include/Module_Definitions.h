#ifndef MODULE_DEFINITIONS_H
#define MODULE_DEFINITIONS_H

//#define MODULE_JL_BATTERY_SENSOR      // (tut)
//#define MODULE_TERMINATOR_PRO
//#define MODULE_DOUBLEDRAGON         // ESP32-c3 - port, no adc
#define MODULE_LONELYDRAGON           // (tut) ESP32-c3 - no port, no adc
//#define MODULE_4WAY_INTEGRATED_8266 // (tut)

// JL-Battery-Sensor hinten (tut)
// 4 AMP-Sensors
// 1 VOLT-Sensor
#ifdef MODULE_JL_BATTERY_SENSOR
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

// Jeepify-Terminator-Pro (ESP32-C3-Mini)
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

// Jeepify-DoubleDragon (ESP32-C3-Mini)
// 2 Switches over port
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

// Jeepify-LonelyDragon (ESP32-C3-Mini) (tut)
// 2 Switches 
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
/*
#ifdef C3_MINI
    #define PORT_USED       1
    #define ADS_USED        1
    #define PAIRING_BUTTON  9
    #define LED_PIN         8
    #define LED_OFF         HIGH
    #define LED_ON          LOW
    #define SDA_PIN         6
    #define SCL_PIN         7
#endif

#ifdef ESP8266
    #define MRD_USED        1
    #define PORT_USED       1
    #define ADS_USED        1
    #define PAIRING_BUTTON  4
    #define LED_PIN         LED_BUILTIN
    #define LED_OFF         LOW
    #define LED_ON          HIGH
    #define SDA_PIN         14
    #define SCL_PIN         12
#endif
*/

#endif
