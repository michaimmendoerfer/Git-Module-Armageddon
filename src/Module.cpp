#include "Module.h"
#include "PeerClass.h"

extern PeerClass Module;
extern MyLinkedList<PeriphClass*> SwitchList;
extern MyLinkedList<PeriphClass*> SensorList;
extern MyLinkedList<PeriphClass*> PeriphList;

void InitModule()
{
    float Vin = BOARD_ANALOG_MAX/BOARD_VOLTAGE;
        
    #ifdef MODULE_JL_BATTERY_SENSOR           
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version        Address   sleep  debug  demo  pair  vMon RelayType       SCA      SCL      voltagedevier 
        Module.Setup(MODULE_NAME, BATTERY_SENSOR, MODULE_VERSION, NULL,     false, true, false, false, 4,  RELAY_NORMAL, SDA_PIN,  SCL_PIN, VOLTAGE_DEVIDER);
        //                      Name     Type            ADC   IO   NULL   VpA   Vin  PeerID
        Module.PeriphSetup(0, "Load",   SENS_TYPE_AMP,    1,    3,  2.53, 0.040,  0,    0);
        Module.PeriphSetup(1, "Extern", SENS_TYPE_AMP,    1,    2,  2.5,  0.066,  0,    0);
        Module.PeriphSetup(2, "Solar",  SENS_TYPE_AMP,    1,    1,  2.5,  0.066,  0,    0);
        Module.PeriphSetup(3, "Intern", SENS_TYPE_AMP,    1,    0,  2.5,  0.066,  0,    0);
        Module.PeriphSetup(4, "VMon",   SENS_TYPE_VOLT,   0,    1,   0,      0,   Vin,  0);  // 8266: 310 = 4095/3.3v
    #endif

    #ifdef MODULE_TERMINATOR_PRO   
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version        Address   sleep  debug  demo  pair  vMon RelayType       SCA      SCL      voltagedevier 
        Module.Setup(MODULE_NAME, BATTERY_SENSOR, MODULE_VERSION, NULL,     false, true,  false, false, 1,  RELAY_NORMAL, SDA_PIN,  SCL_PIN, VOLTAGE_DEVIDER);
        //                      Name     Type             ADS  IO   NULL     VpA      Vin  PeerID   Brother
        Module.PeriphSetup(0, "Sw 1",   SENS_TYPE_COMBO,   1,  0,   0,       0,        0,    0,       4);
        Module.PeriphSetup(1, "Sw 2",   SENS_TYPE_COMBO,   1,  1,   0,       0,        0,    0,       5);
        Module.PeriphSetup(2, "Sw 3",   SENS_TYPE_COMBO,   1,  2,   0,       0,        0,    0,       6);
        Module.PeriphSetup(3, "Sw 4 ",  SENS_TYPE_COMBO,   1,  3,   0,       0,        0,    0,       7);
        Module.PeriphSetup(4, "Amp 1",  SENS_TYPE_AMP,     1,  0,   2.5,     0.066,    0,    0,       0);
        Module.PeriphSetup(5, "Amp 2",  SENS_TYPE_AMP,     1,  1,   2.5,     0.066,    0,    0,       1);
        Module.PeriphSetup(6, "Amp 3",  SENS_TYPE_AMP,     1,  2,   2.5,     0.066,    0,    0,       2);
        Module.PeriphSetup(7, "Amp 4",  SENS_TYPE_AMP,     1,  3,   2.5,     0.066,    0,    0,       3);
        
        Module.PeriphSetup(8, "V-Sens", SENS_TYPE_VOLT,    0,  35,  0,       0,      200,    0); 
    #endif

    #ifdef ESP8266_MODULE_4S_INTEGRATED       
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version        Address   sleep  debug  demo  pair  vMon   RelayType      sda    scl  voltagedevier 
        Module.Setup(MODULE_NAME, BATTERY_SENSOR, MODULE_VERSION, NULL,     false, true,  true, false, -1,  RELAY_NORMAL,   -1,   -1,     -1);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
        Module.PeriphSetup(0, "Sw 1",   SENS_TYPE_SWITCH,   0,  15,    0,      0,       0,    0);
        Module.PeriphSetup(1, "Sw 2",   SENS_TYPE_SWITCH,   0,  14,    0,      0,       0,    0);
        Module.PeriphSetup(2, "Sw 3",   SENS_TYPE_SWITCH,   0,  12,    0,      0,       0,    0);
        Module.PeriphSetup(3, "Sw 4",   SENS_TYPE_SWITCH,   0,  13,    0,      0,       0,    0);
    #endif

    // Register Periphs
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)
    {
        if (Module.isPeriphSensor(SNr)) 
        {
            SensorList.add(Module.GetPeriphPtr(SNr));
            PeriphList.add(Module.GetPeriphPtr(SNr));
        }
        else if (Module.isPeriphSwitch(SNr)) 
        {
            SwitchList.add(Module.GetPeriphPtr(SNr));
            PeriphList.add(Module.GetPeriphPtr(SNr));
        }
    }

    // Set pinModes
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) 
    { 
        switch (Module.GetPeriphType(SNr)) {
            case SENS_TYPE_SWITCH: 
                #ifdef PORT_USED
                    IOBoard.pinMode(Module.GetPeriphIOPort(SNr), OUTPUT);
                #else
                    pinMode(Module.GetPeriphIOPort(SNr), OUTPUT); 
                #endif
                break;
            case SENS_TYPE_VOLT:   pinMode(Module.GetPeriphIOPort(SNr), INPUT ); break;
            case SENS_TYPE_AMP:    
                #ifndef ADC_USED
                    pinMode(Module.GetPeriphIOPort(SNr), INPUT );
                #endif
                break;
        }
    }
    
    pinMode(LED_PIN, OUTPUT);
    
    #ifdef PAIRING_BUTTON
        pinMode(PAIRING_BUTTON, INPUT_PULLUP); 
    #endif
}
/*
void InitModule2()
{    
    ////////////////////////////////////////////////////////
    ////////////// ESP32 ///////////////////////////////////
    ////////////////////////////////////////////////////////
    /*
    ESP32 DevKit:
    possible Inputs:  16,17,18,19,21,22,23,       
    possible Outputs: 16,17,18,19,21,22,23,32,33
    best: (ADC1:32,33,34,35,36,39)
    
    dont´t use
    ADC2 (when Wifi): 0, 2, 4, 12, 13, 14, 15, 25, 26, 27
    SPI-Flash:        6,7,8,9,10,11 
    
    ESP32 C3 Mini:
    don´t use 2, 8, 9
    */

   #ifdef MODULE_JL_BATTERY_SENSOR_C3           // 4-way Battery-Sensor with ADS and VMon C3 #########################################################
      // 4x acs712(30A) over ADC1115, Voltage-Monitor:A0
      #define SWITCHES_PER_SCREEN 4
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, BATTERY_SENSOR, _Version, NULL,     false, true, false, false, 4,  RELAY_NORMAL, 14,  12,     5);
      //                      Name     Type            ADS  IO   NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Load",   SENS_TYPE_AMP,    1,    3,  2.53, 0.040,  0,    0);
      Module.PeriphSetup(1, "Extern", SENS_TYPE_AMP,    1,    2,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(2, "Solar",  SENS_TYPE_AMP,    1,    1,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(3, "Intern", SENS_TYPE_AMP,    1,    0,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(4, "VMon",   SENS_TYPE_VOLT,   0,    1,   0,      0,   BOARD_ANALOG_MAX/BOARD_VOLTAGE,    0);  // 8266: 310 = 4095/3.3v
    #endif

    #ifdef ESP32_MODULE_2S_2A_1V_ADS      // Mixed-Module 2 Relays with Sensor over adc and VMon ######################################################################
        #define SWITCHES_PER_SCREEN 2
        //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
        Module.Setup(_ModuleName, PDC_SENSOR_MIX, _Version, NULL,     false, true,  true, false, 1,  RELAY_NORMAL, -1,  -1,     1.5);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID  Brother
        Module.PeriphSetup(0, "Amp 1",  SENS_TYPE_AMP,     1,  0,    2.5,     0.066,    0,    0,     2);
        Module.PeriphSetup(1, "Amp 2",  SENS_TYPE_AMP,     1,  1,    2.5,     0.066,    0,    0,     3);
        Module.PeriphSetup(2, "Sw 1",   SENS_TYPE_SWITCH,  0,  1,    0,       0,        0,    0,     0);
        Module.PeriphSetup(3, "Sw 2 ",  SENS_TYPE_SWITCH,  0,  2,    0,       0,        0,    0,     1);
        Module.PeriphSetup(4, "V-Sens", SENS_TYPE_VOLT,    0,  3,    0,       0,      200,    0); 
    #endif

    #ifdef ESP32_MODULE_4S_1V_NOADC_PORT    // 4-Way Switch via IOBoard with Voltage-Monitor #####################################################
      #define SWITCHES_PER_SCREEN 4
      //                Name        Type       Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, SWITCH_4_WAY, _Version, NULL,     false, true,  true, false, -1, RELAY_NORMAL, 21,  22,     1);
      //                      Name     Type           ADS  IO  NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "SP-1", SENS_TYPE_SWITCH,  1,  P0,   0,    0,    0,    0);
      Module.PeriphSetup(1, "SP-2", SENS_TYPE_SWITCH,  1,  P1,   0,    0,    0,    0);
      Module.PeriphSetup(2, "SP-3", SENS_TYPE_SWITCH,  1,  P2,   0,    0,    0,    0);
      Module.PeriphSetup(3, "SP-4", SENS_TYPE_SWITCH,  1,  P3,   0,    0,    0,    0);
      Module.PeriphSetup(4, "Volt", SENS_TYPE_VOLT,    0,  35,  0,    0,   200,   0); 
    #endif
    #ifdef ESP32_MODULE_4A_1V_ADC           // 4-way Battery-Sensor with ADC and VMon ############################################################
      // 4x acs712(30A) over ADC1115 on SCL:21, SDA:22, Voltage-Monitor:35
      #define SWITCHES_PER_SCREEN 4
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, BATTERY_SENSOR, _Version, NULL,     false, true,  true, false, 1,  RELAY_NORMAL, 14,  15,     1);
      //                      Name     Type             ADS  IO   NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Sensor_1", SENS_TYPE_AMP,  1,    1,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(1, "Sensor_2", SENS_TYPE_AMP,  1,    2,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(2, "Sensor_3", SENS_TYPE_AMP,  1,    3,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(3, "Sensor_4", SENS_TYPE_AMP,  1,    4,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(4, "VMon",     SENS_TYPE_VOLT, 0,   39,   0,     0,   200,   0); 
    #endif
    #ifdef ESP32_MODULE_2A_2S_1V_NOADS      // Mixed-Module no ADC and VMon ######################################################################
        #define SWITCHES_PER_SCREEN 2
        //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
        Module.Setup(_ModuleName, PDC_SENSOR_MIX, _Version, NULL,     false, true,  true, false, 1,  RELAY_NORMAL, -1,  -1,     1.5);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
        Module.PeriphSetup(0, "Amp 1",  SENS_TYPE_AMP,     0,  34,   2.5,     0.066,    0,    0);
        Module.PeriphSetup(1, "Amp 2",  SENS_TYPE_AMP,     0,  35,   2.5,     0.066,    0,    0);
        Module.PeriphSetup(2, "Sw 1",   SENS_TYPE_SWITCH,  0,  32,   0,       0,        0,    0);
        Module.PeriphSetup(3, "Sw 2 ",  SENS_TYPE_SWITCH,  0,  33,   0,       0,        0,    0);
        Module.PeriphSetup(4, "V-Sens", SENS_TYPE_VOLT,    0,  39,   0,       0,      200,    0); 
    #endif
    #ifdef ESP32_MODULE_2S_PORT             // 2-Way Switch via IOBoard ##########################################################################
        // DoubleDragon
        #define SWITCHES_PER_SCREEN 2       
        //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon    RelayType     sda scl voltagedevier 
        Module.Setup(_ModuleName, SWITCH_2_WAY,   _Version, NULL,     false, true,  true, false, -1,  RELAY_REVERSED, 6,  7,     1.5);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
        Module.PeriphSetup(0, "Sw 1",  SENS_TYPE_SWITCH,   1,  0,     0,       0,        0,    0);
        Module.PeriphSetup(1, "Sw 2",  SENS_TYPE_SWITCH,   1,  1,     0,       0,        0,    0);
    #endif
    #ifdef ESP32_MODULE_2S_NOPORT           // 2-Way Switch via PIO ##############################################################################
        // LonelyDragon
        #define SWITCHES_PER_SCREEN 2       
        //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon    RelayType     sda scl voltagedevier 
        Module.Setup(_ModuleName, SWITCH_2_WAY,   _Version, NULL,     false, true, false, false, -1,  RELAY_REVERSED, -1,  -1,     -1);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
        Module.PeriphSetup(0, "LEDBar",  SENS_TYPE_SWITCH,   0,  4,     0,       0,        0,    0);
        Module.PeriphSetup(1, "Fridge",  SENS_TYPE_SWITCH,   0,  3,     0,       0,        0,    0);
    #endif
    #ifdef ESP32_MODULE_4S_NOPORT           // 4-Way Switch via PIO ##############################################################################
        // LonelyDragon
        #define SWITCHES_PER_SCREEN 4       
        //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon    RelayType     sda scl voltagedevier 
        Module.Setup(_ModuleName, SWITCH_4_WAY,   _Version, NULL,     false, true, false, false, -1,  RELAY_REVERSED, -1,  -1,     -1);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID   Brother
        Module.PeriphSetup(0, "LEDBar",  SENS_TYPE_SWITCH,   0,  0,     0,     0,        0,    0,     4);
        Module.PeriphSetup(1, "Fridge",  SENS_TYPE_SWITCH,   0,  1,     0,     0,        0,    0);
        Module.PeriphSetup(2, "Table",   SENS_TYPE_SWITCH,   0,  2,     0,     0,        0,    0);
        Module.PeriphSetup(3, "Inside",  SENS_TYPE_SWITCH,   0,  3,     0,     0,        0,    0);
        Module.PeriphSetup(4, "Amp",     SENS_TYPE_AMP,      0,  5,    2.5,   0.066,     0,    0);
        Module.PeriphSetup(5, "Volt",    SENS_TYPE_VOLT,     0,  6,     0,     0,        0,    0);
        
    #endif
    #ifdef ESP32_MODULE_4S_4A_1V_ADS_PORT   // Mixed-Module with ADC and Port and VMon ###########################################################
      // TERMINATOR_PRO (untested) - 4 sensed switches with acs712(30A) over ADS and Port, Voltage-Monitor:??
      #define SWITCHES_PER_SCREEN 
      //                Name        Type         Version  Address   sleep  debug  demo   pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, PDC_SENSOR_MIX, _Version, NULL,     false, true,  false, false, 1,  RELAY_NORMAL, 6,  7,     1.5);
      //                      Name     Type             ADS  IO   NULL     VpA      Vin  PeerID   Brother
      Module.PeriphSetup(0, "Sw 1",   SENS_TYPE_COMBO,   1,  0,   0,       0,        0,    0,       4);
      Module.PeriphSetup(1, "Sw 2",   SENS_TYPE_COMBO,   1,  1,   0,       0,        0,    0,       5);
      Module.PeriphSetup(2, "Sw 3",   SENS_TYPE_COMBO,   1,  2,   0,       0,        0,    0,       6);
      Module.PeriphSetup(3, "Sw 4 ",  SENS_TYPE_COMBO,   1,  3,   0,       0,        0,    0,       7);
      Module.PeriphSetup(4, "Amp 1",  SENS_TYPE_AMP,     1,  0,   2.5,     0.066,    0,    0,       0);
      Module.PeriphSetup(5, "Amp 2",  SENS_TYPE_AMP,     1,  1,   2.5,     0.066,    0,    0,       1);
      Module.PeriphSetup(6, "Amp 3",  SENS_TYPE_AMP,     1,  2,   2.5,     0.066,    0,    0,       2);
      Module.PeriphSetup(7, "Amp 4",  SENS_TYPE_AMP,     1,  3,   2.5,     0.066,    0,    0,       3);
      
      Module.PeriphSetup(8, "V-Sens", SENS_TYPE_VOLT,    0,  35,  0,       0,      200,    0); 
    #endif

    ////////////////////////////////////////////////////////
    ////////////// ESP8266 /////////////////////////////////
    ////////////////////////////////////////////////////////
    /*
    possible Inputs:  
    possible Outputs: 
    best: 
    
    dont´t use
    */

   #ifdef MODULE_1WAY_INTEGRATED_8266           // 1-way Switch (10A) - 8266 onBoard +++++++ ############################################################
        // 1 Switch over PIO
      #define SWITCHES_PER_SCREEN 2
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon   RelayType      sda    scl    voltagedevier 
        Module.Setup(_ModuleName, SWITCH_2_WAY,   _Version, NULL,     false, true,  true, false, -1,  RELAY_NORMAL,   -1,   -1,     1.5);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
        Module.PeriphSetup(0, "S10-1",   SENS_TYPE_SWITCH,   0,  4,    0,      0,       0,    0);
    #endif

   #ifdef ESP8266_MODULE_2S_INTEGRATED           // 2-way Switch (30A) - 8266 onBoard +++++++ ############################################################
        // 2 Switch over PIO
      #define SWITCHES_PER_SCREEN 2
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon   RelayType      sda    scl    voltagedevier 
        Module.Setup(_ModuleName, SWITCH_2_WAY,   _Version, NULL,     false, true,  true, false, -1,  RELAY_NORMAL,   -1,   -1,     1.5);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
        Module.PeriphSetup(0, "S30-1",   SENS_TYPE_SWITCH,   0,  14,    0,      0,       0,    0);
        Module.PeriphSetup(1, "S30-2",   SENS_TYPE_SWITCH,   0,  12,    0,      0,       0,    0);
    #endif
    //works
    #ifdef ESP8266_MODULE_4A_1V_ADS           // 4-way Battery-Sensor with ADS and VMon #########################################################
      // 4x acs712(30A) over ADC1115, Voltage-Monitor:A0
      #define SWITCHES_PER_SCREEN 4
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, BATTERY_SENSOR, _Version, NULL,     false, true, false, false, 4,  RELAY_NORMAL, 14,  12,     5);
      //                      Name     Type            ADS  IO   NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Load",   SENS_TYPE_AMP,    1,    0,  2.5,  0.040,  0,    0);
      Module.PeriphSetup(1, "Extern", SENS_TYPE_AMP,    1,    1,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(2, "Solar",  SENS_TYPE_AMP,    1,    2,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(3, "Intern", SENS_TYPE_AMP,    1,    3,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(4, "VMon",   SENS_TYPE_VOLT,   0,   A0,   0,      0,   333,    0);  // 8266: 310 = 1023/3.3v
    #endif
    //works
    #ifdef ESP8266_MODULE_4S_INTEGRATED       // 4-way Switch - 8266 onBoard +++++++ ############################################################
        // 4x Switch over PIO
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon   RelayType      sda    scl    voltagedevier 
        Module.Setup(_ModuleName, SWITCH_4_WAY,   _Version, NULL,     false, true,  true, false, -1,  RELAY_NORMAL,   -1,   -1,     1.5);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
        Module.PeriphSetup(0, "Sw 1",   SENS_TYPE_SWITCH,   0,  15,    0,      0,       0,    0);
        Module.PeriphSetup(1, "Sw 2",   SENS_TYPE_SWITCH,   0,  14,    0,      0,       0,    0);
        Module.PeriphSetup(2, "Sw 3",   SENS_TYPE_SWITCH,   0,  12,    0,      0,       0,    0);
        Module.PeriphSetup(3, "Sw 4",   SENS_TYPE_SWITCH,   0,  13,    0,      0,       0,    0);
    #endif
    #ifdef MODULE_4WAY_INTEGRATED_ESP01       // 4-way Switch klein ESP01 ++++++++++ ############################################################
        // 4x Switch over Serial
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon   RelayType      sda    scl    voltagedevier 
        Module.Setup(_ModuleName, SWITCH_4_WAY,   _Version, NULL,     false, true,  true, false, -1,  RELAY_NORMAL,   -1,   -1,     1.5);
        //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
        Module.PeriphSetup(0, "Sw 1",   SENS_TYPE_SWITCH,   0,  99,    0,      0,       0,    0);
        Module.PeriphSetup(1, "Sw 2",   SENS_TYPE_SWITCH,   0,  99,    0,      0,       0,    0);
        Module.PeriphSetup(2, "Sw 3",   SENS_TYPE_SWITCH,   0,  99,    0,      0,       0,    0);
        Module.PeriphSetup(3, "Sw 4",   SENS_TYPE_SWITCH,   0,  99,    0,      0,       0,    0);
    #endif
    
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)
    {
        if (Module.isPeriphSensor(SNr)) 
        {
            SensorList.add(Module.GetPeriphPtr(SNr));
            PeriphList.add(Module.GetPeriphPtr(SNr));
        }
        else if (Module.isPeriphSwitch(SNr)) 
        {
            SwitchList.add(Module.GetPeriphPtr(SNr));
            PeriphList.add(Module.GetPeriphPtr(SNr));
        }
    }
}
*/
