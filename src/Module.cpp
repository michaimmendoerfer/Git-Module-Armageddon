#include "Module.h"
#include "PeerClass.h"

extern PeerClass Module;
extern MyLinkedList<PeriphClass*> SwitchList;
extern MyLinkedList<PeriphClass*> SensorList;
extern MyLinkedList<PeriphClass*> PeriphList;

const char *ArrNullwert[MAX_PERIPHERALS] = {"NW0",  "NW1",  "NW2",  "NW3",  "NW4",  "NW5",  "NW6",  "NW7",  "NW8"};
const char *ArrRaw[MAX_PERIPHERALS]      = {"Raw0", "Raw1", "Raw2", "Raw3", "Raw4", "Raw5", "Raw6", "Raw7", "Raw8"};
const char *ArrRawVolt[MAX_PERIPHERALS]  = {"RaV0", "RaV1", "RaV2", "RaV3", "RaV4", "RaV5", "RaV6", "RaV7", "RaV8"};
const char *ArrVperAmp[MAX_PERIPHERALS]  = {"VpA0", "VpA1", "VpA2", "VpA3", "VpA4", "VpA5", "VpA6", "VpA7", "VpA8"};
const char *ArrVin[MAX_PERIPHERALS]      = {"Vin0", "Vin1", "Vin2", "Vin3", "Vin4", "Vin5", "Vin6", "Vin7", "Vin8"};
const char *ArrPeriph[MAX_PERIPHERALS]   = {"Per0", "Per1", "Per2", "Per3", "Per4", "Per5", "Per6", "Per6", "Per7"};

void InitModule()
{   
    #ifdef MODULE_JL_BATTERY_SENSOR           
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version        Address   sleep  debug  demo  pair  vMon RelayType       SCA      SCL      voltagedevier 
        //Module.Setup(MODULE_NAME, BATTERY_SENSOR, MODULE_VERSION, NULL,     false, true, false, false, 4,  RELAY_NORMAL, SDA_PIN,  SCL_PIN, VOLTAGE_DEVIDER);
        Module.Setup(MODULE_NAME, BATTERY_SENSOR, MODULE_VERSION, NULL,     false, true, false, false);
        //                      Name     Type            ADC   IO                   NULL     VpA   Vin  PeerID
        Module.PeriphSetup(0, "Load",   SENS_TYPE_AMP,    1,    3,0,0,0,            2.5410,  0.040,  0,    0);
        Module.PeriphSetup(1, "Extern", SENS_TYPE_AMP,    1,    2,0,0,0,            2.4954,  0.066,  0,    0);
        Module.PeriphSetup(2, "Solar",  SENS_TYPE_AMP,    1,    1,0,0,0,            2.5005,  0.066,  0,    0);
        Module.PeriphSetup(3, "Intern", SENS_TYPE_AMP,    1,    0,0,0,0,            2.5005,  0.066,  0,    0);
        Module.PeriphSetup(4, "VMon",   SENS_TYPE_VOLT,   0,   VOLTAGE_PIN,0,0,0,   0,      0,      Vin,   0);  // 8266: 310 = 4095/3.3v
    #endif
    

    #ifdef MODULE_4WAY_ESP32_MONSTER   
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version        Address   sleep  debug  demo  pair  vMon RelayType       SCA      SCL      voltagedevier 
        Module.Setup(MODULE_NAME, SWITCH_2_WAY, MODULE_VERSION, NULL,     false, true,  false, false);
        //                      Name     Type             I2C               IO(0/1)        VOLT  AMP   NULL     VpA      Vin  PeerID  
        Module.PeriphSetup(0, "Sw 0",   SENS_TYPE_LT_AMP,  -1, -1, -1, -1,  27, 26,          33, 34,   0,       0.040,  1241,   0);
        Module.PeriphSetup(1, "Sw 1",   SENS_TYPE_LT_AMP,  -1, -1, -1, -1,  16, 17,          35, 39,   0,       0.040,  1241,   0); //39=vn
        Module.PeriphSetup(2, "VMon",   SENS_TYPE_VOLT,    -1, -1, -1, -1,  -1, -1, VOLTAGE_PIN, -1,   0,       0,      1241,   0);
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

    #ifdef MODULE_4WAY_INTEGRATED_C3      
        #define SWITCHES_PER_SCREEN 4
        //                      Name     Type             I2C               IO(0/1)        VOLT  AMP   NULL     VpA      Vin  PeerID  
        Module.Setup(MODULE_NAME, SWITCH_4_WAY, MODULE_VERSION,   NULL,     false, true, false, false);
        //                      Name     Type             I2C               IO(0/1)        VOLT  AMP   NULL     VpA      Vin  PeerID  
        Module.PeriphSetup(0, "Sw 1",   SENS_TYPE_SWITCH,   -1, -1, -1, -1,  5, -1,          33, 34,   0,       0.040,  1241,   0);
        Module.PeriphSetup(1, "Sw 2",   SENS_TYPE_SWITCH,   -1, -1, -1, -1,  6, -1,          33, 34,   0,       0.040,  1241,   0);
        Module.PeriphSetup(2, "Sw 3",   SENS_TYPE_SWITCH,   -1, -1, -1, -1,  7, -1,          33, 34,   0,       0.040,  1241,   0);
        Module.PeriphSetup(3, "Sw 4",   SENS_TYPE_SWITCH,   -1, -1, -1, -1, 10, -1,          33, 34,   0,       0.040,  1241,   0);
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
        if (Module.GetPeriphIOPort(SNr, 0) >= 0) 
        {
            #ifdef PORT_USED
                if (Module.GetPeriphI2CPort(SNr, 0) >= 0) IOBoard.pinMode(Module.GetPeriphIOPort(SNr, 0), OUTPUT);
            #else
                pinMode(Module.GetPeriphIOPort(SNr, 0), OUTPUT);  // off(lt) oder on/off 
            #endif
        }

        if (Module.GetPeriphIOPort(SNr, 1) >= 0) 
        {
            #ifdef PORT_USED
                if (Module.GetPeriphI2CPort(SNr, 1) >= 0) IOBoard.pinMode(Module.GetPeriphIOPort(SNr, 1), OUTPUT);
            #else
                pinMode(Module.GetPeriphIOPort(SNr, 1), OUTPUT);  // on(lt)
            #endif
        }

        if (Module.GetPeriphIOPort(SNr, 2) >= 0)     
        {
            #ifndef ADC_USED
                pinMode(Module.GetPeriphIOPort(SNr,2), INPUT);
                Serial.printf("setze %d auf INPUT", Module.GetPeriphIOPort(SNr,2));
            #endif
        }

        if (Module.GetPeriphIOPort(SNr, 3) >= 0)     
        {
            #ifndef ADC_USED
                pinMode(Module.GetPeriphIOPort(SNr,3), INPUT);
            #endif
        }
    }
    
    #ifdef PAIRING_BUTTON
        pinMode(PAIRING_BUTTON, INPUT_PULLUP); 
    #endif
    
    #ifdef LED_PIN
        pinMode(LED_PIN, OUTPUT);
    #endif
    
}
