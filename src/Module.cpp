#include "Module.h"
#include "PeerClass.h"

extern PeerClass Module;
extern MyLinkedList<PeriphClass*> SwitchList;
extern MyLinkedList<PeriphClass*> SensorList;
extern MyLinkedList<PeriphClass*> PeriphList;

void InitModule()
{
    float Vin = BOARD_VOLTAGE/BOARD_ANALOG_MAX;
        
    #ifdef MODULE_JL_BATTERY_SENSOR           
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version        Address   sleep  debug  demo  pair  vMon RelayType       SCA      SCL      voltagedevier 
        Module.Setup(MODULE_NAME, BATTERY_SENSOR, MODULE_VERSION, NULL,     false, true, false, false, 4,  RELAY_NORMAL, SDA_PIN,  SCL_PIN, VOLTAGE_DEVIDER);
        //                      Name     Type            ADC   IO   NULL     VpA   Vin  PeerID
        Module.PeriphSetup(0, "Load",   SENS_TYPE_AMP,    1,    3,  2.5410,  0.040,  0,    0);
        Module.PeriphSetup(1, "Extern", SENS_TYPE_AMP,    1,    2,  2.4954,  0.066,  0,    0);
        Module.PeriphSetup(2, "Solar",  SENS_TYPE_AMP,    1,    1,  2.5005,  0.066,  0,    0);
        Module.PeriphSetup(3, "Intern", SENS_TYPE_AMP,    1,    0,  2.5005,  0.066,  0,    0);
        Module.PeriphSetup(4, "VMon",   SENS_TYPE_VOLT,   0,    1,   0,      0,      Vin,  0);  // 8266: 310 = 4095/3.3v
    #endif

    #ifdef MODULE_TERMINATOR_PRO   
        #define SWITCHES_PER_SCREEN 4
        //                Name        Type         Version        Address   sleep  debug  demo  pair  vMon RelayType       SCA      SCL      voltagedevier 
        Module.Setup(MODULE_NAME, BATTERY_SENSOR, MODULE_VERSION, NULL,     false, true,  false, false, 1,  RELAY_NORMAL, SDA_PIN,  SCL_PIN,   5);
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
    
    #ifdef PAIRING_BUTTON
        pinMode(PAIRING_BUTTON, INPUT_PULLUP); 
    #endif
}
