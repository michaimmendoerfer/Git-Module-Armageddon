//#define KILL_NVS 1

// DEBUG_LEVEL: 0 = nothing, 1 = only Errors, 2 = relevant changes, 3 = all
const int DEBUG_LEVEL = 3; 

#pragma region Includes
#include <Arduino.h>
#include "Module_Definitions.h"

#ifdef ESP32_DISPLAY_480
    #include <esp32_smartdisplay.h>
    #define ADS_USED  1
    #define PORT_USED 1
    #include <ui/ui.h>
#endif

#ifdef MRD_USED
    #ifdef ESP8266 // ESP8266_MRD_USE_RTC false
        #define ESP8266_MRD_USE_RTC   false  
    #endif
    #define ESP_MRD_USE_LITTLEFS           true
    #define MULTIRESETDETECTOR_DEBUG       true  //false
    #define MRD_TIMES               3
    #define MRD_TIMEOUT             10
    #define MRD_ADDRESS             0

    #include <ESP_MultiResetDetector.h>
    MultiResetDetector* mrd;
#endif
#pragma region I2C_BUS
#define I2C_FREQ 400000
#define ADS_ADDRESS  0x48
#define PORT_ADDRESS 0x20

#ifdef ADS_USED
    #include <Adafruit_ADS1X15.h>
    Adafruit_ADS1115 ADSBoard;
#endif

#ifdef PORT_USED
    #include "PCF8575.h"
    PCF8575 IOBoard(PORT_ADDRESS, SDA_PIN, SCL_PIN);
#endif
#pragma endregion I2C_BUS

#ifdef ESP32
    #include <esp_now.h>
    #include <WiFi.h>
    #include <nvs_flash.h>
    #define u8 unsigned char
#elif defined(ESP8266)
    #include <ESP8266WiFi.h>
    #include <espnow.h>
#endif 

#include <LinkedList.h>
#include "Jeepify.h"
#include "PeerClass.h"
#include "pref_manager.h"
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Spi.h>

#pragma endregion Includes

const char _Version[]           = "3.41";
const char _Protokoll_Version[] = "1.01";
const char _ModuleName[]        = "LD-2-1";
const bool _LED_SIGNAL          = true;

#pragma region Globals
struct struct_Status {
  String    Msg;
  uint32_t  TSMsg;
};

PeerClass Module;
LinkedList<PeriphClass*> SwitchList = LinkedList<PeriphClass*>();
LinkedList<PeriphClass*> SensorList = LinkedList<PeriphClass*>();

struct_Status Status[MAX_STATUS];

u_int8_t broadcastAddressAll[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; 

volatile uint32_t TSButton = 0;
volatile uint32_t TSSend   = 0;
volatile uint32_t TSPair   = 0;
volatile uint32_t TSLed    = 0;
volatile uint32_t TSStatus = 0;

bool NameChanged = false;

Preferences preferences;
#pragma endregion Globals

#pragma region Functions
#ifdef ESP32 
    void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
#elif defined(ESP8266)
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
#endif
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
#ifdef ESP32 
    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status); 
#elif defined(ESP8266)
    void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);
#endif
void OnDataRecvCommon(const uint8_t * mac, const uint8_t *incomingData, int len);

void   InitModule();

float  ReadAmp (int A);
float  ReadVolt(int V);
void   SendMessage();
void   SendPairingRequest();

void   UpdateSwitches();

void   SetDemoMode (bool Mode);
void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);
void   SetPairMode(bool Mode);

void   SaveModule();
void   AddStatus(String Msg);
void   PrintMAC(const uint8_t * mac_addr);
void   GoToSleep();
void   SetMessageLED(int Color);
void   LEDBlink(int Color, int n, uint8_t ms);

#pragma endregion Functions

void InitModule()
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
    */

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
        Module.PeriphSetup(0, "Sw 1",  SENS_TYPE_SWITCH,   0,  4,     0,       0,        0,    0);
        Module.PeriphSetup(1, "Sw 2",  SENS_TYPE_SWITCH,   0,  3,     0,       0,        0,    0);
    #endif
    #ifdef ESP32_MODULE_4S_4A_1V_ADS_PORT   // Mixed-Module with ADC and Port and VMon ###########################################################
      // TERMINATOR_PRO (untested) - 4 sensed switches with acs712(30A) over ADS and Port, Voltage-Monitor:??
      #define SWITCHES_PER_SCREEN 
      //                Name        Type         Version  Address   sleep  debug  demo   pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, PDC_SENSOR_MIX, _Version, NULL,     false, true,  false, false, 1,  RELAY_NORMAL, 6,  7,     1.5);
      //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
      Module.PeriphSetup(0, "Amp 1",  SENS_TYPE_AMP,     1,  0,   2.5,     0.066,    0,    0);
      Module.PeriphSetup(1, "Amp 2",  SENS_TYPE_AMP,     1,  1,   2.5,     0.066,    0,    0);
      Module.PeriphSetup(2, "Amp 3",  SENS_TYPE_AMP,     1,  2,   2.5,     0.066,    0,    0);
      Module.PeriphSetup(3, "Amp 4",  SENS_TYPE_AMP,     1,  3,   2.5,     0.066,    0,    0);
      Module.PeriphSetup(0, "Sw 1",   SENS_TYPE_SWITCH,  1,  0,   0,       0,        0,    0);
      Module.PeriphSetup(1, "Sw 2",   SENS_TYPE_SWITCH,  1,  1,   0,       0    ,    0,    0);
      Module.PeriphSetup(2, "Sw 3",   SENS_TYPE_SWITCH,  1,  2,   0,       0,        0,    0);
      Module.PeriphSetup(3, "Sw 4 ",  SENS_TYPE_SWITCH,  1,  3,   0,       0,        0,    0);
      Module.PeriphSetup(4, "V-Sens", SENS_TYPE_VOLT,    0,  35,  0,       0,      200,    0); 
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
    //works
    #ifdef ESP8266_MODULE_4A_1V_ADS           // 4-way Battery-Sensor with ADS and VMon #########################################################
      // 4x acs712(30A) over ADC1115, Voltage-Monitor:A0
      #define SWITCHES_PER_SCREEN 4
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, BATTERY_SENSOR, _Version, NULL,     false, true, false, false, 4,  RELAY_NORMAL, 14,  12,     1);
      //                      Name     Type            ADS  IO   NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Amp_1",   SENS_TYPE_AMP,  1,    0,  1.65,  0.066,  0,    0);
      Module.PeriphSetup(1, "Amp_2",   SENS_TYPE_AMP,  1,    1,  1.65,  0.066,  0,    0);
      Module.PeriphSetup(2, "Amp_3",   SENS_TYPE_AMP,  1,    2,  1.65,  0.066,  0,    0);
      Module.PeriphSetup(3, "Amp_4",   SENS_TYPE_AMP,  1,    3,  1.65,  0.066,  0,    0);
      Module.PeriphSetup(4, "VMon",    SENS_TYPE_VOLT, 0,   A0,   0,      0,   310,   0);  // 8266: 310 = 1023/3.3v
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
void setup()
{
    Wire.begin(SDA_PIN, SCL_PIN);

    #ifdef ARDUINO_USB_CDC_ON_BOOT
        delay(3000);
    #endif
    
    #ifdef ESP32
        Serial.begin(460800);
    #elif defined(ESP8266)
        Serial.begin(115200);
    #endif
    
    pinMode(LED_PIN, OUTPUT);

    #ifdef PAIRING_BUTTON
        pinMode(PAIRING_BUTTON, INPUT_PULLUP); 
    #endif
    
    LEDBlink(3, 3, 100);
    
    if (DEBUG_LEVEL > 0)                        // Show free entries
    {
        preferences.begin("JeepifyInit", true);
            Serial.printf("free entries in JeepifyInit now: %d\n\r", preferences.freeEntries());
        preferences.end();
        preferences.begin("JeepifyPeers", true);
            Serial.printf("free entries in JeepifyPeers now: %d\n\r", preferences.freeEntries());
        preferences.end();
    }
    
    #ifdef ESP32_DISPLAY_480
        smartdisplay_init();

        __attribute__((unused)) auto disp = lv_disp_get_default();
        lv_disp_set_rotation(disp, LV_DISP_ROT_90);
    #endif

    InitModule();
    LEDBlink(3, 1, 100);

    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) // Set pinModes
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
                #ifndef ADS_USED
                    pinMode(Module.GetPeriphIOPort(SNr), INPUT );
                #endif
                break;
        }
    }

    #ifdef MRD_USED                             // MultiReset-Check
        mrd = new MultiResetDetector(MRD_TIMEOUT, MRD_ADDRESS);

        if (mrd->detectMultiReset()) {
          Serial.println("Multi Reset Detected");
          digitalWrite(LED_BUILTIN, LED_ON);
          //ClearPeers(); ClearInit(); InitModule(); SaveModule();
          Module.SetPairMode(true); TSPair = millis();
        }
        else {
          Serial.println("No Multi Reset Detected");
          digitalWrite(LED_BUILTIN, LED_OFF);
        }
    #endif
    #ifdef PORT_USED                            // init IOBoard
	      if (!IOBoard.begin())
          {
                if (DEBUG_LEVEL > 0) Serial.println("IOBoard not found!");
                while (1);
          }
          else 
          {
                if (DEBUG_LEVEL > 1) Serial.println("IOBoard initialised.");
          }
    #endif
    #ifdef ADS_USED                             // init ADS
        ADSBoard.setGain(GAIN_TWOTHIRDS);   // 0.1875 mV/Bit .... +- 6,144V
        if (!ADSBoard.begin(ADS_ADDRESS)) { 
          if (DEBUG_LEVEL > 0) Serial.println("ADS not found!");
          while (1);
        }
        else
        {
            if (DEBUG_LEVEL > 1) Serial.println("ADS initialised.");
        }
    #endif
    if (preferences.begin("JeepifyInit", true)) // import saved Module... if available
    {
        String SavedModule   = preferences.getString("Module", "");
            if (DEBUG_LEVEL > 2) Serial.printf("Importiere Modul: %s", SavedModule.c_str());
            char ToImport[250];
            strcpy(ToImport,SavedModule.c_str());
            if (strcmp(ToImport, "") != 0) Module.Import(ToImport);
        preferences.end();
    }
    UpdateSwitches();

    WiFi.mode(WIFI_STA);
    uint8_t MacTemp[6];
    WiFi.macAddress(MacTemp);
    Module.SetBroadcastAddress(MacTemp);

    if (esp_now_init() != 0) 
        if (DEBUG_LEVEL > 0) Serial.println("Error initializing ESP-NOW");
    #ifdef ESP8266
        esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    #endif 
    
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);    

    AddStatus("Init Module");
    
    #ifdef KILL_NVS
        nvs_flash_erase(); nvs_flash_init(); ESP.restart();
    #endif

    if (GetPeers() == 0)                        // Pairmode if no known peers
    {
        Module.SetPairMode(true); 
        TSPair = millis();    
        AddStatus("PairMode on");
    }   

    AddStatus("Get Peers");

    if (DEBUG_LEVEL > 1) ReportAll();    
    
    RegisterPeers();  
    AddStatus("Init fertig");
  
    Module.SetLastContact(millis());
    
    #ifdef ESP32_DISPLAY_480
      ui_init();
    #endif
}
#pragma region Send-Things
void SendMessage () 
{
    //sendet NAME0:Value0, NAME1:Value1... Status:(bitwise)int
    TSLed = millis();
    SetMessageLED(2);

    JsonDocument doc; String jsondata; 
    char buf[100]; 

    doc["Node"] = Module.GetName();   

    for (int SNr=0; SNr<MAX_PERIPHERALS ; SNr++) 
    {
        if (Module.isPeriphEmpty(SNr) == false)
        {
            //temp
            Module.SetPeriphChanged(SNr, true);
            //SWITCH
            if (Module.GetPeriphType(SNr) == SENS_TYPE_SWITCH) 
            {
                dtostrf(Module.GetPeriphValue(SNr), 0, 0, buf);
            }
            //AMP
            else if (Module.GetPeriphType(SNr) == SENS_TYPE_AMP) 
            {
              if (Module.GetDemoMode()) Module.SetPeriphValue(SNr, (float) random(0,300)/10);
              else                      Module.SetPeriphValue(SNr, ReadAmp(SNr));
              
              if (abs(Module.GetPeriphValue(SNr)) > 99) Module.SetPeriphValue(SNr, -99);
              dtostrf(Module.GetPeriphValue(SNr), 0, 2, buf);
              
              if (Module.GetPeriphChanged(SNr))
              {
                  Module.SetPeriphOldValue(SNr, Module.GetPeriphValue(SNr));
                  Module.SetPeriphChanged(SNr, true);
              }
              else {
                  Module.SetPeriphOldValue(SNr, Module.GetPeriphValue(SNr));
                  Module.SetPeriphChanged(SNr, false);
              }
            }
            //VOLT
            else if (Module.GetPeriphType(SNr) == SENS_TYPE_VOLT) {
              if (Module.GetDemoMode()) Module.SetPeriphValue(SNr, (float) random(90,150)/10);
              else                      Module.SetPeriphValue(SNr, ReadVolt(SNr));

              dtostrf(Module.GetPeriphValue(SNr), 0, 2, buf);
              
              if (Module.GetPeriphChanged(SNr)) {
                  Module.SetPeriphOldValue(SNr, Module.GetPeriphValue(SNr));
                  Module.SetPeriphChanged(SNr, true);
              }
              else {
                  Module.SetPeriphOldValue(SNr, Module.GetPeriphValue(SNr));
                  Module.SetPeriphChanged(SNr, false);
              }
              doc["Vin"] = Module.GetPeriphVin(SNr);
            }
            doc[Module.GetPeriphName(SNr)] = buf;
            //if (DEBUG_LEVEL > 2) Serial.printf("doc[%s] = %s, ", Module.GetPeriphName(SNr), buf);
        }
        //if (DEBUG_LEVEL > 2) Serial.println();
    }
  
    // Status bit1 DebugMode, bit2 Sleep, bit3 Demo, bit4 RTP
    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);
    
    doc["Status"]  = Status;

    serializeJson(doc, jsondata);  

    //if (DEBUG_LEVEL > 2) Serial.println(jsondata);

    for (int PNr=0; PNr<PeerList.size(); PNr++) 
    {
        PeerClass *Peer = PeerList.get(PNr);

        if (Peer->GetType() >= MONITOR_ROUND)
        {
            if (DEBUG_LEVEL > 2) Serial.printf("Sending to: %s ", Peer->GetName()); 
            
            if (esp_now_send(Peer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 200) == 0) 
            {
                    if (DEBUG_LEVEL > 2) Serial.println("ESP_OK");  //Sending "jsondata" 
            } 
            else 
            {
                    if (DEBUG_LEVEL > 0) Serial.println("ESP_ERROR"); 
            }     
            
            if (DEBUG_LEVEL > 2) Serial.println(jsondata);
        }
    }

  //AddStatus("SendStatus");
}
void SendPairingRequest() 
{
  // sendet auf Broadcast: "addme", T0:Type, N0:Name, T1:Type, N1:Name...
  digitalWrite(LED_PIN, LED_ON); delay(100); digitalWrite(LED_PIN, LED_OFF);

  TSLed = millis();
  SetMessageLED(3);
  
  JsonDocument doc; String jsondata; 
  char Buf[100] = {};

  doc["Node"]    = Module.GetName();   
  doc["Type"]    = Module.GetType();
  doc["Version"] = Module.GetVersion();
  doc["Order"]   = SEND_CMD_PAIR_ME;
  
  for (int SNr=0 ; SNr<MAX_PERIPHERALS; SNr++) {
    if (!Module.isPeriphEmpty(SNr)) {
      snprintf(Buf, sizeof(Buf), "T%d", SNr); 
      doc[Buf] =Module.GetPeriphType(SNr);
      snprintf(Buf, sizeof(Buf), "N%d", SNr); 
      doc[Buf] = Module.GetPeriphName(SNr);
    }
  }
  serializeJson(doc, jsondata);  

  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  
  
  if (DEBUG_LEVEL > 2) Serial.printf("\nSending: %s\n\r", jsondata.c_str()); 
  
  AddStatus("Send Pairing request...");                                     
}
void SendNameChange(int Pos)
{
    // sendet auf Broadcast: "Order"="UpdateName"; "Pos"="32; "NewName"="Horst"; Pos==99 is ModuleName
  
  TSLed = millis();
  SetMessageLED(4);
  
  JsonDocument doc; String jsondata; 
  
  doc["Node"]    = Module.GetName();   
  doc["Order"]   = SEND_CMD_UPDATE_NAME;
  doc["Pos"]     = Pos;

  //ModuleName (99) or PeriphName(1-...);
  if (Pos == 99) doc["NewName"] = Module.GetName();
  else           doc["NewName"] = Module.GetPeriphName(Pos);
  
  serializeJson(doc, jsondata);  

  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  //Sending "jsondata"  
  
  if (DEBUG_LEVEL > 2) Serial.printf("\nSending: %s\n\r", jsondata.c_str()); 
  
  AddStatus("Send NameChange announce...");        
}
#pragma endregion Send-Things
#pragma region System-Things
void ChangeBrightness(int B)
{
    #ifdef ESP32_DISPLAY_480
        Module.SetBrightness(B);
        smartdisplay_lcd_set_backlight((float) B/100);
        SaveModule();
    #endif
}
void SetDemoMode(bool Mode) 
{
    Module.SetDemoMode(Mode);
    SaveModule();
}
void SetSleepMode(bool Mode) 
{
    Module.SetSleepMode(Mode);
    SaveModule();
}
void SetDebugMode(bool Mode) 
{
    Module.SetDebugMode(Mode);
    SaveModule();
}
void SetPairMode(bool Mode) 
{
    TSPair = millis();
    SetMessageLED(1);
    Module.SetPairMode(Mode);
}
void AddStatus(String Msg) 
{
  /*for (int Si=MAX_STATUS-1; Si>0; Si--) {
    Status[Si].Msg   = Status[Si-1].Msg;
    Status[Si].TSMsg = Status[Si-1].TSMsg;
  }
  Status[0].Msg = Msg;
  Status[0].TSMsg = millis();
  */
}
void ToggleSwitch(int SNr)
{
    int Value = Module.GetPeriphValue(SNr);
    
    if (Value == 0) Value = 1;
    else Value = 0;

    Module.SetPeriphValue(SNr, Value);
    
    UpdateSwitches();
}
void UpdateSwitches() 
{
  for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) 
  {
      if (Module.GetPeriphType(SNr) == SENS_TYPE_SWITCH) 
      {
          uint8_t Value = (uint8_t)Module.GetPeriphValue(SNr);
          if (DEBUG_LEVEL > 1) Serial.printf("Value %d = %f",SNr, (float)Value);
          //Serial.println(Value);
          if (Module.GetRelayType() == RELAY_REVERSED) 
          {
              if (Value == 0) Value = 1;
              else Value = 0;
          }
          
          #ifdef PORT_USED
              IOBoard.digitalWrite(SNr, Value);
              /*if (Value == 1)   
                switch (Module.GetPeriphIOPort(SNr))
                {
                    case 0: IOBoard.digitalWrite(P0,  HIGH); break;
                    case 1: IOBoard.digitalWrite(P1,  HIGH); break;
                    case 2: IOBoard.digitalWrite(P2,  HIGH); break;
                    case 3: IOBoard.digitalWrite(P3,  HIGH); break;
                    case 4: IOBoard.digitalWrite(P4,  HIGH); break;
                    case 5: IOBoard.digitalWrite(P5,  HIGH); break;
                    case 6: IOBoard.digitalWrite(P6,  HIGH); break;
                    case 7: IOBoard.digitalWrite(P7,  HIGH); break;
                }
              else
                switch (Module.GetPeriphIOPort(SNr))
                {
                    case 0: IOBoard.digitalWrite(P0,  LOW); break;
                    case 1: IOBoard.digitalWrite(P1,  LOW); break;
                    case 2: IOBoard.digitalWrite(P2,  LOW); break;
                    case 3: IOBoard.digitalWrite(P3,  LOW); break;
                    case 4: IOBoard.digitalWrite(P4,  LOW); break;
                    case 5: IOBoard.digitalWrite(P5,  LOW); break;
                    case 6: IOBoard.digitalWrite(P6,  LOW); break;
                    case 7: IOBoard.digitalWrite(P7,  LOW); break;
                }
            */
              
          #else
              digitalWrite(Module.GetPeriphIOPort(SNr), Value);
              /*if (Value == 1) digitalWrite(Module.GetPeriphIOPort(SNr), HIGH);
              else digitalWrite(Module.GetPeriphIOPort(SNr), LOW);
              */
          #endif

          if (DEBUG_LEVEL > 2) Serial.printf("Setze %s (Port:%d) auf %d", Module.GetPeriphName(SNr), Module.GetPeriphIOPort(SNr), Serial.print(Value));
      }
  }
  SendMessage();
}
void PrintMAC(const uint8_t * mac_addr)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}
void GoToSleep() 
{
    JsonDocument doc;
    String jsondata;
    
    doc["Node"] = Module.GetName();   
    doc["Type"] = Module.GetType();
    doc["Msg"]  = "GoodBye - going to sleep";
    
    serializeJson(doc, jsondata);  

    esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  //Sending "jsondata"  
    
    if (DEBUG_LEVEL > 2) Serial.printf("\nSending: %s", jsondata.c_str());
    AddStatus("Send Going to sleep..."); 
    
    if (DEBUG_LEVEL > 1) 
    {
        Serial.printf("Going to sleep at: %lu", millis());
        Serial.printf("LastContact    at: %u", Module.GetLastContact()); 
    }
    
    #ifdef ESP32
    gpio_deep_sleep_hold_en();
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) if (Module.GetPeriphType(SNr) == SENS_TYPE_SWITCH) gpio_hold_en((gpio_num_t)Module.GetPeriphIOPort(SNr));  
    
    esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL * 1000);
    esp_deep_sleep_start();
    #endif

}
void SaveModule()
{
    preferences.begin("JeepifyInit", false);
        String ExportStringPeer = Module.Export();
        int PutStringReturn = preferences.putString("Module", ExportStringPeer);

        if (DEBUG_LEVEL > 2) 
        {
            Serial.printf("SaveModule(): putString = %d, writing: %s\n\r", PutStringReturn, ExportStringPeer.c_str());
            if (DEBUG_LEVEL > 2) Serial.printf("Testread Module: %s", preferences.getString("Module", "").c_str());
        }
    preferences.end();
}

void GetModule()
{
    preferences.begin("JeepifyInit", true);
        String ImportStringPeer = "";
        
        ImportStringPeer = preferences.getString("Module", "");

        if (DEBUG_LEVEL > 2) 
        {
            Serial.printf("GetModule(): getString = %s\n\r", ImportStringPeer.c_str());
        }
        
        char ToImport[250];
        strcpy(ToImport,ImportStringPeer.c_str());
        Serial.printf("ToImport = %s\r\n", ToImport);
        
        if (strcmp(ToImport, "") != 0) Module.Import(ToImport);
        
        Serial.printf("Module.Vin[4] = %.2f", Module.GetPeriphVin(4));

    preferences.end();
}
void SetMessageLED(int Color)
{
    // 0-off, 1-Red, 2-Green, 3-Blue, 4=violett
    if (_LED_SIGNAL) 
    switch (Color)
    {
        case 0: 
            #ifdef ESP32_DISPLAY_480
                smartdisplay_led_set_rgb(0, 0, 0);
            #else
                digitalWrite(LED_PIN, LED_OFF);
            #endif
            break;
        case 1:
            #ifdef ESP32_DISPLAY_480
                smartdisplay_led_set_rgb(1, 0, 0);
            #else
                digitalWrite(LED_PIN, LED_ON);
            #endif
            break;
        case 2:
            #ifdef ESP32_DISPLAY_480
                smartdisplay_led_set_rgb(0, 1, 0);
            #else
            #endif
                digitalWrite(LED_PIN, LED_ON);
            break;
        case 3:
            #ifdef ESP32_DISPLAY_480
                smartdisplay_led_set_rgb(0, 0, 1);
            #else
                digitalWrite(LED_PIN, LED_ON);
            #endif
            break;
        case 4:
            #ifdef ESP32_DISPLAY_480
                smartdisplay_led_set_rgb(1, 0, 1);
            #else
                digitalWrite(LED_PIN, LED_ON);
            #endif
            break;  
  }
}
void LEDBlink(int Color, int n, uint8_t ms)
{
    for (int i=0; i<n; i++)
    {
        SetMessageLED(Color);
        delay(ms);
        SetMessageLED(0);
        delay(ms);
    }
}
#pragma endregion System-Things
#pragma region Data-Things
void VoltageCalibration(int SNr, float V) 
{
    char Buf[100] = {}; 
  
    if (DEBUG_LEVEL > 1) Serial.printf("Volt-Messung kalibrieren... Port: %d, Type:%d", Module.GetPeriphIOPort(SNr), Module.GetPeriphType(SNr));
    
    if (Module.GetPeriphType(SNr) == SENS_TYPE_VOLT) {
        float TempRead = 0;
        float NewVin = 0;

        for (int i=0; i<20; i++) 
        {
            TempRead += (float)analogRead(Module.GetPeriphIOPort(SNr));
            delay(10);
        }
        TempRead = (float) TempRead / 20;
        
        Serial.printf("TempRead nach filter = %.2f\n\r", TempRead);
        Serial.printf("Eich-soll Volt: %.2f\n\r", V);
       
        NewVin = TempRead / V;
        Module.SetPeriphVin(SNr, NewVin);        
        Serial.printf("NewVin = %.2f\n\r", Module.GetPeriphVin(SNr));
        
        if (DEBUG_LEVEL > 2)  Serial.printf("S[%d].Vin = %.2f - volt after calibration: %.2fV", SNr, Module.GetPeriphVin(SNr), TempRead/Module.GetPeriphVin(SNr));
        if (DEBUG_LEVEL > 1)  
        {
            snprintf(Buf, sizeof(Buf), "[%d] %s (Type: %d): Spannung ist jetzt: %.2fV", SNr, Module.GetPeriphName(SNr), Module.GetPeriphType(SNr), (float)TempRead/Module.GetPeriphVin(SNr));
            AddStatus(Buf);
        }

        SaveModule();
    }
}
void CurrentCalibration() 
{
    char Buf[100] = {};
    
    for(int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
      if (Module.GetPeriphType(SNr) == SENS_TYPE_AMP) {
        float TempVolt = 0;
        int   MaxValue = 1023;

        #ifdef ESP32
            MaxValue = 4095;
        #endif    
        
        #ifdef ADS_USED
            TempVolt = ADSBoard.computeVolts(ADSBoard.readADC_SingleEnded(Module.GetPeriphIOPort(SNr)));
        #else
            int TempVal = 0;
            for (int i=0; i<20; i++) 
            {
                TempVal += analogRead(Module.GetPeriphIOPort(SNr));
                delay(10);
            }
            TempVal /= 20;
            
            TempVolt = 3.3/MaxValue*TempVal; // 1.5???  
        #endif

        if (DEBUG_LEVEL > 2) { 
          Serial.print("TempVolt: "); Serial.println(TempVolt);
        }
        Module.SetPeriphNullwert(SNr, TempVolt);

        if (DEBUG_LEVEL > 1)  snprintf(Buf, sizeof(Buf), "Eichen fertig: [%d] %s (Type: %d): Gemessene Spannung bei Null: %.2fV", 
                                       SNr, Module.GetPeriphName(SNr), Module.GetPeriphType(SNr), TempVolt);

        AddStatus(Buf);
      }
    }
    SaveModule();
}
float ReadAmp (int SNr) 
{
    float TempVal      = 0;
    float TempVolt     = 0;
    float TempAmp      = 0;
  
    #ifdef ADS_USED
        TempVal  = ADSBoard.readADC_SingleEnded(Module.GetPeriphIOPort(SNr));
        TempVolt = ADSBoard.computeVolts(TempVal); 
        TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr);
        delay(10);
    #else
        TempVal  = analogRead(Module.GetPeriphIOPort(SNr));
        TempVolt = 3.3/4095*TempVal;
        TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr) * Module.GetVoltageDevider();// 1.5 wegen Voltage-Devider
        delay(10);
    #endif
  
    if (DEBUG_LEVEL > 2) {
        Serial.printf("(A): Raw:%.3f Null:%.3f --> %.2fV --> %.2fA\n\r", TempVal, Module.GetPeriphNullwert(SNr), TempVolt, TempAmp);
    } 
    if (abs(TempAmp) < SCHWELLE) TempAmp = 0;
    
    return (TempAmp); 
}
float ReadVolt(int SNr) 
{
    if (!Module.GetPeriphVin(SNr)) { Serial.println("Vin must not be zero !!!"); return 0; }
    
    //Serial.printf("PeriphVin(%d) = %d", SNr, Module.GetPeriphVin(SNr));

    float TempVal  = analogRead(Module.GetPeriphIOPort(SNr));
    float TempVolt = (float) TempVal / Module.GetPeriphVin(SNr);

    if (DEBUG_LEVEL > 2) {
        Serial.printf("(V) Raw: %.1f - Vin:%.2f --> %.2fV\n\r", TempVal, Module.GetPeriphVin(SNr), TempVolt);
    } 
    return TempVolt;
}
#pragma endregion Data-Things
#pragma region ESP-Things
void OnDataRecvCommon(const uint8_t * mac, const uint8_t *incomingData, int len)  
{  
  char* buff = (char*) incomingData;        //char buffer
  JsonDocument doc;
  String jsondata;
  int Pos = -1;
  float NewVperAmp = 0;
  float NewVin     = 0;
  float NewVoltage = 0;
  float NewNullwert = 0;
  String NewName = "";

  jsondata = String(buff);                  //converting into STRING
  
  if (DEBUG_LEVEL > 2) { Serial.print("Recieved from: "); PrintMAC(mac); }
  
  DeserializationError error = deserializeJson(doc, jsondata);
  
  if (!error) {
      String TempName = doc["Node"];
      if (DEBUG_LEVEL > 2) {
          Serial.print("("); Serial.print(TempName); Serial.print(") - ");
          Serial.println(jsondata);    
      }
      
      if (((int)doc["Order"] == SEND_CMD_YOU_ARE_PAIRED) and (doc["Peer"] == Module.GetName())) 
      { 
          //Serial.println("in you are paired und node");
        
          bool exists = esp_now_is_peer_exist((u8 *) mac);
          if (exists) 
          { 
            PrintMAC(mac); Serial.println(" already exists...");
          }
          else 
          {
            PeerClass *Peer = new PeerClass;
                Peer->Setup(doc["Node"], (int) doc["Type"], "xxx", mac, false, false, false, false);
                Peer->SetTSLastSeen(millis());
                PeerList.add(Peer);

                SavePeers();
                RegisterPeers();
                
                if (DEBUG_LEVEL > 1) {
                  Serial.printf("New Peer added: %s (Type:%d), MAC:", Peer->GetName(), Peer->GetType());
                  PrintMAC(Peer->GetBroadcastAddress());
                  Serial.println("\n\rSaving Peers after received new one...");
                  ReportAll();
                }
                Module.SetPairMode(false);
            }
      }
      switch ((int) doc["Order"]) 
      {
        case SEND_CMD_STAY_ALIVE: 
            Module.SetLastContact(millis());
            if (DEBUG_LEVEL > 2) { Serial.print("LastContact: "); Serial.println(Module.GetLastContact()); }
            break;
        case SEND_CMD_SLEEPMODE_ON:
            AddStatus("Sleep: on");  
            SetSleepMode(true);  
            SendMessage(); 
            break;
        case SEND_CMD_SLEEPMODE_OFF:
            AddStatus("Sleep: off"); 
            SetSleepMode(false); 
            SendMessage(); 
            break;
        case SEND_CMD_SLEEPMODE_TOGGLE:
            if (Module.GetSleepMode()) 
            { 
                AddStatus("Sleep: off");   
                SetSleepMode(false); 
                SendMessage(); 
            }
            else 
            { 
                AddStatus("Sleep: on");    
                SetSleepMode(true);  
                SendMessage(); 
            }
            break;
        case SEND_CMD_DEBUGMODE_ON:
            AddStatus("DebugMode: on");  
            SetDebugMode(true);  
            SaveModule();
            SendMessage(); 
            break;
        case SEND_CMD_DEBUGMODE_OFF:
            AddStatus("DebugMode: off"); 
            SetDebugMode(false); 
            SaveModule();
            SendMessage(); 
            break;
        case SEND_CMD_DEBUGMODE_TOGGLE:
            if (Module.GetDebugMode()) 
            {   
                AddStatus("DebugMode: off");   
                SetDebugMode(false); 
                SendMessage(); 
            }
            else 
            { 
                AddStatus("DebugMode: on");    
                SetDebugMode(true);  
                SendMessage(); 
            }
            break;
        case SEND_CMD_DEMOMODE_ON:
            AddStatus("Demo: on");   
            SetDemoMode(true);   
            SendMessage(); 
            break;
        case SEND_CMD_DEMOMODE_OFF:
            AddStatus("Demo: off");  
            SetDemoMode(false);  
            SendMessage(); 
            break;
        case SEND_CMD_DEMOMODE_TOGGLE:
            if (Module.GetDemoMode()) 
            { 
                AddStatus("DemoMode: off"); 
                SetDemoMode(false); 
                SendMessage(); 
            }
            else 
            { 
                AddStatus("DemoMode: on");  
                SetDemoMode(true);  
                SendMessage(); 
            }
            break;
        case SEND_CMD_RESET:
            AddStatus("Clear all"); 
            #ifdef ESP32
                nvs_flash_erase(); nvs_flash_init();
            #elif defined(ESP8266)
                ClearPeers(); ClearInit();
            #endif
            ESP.restart();
            break;
        case SEND_CMD_RESTART:
            ESP.restart(); 
            break;
        case SEND_CMD_PAIRMODE_ON:
            Module.SetPairMode(true);
            AddStatus("Pairing beginnt"); 
            SendMessage(); 
            #ifdef ESP32_DISPLAY_480
              smartdisplay_led_set_rgb(1,0,0);
            #endif
            break;
        case SEND_CMD_CURRENT_CALIB:
            AddStatus("Eichen beginnt"); 
            CurrentCalibration();
            break;
        case SEND_CMD_VOLTAGE_CALIB:
            AddStatus("VoltCalib beginnt");
            NewVoltage = (float) doc["NewVoltage"];

            VoltageCalibration(Module.GetVoltageMon(), NewVoltage) ;
            
            break;
        case SEND_CMD_SWITCH_TOGGLE:
            Pos = doc["Pos"];
            if (Module.isPeriphEmpty(Pos) == false) ToggleSwitch(Pos);
            break;
        case SEND_CMD_UPDATE_NAME:
            Pos = (int) doc["Pos"];
            NewName = doc["NewName"].as<String>();

            if (NewName != "") 
            {
                if (Pos == 99) Module.SetName(NewName.c_str());
                else           Module.SetPeriphName(Pos, NewName.c_str());
            }
            
            SaveModule();
            NameChanged = true;
            //SendNameChange(Pos);
            break;
        case SEND_CMD_UPDATE_VIN:
            NewVin = (float) doc["Value"];
            Pos = (int) doc["Pos"];

            if (NewVin > 0)
            {
                Module.SetPeriphVin(Pos, NewVin);
                SaveModule();
            }
            break;
        case SEND_CMD_UPDATE_VPERAMP:
            Pos = (int) doc["Pos"];
            NewVperAmp = (float) doc["Value"];

            if (NewVperAmp > 0)
            {
                Module.SetPeriphVperAmp(Pos, NewVperAmp);
                SaveModule();
            }
            break;
        case SEND_CMD_UPDATE_NULLWERT:
            Pos = (int) doc["Pos"];
            NewNullwert = (float) doc["Value"];

            if (NewNullwert > 0)
            {
                Module.SetPeriphNullwert(Pos, NewNullwert);
                SaveModule();
            }
            break;
      }
    } // end (!error)
    else // error
    { 
          if (DEBUG_LEVEL > 0) 
          {
              Serial.print(F("deserializeJson() failed: "));  //Just in case of an ERROR of ArduinoJSon
              Serial.println(error.f_str());
          }
    }
}
#ifdef ESP32 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    OnDataRecvCommon(mac, incomingData, len);
}
#elif defined(ESP8266)
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) 
{
    OnDataRecvCommon(mac, incomingData, len);
}
#endif
#ifdef ESP32 //void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{ 
    if (DEBUG_LEVEL > 2) 
        if (status == ESP_NOW_SEND_SUCCESS) Serial.println("\r\nLast Packet Send Status: Delivery Success");
        
    if (DEBUG_LEVEL > 0)  
        if (status != ESP_NOW_SEND_SUCCESS) Serial.println("\r\nLast Packet Send Status: Delivery Fail");
}
#elif defined(ESP8266)
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (DEBUG_LEVEL > 2) 
        if (sendStatus == 0) Serial.println("\r\nLast Packet Send Status: Delivery Success");
        
    if (DEBUG_LEVEL > 0)  
        if (sendStatus != 0) Serial.println("\r\nLast Packet Send Status: Delivery Fail");
}
#endif
#pragma endregion ESP-Things
void loop()
{
    if  ((millis() - TSSend ) > MSG_INTERVAL  )                                 // Send-interval (Message or Pairing-request)
    {
        TSSend = millis();
        if (Module.GetPairMode()) SendPairingRequest();
        else SendMessage();
    }

    if  (((millis() - TSStatus ) > STATUS_INTERVAL) or (NameChanged))          // Send status update (inclusive names)
    {
        TSStatus = millis();
        NameChanged = false;
        SendPairingRequest();
    }

    if (((millis() - TSPair ) > PAIR_INTERVAL ) and (Module.GetPairMode()))     // end Pairing after pairing interval
    {
        TSPair = 0;
        Module.SetPairMode(false);
        AddStatus("Pairing beendet...");
        SetMessageLED(0);
    }

    if ((millis() - TSLed > MSGLIGHT_INTERVAL) and (TSLed > 0))                 // clear LED after LED interval
    {
        TSLed = 0;
        
        if (Module.GetPairMode())
            SetMessageLED(1);
        else
            SetMessageLED(0);
    }

    #ifdef PAIRING_BUTTON                                                       // check for Pairing/Reset Button
        int BB = !digitalRead(PAIRING_BUTTON);
        if (BB == 1) {
            TSPair = millis();
            Module.SetPairMode(true);
            SetMessageLED(1);
    
            AddStatus("Pairing beginnt...");
            
            if (!TSButton) TSButton = millis();
            else 
            {
                if ((millis() - TSButton) > 3000) {
                    if (DEBUG_LEVEL > 1) Serial.println("Button pressed... Clearing Peers and Reset");
                    AddStatus("Clearing Peers and Reset");
                    #ifdef ESP32
                      nvs_flash_erase(); nvs_flash_init();
                    #elif defined(ESP8266)
                      ClearPeers();
                    #endif 
                    ESP.restart();
                }
            }
        }
        else TSButton = 0;
    #endif

    #ifdef ESP32_DISPLAY_480                                                    // start ui if module has display
        lv_timer_handler();
        delay(5);
    #endif
}