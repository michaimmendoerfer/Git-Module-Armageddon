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
const char _ModuleName[]        = "8266-Bat";
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

volatile uint32_t TSBootButton    = 0;
volatile uint32_t TSSend  = 0;
volatile uint32_t TSPair  = 0;
volatile uint32_t TSLed   = 0;

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
      Module.PeriphSetup(0, "Amp 1",  SENS_TYPE_AMP,     0,  34,   2.5,     0.166,    0,    0);
      Module.PeriphSetup(1, "Amp 2",  SENS_TYPE_AMP,     0,  35,   2.5,     0.166,    0,    0);
      Module.PeriphSetup(2, "Sw 1",   SENS_TYPE_SWITCH,  0,  32,   0,       0,        0,    0);
      Module.PeriphSetup(3, "Sw 2 ",  SENS_TYPE_SWITCH,  0,  33,   0,       0,        0,    0);
      Module.PeriphSetup(4, "V-Sens", SENS_TYPE_VOLT,    0,  39,   0,       0,      200,    0); 
    #endif
    #ifdef ESP32_MODULE_2S_PORT             // 2-Way Switch via IOBoard ##########################################################################
        #define SWITCHES_PER_SCREEN 2       
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon    RelayType     sda scl voltagedevier 
      Module.Setup(_ModuleName, SWITCH_2_WAY,   _Version, NULL,     false, true,  true, false, -1,  RELAY_REVERSED, 5,  6,     1.5);
      //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
      Module.PeriphSetup(0, "Sw 1",  SENS_TYPE_SWITCH,   1,  0,     0,       0,        0,    0);
      Module.PeriphSetup(1, "Sw 2",  SENS_TYPE_SWITCH,   1,  1,     0,       0,        0,    0);
    #endif
    #ifdef ESP32_MODULE_2A_2S_1V_ADS_PORT   // Mixed-Module with ADC and Port and VMon ###########################################################
      #define SWITCHES_PER_SCREEN 
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, PDC_SENSOR_MIX, _Version, NULL,     false, true,  true, false, 1,  RELAY_NORMAL, 14,  15,     1.5);
      //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
      Module.PeriphSetup(0, "Amp 1",  SENS_TYPE_AMP,     1,  0,   2.5,     0.166,    0,    0);
      Module.PeriphSetup(1, "Amp 2",  SENS_TYPE_AMP,     1,  1,   2.5,     0.166,    0,    0);
      Module.PeriphSetup(2, "Sw 1",   SENS_TYPE_SWITCH,  1,  0,   0,       0,        0,    0);
      Module.PeriphSetup(3, "Sw 2 ",  SENS_TYPE_SWITCH,  1,  1,   0,       0,        0,    0);
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
    #ifdef ESP8266_MODULE_4A_1V_ADS           // 4-way Battery-Sensor with ADS and VMon #########################################################
      // 4x acs712(30A) over ADC1115, Voltage-Monitor:A0
      #define SWITCHES_PER_SCREEN 4
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, BATTERY_SENSOR, _Version, NULL,     false, true, false, false, 1,  RELAY_NORMAL, 14,  12,     1.5);
      //                      Name     Type            ADS  IO   NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Amp_1",   SENS_TYPE_AMP,  1,    0,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(1, "Amp_2",   SENS_TYPE_AMP,  1,    1,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(2, "Amp_3",   SENS_TYPE_AMP,  1,    2,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(3, "Amp_4",   SENS_TYPE_AMP,  1,    3,  2.5,  0.066,  0,    0);
      Module.PeriphSetup(4, "VMon",    SENS_TYPE_VOLT, 0,   A0,   0,     0,   200,   0); 
    #endif
    #ifdef ESP8266_MODULE_4S_INTEGRATED         // 4-way Switch - 8266 onBoard +++++++ #########################################################
        #define SWITCHES_PER_SCREEN 4
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon   RelayType      sda    scl    voltagedevier 
      Module.Setup(_ModuleName, SWITCH_4_WAY,   _Version, NULL,     false, true,  true, false, -1,  RELAY_NORMAL,   -1,   -1,     1.5);
      //                      Name     Type             ADS  IO    NULL     VpA      Vin  PeerID
      Module.PeriphSetup(0, "Sw 1",   SENS_TYPE_SWITCH,   0,  5,    0,       0,       0,    0);
      Module.PeriphSetup(1, "Sw 2",   SENS_TYPE_SWITCH,   0,  13,    0,      0,       0,    0);
      Module.PeriphSetup(2, "Sw 3",   SENS_TYPE_SWITCH,   0,  14,    0,      0,       0,    0);
      Module.PeriphSetup(3, "Sw 4",   SENS_TYPE_SWITCH,   0,  15,    0,      0,       0,    0);
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
            }
            doc[Module.GetPeriphName(SNr)] = buf;
            if (DEBUG_LEVEL > 2) Serial.printf("doc[%s] = %s, ", Module.GetPeriphName(SNr), buf);
        }
        if (DEBUG_LEVEL > 2) Serial.println();
    }
  
    // Status bit1 DebugMode, bit2 Sleep, bit3 Demo, bit4 RTP
    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);
    
    doc["Status"]  = Status;

    serializeJson(doc, jsondata);  

    if (DEBUG_LEVEL > 2) Serial.println(jsondata);

    for (int PNr=0; PNr<PeerList.size(); PNr++) 
    {
        PeerClass *Peer = PeerList.get(PNr);

        if (Peer->GetType() >= MONITOR_ROUND)
        {
            if (DEBUG_LEVEL > 2) Serial.printf("Sending to: %s\n\r", Peer->GetName()); 
            
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
  preferences.begin("JeepifyInit", false);
    Module.SetBrightness(B);
    smartdisplay_lcd_set_backlight((float) B/100);
    SaveModule();
  preferences.end();
  #endif
}
void SetDemoMode(bool Mode) 
{
  preferences.begin("JeepifyInit", false);
    Module.SetDemoMode(Mode);
    SaveModule();
  preferences.end();
}
void SetSleepMode(bool Mode) 
{
  preferences.begin("JeepifyInit", false);
    Module.SetSleepMode(Mode);
    SaveModule();
  preferences.end();
}
void SetDebugMode(bool Mode) 
{
  preferences.begin("JeepifyInit", false);
    Module.SetDebugMode(Mode);
    SaveModule();
  preferences.end();
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
              if (Value == 1)   
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
              
          #else
              if (Value == 1) digitalWrite(Module.GetPeriphIOPort(SNr), HIGH);
              else digitalWrite(Module.GetPeriphIOPort(SNr), LOW);
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
        Serial.printf("Going to sleep at: %u", millis());
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

        if (DEBUG_LEVEL > 2) 
        {
            Serial.printf("putSring = %d", preferences.putString("Module", ExportStringPeer));
            Serial.printf("schreibe: Module: %s\n\r",ExportStringPeer.c_str());
        }
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
#pragma endregion System-Things
#pragma region Data-Things
void VoltageCalibration(int SNr, float V) 
{
    char Buf[100] = {}; 
  
    if (DEBUG_LEVEL > 1) Serial.println("Volt-Messung kalibrieren...");
    
    preferences.begin("JeepifyInit", false);
  
    if (Module.GetPeriphType(SNr) == SENS_TYPE_VOLT) {
        int TempRead = analogRead(Module.GetPeriphIOPort(SNr));
        
        Module.SetPeriphVin(SNr, TempRead / V);
        
        if (DEBUG_LEVEL > 2)  
        {
            Serial.print("S["); Serial.print(SNr); Serial.print("].Vin = ");
            Serial.println(Module.GetPeriphVin(SNr), 4);
            Serial.print("Volt(nachher) = ");
            Serial.println(TempRead/Module.GetPeriphVin(SNr), 4);
        }
        
        if (DEBUG_LEVEL > 1)  snprintf(Buf, sizeof(Buf), "[%d] %s (Type: %d): Spannung ist jetzt: %.2fV", SNr, Module.GetPeriphName(SNr), Module.GetPeriphType(SNr), (float)TempRead/Module.GetPeriphVin(SNr));
        
        AddStatus(Buf);

        SaveModule();
    }
}
void CurrentCalibration() 
{
    char Buf[100] = {};
    
    for(int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
      if (Module.GetPeriphType(SNr) == SENS_TYPE_AMP) {
        float TempVal  = 0;
        float TempVolt = 0;
        
        #ifdef ADS_USED
        TempVal  = ADSBoard.readADC_SingleEnded(Module.GetPeriphIOPort(SNr));
        TempVolt = ADSBoard.computeVolts(TempVal);
        #else
        //Filter implementieren !!!
        TempVal  = analogRead(Module.GetPeriphIOPort(SNr));
        TempVolt = 3.3/4095*TempVal; // 1.5???
        #endif    

        delay(10);  

        if (DEBUG_LEVEL > 2) { 
          Serial.print("TempVal:");     Serial.println(TempVal);
          Serial.print(", TempVolt: "); Serial.println(TempVolt);
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
        Serial.print("TempVal:  "); Serial.println(TempVal,4);
        Serial.print("TempVolt: "); Serial.println(TempVolt,4);
        Serial.print("Nullwert: "); Serial.println(Module.GetPeriphNullwert(SNr),4);
        Serial.print("VperAmp:  "); Serial.println(Module.GetPeriphVperAmp(SNr),4);
        Serial.print("Amp (TempVolt - S[Si].NullWert) / S[Si].VperAmp * 1.5:  "); Serial.println(TempAmp,4);
    } 
    if (abs(TempAmp) < SCHWELLE) TempAmp = 0;
    
    return (TempAmp); 
}
float ReadVolt(int SNr) 
{
  if (!Module.GetPeriphPtr(SNr)->GetVin()) { Serial.println("Vin must not be zero !!!"); return 0; }
  
  float TempVal  = analogRead(Module.GetPeriphIOPort(SNr));
  float TempVolt = TempVal / Module.GetPeriphVin(SNr);
  
  if (DEBUG_LEVEL > 2) {
    Serial.print("TempVal:  "); Serial.println(TempVal,4);
    Serial.print("Vin:      "); Serial.println(Module.GetPeriphVin(SNr));
    Serial.print("Volt (TempVal / S[V].Vin)): ");     Serial.println(TempVolt,4);
    
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
  float NewVoltage = 0;

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
                preferences.begin("JeepifyInit", false);
                preferences.clear();
                preferences.end();
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
            NewVoltage = doc["NewVoltage"];

            if (Module.GetVoltageMon() != -1)
            {
                VoltageCalibration(Module.GetVoltageMon(), NewVoltage) ;
            }
            break;
        case SEND_CMD_SWITCH_TOGGLE:
            Pos = doc["Pos"];
            if (Module.isPeriphEmpty(Pos) == false) ToggleSwitch(Pos);
            break;
        case SEND_CMD_UPDATE_NAME:
            Pos = (int) doc["Pos"];
            String NewName = doc["NewName"];

            if (NewName != "") 
            {
                if (Pos == 99) Module.SetName(NewName.c_str());
                else           Module.SetPeriphName(Pos, NewName.c_str());
            }
            
            SaveModule();
            SendNameChange(Pos);
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

void setup()
{
    //Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
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
    
    for (int i=0; i<3; i++)
    {
        digitalWrite(LED_PIN, LED_ON);
        delay(100);
        digitalWrite(LED_PIN, LED_OFF);
        delay(100);
    }
    Serial.println("Es geht los");

    #ifdef ESP32_DISPLAY_480
        smartdisplay_init();

        __attribute__((unused)) auto disp = lv_disp_get_default();
        lv_disp_set_rotation(disp, LV_DISP_ROT_90);
    #endif

    InitModule();
    digitalWrite(LED_PIN, LED_ON); delay(100); digitalWrite(LED_PIN, LED_OFF);

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
                #ifndef ADS_USED
                    pinMode(Module.GetPeriphIOPort(SNr), INPUT );
                #endif
                break;
        }
    }

    //Wire.begin(SDA_PIN, SCL_PIN);

    #ifdef MRD_USED //MRD
        mrd = new MultiResetDetector(MRD_TIMEOUT, MRD_ADDRESS);

        if (mrd->detectMultiReset()) {
          Serial.println("Multi Reset Detected");
          digitalWrite(LED_BUILTIN, LED_ON);
          ClearPeers();
          Module.SetPairMode(true); TSPair = millis();
        }
        else {
          Serial.println("No Multi Reset Detected");
          digitalWrite(LED_BUILTIN, LED_OFF);
        }
    #endif

    #ifdef PORT_USED
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
    #ifdef ADS_USED
        ADSBoard.setGain(GAIN_TWOTHIRDS);  // 0.1875 mV/Bit .... +- 6,144V
        if (!ADSBoard.begin(ADS_ADDRESS)) { 
          if (DEBUG_LEVEL > 0) Serial.println("ADS not found!");
          while (1);
        }
        else
        {
            if (DEBUG_LEVEL > 1) Serial.println("ADS initialised.");
        }
    #endif
    
    if (preferences.begin("JeepifyInit", true))
    {
        String SavedModule   = preferences.getString("Module", "");
            if (DEBUG_LEVEL > 2) Serial.printf("Importiere Modul: %s", SavedModule.c_str());
            char ToImport[250];
            strcpy(ToImport,SavedModule.c_str());
            if (strcmp(ToImport, "") != 0) Module.Import(ToImport);
        preferences.end();
    }

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

    GetPeers();       
    AddStatus("Get Peers");
    
    ReportAll();    
    
    RegisterPeers();  
    AddStatus("Init fertig");
  
    Module.SetLastContact(millis());

    #ifdef ESP32_DISPLAY_480
      ui_init();
    #endif
}
void loop()
{
    if  ((millis() - TSSend ) > MSG_INTERVAL  ) {
        TSSend = millis();
        if (Module.GetPairMode()) SendPairingRequest();
        else SendMessage();
    }

    if (((millis() - TSPair ) > PAIR_INTERVAL ) and (Module.GetPairMode())) {
        TSPair = 0;
        Module.SetPairMode(false);
        AddStatus("Pairing beendet...");
        SetMessageLED(0);
    }

    if ((millis() - TSLed > MSGLIGHT_INTERVAL) and (TSLed > 0))
    {
        TSLed = 0;
        
        if (Module.GetPairMode())
            SetMessageLED(1);
        else
            SetMessageLED(0);
    }

    #ifdef ESP32
        int BB = !digitalRead(BOOT_BUTTON);
        if (BB == 1) {
            TSPair = millis();
            Module.SetPairMode(true);
            SetMessageLED(1);
            AddStatus("Pairing beginnt...");
            
            if (!TSBootButton) TSBootButton = millis();
            else 
            {
                if ((millis() - TSBootButton) > 3000) {
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
        else TSBootButton = 0;
        #endif

    #ifdef ESP32_DISPLAY_480
        lv_timer_handler();
        delay(5);
    #endif
}