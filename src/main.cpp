//#define KILL_NVS 0

//#define MODULE_1S
//#define MODULE_2S
//#define MODULE_4A_1V_ADC
#define MODULE_4A_1V_NOADC
//#define MODULE_4S_1V_ADC
//#define MODULE_4S_1V_NOADC
//#define MODULE_2A_2S_1V_NOADC

#define DISPLAY_NO
//#define DISPLAY_C3_ROUND
//#define DISPLAY_480

#pragma region Includes
#include <Arduino.h>

#ifdef DISPLAY_480
  #include <esp32_smartdisplay.h>
#endif
#ifndef DISPLAY_NO
  #include <ui/ui.h>
#endif

#include <LinkedList.h>
#include <esp_now.h>
#include <WiFi.h>
#include <nvs_flash.h>
#include "Jeepify.h"
#include "PeerClass.h"
#include "pref_manager.h"
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Spi.h>

#pragma endregion Includes

const char _Version[]           = "3.01";
const char _Protokoll_Version[] = "1.01";
const char _ModuleName[]        = "C3-Arma";

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

#pragma region Functions
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void   InitModule();

float  ReadAmp (int A);
float  ReadVolt(int V);
void   SendMessage();
void   SendPairingRequest();

void   UpdateSwitches();

void   SetDemoMode (bool Mode);
void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);

void   AddStatus(String Msg);

void   PrintMAC(const uint8_t * mac_addr);

void   GoToSleep();
#pragma endregion Functions

void InitModule()
{
    /*
    ESP32 DevKit:
    possible Inputs:  4,13,16,17,18,19,21,22,23,25,26,27,(ADC1:32,33,34,35,36,39)
    possible Outputs: 4,13,16,17,18,19,21,22,23,25,26,27,32,33
    SPI-Flash:        6,7,8,9,10,11 (don´t use)
    ADC2 (when Wifi): 32,33,34,35,36,39

    */
    //uint8_t MacUId[7];
    
    #ifdef MODULE_4S_1V_NOADC   // 4-Way Switch with Voltage-Monitor #################################################################
      #define SWITCHES_PER_SCREEN 4

      //                Name        Type       Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, SWITCH_4_WAY, _Version, NULL,     false, true,  true, false, -1, RELAY_NORMAL, -1,  -1,     1);

      //                      Name     Type             ADS  IO  NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Extern", SENS_TYPE_SWITCH,  0,  25,   0,    0,    0,    0);
      Module.PeriphSetup(1, "In-Car", SENS_TYPE_SWITCH,  0,  26,   0,    0,    0,    0);
      Module.PeriphSetup(2, "Solar",  SENS_TYPE_SWITCH,  0,  32,   0,    0,    0,    0);
      Module.PeriphSetup(3, "Load",   SENS_TYPE_SWITCH,  0,  33,   0,    0,    0,    0);
      Module.PeriphSetup(4, "Lipo",   SENS_TYPE_VOLT,    0,  39,   0,    0,   200,   0); 
    #endif
    #ifdef MODULE_4S_1V_ADC     // 4-Way Switch no Voltage-Monitor ###################################################################
      #define SWITCHES_PER_SCREEN 4

      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, BATTERY_SENSOR, _Version, NULL,     false, true,  true, false, 1,  RELAY_NORMAL, -1,  -1,     1);

      //                      Name     Type             ADS  IO  NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Extern", SENS_TYPE_SWITCH,  0,  25,   0,    0,    0,    0);
      Module.PeriphSetup(1, "In-Car", SENS_TYPE_SWITCH,  0,  26,   0,    0,    0,    0);
      Module.PeriphSetup(2, "Solar",  SENS_TYPE_SWITCH,  0,  32,   0,    0,    0,    0);
      Module.PeriphSetup(3, "Load",   SENS_TYPE_SWITCH,  0,  33,   0,    0,    0,    0);
      Module.PeriphSetup(4, "Lipo",   SENS_TYPE_VOLT,    0,  39,   0,    0,   200,   0); 
    #endif
    #ifdef MODULE_1S            // 1-Way Switch ######################################################################################
      #define SWITCHES_PER_SCREEN 1
      //                Name        Type       Version  Address   sleep  debug  demo   pair  vMon RelayType      adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, SWITCH_1_WAY, _Version, NULL,     false, true,  false, false, -1, RELAY_REVERSED, -1,  -1,     1);

      //                      Name     Type             ADS    IO  NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Switch_1", SENS_TYPE_SWITCH,  0,  25,   0,    0,    0,    0);
    #endif
    #ifdef MODULE_2S            // 2-Way Switch ######################################################################################
      #define SWITCHES_PER_SCREEN 2

      //                Name        Type       Version  Address   sleep  debug  demo   pair  vMon RelayType      adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, SWITCH_2_WAY, _Version, NULL,     false, true,  false, false, -1, RELAY_REVERSED, -1,  -1,     1);
      
      //                      Name     Type             ADS    IO  NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Switch_1", SENS_TYPE_SWITCH,  0,  25,   0,    0,    0,    0);
      Module.PeriphSetup(0, "Switch_2", SENS_TYPE_SWITCH,  0,  26,   0,    0,    0,    0);
    #endif
    #ifdef MODULE_4A_1V_ADC     // 4-way Battery-Sensor with ADC and VMon ############################################################
      #define SWITCHES_PER_SCREEN 4
      
      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, BATTERY_SENSOR, _Version, NULL,     false, true,  true, false, 1,  RELAY_NORMAL, 14,  15,     1);

      //                      Name     Type             ADS  IO  NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Sensor_1", SENS_TYPE_AMP,  1,    1,   0,  0.066,  0,    0);
      Module.PeriphSetup(1, "Sensor_2", SENS_TYPE_AMP,  1,    2,   0,  0.066,  0,    0);
      Module.PeriphSetup(2, "Sensor_3", SENS_TYPE_AMP,  1,    3,   0,  0.066,  0,    0);
      Module.PeriphSetup(3, "Sensor_4", SENS_TYPE_AMP,  1,    4,   0,  0.066,  0,    0);
      Module.PeriphSetup(4, "VMon",     SENS_TYPE_VOLT, 0,   39,   0,    0,   200,   0); 
    #endif
    #ifdef MODULE_4A_1V_NOADC   // 4-way Battery-Sensor no ADC and VMon ##############################################################
      #define SWITCHES_PER_SCREEN 4

      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, BATTERY_SENSOR, _Version, NULL,     false, true,  true, false, 1,  RELAY_NORMAL, -1,  -1,     1.5);

      //                      Name     Type             ADS  IO  NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Amp 1",  SENS_TYPE_AMP,     0,  25,   0,    0,    0,    0);
      Module.PeriphSetup(1, "Amp 2",  SENS_TYPE_AMP,     0,  26,   0,    0,    0,    0);
      Module.PeriphSetup(2, "Amp 3",  SENS_TYPE_AMP,     0,  32,   0,    0,    0,    0);
      Module.PeriphSetup(3, "Amp 4",  SENS_TYPE_AMP,     0,  33,   0,    0,    0,    0);
      Module.PeriphSetup(4, "V-Sens", SENS_TYPE_VOLT,    0,  39,   0,    0,   200,   0); 
    #endif
    #ifdef MODULE_2A_2S_1V_NOADC   // 4-way Battery-Sensor no ADC and VMon ##############################################################
      #define SWITCHES_PER_SCREEN 2

      //                Name        Type         Version  Address   sleep  debug  demo  pair  vMon RelayType    adc1 adc2 voltagedevier 
      Module.Setup(_ModuleName, PDC_SENSOR_MIX, _Version, NULL,     false, true,  true, false, 1,  RELAY_NORMAL, -1,  -1,     1.5);

      //                      Name     Type             ADS  IO  NULL   VpA   Vin  PeerID
      Module.PeriphSetup(0, "Amp 1",  SENS_TYPE_AMP,     0,  25,   0,    0,    0,    0);
      Module.PeriphSetup(1, "Amp 2",  SENS_TYPE_AMP,     0,  26,   0,    0,    0,    0);
      Module.PeriphSetup(2, "Sw 1",   SENS_TYPE_SWITCH,  0,  32,   0,    0,    0,    0);
      Module.PeriphSetup(3, "Sw 2 ",  SENS_TYPE_SWITCH,  0,  33,   0,    0,    0,    0);
      Module.PeriphSetup(4, "V-Sens", SENS_TYPE_VOLT,    0,  39,   0,    0,   200,   0); 
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
    //digitalWrite(LED_BUILTIN, LED_ON);

    JsonDocument doc;; String jsondata; 
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
              if (Module.GetDemoMode()) Module.SetPeriphValue(SNr, random(0,30));
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
              if (Module.GetDemoMode()) Module.SetPeriphValue(SNr, random(10,15));
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
            Serial.printf("doc[%s] = %s, ", Module.GetPeriphName(SNr), buf);
        }
        Serial.println();
  }
  
  // Status bit1 DebugMode, bit2 Sleep, bit3 Demo, bit4 RTP
  int Status = 0;
  if (Module.GetDebugMode())   bitSet(Status, 0);
  if (Module.GetSleepMode())   bitSet(Status, 1);
  if (Module.GetDemoMode())    bitSet(Status, 2);
  if (Module.GetPairMode())    bitSet(Status, 3);
  
  doc["Status"]  = Status;

  serializeJson(doc, jsondata);  

  Serial.println(jsondata);

  for (int PNr=0; PNr<PeerList.size(); PNr++) 
  {
      PeerClass *Peer = PeerList.get(PNr);

      if (Peer->GetType() >= MONITOR_ROUND) {
      Serial.print("Sending to: "); Serial.print(Peer->GetName()); 
      Serial.println();
      if (esp_now_send(Peer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 200) == 0) Serial.println("ESP_OK");  //Sending "jsondata"  
      else Serial.println("ESP_ERROR"); 
      Serial.println(jsondata);
    }
  }

  //AddStatus("SendStatus");
}
void SendPairingRequest() 
{
  // sendet auf Broadcast: "addme", T0:Type, N0:Name, T1:Type, N1:Name...
  TSLed = millis();
  //digitalWrite(LED_BUILTIN, LED_ON);
  
  JsonDocument doc;; String jsondata; 
  char Buf[100] = {};
  char UIdStr[21];
  uint8_t *BrTemp;

  doc["Node"]    = Module.GetName();   
  doc["Type"]    = Module.GetType();
  doc["Version"] = Module.GetVersion();
  doc["Pairing"] = "add me";
  
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
  
  if (Module.GetDebugMode()) { Serial.print("\nSending: "); Serial.println(jsondata); }
  AddStatus("Send Pairing request...");                                     
}
void SendNameChange(int Pos)
{
    // sendet auf Broadcast: "Order"="UpdateName"; "Pos"="32; "NewName"="Horst"; Pos==99 is ModuleName
  
  TSLed = millis();
  
  JsonDocument doc;; String jsondata; 
  char Buf[100] = {};
  
  doc["Node"]    = Module.GetName();   
  doc["Order"]   = "UpdateName";
  doc["Pos"]     = Pos;

  //ModuleName (99) or PeriphName(1-...);
  if (Pos == 99) doc["NewName"] = Module.GetName();
  else           doc["NewName"] = Module.GetPeriphName(Pos);
  
  serializeJson(doc, jsondata);  

  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  //Sending "jsondata"  
  
  if (Module.GetDebugMode()) { Serial.print("\nSending: "); Serial.println(jsondata); }
  AddStatus("Send NameChange announce...");        
}
#pragma endregion Send-Things
#pragma region System-Things
void SetDemoMode(bool Mode) 
{
  preferences.begin("JeepifyInit", false);
    Module.SetDemoMode(Mode);
    if (preferences.getBool("DemoMode", false) != Module.GetDemoMode()) preferences.putBool("DemoMode", Module.GetDemoMode());
  preferences.end();
}
void SetSleepMode(bool Mode) 
{
  preferences.begin("JeepifyInit", false);
    Module.SetSleepMode(Mode);
    if (preferences.getBool("SleepMode", false) != Module.GetSleepMode()) preferences.putBool("SleepMode", Module.GetSleepMode());
  preferences.end();
}
void SetDebugMode(bool Mode) 
{
  preferences.begin("JeepifyInit", false);
    Module.SetDebugMode(Mode);
    if (preferences.getBool("DebugMode", false) != Module.GetDebugMode()) preferences.putBool("DebugMode", Module.GetDebugMode());
  preferences.end();
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

    Serial.printf("Value is now %d", Value);
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
          Serial.printf("Value %d = ",SNr);
          Serial.println(Value);
          if (Module.GetRelayType() == RELAY_REVERSED) 
          {
              if (Value == 0) Value = 1;
              else Value = 0;
          }
          /*if (Value == 1) digitalWrite(Module.GetPeriphIOPort(SNr), HIGH);
          else digitalWrite(Module.GetPeriphIOPort(SNr), LOW);
          
          Serial.print(Value); Serial.println(" geschrieben");
          */
      }
  }
  SendMessage();
}
void  PrintMAC(const uint8_t * mac_addr)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}
void  GoToSleep() {
  JsonDocument doc;;
  String jsondata;
  
  doc["Node"] = Module.GetName();   
  doc["Type"] = Module.GetType();
  doc["Msg"]  = "GoodBye - going to sleep";
  
  serializeJson(doc, jsondata);  

  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  //Sending "jsondata"  
  
  if (Module.GetDebugMode()) { Serial.print("\nSending: "); Serial.println(jsondata); }
  AddStatus("Send Going to sleep..."); 
  
  Serial.print("Going to sleep at: "); Serial.println(millis());
  Serial.print("LastContact    at: "); Serial.println(Module.GetLastContact());
  
  gpio_deep_sleep_hold_en();
  for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) if (Module.GetPeriphType(SNr) == SENS_TYPE_SWITCH) gpio_hold_en((gpio_num_t)Module.GetPeriphIOPort(SNr));  
  
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL * 1000);
  esp_deep_sleep_start();

}
void SaveModule()
{
      preferences.begin("JeepifyInit", false);
          String ExportStringPeer = Module.Export();

          Serial.printf("putSring = %d", preferences.putString("Module", ExportStringPeer));
          Serial.printf("schreibe: Module: %s",ExportStringPeer.c_str());
          Serial.println();
      preferences.end();
}
#pragma endregion System-Things
#pragma region Data-Things
void VoltageCalibration(int SNr, float V) 
{
    char Buf[100] = {}; 
  
    if (Module.GetDebugMode()) Serial.println("Volt-Messung kalibrieren...");
    
    preferences.begin("JeepifyInit", false);
  
    if (Module.GetPeriphType(SNr) == SENS_TYPE_VOLT) {
        int TempRead = analogRead(Module.GetPeriphIOPort(SNr));
        
        Module.SetPeriphVin(SNr, TempRead / V);
        
        if (Module.GetDebugMode()) {
          Serial.print("S["); Serial.print(SNr); Serial.print("].Vin = ");
          Serial.println(Module.GetPeriphVin(SNr), 4);
          Serial.print("Volt(nachher) = ");
          Serial.println(TempRead/Module.GetPeriphVin(SNr), 4);
        }
        
        snprintf(Buf, sizeof(Buf), "[%d] %s (Type: %d): Spannung ist jetzt: %.2fV", SNr, Module.GetPeriphName(SNr), Module.GetPeriphType(SNr), TempRead/Module.GetPeriphVin(SNr));
        
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
        
        #ifdef ADC_USED
        TempVal  = ads.readADC_SingleEnded(S[SNr].IOPort);
        TempVolt = ads.computeVolts(TempVal);
        #else
        //Filter implementieren !!!
        TempVal  = analogRead(Module.GetPeriphIOPort(SNr));
        TempVolt = 3.3/4095*TempVal; // 1.5???
        #endif    

        delay(10);  

        if (Module.GetDebugMode()) { 
          Serial.print("TempVal:");     Serial.println(TempVal);
          Serial.print(", TempVolt: "); Serial.println(TempVolt);
        }
        Module.SetPeriphNullwert(SNr, TempVolt);

        snprintf(Buf, sizeof(Buf), "Eichen fertig: [%d] %s (Type: %d): Gemessene Spannung bei Null: %.2fV", 
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
  
  if (Module.GetADCPort1() != -1)
  {
      #ifdef ADC_USED
        TempVal  = ads.readADC_SingleEnded(Module.GetPeriphIOPort(SNr));
        TempVolt = ads.computeVolts(TempVal); 
        TempAmp  = (TempVolt - Module.GetPeriphNullwertSNr()) / Module.GetPeriphVperAmp(SNr);
        delay(10);
      #endif
  }
  else
  {
      TempVal  = analogRead(Module.GetPeriphIOPort(SNr));
      TempVolt = 3.3/4095*TempVal;
      TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr) * Module.GetVoltageDevider();// 1.5 wegen Voltage-Devider
      delay(10);
  }
  
  if (Module.GetDebugMode()) {
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
  
  if (Module.GetDebugMode()) {
    Serial.print("TempVal:  "); Serial.println(TempVal,4);
    Serial.print("Vin:      "); Serial.println(Module.GetPeriphVin(SNr));
    Serial.print("Volt (TempVal / S[V].Vin)): ");     Serial.println(TempVolt,4);
    
  } 
  return TempVolt;
}
#pragma endregion Data-Things
#pragma region ESP-Things
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{  
  char* buff = (char*) incomingData;        //char buffer
  JsonDocument doc;;
  String jsondata;
  
  jsondata = String(buff);                  //converting into STRING
  
  Serial.print("Recieved from: "); PrintMAC(mac); 
  
  DeserializationError error = deserializeJson(doc, jsondata);
  
  if (!error) {
      String TempName = doc["Node"];
      Serial.print("("); Serial.print(TempName); Serial.print(") - ");
      Serial.println(jsondata);    
      
      if ((doc["Pairing"] == "you are paired") and (doc["Peer"] == Module.GetName())) 
      { 
          Serial.println("in you are paired und node");
        
          bool exists = esp_now_is_peer_exist(mac);
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
                
                if (Module.GetDebugMode()) {
                  Serial.print("Name: "); Serial.print(Peer->GetName());
                  Serial.print(" (");PrintMAC(Peer->GetBroadcastAddress()); Serial.println(")\n");
                  Serial.print("Saving Peers after received new one...");
                  ReportAll();
                }
                Module.SetPairMode(false);
            }
      }
      if      (doc["Order"] == "stay alive")       
      {   Module.SetLastContact(millis());
          if (Module.GetDebugMode()) { Serial.print("LastContact: "); Serial.println(Module.GetLastContact()); }
      }
      else if (doc["Order"] == "SleepMode On")     
      { 
          AddStatus("Sleep: on");  
          SetSleepMode(true);  
          SendMessage(); 
      }
      else if (doc["Order"] == "SleepMode Off")    
      { 
          AddStatus("Sleep: off"); 
          SetSleepMode(false); 
          SendMessage(); 
      }
      else if (doc["Order"] == "SleepMode Toggle") 
      { 
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
      } 
      else if (doc["Order"] == "DebugMode on")     
      { 
          AddStatus("DebugMode: on");  
          SetDebugMode(true);  
          SendMessage(); 
      }
      else if (doc["Order"] == "DebugMode off")    
      { 
          AddStatus("DebugMode: off"); 
          SetDebugMode(false); 
          SendMessage(); 
      }
      else if (doc["Order"] == "DebugMode Toggle") 
      { 
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
      }
      else if (doc["Order"] == "DemoMode on")      
      { 
          AddStatus("Demo: on");   
          SetDemoMode(true);   
          SendMessage(); 
      }
      else if (doc["Order"] == "DemoMode off")     
      { 
          AddStatus("Demo: off");  
          SetDemoMode(false);  
          SendMessage(); 
      }
      else if (doc["Order"] == "DemoMode Toggle")  
      { 
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
      }
      else if (doc["Order"] == "Reset")         
      { 
          AddStatus("Clear all"); 
          nvs_flash_erase(); 
          nvs_flash_init();
          ESP.restart();
      }
      else if (doc["Order"] == "Restart")       
      { 
          ESP.restart(); 
      }
      else if (doc["Order"] == "Pair")          
      {   
          TSPair = millis(); 
          Module.SetPairMode(true); 
          AddStatus("Pairing beginnt"); 
          SendMessage(); 
          #ifdef DISPLAY_480
            smartdisplay_led_set_rgb(1,0,0);
          #endif
      }
      else if (doc["Order"] == "Eichen")        
      {   
          AddStatus("Eichen beginnt"); 
          CurrentCalibration();
      }
      else if (doc["Order"] == "VoltCalib")     
      { 
          AddStatus("VoltCalib beginnt");
          float NewVoltage = doc["NewVoltage"];

          if (Module.GetVoltageMon() != -1)
          {
              VoltageCalibration(Module.GetVoltageMon(), NewVoltage) ;
          }
      }
      else if (doc["Order"] == "ToggleSwitch")  
      { 
          int Pos = doc["Pos"];
          if (Module.isPeriphEmpty(Pos) == false) ToggleSwitch(Pos);
      }  
      else if (doc["Order"] == "UpdateName")
      {
          int Pos = (int) doc["Pos"];
                String NewName = doc["NewName"];

                if (NewName != "") 
                {
                    if (Pos == 99) Module.SetName(NewName.c_str());
                    else           Module.SetPeriphName(Pos, NewName.c_str());
                }
                
                SaveModule();
		            SendNameChange(Pos);
      }
    } // end (!error)
    else // error
    { 
          Serial.print(F("deserializeJson() failed: "));  //Just in case of an ERROR of ArduinoJSon
          Serial.println(error.f_str());
    }
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{ 
    if (Module.GetDebugMode()) {
        //Serial.print("\r\nLast Packet Send Status:\t");
        //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }
}
#pragma endregion ESP-Things

void setup()
{
    #ifdef ARDUINO_USB_CDC_ON_BOOT
        delay(5000);
    #endif

    Serial.begin(115200);

    #ifdef DISPLAY_480
        smartdisplay_init();

        __attribute__((unused)) auto disp = lv_disp_get_default();
        lv_disp_set_rotation(disp, LV_DISP_ROT_90);
    #endif

    #ifdef ADC_USED
    if (Module.ADCPort != -1)
    {
        Wire.begin(D5, D6);
        ads.setGain(GAIN_TWOTHIRDS);  // 0.1875 mV/Bit .... +- 6,144V
        ads.begin();
    }
    #endif

    InitModule();

    /*for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)  
    { 
        switch (Module.GetPeriphType(SNr)) {
            case SENS_TYPE_SWITCH: pinMode(Module.GetPeriphIOPort(SNr), OUTPUT); break;
            case SENS_TYPE_VOLT:   pinMode(Module.GetPeriphIOPort(SNr), INPUT ); break;
            case SENS_TYPE_AMP:    pinMode(Module.GetPeriphIOPort(SNr), INPUT ); break;
        }
    }
  */
    if (preferences.begin("JeepifyInit", true))
    {
        String SavedModule   = preferences.getString("Module", "");
            Serial.printf("Importiere Modul: %s", SavedModule.c_str());
            char ToImport[250];
            strcpy(ToImport,SavedModule.c_str());
            if (strcmp(ToImport, "") != 0) Module.Import(ToImport);
        preferences.end();
    }

    WiFi.mode(WIFI_STA);
    uint8_t MacTemp[6];
    WiFi.macAddress(MacTemp);
    Module.SetBroadcastAddress(MacTemp);

    if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); }
  
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);    

    AddStatus("Init Module");
    
    #ifdef KILL_NVS
      nvs_flash_erase(); nvs_flash_init(); ESP.restart();
    #endif

    int PeerCount = GetPeers();       
    AddStatus("Get Peers");
    
    ReportAll();    
    
    RegisterPeers();  
    AddStatus("Init fertig");
  
    Module.SetLastContact(millis());
    
    /*
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) if (Module.GetPeriphType(SNr) == SENS_TYPE_SWITCH) 
    {
        gpio_hold_dis((gpio_num_t)Module.GetPeriphIOPort(SNr));  
    }

    gpio_deep_sleep_hold_dis(); 
  */
    //UpdateSwitches();

    #ifndef DISPLAY_NO
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
    #ifdef DISPLAY_480
      smartdisplay_led_set_rgb(0, 0, 0);
    #endif
  }
    lv_timer_handler();
    delay(5);
}