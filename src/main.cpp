#pragma region Includes
#include <Arduino.h>

#include <esp32_smartdisplay.h>
#include <ui/ui.h>
#include "LinkedList.h"
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

struct struct_Status {
  String    Msg;
  uint32_t  TSMsg;
};

PeerClass Module;
PeerClass P[MAX_PEERS];

struct_Status Status[MAX_STATUS];

volatile u_int8_t TempBroadcast[6];
u_int8_t broadcastAddressAll[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; 

volatile uint32_t TSLastSend      = 0;
volatile uint32_t TSBootButton    = 0;
volatile uint32_t TSSend  = 0;
volatile uint32_t TSPair  = 0;
volatile uint32_t TSLed   = 0;

int PeerCount = 0;

Preferences preferences;

#pragma region Functions
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

float  ReadAmp (int A);
float  ReadVolt(int V);
void   SendMessage();
void   SendPairingRequest();
void   InitModule();
void   SavePeers();
void   GetPeers();
void   ReportPeers();
void   RegisterPeers();
void   ClearPeers();

void   UpdateSwitches();

void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);
void   AddStatus(String Msg);

void   PrintMAC(const uint8_t * mac_addr);

void   GoToSleep();
#pragma endregion Functions

void InitModule()
{
    Module.Setup("ESP32-3.5", SWITCH_4_WAY, "3.41", NULL, false, true, true, false, -1, 1, -1, -1, 1);
    //                      Name     Type             ADS  IO  NULL   VpA   Vin  PeerID
    Module.GetPeriphPtr(0)->Setup("Extern", SENS_TYPE_AMP,     0,  0, 3134, 0.066,   0,   0);
    Module.GetPeriphPtr(1)->Setup("In-Car", SENS_TYPE_AMP,     0,  1, 3134, 0.066,   0,   0);
    Module.GetPeriphPtr(2)->Setup("Solar",  SENS_TYPE_AMP,     0,  2, 3134, 0.066,   0,   0);
    Module.GetPeriphPtr(3)->Setup("Load",   SENS_TYPE_AMP,     0,  3, 3150, 0.066,   0,   0);
    Module.GetPeriphPtr(4)->Setup("Lipo",   SENS_TYPE_VOLT,    0,  2, 0,    0,     200,   0); 
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { 
    if (Module.GetDebugMode()) {
        //Serial.print("\r\nLast Packet Send Status:\t");
        //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }
}
#pragma region Send-Things
void SendMessage () {
  //sendet NAME0:Value0, NAME1:Value1, SLEEP:Status, Debug:Status
  TSLed = millis();
  //digitalWrite(LED_BUILTIN, LED_ON);

  StaticJsonDocument<500> doc; String jsondata; doc.clear(); jsondata = "";
  char buf[100]; 

  doc["Node"] = Module.GetName();   

  for (int SNr=0; SNr<MAX_PERIPHERALS ; SNr++) {
    if (Module.GetPeriphPtr(SNr)->GetType() == SENS_TYPE_SWITCH) {
      doc[Module.GetPeriphPtr(SNr)->GetName()] = Module.GetPeriphPtr(SNr)->GetValue();
    }
    if (Module.GetPeriphPtr(SNr)->GetType() == SENS_TYPE_AMP) {
      if (Module.GetDemoMode()) Module.GetPeriphPtr(SNr)->SetValue(random(0,30));
      else                 Module.GetPeriphPtr(SNr)->SetValue(ReadAmp(SNr));
    	
      if (abs(Module.GetPeriphPtr(SNr)->GetValue()) > 99) Module.GetPeriphPtr(SNr)->SetValue(-99);
      dtostrf(Module.GetPeriphPtr(SNr)->GetValue(), 0, 2, buf);
      
      if (Module.GetPeriphPtr(SNr)->hasChanged())
      {
          Module.GetPeriphPtr(SNr)->SetOldValue(Module.GetPeriphPtr(SNr)->GetValue());
          Module.GetPeriphPtr(SNr)->SetChanged(true);
      }
      else {
          Module.GetPeriphPtr(SNr)->SetOldValue(Module.GetPeriphPtr(SNr)->GetValue());
          Module.GetPeriphPtr(SNr)->SetChanged(false);
      }

      //if (S[SNr].Changed) 
      doc[Module.GetPeriphPtr(SNr)->GetName()] = buf;
    }

    if (Module.GetPeriphPtr(SNr)->GetType() == SENS_TYPE_VOLT) {
      if (Module.GetDemoMode()) Module.GetPeriphPtr(SNr)->SetValue(random(10,15));
      else                 Module.GetPeriphPtr(SNr)->SetValue(ReadVolt(SNr));

      dtostrf(Module.GetPeriphPtr(SNr)->GetValue(), 0, 2, buf);
      
      if (Module.GetPeriphPtr(SNr)->hasChanged()) {
          Module.GetPeriphPtr(SNr)->SetOldValue(Module.GetPeriphPtr(SNr)->GetValue());
          Module.GetPeriphPtr(SNr)->SetChanged(true);
      }
      else {
          Module.GetPeriphPtr(SNr)->SetOldValue(Module.GetPeriphPtr(SNr)->GetValue());
          Module.GetPeriphPtr(SNr)->SetChanged(false);
      }

      //if (S[SNr].Changed) 
      doc[Module.GetPeriphPtr(SNr)->GetName()] = buf;
    }
  }
  
  // Status bit1 DebugMode, bit2 Sleep, bit3 Demo, bit4 RTP
  int Status = 0;
  if (Module.GetDebugMode())   bitSet(Status, 0);
  if (Module.GetSleepMode())   bitSet(Status, 1);
  if (Module.GetDemoMode())    bitSet(Status, 2);
  if (Module.GetPairMode())    bitSet(Status, 3);
  
  doc["Status"]  = Status;

  serializeJson(doc, jsondata);  

  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].GetType() >= MONITOR_ROUND) {
      Serial.print("Sending to: "); Serial.print(P[PNr].GetName()); 
      if (esp_now_send(P[PNr].GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 200) == 0) Serial.println("ESP_OK");  //Sending "jsondata"  
      else Serial.println("ESP_ERROR"); 
      Serial.println(jsondata);
    }
  }

  AddStatus("SendStatus");
}
void SendPairingRequest() {
  // sendet auf Broadcast: "addme", T0:Type, N0:Name, T1:Type, N1:Name...
  TSLed = millis();
  //digitalWrite(LED_BUILTIN, LED_ON);
  
  StaticJsonDocument<500> doc; String jsondata; jsondata = ""; doc.clear();
  char Buf[100] = {};
  
  doc["Node"]    = Module.GetName();   
  doc["Type"]    = Module.GetType();
  doc["Pairing"] = "add me";
  
  for (int SNr=0 ; SNr<MAX_PERIPHERALS; SNr++) {
    if (Module.GetPeriphPtr(SNr)->GetType()) {
      snprintf(Buf, sizeof(Buf), "T%d", SNr); 
      doc[Buf] =Module.GetPeriphPtr(SNr)->GetType();
      snprintf(Buf, sizeof(Buf), "N%d", SNr); 
      doc[Buf] = Module.GetPeriphPtr(SNr)->GetName();
    }
  }
  serializeJson(doc, jsondata);  

  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  //Sending "jsondata"  
  
  if (Module.GetDebugMode()) { Serial.print("\nSending: "); Serial.println(jsondata); }
  AddStatus("Send Pairing request...");                                     
}
#pragma endregion Send-Things
#pragma region System-Things
void SetDemoMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    Module.SetDemoMode(Mode);
    if (preferences.getBool("DemoMode", false) != Module.GetDemoMode()) preferences.putBool("DemoMode", Module.GetDemoMode());
  preferences.end();
}
void SetSleepMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    Module.SetSleepMode(Mode);
    if (preferences.getBool("SleepMode", false) != Module.GetSleepMode()) preferences.putBool("SleepMode", Module.GetSleepMode());
  preferences.end();
}
void SetDebugMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    Module.SetDebugMode(Mode);
    if (preferences.getBool("DebugMode", false) != Module.GetDebugMode()) preferences.putBool("DebugMode", Module.GetDebugMode());
  preferences.end();
}
void AddStatus(String Msg) {
  for (int Si=MAX_STATUS-1; Si>0; Si--) {
    Status[Si].Msg   = Status[Si-1].Msg;
    Status[Si].TSMsg = Status[Si-1].TSMsg;
  }
  Status[0].Msg = Msg;
  Status[0].TSMsg = millis();
}
void UpdateSwitches() {
  for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) if (Module.GetPeriphPtr(SNr)->GetType() == SENS_TYPE_SWITCH) digitalWrite(Module.GetPeriphPtr(SNr)->GetIOPort(), Module.GetPeriphPtr(SNr)->GetValue()*Module.GetRelayType()); // ???
  SendMessage();
}
void  PrintMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}
float ReadAmp (int SNr) {
  float TempVal      = 0;
  float TempVolt     = 0;
  float TempAmp      = 0;
  
  if (Module.GetADCPort1() != -1)
  {
      /*
      TempVal  = ads.readADC_SingleEnded(Module.GetPeriphPtr(SNr)->GetIOPort());
      TempVolt = ads.computeVolts(TempVal); 
      TempAmp  = (TempVolt - Module.GetPeriphPtr(SNr)->GetNullwert()) / Module.GetPeriphPtr(SNr)->GetVperAmp();
      delay(10);
      */
  }
  else
  {
      float TempVoltOverNull = 0;
      TempVal  = analogRead(Module.GetPeriphPtr(SNr)->GetIOPort());
      TempVolt = 3.3/4095*TempVal;
      TempAmp  = (TempVolt - Module.GetPeriphPtr(SNr)->GetNullwert()) / Module.GetPeriphPtr(SNr)->GetVperAmp() * Module.GetVoltageDevider();// 1.5 wegen Voltage-Devider
      delay(10);
  }
  
  if (Module.GetDebugMode()) {
    Serial.print("TempVal:  "); Serial.println(TempVal,4);
    Serial.print("TempVolt: "); Serial.println(TempVolt,4);
    Serial.print("Nullwert: "); Serial.println(Module.GetPeriphPtr(SNr)->GetNullwert(),4);
    Serial.print("VperAmp:  "); Serial.println(Module.GetPeriphPtr(SNr)->GetVperAmp(),4);
    Serial.print("Amp (TempVolt - S[Si].NullWert) / S[Si].VperAmp * 1.5:  "); Serial.println(TempAmp,4);
  } 
  if (abs(TempAmp) < SCHWELLE) TempAmp = 0;
  
  return (TempAmp); //TempAmp;
}
float ReadVolt(int SNr) {
  if (!Module.GetPeriphPtr(SNr)->GetVin()) { Serial.println("Vin must not be zero !!!"); return 0; }
  
  float TempVal  = analogRead(Module.GetPeriphPtr(SNr)->GetIOPort());
  float TempVolt = TempVal / Module.GetPeriphPtr(SNr)->GetVin();
  
  if (Module.GetDebugMode()) {
    Serial.print("TempVal:  "); Serial.println(TempVal,4);
    Serial.print("Vin:      "); Serial.println(Module.GetPeriphPtr(SNr)->GetVin());
    Serial.print("Volt (TempVal / S[V].Vin)): ");     Serial.println(TempVolt,4);
    
  } 
  return TempVolt;
}
void  GoToSleep() {
  StaticJsonDocument<500> doc;
  String jsondata;

  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
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
  for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) if (Module.GetPeriphPtr(SNr)->GetType() == SENS_TYPE_SWITCH) gpio_hold_en((gpio_num_t)Module.GetPeriphPtr(SNr)->GetIOPort());  
  
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL * 1000);
  esp_deep_sleep_start();

}
#pragma endregion System-Things
#pragma region OnDataRecv
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{  
  char* buff = (char*) incomingData;        //char buffer
  StaticJsonDocument<500> doc;
  String jsondata;
  doc.clear();
  jsondata = "";
  bool PairingSuccess = false;
  
  jsondata = String(buff);                  //converting into STRING
  
  Serial.print("Recieved from: "); PrintMAC(mac); 
  
  DeserializationError error = deserializeJson(doc, jsondata);
  
  if (!error) {
    String TempName = doc["Node"];
    Serial.print("("); Serial.print(TempName); Serial.print(") - ");
    Serial.println(jsondata);    
    
    if ((doc["Pairing"] == "you are paired") and (doc["Peer"] == Module.GetName())) { 
      Serial.println("in you are paired und node");
    
      bool exists = esp_now_is_peer_exist(mac);
      if (exists) { 
        PrintMAC(mac); Serial.println(" already exists...");
      }
      else {
        for (int PNr=0; PNr<MAX_PEERS; PNr++) {
          Serial.print("P["); Serial.print(PNr); Serial.print("].Type = "); Serial.println(P[PNr].GetType());
          if ((P[PNr].GetType() == 0) and (!PairingSuccess)) {
            Serial.println("leerer Slot gefunden");
            P[PNr].SetType((int) doc["Type"]);
            
            P[PNr].SetName(doc["Node"]);
            
            memcpy(P[PNr].GetBroadcastAddress(), mac, 6);
            P[PNr].SetTSLastSeen(millis());
            
            PairingSuccess = true; 
            SavePeers();
            RegisterPeers();
            
            if (Module.GetDebugMode()) {
              Serial.print("Adding in slot: "); Serial.println(PNr);
              Serial.print("Name: "); Serial.print(P[PNr].GetName());
              Serial.print(" (");PrintMAC(P[PNr].GetBroadcastAddress()); Serial.println(")\n");
              Serial.print("Saving Peers after received new one...");
              ReportAll();
            }
            Module.SetPairMode(false);
          }
        }
        if (!PairingSuccess) { PrintMAC(mac); Serial.println(" adding failed..."); } 
      }
    }
    if      (doc["Order"] == "stay alive")       { Module.SetLastContact(millis());
                                                   if (Module.GetDebugMode()) { Serial.print("LastContact: "); Serial.println(Module.GetLastContact()); }
                                                 }
    else if (doc["Order"] == "SleepMode On")     { AddStatus("Sleep: on");  SetSleepMode(true);  SendMessage(); }
    else if (doc["Order"] == "SleepMode Off")    { AddStatus("Sleep: off"); SetSleepMode(false); SendMessage(); }
    else if (doc["Order"] == "SleepMode Toggle") { if (Module.GetSleepMode()) { AddStatus("Sleep: off");   SetSleepMode(false); SendMessage(); }
                                                                    else { AddStatus("Sleep: on");    SetSleepMode(true);  SendMessage(); }
                                                 } 
    else if (doc["Order"] == "DebugMode on")     { AddStatus("DebugMode: on");  SetDebugMode(true);  SendMessage(); }
    else if (doc["Order"] == "DebugMode off")    { AddStatus("DebugMode: off"); SetDebugMode(false); SendMessage(); }
    else if (doc["Order"] == "DebugMode Toggle") { if (Module.GetDebugMode()) { AddStatus("DebugMode: off");   SetDebugMode(false); SendMessage(); }
                                                                         else { AddStatus("DebugMode: on");    SetDebugMode(true);  SendMessage(); }
                                                 }
    else if (doc["Order"] == "DemoMode on")      { AddStatus("Demo: on");   SetDemoMode(true);   SendMessage(); }
    else if (doc["Order"] == "DemoMode off")     { AddStatus("Demo: off");  SetDemoMode(false);  SendMessage(); }
    else if (doc["Order"] == "DemoMode Toggle")  { if (Module.GetDemoMode()) { AddStatus("DemoMode: off"); SetDemoMode(false); SendMessage(); }
                                                                        else { AddStatus("DemoMode: on");  SetDemoMode(true);  SendMessage(); }
                                              }
    else if (doc["Order"] == "Reset")         { AddStatus("Clear all"); ClearPeers(); ClearInit(); ESP.restart(); }
    else if (doc["Order"] == "Restart")       { ESP.restart(); }
    else if (doc["Order"] == "Pair")          { TSPair = millis(); Module.SetPairMode(true); AddStatus("Pairing beginnt"); SendMessage(); }

    else if (doc["Order"] == "Eichen")        { AddStatus("Eichen beginnt"); }
    else if (doc["Order"] == "VoltCalib")     { AddStatus("VoltCalib beginnt");}
    else if (doc["Order"] == "ToggleSwitch")  { for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
                                                  if ((Module.GetPeriphPtr(SNr)->GetName() == doc["Value"]) and (Module.GetPeriphPtr(SNr)->GetType() == SENS_TYPE_SWITCH)) {
                                                    Module.GetPeriphPtr(SNr)->SetValue(!Module.GetPeriphPtr(SNr)->GetValue()); 
                                                    String Nr = doc["Value"];
                                                    AddStatus("ToggleSwitch "+Nr);
                                                    UpdateSwitches();
                                                  }
                                                }
                                              }      
  } // end (!error)
  else {  // error
        Serial.print(F("deserializeJson() failed: "));  //Just in case of an ERROR of ArduinoJSon
        Serial.println(error.f_str());
  }
}


void setup()
{
#ifdef ARDUINO_USB_CDC_ON_BOOT
    delay(5000);
#endif
    Serial.begin(115200);

    smartdisplay_init();

    __attribute__((unused)) auto disp = lv_disp_get_default();
    lv_disp_set_rotation(disp, LV_DISP_ROT_90);

    /*
    if (Module.ADCPort != -1)
    {
        Wire.begin(D5, D6);
        ads.setGain(GAIN_TWOTHIRDS);  // 0.1875 mV/Bit .... +- 6,144V
        ads.begin();
    }
    */
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)  
    { 
        switch (Module.GetPeriphType(SNr)) {
            case SENS_TYPE_SWITCH: pinMode(Module.GetPeriphIOPort(SNr), OUTPUT); break;
            case SENS_TYPE_VOLT:   pinMode(Module.GetPeriphIOPort(SNr), INPUT ); break;
            case SENS_TYPE_AMP:    pinMode(Module.GetPeriphIOPort(SNr), INPUT ); break;
        }
    }
  
    preferences.begin("JeepifyInit", true);
        Module.SetDebugMode(preferences.getBool("DebugMode", Module.GetDebugMode()));
        Module.SetSleepMode(preferences.getBool("SleepMode", Module.GetSleepMode()));
        Module.SetDemoMode (preferences.getBool("DemoMode",  Module.GetDemoMode()));
        String NewName   = preferences.getString("ModuleName", "");
        if (NewName != "") Module.SetName(NewName.c_str());
    preferences.end();

    WiFi.mode(WIFI_STA);
  
    if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); }
  
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);    

    AddStatus("Init Module");
    GetPeers();       AddStatus("Get Peers");
    ReportAll();    
    RegisterPeers();  AddStatus("Init fertig");
  
    //if (PeerCount == 0) { AddStatus("Pairing beginnt"); Module.PairMode = true; TSPair = millis(); }
  
    Module.SetLastContact(millis());

    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) if (Module.GetPeriphType(SNr) == SENS_TYPE_SWITCH) 
    {
        gpio_hold_dis((gpio_num_t)Module.GetPeriphIOPort(SNr));  
    }

    gpio_deep_sleep_hold_dis(); 
  
    UpdateSwitches();

    ui_init(); 



    ui_init();
}

void loop()
{


#ifdef BOARD_HAS_RGB_LED
        auto const rgb = (millis() / 2000) % 8;
        //smartdisplay_led_set_rgb(rgb & 0x01, rgb & 0x02, rgb & 0x04);
#endif



    lv_timer_handler();
}