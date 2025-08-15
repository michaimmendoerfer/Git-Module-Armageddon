//#define KILL_NVS 1

// DEBUG_LEVEL: 0 = nothing, 1 = only Errors, 2 = relevant changes, 3 = all
#define DEBUG1(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 0)) Serial.printf(__VA_ARGS__)
#define DEBUG2(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 1)) Serial.printf(__VA_ARGS__)
#define DEBUG3(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 2)) Serial.printf(__VA_ARGS__)
#define JX(...) (doc[__VA_ARGS__].is<JsonVariant>())

#include <Arduino.h>
#include <Module.h>

const int DEBUG_LEVEL = 1; 
const int _LED_SIGNAL = 1;

#define WAIT_ALIVE       15000
#define WAIT_AFTER_SLEEP  3000
#define RELAY_CHECK      100

uint32_t WaitForContact = WAIT_AFTER_SLEEP;

#pragma region Includes

#ifdef MODULE_TERMINATOR_PRO
    #include <esp32_smartdisplay.h>
#endif
#ifdef MODULA_HAS_DISPLAY
    #include <ui/ui.h>
#endif
#ifdef RGBLED_PIN
    #include <Adafruit_NeoPixel.h>
    Adafruit_NeoPixel pixels (1, RGBLED_PIN, NEO_GRB + NEO_KHZ800);
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

#if defined(PORT0) || defined(ADC0)
    #include <Wire.h>
    #include <Spi.h>
    #define I2C_FREQ 400000
    #ifdef ADC0
        #include <Adafruit_ADS1X15.h>
        Adafruit_ADS1115 ADCBoard[4];
    #endif
        
    #ifdef PORT0
        #include "PCF8575.h"
        PCF8575 *IOBoard[4];
        PCF8575 IOBoard0 (PORT0, SDA_PIN, SCL_PIN); 
        
        #ifdef PORT1
            PCF8575 IOBoard1 (PORT1, SDA_PIN, SCL_PIN); 
        #endif
        #ifdef PORT2
            PCF8575 IOBoard2 (PORT1, SDA_PIN, SCL_PIN); 
        #endif
        #ifdef PORT3
            PCF8575 IOBoard3 (PORT1, SDA_PIN, SCL_PIN); 
        #endif
    #endif
#endif

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
    
#pragma endregion Includes

#pragma region Globals

struct struct_Status {
  String    Msg;
  uint32_t  TSMsg;
};

struct ReceivedMessagesStruct {
    uint8_t  From[6];
    uint32_t TS;
    uint32_t SaveTime;
};

MyLinkedList<ReceivedMessagesStruct*> ReceivedMessagesList = MyLinkedList<ReceivedMessagesStruct*>();

PeerClass Module;
MyLinkedList<PeriphClass*> SwitchList = MyLinkedList<PeriphClass*>();
MyLinkedList<PeriphClass*> SensorList = MyLinkedList<PeriphClass*>();

struct_Status Status[MAX_STATUS];

u_int8_t broadcastAddressAll[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; 
const char *broadCastAddressAllC = "FFFFFFFFFFFF";

volatile uint32_t TSButton   = 0;
volatile uint32_t TSSend     = 0;
volatile uint32_t TSPair     = 0;
volatile uint32_t TSLed      = 0;
volatile uint32_t TSStatus   = 0;
volatile uint32_t TSSettings = 0;
volatile uint32_t TSCheckRel = 0;

int lastPeriphSent = -1;

bool NameChanged = false;

Preferences preferences;
#pragma endregion Globals

#pragma region Functions
#ifdef ESP32 
    void OnDataRecv(const esp_now_recv_info *info, const uint8_t* incomingData, int len);
#elif defined(ESP8266)
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
#endif
//void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
#ifdef ESP32 
    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status); 
#elif defined(ESP8266)
    void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);
#endif
void OnDataRecvCommon(const uint8_t * mac, const uint8_t *incomingData, int len);

void   InitSCL();
void   InitMRD();

float  ReadAmp (int SNr);
float  ReadVolt(int SNr);
void   SendStatus (int Pos=-1);
void   SendPairingRequest();

bool   GetRelayState(int SNr);
void   SetRelayState(int SNr, bool State);

void   UpdateDataFromSwitches();

void   SetDemoMode (bool Mode);
void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);
void   SetPairMode(bool Mode);

void   SaveModule();
void   AddStatus(String Msg);
void   GoToSleep();
void   SetMessageLED(int Color);
void   LEDBlink(int Color, int n, uint8_t ms);
void   MacCharToByte(uint8_t *mac, char *MAC);
void   MacByteToChar(char *MAC, uint8_t *mac);
bool   MACequals( uint8_t *MAC1, uint8_t *MAC2);

#pragma endregion Functions

void setup()
{
    if (DEBUG_LEVEL > 0)
    {
        #ifdef ARDUINO_USB_CDC_ON_BOOT
            //delay(3000);
        #endif
    }
   
    if (DEBUG_LEVEL > 0)
    {
        #ifdef ESP32
            Serial.begin(115200);
        #elif defined(ESP8266)
            Serial.begin(74880);
        #endif
    }

    #ifdef KILL_NVS
        nvs_flash_erase(); nvs_flash_init(); while (1) {};
    #endif


    InitSCL();

    if (DEBUG_LEVEL > 0)                        // Show free entries
    {
        preferences.begin("JeepifyInit", true);
            Serial.printf("free entries in JeepifyInit now: %d\n\r", preferences.freeEntries());
        preferences.end();
        preferences.begin("JeepifyPeers", true);
            Serial.printf("free entries in JeepifyPeers now: %d\n\r", preferences.freeEntries());
        preferences.end();
    }
    
    #ifdef MODULE_TERMINATOR_PRO
        smartdisplay_init();

        __attribute__((unused)) auto disp = lv_disp_get_default();
        lv_disp_set_rotation(disp, LV_DISP_ROT_90);
    #endif

    InitModule();
    
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:    
            WaitForContact = WAIT_AFTER_SLEEP; 
            LEDBlink(4, 1, 100);
            break;
        default:                        
            WaitForContact = WAIT_ALIVE; 
            LEDBlink(3, 3, 100);
            break;
    }

    if (preferences.begin("JeepifyInit", true)) // import saved Module... if available
    {
        String SavedModule   = preferences.getString("Module", "");
            DEBUG2 ("Importiere Modul: %s\n\r", SavedModule.c_str());
            char ToImport[250];
            strcpy(ToImport,SavedModule.c_str());
            if (strcmp(ToImport, "") != 0) Module.Import(ToImport);
        preferences.end();
    }
    
    UpdateDataFromSwitches();
    //UpdateSwitchesFromData();

    WiFi.mode(WIFI_STA);
    WiFi.STA.begin();
    uint8_t MacTemp[6];
    WiFi.macAddress(MacTemp);
    Module.SetBroadcastAddress(MacTemp);

    Module.SetDebugMode(true);

    if (esp_now_init() != 0) 
        DEBUG1 ("Error initializing ESP-NOW\n\r");
    #ifdef ESP8266
        esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    #endif 
    
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);    

    AddStatus("Init Module");
    
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
    
    #ifdef MODULE_HAS_DISPLAY
      ui_init();
    #endif
    
}
#pragma region Send-Things
void GarbageMessages()
{
    if (ReceivedMessagesList.size() > 0)
    {  
        for (int i=ReceivedMessagesList.size()-1; i>=0; i--)
        {
            ReceivedMessagesStruct *RMItem = ReceivedMessagesList.get(i);
            
            if (millis() > RMItem->SaveTime + SEND_CMD_MSG_HOLD*1000)
            {
                DEBUG3 ("Message aus RMList entfernt\n\r");
                ReceivedMessagesList.remove(i);
                delete RMItem;
            }
        }
    }
}
void SendStatus (int Pos) 
{
    JsonDocument doc; 
    String jsondata; 
    
    char buf[250]; 
    
    char mac[13];
    MacByteToChar(mac, Module.GetBroadcastAddress());
    
    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);    
    
    doc[SEND_CMD_JSON_FROM]   = mac;
    doc[SEND_CMD_JSON_TO]     = broadCastAddressAllC;
    doc[SEND_CMD_JSON_TS]     = millis();
    doc[SEND_CMD_JSON_TTL]    = SEND_CMD_MSG_TTL;
    doc[SEND_CMD_JSON_STATUS] = Status;
    doc[SEND_CMD_JSON_ORDER]  = SEND_CMD_STATUS;
    
    int SNrStart = lastPeriphSent+1;
    int SNrMax = MAX_PERIPHERALS;

    int PeriphsSent = 0;
    for (int SNr=SNrStart; SNr<SNrMax ; SNr++) 
    {   
        lastPeriphSent = SNr;
        if (!Module.isPeriphEmpty(SNr))
        {
            if (Module.isPeriphSwitch(SNr))
            {
                
                DEBUG3 ("SendStatus(%d) - %s (Switch): %.0f\n\r",SNr, Module.GetPeriphName(SNr), Module.GetPeriphValue(SNr, 0));
            }
        
            if (Module.GetPeriphIOPort(2) > -1)
                Module.SetPeriphValue(SNr, ReadVolt(SNr),      2);

            if (Module.GetPeriphIOPort(3) > -1)
                Module.SetPeriphValue(SNr, ReadAmp(SNr),       3);
            
            char FormatedValue2[10] = "0";
            char FormatedValue3[10] = "0";
            
            if (Module.GetPeriphValue(SNr, 2)) sprintf(FormatedValue2, "%.2f", Module.GetPeriphValue(SNr, 2));
            if (Module.GetPeriphValue(SNr, 3)) sprintf(FormatedValue3, "%.2f", Module.GetPeriphValue(SNr, 3));
            
            snprintf(buf, sizeof(buf), "%d;%s;%.0f;%.0f;%s;%s", //besser 0
                Module.GetPeriphType(SNr),   //-----------------------weg
                Module.GetPeriphName(SNr),   //---------------------weg
                Module.GetPeriphValue(SNr, 0),
                Module.GetPeriphValue(SNr, 1),
                FormatedValue2,
                FormatedValue3);
                        
            doc[ArrPeriph[SNr]] = buf;
            PeriphsSent++;

            //send first Periphs
            jsondata = "";
            if (serializeJson(doc, jsondata) > 240) break;
        }
    }
    if (lastPeriphSent == MAX_PERIPHERALS-1) lastPeriphSent = -1;
    if (PeriphsSent > 0)
    {
        SetMessageLED(2);
        
        //DEBUG2 ("JSON: %s\n\r", jsondata.c_str());
        if (esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 250) == 0) 
        {
            //DEBUG3("ESP_OK\\r");  
        }
        else 
        {
            DEBUG1 ("ESP_ERROR (SendStatus-2)\n\r"); 
        }
        DEBUG3 ("Länge: %d - %s\n\r", strlen(jsondata.c_str()), jsondata.c_str());
    }
}

void SendPairingRequest() 
{
    SetMessageLED(3);
    
    JsonDocument doc; String jsondata; 
    char buf[300] = {};
    char mac[20];

    MacByteToChar(mac, Module.GetBroadcastAddress());
    doc[SEND_CMD_JSON_FROM]  = mac;
    doc[SEND_CMD_JSON_TO]    = broadCastAddressAllC;
    doc[SEND_CMD_JSON_TS]    = (uint32_t) millis();
    doc[SEND_CMD_JSON_TTL]   = SEND_CMD_MSG_TTL;
    

    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);    
    
    doc[SEND_CMD_JSON_STATUS]      = Status;
    doc[SEND_CMD_JSON_ORDER]       = SEND_CMD_PAIR_ME;
    doc[SEND_CMD_JSON_MODULE_TYPE] = Module.GetType();
    doc[SEND_CMD_JSON_VERSION]     = Module.GetVersion();
    doc[SEND_CMD_JSON_PEER_NAME]   = Module.GetName();
    
    for (int SNr=0 ; SNr<MAX_PERIPHERALS; SNr++) {
        if (!Module.isPeriphEmpty(SNr)) 
        {
            snprintf(buf, sizeof(buf), "%d;%s", Module.GetPeriphType(SNr), Module.GetPeriphName(SNr));
            doc[ArrPeriph[SNr]] = buf;
	    }
    }
         
    serializeJson(doc, jsondata);  

    esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 240);  
    
    //DEBUG2 ("\nSending: %s\n\r", jsondata.c_str());                               
}

void SendConfirm(const uint8_t * MAC, uint32_t TSConfirm) 
{
    SetMessageLED(3);
    
    JsonDocument doc; String jsondata; 
    
    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);    

    char _mac[13];
    MacByteToChar(_mac, Module.GetBroadcastAddress());
    doc[SEND_CMD_JSON_FROM]   = _mac;
    MacByteToChar(_mac, (uint8_t *) MAC);
    doc[SEND_CMD_JSON_TO]     = _mac;
    doc[SEND_CMD_JSON_TS]     = TSConfirm;
    doc[SEND_CMD_JSON_TTL]    = SEND_CMD_MSG_TTL;
    doc[SEND_CMD_JSON_STATUS] = Status;
    doc[SEND_CMD_JSON_ORDER]  = SEND_CMD_CONFIRM;

    serializeJson(doc, jsondata); 

    DEBUG2 ("%lu: Sending Confirm (%lu) to: %s ", millis(), TSConfirm, FindPeerByMAC(MAC)->GetName()); 
            
    if (esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200) == 0) 
    {
        DEBUG3 ("ESP_OK\n\r");  
    }
    else 
    {
        DEBUG1 ("ESP_ERROR\n\r"); 
    }     
    
    DEBUG3 ("%s", jsondata.c_str());
    
    AddStatus("Send Confirm...");                                     
}

#pragma endregion Send-Things
#pragma region System-Things
void ChangeBrightness(int B)
{
    #ifdef MODULE_TERMINATOR_PRO
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
void ToggleSwitch(int SNr, int State=2)
{
    int Value = Module.GetPeriphValue(SNr, 0);
    Module.SetPeriphOldValue(SNr, Value, 0);
    
    switch (State)
    {
        case 0: SetRelayState(SNr, false); break;
        case 1: SetRelayState(SNr, true);  break;
        case 2: if (Value == 0) SetRelayState(SNr, true); 
                if (Value == 1) SetRelayState(SNr, false); 
                break;
    }

    UpdateDataFromSwitches();
}
bool GetRelayState(int SNr)
{
    int _Type = Module.GetPeriphType(SNr);
	if ((_Type == SENS_TYPE_LT) or (_Type == SENS_TYPE_LT_AMP))
    {    
        int ADC_Module = Module.GetPeriphI2CPort(SNr, 2);
        
        if (ADC_Module > -1)
        {
            #ifdef ADC0
                //use ADC
                float TempVal  = ADCBoard[ADC_Module].readADC_SingleEnded(Module.GetPeriphIOPort(SNr, 2));
                float TempVolt = ADCBoard[ADC_Module].computeVolts(TempVal) * VOLTAGE_DEVIDER_V; 
                delay(10);
                DEBUG3 ("Relaystate: SNr:%d - TempVal: %.2f - V:%.2f\n\r", SNr, TempVal, TempVolt);
                if (TempVolt > 8) return true;
            #else
                DEBUG1 ("Critical Config-Error ADC - Pos 1");
            #endif
        }
        else
        {
            if (digitalRead(Module.GetPeriphIOPort(SNr, 2))) return true;
        }
    }
    else if ((_Type == SENS_TYPE_SWITCH) or (_Type == SENS_TYPE_SW_AMP))
    {
        int RawState = 0;
        
        int PORT_Module = Module.GetPeriphI2CPort(SNr,2);
        if (PORT_Module > -1)
        {
            #if defined(PORT0) && defined(ADC0)
                RawState = IOBoard[PORT_Module]->digitalRead(Module.GetPeriphIOPort(SNr, 0));
                DEBUG3 ("Relay(%d)-State = %d (IOBoard[PORT_Module]->DigitalRead of port %d)\n\r", SNr, RawState, Module.GetPeriphIOPort(SNr, 0));
            #endif
        }
        else
        {
            RawState = digitalRead(Module.GetPeriphIOPort(SNr, 0));
            //DEBUG3 ("Relay(%d)-State = %d (DigitalRead of port %d)\n\r", SNr, RawState, Module.GetPeriphIOPort(SNr, 0));
        }
        
        if ((RawState == 0) and (Module.GetRelayType() == RELAY_REVERSED)) { return true; }
        if ((RawState == 1) and (Module.GetRelayType() == RELAY_NORMAL))   { return true; }
    }

    return false;
}
void SetRelayState(int SNr, bool State)
{
	int _Type = Module.GetPeriphType(SNr);
	        
    if ((_Type == SENS_TYPE_SWITCH) or (_Type == SENS_TYPE_SW_AMP))
    {
        int PORT_Module = Module.GetPeriphI2CPort(SNr,0);
        if (PORT_Module > -1)
        {
            #ifdef PORT0
                if (Module.GetRelayType() == RELAY_NORMAL) 
                    IOBoard[PORT_Module]->digitalWrite(Module.GetPeriphIOPort(SNr, 0), State);
                else 
                    IOBoard[PORT_Module]->digitalWrite(Module.GetPeriphIOPort(SNr, 0), !State);
            #endif
        }
        else
        {
            if (Module.GetRelayType() == RELAY_REVERSED) 
            {
                digitalWrite(Module.GetPeriphIOPort(SNr, 0), !State);
            }
            else
            {
                digitalWrite(Module.GetPeriphIOPort(SNr, 0), State);
                Serial.printf("Setze Port %d auf %d\n\r",Module.GetPeriphIOPort(SNr, 0), State);
            }
        }
    }
    if ((_Type == SENS_TYPE_LT) or (_Type == SENS_TYPE_LT_AMP))
    {
        int _Port;
        int _PORT_Module;

        if (State == false)
        {
            _Port = Module.GetPeriphIOPort(SNr, 0);
            _PORT_Module = Module.GetPeriphI2CPort(SNr,0);
        }
        else
        {
            _Port = Module.GetPeriphIOPort(SNr, 1);
            _PORT_Module = Module.GetPeriphI2CPort(SNr,1);
        }

        if (_PORT_Module > -1)
        {
            #ifdef PORT0
                IOBoard[_PORT_Module]->digitalWrite(_Port, 1);
                delay(500);
                IOBoard[_PORT_Module]->digitalWrite(_Port, 0);
            #endif
        }
        else
        {
            digitalWrite(_Port, 1);
            DEBUG2 ("Setze _Port:%d auf on\n\r", _Port);
            delay(500); //evtl tiefer
            digitalWrite(_Port, 0);
            DEBUG2 ("Setze _Port:%d auf off\n\r", _Port);
        }
    }
    
}
void PrintMAC(const uint8_t * mac_addr)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if (DEBUG_LEVEL > 0) Serial.print(macStr);
}
void UpdateDataFromSwitches()
{
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)
    {
        if (Module.isPeriphSwitch(SNr))
        {
            Module.SetPeriphValue(SNr, GetRelayState(SNr), 0);
            //Serial.printf("SNr:%d, RelayState:%d, GetPeriphValue:%.2f\n\r", SNr, Value, Module.GetPeriphValue(SNr, 0));
        }
    }
}
void GoToSleep() 
{
    /*JsonDocument doc;
    String jsondata;
    
    doc["Node"] = Module.GetName();   
    doc["Type"] = Module.GetType();
    doc["Msg"]  = "GoodBye - going to sleep";
    
    serializeJson(doc, jsondata);  
        
    esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  //Sending "jsondata"  
    delay(50);
    DEBUG2 ("\nSending: %s\n\r", jsondata.c_str());
    */
    AddStatus("Send Going to sleep......"); 
    
    DEBUG2 ("Going to sleep at: %lu....................................................................................\n\r", millis());
    DEBUG2 ("LastContact    at: %lu\n\r", Module.GetLastContact()); 
    
    #ifdef ESP32
    //gpio_deep_sleep_hold_en();
    //for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) if (Module.GetPeriphType(SNr) == SENS_TYPE_SWITCH) gpio_hold_en((gpio_num_t)Module.GetPeriphIOPort(SNr));  
    
    LEDBlink(4,2,50);
    
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
            DEBUG2 ("Testread Module: %s\n\r", preferences.getString("Module", "").c_str());
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
        if (DEBUG_LEVEL > 0) Serial.printf("ToImport = %s\r\n", ToImport);
        
        if (strcmp(ToImport, "") != 0) Module.Import(ToImport);
        
        if (DEBUG_LEVEL > 0) Serial.printf("Module.Vin[4] = %.2f", Module.GetPeriphVin(4));

    preferences.end();
}
void SetMessageLED(int Color)
{
    // 0-off, 1-Red, 2-Green, 3-Blue, 4=violett
    if (Color > 0) TSLed = millis();
    else TSLed = 0;

    #if defined(LED_PIN) || defined(RGBLED_PIN)    
        if (_LED_SIGNAL) 
        switch (Color)
        {
            case 0: 
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(0, 0, 0);
                #else
                    #ifdef RGBLED_PIN
                        pixels.clear();
                        pixels.show();
                    #endif
                    #ifdef LED_PIN
                        digitalWrite(LED_PIN, LED_OFF);
                    #endif
                #endif
                break;
            case 1:
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(1, 0, 0);
                #else
                    #ifdef RGBLED_PIN
                        pixels.clear();
                        pixels.setPixelColor(0, pixels.Color (255,0,0));
                        pixels.show();
                    #endif
                    #ifdef LED_PIN
                        digitalWrite(LED_PIN, LED_ON);
                    #endif
                #endif
                break;
            case 2:
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(0, 1, 0);
                #else   
                    #ifdef RGBLED_PIN
                        pixels.clear();
                        pixels.setPixelColor(0, pixels.Color (0,255,0));
                        pixels.show();
                    #endif
                    #ifdef LED_PIN
                        digitalWrite(LED_PIN, LED_ON);
                    #endif
                #endif
                break;
            case 3:
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(0, 0, 1);
                #else
                    #ifdef RGBLED_PIN
                        pixels.clear();
                        pixels.setPixelColor(0, pixels.Color (0,0,255));
                        pixels.show();
                    #endif
                    #ifdef LED_PIN
                        digitalWrite(LED_PIN, LED_ON);
                    #endif
                #endif
                break;
            case 4:
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(1, 0, 1);
                #else
                    #ifdef RGBLED_PIN
                        pixels.clear();
                        pixels.setPixelColor(0, pixels.Color (255,0,255));
                        pixels.show();
                    #endif
                    #ifdef LED_PIN
                        digitalWrite(LED_PIN, LED_ON);
                    #endif
                #endif
                break;  
        }
    #endif
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
  
    DEBUG1 ("SNr %d: Volt-Messung kalibrieren... Port: %d, Type:%d", SNr, Module.GetPeriphIOPort(SNr, 2), Module.GetPeriphType(SNr));
    
    if (Module.GetPeriphType(SNr) == SENS_TYPE_VOLT) {
        float TempRead = 0;
        float NewVin = 0;

        for (int i=0; i<20; i++) 
        {
            TempRead += (float)analogRead(Module.GetPeriphIOPort(SNr, 2));
            delay(10);
        }
        TempRead = (float) TempRead / 20;
        
        DEBUG3 ("TempRead nach filter = %.2f\n\r", TempRead);
        DEBUG3 ("Eich-soll Volt: %.2f\n\r", V);
       
        NewVin = TempRead / V * VOLTAGE_DEVIDER_V;
        Module.SetPeriphVin(SNr, NewVin);        
        DEBUG3 ("NewVin = %.2f\n\r", Module.GetPeriphVin(SNr));
        
        DEBUG1 ("S[%d].Vin = %.2f - volt after calibration: %.2fV", SNr, Module.GetPeriphVin(SNr), TempRead/Module.GetPeriphVin(SNr));
        if (DEBUG_LEVEL > 1)  
        {
            snprintf(Buf, sizeof(Buf), "[%d] %s (Type: %d): Spannung ist jetzt: %.2fV", SNr, Module.GetPeriphName(SNr), Module.GetPeriphType(SNr), (float)TempRead/Module.GetPeriphVin(SNr));
            AddStatus(Buf);
        }
        //SendCommand(SEND_CMD_CONFIRM_VOLT);
        SaveModule();
    }
}
void CurrentCalibration() 
{
    char Buf[100] = {};
    
    for(int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
        int _Type = Module.GetPeriphType(SNr);
        if ((_Type == SENS_TYPE_AMP) or (_Type == SENS_TYPE_SW_AMP) or (_Type == SENS_TYPE_LT_AMP)) 
        {
            float TempVolt = 0;
            
            int ADC_Module = Module.GetPeriphI2CPort(SNr, 3);
            if (ADC_Module > -1)
            {
                #ifdef ADC0
                    //for (int i=0; i<20; i++) 
                    {
                        TempVolt += ADCBoard[ADC_Module].computeVolts(ADCBoard[ADC_Module].readADC_SingleEnded(Module.GetPeriphIOPort(SNr, 3)));
                        delay(10);
                    }
                    //TempVolt /= 20;
                #endif
            }
            else
            {
                int TempVal = 0;
                for (int i=0; i<20; i++) 
                {
                    TempVal += analogRead(Module.GetPeriphIOPort(SNr, 3));
                    delay(10);
                }
                TempVal /= 20;
                
                TempVolt = BOARD_VOLTAGE / BOARD_ANALOG_MAX * TempVal;
            }

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
    if (Module.GetPeriphIOPort(SNr,3) < 0) { DEBUG3 ("SNR=%d, no IOPort[3] specified !!!\n\r",SNr);  return 0; }

    float TempVal      = 0;
    float TempVolt     = 0;
    float TempAmp      = 0;
    
    int ADC_Module = Module.GetPeriphI2CPort(SNr, 3);
    float AmpSamples = 0;

    for (int av=0; av<10; av++)
    {
        if (ADC_Module > -1)
        {
            #ifdef ADC0
                TempVal  = ADCBoard[ADC_Module].readADC_SingleEnded(Module.GetPeriphIOPort(SNr, 3));
                TempVolt = ADCBoard[ADC_Module].computeVolts(TempVal); 
                TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr);
            #endif
        }
        else
        {
            TempVal  = analogRead(Module.GetPeriphIOPort(SNr, 3));
            TempVolt = BOARD_VOLTAGE/BOARD_ANALOG_MAX*TempVal;
            TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr) * VOLTAGE_DEVIDER_A;
        }
        AmpSamples += TempAmp;
    }
    
    TempAmp = AmpSamples/10;
    DEBUG3 ("ReadAmp %d - raw %.3f (%.2f)", SNr, TempVolt, TempAmp);
    //DEBUG3 ("ReadAmp: SNr=%d, port=%d: Raw:%.3f=%.3fV Null:%.4f --> %.4fV --> %.4fA", SNr, Module.GetPeriphIOPort(SNr, 3), TempVal, TempVolt, Module.GetPeriphNullwert(SNr), TempVolt, TempAmp);

    if (abs(TempAmp) < SCHWELLE) TempAmp = 0;
    DEBUG3 (" --> %.2fA\n\r", TempAmp);
    
    return (TempAmp); 
}
float ReadVolt(int SNr) 
{
    if (Module.GetPeriphIOPort(SNr, 2) < 0) { DEBUG3 ("SNr=%d - no IOPort[2] - no volt-sensor!!!\n\r", SNr);  return 0; }
    
    float TempVal = 0;
    float TempVolt = 0;
    
    int ADC_Module = Module.GetPeriphI2CPort(SNr, 2);

    float VoltSamples = 0;

    for (int av=0; av<10; av++)
    {
        if (ADC_Module > -1)
        {
            #ifdef ADC0
                //use ADC
                TempVal  = ADCBoard[ADC_Module].readADC_SingleEnded(Module.GetPeriphIOPort(SNr, 2));
                TempVolt = ADCBoard[ADC_Module].computeVolts(TempVal) * VOLTAGE_DEVIDER_V; 
                delay(10);
            #else
                DEBUG1 ("Critical Config-Error ADC");
            #endif
        }
        else
        {
            //use io
            if (Module.GetPeriphVin(SNr) == 0) 
            { 
                //DEBUG3 ("SNr=%d - Vin must not be zero !!!\n\r", SNr); 
                return 0; 
            }
            TempVal  = analogRead(Module.GetPeriphIOPort(SNr, 2));
            TempVolt = (float) TempVal / Module.GetPeriphVin(SNr) * VOLTAGE_DEVIDER_V;
            delay(10);
        }

        VoltSamples += TempVolt;
    }
  
    TempVolt = VoltSamples/10;

    DEBUG2 ("ReadVolt: SNr=%d, port=%d: Raw: %.1f / Vin:%.2f * V-Devider:%.2f--> %.2fV\n\r", SNr, Module.GetPeriphIOPort(SNr, 2), TempVal, Module.GetPeriphVin(SNr), (float) VOLTAGE_DEVIDER_V, TempVolt);
 
    return TempVolt;
}
#pragma endregion Data-Things
#pragma region ESP-Things
void OnDataRecvCommon(const uint8_t * dummymac, const uint8_t *incomingData, int len)  
{  
    char* buff = (char*) incomingData;        //char buffer
    JsonDocument doc;
    String jsondata;
    int Pos = -1;
    float  NewVperAmp  = 0;
    float  NewVin      = 0;
    float  NewVoltage  = 0;
    float  NewNullwert = 0;
    String NewName     = "";

    jsondata = String(buff);                 
    DeserializationError error = deserializeJson(doc, jsondata);

    if (!error) 
    {
        uint8_t _From[6];
        uint8_t _To[6];
        
        String MacFromS;
        String MacToS;
        uint32_t _TS;

        if ( JX(SEND_CMD_JSON_FROM) and JX(SEND_CMD_JSON_TO) and JX(SEND_CMD_JSON_TS))
        {
            MacFromS = (String) doc[SEND_CMD_JSON_FROM];
            MacCharToByte(_From, (char *) MacFromS.c_str());
            MacToS = (String) doc[SEND_CMD_JSON_TO];
            MacCharToByte(_To, (char *) MacToS.c_str());
            _TS = (uint32_t)doc[SEND_CMD_JSON_TS];
        }
        else
        {
            return;
        }

        if ( (memcmp(_To, Module.GetBroadcastAddress(), 6) == 0) or (memcmp(_To, broadcastAddressAll, 6) == 0) )
        {
            DEBUG3 ("%lu: Recieved from: %s\n\r", _TS, (char *)MacFromS.c_str()); 

            //already recevied?
            if (ReceivedMessagesList.size() > 0)
            { 
                for (int i=ReceivedMessagesList.size()-1; i>=0; i--)
                {
                    ReceivedMessagesStruct *RMItem = ReceivedMessagesList.get(i);
                    
                    if ( (memcmp(RMItem->From, _From, 6) == 0) and (RMItem->TS ==_TS) )
                    {
                        DEBUG3 ("Message %lu: %s schon verarbeitet\n\r", _TS, MacFromS.c_str());
                        return;
                    }
                }
            }     
            
            ReceivedMessagesStruct *RMItem = new ReceivedMessagesStruct;
            memcpy(RMItem->From, _From, 6);
            RMItem->TS = _TS;
            RMItem->SaveTime = millis();
            ReceivedMessagesList.add(RMItem);
            DEBUG3 ("%d.Message %lu: %s gespeichert\n\r", ReceivedMessagesList.size(), _TS, MacFromS.c_str());
            
            if (JX(SEND_CMD_JSON_CONFIRM)) SendConfirm(_From, _TS);
            
            if (JX(SEND_CMD_JSON_ORDER))
            {
                switch ((int) doc[SEND_CMD_JSON_ORDER]) 
                {
                    case SEND_CMD_YOU_ARE_PAIRED:
                        if (esp_now_is_peer_exist((unsigned char *) _From)) 
                        { 
                            DEBUG1 ("%s already exists...\n\r", MacFromS.c_str()); 
                        }
                        else 
                        {
                            PeerClass *Peer = new PeerClass;
                            Peer->Setup(doc[SEND_CMD_JSON_PEER_NAME], (int) doc[SEND_CMD_JSON_MODULE_TYPE], MODULE_VERSION, _From, false, false, false, false);
                            Peer->SetLastContact(millis());
                            WaitForContact = WAIT_AFTER_SLEEP; 
                            PeerList.add(Peer);

                            SavePeers();
                            RegisterPeers();
                            
                            if (DEBUG_LEVEL > 1) 
                            {
                                Serial.printf("New Peer added: %s (Type:%d), MAC:%s\n\r", Peer->GetName(), Peer->GetType(), MacFromS.c_str());
                                Serial.println("Saving Peers after received new one...");
                                ReportAll();
                            }
                        }
                        Module.SetPairMode(false);
                        break;
                    case SEND_CMD_STAY_ALIVE: 
                        Module.SetLastContact(millis());
                        WaitForContact = WAIT_ALIVE; 
                        DEBUG2 ("LastContact: %6lu\n\r", Module.GetLastContact());
                        break;
                    case SEND_CMD_SLEEPMODE_ON:
                        AddStatus("Sleep: on");  
                        SetSleepMode(true);  
                        SendStatus();
                        break;
                    case SEND_CMD_SLEEPMODE_OFF:
                        AddStatus("Sleep: off"); 
                        SetSleepMode(false); 
                        SendStatus();
                        break;
                    case SEND_CMD_SLEEPMODE_TOGGLE:
                        if (Module.GetSleepMode()) 
                        { 
                            AddStatus("Sleep: off");   
                            SetSleepMode(false); 
                            SendStatus();
                        }
                        else 
                        { 
                            AddStatus("Sleep: on");    
                            SetSleepMode(true);  
                            SendStatus();
                        }
                        break;
                    case SEND_CMD_DEBUGMODE_ON:
                        AddStatus("DebugMode: on");  
                        SetDebugMode(true);  
                        SaveModule();
                        SendStatus();
                        break;
                    case SEND_CMD_DEBUGMODE_OFF:
                        AddStatus("DebugMode: off"); 
                        SetDebugMode(false); 
                        SaveModule();
                        SendStatus();
                        break;
                    case SEND_CMD_DEBUGMODE_TOGGLE:
                        if (Module.GetDebugMode()) 
                        {   
                            AddStatus("DebugMode: off");   
                            SetDebugMode(false);  
                        }
                        else 
                        { 
                            AddStatus("DebugMode: on");    
                            SetDebugMode(true);  
                        }
                        SendStatus();
                        break;
                    case SEND_CMD_DEMOMODE_ON:
                        AddStatus("Demo: on");   
                        SetDemoMode(true);   
                        SendStatus();
                        break;
                    case SEND_CMD_DEMOMODE_OFF:
                        AddStatus("Demo: off");  
                        SetDemoMode(false);  
                        SendStatus();
                        break;
                    case SEND_CMD_DEMOMODE_TOGGLE:
                        if (Module.GetDemoMode()) 
                        { 
                            AddStatus("DemoMode: off"); 
                            SetDemoMode(false);
                        }
                        else 
                        { 
                            AddStatus("DemoMode: on");  
                            SetDemoMode(true);  
                        }
                        SendStatus();
                        break;
                    case SEND_CMD_RESET:
                        AddStatus("Clear all"); 
                        #ifdef ESP32
                            ClearPeers(); ClearInit(); nvs_flash_erase(); nvs_flash_init();
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
                        TSPair = millis();    
                        AddStatus("Pairing beginnt"); 
                        SendStatus();
                        #ifdef MODULE_TERMINATOR_PRO
                        smartdisplay_led_set_rgb(1,0,0);
                        #endif
                        break;
                    case SEND_CMD_CURRENT_CALIB:
                        AddStatus("Eichen beginnt"); 
                        CurrentCalibration();
                        break;
                    case SEND_CMD_VOLTAGE_CALIB:
                        if (JX(SEND_CMD_JSON_VALUE))
                        {
                            AddStatus("VoltCalib beginnt");
                            NewVoltage = (float) doc[SEND_CMD_JSON_VALUE];
                            for (int SNr=0 ; SNr<MAX_PERIPHERALS; SNr++)
                            {
                                if (Module.GetPeriphType(SNr) == SENS_TYPE_VOLT)
                                { 
                                    VoltageCalibration(SNr, NewVoltage) ;
                                    break;
                                }
                            } 
                        }                
                        break;
                    case SEND_CMD_SWITCH_TOGGLE:
                        if (JX(SEND_CMD_JSON_PERIPH_POS))    
                        {
                            Pos = doc[SEND_CMD_JSON_PERIPH_POS];
                            if (Module.isPeriphEmpty(Pos) == false) ToggleSwitch(Pos);
                        }
                        break;
                    case SEND_CMD_UPDATE_NAME:
                        if ( JX(SEND_CMD_JSON_PERIPH_POS) and JX(SEND_CMD_JSON_VALUE) )
                        {
                            Pos = (int) doc[SEND_CMD_JSON_PERIPH_POS];
                            NewName = doc[SEND_CMD_JSON_VALUE].as<String>();
                            if (NewName != "") 
                            {
                                if (Pos == 99) Module.SetName(NewName.c_str());
                                else           Module.SetPeriphName(Pos, NewName.c_str());
                            }
                            
                            SaveModule();
                            NameChanged = true;
                        }
                            //SendNameChange(Pos);
                        break;
                    case SEND_CMD_UPDATE_VIN:
                        if  ( JX(SEND_CMD_JSON_VALUE) and JX(SEND_CMD_JSON_PERIPH_POS) )
                        {   
                            NewVin = (float) doc[SEND_CMD_JSON_VALUE];
                            Pos = (int) doc[SEND_CMD_JSON_PERIPH_POS];

                            if (NewVin > 0)
                            {
                                Module.SetPeriphVin(Pos, NewVin);
                                SaveModule();
                            }
                        }
                        break;
                    case SEND_CMD_UPDATE_VPERAMP:
                        if  ( JX(SEND_CMD_JSON_VALUE) and JX(SEND_CMD_JSON_PERIPH_POS) )
                        {
                            Pos = (int) doc[SEND_CMD_JSON_PERIPH_POS];
                            NewVperAmp = (float) doc[SEND_CMD_JSON_VALUE];

                            if (NewVperAmp > 0)
                            {
                                Module.SetPeriphVperAmp(Pos, NewVperAmp);
                                SaveModule();
                                DEBUG1 ("Updated VperAmp at Pos:%d to %.3f\n\r", Pos, NewVperAmp);
                            }
                        }
                        break;
                    case SEND_CMD_UPDATE_NULLWERT:
                        if  ( JX(SEND_CMD_JSON_VALUE) and JX(SEND_CMD_JSON_PERIPH_POS) )
                        {
                            Pos = (int) doc[SEND_CMD_JSON_PERIPH_POS];
                            NewNullwert = (float) doc[SEND_CMD_JSON_VALUE];

                            if (NewNullwert > 0)
                            {
                                Module.SetPeriphNullwert(Pos, NewNullwert);
                                SaveModule();
                                DEBUG1 ("Updated Nullwert at Pos:%d to %.3f\n\r", Pos, NewNullwert);
                            }
                        }
                        break;
                    case SEND_CMD_SEND_STATE:
                        if (JX(SEND_CMD_JSON_PERIPH_POS))    
                        {
                            Pos = (int) doc[SEND_CMD_JSON_PERIPH_POS];
                            SendStatus(Pos);
                        }
                        break;
                }
            }
        } 
    } // end (!error)
    else // error
    { 
          DEBUG1 ("deserializeJson failed: %s\n\r", error.c_str());
    }
}
#ifdef ESP32 
void OnDataRecv(const esp_now_recv_info *info, const uint8_t* incomingData, int len)
{
    OnDataRecvCommon(info->src_addr, incomingData, len);
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
    //if (DEBUG_LEVEL > 2) 
    //    if (status == ESP_NOW_SEND_SUCCESS) Serial.println("\r\nLast Packet Send Status: Delivery Success");
        
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
    uint32_t actTime = millis();
    
    if  ((actTime - TSSend ) > MSG_INTERVAL/2  )                                 // Send-interval (Message or Pairing-request)
    {
        TSSend = actTime;
        if (Module.GetPairMode()) SendPairingRequest();
        else SendStatus();
        GarbageMessages();
    }

    if  ((actTime - TSCheckRel ) > RELAY_CHECK )                                 // Check Relay-State
    {
        TSCheckRel = actTime;
        UpdateDataFromSwitches();
    }

    if (((actTime - TSPair ) > PAIR_INTERVAL ) and (Module.GetPairMode()))     // end Pairing after pairing interval
    {
        TSPair = 0;
        Module.SetPairMode(false);
        AddStatus("Pairing beendet...");
        SetMessageLED(0);
    }

    if ((millis() - TSLed > MSGLIGHT_INTERVAL) and (TSLed > 0))                 // clear LED after LED interval
    {
        if (Module.GetPairMode())
            SetMessageLED(1);
        else
            SetMessageLED(0);
    }

    if ((Module.GetSleepMode()) and (!Module.GetPairMode()) and (actTime+100 - Module.GetLastContact() > WaitForContact))       
    {
        DEBUG1 ("actTime:%lu, LastContact:%lu - (actTime - Module.GetLastContact()) = %lu, WaitForContact = %lu, - Try to sleep...........................................................\n\r", actTime, Module.GetLastContact(), actTime - Module.GetLastContact(), WaitForContact);
        Module.SetLastContact(millis());
        GoToSleep();
    }

    #ifdef PAIRING_BUTTON                                                       // check for Pairing/Reset Button
        int BB1 = !digitalRead(PAIRING_BUTTON);
        int BB2 = 0;//!digitalRead(0);
        if ((BB1 == 1) or (BB2 == 1)) 
        {
            TSPair = actTime;
            Module.SetPairMode(true);
            SetMessageLED(1);
    
            AddStatus("Pairing beginnt...");
            
            if (!TSButton) TSButton = actTime;
            else 
            {
                if ((actTime - TSButton) > 5000) {
                    DEBUG1 ("Button pressed... Clearing Peers and Reset");
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

    #ifdef MODULE_HAS_DISPLAY                                               // start ui if module has display
        lv_timer_handler();
        delay(5);
    #endif
}

void MacCharToByte(uint8_t *mac, char *MAC)
{
    sscanf(MAC, "%2x%2x%2x%2x%2x%2x", (unsigned int*) &mac[0], (unsigned int*) &mac[1], (unsigned int*) &mac[2], (unsigned int*) &mac[3], (unsigned int*) &mac[4], (unsigned int*) &mac[5]);
}
void MacByteToChar(char *MAC, uint8_t *mac)
{
    sprintf(MAC, "%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool MACequals( uint8_t *MAC1, uint8_t *MAC2)
{
    for (int i=0; i<6; i++)
    {
        if (MAC1[i] != MAC2[i]) return false;
    }
    return true;
}
void InitSCL()
{
    if (DEBUG_LEVEL > 0)
    {
        #if defined(PORT0) || defined(ADC0)
            byte error, address;
            int nDevices;
            Serial.println("Scanning...");
            nDevices = 0;

            Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);

            for(address = 1; address < 127; address++ )
            {
                // The i2c_scanner uses the return value of
                // the Write.endTransmisstion to see if
                // a device did acknowledge to the address.
                Wire.beginTransmission(address);
                error = Wire.endTransmission();
                if (error == 0)
                {
                Serial.print("I2C device found at address 0x");
                if (address<16)
                    Serial.print("0");
                Serial.print(address,HEX);
                Serial.println("  !");
                nDevices++;
                }
                else if (error==4)
                {
                Serial.print("Unknown error at address 0x");
                if (address<16)
                    Serial.print("0");
                Serial.println(address,HEX);
                }    
            }
            if (nDevices == 0)
            {
                Serial.println("No I2C devices found\n");
                while(1);
            }
            else
            {
                Serial.println("done\n");
                delay(1000);
            }
        #endif
    }

    #ifdef PORT0                            // init IOBoard0
	    IOBoard[0] = &IOBoard0;  
        for (int i=0; i<16; i++) IOBoard[0]->digitalWrite(i, 0);
        DEBUG1 ("IOBoard0 initialised.\n\r");
    #endif
    #ifdef PORT1                            // init IOBoard1
        IOBoard[1] = &IOBoard1; 
        for (int i=0; i<16; i++) IOBoard[1]->digitalWrite(i, 0);
        DEBUG1 ("IOBoard1 initialised.\n\r");
    #endif
    #ifdef PORT2                            // init IOBoard2
        IOBoard[2] = &IOBoard2; 
        for (int i=0; i<16; i++) IOBoard[2]->digitalWrite(i, 0);
        DEBUG1 ("IOBoard2 not found!\n\r");
    #endif
    #ifdef PORT3                            // init IOBoard3
        IOBoard[3] = &IOBoard3; 
        for (int i=0; i<16; i++) IOBoard[3]->digitalWrite(i, 0); 
        DEBUG1 ("IOBoard3 not found!\n\r");
    #endif
    #ifdef ADC0                             // init ADS
        ADCBoard[0].setGain(GAIN_TWOTHIRDS);   // 0.1875 mV/Bit .... +- 6,144V
        if (!ADCBoard[0].begin(ADC0)) { 
            DEBUG1 ("ADC0 not found!\n\r");
            while (1);
        }
        else
        {
            DEBUG2 ("ADC0 initialised.\n\r");
        }
    #endif
    #ifdef ADC1                             // init ADS
        ADCBoard[1].setGain(GAIN_TWOTHIRDS);   // 0.1875 mV/Bit .... +- 6,144V
        if (!ADCBoard[1].begin(ADC1)) { 
            DEBUG1 ("ADC1 not found!\n\r");
            while (1);
        }
        else
        {
            DEBUG2 ("ADC1 initialised.\n\r");
        }
    #endif
    #ifdef ADC2                             // init ADS
        ADCBoard[2].setGain(GAIN_TWOTHIRDS);   // 0.1875 mV/Bit .... +- 6,144V
        if (!ADCBoard[2].begin(ADC2)) { 
            DEBUG1 ("ADC2 not found!\n\r");
            while (1);
        }
        else
        {
            DEBUG2 ("ADC2 initialised.\n\r");
        }
    #endif
    #ifdef ADC3                            // init ADS
        ADCBoard[3].setGain(GAIN_TWOTHIRDS);   // 0.1875 mV/Bit .... +- 6,144V
        if (!ADCBoard[3].begin(ADC3)) { 
            DEBUG1 ("ADC3 not found!\n\r");
            while (1);
        }
        else
        {
            DEBUG2 ("ADC3 initialised.\n\r");
        }
    #endif
}
void InitMRD()
{
    #ifdef MRD_USED                             // MultiReset-Check
        mrd = new MultiResetDetector(MRD_TIMEOUT, MRD_ADDRESS);

        if (mrd->detectMultiReset()) {
          DEBUG1 ("Multi Reset Detected\n\r");
          digitalWrite(LED_BUILTIN, LED_ON);
          //ClearPeers(); ClearInit(); InitModule(); SaveModule(); delay(10000); ESP.restart();
          Module.SetPairMode(true); TSPair = millis();
        }
        else {
          DEBUG1 ("No Multi Reset Detected\n\r");
          digitalWrite(LED_BUILTIN, LED_OFF);
        }
    #endif
}