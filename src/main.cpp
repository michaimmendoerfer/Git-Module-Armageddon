//#define KILL_NVS 1

// DEBUG_LEVEL: 0 = nothing, 1 = only Errors, 2 = relevant changes, 3 = all
#define DEBUG1(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 0)) Serial.printf(__VA_ARGS__)
#define DEBUG2(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 1)) Serial.printf(__VA_ARGS__)
#define DEBUG3(...) if ((Module.GetDebugMode()) and (DEBUG_LEVEL > 2)) Serial.printf(__VA_ARGS__)

#include <Arduino.h>
#include <Module.h>

const int DEBUG_LEVEL = 3; 
const int _LED_SIGNAL = 1;

#define WAIT_ALIVE       15000
#define WAIT_AFTER_SLEEP  3000

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

#if defined(PORT_USED) || defined(ADC_USED)
    #include <Wire.h>
    #include <Spi.h>
    #define I2C_FREQ 400000
    #ifdef ADC_USED
        #include <Adafruit_ADS1X15.h>
        Adafruit_ADS1115 ADCBoard;
    #endif
    #ifdef PORT_USED
    #include "PCF8575.h"
        PCF8575 IOBoard(PORT_USED, SDA_PIN, SCL_PIN);
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

PeerClass Module;
MyLinkedList<PeriphClass*> SwitchList = MyLinkedList<PeriphClass*>();
MyLinkedList<PeriphClass*> SensorList = MyLinkedList<PeriphClass*>();

struct_Status Status[MAX_STATUS];

u_int8_t broadcastAddressAll[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; 

volatile uint32_t TSButton   = 0;
volatile uint32_t TSSend     = 0;
volatile uint32_t TSPair     = 0;
volatile uint32_t TSLed      = 0;
volatile uint32_t TSStatus   = 0;
volatile uint32_t TSSettings = 0;

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

void setup()
{
    #ifdef ARDUINO_USB_CDC_ON_BOOT
        delay(3000);
    #endif
    #ifdef LED_PIN
        pinMode(LED_PIN, OUTPUT);
    #endif
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

    #ifdef ESP32
        Serial.begin(115200);
    #elif defined(ESP8266)
        Serial.begin(74880);
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
    
    if (preferences.begin("JeepifyInit", true)) // import saved Module... if available
    {
        String SavedModule   = preferences.getString("Module", "");
            DEBUG2 ("Importiere Modul: %s\n\r", SavedModule.c_str());
            char ToImport[250];
            strcpy(ToImport,SavedModule.c_str());
            if (strcmp(ToImport, "") != 0) Module.Import(ToImport);
        preferences.end();
    }
    
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) Module.SetPeriphValue(SNr, GetRelayState(SNr), 0);
    UpdateSwitches();

    WiFi.mode(WIFI_STA);
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
    
    #ifdef KILL_NVS
        nvs_flash_erase(); nvs_flash_init(); while (1) {};
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
    
    #ifdef MODULE_HAS_DISPLAY
      ui_init();
    #endif
    
}
#pragma region Send-Things
void SendStatus (int Pos) 
{
    TSLed = millis();
    SetMessageLED(2);

    JsonDocument doc; String jsondata; 
    char buf[200]; 
    char bufV[10];
    
    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);    
    
    snprintf(buf, sizeof(buf), "%s;%lu;%d", Module.GetName(), millis(), Status);
    doc["Node"]  = buf;
    
    int SNrStart = 0;
    int SNrMax = MAX_PERIPHERALS;
    if ((Pos >= 0) and (Pos < MAX_PERIPHERALS))
    {
        SNrStart = Pos;
        SNrMax   = Pos+1;
        doc["Order"] = SEND_CMD_RETURN_STATE; 
    }  
    else 
    {
        doc["Order"] = SEND_CMD_STATUS;
    }

    int PeriphsSent = 0;
    for (int SNr=SNrStart; SNr<SNrMax ; SNr++) 
    {   
        if (!Module.isPeriphEmpty(SNr))
        {
            DEBUG3 ("SendStatus(%d) - %s (Type %d):\n\r",SNr, Module.GetPeriphName(SNr), Module.GetPeriphType(SNr));

            if (GetRelayState(SNr)) Module.SetPeriphValue(SNr, 1, 0);
            else Module.SetPeriphValue(SNr, 0, 0);
            
            DEBUG3 ("GetPeriphValue(%d, 0) = %d\n\r", SNr, Module.GetPeriphValue(SNr, 0));
            
            Module.SetPeriphValue(SNr, ReadVolt(SNr),      2);
            Module.SetPeriphValue(SNr, ReadAmp(SNr),       3);
            
            snprintf(buf, sizeof(buf), "%d;%s;%.0f;%.0f;%.2f;%.2f", 
            Module.GetPeriphType(SNr), 
            Module.GetPeriphName(SNr), 
            Module.GetPeriphValue(SNr, 0),
            
            Module.GetPeriphValue(SNr, 1),
            Module.GetPeriphValue(SNr, 2),
            Module.GetPeriphValue(SNr, 3));
                     
            doc[ArrPeriph[SNr]] = buf;
            PeriphsSent++;

            //send first 4 Periphs
            if (PeriphsSent == 4)
            {
                serializeJson(doc, jsondata);  

                for (int PNr=0; PNr<PeerList.size(); PNr++) 
                {
                    PeerClass *Peer = PeerList.get(PNr);

                    if (Peer->GetType() >= MONITOR_ROUND)
                    {
                        DEBUG3 ("Sending to: %s ", Peer->GetName()); 
                        if (esp_now_send(Peer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 250) == 0) DEBUG3("ESP_OK\\r");  
                        else DEBUG1 ("ESP_ERROR\n\r"); 
                        DEBUG3 ("Länge: %d - %s\n\r", strlen(jsondata.c_str()), jsondata.c_str());
                    }
                }
                doc.remove(ArrPeriph[0]);
                doc.remove(ArrPeriph[1]);
                doc.remove(ArrPeriph[2]);
                doc.remove(ArrPeriph[3]);

                PeriphsSent = 0;
                delay(100);//?
            }
        }
	}
	
	serializeJson(doc, jsondata);  

    for (int PNr=0; PNr<PeerList.size(); PNr++) 
    {
        PeerClass *Peer = PeerList.get(PNr);

        if (Peer->GetType() >= MONITOR_ROUND)
        {
            DEBUG3 ("Sending to: %s ", Peer->GetName()); 
            if (esp_now_send(Peer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 250) == 0) DEBUG3("ESP_OK\\r");  
            else DEBUG1 ("ESP_ERROR\n\r"); 
            DEBUG3 ("Länge: %d - %s\n\r", strlen(jsondata.c_str()), jsondata.c_str());
        }
    }
}

void SendPairingRequest() 
{
    TSLed = millis();
    SetMessageLED(3);
    
    JsonDocument doc; String jsondata; 
    char buf[200] = {};

    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);    
    
    snprintf(buf, sizeof(buf), "%s;%lu;%d", Module.GetName(), millis(), Status);
    doc["Node"]    = buf;
    doc["Type"]    = Module.GetType();
    doc["Version"] = Module.GetVersion();
    doc["Order"]   = SEND_CMD_PAIR_ME;
    
    for (int SNr=0 ; SNr<MAX_PERIPHERALS; SNr++) {
        if (!Module.isPeriphEmpty(SNr)) 
        {
            snprintf(buf, sizeof(buf), "%d;%s", Module.GetPeriphType(SNr), Module.GetPeriphName(SNr));
            doc[ArrPeriph[SNr]] = buf;
	    }
    }
         
    serializeJson(doc, jsondata);  

    esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  
    
    DEBUG2 ("\nSending: %s\n\r", jsondata.c_str()); 
    
    //AddStatus("Send Pairing request...");                                     
}

void SendConfirm(const uint8_t * mac, uint32_t TSConfirm) 
{
    TSLed = millis();
    SetMessageLED(3);
    
    JsonDocument doc; String jsondata; 
    char buf[100];
    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);   

    snprintf(buf, sizeof(buf), "%s;%lu;%d", Module.GetName(), millis(), Status);
    doc["Node"]        = buf;
    doc["Order"]       = SEND_CMD_CONFIRM;
    doc["TSConfirm"]   = TSConfirm;

    serializeJson(doc, jsondata); 

    DEBUG2 ("%lu: Sending Confirm (%lu) to: %s ", millis(), TSConfirm, FindPeerByMAC(mac)->GetName()); 
            
    if (esp_now_send((unsigned char *) mac, (uint8_t *) jsondata.c_str(), 200) == 0) 
    {
            DEBUG3 ("ESP_OK\n\r");  //Sending "jsondata" 
    } 
    else 
    {
            DEBUG1 ("ESP_ERROR\n\r"); 
    }     
    
    DEBUG3 (jsondata.c_str());
    
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
Serial.printf("Value vor switch:%d\n\r", Value);
    Module.SetPeriphOldValue(SNr, Value, 0);
    
    switch (State)
    {
        case 0: Value = 0; break;
        case 1: Value = 1; break;
        case 2: Value = Value ? 0 : 1; break;
    }
    Module.SetPeriphValue(SNr, Value, 0);
    DEBUG3 ("Value nach switch:%d\n\r", Module.GetPeriphValue(SNr, 0));
    UpdateSwitches();
}
bool GetRelayState(int SNr)
{
	DEBUG3 ("GetRelayState(%d):\n\r", SNr);

    int _Type = Module.GetPeriphType(SNr);
	if ((_Type == SENS_TYPE_LT) or (_Type == SENS_TYPE_LT_AMP))
        {
            	if (ReadVolt(SNr) > 5) return true;
        }
    else if ((_Type == SENS_TYPE_SWITCH) or (_Type == SENS_TYPE_SW_AMP))
        {
            int RawState = 0;
            
            #ifdef PORT_USED
                RawState = IOBoard.digitalRead(Module.GetPeriphIOPort(SNr, 0));
                DEBUG3 ("Relay(%d)-State = %d (IOBoard.DigitalRead of port %d)", SNr, RawState, Module.GetPeriphIOPort(SNr, 0));
            #else
                RawState = digitalRead(Module.GetPeriphIOPort(SNr, 0));
                DEBUG3 ("Relay(%d)-State = %d (DigitalRead of port %d)", SNr, RawState, Module.GetPeriphIOPort(SNr, 0));
            #endif
            
            if ((RawState == 0) and (Module.GetRelayType() == RELAY_REVERSED)) { DEBUG3 ("Relaystate Ende"); return true;}
            if ((RawState == 1) and (Module.GetRelayType() == RELAY_NORMAL))   { DEBUG3 ("Relaystate Ende"); return true;}
        }
	DEBUG3 ("Relaystate Ende");
    return false;
}
void SetRelayState(int SNr, bool State)
{
	int _Type = Module.GetPeriphType(SNr);
	
    if ((_Type == SENS_TYPE_SWITCH) or (_Type == SENS_TYPE_SW_AMP))
    {
        #ifdef PORT_USED
            if (Module.GetRelayType() == RELAY_NORMAL) 
                IOBoard.digitalWrite(Module.GetPeriphIOPort(SNr, 0), State);
            else 
                IOBoard.digitalWrite(Module.GetPeriphIOPort(SNr, 0), !State);
        #else
            if (Module.GetRelayType() == RELAY_NORMAL) 
                digitalWrite(Module.GetPeriphIOPort(SNr, 0), State);
            else
                digitalWrite(Module.GetPeriphIOPort(SNr, 0), !State);
        #endif
    }
    if ((_Type == SENS_TYPE_LT) or (_Type == SENS_TYPE_LT_AMP))
    {
        int _Port;
        if (State == false) _Port = Module.GetPeriphIOPort(SNr, 0);
        else _Port = Module.GetPeriphIOPort(SNr, 1);

        #ifdef PORT_USED
            IOBoard.digitalWrite(_Port, 1);
            delay(100);
            IOBoard.digitalWrite(_Port, 0);
        #else
            digitalWrite(_Port, 1);
            DEBUG2 ("Setze _Port:%d auf on\n\r", _Port);
            delay(1000);
            digitalWrite(_Port, 0);
            DEBUG2 ("Setze _Port:%d auf off\n\r", _Port);
        #endif
    }
    
}
void UpdateSwitches() 
{
	Serial.println("UpdateSwitches");
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) 
	{
		if ((Module.GetPeriphType(SNr) > 0) and (Module.GetPeriphValue(SNr, 0) != GetRelayState(SNr)))
        {
            if (Module.GetPeriphValue(SNr, 0) == 0) SetRelayState(SNr, 0);
            else SetRelayState(SNr, 1);
            DEBUG3 ("Setze %s (Port:%d) auf %.0f\n\r", Module.GetPeriphName(SNr), Module.GetPeriphIOPort(SNr, 0), Module.GetPeriphValue(SNr, 0));
        }
    }
    SendStatus(0);
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
    delay(500);
    DEBUG2 ("\nSending: %s\n\r", jsondata.c_str());
    AddStatus("Send Going to sleep......"); 
    
    DEBUG2 ("Going to sleep at: %lu....................................................................................\n\r", millis());
    DEBUG2 ("LastContact    at: %lu\n\r", Module.GetLastContact()); 
    
    #ifdef ESP32
    //gpio_deep_sleep_hold_en();
    //for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) if (Module.GetPeriphType(SNr) == SENS_TYPE_SWITCH) gpio_hold_en((gpio_num_t)Module.GetPeriphIOPort(SNr));  
    
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
    #if defined(LED_PIN) || defined(RGBLED_PIN)    
        if (_LED_SIGNAL) 
        switch (Color)
        {
            case 0: 
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(0, 0, 0);
                #elif RGBLED_PIN
                    pixels.clear();
                    pixels.show();
                #else
                    digitalWrite(LED_PIN, LED_OFF);
                #endif
                break;
            case 1:
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(1, 0, 0);
                #elif RGBLED_PIN
                    pixels.clear();
                    pixels.setPixelColor(0, pixels.Color (255,0,0));
                    pixels.show();
                #else
                    digitalWrite(LED_PIN, LED_ON);
                #endif
                break;
            case 2:
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(0, 1, 0);
                #elif RGBLED_PIN
                    pixels.clear();
                    pixels.setPixelColor(0, pixels.Color (0,255,0));
                    pixels.show();
                #else
                    digitalWrite(LED_PIN, LED_ON);
                #endif
                break;
            case 3:
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(0, 0, 1);
                #elif RGBLED_PIN
                    pixels.clear();
                    pixels.setPixelColor(0, pixels.Color (0,0,255));
                    pixels.show();
                #else
                    digitalWrite(LED_PIN, LED_ON);
                #endif
                break;
            case 4:
                #ifdef MODULE_TERMINATOR_PRO
                    smartdisplay_led_set_rgb(1, 0, 1);
                #elif RGBLED_PIN
                    pixels.clear();
                    pixels.setPixelColor(0, pixels.Color (255,0,255));
                    pixels.show();
                #else
                    digitalWrite(LED_PIN, LED_ON);
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
  
    if (DEBUG_LEVEL > 1) Serial.printf("SNr %d: Volt-Messung kalibrieren... Port: %d, Type:%d", SNr, Module.GetPeriphIOPort(SNr, 2), Module.GetPeriphType(SNr));
    
    if (Module.GetPeriphType(SNr) == SENS_TYPE_VOLT) {
        float TempRead = 0;
        float NewVin = 0;

        for (int i=0; i<20; i++) 
        {
            TempRead += (float)analogRead(Module.GetPeriphIOPort(SNr, 2));
            delay(10);
        }
        TempRead = (float) TempRead / 20;
        
        if (DEBUG_LEVEL > 0) Serial.printf("TempRead nach filter = %.2f\n\r", TempRead);
        if (DEBUG_LEVEL > 0) Serial.printf("Eich-soll Volt: %.2f\n\r", V);
       
        NewVin = TempRead / V * VOLTAGE_DEVIDER;
        Module.SetPeriphVin(SNr, NewVin);        
        if (DEBUG_LEVEL > 0) Serial.printf("NewVin = %.2f\n\r", Module.GetPeriphVin(SNr));
        
        if (DEBUG_LEVEL > 2)  Serial.printf("S[%d].Vin = %.2f - volt after calibration: %.2fV", SNr, Module.GetPeriphVin(SNr), TempRead/Module.GetPeriphVin(SNr));
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
            
            #ifdef ADC_USED
                //for (int i=0; i<20; i++) 
                {
                    TempVolt += ADCBoard.computeVolts(ADCBoard.readADC_SingleEnded(Module.GetPeriphIOPort(SNr, 3)));
                    delay(10);
                }
                //TempVolt /= 20;
            #else
                int TempVal = 0;
                for (int i=0; i<20; i++) 
                {
                    TempVal += analogRead(Module.GetPeriphIOPort(SNr, 3));
                    delay(10);
                }
                TempVal /= 20;
                
                TempVolt = BOARD_VOLTAGE / BOARD_ANALOG_MAX * TempVal; // 1.5???  
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
    //SendCommand(SEND_CMD_CONFIRM_CURRENT);
    SaveModule();
}
float ReadAmp (int SNr) 
{
    if (Module.GetPeriphIOPort(SNr,3) < 0) { if (DEBUG_LEVEL > 0) Serial.printf("SNR=%d, no IOPort[3] specified !!!\n\r",SNr);  return 0; }

    float TempVal      = 0;
    float TempVolt     = 0;
    float TempAmp      = 0;
  
    #ifdef ADC_USED
        TempVal  = ADCBoard.readADC_SingleEnded(Module.GetPeriphIOPort(SNr, 3));
        TempVolt = ADCBoard.computeVolts(TempVal); 
        TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr);
        delay(10);
    #else
        TempVal  = analogRead(Module.GetPeriphIOPort(SNr, 3));
        Serial.printf("ReadAmp: SNr=%d - analogRead of port %d = %.0f", SNr, Module.GetPeriphIOPort(SNr, 3), TempVal);

        TempVolt = BOARD_VOLTAGE/BOARD_ANALOG_MAX*TempVal;
        TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr) * VOLTAGE_DEVIDER;// 1.5 wegen Voltage-Devider
        delay(10);
    #endif
  
    if (DEBUG_LEVEL > 2) {
        Serial.printf("ReadAmp: SNr=%d, port=%d: Raw:%.3f Null:%.4f --> %.4fV --> %.4fA", SNr, Module.GetPeriphIOPort(SNr, 3), TempVal, Module.GetPeriphNullwert(SNr), TempVolt, TempAmp);
    } 
    if (abs(TempAmp) < SCHWELLE) TempAmp = 0;
    if (DEBUG_LEVEL > 2) {
        Serial.printf(" --> %.2fA\n\r", TempAmp);
    } 
    
    return (TempAmp); 
}
float ReadVolt(int SNr) 
{
    if (Module.GetPeriphVin(SNr)      == 0) { if (DEBUG_LEVEL > 0) Serial.printf("SNr=%d - Vin must not be zero !!!", SNr);    return 0; }
    if (Module.GetPeriphIOPort(SNr, 2) < 0) { if (DEBUG_LEVEL > 0) Serial.printf("SNr=%d - no IOPort[2] specified !!!", SNr);  return 0; }

    //Serial.printf("PeriphVin(%d) = %d", SNr, Module.GetPeriphVin(SNr));

    float TempVal  = analogRead(Module.GetPeriphIOPort(SNr, 2));
    //float TempVolt = (float) TempVal / Module.GetPeriphVin(SNr) * Module.GetVoltageDevider();
    float TempVolt = (float) TempVal / Module.GetPeriphVin(SNr) * VOLTAGE_DEVIDER;

    if (DEBUG_LEVEL > 2) {
        Serial.printf("ReadVolt: SNr=%d, port=%d: Raw: %.1f / Vin:%.2f * V-Devider:%.2f--> %.2fV\n\r", SNr, Module.GetPeriphIOPort(SNr, 2), TempVal, Module.GetPeriphVin(SNr), VOLTAGE_DEVIDER, TempVolt);
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
    float  NewVperAmp  = 0;
    float  NewVin      = 0;
    float  NewVoltage  = 0;
    float  NewNullwert = 0;
    String NewName     = "";

    jsondata = String(buff);                 
    if (DEBUG_LEVEL > 2) { Serial.printf("%lu: Recieved from: ", millis()); PrintMAC(mac); }
    
    DeserializationError error = deserializeJson(doc, jsondata);
    
    if (!error) {
        String TempName = doc["Node"];
        DEBUG2 ("(%s) - %s\n\r", TempName.c_str(), jsondata.c_str());    

        if (doc["TSConfirm"].is<JsonVariant>()) SendConfirm(mac, (uint32_t)doc["TSConfirm"]);
        
        switch ((int) doc["Order"]) 
        {
            case SEND_CMD_YOU_ARE_PAIRED:
                if (doc["Peer"] == Module.GetName())
                {
                    if (esp_now_is_peer_exist((unsigned char *) mac)) 
                    { 
                        if (DEBUG_LEVEL > 0) { PrintMAC(mac); Serial.println(" already exists..."); }
                        Module.SetPairMode(false);
                    }
                    else 
                    {
                        PeerClass *Peer = new PeerClass;
                        Peer->Setup(doc["Node"], (int) doc["Type"], "xxx", mac, false, false, false, false);
                        Peer->SetLastContact(millis());
                        WaitForContact = WAIT_AFTER_SLEEP; 
                        PeerList.add(Peer);

                        SavePeers();
                        RegisterPeers();
                        
                        if (DEBUG_LEVEL > 1) 
                        {
                            Serial.printf("New Peer added: %s (Type:%d), MAC:", Peer->GetName(), Peer->GetType());
                            PrintMAC(Peer->GetBroadcastAddress());
                            Serial.println("\n\rSaving Peers after received new one...");
                            ReportAll();
                        }
                        Module.SetPairMode(false);
                    }
                }
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
                AddStatus("VoltCalib beginnt");
                NewVoltage = (float) doc["NewVoltage"];
                //besser machen
                VoltageCalibration(2, NewVoltage) ;
                
                break;
            case SEND_CMD_SWITCH_TOGGLE:
                Pos = doc["PeriphPos"];
                if (Module.isPeriphEmpty(Pos) == false) ToggleSwitch(Pos);
                break;
            case SEND_CMD_UPDATE_NAME:
                Pos = (int) doc["PeriphPos"];
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
                Pos = (int) doc["PeriphPos"];

                if (NewVin > 0)
                {
                    Module.SetPeriphVin(Pos, NewVin);
                    SaveModule();
                }
                break;
            case SEND_CMD_UPDATE_VPERAMP:
                Pos = (int) doc["PeriphPos"];
                NewVperAmp = (float) doc["Value"];

                if (NewVperAmp > 0)
                {
                    Module.SetPeriphVperAmp(Pos, NewVperAmp);
                    SaveModule();
                    Serial.printf("Updated VperAmp at Pos:%d to %.3f\n\r", Pos, NewVperAmp);
                }
                break;
            case SEND_CMD_UPDATE_NULLWERT:
                Pos = (int) doc["PeriphPos"];
                NewNullwert = (float) doc["Value"];

                if (NewNullwert > 0)
                {
                    Module.SetPeriphNullwert(Pos, NewNullwert);
                    SaveModule();
                    Serial.printf("Updated Nullwert at Pos:%d to %.3f\n\r", Pos, NewNullwert);
                }
                break;
            case SEND_CMD_SEND_STATE:
                Pos = (int) doc["PeriphPos"];
                SendStatus(Pos);
                break;
        } // end (!error)
    }
    else // error
    { 
          DEBUG1 ("deserializeJson() failed: %s\n\r"), error.f_str();
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
    uint32_t actTime = millis();
    
    if  ((actTime - TSSend ) > MSG_INTERVAL  )                                 // Send-interval (Message or Pairing-request)
    {
        TSSend = actTime;
        if (Module.GetPairMode()) SendPairingRequest();
        else SendStatus();
    }

    if (((actTime - TSPair ) > PAIR_INTERVAL ) and (Module.GetPairMode()))     // end Pairing after pairing interval
    {
        TSPair = 0;
        Module.SetPairMode(false);
        AddStatus("Pairing beendet...");
        SetMessageLED(0);
    }

    if ((actTime - TSLed > MSGLIGHT_INTERVAL) and (TSLed > 0))                 // clear LED after LED interval
    {
        TSLed = 0;
        
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
        if ((BB1 == 1) or (BB2 == 1)) {
            TSPair = actTime;
            Module.SetPairMode(true);
            SetMessageLED(1);
    
            AddStatus("Pairing beginnt...");
            
            if (!TSButton) TSButton = actTime;
            else 
            {
                if ((actTime - TSButton) > 3000) {
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

void InitSCL()
{
    #if defined(PORT_USED) || defined(ADC_USED)
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

    #ifdef PORT_USED                            // init IOBoard
	      if (!IOBoard.begin())
          {
                DEBUG1 ("IOBoard not found!\n\r");
                while (1);
          }
          else 
          {
                DEBUG1 ("IOBoard initialised.\n\r");
          }
    #endif
    #ifdef ADC_USED                             // init ADS
        ADCBoard.setGain(GAIN_TWOTHIRDS);   // 0.1875 mV/Bit .... +- 6,144V
        if (!ADCBoard.begin(ADC_USED)) { 
            DEBUG1 ("ADS not found!\n\r");
            while (1);
        }
        else
        {
            DEBUG2 ("ADS initialised.\n\r");
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
