//#define KILL_NVS 1

// DEBUG_LEVEL: 0 = nothing, 1 = only Errors, 2 = relevant changes, 3 = all

#include <Arduino.h>
#include <Module.h>

const int DEBUG_LEVEL = 1; 
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
        #define ADC_ADDRESS  0x48
    #endif
    #ifdef PORT_USED
    #include "PCF8575.h"
        PCF8575 IOBoard(PORT_ADDRESS, SDA_PIN, SCL_PIN);
        #define PORT_ADDRESS 0x20
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
	#define BOARD_VOLTAGE 3.3
	#define BOARD_ANALOG_MAX 1023
#endif 

#include <LinkedList.h>
#include "Jeepify.h"
#include "PeerClass.h"
#include "pref_manager.h"
#include <Preferences.h>
#include <ArduinoJson.h>
    
#pragma endregion Includes

#pragma region Globals

const char *ArrNullwert[MAX_PERIPHERALS] = {"NW0",  "NW1",  "NW2",  "NW3",  "NW4",  "NW5",  "NW6",  "NW7",  "NW8"};
const char *ArrRaw[MAX_PERIPHERALS]      = {"Raw0", "Raw1", "Raw2", "Raw3", "Raw4", "Raw5", "Raw6", "Raw7", "Raw8"};
const char *ArrRawVolt[MAX_PERIPHERALS]  = {"RaV0", "RaV1", "RaV2", "RaV3", "RaV4", "RaV5", "RaV6", "RaV7", "RaV8"};
const char *ArrVperAmp[MAX_PERIPHERALS]  = {"VpA0", "VpA1", "VpA2", "VpA3", "VpA4", "VpA5", "VpA6", "VpA7", "VpA8"};
const char *ArrVin[MAX_PERIPHERALS]      = {"Vin0", "Vin1", "Vin2", "Vin3", "Vin4", "Vin5", "Vin6", "Vin7", "Vin8"};
const char *ArrPeriph[MAX_PERIPHERALS]   = {"Per0", "Per1", "Per2", "Per3", "Per4", "Per5", "Per6", "Per6", "Per7"};

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
void   SendStatus(int);
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

void setup()
{
    //#ifdef ARDUINO_USB_CDC_ON_BOOT
    //    delay(3000);
    //#endif
    
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
        Serial.begin(460800);
    #elif defined(ESP8266)
        Serial.begin(74880);
    #endif

    //delay(1000);
    //while (!Serial);

    #ifdef LED_PIN
        pinMode(LED_PIN, OUTPUT);
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
void SendStatus (int Pos=-1) 
{
    TSLed = millis();
    SetMessageLED(2);

    JsonDocument doc; String jsondata; 
    char buf[200]; 
    float TempValue = 0;
	
    int Status = 0;
    if (Module.GetDebugMode())   bitSet(Status, 0);
    if (Module.GetSleepMode())   bitSet(Status, 1);
    if (Module.GetDemoMode())    bitSet(Status, 2);
    if (Module.GetPairMode())    bitSet(Status, 3);    
    
    snprintf(buf, sizeof(buf), "%s;%ul;%d", Module.GetName(), millis(), Status);
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

    for (int SNr=SNrStart; SNr<SNrMax ; SNr++) 
    {   
        snprintf(buf, sizeof(buf), "%d;%s;%.3f;%.3f;%.3f;%.3f", 
            Module.GetType(), 
            Module.GetName(), 
            Module.GetPeriphValue(SNr, 0),
            Module.GetPeriphValue(SNr, 1),
            Module.GetPeriphValue(SNr, 2),
            Module.GetPeriphValue(SNr, 3));
                        
        doc[ArrPeriph[SNr]] = buf;
	}
	
	serializeJson(doc, jsondata);  

    for (int PNr=0; PNr<PeerList.size(); PNr++) 
    {
        PeerClass *Peer = PeerList.get(PNr);

        if (Peer->GetType() >= MONITOR_ROUND)
        {
            if (DEBUG_LEVEL > 2) Serial.printf("Sending to: %s ", Peer->GetName()); 
            
            if (esp_now_send(Peer->GetBroadcastAddress(), (uint8_t *) jsondata.c_str(), 250) == 0) 
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
    
    snprintf(buf, sizeof(buf), "%s;%ul;%d", Module.GetName(), millis(), Status);
    doc["Node"]    = buf;
    doc["Type"]    = Module.GetType();
    doc["Version"] = Module.GetVersion();
    doc["Order"]   = SEND_CMD_PAIR_ME;
    
    for (int SNr=0 ; SNr<MAX_PERIPHERALS; SNr++) {
        if (!Module.isPeriphEmpty(SNr)) 
        {
            for (int SNr=0; SNr<MAX_PERIPHERALS ; SNr++) 
            {   
                snprintf(buf, sizeof(buf), "%d;%s", Module.GetType(), Module.GetName());
                doc[ArrPeriph[SNr]] = buf;
	        }
        }
    }
         
    serializeJson(doc, jsondata);  

    esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  
    
    if (DEBUG_LEVEL > 2) Serial.printf("\nSending: %s\n\r", jsondata.c_str()); 
    
    //AddStatus("Send Pairing request...");                                     
}

void SendConfirm(const uint8_t * mac, uint32_t TSConfirm) 
{
    TSLed = millis();
    SetMessageLED(3);
    
    JsonDocument doc; String jsondata; 
    char buf[100];

    snprintf(buf, sizeof(buf), "%s;%ul;%d", Module.GetName(), millis(), Status);
    doc["Node"]        = buf;
    doc["TSConfirm"]   = TSConfirm;

    serializeJson(doc, jsondata); 

    if (DEBUG_LEVEL > 2) Serial.printf("%lu: Sending Confirm (%lu) to: %s ", millis(), TSConfirm, FindPeerByMAC(mac)->GetName()); 
            
    if (esp_now_send((u8 *) mac, (uint8_t *) jsondata.c_str(), 200) == 0) 
    {
            if (DEBUG_LEVEL > 2) Serial.println("ESP_OK");  //Sending "jsondata" 
    } 
    else 
    {
            if (DEBUG_LEVEL > 0) Serial.println("ESP_ERROR"); 
    }     
    
    if (DEBUG_LEVEL > 2) Serial.println(jsondata);
    
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
void ToggleSwitch(int SNr)
{
    int Value = Module.GetPeriphValue(SNr);
    
    Module.SetPeriphOldValue(SNr, Value);
    
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
          
          if (Module.GetRelayType() == RELAY_REVERSED) 
          {
              if (Value == 0) Value = 1;
              else Value = 0;
          }
          
          #ifdef PORT_USED
              IOBoard.digitalWrite(SNr, Value);
          #else
              //noch latching machen
              digitalWrite(Module.GetPeriphIOPort(SNr), Value);
          #endif

          if (DEBUG_LEVEL > 2) Serial.printf("Setze %s (Port:%d) auf %d", Module.GetPeriphName(SNr), Module.GetPeriphIOPort(SNr), Serial.print(Value));
      }
  }
  SendStatus();
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
    if (DEBUG_LEVEL > 2) Serial.printf("\nSending: %s", jsondata.c_str());
    AddStatus("Send Going to sleep......"); 
    
    if (DEBUG_LEVEL > 1) 
    {
        Serial.printf("Going to sleep at: %lu....................................................................................\n\r", millis());
        Serial.printf("LastContact    at: %lu", Module.GetLastContact()); 
    }
    
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
      if (Module.GetPeriphType(SNr) == SENS_TYPE_AMP) {
        float TempVolt = 0;
        
        #ifdef ADC_USED
            //for (int i=0; i<20; i++) 
            {
                TempVolt += ADCBoard.computeVolts(ADCBoard.readADC_SingleEnded(Module.GetPeriphIOPort(SNr)));
                delay(10);
            }
            //TempVolt /= 20;
        #else
            int TempVal = 0;
            for (int i=0; i<20; i++) 
            {
                TempVal += analogRead(Module.GetPeriphIOPort(SNr));
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
    float TempVal      = 0;
    float TempVolt     = 0;
    float TempAmp      = 0;
  
    #ifdef ADC_USED
        TempVal  = ADCBoard.readADC_SingleEnded(Module.GetPeriphIOPort(SNr));
        TempVolt = ADCBoard.computeVolts(TempVal); 
        TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr);
        delay(10);
    #else
        TempVal  = analogRead(Module.GetPeriphIOPort(SNr));
        TempVolt = BOARD_VOLTAGE/BOARD_ANALOG_MAX*TempVal;
        TempAmp  = (TempVolt - Module.GetPeriphNullwert(SNr)) / Module.GetPeriphVperAmp(SNr) * Module.GetVoltageDevider();// 1.5 wegen Voltage-Devider
        delay(10);
    #endif
  
    if (DEBUG_LEVEL > 2) {
        Serial.printf("(A): Raw:%.3f Null:%.4f --> %.4fV --> %.4fA", TempVal, Module.GetPeriphNullwert(SNr), TempVolt, TempAmp);
    } 
    if (abs(TempAmp) < SCHWELLE) TempAmp = 0;
    if (DEBUG_LEVEL > 2) {
        Serial.printf(" --> %.2fA\n\r", TempAmp);
    } 
    
    return (TempAmp); 
}
float ReadVolt(int SNr) 
{
    if (!Module.GetPeriphVin(SNr)) { if (DEBUG_LEVEL > 0) Serial.println("Vin must not be zero !!!"); return 0; }
    
    //Serial.printf("PeriphVin(%d) = %d", SNr, Module.GetPeriphVin(SNr));

    float TempVal  = analogRead(Module.GetPeriphIOPort(SNr));
    //float TempVolt = (float) TempVal / Module.GetPeriphVin(SNr) * Module.GetVoltageDevider();
    float TempVolt = (float) TempVal / Module.GetPeriphVin(SNr) * VOLTAGE_DEVIDER;

    if (DEBUG_LEVEL > 2) {
        Serial.printf("(V) Raw: %.1f / Vin:%.2f * V-Devider:%d--> %.2fV\n\r", TempVal, Module.GetPeriphVin(SNr), VOLTAGE_DEVIDER, TempVolt);
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
        if (DEBUG_LEVEL > 2) Serial.printf("(%s) - %s\n\r", TempName.c_str(), jsondata.c_str());    

        uint32_t TempTSConfirm = (uint32_t) doc["TSConfirm"];
        if (TempTSConfirm) SendConfirm(mac, TempTSConfirm);

        switch ((int) doc["Order"]) 
        {
            case SEND_CMD_YOU_ARE_PAIRED:
                if (doc["Peer"] == Module.GetName())
                {
                    if (esp_now_is_peer_exist((u8 *) mac)) 
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
                if (DEBUG_LEVEL > 2) Serial.printf("LastContact: %6lu\n\r", Module.GetLastContact());
                break;
            case SEND_CMD_SLEEPMODE_ON:
                AddStatus("Sleep: on");  
                SetSleepMode(true);  
                SendMessage(true, false, false); 
                break;
            case SEND_CMD_SLEEPMODE_OFF:
                AddStatus("Sleep: off"); 
                SetSleepMode(false); 
                SendMessage(true, false, false); 
                break;
            case SEND_CMD_SLEEPMODE_TOGGLE:
                if (Module.GetSleepMode()) 
                { 
                    AddStatus("Sleep: off");   
                    SetSleepMode(false); 
                    SendMessage(true, false, false); 
                }
                else 
                { 
                    AddStatus("Sleep: on");    
                    SetSleepMode(true);  
                    SendMessage(true, false, false); 
                }
                break;
            case SEND_CMD_DEBUGMODE_ON:
                AddStatus("DebugMode: on");  
                SetDebugMode(true);  
                SaveModule();
                SendMessage(true, false, false); 
                break;
            case SEND_CMD_DEBUGMODE_OFF:
                AddStatus("DebugMode: off"); 
                SetDebugMode(false); 
                SaveModule();
                SendMessage(true, false, false); 
                break;
            case SEND_CMD_DEBUGMODE_TOGGLE:
                if (Module.GetDebugMode()) 
                {   
                    AddStatus("DebugMode: off");   
                    SetDebugMode(false); 
                    SendMessage(true, false, false); 
                }
                else 
                { 
                    AddStatus("DebugMode: on");    
                    SetDebugMode(true);  
                    SendMessage(true, false, false); 
                }
                break;
            case SEND_CMD_DEMOMODE_ON:
                AddStatus("Demo: on");   
                SetDemoMode(true);   
                SendMessage(true, false, false); 
                break;
            case SEND_CMD_DEMOMODE_OFF:
                AddStatus("Demo: off");  
                SetDemoMode(false);  
                SendMessage(true, false, false); 
                break;
            case SEND_CMD_DEMOMODE_TOGGLE:
                if (Module.GetDemoMode()) 
                { 
                    AddStatus("DemoMode: off"); 
                    SetDemoMode(false); 
                    SendMessage(true, false, false); 
                }
                else 
                { 
                    AddStatus("DemoMode: on");  
                    SetDemoMode(true);  
                    SendMessage(true, false, false); 
                }
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
                SendMessage(true, false, false); 
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

                VoltageCalibration(VOLTAGE_DEVIDER, NewVoltage) ;
                
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
                    Serial.printf("Updated VperAmp at Pos:%d to %.3f\n\r", Pos, NewVperAmp);
                }
                break;
            case SEND_CMD_UPDATE_NULLWERT:
                Pos = (int) doc["Pos"];
                NewNullwert = (float) doc["Value"];

                if (NewNullwert > 0)
                {
                    Module.SetPeriphNullwert(Pos, NewNullwert);
                    SaveModule();
                    Serial.printf("Updated Nullwert at Pos:%d to %.3f\n\r", Pos, NewNullwert);
                }
                break;
            case SEND_CMD_SEND_STATE:
                Pos = (int) doc["Pos"];
                SendMessage(true, false, true, Pos);
                break;
        } // end (!error)
    }
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
        Serial.printf("actTime:%lu, LastContact:%lu - (actTime - Module.GetLastContact()) = %lu, WaitForContact = %lu, - Try to sleep...........................................................\n\r", actTime, Module.GetLastContact(), actTime - Module.GetLastContact(), WaitForContact);
        Module.SetLastContact(millis());
        GoToSleep();
    }

    #ifdef PAIRING_BUTTON                                                       // check for Pairing/Reset Button
        int BB = !digitalRead(PAIRING_BUTTON);
        if (BB == 1) {
            TSPair = actTime;
            Module.SetPairMode(true);
            SetMessageLED(1);
    
            AddStatus("Pairing beginnt...");
            
            if (!TSButton) TSButton = actTime;
            else 
            {
                if ((actTime - TSButton) > 3000) {
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
                if (DEBUG_LEVEL > 0) Serial.println("IOBoard not found!");
                while (1);
          }
          else 
          {
                if (DEBUG_LEVEL > 1) Serial.println("IOBoard initialised.");
          }
    #endif
    #ifdef ADC_USED                             // init ADS
        ADCBoard.setGain(GAIN_TWOTHIRDS);   // 0.1875 mV/Bit .... +- 6,144V
        if (!ADCBoard.begin(ADC_ADDRESS)) { 
          if (DEBUG_LEVEL > 0) Serial.println("ADS not found!");
          while (1);
        }
        else
        {
            if (DEBUG_LEVEL > 1) Serial.println("ADS initialised.");
        }
    #endif
}
void InitMRD()
{
    #ifdef MRD_USED                             // MultiReset-Check
        mrd = new MultiResetDetector(MRD_TIMEOUT, MRD_ADDRESS);

        if (mrd->detectMultiReset()) {
          if (DEBUG_LEVEL > 0) Serial.println("Multi Reset Detected");
          digitalWrite(LED_BUILTIN, LED_ON);
          //ClearPeers(); ClearInit(); InitModule(); SaveModule(); delay(10000); ESP.restart();
          Module.SetPairMode(true); TSPair = millis();
        }
        else {
          if (DEBUG_LEVEL > 0) Serial.println("No Multi Reset Detected");
          digitalWrite(LED_BUILTIN, LED_OFF);
        }
    #endif
}
