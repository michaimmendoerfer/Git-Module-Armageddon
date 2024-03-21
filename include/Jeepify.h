#ifndef JEEPIFY_H
#define JEEPIFY_H
#include <Arduino.h>

#define MAX_PERIPHERALS 5
#define MAX_PEERS       10
#define MAX_STATUS      10
#define SCHWELLE        0.0005

#define MSG_CHANGED_SWITCH     10 // Payload PeriphId
#define MSG_CHANGED_SENSOR     20 // Payload PeriphId
#define MSG_CHANGED_PEERNAME   30 // Payload PeerID
#define MSG_CHANGED_AVAIL      40 // Payload PeerId

// Module-Types
#define SWITCH_1_WAY        1
#define SWITCH_2_WAY        2
#define SWITCH_4_WAY        4
#define SWITCH_8_WAY        8
#define PDC                 9
#define PDC_SENSOR_MIX      10
#define BATTERY_SENSOR      20
#define MONITOR_ROUND       30
#define MONITOR_BIG         31
#define MODULE_ALL          99
#define RELAY_REVERSED      -1
#define RELAY_NORMAL        1

// Sensor-Types
#define SENS_TYPE_SWITCH  101
#define SENS_TYPE_AMP     102
#define SENS_TYPE_VOLT    103
#define SENS_TYPE_SENS    104
#define SENS_TYPE_EQUAL   105
#define SENS_TYPE_ALL     106
#define NOT_FOUND         199

// Intervals
#define MSGLIGHT_INTERVAL 300
#define PING_INTERVAL     1000
#define MSG_INTERVAL      1000
#define LOGO_INTERVAL     3000
#define OFFLINE_INTERVAL  20000
#define PAIR_INTERVAL     30000
#define SLEEP_INTERVAL    5000

// Round-Monitor Touch
const int I2C_SDA  = 4;
const int I2C_SCL  = 5;
const int TP_INT   = 0;
const int TP_RST   = 1;
const int Rotation = 0;

// ESP32-3248S035 Touch
#define TOUCH_SDA  33
#define TOUCH_SCL  32
#define TOUCH_INT  21
#define TOUCH_RST 25
#define TOUCH_WIDTH  480
#define TOUCH_HEIGHT 320

//structs
struct struct_Periph {
    char        Name[20];
    int         Id;
    int         Type;      //1=Switch, 2=Amp, 3=Volt
    bool        isADS;
    int         IOPort;
    float       NullWert;
    float       VperAmp;
    int         Vin;
    float       Value;
    float       OldValue;
    bool        Changed;
    int         PeerId;
    void        *Peer;
};
typedef struct struct_Periph struct_Periph;

struct struct_Peer {
    char       Name[20];
    int        Id;
    int        PNumber;
    u_int8_t   BroadcastAddress[6];
    uint32_t   TSLastSeen;
    int        Type;  // 
    bool       SleepMode;
    bool       DebugMode;
    bool       DemoMode;
    bool       ReadyToPair;
    struct_Periph Periph[MAX_PERIPHERALS]; 
};
typedef struct struct_Peer struct_Peer;

#define MULTI_SCREENS 4
#define MULTI_SCREEN_ROWS 2
#define MULTI_SCREEN_COLS 2
#define PERIPH_PER_SCREEN 4

struct struct_MultiScreen {
  char           Name[20];
  int            Id;
  struct_Peer   *Peer    [PERIPH_PER_SCREEN];
  int            PeerId  [PERIPH_PER_SCREEN];
  struct_Periph *Periph  [PERIPH_PER_SCREEN];
  int            PeriphId[PERIPH_PER_SCREEN];
  bool           Used;
};

typedef struct struct_MultiScreen struct_MultiScreen;

extern uint8_t broadcastAddressAll[6];
#endif