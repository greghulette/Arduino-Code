// =============================================================================
//  SBUSController.ino  —  Browser-based Virtual SBUS Controller
//  Target: ESP32-S3
//  Board library: esp32 by Espressif (3.x)
//  AsyncTCP: use mathieucarbou/AsyncTCP (not me-no-dev) for ESP32 core 3.x
//
//  Required libraries (Library Manager):
//    • ESPAsyncWebServer  (mathieucarbou fork or me-no-dev)
//    • AsyncTCP           (mathieucarbou/AsyncTCP)
//    • ArduinoJson        (Benoit Blanchon) v6.x
//
//  ─── Overview ───────────────────────────────────────────────────────────────
//   Browser controls → WebSocket → ESP32-S3 → SBUS stream → Kyber
//   Controls modelled on FrSky TANDEM X18:
//     4 joystick axes  (Mode 2: RX=AIL, RY=ELE, LY=THR, LX=RUD; per-axis reverse)
//     10 switches      SA-SJ  (3-pos / 2-pos / momentary; SI/SJ replace former RB1/RB2)
//     2 sliders        LS, RS
//     6 trim rockers   T1-T6  (hold-to-repeat)
//     6 buttons        S1-S6  (matrix; former RB1/RB2 are now switches SI/SJ)
//
//  ─── Hardware Wiring ────────────────────────────────────────────────────────
//   Kyber SBUS input  ←  GPIO 9  (Serial5 TX on WCB v3.x, inverted 100 kbaud 8E2)
//   RC PWM output 1   ←  GPIO 4  (Serial1 TX on WCB v3.x)
//   RC PWM output 2   ←  GPIO 6  (Serial2 TX on WCB v3.x)
//   RC PWM output 3   ←  GPIO 15 (Serial3 TX on WCB v3.x)
//   RC PWM output 4   ←  GPIO 17 (Serial4 TX on WCB v3.x)
//   USB               ↔  Serial  (debug @ 115200)
//
//  ─── Default Channel Map ────────────────────────────────────────────────────
//   CH1=RX(AIL)  CH2=RY(ELE)  CH3=LY(THR)  CH4=LX(RUD)
//   CH5=SA  CH6=SB  CH7=SC  CH8=SD  CH9=SE  CH10=SF  CH11=SG  CH12=SH
//   CH13=LS  CH14=RS
//   CH15=T1  CH16=T2  CH17=T3  CH18=T4  CH19=T5  CH20=T6
//   CH21=S1  CH22=S2  CH23=S3  CH24=S4   S5/S6/SI/SJ=unassigned by default
//
//   X20 additions (channels CONFLICT with T6/S1/S2 above — when testing
//   the FrSky Twin X20 model on a real RC-Controller you'll need to
//   reassign T6/S1/S2 or remap MS/J5/J6 in the SBUSController settings
//   to match):
//     CH20=MS (X20 middle slider; collides with T6)
//     CH21=J5 (X20 L-stick twist;   collides with S1)
//     CH22=J6 (X20 R-stick twist;   collides with S2)
//     L-Stick Click / R-Stick Click — momentary buttons that ride the
//     matrix channel (CH7 by default) with PWM tier values matching the
//     RC-Controller's bands[] for slots 19/20 (1074 / 1033).
//
//  ─── WiFi ───────────────────────────────────────────────────────────────────
//   Cascading: tries RHN-COMM → HelloEverybody → AP fallback (SBUSCtrl)
// =============================================================================

#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ─── WiFi ─────────────────────────────────────────────────────────────────────
#define MAX_WIFI_NETS       4
#define WIFI_AP_SSID        "SBUSCtrl"
#define WIFI_AP_PASS        "sbus1234"
#define WIFI_STA_TIMEOUT_MS 5000
#define MDNS_HOST           "sbusctrl"

// ─── Pins ─────────────────────────────────────────────────────────────────────
// WCB v3.x pin assignments (matches wcb_pin_map.cpp v3.2)
#define SBUS_TX_PIN     9    // Serial5 TX → SBUS output (inverted 100kbaud 8E2)
#define PWM_PIN_0       4    // Serial1 TX → RC PWM CH output 1
#define PWM_PIN_1       6    // Serial2 TX → RC PWM CH output 2
#define PWM_PIN_2       15   // Serial3 TX → RC PWM CH output 3
#define PWM_PIN_3       17   // Serial4 TX → RC PWM CH output 4
#define PWM_CH_COUNT    4

// ─── SBUS Protocol ────────────────────────────────────────────────────────────
#define SBUS_BAUD           100000
#define SBUS_HEADER         0x0F
#define SBUS_FOOTER         0x00
#define SBUS_FLAGS          0x00
#define SBUS_MIN            172
#define SBUS_MAX            1811
#define SBUS_CENTER         992
// Extended user-configurable value range (allows testing out-of-spec values)
// Note: SBUS protocol is 11-bit so the frame packer hard-caps at 2047
#define SBUS_USER_MIN       1
#define SBUS_USER_MAX       2047
#define SBUS_FRAME_MS       9     // 9 ms = ~111 Hz (FrSky standard)

#define SBUS_CH_COUNT_16    16
#define SBUS_FRAME_LEN_16   25    // 0x0F + 22 data + flags + 0x00
#define SBUS_CH_COUNT_24    24
#define SBUS_FRAME_LEN_24   36    // 0x0F + 33 data + flags + 0x00

bool g_sbus24 = true;
inline int sbusChCount()  { return g_sbus24 ? SBUS_CH_COUNT_24 : SBUS_CH_COUNT_16; }
inline int sbusFrameLen() { return g_sbus24 ? SBUS_FRAME_LEN_24 : SBUS_FRAME_LEN_16; }

// ─── SBUS Debug ───────────────────────────────────────────────────────────────
#define SBUS_DEBUG
#define SBUS_DEBUG_INTERVAL_MS  50    // serial dump rate (ms) — 50ms = 20 Hz

#ifdef SBUS_DEBUG
bool g_sbusDebug   = false;   // live channel monitor (WebSocket)
bool g_serialDebug = false;   // verbose serial dump — off by default; toggle with 'd' command
#endif

// ─── Config ───────────────────────────────────────────────────────────────────
#define CONFIG_FILE   "/config.json"
#define CFG_VER       3
#define MAX_SWITCHES  10   // SA SB SC SD SE SF SG SH SI SJ  (SI/SJ promoted from former rear buttons RB1/RB2)
#define MAX_SLIDERS   7    // LS RS S1 S2 + X20 additions (MS middle slider, J5/J6 stick twist axes)
#define MAX_TRIMS     6    // T1-T6
#define MAX_BUTTONS   8    // S1-S6  + X20 additions (L-Stick Click / R-Stick Click — matrix momentary buttons that come with the 3-axis gimbal upgrade)
#define MAX_LUA_BTNS  15   // configurable Lua / virtual buttons

enum SwType : uint8_t { SW_3POS=0, SW_2POS=1, SW_MOMENT=2 };

struct SwCfg {
  char     label[4];    // "SA".."SH"
  uint8_t  ch;          // 1-based SBUS channel; 0=unassigned
  SwType   type;
  uint16_t val[3];      // SBUS values for positions 0,1,2
  uint8_t  defaultPos;  // position on boot: 0=low, 1=center, 2=high
};

struct SliderCfg {
  char    label[4];     // "LS" or "RS"
  uint8_t ch;
};

struct TrimCfg {
  char     label[4];    // "T1".."T6"
  uint8_t  ch;
  uint8_t  step;        // step mode: SBUS units per click
  uint8_t  mode;        // 0 = step (default RC trim)  ·  1 = button (momentary)
  uint16_t valL;        // button mode: value sent on left/down  press
  uint16_t valR;        // button mode: value sent on right/up  press
};

struct BtnCfg {
  char     label[32];
  uint8_t  ch;
  uint16_t val;
};

// Configurable virtual buttons (Lua / on-screen buttons)
// Each maps a momentary press to a specific SBUS channel+value, same as original BtnCfg.
struct LuaBtnCfg {
  char     label[32];
  uint8_t  ch;     // 1-based; 0=unassigned
  char     color[8]; // "#RRGGBB\0" — button accent color
  uint16_t val;    // value when pressed; center when released
};

struct WifiNetCfg {
  char ssid[33];   // up to 32-char SSID + null
  char pass[65];   // up to 64-char password + null
};

struct PwmOutCfg {
  uint8_t ch;   // 1-based SBUS channel to mirror; 0 = disabled
};

struct Config {
  uint8_t   ver;
  uint8_t   joyRX;   // CH1 Aileron
  uint8_t   joyRY;   // CH2 Elevator
  uint8_t   joyLY;   // CH3 Throttle
  uint8_t   joyLX;   // CH4 Rudder
  SwCfg     sw[MAX_SWITCHES];
  SliderCfg slider[MAX_SLIDERS];
  TrimCfg   trim[MAX_TRIMS];
  BtnCfg    btn[MAX_BUTTONS];
  LuaBtnCfg luaBtn[MAX_LUA_BTNS];
  bool      sbus24;
  // Per-axis output range (SBUS units).  axisMin[0]=RX, [1]=RY, [2]=LY, [3]=LX
  uint16_t  axisMin[4];
  uint16_t  axisMax[4];
  // Per-axis reverse flag (flips stick direction).  Same index order as axisMin/axisMax.
  bool      axisReverse[4];
  // WiFi networks (tried in order; 0=auto cascade)
  WifiNetCfg wifiNets[MAX_WIFI_NETS];
  uint8_t    wifiCount;  // number of configured networks
  uint8_t    wifiPref;   // 0=auto, 1..wifiCount=specific net, 255=AP only
  // RC PWM outputs (4 pins mirroring selected SBUS channels)
  PwmOutCfg  pwm[PWM_CH_COUNT];
  bool       pwmExtended;  // false = standard 1000-2000µs, true = extended 500-2500µs
};

Config cfg;
int8_t g_wifiNet = -1;   // index into cfg.wifiNets of active connection; -1 = AP mode

// ─── Runtime state ────────────────────────────────────────────────────────────
uint16_t sbusChannels[SBUS_CH_COUNT_24];
uint8_t  swPos[MAX_SWITCHES];      // 0,1,2
uint8_t  sliderPct[MAX_SLIDERS];   // 0-100
int16_t  trimVal[MAX_TRIMS];       // current SBUS value for each trim
uint32_t lastFrameMs = 0;

// ─── Web server / WebSocket ───────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =============================================================================
//  CONFIG  —  defaults / init / load / save / build
// =============================================================================

void applyConfigDefaults() {
  cfg.ver   = CFG_VER;
  cfg.joyRX = 1;  cfg.joyRY = 2;  cfg.joyLY = 3;  cfg.joyLX = 4;
  cfg.sbus24 = true;

  // Switches SA-SJ.  SA-SH live on CH5-CH12; SI/SJ (formerly rear buttons RB1/RB2)
  // are unassigned by default — user picks a channel in the config UI.
  const char* swLbls[]   = {"SA","SB","SC","SD","SE","SF","SG","SH","SI","SJ"};
  SwType      swTypes[]  = {SW_3POS,SW_3POS,SW_3POS,SW_3POS,SW_3POS,SW_2POS,SW_3POS,SW_MOMENT,SW_3POS,SW_3POS};
  for (int i = 0; i < MAX_SWITCHES; i++) {
    strlcpy(cfg.sw[i].label, swLbls[i], 4);
    cfg.sw[i].ch   = (i < 8) ? (5 + i) : 0;   // SA..SH = CH5..CH12; SI/SJ unassigned
    cfg.sw[i].type = swTypes[i];
    // 3-pos: low/center/high; 2-pos & momentary: low/high/(unused)
    cfg.sw[i].val[0] = SBUS_MIN;
    cfg.sw[i].val[1] = (swTypes[i] == SW_3POS) ? SBUS_CENTER : SBUS_MAX;
    cfg.sw[i].val[2] = SBUS_MAX;
    cfg.sw[i].defaultPos = 0;
  }

  // Sliders: LS, RS (sides of sticks), S1 & S2 (centre pots).
  // X20 additions (slots 4-6): MS = middle slider that sits between
  // the B1-B6 face buttons; J5/J6 = twist axes on each gimbal stick
  // that come with the optional 3-axis upgrade.  Default channels
  // match the X20 model defaults in RC-Controller's TX_MODELS entry
  // (config_tool/index.html): MS=CH20, J5=CH21, J6=CH22.
  strlcpy(cfg.slider[0].label, "LS", 4);  cfg.slider[0].ch = 13;
  strlcpy(cfg.slider[1].label, "RS", 4);  cfg.slider[1].ch = 14;
  strlcpy(cfg.slider[2].label, "S1", 4);  cfg.slider[2].ch = 0;  // unassigned — user sets channel
  strlcpy(cfg.slider[3].label, "S2", 4);  cfg.slider[3].ch = 0;
  strlcpy(cfg.slider[4].label, "S3", 4);  cfg.slider[4].ch = 20; // X20 middle slider (was "MS")
  strlcpy(cfg.slider[5].label, "J5", 4);  cfg.slider[5].ch = 21; // X20 L-stick twist (3-axis)
  strlcpy(cfg.slider[6].label, "J6", 4);  cfg.slider[6].ch = 22; // X20 R-stick twist (3-axis)

  // Trims T1-T6
  for (int i = 0; i < MAX_TRIMS; i++) {
    char lb[4]; snprintf(lb, 4, "T%d", i + 1);
    strlcpy(cfg.trim[i].label, lb, 4);
    cfg.trim[i].ch   = 15 + i;   // CH15..CH20
    cfg.trim[i].step = 10;
    cfg.trim[i].mode = 0;        // 0 = step mode (default)
    cfg.trim[i].valL = SBUS_MIN;
    cfg.trim[i].valR = SBUS_MAX;
  }

  // Axis output range — full range by default
  for (int i = 0; i < 4; i++) { cfg.axisMin[i] = SBUS_MIN; cfg.axisMax[i] = SBUS_MAX; }
  // Axis reverse — off by default; user toggles per-axis in the joystick config UI
  for (int i = 0; i < 4; i++) cfg.axisReverse[i] = false;

  // Lua / virtual buttons (configurable, restored from original design)
  for (int i = 0; i < MAX_LUA_BTNS; i++) {
    snprintf(cfg.luaBtn[i].label, sizeof(cfg.luaBtn[i].label), "Button %d", i + 1);
    cfg.luaBtn[i].ch  = 0;
    cfg.luaBtn[i].val = SBUS_MAX;
    strlcpy(cfg.luaBtn[i].color, "#4fc3f7", sizeof(cfg.luaBtn[i].color));
  }

  // Physical momentary matrix buttons.
  //   Slots 0-5 (S1-S6): X18-style face buttons.  S1-S4 mapped to CH21-CH24
  //     by default; S5/S6 unassigned.  Former rear buttons RB1/RB2 have been
  //     promoted to switches SI/SJ.
  //   Slots 6-7 (L-Stick Click / R-Stick Click): X20 3-axis-gimbal
  //     momentaries.  Default to the matrix channel (CH7) with the X20
  //     band centres so they decode as slots 19/20 on the RC-Controller
  //     without any extra setup (see rc_config.h bands[] for the source
  //     of these PWM values).
  const char* btnLbls[] = {"S1","S2","S3","S4","S5","S6","SK","SL"};  // SK/SL = X20 L/R stick-click momentaries
  for (int i = 0; i < MAX_BUTTONS; i++) {
    strlcpy(cfg.btn[i].label, btnLbls[i], sizeof(cfg.btn[i].label));
    cfg.btn[i].ch  = (i < 4) ? 21 + i : 0;  // S1=CH21..S4=CH24
    cfg.btn[i].val = SBUS_MAX;
  }
  // X20 stick-click momentaries — channel + matrix-tier PWM defaults.
  cfg.btn[6].ch = 7;  cfg.btn[6].val = 1074;   // L-Stick Click (RC-Controller slot 19, band 1062-1086)
  cfg.btn[7].ch = 7;  cfg.btn[7].val = 1033;   // R-Stick Click (RC-Controller slot 20, band 1021-1045)

  // WiFi networks — three defaults, tried in order
  cfg.wifiCount = 3;
  cfg.wifiPref  = 0;   // 0 = auto cascade
  strlcpy(cfg.wifiNets[0].ssid, "RHN-COMM",        sizeof(cfg.wifiNets[0].ssid));
  strlcpy(cfg.wifiNets[0].pass, "0o9i8u7y)O(I*U&Y",sizeof(cfg.wifiNets[0].pass));
  strlcpy(cfg.wifiNets[1].ssid, "HelloEverybody",   sizeof(cfg.wifiNets[1].ssid));
  strlcpy(cfg.wifiNets[1].pass, "thedeskisbrown",   sizeof(cfg.wifiNets[1].pass));
  strlcpy(cfg.wifiNets[2].ssid, "KYBER_0908",       sizeof(cfg.wifiNets[2].ssid));
  strlcpy(cfg.wifiNets[2].pass, "12345678",         sizeof(cfg.wifiNets[2].pass));
  memset(&cfg.wifiNets[3], 0, sizeof(cfg.wifiNets[3]));

  // PWM outputs — default to mirroring the first 4 SBUS channels
  for (int i = 0; i < PWM_CH_COUNT; i++) cfg.pwm[i].ch = i + 1;
}

void initRuntimeState() {
  for (int i = 0; i < SBUS_CH_COUNT_24; i++) sbusChannels[i] = SBUS_CENTER;
  for (int i = 0; i < MAX_SWITCHES; i++)
    swPos[i] = min(cfg.sw[i].defaultPos, (uint8_t)(cfg.sw[i].type == SW_3POS ? 2 : 1));
  for (int i = 0; i < MAX_SLIDERS; i++) sliderPct[i] = 50;
  for (int i = 0; i < MAX_TRIMS; i++)   trimVal[i]   = SBUS_CENTER;
}

// =============================================================================
//  RC PWM outputs  —  50 Hz  (mirrors selected SBUS channels)
// =============================================================================
static const uint8_t PWM_PINS[PWM_CH_COUNT] = { PWM_PIN_0, PWM_PIN_1, PWM_PIN_2, PWM_PIN_3 };

// Map SBUS raw value to RC pulse width in microseconds.
// Standard:  172→1000 µs, 992→1500 µs, 1811→2000 µs  (clamped 1000–2000)
// Extended:  0→500 µs,    992→1500 µs, 2047→2500 µs  (piecewise, center locked)
inline uint32_t sbusToPwmUs(uint16_t sbus, bool extended) {
  if (extended) {
    int32_t us;
    if (sbus <= 992) {
      us = 500 + (int32_t)sbus * 1000 / 992;
    } else {
      us = 1500 + (int32_t)(sbus - 992) * 1000 / (2047 - 992);
    }
    return (uint32_t)constrain(us, 500, 2500);
  }
  int32_t us = 1000 + (int32_t)(sbus - 172) * 1000 / (1811 - 172);
  return (uint32_t)constrain(us, 1000, 2000);
}

// Convert microseconds to 16-bit LEDC duty at 50 Hz (20000 µs period)
inline uint32_t pwmUsToDuty(uint32_t us) {
  return (uint32_t)((uint64_t)us * 65536UL / 20000UL);
}

void initPwmOutputs() {
  for (int i = 0; i < PWM_CH_COUNT; i++) {
    // LEDC: 50 Hz RC PWM, 16-bit resolution (65536 ticks per 20 ms period)
    ledcAttach(PWM_PINS[i], 50, 16);
    // Start at center (1500 µs)
    ledcWrite(PWM_PINS[i], pwmUsToDuty(1500));
    Serial.printf("[PWM] Output %d on GPIO%d -> CH%d\n", i + 1, PWM_PINS[i], cfg.pwm[i].ch);
  }
}

void updatePwmOutputs() {
  for (int i = 0; i < PWM_CH_COUNT; i++) {
    uint8_t ch = cfg.pwm[i].ch;
    if (ch >= 1 && ch <= SBUS_CH_COUNT_24) {
      ledcWrite(PWM_PINS[i], pwmUsToDuty(sbusToPwmUs(sbusChannels[ch - 1], cfg.pwmExtended)));
    }
  }
}

// Write switch/slider/trim current state into sbusChannels.
// Called once at boot; thereafter each WS message updates channels directly.
void applyAllControls() {
  for (int i = 0; i < MAX_SWITCHES; i++) {
    auto& s = cfg.sw[i];
    if (s.ch >= 1 && s.ch <= SBUS_CH_COUNT_24) {
      uint8_t p = min(swPos[i], (uint8_t)2);
      sbusChannels[s.ch - 1] = s.val[p];
    }
  }
  for (int i = 0; i < MAX_SLIDERS; i++) {
    auto& sl = cfg.slider[i];
    if (sl.ch >= 1 && sl.ch <= SBUS_CH_COUNT_24) {
      float pct = sliderPct[i] / 100.0f;
      sbusChannels[sl.ch - 1] = (uint16_t)(pct * (SBUS_MAX - SBUS_MIN) + SBUS_MIN + 0.5f);
    }
  }
  for (int i = 0; i < MAX_TRIMS; i++) {
    auto& tr = cfg.trim[i];
    if (tr.ch < 1 || tr.ch > SBUS_CH_COUNT_24) continue;
    // Step-mode trims own their channel — write the persistent value.
    // Button-mode trims are momentary; the WS handler updates the channel
    // only on press/release, so we skip them here to avoid stomping on
    // other buttons sharing the same channel (e.g. all six trims on CH6).
    if (tr.mode == 0) {
      sbusChannels[tr.ch - 1] = (uint16_t)constrain((int)trimVal[i], SBUS_MIN, SBUS_MAX);
    }
  }
}

void loadConfig() {
  applyConfigDefaults();
  if (!LittleFS.exists(CONFIG_FILE)) {
    Serial.println("[SBUS] No config file — using defaults.");
    return;
  }
  File f = LittleFS.open(CONFIG_FILE, FILE_READ);
  if (!f) { Serial.println("[SBUS] Cannot open config."); return; }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.println("[SBUS] Config parse error — using defaults."); return; }

  int ver = doc["ver"] | 0;
  if (ver < CFG_VER) {
    Serial.printf("[SBUS] Config v%d is older than expected v%d — using defaults.\n", ver, CFG_VER);
    return;
  }

  cfg.joyRX  = constrain((int)(doc["rx"] | 1), 1, SBUS_CH_COUNT_24);
  cfg.joyRY  = constrain((int)(doc["ry"] | 2), 1, SBUS_CH_COUNT_24);
  cfg.joyLY  = constrain((int)(doc["ly"] | 3), 1, SBUS_CH_COUNT_24);
  cfg.joyLX  = constrain((int)(doc["lx"] | 4), 1, SBUS_CH_COUNT_24);
  cfg.sbus24 = doc["sbus24"] | true;
  g_sbus24   = cfg.sbus24;

  JsonArray swArr = doc["sw"].as<JsonArray>();
  int idx = 0;
  for (JsonObject o : swArr) {
    if (idx >= MAX_SWITCHES) break;
    strlcpy(cfg.sw[idx].label, o["l"] | cfg.sw[idx].label, 4);
    cfg.sw[idx].ch         = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    cfg.sw[idx].type       = (SwType)constrain((int)(o["t"] | 0), 0, 2);
    cfg.sw[idx].defaultPos = constrain((int)(o["d"] | 0), 0, 2);
    if (o["v"].is<JsonArray>()) {
      JsonArray va = o["v"].as<JsonArray>();
      for (int j = 0; j < 3; j++)
        cfg.sw[idx].val[j] = constrain((int)(va[j] | SBUS_CENTER), SBUS_USER_MIN, SBUS_USER_MAX);
    }
    idx++;
  }

  JsonArray slArr = doc["sl"].as<JsonArray>();
  idx = 0;
  for (JsonObject o : slArr) {
    if (idx >= MAX_SLIDERS) break;
    strlcpy(cfg.slider[idx].label, o["l"] | cfg.slider[idx].label, 4);
    cfg.slider[idx].ch = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    idx++;
  }

  JsonArray trArr = doc["tr"].as<JsonArray>();
  idx = 0;
  for (JsonObject o : trArr) {
    if (idx >= MAX_TRIMS) break;
    strlcpy(cfg.trim[idx].label, o["l"] | cfg.trim[idx].label, 4);
    cfg.trim[idx].ch   = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    cfg.trim[idx].step = constrain((int)(o["s"] | 10), 1, 100);
    cfg.trim[idx].mode = constrain((int)(o["m"] | 0), 0, 1);
    cfg.trim[idx].valL = constrain((int)(o["vL"] | (int)cfg.trim[idx].valL), 0, 2047);
    cfg.trim[idx].valR = constrain((int)(o["vR"] | (int)cfg.trim[idx].valR), 0, 2047);
    idx++;
  }

  JsonArray btnArr = doc["btn"].as<JsonArray>();
  idx = 0;
  for (JsonObject o : btnArr) {
    if (idx >= MAX_BUTTONS) break;
    strlcpy(cfg.btn[idx].label, o["l"] | "", sizeof(cfg.btn[idx].label));
    cfg.btn[idx].ch  = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    cfg.btn[idx].val = constrain((int)(o["v"] | SBUS_MAX), SBUS_USER_MIN, SBUS_USER_MAX);
    idx++;
  }
  // Lua buttons
  JsonArray luaArr = doc["lua"].as<JsonArray>();
  idx = 0;
  for (JsonObject o : luaArr) {
    if (idx >= MAX_LUA_BTNS) break;
    strlcpy(cfg.luaBtn[idx].label, o["l"] | cfg.luaBtn[idx].label, sizeof(cfg.luaBtn[idx].label));
    cfg.luaBtn[idx].ch  = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    cfg.luaBtn[idx].val = constrain((int)(o["v"] | SBUS_MAX), SBUS_USER_MIN, SBUS_USER_MAX);
    { const char* kv = o["k"] | ""; strlcpy(cfg.luaBtn[idx].color, kv[0] ? kv : cfg.luaBtn[idx].color, sizeof(cfg.luaBtn[idx].color)); }
    idx++;
  }
  // Axis range
  if (doc["aMin"].is<JsonArray>()) {
    JsonArray mn = doc["aMin"].as<JsonArray>();
    for (int i = 0; i < 4; i++) cfg.axisMin[i] = constrain((int)(mn[i] | SBUS_MIN), SBUS_USER_MIN, SBUS_USER_MAX);
  }
  if (doc["aMax"].is<JsonArray>()) {
    JsonArray mx = doc["aMax"].as<JsonArray>();
    for (int i = 0; i < 4; i++) cfg.axisMax[i] = constrain((int)(mx[i] | SBUS_MAX), SBUS_USER_MIN, SBUS_USER_MAX);
  }
  // Axis reverse (per-axis bool flag) — absent in old configs, treat as false
  if (doc["aRev"].is<JsonArray>()) {
    JsonArray rv = doc["aRev"].as<JsonArray>();
    for (int i = 0; i < 4; i++) cfg.axisReverse[i] = rv[i] | false;
  }

  // WiFi networks
  cfg.wifiPref = doc["wifiPref"] | cfg.wifiPref;
  if (doc["wifiNets"].is<JsonArray>()) {
    JsonArray wa = doc["wifiNets"].as<JsonArray>();
    cfg.wifiCount = 0;
    for (JsonObject o : wa) {
      if (cfg.wifiCount >= MAX_WIFI_NETS) break;
      strlcpy(cfg.wifiNets[cfg.wifiCount].ssid, o["s"] | "", sizeof(cfg.wifiNets[0].ssid));
      strlcpy(cfg.wifiNets[cfg.wifiCount].pass, o["p"] | "", sizeof(cfg.wifiNets[0].pass));
      if (cfg.wifiNets[cfg.wifiCount].ssid[0]) cfg.wifiCount++;
    }
  } else {
    // File predates WiFi config support — resave immediately so export includes networks
    Serial.println("[SBUS] WiFi section missing from config — upgrading file.");
    saveConfig();
  }

  // PWM outputs
  if (doc["pwm"].is<JsonArray>()) {
    JsonArray pa = doc["pwm"].as<JsonArray>();
    int idx = 0;
    for (JsonObject o : pa) {
      if (idx >= PWM_CH_COUNT) break;
      cfg.pwm[idx].ch = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
      idx++;
    }
  }
  cfg.pwmExtended = doc["pwmExt"] | false;

  Serial.println("[SBUS] Config loaded.");
}

void saveConfig() {
  File f = LittleFS.open(CONFIG_FILE, FILE_WRITE);
  if (!f) { Serial.println("[SBUS] Cannot write config."); return; }

  JsonDocument doc;
  doc["ver"]    = CFG_VER;
  doc["rx"]     = cfg.joyRX;
  doc["ry"]     = cfg.joyRY;
  doc["ly"]     = cfg.joyLY;
  doc["lx"]     = cfg.joyLX;
  doc["sbus24"] = cfg.sbus24;

  JsonArray swArr = doc.createNestedArray("sw");
  for (int i = 0; i < MAX_SWITCHES; i++) {
    JsonObject o = swArr.createNestedObject();
    o["l"] = cfg.sw[i].label;
    o["c"] = cfg.sw[i].ch;
    o["t"] = (int)cfg.sw[i].type;
    o["d"] = cfg.sw[i].defaultPos;
    JsonArray va = o.createNestedArray("v");
    va.add(cfg.sw[i].val[0]); va.add(cfg.sw[i].val[1]); va.add(cfg.sw[i].val[2]);
  }

  JsonArray slArr = doc.createNestedArray("sl");
  for (int i = 0; i < MAX_SLIDERS; i++) {
    JsonObject o = slArr.createNestedObject();
    o["l"] = cfg.slider[i].label;
    o["c"] = cfg.slider[i].ch;
  }

  JsonArray trArr = doc.createNestedArray("tr");
  for (int i = 0; i < MAX_TRIMS; i++) {
    JsonObject o = trArr.createNestedObject();
    o["l"]  = cfg.trim[i].label;
    o["c"]  = cfg.trim[i].ch;
    o["s"]  = cfg.trim[i].step;
    o["m"]  = cfg.trim[i].mode;
    o["vL"] = cfg.trim[i].valL;
    o["vR"] = cfg.trim[i].valR;
  }

  JsonArray btnArr = doc.createNestedArray("btn");
  for (int i = 0; i < MAX_BUTTONS; i++) {
    JsonObject o = btnArr.createNestedObject();
    o["l"] = cfg.btn[i].label;
    o["c"] = cfg.btn[i].ch;
    o["v"] = cfg.btn[i].val;
  }

  JsonArray luaArr = doc.createNestedArray("lua");
  for (int i = 0; i < MAX_LUA_BTNS; i++) {
    JsonObject o = luaArr.createNestedObject();
    o["l"] = cfg.luaBtn[i].label;
    o["c"] = cfg.luaBtn[i].ch;
    o["v"] = cfg.luaBtn[i].val;
    o["k"] = cfg.luaBtn[i].color;
  }

  JsonArray mnArr = doc.createNestedArray("aMin");
  JsonArray mxArr = doc.createNestedArray("aMax");
  for (int i = 0; i < 4; i++) { mnArr.add(cfg.axisMin[i]); mxArr.add(cfg.axisMax[i]); }
  JsonArray rvArr = doc.createNestedArray("aRev");
  for (int i = 0; i < 4; i++) rvArr.add(cfg.axisReverse[i]);

  doc["wifiPref"] = cfg.wifiPref;
  JsonArray wArr = doc.createNestedArray("wifiNets");
  for (int i = 0; i < cfg.wifiCount; i++) {
    JsonObject o = wArr.createNestedObject();
    o["s"] = cfg.wifiNets[i].ssid;
    o["p"] = cfg.wifiNets[i].pass;
  }

  JsonArray pwmArr = doc.createNestedArray("pwm");
  for (int i = 0; i < PWM_CH_COUNT; i++) {
    JsonObject o = pwmArr.createNestedObject();
    o["c"] = cfg.pwm[i].ch;
  }
  doc["pwmExt"] = cfg.pwmExtended;

  serializeJson(doc, f);
  f.close();
  Serial.println("[SBUS] Config saved.");
}

String buildCfgJson() {
  JsonDocument doc;
  doc["e"]      = "cfg";
  doc["ver"]    = CFG_VER;
  doc["rx"]     = cfg.joyRX;
  doc["ry"]     = cfg.joyRY;
  doc["ly"]     = cfg.joyLY;
  doc["lx"]     = cfg.joyLX;
  doc["sbus24"] = cfg.sbus24;
#ifdef SBUS_DEBUG
  doc["dbg"]    = g_sbusDebug;
#else
  doc["dbg"]    = false;
#endif

  JsonArray swArr = doc.createNestedArray("sw");
  for (int i = 0; i < MAX_SWITCHES; i++) {
    JsonObject o = swArr.createNestedObject();
    o["l"]   = cfg.sw[i].label;
    o["c"]   = cfg.sw[i].ch;
    o["t"]   = (int)cfg.sw[i].type;
    o["d"]   = cfg.sw[i].defaultPos;
    o["pos"] = swPos[i];
    JsonArray va = o.createNestedArray("v");
    va.add(cfg.sw[i].val[0]); va.add(cfg.sw[i].val[1]); va.add(cfg.sw[i].val[2]);
  }

  JsonArray slArr = doc.createNestedArray("sl");
  for (int i = 0; i < MAX_SLIDERS; i++) {
    JsonObject o = slArr.createNestedObject();
    o["l"]   = cfg.slider[i].label;
    o["c"]   = cfg.slider[i].ch;
    o["pct"] = sliderPct[i];
  }

  JsonArray trArr = doc.createNestedArray("tr");
  for (int i = 0; i < MAX_TRIMS; i++) {
    JsonObject o = trArr.createNestedObject();
    o["l"]   = cfg.trim[i].label;
    o["c"]   = cfg.trim[i].ch;
    o["s"]   = cfg.trim[i].step;
    o["m"]   = cfg.trim[i].mode;
    o["vL"]  = cfg.trim[i].valL;
    o["vR"]  = cfg.trim[i].valR;
    o["cur"] = trimVal[i];
  }

  JsonArray btnArr = doc.createNestedArray("btn");
  for (int i = 0; i < MAX_BUTTONS; i++) {
    JsonObject o = btnArr.createNestedObject();
    o["l"] = cfg.btn[i].label;
    o["c"] = cfg.btn[i].ch;
    o["v"] = cfg.btn[i].val;
  }

  JsonArray luaArr2 = doc.createNestedArray("lua");
  for (int i = 0; i < MAX_LUA_BTNS; i++) {
    JsonObject o = luaArr2.createNestedObject();
    o["l"] = cfg.luaBtn[i].label;
    o["c"] = cfg.luaBtn[i].ch;
    o["v"] = cfg.luaBtn[i].val;
    o["k"] = cfg.luaBtn[i].color;
  }

  JsonArray mnArr2 = doc.createNestedArray("aMin");
  JsonArray mxArr2 = doc.createNestedArray("aMax");
  for (int i = 0; i < 4; i++) { mnArr2.add(cfg.axisMin[i]); mxArr2.add(cfg.axisMax[i]); }
  JsonArray rvArr2 = doc.createNestedArray("aRev");
  for (int i = 0; i < 4; i++) rvArr2.add(cfg.axisReverse[i]);

  // WiFi status + configured networks
  doc["wifiPref"] = cfg.wifiPref;
  doc["wifiNet"]  = g_wifiNet;
  doc["wifiIP"]   = (g_wifiNet >= 0) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  doc["wifiMDNS"] = MDNS_HOST ".local";
  JsonArray wArr2 = doc.createNestedArray("wifiNets");
  for (int i = 0; i < cfg.wifiCount; i++) {
    JsonObject o = wArr2.createNestedObject();
    o["s"] = cfg.wifiNets[i].ssid;
    o["p"] = cfg.wifiNets[i].pass;
  }

  JsonArray pwmArr2 = doc.createNestedArray("pwm");
  for (int i = 0; i < PWM_CH_COUNT; i++) {
    JsonObject o = pwmArr2.createNestedObject();
    o["c"] = cfg.pwm[i].ch;
  }
  doc["pwmExt"] = cfg.pwmExtended;

  String out;
  serializeJson(doc, out);
  return out;
}

// =============================================================================
//  SBUS  —  encode and transmit
// =============================================================================

void buildSbusFrame(uint8_t* frame) {
  const int flen  = sbusFrameLen();
  const int chcnt = sbusChCount();
  memset(frame, 0, flen);
  frame[0]        = SBUS_HEADER;
  frame[flen - 2] = SBUS_FLAGS;
  frame[flen - 1] = SBUS_FOOTER;
  for (int i = 0; i < chcnt; i++) {
    uint16_t val = (uint16_t)constrain((int)sbusChannels[i], 0, 2047);  // 11-bit protocol max
    int b = i * 11;
    frame[1 + b / 8]     |= (uint8_t)((val << (b % 8)) & 0xFF);
    frame[1 + b / 8 + 1] |= (uint8_t)((val >> (8 - b % 8)) & 0xFF);
    if ((b % 8) > 5)
      frame[1 + b / 8 + 2] |= (uint8_t)((val >> (16 - b % 8)) & 0xFF);
  }
}

// =============================================================================
//  SBUS DEBUG
// =============================================================================

#ifdef SBUS_DEBUG
static uint32_t s_lastSerialDbgMs = 0;
static uint32_t s_lastWsDbgMs     = 0;
static uint16_t s_prevCh[SBUS_CH_COUNT_24];
static bool     s_prevChInit = false;

static void decodeSbusFrame(const uint8_t* frame, uint16_t* out, int chcnt) {
  for (int i = 0; i < chcnt; i++) {
    int b = i * 11;
    uint16_t raw = 0;
    raw  = (uint16_t)(frame[1 + b/8])     >> (b % 8);
    raw |= (uint16_t)(frame[1 + b/8 + 1]) << (8 - b % 8);
    if ((b % 8) > 5)
      raw |= (uint16_t)(frame[1 + b/8 + 2]) << (16 - b % 8);
    out[i] = raw & 0x07FF;
  }
}

// ── Non-blocking logging ────────────────────────────────────────────────────
// Never stall on Serial. On WCB HW 3.2 the USB serial path back-pressures when
// no host is draining it, so a plain Serial.printf() on a recurring path (WS
// connect/disconnect, HTTP hits, WiFi events) freezes the whole controller once
// the TX buffer fills. vlogf() writes ONLY if the TX buffer has room right now,
// otherwise the line is silently dropped — it can never block. Use it for any
// recurring/background log; reserve plain Serial.* for one-shot boot output and
// replies to host commands (a host is by definition present and draining then).
static void vlogf(const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(buf)) n = sizeof(buf);
  if (Serial.availableForWrite() >= n) Serial.write((const uint8_t*)buf, (size_t)n);
}

static void printSbusDebug(const uint8_t* frame, int flen, int chcnt) {
  if (!g_serialDebug) return;
  const uint32_t now = millis();
  if (now - s_lastSerialDbgMs < (uint32_t)SBUS_DEBUG_INTERVAL_MS) return;
  s_lastSerialDbgMs = now;

  uint16_t cur[SBUS_CH_COUNT_24];
  decodeSbusFrame(frame, cur, chcnt);

  Serial.printf("SBUS-%d [%dB]  hdr=%02X |", chcnt, flen, frame[0]);
  for (int i = 1; i < flen - 2; i++) Serial.printf(" %02X", frame[i]);
  Serial.printf(" | fl=%02X end=%02X\n", frame[flen - 2], frame[flen - 1]);

  if (!s_prevChInit) {
    memset(s_prevCh, 0xFF, sizeof(s_prevCh));
    s_prevChInit = true;
  }
  bool anyChange = false;
  for (int i = 0; i < chcnt; i++) if (cur[i] != s_prevCh[i]) { anyChange = true; break; }
  if (anyChange) {
    Serial.print(F("  \xce\x94"));
    for (int i = 0; i < chcnt; i++)
      if (cur[i] != s_prevCh[i]) Serial.printf("  CH%02d:%4u", i + 1, cur[i]);
    Serial.println();
    memcpy(s_prevCh, cur, sizeof(uint16_t) * chcnt);
  }
}

static void sendWsDebug(const uint8_t* frame, int flen, int chcnt) {
  if (!g_sbusDebug) return;
  const uint32_t now = millis();
  if (now - s_lastWsDbgMs < 100) return;
  s_lastWsDbgMs = now;

  uint16_t cur[SBUS_CH_COUNT_24];
  decodeSbusFrame(frame, cur, chcnt);

  String msg;
  msg.reserve(64 + chcnt * 5);
  msg  = "{\"e\":\"chdata\",\"mode\":";
  msg += chcnt;
  msg += ",\"fl\":";
  msg += flen;
  msg += ",\"ch\":[";
  for (int i = 0; i < chcnt; i++) { msg += cur[i]; if (i < chcnt - 1) msg += ','; }
  msg += "]}";
  ws.textAll(msg);
}
#endif  // SBUS_DEBUG

void transmitSbus() {
  uint8_t frame[SBUS_FRAME_LEN_24];
  buildSbusFrame(frame);
#ifdef SBUS_DEBUG
  const int flen  = sbusFrameLen();
  const int chcnt = sbusChCount();
  printSbusDebug(frame, flen, chcnt);
  sendWsDebug(frame, flen, chcnt);
#endif
  Serial1.write(frame, sbusFrameLen());
  updatePwmOutputs();
}

// =============================================================================
//  AXIS MAPPING
// =============================================================================

inline uint16_t axisToSbus(float v) {
  v = constrain(v, -1.0f, 1.0f);
  return (uint16_t)((v * 0.5f + 0.5f) * (float)(SBUS_MAX - SBUS_MIN) + SBUS_MIN + 0.5f);
}

// Range-aware version: maps -1..+1 → mn..mx (supports reversal when mn > mx)
inline uint16_t axisToSbusRange(float v, uint16_t mn, uint16_t mx) {
  v = constrain(v, -1.0f, 1.0f);
  float mapped = (v * 0.5f + 0.5f) * ((float)(int)mx - (float)(int)mn) + (float)(int)mn;
  return (uint16_t)constrain((int)(mapped + 0.5f), 0, 2047);  // 11-bit protocol max; frame packer also caps
}

// =============================================================================
//  WEBSOCKET HANDLER
// =============================================================================

// Forward declarations (Arduino's auto-prototyper sometimes misses these
// when the file contains a large raw string literal).
void switchWifi(uint8_t pref);

void handleWsMessage(AsyncWebSocketClient* client, const char* json) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) return;
  const char* t = doc["t"] | "";

  // ── Joystick axes ────────────────────────────────────────────────────────
  // { "t":"a", "lx":f, "ly":f, "rx":f, "ry":f }
  // RY/LY are negated so "stick up" → positive value; per-axis cfg.axisReverse
  // flips the final sign so the user can correct a downstream device whose
  // direction is opposite of what the X18 model expects.
  if (!strcmp(t, "a")) {
    float rx = (doc["rx"] | 0.0f);
    float ry = -(doc["ry"] | 0.0f);
    float ly = -(doc["ly"] | 0.0f);
    float lx = (doc["lx"] | 0.0f);
    if (cfg.axisReverse[0]) rx = -rx;
    if (cfg.axisReverse[1]) ry = -ry;
    if (cfg.axisReverse[2]) ly = -ly;
    if (cfg.axisReverse[3]) lx = -lx;
    sbusChannels[cfg.joyRX - 1] = axisToSbusRange(rx, cfg.axisMin[0], cfg.axisMax[0]);
    sbusChannels[cfg.joyRY - 1] = axisToSbusRange(ry, cfg.axisMin[1], cfg.axisMax[1]);
    sbusChannels[cfg.joyLY - 1] = axisToSbusRange(ly, cfg.axisMin[2], cfg.axisMax[2]);
    sbusChannels[cfg.joyLX - 1] = axisToSbusRange(lx, cfg.axisMin[3], cfg.axisMax[3]);
  }

  // ── Switch position ───────────────────────────────────────────────────────
  // { "t":"sw", "i":idx, "p":pos }  pos = 0/1/2
  else if (!strcmp(t, "sw")) {
    int idx = doc["i"] | -1;
    int pos = doc["p"] | 0;
    if (idx >= 0 && idx < MAX_SWITCHES) {
      auto& s = cfg.sw[idx];
      uint8_t maxPos = (s.type == SW_3POS) ? 2 : 1;
      swPos[idx] = (uint8_t)constrain(pos, 0, (int)maxPos);
      if (s.ch >= 1 && s.ch <= SBUS_CH_COUNT_24)
        sbusChannels[s.ch - 1] = s.val[swPos[idx]];
    }
  }

  // ── Slider value ──────────────────────────────────────────────────────────
  // { "t":"sl", "i":idx, "v":pct }  pct = 0-100
  else if (!strcmp(t, "sl")) {
    int idx = doc["i"] | -1;
    int pct = doc["v"] | 50;
    if (idx >= 0 && idx < MAX_SLIDERS) {
      sliderPct[idx] = (uint8_t)constrain(pct, 0, 100);
      auto& sl = cfg.slider[idx];
      if (sl.ch >= 1 && sl.ch <= SBUS_CH_COUNT_24) {
        float p = sliderPct[idx] / 100.0f;
        sbusChannels[sl.ch - 1] = (uint16_t)(p * (SBUS_MAX - SBUS_MIN) + SBUS_MIN + 0.5f);
      }
    }
  }

  // ── Trim delta / button press ────────────────────────────────────────────
  // Step mode (tr.mode == 0):
  //   { "t":"tr", "i":idx, "d":delta }   d: +1/-1 step, 0 = reset to center
  // Button mode (tr.mode == 1):
  //   { "t":"tr", "i":idx, "d":delta, "p":bool }
  //     p=true  → channel = valR (d>0) or valL (d<0)
  //     p=false → channel = SBUS_CENTER (release)
  else if (!strcmp(t, "tr")) {
    int idx   = doc["i"] | -1;
    int delta = doc["d"] | 0;
    bool hasP = doc.containsKey("p");
    bool pressed = doc["p"] | false;
    if (idx >= 0 && idx < MAX_TRIMS) {
      auto& tr = cfg.trim[idx];
      if (tr.mode == 1) {
        // ── Button mode: send specific value while pressed, center on release
        if (hasP && !pressed) {
          trimVal[idx] = SBUS_CENTER;
        } else {
          trimVal[idx] = (delta > 0) ? tr.valR : tr.valL;
        }
      } else {
        // ── Step mode (original behavior)
        if (delta == 0) {
          trimVal[idx] = SBUS_CENTER;
        } else {
          int newVal = trimVal[idx] + (delta > 0 ? (int)tr.step : -(int)tr.step);
          trimVal[idx] = (int16_t)constrain(newVal, SBUS_MIN, SBUS_MAX);
        }
      }
      if (tr.ch >= 1 && tr.ch <= SBUS_CH_COUNT_24)
        sbusChannels[tr.ch - 1] = trimVal[idx];
    }
  }

  // ── Physical button press/release ────────────────────────────────────────
  // { "t":"btn", "i":idx, "p":bool }
  else if (!strcmp(t, "btn")) {
    int  idx     = doc["i"] | -1;
    bool pressed = doc["p"] | false;
    if (idx >= 0 && idx < MAX_BUTTONS) {
      auto& b = cfg.btn[idx];
      if (b.ch >= 1 && b.ch <= SBUS_CH_COUNT_24)
        sbusChannels[b.ch - 1] = pressed ? b.val : SBUS_CENTER;
    }
  }

  // ── Lua / virtual button press/release ───────────────────────────────────
  // { "t":"lua", "i":idx, "p":bool }
  else if (!strcmp(t, "lua")) {
    int  idx     = doc["i"] | -1;
    bool pressed = doc["p"] | false;
    if (idx >= 0 && idx < MAX_LUA_BTNS) {
      auto& b = cfg.luaBtn[idx];
      if (b.ch >= 1 && b.ch <= SBUS_CH_COUNT_24)
        sbusChannels[b.ch - 1] = pressed ? b.val : SBUS_CENTER;
    }
  }

  // ── SBUS mode toggle ──────────────────────────────────────────────────────
  // { "t":"mode", "sbus24":bool }
  else if (!strcmp(t, "mode")) {
    bool newMode = doc["sbus24"] | g_sbus24;
    if (newMode != g_sbus24) {
      g_sbus24   = newMode;
      cfg.sbus24 = newMode;
      saveConfig();
      for (int i = 0; i < SBUS_CH_COUNT_24; i++) sbusChannels[i] = SBUS_CENTER;
      applyAllControls();
      Serial.printf("[SBUS] Mode → SBUS-%d (%d bytes/frame)\n", sbusChCount(), sbusFrameLen());
    }
    ws.textAll(buildCfgJson());
  }

  // ── Debug toggle ──────────────────────────────────────────────────────────
  // { "t":"dbg", "on":bool }
  else if (!strcmp(t, "dbg")) {
#ifdef SBUS_DEBUG
    g_sbusDebug = doc["on"] | !g_sbusDebug;
    Serial.printf("[SBUS] Debug %s\n", g_sbusDebug ? "ENABLED" : "DISABLED");
    ws.textAll(buildCfgJson());
#endif
  }

  // ── Config save (axis channels + all control channels) ────────────────────
  // { "t":"cfg", "rx":n, "ry":n, "ly":n, "lx":n, "sw":[...], ... }
  else if (!strcmp(t, "cfg")) {
    cfg.joyRX = constrain((int)(doc["rx"] | cfg.joyRX), 1, SBUS_CH_COUNT_24);
    cfg.joyRY = constrain((int)(doc["ry"] | cfg.joyRY), 1, SBUS_CH_COUNT_24);
    cfg.joyLY = constrain((int)(doc["ly"] | cfg.joyLY), 1, SBUS_CH_COUNT_24);
    cfg.joyLX = constrain((int)(doc["lx"] | cfg.joyLX), 1, SBUS_CH_COUNT_24);

    // Optional switch updates — type is editable per-switch from the UI
    // (SwType: 0=3POS, 1=2POS, 2=MOMENT).  When the type changes we also clamp
    // the saved defaultPos to a position that is valid for the new type so a
    // momentary/2-pos switch can't carry a stale "1" (center) selection that
    // would never be reachable.
    if (doc["sw"].is<JsonArray>()) {
      JsonArray arr = doc["sw"].as<JsonArray>();
      int i = 0;
      for (JsonObject o : arr) {
        if (i >= MAX_SWITCHES) break;
        strlcpy(cfg.sw[i].label, o["l"] | cfg.sw[i].label, 4);
        cfg.sw[i].ch   = constrain((int)(o["c"] | cfg.sw[i].ch), 0, SBUS_CH_COUNT_24);
        cfg.sw[i].type = (SwType)constrain((int)(o["t"] | (int)cfg.sw[i].type), 0, 2);
        int dRaw = constrain((int)(o["d"] | cfg.sw[i].defaultPos), 0, 2);
        // 2-pos and momentary only have positions 0 and 2; snap a stray 1 to 0.
        if (cfg.sw[i].type != SW_3POS && dRaw == 1) dRaw = 0;
        cfg.sw[i].defaultPos = (uint8_t)dRaw;
        if (o["v"].is<JsonArray>()) {
          JsonArray va = o["v"].as<JsonArray>();
          for (int j = 0; j < 3; j++)
            cfg.sw[i].val[j] = constrain((int)(va[j] | cfg.sw[i].val[j]), SBUS_USER_MIN, SBUS_USER_MAX);
        }
        i++;
      }
    }
    // Optional slider channel updates
    if (doc["sl"].is<JsonArray>()) {
      JsonArray arr = doc["sl"].as<JsonArray>();
      int i = 0;
      for (JsonObject o : arr) {
        if (i >= MAX_SLIDERS) break;
        cfg.slider[i].ch = constrain((int)(o["c"] | cfg.slider[i].ch), 0, SBUS_CH_COUNT_24);
        i++;
      }
    }
    // Optional trim updates
    if (doc["tr"].is<JsonArray>()) {
      JsonArray arr = doc["tr"].as<JsonArray>();
      int i = 0;
      for (JsonObject o : arr) {
        if (i >= MAX_TRIMS) break;
        cfg.trim[i].ch   = constrain((int)(o["c"]  | cfg.trim[i].ch),   0, SBUS_CH_COUNT_24);
        cfg.trim[i].step = constrain((int)(o["s"]  | cfg.trim[i].step),  1, 100);
        cfg.trim[i].mode = constrain((int)(o["m"]  | cfg.trim[i].mode),  0, 1);
        cfg.trim[i].valL = constrain((int)(o["vL"] | (int)cfg.trim[i].valL), 0, 2047);
        cfg.trim[i].valR = constrain((int)(o["vR"] | (int)cfg.trim[i].valR), 0, 2047);
        i++;
      }
    }
    // Optional button updates
    if (doc["btn"].is<JsonArray>()) {
      JsonArray arr = doc["btn"].as<JsonArray>();
      int i = 0;
      for (JsonObject o : arr) {
        if (i >= MAX_BUTTONS) break;
        strlcpy(cfg.btn[i].label, o["l"] | cfg.btn[i].label, sizeof(cfg.btn[i].label));
        cfg.btn[i].ch  = constrain((int)(o["c"] | cfg.btn[i].ch),  0, SBUS_CH_COUNT_24);
        cfg.btn[i].val = constrain((int)(o["v"] | cfg.btn[i].val), SBUS_USER_MIN, SBUS_USER_MAX);
        i++;
      }
    }
    // Optional Lua button updates
    if (doc["lua"].is<JsonArray>()) {
      JsonArray arr = doc["lua"].as<JsonArray>();
      int i = 0;
      for (JsonObject o : arr) {
        if (i >= MAX_LUA_BTNS) break;
        strlcpy(cfg.luaBtn[i].label, o["l"] | cfg.luaBtn[i].label, sizeof(cfg.luaBtn[i].label));
        cfg.luaBtn[i].ch  = constrain((int)(o["c"] | cfg.luaBtn[i].ch),  0, SBUS_CH_COUNT_24);
        cfg.luaBtn[i].val = constrain((int)(o["v"] | cfg.luaBtn[i].val), SBUS_USER_MIN, SBUS_USER_MAX);
        { const char* kv = o["k"] | ""; strlcpy(cfg.luaBtn[i].color, kv[0] ? kv : cfg.luaBtn[i].color, sizeof(cfg.luaBtn[i].color)); }
        i++;
      }
    }
    // Axis range
    if (doc["aMin"].is<JsonArray>()) {
      JsonArray mn = doc["aMin"].as<JsonArray>();
      for (int i = 0; i < 4; i++) cfg.axisMin[i] = constrain((int)(mn[i] | SBUS_MIN), SBUS_USER_MIN, SBUS_USER_MAX);
    }
    if (doc["aMax"].is<JsonArray>()) {
      JsonArray mx = doc["aMax"].as<JsonArray>();
      for (int i = 0; i < 4; i++) cfg.axisMax[i] = constrain((int)(mx[i] | SBUS_MAX), SBUS_USER_MIN, SBUS_USER_MAX);
    }
    // Axis reverse flags
    if (doc["aRev"].is<JsonArray>()) {
      JsonArray rv = doc["aRev"].as<JsonArray>();
      for (int i = 0; i < 4; i++) cfg.axisReverse[i] = rv[i] | false;
    }
    // PWM channel assignments
    if (doc["pwm"].is<JsonArray>()) {
      JsonArray pa = doc["pwm"].as<JsonArray>();
      int i = 0;
      for (JsonObject o : pa) {
        if (i >= PWM_CH_COUNT) break;
        cfg.pwm[i].ch = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
        i++;
      }
    }
    if (doc["pwmExt"].is<bool>()) cfg.pwmExtended = doc["pwmExt"].as<bool>();
    saveConfig();
    applyAllControls();
    ws.textAll(buildCfgJson());
    Serial.println("[SBUS] Config updated via WebSocket.");

  // ── WiFi config + switch ───────────────────────────────────────────────────
  // { "t":"wificfg", "pref":N, "nets":[{"s":"SSID","p":"pass"},...] }
  } else if (!strcmp(t, "wificfg")) {
    // Save updated network list
    if (doc["nets"].is<JsonArray>()) {
      JsonArray na = doc["nets"].as<JsonArray>();
      cfg.wifiCount = 0;
      for (JsonObject o : na) {
        if (cfg.wifiCount >= MAX_WIFI_NETS) break;
        strlcpy(cfg.wifiNets[cfg.wifiCount].ssid, o["s"] | "", sizeof(cfg.wifiNets[0].ssid));
        strlcpy(cfg.wifiNets[cfg.wifiCount].pass, o["p"] | "", sizeof(cfg.wifiNets[0].pass));
        if (cfg.wifiNets[cfg.wifiCount].ssid[0]) cfg.wifiCount++;
      }
    }
    uint8_t pref = (uint8_t)(doc["pref"] | cfg.wifiPref);
    cfg.wifiPref = pref;
    saveConfig();
    // Notify browser BEFORE switching (IP may change)
    const char* targetSSID = (pref == 0 && cfg.wifiCount > 0) ? cfg.wifiNets[0].ssid :
                             (pref >= 1 && pref <= cfg.wifiCount) ? cfg.wifiNets[pref-1].ssid :
                             WIFI_AP_SSID;
    JsonDocument nd;
    nd["e"]    = "wifi_switching";
    nd["ssid"] = targetSSID;
    nd["mdns"] = MDNS_HOST ".local";
    String ns; serializeJson(nd, ns);
    ws.textAll(ns);
    delay(300);   // let WS frame flush before IP changes
    switchWifi(pref);
    ws.textAll(buildCfgJson());
  }
}

// Accumulation buffer for multi-frame WebSocket messages.
// Large payloads (e.g. full config with 15 lua buttons) exceed a single TCP
// frame (~1400 B) and arrive fragmented.  We reassemble here before parsing.
static String g_wsRxBuf;

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    vlogf("[SBUS] WS#%u connected.\n", client->id());
    client->text(buildCfgJson());
  } else if (type == WS_EVT_DISCONNECT) {
    vlogf("[SBUS] WS#%u disconnected.\n", client->id());
    g_wsRxBuf = "";
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->opcode != WS_TEXT) return;

    // The library may deliver one logical message across multiple callbacks.
    // info->index = byte offset of this chunk within the frame.
    // info->len   = total frame length.
    // info->final = true on the last WebSocket frame of the message.
    // Done when: we have all bytes of this frame AND it is the final frame.
    if (info->index == 0) {
      g_wsRxBuf = "";
      g_wsRxBuf.reserve((size_t)info->len + 1);
    }
    g_wsRxBuf.concat((const char*)data, len);

    if (info->final && (info->index + len == info->len)) {
      handleWsMessage(client, g_wsRxBuf.c_str());
      g_wsRxBuf = "";
    }
  }
}

// =============================================================================
//  EMBEDDED HTML
// =============================================================================

#include "SBUSController_html.h"

// =============================================================================
//  SETUP
// =============================================================================
//  WIFI CONNECT
// =============================================================================

// pref: 0=auto cascade, 1..wifiCount=specific net, 255=AP only
void switchWifi(uint8_t pref) {
  g_wifiNet = -1;

  if (pref != 255) {
    WiFi.mode(WIFI_STA);

    auto tryNet = [](int idx) -> bool {
      if (idx < 0 || idx >= cfg.wifiCount || !cfg.wifiNets[idx].ssid[0]) return false;
      Serial.printf("[SBUS] Trying \"%s\"...\n", cfg.wifiNets[idx].ssid);
      WiFi.disconnect(false);
      WiFi.begin(cfg.wifiNets[idx].ssid, cfg.wifiNets[idx].pass);
      uint32_t t = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t < WIFI_STA_TIMEOUT_MS) delay(100);
      return WiFi.status() == WL_CONNECTED;
    };

    if (pref == 0) {
      for (int i = 0; i < cfg.wifiCount && g_wifiNet < 0; i++)
        if (tryNet(i)) g_wifiNet = i;
    } else {
      int idx = (int)pref - 1;
      if (tryNet(idx)) g_wifiNet = idx;
    }
  }

  if (g_wifiNet >= 0) {
    Serial.printf("[SBUS] Connected to \"%s\"\n", cfg.wifiNets[g_wifiNet].ssid);
    Serial.printf("[SBUS]   IP:   http://%s\n", WiFi.localIP().toString().c_str());
    if (MDNS.begin(MDNS_HOST))
      Serial.printf("[SBUS]   mDNS: http://%s.local\n", MDNS_HOST);
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.printf("[SBUS] AP mode  SSID: %s  Pass: %s\n", WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.printf("[SBUS]   IP:   http://%s\n", WiFi.softAPIP().toString().c_str());
  }
}

// =============================================================================

void setup() {
  // TX ring buffer so the boot prints below (and runtime vlogf) have headroom
  // to write into when no host is draining the port. On WCB HW 3.2 the USB
  // serial path back-pressures with no host attached; the default ~128 B TX
  // buffer fills mid-setup() and a plain Serial.print() then blocks forever,
  // so loop() is never reached and no SBUS goes out. Must precede begin().
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(400);
  Serial.println("\n[SBUS] SBUSController booting (X18 edition)...");

  // SBUS output: 100 kbaud, 8E2, inverted (GPIO9 = Serial5 TX on WCB v3.x)
  Serial1.begin(SBUS_BAUD, SERIAL_8E2, -1, SBUS_TX_PIN, true);
  Serial.printf("[SBUS] SBUS TX on GPIO%d  (100kbaud 8E2 inverted)\n", SBUS_TX_PIN);

  // RC PWM outputs on GPIO4/6/15/17 (Serial1-4 TX on WCB v3.x)
  initPwmOutputs();

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("[SBUS] ERROR: LittleFS mount failed!");
  } else {
    loadConfig();
  }

  // Init runtime state after config is loaded
  initRuntimeState();
  applyAllControls();

  // Start web server BEFORE WiFi (avoids AsyncTCP mutex race on ESP32 core 3.x)
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
    vlogf("[SBUS] HTTP GET /  from %s\n", req->client()->remoteIP().toString().c_str());
    req->send_P(200, "text/html", (const uint8_t*)HTML, sizeof(HTML) - 1);  // explicit length avoids uint16 overflow on large pages
  });
  // Lightweight health-check
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send(200, "text/plain", "pong");
  });
  // Config export — serve the raw LittleFS JSON so the browser can download it
  server.on("/cfg", HTTP_GET, [](AsyncWebServerRequest* req){
    if (LittleFS.exists(CONFIG_FILE))
      req->send(LittleFS, CONFIG_FILE, "application/json");
    else
      req->send(404, "text/plain", "No config saved yet");
  });
  server.onNotFound([](AsyncWebServerRequest* req){
    vlogf("[SBUS] 404: %s\n", req->url().c_str());
    req->send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("[SBUS] Web server started.");

  // WiFi — connect based on saved preference
  switchWifi(cfg.wifiPref);

  Serial.printf("[SBUS] Mode: SBUS-%d  (%d ch, %d bytes/frame)\n",
                sbusChCount(), sbusChCount(), sbusFrameLen());
  Serial.println("[SBUS] Serial cmds:  m=toggle mode  d=toggle debug  ?=status  w=wifi  w1-w4=switch net  wa=AP");
  Serial.println("[SBUS] Ready.");
}

// =============================================================================
//  LOOP
// =============================================================================

static void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;

    if (c == 'm') {
      g_sbus24   = !g_sbus24;
      cfg.sbus24 = g_sbus24;
      saveConfig();
      for (int i = 0; i < SBUS_CH_COUNT_24; i++) sbusChannels[i] = SBUS_CENTER;
      applyAllControls();
      Serial.printf("[SBUS] Mode -> SBUS-%d (%d bytes/frame)\n", sbusChCount(), sbusFrameLen());
      ws.textAll(buildCfgJson());

    } else if (c == 'd') {
#ifdef SBUS_DEBUG
      g_serialDebug = !g_serialDebug;
      Serial.printf("[SBUS] Serial debug %s\n", g_serialDebug ? "ENABLED" : "DISABLED");
#else
      Serial.println("[SBUS] Debug not compiled in — uncomment #define SBUS_DEBUG");
#endif

    } else if (c == 'w') {
      // WiFi switch: send 'w' then immediately '0'-'4' or 'a' (no Enter needed)
      // w0=auto  w1..w4=specific net  wa=AP only  w alone=show status
      delay(30);  // brief wait for the digit to arrive in the buffer
      // Drain whitespace/newlines so Enter alone doesn't trigger a switch
      while (Serial.available() && (Serial.peek() == '\r' || Serial.peek() == '\n' || Serial.peek() == ' '))
        Serial.read();
      {
        char nc = Serial.available() ? (char)Serial.read() : 0;
        bool doSwitch = false;
        uint8_t pref  = 255;
        if      (nc >= '0' && nc <= '4') { pref = nc - '0'; doSwitch = true; }
        else if (nc == 'a' || nc == 'A') { pref = 255;      doSwitch = true; }
        if (doSwitch) {
          Serial.printf("[WiFi] Switching (pref=%u)...\n", pref);
          switchWifi(pref);
          cfg.wifiPref = pref;
          saveConfig();
          ws.textAll(buildCfgJson());
        } else {
          // Status only
          if (g_wifiNet >= 0)
            Serial.printf("[WiFi] Connected: \"%s\"  IP: %s  (net %d)\n",
              cfg.wifiNets[g_wifiNet].ssid, WiFi.localIP().toString().c_str(), g_wifiNet + 1);
          else
            Serial.printf("[WiFi] AP mode  IP: %s\n", WiFi.softAPIP().toString().c_str());
          Serial.println("[WiFi] Cmds: w0=auto  w1-w4=net  wa=AP");
          for (int i = 0; i < cfg.wifiCount; i++)
            Serial.printf("  [%d] %s\n", i + 1, cfg.wifiNets[i].ssid);
        }
      }

    } else if (c == '?') {
      Serial.printf("[SBUS] Mode: SBUS-%d  |  Frame: %d bytes  |  Channels: %d",
                    sbusChCount(), sbusFrameLen(), sbusChCount());
#ifdef SBUS_DEBUG
      Serial.printf("  |  Monitor: %s  |  SerialDbg: %s", g_sbusDebug ? "ON" : "OFF", g_serialDebug ? "ON" : "OFF");
#endif
      Serial.println();
      Serial.println("[SBUS] Switch positions:");
      for (int i = 0; i < MAX_SWITCHES; i++)
        Serial.printf("  %s(CH%d)=pos%d  ", cfg.sw[i].label, cfg.sw[i].ch, swPos[i]);
      Serial.println();
      Serial.println("[SBUS] Trim values (offset from center):");
      for (int i = 0; i < MAX_TRIMS; i++)
        Serial.printf("  %s(CH%d)=%+d  ", cfg.trim[i].label, cfg.trim[i].ch,
                      (int)trimVal[i] - SBUS_CENTER);
      Serial.println();
    }
  }
}

void loop() {
  handleSerialCommands();

  uint32_t now = millis();
  if (now - lastFrameMs >= SBUS_FRAME_MS) {
    lastFrameMs = now;
    transmitSbus();
  }

  ws.cleanupClients();
}
