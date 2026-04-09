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
//     4 joystick axes  (Mode 2: RX=AIL, RY=ELE, LY=THR, LX=RUD)
//     8 switches       SA-SH  (3-pos / 2-pos / momentary)
//     2 sliders        LS, RS
//     6 trim rockers   T1-T6  (hold-to-repeat)
//     8 buttons        S1-S6, RB1, RB2
//
//  ─── Hardware Wiring ────────────────────────────────────────────────────────
//   Kyber SBUS input  ←  GPIO 5  (Serial1 TX, inverted 100 kbaud 8E2)
//   USB               ↔  Serial  (debug @ 115200)
//
//  ─── Default Channel Map ────────────────────────────────────────────────────
//   CH1=RX(AIL)  CH2=RY(ELE)  CH3=LY(THR)  CH4=LX(RUD)
//   CH5=SA  CH6=SB  CH7=SC  CH8=SD  CH9=SE  CH10=SF  CH11=SG  CH12=SH
//   CH13=LS  CH14=RS
//   CH15=T1  CH16=T2  CH17=T3  CH18=T4  CH19=T5  CH20=T6
//   CH21=S1  CH22=S2  CH23=S3  CH24=S4   S5/S6/RB1/RB2=unassigned
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
#define SBUS_TX_PIN     5    // Serial1 TX → Kyber SBUS input (inverted)

// ─── SBUS Protocol ────────────────────────────────────────────────────────────
#define SBUS_BAUD           100000
#define SBUS_HEADER         0x0F
#define SBUS_FOOTER         0x00
#define SBUS_FLAGS          0x00
#define SBUS_MIN            172
#define SBUS_MAX            1811
#define SBUS_CENTER         992
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
bool g_sbusDebug = false;
#endif

// ─── Config ───────────────────────────────────────────────────────────────────
#define CONFIG_FILE   "/config.json"
#define CFG_VER       2
#define MAX_SWITCHES  8    // SA SB SC SD SE SF SG SH
#define MAX_SLIDERS   4    // LS RS S1 S2  (S1/S2 are the centre pots)
#define MAX_TRIMS     6    // T1-T6
#define MAX_BUTTONS   8    // S1-S6 + RB1 RB2  (physical momentary switches)
#define MAX_LUA_BTNS  15   // configurable Lua / virtual buttons

enum SwType : uint8_t { SW_3POS=0, SW_2POS=1, SW_MOMENT=2 };

struct SwCfg {
  char     label[4];    // "SA".."SH"
  uint8_t  ch;          // 1-based SBUS channel; 0=unassigned
  SwType   type;
  uint16_t val[3];      // SBUS values for positions 0,1,2
};

struct SliderCfg {
  char    label[4];     // "LS" or "RS"
  uint8_t ch;
};

struct TrimCfg {
  char    label[4];     // "T1".."T6"
  uint8_t ch;
  uint8_t step;         // SBUS units per click
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
  // WiFi networks (tried in order; 0=auto cascade)
  WifiNetCfg wifiNets[MAX_WIFI_NETS];
  uint8_t    wifiCount;  // number of configured networks
  uint8_t    wifiPref;   // 0=auto, 1..wifiCount=specific net, 255=AP only
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

  // Switches SA-SH
  const char* swLbls[]   = {"SA","SB","SC","SD","SE","SF","SG","SH"};
  SwType      swTypes[]  = {SW_3POS,SW_3POS,SW_3POS,SW_3POS,SW_3POS,SW_2POS,SW_3POS,SW_MOMENT};
  for (int i = 0; i < MAX_SWITCHES; i++) {
    strlcpy(cfg.sw[i].label, swLbls[i], 4);
    cfg.sw[i].ch   = 5 + i;   // CH5..CH12
    cfg.sw[i].type = swTypes[i];
    // 3-pos: low/center/high; 2-pos & momentary: low/high/(unused)
    cfg.sw[i].val[0] = SBUS_MIN;
    cfg.sw[i].val[1] = (swTypes[i] == SW_3POS) ? SBUS_CENTER : SBUS_MAX;
    cfg.sw[i].val[2] = SBUS_MAX;
  }

  // Sliders: LS, RS (sides of sticks), S1 & S2 (centre pots)
  strlcpy(cfg.slider[0].label, "LS", 4);  cfg.slider[0].ch = 13;
  strlcpy(cfg.slider[1].label, "RS", 4);  cfg.slider[1].ch = 14;
  strlcpy(cfg.slider[2].label, "S1", 4);  cfg.slider[2].ch = 0;  // unassigned — user sets channel
  strlcpy(cfg.slider[3].label, "S2", 4);  cfg.slider[3].ch = 0;

  // Trims T1-T6
  for (int i = 0; i < MAX_TRIMS; i++) {
    char lb[4]; snprintf(lb, 4, "T%d", i + 1);
    strlcpy(cfg.trim[i].label, lb, 4);
    cfg.trim[i].ch   = 15 + i;   // CH15..CH20
    cfg.trim[i].step = 10;
  }

  // Axis output range — full range by default; swap min/max to reverse
  for (int i = 0; i < 4; i++) { cfg.axisMin[i] = SBUS_MIN; cfg.axisMax[i] = SBUS_MAX; }

  // Lua / virtual buttons (configurable, restored from original design)
  for (int i = 0; i < MAX_LUA_BTNS; i++) {
    snprintf(cfg.luaBtn[i].label, sizeof(cfg.luaBtn[i].label), "Button %d", i + 1);
    cfg.luaBtn[i].ch  = 0;
    cfg.luaBtn[i].val = SBUS_MAX;
    strlcpy(cfg.luaBtn[i].color, "#4fc3f7", sizeof(cfg.luaBtn[i].color));
  }

  // Physical momentary buttons S1-S4 assigned, S5/S6/RB1/RB2 unassigned
  const char* btnLbls[] = {"S1","S2","S3","S4","S5","S6","RB1","RB2"};
  for (int i = 0; i < MAX_BUTTONS; i++) {
    strlcpy(cfg.btn[i].label, btnLbls[i], sizeof(cfg.btn[i].label));
    cfg.btn[i].ch  = (i < 4) ? 21 + i : 0;  // S1=CH21..S4=CH24
    cfg.btn[i].val = SBUS_MAX;
  }

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
}

void initRuntimeState() {
  for (int i = 0; i < SBUS_CH_COUNT_24; i++) sbusChannels[i] = SBUS_CENTER;
  for (int i = 0; i < MAX_SWITCHES; i++)
    swPos[i] = (cfg.sw[i].type == SW_3POS) ? 1 : 0;
  for (int i = 0; i < MAX_SLIDERS; i++) sliderPct[i] = 50;
  for (int i = 0; i < MAX_TRIMS; i++)   trimVal[i]   = SBUS_CENTER;
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
    if (tr.ch >= 1 && tr.ch <= SBUS_CH_COUNT_24)
      sbusChannels[tr.ch - 1] = (uint16_t)constrain((int)trimVal[i], SBUS_MIN, SBUS_MAX);
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
    cfg.sw[idx].ch   = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    cfg.sw[idx].type = (SwType)constrain((int)(o["t"] | 0), 0, 2);
    if (o["v"].is<JsonArray>()) {
      JsonArray va = o["v"].as<JsonArray>();
      for (int j = 0; j < 3; j++)
        cfg.sw[idx].val[j] = constrain((int)(va[j] | SBUS_CENTER), SBUS_MIN, SBUS_MAX);
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
    idx++;
  }

  JsonArray btnArr = doc["btn"].as<JsonArray>();
  idx = 0;
  for (JsonObject o : btnArr) {
    if (idx >= MAX_BUTTONS) break;
    strlcpy(cfg.btn[idx].label, o["l"] | "", sizeof(cfg.btn[idx].label));
    cfg.btn[idx].ch  = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    cfg.btn[idx].val = constrain((int)(o["v"] | SBUS_MAX), SBUS_MIN, SBUS_MAX);
    idx++;
  }
  // Lua buttons
  JsonArray luaArr = doc["lua"].as<JsonArray>();
  idx = 0;
  for (JsonObject o : luaArr) {
    if (idx >= MAX_LUA_BTNS) break;
    strlcpy(cfg.luaBtn[idx].label, o["l"] | cfg.luaBtn[idx].label, sizeof(cfg.luaBtn[idx].label));
    cfg.luaBtn[idx].ch  = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    cfg.luaBtn[idx].val = constrain((int)(o["v"] | SBUS_MAX), SBUS_MIN, SBUS_MAX);
    { const char* kv = o["k"] | ""; strlcpy(cfg.luaBtn[idx].color, kv[0] ? kv : cfg.luaBtn[idx].color, sizeof(cfg.luaBtn[idx].color)); }
    idx++;
  }
  // Axis range
  if (doc["aMin"].is<JsonArray>()) {
    JsonArray mn = doc["aMin"].as<JsonArray>();
    for (int i = 0; i < 4; i++) cfg.axisMin[i] = constrain((int)(mn[i] | SBUS_MIN), SBUS_MIN, SBUS_MAX);
  }
  if (doc["aMax"].is<JsonArray>()) {
    JsonArray mx = doc["aMax"].as<JsonArray>();
    for (int i = 0; i < 4; i++) cfg.axisMax[i] = constrain((int)(mx[i] | SBUS_MAX), SBUS_MIN, SBUS_MAX);
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
    o["l"] = cfg.trim[i].label;
    o["c"] = cfg.trim[i].ch;
    o["s"] = cfg.trim[i].step;
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

  doc["wifiPref"] = cfg.wifiPref;
  JsonArray wArr = doc.createNestedArray("wifiNets");
  for (int i = 0; i < cfg.wifiCount; i++) {
    JsonObject o = wArr.createNestedObject();
    o["s"] = cfg.wifiNets[i].ssid;
    o["p"] = cfg.wifiNets[i].pass;
  }

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
    uint16_t val = constrain(sbusChannels[i], SBUS_MIN, SBUS_MAX);
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

static void printSbusDebug(const uint8_t* frame, int flen, int chcnt) {
  if (!g_sbusDebug) return;
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
  return (uint16_t)constrain((int)(mapped + 0.5f), SBUS_MIN, SBUS_MAX);
}

// =============================================================================
//  WEBSOCKET HANDLER
// =============================================================================

void handleWsMessage(AsyncWebSocketClient* client, const char* json) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) return;
  const char* t = doc["t"] | "";

  // ── Joystick axes ────────────────────────────────────────────────────────
  // { "t":"a", "lx":f, "ly":f, "rx":f, "ry":f }
  if (!strcmp(t, "a")) {
    sbusChannels[cfg.joyRX - 1] = axisToSbusRange( (doc["rx"] | 0.0f),  cfg.axisMin[0], cfg.axisMax[0]);
    sbusChannels[cfg.joyRY - 1] = axisToSbusRange(-(doc["ry"] | 0.0f),  cfg.axisMin[1], cfg.axisMax[1]);
    sbusChannels[cfg.joyLY - 1] = axisToSbusRange(-(doc["ly"] | 0.0f),  cfg.axisMin[2], cfg.axisMax[2]);
    sbusChannels[cfg.joyLX - 1] = axisToSbusRange( (doc["lx"] | 0.0f),  cfg.axisMin[3], cfg.axisMax[3]);
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

  // ── Trim delta ────────────────────────────────────────────────────────────
  // { "t":"tr", "i":idx, "d":delta }  d: +1/-1 step, 0 = reset to center
  else if (!strcmp(t, "tr")) {
    int idx   = doc["i"] | -1;
    int delta = doc["d"] | 0;
    if (idx >= 0 && idx < MAX_TRIMS) {
      auto& tr = cfg.trim[idx];
      if (delta == 0) {
        trimVal[idx] = SBUS_CENTER;
      } else {
        int newVal = trimVal[idx] + (delta > 0 ? (int)tr.step : -(int)tr.step);
        trimVal[idx] = (int16_t)constrain(newVal, SBUS_MIN, SBUS_MAX);
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

    // Optional switch channel updates
    if (doc["sw"].is<JsonArray>()) {
      JsonArray arr = doc["sw"].as<JsonArray>();
      int i = 0;
      for (JsonObject o : arr) {
        if (i >= MAX_SWITCHES) break;
        strlcpy(cfg.sw[i].label, o["l"] | cfg.sw[i].label, 4);
        cfg.sw[i].ch = constrain((int)(o["c"] | cfg.sw[i].ch), 0, SBUS_CH_COUNT_24);
        if (o["v"].is<JsonArray>()) {
          JsonArray va = o["v"].as<JsonArray>();
          for (int j = 0; j < 3; j++)
            cfg.sw[i].val[j] = constrain((int)(va[j] | cfg.sw[i].val[j]), SBUS_MIN, SBUS_MAX);
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
        cfg.btn[i].val = constrain((int)(o["v"] | cfg.btn[i].val), SBUS_MIN, SBUS_MAX);
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
        cfg.luaBtn[i].val = constrain((int)(o["v"] | cfg.luaBtn[i].val), SBUS_MIN, SBUS_MAX);
        { const char* kv = o["k"] | ""; strlcpy(cfg.luaBtn[i].color, kv[0] ? kv : cfg.luaBtn[i].color, sizeof(cfg.luaBtn[i].color)); }
        i++;
      }
    }
    // Axis range
    if (doc["aMin"].is<JsonArray>()) {
      JsonArray mn = doc["aMin"].as<JsonArray>();
      for (int i = 0; i < 4; i++) cfg.axisMin[i] = constrain((int)(mn[i] | SBUS_MIN), SBUS_MIN, SBUS_MAX);
    }
    if (doc["aMax"].is<JsonArray>()) {
      JsonArray mx = doc["aMax"].as<JsonArray>();
      for (int i = 0; i < 4; i++) cfg.axisMax[i] = constrain((int)(mx[i] | SBUS_MAX), SBUS_MIN, SBUS_MAX);
    }
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
    Serial.printf("[SBUS] WS#%u connected.\n", client->id());
    client->text(buildCfgJson());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[SBUS] WS#%u disconnected.\n", client->id());
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

static const char HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,user-scalable=no">
<title>SBUS Controller</title>
<style>
  :root {
    --bg:       #0d0f14;
    --panel:    #161920;
    --border:   #2a2d38;
    --accent:   #4fc3f7;
    --green:    #66bb6a;
    --red:      #ef5350;
    --yellow:   #ffa726;
    --text:     #e0e4f0;
    --muted:    #6b7280;
    --stick-bg: #1a1d28;
    --stick-rim:#2e3450;
    --thumb:    #4fc3f7;
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{
    background:var(--bg);color:var(--text);
    font-family:'Segoe UI',system-ui,sans-serif;
    min-height:100vh;display:flex;flex-direction:column;
    align-items:center;padding:12px;gap:10px;
  }

  /* ── Header ────────────────────────────────────────────────────────────── */
  header{
    width:100%;max-width:960px;
    display:flex;align-items:center;justify-content:space-between;
    flex-wrap:wrap;gap:8px;
  }
  .logo{font-size:1.05rem;font-weight:700;letter-spacing:.1em;color:var(--accent);}
  #connBadge{
    font-size:.7rem;font-weight:600;letter-spacing:.06em;
    padding:4px 12px;border-radius:20px;
    background:var(--panel);border:1px solid var(--red);color:var(--red);
    transition:all .3s;
  }
  #connBadge.connected{border-color:var(--green);color:var(--green);}
  .hdr-btn{
    font-size:.7rem;font-weight:700;letter-spacing:.07em;
    padding:4px 12px;border-radius:20px;cursor:pointer;
    background:var(--panel);border:1px solid var(--border);color:var(--muted);
    transition:all .2s;user-select:none;
  }
  .hdr-btn.active-16{border-color:var(--yellow);color:var(--yellow);}
  .hdr-btn.active-24{border-color:var(--accent);color:var(--accent);}
  .hdr-btn.dbg-on   {border-color:var(--green);color:var(--green);}

  /* ── Switch pyramids: SF/SH top, SE/SG mid, SA+SB / SC+SD bottom pair ─── */
  .sw-pyramid{
    display:flex;flex-direction:column;align-items:center;gap:5px;flex-shrink:0;
  }
  .sw-pair{
    display:flex;flex-direction:row;gap:5px;
  }
  .sw-card{
    background:var(--panel);border:1px solid var(--border);border-radius:10px;
    padding:7px 6px;display:flex;flex-direction:column;align-items:center;gap:4px;
    width:72px;
  }
  .sw-name{font-size:.62rem;font-weight:700;letter-spacing:.1em;color:var(--accent);text-transform:uppercase;}
  .sw-ch  {font-size:.55rem;color:var(--muted);}
  .sw-toggle{display:flex;flex-direction:column;gap:3px;width:100%;}
  .sw-seg{
    padding:5px 0;width:100%;border-radius:5px;cursor:pointer;
    border:1px solid var(--border);background:var(--bg);
    color:var(--muted);font-size:.68rem;font-weight:600;text-align:center;
    transition:all .12s;user-select:none;
  }
  .sw-seg.sel{background:var(--accent);color:#000;border-color:var(--accent);}
  .sw-seg:hover:not(.sel){border-color:var(--accent);color:var(--text);}
  .sw-mom{
    padding:10px 0;width:100%;border-radius:6px;cursor:pointer;
    border:1px solid var(--border);background:var(--bg);
    color:var(--text);font-size:.75rem;font-weight:700;text-align:center;
    user-select:none;touch-action:none;transition:all .1s;
  }
  .sw-mom.held{background:var(--accent);color:#000;border-color:var(--accent);}

  /* ── Main control area ──────────────────────────────────────────────────── */
  .ctrl-area{
    width:100%;max-width:960px;
    display:flex;gap:6px;align-items:flex-start;justify-content:center;flex-wrap:wrap;
  }

  /* Trim+Slider outer columns */
  .ts-col{
    display:flex;flex-direction:column;align-items:center;gap:8px;
    flex:0 0 auto;
  }

  /* Trim widget */
  .trim-widget{
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:7px 8px;display:flex;flex-direction:column;align-items:center;gap:3px;
  }
  .trim-lbl{font-size:.58rem;font-weight:700;color:var(--accent);letter-spacing:.08em;}
  .trim-ch {font-size:.52rem;color:var(--muted);}
  .trim-btn{
    width:40px;height:26px;border-radius:4px;cursor:pointer;
    border:1px solid var(--border);background:var(--bg);color:var(--text);
    font-size:.8rem;font-weight:700;user-select:none;touch-action:none;
    transition:background .1s,border-color .1s;
  }
  .trim-btn:active,.trim-btn.held{background:var(--accent);color:#000;border-color:var(--accent);}
  .trim-val{
    font-size:.65rem;font-variant-numeric:tabular-nums;
    color:var(--text);min-width:40px;text-align:center;
  }
  .trim-rst{
    width:40px;height:16px;border-radius:3px;cursor:pointer;
    border:1px solid var(--border);background:transparent;color:var(--muted);
    font-size:.52rem;font-weight:700;
  }

  /* Slider widget */
  .slider-widget{
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:8px;display:flex;flex-direction:column;align-items:center;gap:5px;
  }
  .slider-lbl{font-size:.6rem;font-weight:700;color:var(--accent);letter-spacing:.08em;}
  .slider-ch {font-size:.52rem;color:var(--muted);}
  .slider-wrap{width:40px;height:160px;display:flex;align-items:center;justify-content:center;overflow:visible;}
  .slider-inp{
    width:140px;
    transform:rotate(-90deg);transform-origin:center;
    cursor:pointer;accent-color:var(--accent);
  }
  .slider-val{font-size:.65rem;color:var(--muted);font-variant-numeric:tabular-nums;}

  /* Stick block */
  .stick-block{
    display:flex;flex-direction:column;align-items:center;gap:8px;flex:0 0 auto;
  }
  .stick-card{
    background:var(--panel);border:1px solid var(--border);border-radius:12px;
    padding:12px;display:flex;flex-direction:column;align-items:center;gap:7px;
  }
  .stick-lbl{font-size:.62rem;font-weight:700;letter-spacing:.09em;color:var(--muted);text-transform:uppercase;}
  .stick-wrap{
    position:relative;width:min(220px,38vw);aspect-ratio:1;
    border-radius:50%;background:var(--stick-bg);border:2px solid var(--stick-rim);
    touch-action:none;user-select:none;cursor:crosshair;
  }
  .stick-wrap::before,.stick-wrap::after{content:'';position:absolute;background:var(--stick-rim);}
  .stick-wrap::before{width:1px;height:100%;left:50%;top:0;}
  .stick-wrap::after {width:100%;height:1px;top:50%;left:0;}
  .thumb{
    position:absolute;width:36px;height:36px;border-radius:50%;
    background:radial-gradient(circle at 35% 35%,#7fd6ff,var(--thumb));
    box-shadow:0 0 10px rgba(79,195,247,.4);
    transform:translate(-50%,-50%);pointer-events:none;
  }
  .stick-readout{font-size:.65rem;color:var(--muted);font-variant-numeric:tabular-nums;white-space:nowrap;}

  /* Trim row below sticks */
  .trim-row{display:flex;gap:8px;justify-content:center;}
  .trim-h{
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:6px 8px;display:flex;flex-direction:column;align-items:center;gap:3px;
  }
  .trim-h-btns{display:flex;gap:4px;align-items:center;}

  /* ── Stick center wrapper (sticks + tucked-under controls) ─────────────── */
  .stick-center{
    display:flex;flex-direction:column;align-items:center;gap:6px;flex:0 0 auto;
  }
  .sticks-row{
    display:flex;gap:10px;align-items:flex-start;
  }

  /* ── S1/S2 pot row ─────────────────────────────────────────────────────── */
  .pot-row{
    display:flex;gap:12px;justify-content:center;flex-wrap:wrap;width:100%;
  }
  .pot-widget{
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:8px 16px;display:flex;flex-direction:column;align-items:center;gap:5px;
    min-width:160px;
  }
  .pot-lbl{font-size:.62rem;font-weight:700;color:var(--accent);letter-spacing:.08em;}
  .pot-ch {font-size:.52rem;color:var(--muted);}
  .pot-inp{width:180px;cursor:pointer;accent-color:var(--accent);}
  .pot-val{font-size:.65rem;color:var(--muted);font-variant-numeric:tabular-nums;}

  /* ── Trim bank (all trims in a row, tucked under sticks) ─────────────── */
  .trim-bank{
    display:flex;gap:6px;justify-content:center;flex-wrap:wrap;width:100%;
  }

  /* ── Lua button grid ────────────────────────────────────────────────────── */
  .lua-section{width:100%;max-width:960px;}
  .lua-header{margin-bottom:6px;}
  .lua-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:7px;}
  @media(max-width:600px){.lua-grid{grid-template-columns:repeat(3,1fr);}}
  .lua-btn{
    --btn-color: #4fc3f7;
    padding:14px 6px;
    border:1px solid var(--border);border-radius:8px;
    background:#1a1d28;color:var(--text);
    font-size:.75rem;font-weight:600;cursor:pointer;text-align:center;word-break:break-word;
    transition:border-color .12s,background .12s,color .12s,transform .1s;
    -webkit-tap-highlight-color:transparent;user-select:none;touch-action:none;
  }
  .lua-btn:not(.unassigned){background:var(--btn-color);color:#000;border-color:var(--btn-color);}
  .lua-btn.unassigned{background:transparent;border-color:var(--btn-color);border-style:dashed;color:var(--muted);opacity:.55;cursor:default;}
  .lua-btn.pressed{filter:brightness(.75);transform:scale(.96);}
  .lua-btn:not(.unassigned):hover{filter:brightness(1.15);}

  /* ── Button bank ────────────────────────────────────────────────────────── */
  .btn-bank{
    display:flex;gap:6px;flex-wrap:wrap;justify-content:center;width:100%;
  }
  .btn-group{display:flex;gap:6px;flex-wrap:wrap;justify-content:center;}
  .sep{width:1px;background:var(--border);align-self:stretch;margin:0 4px;}
  .ctrl-btn{
    padding:12px 8px;min-width:56px;
    border:1px solid var(--border);border-radius:8px;
    background:#1a1d28;color:var(--text);
    font-size:.75rem;font-weight:600;cursor:pointer;text-align:center;
    transition:all .12s;user-select:none;touch-action:none;
    -webkit-tap-highlight-color:transparent;
  }
  .ctrl-btn.unassigned{color:var(--muted);border-style:dashed;cursor:default;}
  .ctrl-btn.pressed{background:var(--accent);color:#000;border-color:var(--accent);}
  .ctrl-btn:not(.unassigned):hover{border-color:var(--accent);}

  /* ── Settings panel ─────────────────────────────────────────────────────── */
  .settings-wrap{width:100%;max-width:960px;}
  details{width:100%;}
  summary{
    cursor:pointer;font-size:.7rem;font-weight:700;letter-spacing:.08em;text-transform:uppercase;
    color:var(--muted);padding:9px 14px;
    background:var(--panel);border:1px solid var(--border);border-radius:10px;
    list-style:none;display:flex;align-items:center;gap:8px;user-select:none;
  }
  summary::-webkit-details-marker{display:none;}
  summary::before{content:'▶';transition:transform .2s;font-size:.58rem;}
  details[open] summary{border-radius:10px 10px 0 0;}
  details[open] summary::before{transform:rotate(90deg);}
  .settings-body{
    background:var(--panel);border:1px solid var(--border);border-top:none;
    border-radius:0 0 10px 10px;padding:14px;display:flex;flex-direction:column;gap:14px;
  }
  .sec-title{font-size:.62rem;font-weight:700;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);margin-bottom:4px;}
  .cfg-table{width:100%;border-collapse:collapse;font-size:.74rem;}
  .cfg-table th{font-size:.6rem;font-weight:700;letter-spacing:.07em;text-transform:uppercase;
    color:var(--muted);padding:4px 6px;text-align:left;border-bottom:1px solid var(--border);}
  .cfg-table td{padding:4px 6px;vertical-align:middle;}
  .cfg-table tr:nth-child(even) td{background:rgba(255,255,255,.02);}
  .cfg-table input,.cfg-table select{
    background:var(--bg);border:1px solid var(--border);border-radius:4px;
    color:var(--text);padding:4px 6px;font-size:.72rem;width:100%;
  }
  .cfg-table input:focus,.cfg-table select:focus{outline:none;border-color:var(--accent);}
  .axis-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;}
  .axis-item{display:flex;flex-direction:column;gap:4px;}
  .axis-item label{font-size:.65rem;color:var(--muted);}
  .axis-item select{background:var(--bg);border:1px solid var(--border);border-radius:4px;
    color:var(--text);padding:5px 6px;font-size:.74rem;}
  .save-btn{
    padding:8px 20px;border-radius:8px;
    border:1px solid var(--accent);background:transparent;color:var(--accent);
    font-size:.78rem;font-weight:700;cursor:pointer;transition:all .15s;
  }
  .save-btn:hover{background:var(--accent);color:#000;}
  .cfg-io-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;justify-content:flex-end;}
  .io-btn{
    padding:8px 16px;border-radius:8px;font-size:.78rem;font-weight:700;cursor:pointer;transition:all .15s;
  }
  .io-btn.export{border:1px solid var(--green);background:transparent;color:var(--green);}
  .io-btn.export:hover{background:var(--green);color:#000;}
  .io-btn.import{border:1px solid var(--yellow);background:transparent;color:var(--yellow);}
  .io-btn.import:hover{background:var(--yellow);color:#000;}
  #importStatus{font-size:.72rem;color:var(--muted);}

  /* ── WiFi panel ─────────────────────────────────────────────────────────── */
  .wifi-status{font-size:.72rem;color:var(--muted);display:flex;gap:14px;flex-wrap:wrap;margin-bottom:8px;}
  .wifi-status b{color:var(--text);}
  .wifi-net-row{display:grid;grid-template-columns:auto 1fr 1fr auto auto auto;gap:6px;align-items:center;margin-bottom:5px;}
  .wifi-net-row .drag-num{font-size:.65rem;color:var(--muted);text-align:center;width:18px;}
  .wifi-net-row input{background:var(--bg);border:1px solid var(--border);border-radius:4px;color:var(--text);padding:4px 6px;font-size:.72rem;}
  .wifi-net-row input:focus{outline:none;border-color:var(--accent);}
  .wifi-pass-wrap{position:relative;display:flex;}
  .wifi-pass-wrap input{flex:1;padding-right:28px;}
  .wifi-pass-wrap button{position:absolute;right:4px;top:50%;transform:translateY(-50%);
    background:none;border:none;color:var(--muted);cursor:pointer;font-size:.8rem;padding:0;}
  .wifi-arrow{background:none;border:1px solid var(--border);border-radius:4px;color:var(--muted);
    cursor:pointer;font-size:.7rem;padding:3px 6px;line-height:1;}
  .wifi-arrow:hover{border-color:var(--accent);color:var(--accent);}
  .wifi-del{background:none;border:1px solid var(--border);border-radius:4px;color:var(--muted);
    cursor:pointer;font-size:.7rem;padding:3px 6px;}
  .wifi-del:hover{border-color:var(--red);color:var(--red);}
  .wifi-add{background:none;border:1px dashed var(--border);border-radius:6px;color:var(--muted);
    cursor:pointer;font-size:.72rem;padding:5px;width:100%;margin-top:4px;transition:all .15s;}
  .wifi-add:hover{border-color:var(--accent);color:var(--accent);}
  .wifi-footer{display:flex;gap:8px;align-items:center;margin-top:10px;flex-wrap:wrap;}
  .wifi-footer label{font-size:.7rem;color:var(--muted);}
  .wifi-footer select{background:var(--bg);border:1px solid var(--border);border-radius:4px;
    color:var(--text);padding:4px 8px;font-size:.72rem;}
  .wifi-apply{padding:7px 18px;border-radius:8px;border:1px solid var(--accent);
    background:transparent;color:var(--accent);font-size:.78rem;font-weight:700;cursor:pointer;transition:all .15s;}
  .wifi-apply:hover{background:var(--accent);color:#000;}
  #wifiMsg{font-size:.72rem;color:var(--yellow);display:none;}

  /* ── Debug panel ────────────────────────────────────────────────────────── */
  .dbg-grid{display:grid;grid-template-columns:repeat(8,1fr);gap:4px;}
  @media(max-width:600px){.dbg-grid{grid-template-columns:repeat(4,1fr);}}
  .dbg-cell{background:var(--bg);border:1px solid var(--border);border-radius:4px;
    padding:4px;text-align:center;transition:border-color .1s;}
  .dbg-cell .dcn{font-size:.55rem;color:var(--muted);}
  .dbg-cell .dcv{font-size:.75rem;font-variant-numeric:tabular-nums;color:var(--text);display:block;margin-top:1px;}
  .dbg-cell.lit{border-color:var(--accent);}
  .dbg-cell.lit .dcv{color:var(--accent);}
  .dbg-info{font-size:.65rem;color:var(--muted);margin-top:5px;display:flex;gap:14px;flex-wrap:wrap;}
  .dbg-info .di-val{color:var(--text);}

  @media(max-width:620px){
    .ctrl-area{flex-direction:column;align-items:center;}
    .axis-grid{grid-template-columns:repeat(2,1fr);}
  }
</style>
</head>
<body>

<!-- ── Header ──────────────────────────────────────────────────────────────── -->
<header>
  <div class="logo">SBUS CONTROLLER</div>
  <div style="display:flex;gap:7px;align-items:center;flex-wrap:wrap;">
    <button class="hdr-btn" id="modeBtn"  onclick="toggleMode()">SBUS-24</button>
    <button class="hdr-btn" id="debugBtn" onclick="toggleDebug()">DEBUG OFF</button>
    <div id="connBadge">DISCONNECTED</div>
  </div>
</header>

<!-- ── Lua / virtual buttons — top of page for easy access ─────────────────── -->
<div class="lua-section" id="luaSection">
  <div class="lua-header">
    <span style="font-size:.62rem;font-weight:700;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);">Lua Buttons</span>
  </div>
  <div class="lua-grid" id="luaGrid"></div>
</div>

<!-- ── Main row: [L-pyramid] [LS] [stick-center] [RS] [R-pyramid] -->
<div class="ctrl-area">

  <!-- Left pyramid: SF top, SE mid, SA+SB bottom pair -->
  <div class="sw-pyramid" id="swPyramidLeft"></div>

  <!-- LS slider -->
  <div class="ts-col" id="slLSCol"></div>

  <!-- Center: sticks + all sub-controls tucked underneath -->
  <div class="stick-center">
    <div class="sticks-row">

      <!-- Left stick -->
      <div class="stick-block">
        <div class="stick-card">
          <div class="stick-lbl" id="lblL">LEFT  LX:CH4  LY:CH3</div>
          <div class="stick-wrap" id="stickL">
            <div class="thumb" id="thumbL" style="left:50%;top:50%"></div>
          </div>
          <div class="stick-readout" id="readL">LX: 992&nbsp;&nbsp;LY: 992</div>
        </div>
      </div>

      <!-- Right stick -->
      <div class="stick-block">
        <div class="stick-card">
          <div class="stick-lbl" id="lblR">RIGHT  RX:CH1  RY:CH2</div>
          <div class="stick-wrap" id="stickR">
            <div class="thumb" id="thumbR" style="left:50%;top:50%"></div>
          </div>
          <div class="stick-readout" id="readR">RX: 992&nbsp;&nbsp;RY: 992</div>
        </div>
      </div>

    </div><!-- /sticks-row -->

    <!-- S1/S2 pots tucked directly under sticks -->
    <div class="pot-row" id="potRow"></div>

    <!-- Trims tucked under pots -->
    <div class="trim-bank" id="trimBank"></div>

    <!-- Physical buttons tucked under trims -->
    <div class="btn-bank" id="btnBank"></div>

  </div><!-- /stick-center -->

  <!-- RS slider -->
  <div class="ts-col" id="slRSCol"></div>

  <!-- Right pyramid: SH top, SG mid, SC+SD bottom pair -->
  <div class="sw-pyramid" id="swPyramidRight"></div>

</div>

<!-- ── Settings ─────────────────────────────────────────────────────────────── -->
<div class="settings-wrap">
  <details>
    <summary>Settings</summary>
    <div class="settings-body">

      <div>
        <div class="sec-title">Joystick Channels &amp; Range</div>
        <table class="cfg-table" style="min-width:480px;">
          <thead><tr><th>Axis</th><th>Channel</th><th>Min (SBUS)</th><th>Max (SBUS)</th><th style="font-size:.55rem;color:var(--muted)">swap min/max to reverse</th></tr></thead>
          <tbody>
            <tr><td>Right X (AIL)</td><td><select id="selRX"></select></td><td><input type="number" id="axMin0" min="172" max="1811" value="172" style="width:70px"></td><td><input type="number" id="axMax0" min="172" max="1811" value="1811" style="width:70px"></td><td></td></tr>
            <tr><td>Right Y (ELE)</td><td><select id="selRY"></select></td><td><input type="number" id="axMin1" min="172" max="1811" value="172" style="width:70px"></td><td><input type="number" id="axMax1" min="172" max="1811" value="1811" style="width:70px"></td><td></td></tr>
            <tr><td>Left Y (THR)</td> <td><select id="selLY"></select></td><td><input type="number" id="axMin2" min="172" max="1811" value="172" style="width:70px"></td><td><input type="number" id="axMax2" min="172" max="1811" value="1811" style="width:70px"></td><td></td></tr>
            <tr><td>Left X (RUD)</td> <td><select id="selLX"></select></td><td><input type="number" id="axMin3" min="172" max="1811" value="172" style="width:70px"></td><td><input type="number" id="axMax3" min="172" max="1811" value="1811" style="width:70px"></td><td></td></tr>
          </tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Switches</div>
        <table class="cfg-table">
          <thead><tr><th>Name</th><th>Channel</th><th>Low Val</th><th>Mid Val</th><th>High Val</th></tr></thead>
          <tbody id="swCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Sliders &amp; Pots (LS / RS / S1 / S2)</div>
        <table class="cfg-table">
          <thead><tr><th>Name</th><th>Channel</th></tr></thead>
          <tbody id="slCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Trims</div>
        <table class="cfg-table">
          <thead><tr><th>Name</th><th>Channel</th><th>Step (SBUS units)</th></tr></thead>
          <tbody id="trCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Physical Buttons (S1–S6, RB1, RB2)</div>
        <table class="cfg-table">
          <thead><tr><th>Name</th><th>Label</th><th>Channel</th><th>Pressed Value</th></tr></thead>
          <tbody id="btnCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Lua Buttons (15 virtual)</div>
        <table class="cfg-table">
          <thead><tr><th>#</th><th>Label</th><th>Channel</th><th>Pressed Value</th><th>Color</th></tr></thead>
          <tbody id="luaCfgBody"></tbody>
        </table>
      </div>

      <div class="cfg-io-row">
        <span id="importStatus"></span>
        <button class="io-btn export" onclick="exportConfig()">&#11015; Export JSON</button>
        <label class="io-btn import" style="display:inline-block;">
          &#11014; Import JSON
          <input type="file" id="importFile" accept=".json,application/json"
                 style="display:none" onchange="importConfig(this)">
        </label>
        <button class="save-btn" onclick="saveConfig()">&#128190; Save Config</button>
      </div>
    </div>
  </details>
</div>

<!-- ── WiFi panel ────────────────────────────────────────────────────────────── -->
<div class="settings-wrap">
  <details id="wifiDetails">
    <summary>WiFi Networks</summary>
    <div class="settings-body">
      <div class="wifi-status">
        <span>Status: <b id="wifiStatusSSID">—</b></span>
        <span>IP: <b id="wifiStatusIP">—</b></span>
        <span>mDNS: <b id="wifiStatusMDNS">—</b></span>
      </div>
      <div id="wifiNetList"></div>
      <button class="wifi-add" onclick="wifiAddRow()">+ Add Network</button>
      <div class="wifi-footer">
        <label>Connect:</label>
        <select id="wifiPrefSel"></select>
        <button class="wifi-apply" onclick="wifiApply()">&#8646; Save &amp; Connect</button>
        <span id="wifiMsg"></span>
      </div>
    </div>
  </details>
</div>

<!-- ── Debug panel ───────────────────────────────────────────────────────────── -->
<div class="settings-wrap">
  <details id="dbgDetails" open>
    <summary>Live Channel Monitor</summary>
    <div class="settings-body">
      <div id="dbgOffMsg" style="font-size:.74rem;color:var(--muted);text-align:center;padding:6px 0;">
        Enable <strong style="color:var(--text)">DEBUG</strong> to start live data
      </div>
      <div class="dbg-grid" id="dbgGrid" style="display:none"></div>
      <div class="dbg-info" id="dbgInfo" style="display:none">
        <span>Mode: <span class="di-val" id="diMode">—</span></span>
        <span>Frame: <span class="di-val" id="diFrame">—</span> bytes</span>
        <span>Channels: <span class="di-val" id="diCh">—</span></span>
        <span>Updates: <span class="di-val" id="diRate">—</span></span>
      </div>
    </div>
  </details>
</div>

<script>
// =============================================================================
//  Constants & state
// =============================================================================
const SBUS_MIN = 172, SBUS_MAX = 1811, SBUS_CENTER = 992;

let ws;
let cfg = {
  rx:1, ry:2, ly:3, lx:4, sbus24:true,
  aMin: [172, 172, 172, 172],
  aMax: [1811,1811,1811,1811],
  sw:  Array.from({length:8},  (_,i)=>({l:`S${i}`, c:0, t:0, v:[172,992,1811], pos:1})),
  sl:  Array.from({length:4},  (_,i)=>({l:['LS','RS','S1','S2'][i]||`SL${i}`, c:0, pct:50})),
  tr:  Array.from({length:6},  (_,i)=>({l:`T${i+1}`, c:0, s:10, cur:992})),
  btn: Array.from({length:8},  (_,i)=>({l:`Btn${i+1}`, c:0, v:1811})),
  lua: Array.from({length:15}, (_,i)=>({l:`Button ${i+1}`, c:0, v:1811}))
};
let g_dbgOn = false;

// Local trim values (SBUS units) — mirrored from server on connect, then tracked locally
let trimCur = Array(6).fill(SBUS_CENTER);

// Joystick state
const sticks = { L:{x:0,y:0,active:false,touchId:null}, R:{x:0,y:0,active:false,touchId:null} };
let joyTimer = null;

// =============================================================================
//  WebSocket
// =============================================================================
function connect() {
  ws = new WebSocket(`ws://${location.hostname}/ws`);
  ws.onopen  = () => { setBadge(true); };
  ws.onclose = () => { setBadge(false); setTimeout(connect, 3000); };
  ws.onerror = () => {};
  ws.onmessage = (e) => {
    try {
      const msg = JSON.parse(e.data);
      if      (msg.e === 'cfg')            applyCfg(msg);
      else if (msg.e === 'chdata')         handleChData(msg);
      else if (msg.e === 'wifi_switching') handleWifiSwitching(msg);
    } catch(_) {}
  };
}
function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}
function setBadge(on) {
  const b = document.getElementById('connBadge');
  b.textContent = on ? 'CONNECTED' : 'DISCONNECTED';
  b.className   = on ? 'connected' : '';
}

// =============================================================================
//  Config
// =============================================================================
function applyCfg(msg) {
  cfg.rx     = msg.rx    || 1;
  cfg.ry     = msg.ry    || 2;
  cfg.ly     = msg.ly    || 3;
  cfg.lx     = msg.lx    || 4;
  cfg.sbus24 = (msg.sbus24 !== undefined) ? msg.sbus24 : true;
  g_dbgOn    = msg.dbg   || false;
  if (Array.isArray(msg.aMin)) cfg.aMin = msg.aMin;
  if (Array.isArray(msg.aMax)) cfg.aMax = msg.aMax;
  if (Array.isArray(msg.sw))  cfg.sw  = msg.sw;
  if (Array.isArray(msg.sl))  cfg.sl  = msg.sl;
  if (Array.isArray(msg.tr))  {
    cfg.tr  = msg.tr;
    trimCur = cfg.tr.map(t => t.cur !== undefined ? t.cur : SBUS_CENTER);
  }
  if (Array.isArray(msg.btn)) cfg.btn = msg.btn;
  if (Array.isArray(msg.lua)) {
    cfg.lua = msg.lua;
  } else if (!cfg.lua) {
    cfg.lua = Array.from({length:15}, (_,i)=>({l:`Button ${i+1}`,c:0,v:1811,k:'#4fc3f7'}));
  }
  // WiFi
  if (Array.isArray(msg.wifiNets)) cfg.wifiNets = msg.wifiNets;
  if (msg.wifiPref  != null) cfg.wifiPref  = msg.wifiPref;
  if (msg.wifiNet   != null) cfg.wifiNet   = msg.wifiNet;
  if (msg.wifiIP    != null) cfg.wifiIP    = msg.wifiIP;
  if (msg.wifiMDNS  != null) cfg.wifiMDNS  = msg.wifiMDNS;
  renderAll();
  renderWifiPanel();
}

function toggleMode() {
  cfg.sbus24 = !cfg.sbus24;
  send({t:'mode', sbus24:cfg.sbus24});
  updateModeBtn();
}
function toggleDebug() {
  g_dbgOn = !g_dbgOn;
  send({t:'dbg', on:g_dbgOn});
  updateDebugBtn();
}
function updateModeBtn() {
  const btn = document.getElementById('modeBtn');
  if (cfg.sbus24) { btn.textContent='SBUS-24'; btn.className='hdr-btn active-24'; }
  else            { btn.textContent='SBUS-16'; btn.className='hdr-btn active-16'; }
}
function updateDebugBtn() {
  const btn    = document.getElementById('debugBtn');
  const offMsg = document.getElementById('dbgOffMsg');
  if (g_dbgOn) {
    btn.textContent='DEBUG ON'; btn.className='hdr-btn dbg-on';
    if (offMsg) offMsg.style.display='none';
  } else {
    btn.textContent='DEBUG OFF'; btn.className='hdr-btn';
    if (offMsg) offMsg.style.display='';
    const grid=document.getElementById('dbgGrid');
    const info=document.getElementById('dbgInfo');
    if (grid) grid.style.display='none';
    if (info) info.style.display='none';
  }
}

// =============================================================================
//  Render
// =============================================================================
function axisToSbus(v) {
  return Math.round((v*0.5+0.5)*(SBUS_MAX-SBUS_MIN)+SBUS_MIN);
}
function axisToSbusRange(v, mn, mx) {
  return Math.max(SBUS_MIN, Math.min(SBUS_MAX, Math.round((v*0.5+0.5)*(mx-mn)+mn)));
}
function sbusFromPct(pct) {
  return Math.round(pct/100*(SBUS_MAX-SBUS_MIN)+SBUS_MIN);
}
function trimOffset(cur) {
  const d = cur - SBUS_CENTER;
  return (d >= 0 ? '+' : '') + d;
}
function buildChSel(val, includeNone) {
  const sel = document.createElement('select');
  if (includeNone) {
    const o=document.createElement('option'); o.value='0'; o.textContent='None';
    if (!val) o.selected=true; sel.appendChild(o);
  }
  for (let c=1;c<=24;c++) {
    const o=document.createElement('option'); o.value=String(c); o.textContent=`CH${c}`;
    if (c===val) o.selected=true; sel.appendChild(o);
  }
  return sel;
}

function renderAll() {
  updateModeBtn();
  updateDebugBtn();
  updateStickLabels();
  updateReadouts();
  renderSwitchCols();
  renderSliders();      // LS beside left stick, RS beside right stick
  renderPots();         // S1/S2 in pot row above sticks
  renderTrimBank();     // all 6 trims in a row below sticks
  renderButtons();      // physical S1-S6, RB1-RB2
  renderLuaButtons();   // 15 configurable virtual buttons
  renderSettings();
}

function updateStickLabels() {
  document.getElementById('lblL').textContent =
    `LEFT  LX:CH${cfg.lx}  LY:CH${cfg.ly}`;
  document.getElementById('lblR').textContent =
    `RIGHT  RX:CH${cfg.rx}  RY:CH${cfg.ry}`;
}
function updateReadouts() {
  // Use per-axis range so readout reflects actual SBUS output
  document.getElementById('readL').innerHTML =
    `LX:&nbsp;${axisToSbusRange(sticks.L.x,  cfg.aMin[3],cfg.aMax[3])}&nbsp;&nbsp;` +
    `LY:&nbsp;${axisToSbusRange(-sticks.L.y, cfg.aMin[2],cfg.aMax[2])}`;
  document.getElementById('readR').innerHTML =
    `RX:&nbsp;${axisToSbusRange(sticks.R.x,  cfg.aMin[0],cfg.aMax[0])}&nbsp;&nbsp;` +
    `RY:&nbsp;${axisToSbusRange(-sticks.R.y, cfg.aMin[1],cfg.aMax[1])}`;
}

// ── Switches ──────────────────────────────────────────────────────────────────
// Build a single switch card for index i in cfg.sw
function makeSwitchCard(i) {
  const sw   = cfg.sw[i];
  const card = document.createElement('div');
  card.className = 'sw-card';

  const nm = document.createElement('div'); nm.className='sw-name'; nm.textContent=sw.l;
  const ch = document.createElement('div'); ch.className='sw-ch';
  ch.textContent = sw.c ? `CH${sw.c}` : 'N/A';
  card.appendChild(nm); card.appendChild(ch);

  if (sw.t === 2) {
    // Momentary
    const btn = document.createElement('button');
    btn.className = 'sw-mom'; btn.textContent = sw.l;
    const press = () => { btn.classList.add('held');    send({t:'sw',i,p:1}); };
    const rel   = () => { btn.classList.remove('held'); send({t:'sw',i,p:0}); };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('mouseup',    rel);
    btn.addEventListener('mouseleave', rel);
    btn.addEventListener('touchstart', (e)=>{e.preventDefault();press();},{passive:false});
    btn.addEventListener('touchend',   (e)=>{e.preventDefault();rel();},  {passive:false});
    card.appendChild(btn);
  } else {
    const toggle  = document.createElement('div'); toggle.className='sw-toggle';
    const numPos  = (sw.t===1) ? 2 : 3;
    const labels  = numPos===2 ? ['▼','▲'] : ['▼','—','▲'];
    // Render high→mid→low top-to-bottom
    for (let p = numPos-1; p >= 0; p--) {
      const seg = document.createElement('button');
      seg.className = 'sw-seg' + (sw.pos===p ? ' sel' : '');
      seg.textContent = labels[p];
      seg.addEventListener('click', () => {
        sw.pos = p;
        toggle.querySelectorAll('.sw-seg').forEach((s,si) => {
          s.classList.toggle('sel', (numPos-1-si)===p);
        });
        send({t:'sw', i, p});
      });
      toggle.appendChild(seg);
    }
    card.appendChild(toggle);
  }
  return card;
}

// Build a switch pyramid:  topIdx single card, midIdx single card, then pair side-by-side
//   Left:  SF(5) top, SE(4) mid, SA(0)+SB(1) bottom pair
//   Right: SH(7) top, SG(6) mid, SC(2)+SD(3) bottom pair
function renderSwitchCols() {
  function buildPyramid(id, topIdx, midIdx, pairA, pairB) {
    const root = document.getElementById(id);
    root.innerHTML = '';
    root.appendChild(makeSwitchCard(topIdx));   // SF or SH
    root.appendChild(makeSwitchCard(midIdx));   // SE or SG
    const pair = document.createElement('div');
    pair.className = 'sw-pair';
    pair.appendChild(makeSwitchCard(pairA));    // SA or SC
    pair.appendChild(makeSwitchCard(pairB));    // SB or SD
    root.appendChild(pair);
  }
  buildPyramid('swPyramidLeft',  5, 4, 0, 1);  // SF SE | SA SB
  buildPyramid('swPyramidRight', 7, 6, 2, 3);  // SH SG | SC SD
}

// ── Trim + Slider columns ─────────────────────────────────────────────────────
// Left col: T5 (idx=4) + LS (idx=0)
// Right col: RS (idx=1) + T6 (idx=5)
function makeTrimWidget(trIdx) {
  const tr  = cfg.tr[trIdx];
  const div = document.createElement('div'); div.className='trim-widget';

  const lbl = document.createElement('div'); lbl.className='trim-lbl'; lbl.textContent=tr.l;
  const ch  = document.createElement('div'); ch.className='trim-ch';
  ch.textContent = tr.c ? `CH${tr.c}` : 'N/A';

  const valEl = document.createElement('div'); valEl.className='trim-val';
  valEl.id = `trv${trIdx}`; valEl.textContent = trimOffset(trimCur[trIdx]);

  const btnUp  = document.createElement('button'); btnUp.className='trim-btn'; btnUp.textContent='▲';
  const btnDn  = document.createElement('button'); btnDn.className='trim-btn'; btnDn.textContent='▼';
  const btnRst = document.createElement('button'); btnRst.className='trim-rst'; btnRst.textContent='RST';

  btnRst.onclick = () => {
    trimCur[trIdx] = SBUS_CENTER;
    valEl.textContent = trimOffset(trimCur[trIdx]);
    send({t:'tr', i:trIdx, d:0});
  };

  function applyTrimDelta(delta) {
    const step = tr.s || 10;
    trimCur[trIdx] = Math.max(SBUS_MIN, Math.min(SBUS_MAX, trimCur[trIdx] + delta*step));
    valEl.textContent = trimOffset(trimCur[trIdx]);
    send({t:'tr', i:trIdx, d:delta});
  }

  function makeRepeater(btn, delta) {
    let hold=null, rep=null;
    const start = () => {
      applyTrimDelta(delta);
      hold = setTimeout(()=>{ rep=setInterval(()=>applyTrimDelta(delta), 80); }, 400);
    };
    const stop = () => {
      if (hold) { clearTimeout(hold); hold=null; }
      if (rep)  { clearInterval(rep); rep=null;  }
    };
    btn.addEventListener('mousedown',  start);
    btn.addEventListener('mouseup',    stop);
    btn.addEventListener('mouseleave', stop);
    btn.addEventListener('touchstart', (e)=>{e.preventDefault();start();},{passive:false});
    btn.addEventListener('touchend',   (e)=>{e.preventDefault();stop(); },{passive:false});
  }
  makeRepeater(btnUp, +1);
  makeRepeater(btnDn, -1);

  div.appendChild(lbl); div.appendChild(ch); div.appendChild(btnUp);
  div.appendChild(valEl); div.appendChild(btnDn); div.appendChild(btnRst);
  return div;
}

function makeSliderWidget(slIdx) {
  const sl  = cfg.sl[slIdx];
  const div = document.createElement('div'); div.className='slider-widget';

  const lbl = document.createElement('div'); lbl.className='slider-lbl'; lbl.textContent=sl.l;
  const ch  = document.createElement('div'); ch.className='slider-ch';
  ch.textContent = sl.c ? `CH${sl.c}` : 'N/A';

  const wrap = document.createElement('div'); wrap.className='slider-wrap';
  const inp  = document.createElement('input');
  inp.type='range'; inp.className='slider-inp';
  inp.min='0'; inp.max='100'; inp.value=String(sl.pct||50);
  wrap.appendChild(inp);

  const valEl = document.createElement('div'); valEl.className='slider-val';
  valEl.id = `slv${slIdx}`;
  valEl.textContent = sbusFromPct(sl.pct||50);

  inp.oninput = () => {
    const pct = parseInt(inp.value);
    valEl.textContent = sbusFromPct(pct);
    send({t:'sl', i:slIdx, v:pct});
  };

  div.appendChild(lbl); div.appendChild(ch); div.appendChild(wrap); div.appendChild(valEl);
  return div;
}

// ── Sliders: LS left of left stick, RS right of right stick ──────────────────
function renderSliders() {
  // LS (index 0) beside left stick
  const lsCol = document.getElementById('slLSCol');
  lsCol.innerHTML = '';
  lsCol.appendChild(makeSliderWidget(0));

  // RS (index 1) beside right stick
  const rsCol = document.getElementById('slRSCol');
  rsCol.innerHTML = '';
  rsCol.appendChild(makeSliderWidget(1));
}

// ── S1/S2 pots: horizontal sliders in their own row ──────────────────────────
function renderPots() {
  const row = document.getElementById('potRow');
  row.innerHTML = '';
  // cfg.sl[2] = S1, cfg.sl[3] = S2
  for (let i = 2; i < Math.min(cfg.sl.length, 4); i++) {
    const sl  = cfg.sl[i];
    const div = document.createElement('div'); div.className='pot-widget';

    const lbl = document.createElement('div'); lbl.className='pot-lbl'; lbl.textContent=sl.l;
    const ch  = document.createElement('div'); ch.className='pot-ch';
    ch.textContent = sl.c ? `CH${sl.c}` : 'N/A';

    const inp = document.createElement('input');
    inp.type='range'; inp.className='pot-inp';
    inp.min='0'; inp.max='100'; inp.value=String(sl.pct||50);

    const valEl = document.createElement('div'); valEl.className='pot-val';
    valEl.id=`slv${i}`; valEl.textContent=sbusFromPct(sl.pct||50);

    const idx = i; // capture
    inp.oninput = () => {
      const pct = parseInt(inp.value);
      valEl.textContent = sbusFromPct(pct);
      send({t:'sl', i:idx, v:pct});
    };
    div.appendChild(lbl); div.appendChild(ch); div.appendChild(inp); div.appendChild(valEl);
    row.appendChild(div);
  }
}

// ── All 6 trims in a single row below the sticks ─────────────────────────────
// Order on X18 (left→right): T4, T5, T3, T6, T1, T2  (roughly)
// Simplified order by index: T1..T6 left-to-right; user can re-read as needed
function renderTrimBank() {
  const bank = document.getElementById('trimBank');
  bank.innerHTML = '';
  for (let i = 0; i < cfg.tr.length; i++) {
    bank.appendChild(makeTrimWidget(i));
  }
}

// ── Buttons S1-S6, RB1-RB2 ───────────────────────────────────────────────────
function renderButtons() {
  const bank = document.getElementById('btnBank');
  bank.innerHTML = '';

  const grpS = document.createElement('div'); grpS.className='btn-group';
  const grpR = document.createElement('div'); grpR.className='btn-group';
  const sep  = document.createElement('div'); sep.className='sep';

  cfg.btn.forEach((btn, i) => {
    const el = document.createElement('button');
    el.textContent = btn.l || `Btn${i+1}`;
    el.className   = 'ctrl-btn' + (btn.c===0 ? ' unassigned' : '');
    if (btn.c !== 0) {
      const press = () => { el.classList.add('pressed');   send({t:'btn',i,p:true}); };
      const rel   = () => { el.classList.remove('pressed'); send({t:'btn',i,p:false}); };
      el.addEventListener('mousedown',  press);
      el.addEventListener('mouseup',    rel);
      el.addEventListener('mouseleave', rel);
      el.addEventListener('touchstart', (e)=>{e.preventDefault();press();},{passive:false});
      el.addEventListener('touchend',   (e)=>{e.preventDefault();rel();},  {passive:false});
    }
    // S1-S6 → first 6; RB1/RB2 → last 2
    (i < 6 ? grpS : grpR).appendChild(el);
  });

  bank.appendChild(grpS);
  bank.appendChild(sep);
  bank.appendChild(grpR);
}

// ── Lua / virtual button grid (15 buttons) ───────────────────────────────────
function renderLuaButtons() {
  const grid = document.getElementById('luaGrid');
  grid.innerHTML = '';
  (cfg.lua || []).forEach((btn, i) => {
    const el = document.createElement('button');
    el.textContent = btn.l || `Button ${i+1}`;
    const assigned = btn.c !== 0;
    el.className   = 'lua-btn' + (assigned ? '' : ' unassigned');
    // Always apply color so it's visible even on unassigned buttons
    el.style.setProperty('--btn-color', btn.k && btn.k.length === 7 ? btn.k : '#4fc3f7');
    if (assigned) {
      const press = () => { el.classList.add('pressed');    send({t:'lua',i,p:true}); };
      const rel   = () => { el.classList.remove('pressed'); send({t:'lua',i,p:false}); };
      el.addEventListener('mousedown',  press);
      el.addEventListener('mouseup',    rel);
      el.addEventListener('mouseleave', rel);
      el.addEventListener('touchstart', (e)=>{e.preventDefault();press();},{passive:false});
      el.addEventListener('touchend',   (e)=>{e.preventDefault();rel();},  {passive:false});
    }
    grid.appendChild(el);
  });
}

// ── Settings panel ────────────────────────────────────────────────────────────
function renderSettings() {
  // Axis channel dropdowns
  [['selRX',cfg.rx],['selRY',cfg.ry],['selLY',cfg.ly],['selLX',cfg.lx]].forEach(([id,val])=>{
    const cont = document.getElementById(id).parentNode;
    const sel  = buildChSel(val, false); sel.id=id;
    cont.replaceChild(sel, document.getElementById(id));
  });
  // Axis min/max inputs
  [0,1,2,3].forEach(i=>{
    const mnEl=document.getElementById(`axMin${i}`);
    const mxEl=document.getElementById(`axMax${i}`);
    if(mnEl) mnEl.value=cfg.aMin[i]??172;
    if(mxEl) mxEl.value=cfg.aMax[i]??1811;
  });

  // Switch table
  const swTb = document.getElementById('swCfgBody'); swTb.innerHTML='';
  cfg.sw.forEach((sw, i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${sw.l}</td>`;
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(sw.c,true)); tr.appendChild(tdCh);
    for(let j=0;j<3;j++){
      const td=document.createElement('td');
      const inp=document.createElement('input'); inp.type='number';
      inp.min=SBUS_MIN; inp.max=SBUS_MAX; inp.value=sw.v[j]||SBUS_CENTER;
      td.appendChild(inp); tr.appendChild(td);
    }
    swTb.appendChild(tr);
  });

  // Slider/pot table (all 4: LS, RS, S1, S2)
  const slTb = document.getElementById('slCfgBody'); slTb.innerHTML='';
  cfg.sl.forEach((sl,i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${sl.l}</td>`;
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(sl.c,true)); tr.appendChild(tdCh);
    slTb.appendChild(tr);
  });

  // Trim table
  const trTb = document.getElementById('trCfgBody'); trTb.innerHTML='';
  cfg.tr.forEach((t,i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${t.l}</td>`;
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(t.c,false)); tr.appendChild(tdCh);
    const tdS=document.createElement('td');
    const si=document.createElement('input'); si.type='number'; si.min=1; si.max=100; si.value=t.s||10;
    si.style.width='60px'; tdS.appendChild(si); tr.appendChild(tdS);
    trTb.appendChild(tr);
  });

  // Physical button table
  const btnTb = document.getElementById('btnCfgBody'); btnTb.innerHTML='';
  cfg.btn.forEach((b,i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${b.l}</td>`;
    const tdL=document.createElement('td');
    const li=document.createElement('input'); li.type='text'; li.value=b.l; li.maxLength=31;
    tdL.appendChild(li); tr.appendChild(tdL);
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(b.c,true)); tr.appendChild(tdCh);
    const tdV=document.createElement('td');
    const vi=document.createElement('input'); vi.type='number'; vi.min=SBUS_MIN; vi.max=SBUS_MAX; vi.value=b.v;
    vi.style.width='70px'; tdV.appendChild(vi); tr.appendChild(tdV);
    btnTb.appendChild(tr);
  });

  // Lua button table
  const luaTb = document.getElementById('luaCfgBody'); luaTb.innerHTML='';
  (cfg.lua||[]).forEach((b,i)=>{
    const tr=document.createElement('tr');
    // # column
    const tdN=document.createElement('td'); tdN.textContent=i+1;
    tdN.style.cssText='font-size:.68rem;color:var(--muted);text-align:center;';
    tr.appendChild(tdN);
    // Label
    const tdL=document.createElement('td');
    const li=document.createElement('input'); li.type='text'; li.value=b.l; li.maxLength=31;
    tdL.appendChild(li); tr.appendChild(tdL);
    // Channel
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(b.c,true)); tr.appendChild(tdCh);
    // Value
    const tdV=document.createElement('td');
    const vi=document.createElement('input'); vi.type='number'; vi.min=SBUS_MIN; vi.max=SBUS_MAX; vi.value=b.v||SBUS_MAX;
    vi.style.width='70px'; tdV.appendChild(vi); tr.appendChild(tdV);
    // Color
    const tdK=document.createElement('td'); tdK.style.textAlign='center';
    const ki=document.createElement('input'); ki.type='color'; ki.value=b.k||'#4fc3f7';
    ki.style.cssText='width:40px;height:28px;padding:2px;border:none;border-radius:4px;cursor:pointer;background:none;';
    // Live-preview: update the button color in the grid as the user picks
    ki.addEventListener('input', () => {
      const btns = document.querySelectorAll('#luaGrid .lua-btn');
      if (btns[i]) btns[i].style.setProperty('--btn-color', ki.value);
    });
    tdK.appendChild(ki); tr.appendChild(tdK);
    luaTb.appendChild(tr);
  });
}

// =============================================================================
//  WiFi panel
// =============================================================================

function renderWifiPanel() {
  // Status bar
  const nets = cfg.wifiNets || [];
  const ni   = cfg.wifiNet != null ? cfg.wifiNet : -1;
  document.getElementById('wifiStatusSSID').textContent =
    ni >= 0 && nets[ni] ? nets[ni].s : (ni === -1 ? 'AP (' + (cfg.wifiIP||'') + ')' : '—');
  document.getElementById('wifiStatusIP').textContent   = cfg.wifiIP   || '—';
  document.getElementById('wifiStatusMDNS').textContent = cfg.wifiMDNS || '—';

  // Network rows
  const list = document.getElementById('wifiNetList');
  list.innerHTML = '';
  nets.forEach((net, i) => {
    const row = document.createElement('div');
    row.className = 'wifi-net-row';
    row.dataset.idx = i;

    const num  = document.createElement('span'); num.className='drag-num'; num.textContent=i+1+'.';
    const ssid = document.createElement('input'); ssid.type='text'; ssid.placeholder='SSID';
    ssid.value = net.s || ''; ssid.maxLength = 32;

    const passWrap = document.createElement('div'); passWrap.className='wifi-pass-wrap';
    const pass = document.createElement('input'); pass.type='password'; pass.placeholder='Password';
    pass.value = net.p || ''; pass.maxLength = 64;
    const eye  = document.createElement('button'); eye.textContent='👁'; eye.title='Show/hide';
    eye.onclick = () => { pass.type = pass.type==='password' ? 'text' : 'password'; };
    passWrap.appendChild(pass); passWrap.appendChild(eye);

    const up  = document.createElement('button'); up.className='wifi-arrow';  up.textContent='▲';
    const dn  = document.createElement('button'); dn.className='wifi-arrow';  dn.textContent='▼';
    const del = document.createElement('button'); del.className='wifi-del';   del.textContent='✕';
    up.onclick  = () => { if(i>0){const t=nets[i];nets[i]=nets[i-1];nets[i-1]=t;renderWifiPanel();} };
    dn.onclick  = () => { if(i<nets.length-1){const t=nets[i];nets[i]=nets[i+1];nets[i+1]=t;renderWifiPanel();} };
    del.onclick = () => { nets.splice(i,1); renderWifiPanel(); };

    row.appendChild(num); row.appendChild(ssid); row.appendChild(passWrap);
    row.appendChild(up);  row.appendChild(dn);   row.appendChild(del);
    list.appendChild(row);
  });

  // Preference dropdown
  const sel = document.getElementById('wifiPrefSel');
  sel.innerHTML = '<option value="0">Auto (try in order)</option>';
  nets.forEach((n, i) => {
    const o = document.createElement('option');
    o.value = i + 1;
    o.textContent = `Net ${i+1}: ${n.s || '(blank)'}`;
    sel.appendChild(o);
  });
  const apOpt = document.createElement('option'); apOpt.value=255; apOpt.textContent='AP Only';
  sel.appendChild(apOpt);
  sel.value = cfg.wifiPref != null ? cfg.wifiPref : 0;
}

function wifiAddRow() {
  if (!cfg.wifiNets) cfg.wifiNets = [];
  if (cfg.wifiNets.length >= 4) return;
  cfg.wifiNets.push({s:'', p:''});
  renderWifiPanel();
}

// Read current row values back into cfg.wifiNets before sending
function wifiCollect() {
  const rows = document.getElementById('wifiNetList').children;
  cfg.wifiNets = [];
  for (const row of rows) {
    const inputs = row.querySelectorAll('input');
    const s = inputs[0].value.trim();
    const p = inputs[1].value;
    if (s) cfg.wifiNets.push({s, p});
  }
}

function wifiApply() {
  wifiCollect();
  cfg.wifiPref = parseInt(document.getElementById('wifiPrefSel').value) || 0;
  send({t:'wificfg', pref: cfg.wifiPref, nets: cfg.wifiNets});
  const msg = document.getElementById('wifiMsg');
  msg.style.display = 'inline';
  msg.textContent = 'Connecting…';
}

function handleWifiSwitching(msg) {
  const el = document.getElementById('wifiMsg');
  el.style.display = 'inline';
  el.textContent = `Switching to "${msg.ssid}"… reconnect at http://${msg.mdns}`;
}

// =============================================================================
//  Export / Import
// =============================================================================

// ── Military DTG: DDHHMMZMONYR  e.g. 091234ZAPR26 ───────────────────────────
function getDTG() {
  const now = new Date();
  const mo  = ['JAN','FEB','MAR','APR','MAY','JUN','JUL','AUG','SEP','OCT','NOV','DEC'];
  const dd  = String(now.getUTCDate()).padStart(2,'0');
  const hh  = String(now.getUTCHours()).padStart(2,'0');
  const mm  = String(now.getUTCMinutes()).padStart(2,'0');
  const yy  = String(now.getUTCFullYear()).slice(-2);
  return `${dd}${hh}${mm}Z${mo[now.getUTCMonth()]}${yy}`;
}

// ── Export: fetch the saved JSON from the ESP and trigger a browser download ──
function exportConfig() {
  fetch('/cfg')
    .then(r => { if (!r.ok) throw new Error('No config'); return r.blob(); })
    .then(blob => {
      const url = URL.createObjectURL(blob);
      const a   = document.createElement('a');
      a.href = url;
      a.download = `sbus_config_${getDTG()}.json`;
      a.click();
      URL.revokeObjectURL(url);
    })
    .catch(e => {
      const el = document.getElementById('importStatus');
      el.style.color = 'var(--red)';
      el.textContent = 'Export failed: ' + e.message;
    });
}

// ── Import: read a JSON file and send it as a cfg WS message ─────────────────
function importConfig(input) {
  const file = input.files[0];
  if (!file) return;
  const status = document.getElementById('importStatus');
  status.style.color = 'var(--muted)';
  status.textContent = 'Reading…';
  const reader = new FileReader();
  reader.onload = e => {
    try {
      const data = JSON.parse(e.target.result);
      // Remap LittleFS format to WS cfg message format
      const payload = { t: 'cfg' };
      if (data.rx    != null) payload.rx = data.rx;
      if (data.ry    != null) payload.ry = data.ry;
      if (data.ly    != null) payload.ly = data.ly;
      if (data.lx    != null) payload.lx = data.lx;
      if (data.aMin  != null) payload.aMin = data.aMin;
      if (data.aMax  != null) payload.aMax = data.aMax;
      if (data.sw    != null) payload.sw   = data.sw;
      if (data.sl    != null) payload.sl   = data.sl;
      if (data.tr    != null) payload.tr   = data.tr;
      if (data.btn   != null) payload.btn  = data.btn;
      if (data.lua   != null) payload.lua  = data.lua;
      send(payload);
      status.style.color = 'var(--green)';
      status.textContent = '\u2713 Imported — config applied & saved.';
    } catch(err) {
      status.style.color = 'var(--red)';
      status.textContent = 'Import failed: ' + err.message;
    }
    input.value = '';   // reset so the same file can be re-imported if needed
  };
  reader.readAsText(file);
}

function saveConfig() {
  const payload = {t:'cfg'};
  payload.rx = parseInt(document.getElementById('selRX').value);
  payload.ry = parseInt(document.getElementById('selRY').value);
  payload.ly = parseInt(document.getElementById('selLY').value);
  payload.lx = parseInt(document.getElementById('selLX').value);
  payload.aMin = [0,1,2,3].map(i => parseInt(document.getElementById(`axMin${i}`).value)||172);
  payload.aMax = [0,1,2,3].map(i => parseInt(document.getElementById(`axMax${i}`).value)||1811);

  const swRows = document.getElementById('swCfgBody').rows;
  payload.sw = cfg.sw.map((sw, i) => {
    const row = swRows[i];
    const ch = parseInt(row.cells[1].querySelector('select').value);
    const v = [0,1,2].map(j=>parseInt(row.cells[2+j].querySelector('input').value));
    return {l:sw.l, c:ch, t:sw.t, v};
  });

  const slRows = document.getElementById('slCfgBody').rows;
  payload.sl = cfg.sl.map((sl, i) => ({
    l: sl.l,
    c: parseInt(slRows[i].cells[1].querySelector('select').value)
  }));

  const trRows = document.getElementById('trCfgBody').rows;
  payload.tr = cfg.tr.map((t, i) => ({
    l: t.l,
    c: parseInt(trRows[i].cells[1].querySelector('select').value),
    s: parseInt(trRows[i].cells[2].querySelector('input').value)
  }));

  const btnRows = document.getElementById('btnCfgBody').rows;
  payload.btn = cfg.btn.map((b, i) => ({
    l: btnRows[i].cells[1].querySelector('input').value,
    c: parseInt(btnRows[i].cells[2].querySelector('select').value),
    v: parseInt(btnRows[i].cells[3].querySelector('input').value)
  }));

  const luaRows = document.getElementById('luaCfgBody').rows;
  payload.lua = (cfg.lua||[]).map((b, i) => ({
    l: luaRows[i].cells[1].querySelector('input').value,
    c: parseInt(luaRows[i].cells[2].querySelector('select').value),
    v: parseInt(luaRows[i].cells[3].querySelector('input').value),
    k: luaRows[i].cells[4].querySelector('input[type=color]').value
  }));

  send(payload);
}

// =============================================================================
//  Debug channel grid
// =============================================================================
let dbgCells=[], dbgLitTimers=[], dbgUpdatesInWindow=0, dbgRateTimer=null;

function initDbgGrid(n) {
  const grid=document.getElementById('dbgGrid'); if(!grid)return;
  grid.innerHTML=''; dbgCells=[]; dbgLitTimers=new Array(n).fill(null);
  for(let i=0;i<n;i++){
    const cell=document.createElement('div'); cell.className='dbg-cell';
    const cn=document.createElement('div'); cn.className='dcn'; cn.textContent=`CH${String(i+1).padStart(2,'0')}`;
    const cv=document.createElement('span'); cv.className='dcv'; cv.textContent='—';
    cell.appendChild(cn); cell.appendChild(cv); grid.appendChild(cell);
    dbgCells.push({cell,cv});
  }
}

function handleChData(msg) {
  const ch=msg.ch, mode=msg.mode, fl=msg.fl;
  if(!Array.isArray(ch)) return;
  const offMsg=document.getElementById('dbgOffMsg');
  const grid=document.getElementById('dbgGrid');
  const info=document.getElementById('dbgInfo');
  if(offMsg) offMsg.style.display='none';
  if(grid)   grid.style.display='';
  if(info)   info.style.display='';
  if(dbgCells.length!==ch.length) initDbgGrid(ch.length);
  ch.forEach((val,i)=>{
    const e=dbgCells[i]; if(!e) return;
    const prev=parseInt(e.cv.textContent)||-1;
    e.cv.textContent=val;
    if(val!==prev){
      e.cell.classList.add('lit');
      if(dbgLitTimers[i]) clearTimeout(dbgLitTimers[i]);
      dbgLitTimers[i]=setTimeout(()=>{e.cell.classList.remove('lit');dbgLitTimers[i]=null;},300);
    }
  });
  const mEl=document.getElementById('diMode'); if(mEl) mEl.textContent=`SBUS-${mode}`;
  const fEl=document.getElementById('diFrame'); if(fEl) fEl.textContent=fl;
  const cEl=document.getElementById('diCh');   if(cEl) cEl.textContent=ch.length;
  dbgUpdatesInWindow++;
  if(!dbgRateTimer){
    dbgRateTimer=setInterval(()=>{
      const rEl=document.getElementById('diRate'); if(rEl) rEl.textContent=`${dbgUpdatesInWindow}/s`;
      dbgUpdatesInWindow=0;
    },1000);
  }
}

// =============================================================================
//  Joystick
// =============================================================================
function scheduleJoySend() {
  if(joyTimer) return;
  joyTimer=setTimeout(()=>{
    joyTimer=null;
    send({t:'a', lx:sticks.L.x, ly:sticks.L.y, rx:sticks.R.x, ry:sticks.R.y});
    updateReadouts();
  },30);
}
function posFromXY(cx,cy,wrap){
  const r=wrap.getBoundingClientRect();
  let nx=(cx-r.left-r.width/2)/(r.width/2);
  let ny=(cy-r.top-r.height/2)/(r.height/2);
  const mag=Math.sqrt(nx*nx+ny*ny); if(mag>1){nx/=mag;ny/=mag;}
  return[+nx.toFixed(3),+ny.toFixed(3)];
}
function applyStick(side,nx,ny){
  sticks[side].x=nx; sticks[side].y=ny;
  const th=document.getElementById('thumb'+side);
  th.style.left=(50+nx*44)+'%'; th.style.top=(50+ny*44)+'%';
  scheduleJoySend();
}
function springBack(side){ applyStick(side,0,0); }

function initStick(side){
  const wrap=document.getElementById('stick'+side);
  wrap.addEventListener('touchstart',(e)=>{
    e.preventDefault(); const t=e.changedTouches[0];
    sticks[side].touchId=t.identifier; sticks[side].active=true;
    wrap.classList.add('active');
    const[nx,ny]=posFromXY(t.clientX,t.clientY,wrap); applyStick(side,nx,ny);
  },{passive:false});
  wrap.addEventListener('touchmove',(e)=>{
    e.preventDefault();
    for(const t of e.changedTouches){
      if(t.identifier===sticks[side].touchId){
        const[nx,ny]=posFromXY(t.clientX,t.clientY,wrap); applyStick(side,nx,ny);
      }
    }
  },{passive:false});
  wrap.addEventListener('touchend',(e)=>{
    e.preventDefault();
    for(const t of e.changedTouches){
      if(t.identifier===sticks[side].touchId){
        sticks[side].active=false; sticks[side].touchId=null;
        wrap.classList.remove('active'); springBack(side);
      }
    }
  },{passive:false});
  wrap.addEventListener('mousedown',(e)=>{
    sticks[side].active=true; wrap.classList.add('active');
    const[nx,ny]=posFromXY(e.clientX,e.clientY,wrap); applyStick(side,nx,ny);
    const onMove=(ev)=>{ if(!sticks[side].active)return; const[nx2,ny2]=posFromXY(ev.clientX,ev.clientY,wrap); applyStick(side,nx2,ny2); };
    const onUp=()=>{ sticks[side].active=false; wrap.classList.remove('active'); springBack(side); document.removeEventListener('mousemove',onMove); document.removeEventListener('mouseup',onUp); };
    document.addEventListener('mousemove',onMove); document.addEventListener('mouseup',onUp);
  });
}

// =============================================================================
//  Serial command hint (displayed on Serial Monitor)
// =============================================================================

// =============================================================================
//  Boot
// =============================================================================
initStick('L');
initStick('R');
renderAll();
renderWifiPanel();
connect();
</script>
</body>
</html>)rawhtml";

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
  Serial.begin(115200);
  delay(400);
  Serial.println("\n[SBUS] SBUSController booting (X18 edition)...");

  // SBUS output: 100 kbaud, 8E2, inverted
  Serial1.begin(SBUS_BAUD, SERIAL_8E2, -1, SBUS_TX_PIN, true);
  Serial.printf("[SBUS] Serial1 TX on GPIO%d  (100kbaud 8E2 inverted)\n", SBUS_TX_PIN);

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
    Serial.printf("[SBUS] HTTP GET /  from %s\n", req->client()->remoteIP().toString().c_str());
    req->send(200, "text/html", HTML);  // send() preferred over send_P() in ESP32Async v3.x
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
    Serial.printf("[SBUS] 404: %s\n", req->url().c_str());
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
      g_sbusDebug = !g_sbusDebug;
      Serial.printf("[SBUS] Debug %s\n", g_sbusDebug ? "ENABLED" : "DISABLED");
      ws.textAll(buildCfgJson());
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
      Serial.printf("  |  Debug: %s", g_sbusDebug ? "ON" : "OFF");
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
