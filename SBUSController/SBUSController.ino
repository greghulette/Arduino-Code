// =============================================================================
//  SBUSController.ino  —  Browser-based Virtual SBUS Controller
//  Target: ESP32 (original)
//  Board library: esp32 by Espressif  *** PIN TO 3.3.4 — avoid 3.3.5 ***
//
//  Required libraries (install via Library Manager):
//    • ESPAsyncWebServer  (me-no-dev)
//    • AsyncTCP           (me-no-dev)
//    • ArduinoJson        (Benoit Blanchon)  v6.x
//
//  ─── Overview ───────────────────────────────────────────────────────────────
//   Browser joysticks + buttons → WebSocket → ESP32 → SBUS-24 stream → Kyber
//   No passthrough, no capture/replay — pure virtual controller output.
//   SBUS-24: 36-byte frame  (0x0F + 33 data bytes + flags + 0x00)
//            24 channels × 11 bits = 264 bits packed LSB-first.
//
//  ─── Hardware Wiring ────────────────────────────────────────────────────────
//   Kyber SBUS input  ←  GPIO 17  (Serial1 TX, inverted 100 kbaud 8E2)
//   USB               ↔  Serial   (debug @ 115200)
//
//  ─── WiFi ───────────────────────────────────────────────────────────────────
//   Cascading: tries RHN-COMM → HelloEverybody → AP fallback (SBUSCtrl)
//   5-second timeout per network attempt.
//   In AP mode browse to  http://192.168.4.1
//   In STA mode the assigned IP is printed to Serial on boot.
//
//  ─── Config (/config.json in LittleFS) ──────────────────────────────────────
//   {"lx":1,"ly":2,"rx":3,"ry":4,"b":[{"l":"Button 1","c":5,"v":1811},...]}
//   lx/ly/rx/ry : 1-based SBUS channel index for each joystick axis  (1–24)
//   b           : array of 15 buttons; l=label, c=channel (0=unassigned, 1–24), v=pressed value
//
// =============================================================================

#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ─── WiFi ─────────────────────────────────────────────────────────────────────
// Cascading connection order:
//   1. RHN-COMM  (primary home/shop network)
//   2. HelloEverybody  (fallback network)
//   3. AP mode  (last resort — always works)

#define WIFI_PRIMARY_SSID   "RHN-COMM"
#define WIFI_PRIMARY_PASS   "0o9i8u7y)O(I*U&Y"
#define WIFI_FALLBACK_SSID  "HelloEverybody"
#define WIFI_FALLBACK_PASS  "thedeskisbrown"
#define WIFI_AP_SSID        "SBUSCtrl"
#define WIFI_AP_PASS        "sbus1234"
#define WIFI_STA_TIMEOUT_MS 5000   // ms to wait per network attempt

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
#define SBUS_FRAME_MS       9     // transmit a frame every 9 ms (~111 Hz, FrSky standard)

// SBUS-16  —  standard, well-documented, 25-byte frame
#define SBUS_CH_COUNT_16    16
#define SBUS_FRAME_LEN_16   25    // 0x0F + 22 data + flags + 0x00

// SBUS-24  —  FrSky ACCESS extended, 36-byte frame
#define SBUS_CH_COUNT_24    24
#define SBUS_FRAME_LEN_24   36    // 0x0F + 33 data (264 bits) + flags + 0x00

// ── Runtime mode selection ────────────────────────────────────────────────────
// Changed via web UI toggle; persisted in config.json.
// false = SBUS-16 (25 bytes, 16 channels)
// true  = SBUS-24 (36 bytes, 24 channels)
bool g_sbus24 = true;

inline int sbusChCount()  { return g_sbus24 ? SBUS_CH_COUNT_24 : SBUS_CH_COUNT_16; }
inline int sbusFrameLen() { return g_sbus24 ? SBUS_FRAME_LEN_24 : SBUS_FRAME_LEN_16; }

// ─── SBUS Debug ───────────────────────────────────────────────────────────────
// Uncomment SBUS_DEBUG to compile in the debug capability.
// Once compiled in, toggle at runtime via:
//   • Web UI  — Debug button in the header (green = on, muted = off)
//   • Serial  — type  'd'  in Serial Monitor and press Enter
//
// SBUS_DEBUG_INTERVAL_MS : how often to print (ms).  0 = every frame (spammy).
//
#define SBUS_DEBUG
#define SBUS_DEBUG_INTERVAL_MS  50    // serial dump rate (ms) — 50ms = 20 Hz

#ifdef SBUS_DEBUG
bool g_sbusDebug = false;  // runtime on/off; toggled via web or serial
#endif

// ─── Config ───────────────────────────────────────────────────────────────────

#define CONFIG_FILE     "/config.json"
#define MAX_BUTTONS     15

struct BtnCfg {
  char     label[32];
  uint8_t  ch;     // 1-based SBUS channel; 0 = unassigned
  uint16_t val;    // value to send when pressed (172–1811)
};

struct Config {
  uint8_t joyLX;           // 1-based channel
  uint8_t joyLY;
  uint8_t joyRX;
  uint8_t joyRY;
  BtnCfg  buttons[MAX_BUTTONS];
  bool    sbus24;           // true = SBUS-24 (36-byte), false = SBUS-16 (25-byte)
};

Config cfg;

// ─── Runtime State ────────────────────────────────────────────────────────────

uint16_t sbusChannels[SBUS_CH_COUNT_24];  // always sized for max; only [0..sbusChCount()-1] are sent
uint32_t lastFrameMs = 0;

// ─── Web server / WebSocket ───────────────────────────────────────────────────

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =============================================================================
//  CONFIG  —  load / save / defaults
// =============================================================================

void applyConfigDefaults() {
  cfg.joyLX  = 1;
  cfg.joyLY  = 2;
  cfg.joyRX  = 3;
  cfg.joyRY  = 4;
  cfg.sbus24 = true;   // default to SBUS-24; change to false to start on SBUS-16
  for (int i = 0; i < MAX_BUTTONS; i++) {
    snprintf(cfg.buttons[i].label, sizeof(cfg.buttons[i].label), "Button %d", i + 1);
    cfg.buttons[i].ch  = 0;
    cfg.buttons[i].val = SBUS_MAX;
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

  StaticJsonDocument<4096> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.println("[SBUS] Config parse error — using defaults."); return; }

  cfg.joyLX  = doc["lx"]     | 1;
  cfg.joyLY  = doc["ly"]     | 2;
  cfg.joyRX  = doc["rx"]     | 3;
  cfg.joyRY  = doc["ry"]     | 4;
  cfg.sbus24 = doc["sbus24"] | true;
  g_sbus24   = cfg.sbus24;

  // Clamp axis channels to 1–24 (max possible; unused in SBUS-16 mode)
  auto clampCh = [](uint8_t v) -> uint8_t { return constrain(v, 1, SBUS_CH_COUNT_24); };
  cfg.joyLX = clampCh(cfg.joyLX);
  cfg.joyLY = clampCh(cfg.joyLY);
  cfg.joyRX = clampCh(cfg.joyRX);
  cfg.joyRY = clampCh(cfg.joyRY);

  JsonArray arr = doc["b"].as<JsonArray>();
  int i = 0;
  for (JsonObject o : arr) {
    if (i >= MAX_BUTTONS) break;
    strlcpy(cfg.buttons[i].label, o["l"] | "", sizeof(cfg.buttons[i].label));
    cfg.buttons[i].ch  = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
    cfg.buttons[i].val = constrain((int)(o["v"] | SBUS_MAX), SBUS_MIN, SBUS_MAX);
    i++;
  }
  Serial.println("[SBUS] Config loaded.");
}

void saveConfig() {
  File f = LittleFS.open(CONFIG_FILE, FILE_WRITE);
  if (!f) { Serial.println("[SBUS] Cannot write config."); return; }

  StaticJsonDocument<4096> doc;
  doc["lx"]     = cfg.joyLX;
  doc["ly"]     = cfg.joyLY;
  doc["rx"]     = cfg.joyRX;
  doc["ry"]     = cfg.joyRY;
  doc["sbus24"] = cfg.sbus24;

  JsonArray arr = doc.createNestedArray("b");
  for (int i = 0; i < MAX_BUTTONS; i++) {
    JsonObject o = arr.createNestedObject();
    o["l"] = cfg.buttons[i].label;
    o["c"] = cfg.buttons[i].ch;
    o["v"] = cfg.buttons[i].val;
  }
  serializeJson(doc, f);
  f.close();
  Serial.println("[SBUS] Config saved.");
}

// Build the cfg event JSON string to send to browser clients
String buildCfgJson() {
  StaticJsonDocument<4096> doc;
  doc["e"]      = "cfg";
  doc["lx"]     = cfg.joyLX;
  doc["ly"]     = cfg.joyLY;
  doc["rx"]     = cfg.joyRX;
  doc["ry"]     = cfg.joyRY;
  doc["sbus24"] = cfg.sbus24;
#ifdef SBUS_DEBUG
  doc["dbg"]    = g_sbusDebug;
#else
  doc["dbg"]    = false;
#endif

  JsonArray arr = doc.createNestedArray("b");
  for (int i = 0; i < MAX_BUTTONS; i++) {
    JsonObject o = arr.createNestedObject();
    o["l"] = cfg.buttons[i].label;
    o["c"] = cfg.buttons[i].ch;
    o["v"] = cfg.buttons[i].val;
  }
  String out;
  serializeJson(doc, out);
  return out;
}

// =============================================================================
//  SBUS  —  encode and transmit
// =============================================================================

// Pack N × 11-bit channel values into an SBUS frame.
//
// SBUS-16 (25 bytes):  0x0F + 22 data bytes (176 bits) + flags + 0x00
// SBUS-24 (36 bytes):  0x0F + 33 data bytes (264 bits) + flags + 0x00
//
// Bit packing: channel i occupies bits [i*11 .. i*11+10], LSB-first.
// Three bytes may be touched per channel; the third only when bit offset > 5.
//
// caller must supply a buffer of at least SBUS_FRAME_LEN_24 (36) bytes.
//
void buildSbusFrame(uint8_t* frame) {
  const int flen  = sbusFrameLen();
  const int chcnt = sbusChCount();

  memset(frame, 0, flen);
  frame[0]        = SBUS_HEADER;
  frame[flen - 2] = SBUS_FLAGS;
  frame[flen - 1] = SBUS_FOOTER;

  for (int i = 0; i < chcnt; i++) {
    uint16_t val = constrain(sbusChannels[i], SBUS_MIN, SBUS_MAX);
    int b = i * 11;                    // bit offset within payload

    frame[1 + b / 8]     |= (uint8_t)((val << (b % 8)) & 0xFF);
    frame[1 + b / 8 + 1] |= (uint8_t)((val >> (8 - b % 8)) & 0xFF);
    if ((b % 8) > 5) {
      frame[1 + b / 8 + 2] |= (uint8_t)((val >> (16 - b % 8)) & 0xFF);
    }
  }
}

// =============================================================================
//  SBUS DEBUG DUMP
// =============================================================================

#ifdef SBUS_DEBUG
static uint32_t s_lastSerialDbgMs = 0;
static uint32_t s_lastWsDbgMs     = 0;
static uint16_t s_prevCh[SBUS_CH_COUNT_24];
static bool     s_prevChInit = false;

// ── Decode all channels from a packed SBUS frame into out[] ──────────────────
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

// ── Serial: raw hex frame every print + delta channels on a second line ───────
//
// Output example (SBUS-24, joystick moved):
//   SBUS-24 [36B]  hdr=0F | E0 07 3E 78 ... | fl=00 end=00
//   Δ CH01: 750  CH02:1200
//
// When nothing changes the Δ line is omitted — the hex line alone confirms
// the frame is being sent at the right rate and format.
//
static void printSbusDebug(const uint8_t* frame, int flen, int chcnt) {
  if (!g_sbusDebug) return;
  const uint32_t now = millis();
  if (now - s_lastSerialDbgMs < (uint32_t)SBUS_DEBUG_INTERVAL_MS) return;
  s_lastSerialDbgMs = now;

  uint16_t cur[SBUS_CH_COUNT_24];
  decodeSbusFrame(frame, cur, chcnt);

  // ── Line 1: raw hex with annotated header / flags / footer ───────────────
  Serial.printf("SBUS-%d [%dB]  hdr=%02X |", chcnt, flen, frame[0]);
  for (int i = 1; i < flen - 2; i++) Serial.printf(" %02X", frame[i]);
  Serial.printf(" | fl=%02X end=%02X\n", frame[flen - 2], frame[flen - 1]);

  // ── Line 2: only channels that changed since last print ───────────────────
  if (!s_prevChInit) {
    memset(s_prevCh, 0xFF, sizeof(s_prevCh));   // force everything to print first time
    s_prevChInit = true;
  }

  bool anyChange = false;
  for (int i = 0; i < chcnt; i++) if (cur[i] != s_prevCh[i]) { anyChange = true; break; }

  if (anyChange) {
    Serial.print(F("  \xce\x94"));               // Δ in UTF-8
    for (int i = 0; i < chcnt; i++) {
      if (cur[i] != s_prevCh[i]) Serial.printf("  CH%02d:%4u", i + 1, cur[i]);
    }
    Serial.println();
    memcpy(s_prevCh, cur, sizeof(uint16_t) * chcnt);
  }
}

// ── WebSocket: push channel array to browser at ~10 Hz ───────────────────────
// Message: {"e":"chdata","ch":[v0,v1,...vN],"mode":24,"fl":36}
static void sendWsDebug(const uint8_t* frame, int flen, int chcnt) {
  if (!g_sbusDebug) return;
  const uint32_t now = millis();
  if (now - s_lastWsDbgMs < 100) return;   // 10 Hz
  s_lastWsDbgMs = now;

  uint16_t cur[SBUS_CH_COUNT_24];
  decodeSbusFrame(frame, cur, chcnt);

  // Build JSON manually to avoid heap allocation of StaticJsonDocument
  String msg;
  msg.reserve(64 + chcnt * 5);
  msg  = "{\"e\":\"chdata\",\"mode\":";
  msg += chcnt;
  msg += ",\"fl\":";
  msg += flen;
  msg += ",\"ch\":[";
  for (int i = 0; i < chcnt; i++) {
    msg += cur[i];
    if (i < chcnt - 1) msg += ',';
  }
  msg += "]}";
  ws.textAll(msg);
}
#endif  // SBUS_DEBUG

void transmitSbus() {
  uint8_t frame[SBUS_FRAME_LEN_24];  // always allocate max; only flen bytes are sent
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

// Map a float -1.0..+1.0 joystick value to an SBUS channel value 172..1811.
inline uint16_t axisToSbus(float v) {
  // constrain first so out-of-range floats don't wrap
  v = constrain(v, -1.0f, 1.0f);
  return (uint16_t)((v * 0.5f + 0.5f) * (float)(SBUS_MAX - SBUS_MIN) + SBUS_MIN + 0.5f);
}

// =============================================================================
//  WEBSOCKET HANDLER
// =============================================================================

void handleWsMessage(AsyncWebSocketClient* client, uint8_t* data, size_t len) {
  data[len] = 0;  // null-terminate for JSON parser

  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, (char*)data)) return;

  const char* t = doc["t"] | "";

  // ── Axis update ──────────────────────────────────────────────────────────
  // { "t":"a", "lx":f, "ly":f, "rx":f, "ry":f }   values -1.0 to +1.0
  if (!strcmp(t, "a")) {
    float lx = doc["lx"] | 0.0f;
    float ly = doc["ly"] | 0.0f;
    float rx = doc["rx"] | 0.0f;
    float ry = doc["ry"] | 0.0f;

    // Y axes are inverted: screen-up (negative float) → high SBUS value
    sbusChannels[cfg.joyLX - 1] = axisToSbus(lx);
    sbusChannels[cfg.joyLY - 1] = axisToSbus(-ly);
    sbusChannels[cfg.joyRX - 1] = axisToSbus(rx);
    sbusChannels[cfg.joyRY - 1] = axisToSbus(-ry);
  }

  // ── Button press / release ────────────────────────────────────────────────
  // { "t":"b", "i":index, "p":true/false }
  else if (!strcmp(t, "b")) {
    int  idx     = doc["i"] | -1;
    bool pressed = doc["p"] | false;
    if (idx >= 0 && idx < MAX_BUTTONS) {
      BtnCfg& btn = cfg.buttons[idx];
      if (btn.ch >= 1 && btn.ch <= SBUS_CH_COUNT_24) {
        sbusChannels[btn.ch - 1] = pressed ? btn.val : SBUS_CENTER;
      }
    }
  }

  // ── Config save ───────────────────────────────────────────────────────────
  // { "t":"cfg", "lx":1, "ly":2, "rx":3, "ry":4, "b":[...] }
  else if (!strcmp(t, "cfg")) {
    cfg.joyLX = constrain((int)(doc["lx"] | 1), 1, SBUS_CH_COUNT_24);
    cfg.joyLY = constrain((int)(doc["ly"] | 2), 1, SBUS_CH_COUNT_24);
    cfg.joyRX = constrain((int)(doc["rx"] | 3), 1, SBUS_CH_COUNT_24);
    cfg.joyRY = constrain((int)(doc["ry"] | 4), 1, SBUS_CH_COUNT_24);

    JsonArray arr = doc["b"].as<JsonArray>();
    int i = 0;
    for (JsonObject o : arr) {
      if (i >= MAX_BUTTONS) break;
      strlcpy(cfg.buttons[i].label, o["l"] | "", sizeof(cfg.buttons[i].label));
      cfg.buttons[i].ch  = constrain((int)(o["c"] | 0), 0, SBUS_CH_COUNT_24);
      cfg.buttons[i].val = constrain((int)(o["v"] | SBUS_MAX), SBUS_MIN, SBUS_MAX);
      i++;
    }
    saveConfig();
    // Echo updated config back to all clients
    String cfgJson = buildCfgJson();
    ws.textAll(cfgJson);
    Serial.println("[SBUS] Config updated via WebSocket.");
  }

  // ── SBUS mode toggle ──────────────────────────────────────────────────────
  // { "t":"mode", "sbus24": true|false }
  else if (!strcmp(t, "mode")) {
    bool newMode = doc["sbus24"] | g_sbus24;
    if (newMode != g_sbus24) {
      g_sbus24   = newMode;
      cfg.sbus24 = newMode;
      saveConfig();
      // Reset all channels to center so no stale values carry over
      for (int i = 0; i < SBUS_CH_COUNT_24; i++) sbusChannels[i] = SBUS_CENTER;
      Serial.printf("[SBUS] Mode switched → SBUS-%d (%d bytes/frame)\n",
                    sbusChCount(), sbusFrameLen());
    }
    ws.textAll(buildCfgJson());  // broadcast new state to all clients
  }

  // ── Debug toggle ──────────────────────────────────────────────────────────
  // { "t":"dbg", "on": true|false }
  else if (!strcmp(t, "dbg")) {
#ifdef SBUS_DEBUG
    g_sbusDebug = doc["on"] | !g_sbusDebug;
    Serial.printf("[SBUS] Debug output %s\n", g_sbusDebug ? "ENABLED" : "DISABLED");
    ws.textAll(buildCfgJson());  // broadcast updated dbg state
#else
    Serial.println("[SBUS] Debug not compiled in — define SBUS_DEBUG to enable.");
    client->text("{\"e\":\"err\",\"msg\":\"Recompile with SBUS_DEBUG defined\"}");
#endif
  }
}

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[SBUS] WS#%u connected.\n", client->id());
    // Send current config to newly connected client
    client->text(buildCfgJson());

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[SBUS] WS#%u disconnected.\n", client->id());

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    // Only process complete, single-frame text messages
    if (info->final && info->index == 0 && info->len == len
        && info->opcode == WS_TEXT) {
      handleWsMessage(client, data, len);
    }
  }
}

// =============================================================================
//  EMBEDDED HTML  (served from PROGMEM)
// =============================================================================

// Raw string delimiter 'rawhtml' is chosen to be safe from any HTML content.
static const char HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,user-scalable=no">
<title>SBUS Controller</title>
<style>
  :root {
    --bg:     #0d0f14;
    --panel:  #161920;
    --border: #2a2d38;
    --accent: #4fc3f7;
    --green:  #66bb6a;
    --red:    #ef5350;
    --text:   #e0e4f0;
    --muted:  #6b7280;
    --stick-bg:  #1a1d28;
    --stick-rim: #2e3450;
    --thumb:     #4fc3f7;
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{
    background:var(--bg);color:var(--text);
    font-family:'Segoe UI',system-ui,sans-serif;
    min-height:100vh;display:flex;flex-direction:column;
    align-items:center;padding:14px;gap:12px;
  }

  /* ── Header ────────────────────────────────────────────────────────────── */
  header{
    width:100%;max-width:900px;
    display:flex;align-items:center;justify-content:space-between;
    flex-wrap:wrap;gap:8px;
  }
  .logo{font-size:1.15rem;font-weight:700;letter-spacing:.1em;color:var(--accent);}
  #connBadge{
    font-size:.72rem;font-weight:600;letter-spacing:.06em;
    padding:4px 12px;border-radius:20px;
    background:var(--panel);border:1px solid var(--red);color:var(--red);
    transition:all .3s;
  }
  #connBadge.connected{border-color:var(--green);color:var(--green);}

  /* mode + debug toggle buttons */
  .hdr-btn{
    font-size:.72rem;font-weight:700;letter-spacing:.07em;
    padding:4px 13px;border-radius:20px;cursor:pointer;
    background:var(--panel);border:1px solid var(--border);color:var(--muted);
    transition:all .2s;user-select:none;
  }
  .hdr-btn.active-16 { border-color:#ffa726;color:#ffa726; }
  .hdr-btn.active-24 { border-color:var(--accent);color:var(--accent); }
  .hdr-btn.dbg-on    { border-color:var(--green);color:var(--green); }

  /* ── Joystick area ─────────────────────────────────────────────────────── */
  .sticks-row{
    width:100%;max-width:900px;
    display:flex;gap:16px;justify-content:center;flex-wrap:wrap;
  }
  .stick-card{
    background:var(--panel);border:1px solid var(--border);border-radius:12px;
    padding:14px;display:flex;flex-direction:column;align-items:center;gap:8px;
    flex:0 0 auto;
  }
  .stick-label{
    font-size:.65rem;font-weight:700;letter-spacing:.1em;
    text-transform:uppercase;color:var(--muted);
  }
  .stick-wrap{
    position:relative;
    width:min(240px,42vw);aspect-ratio:1;
    border-radius:50%;
    background:var(--stick-bg);border:2px solid var(--stick-rim);
    touch-action:none;user-select:none;cursor:crosshair;
  }
  /* crosshair */
  .stick-wrap::before,.stick-wrap::after{
    content:'';position:absolute;background:var(--stick-rim);
  }
  .stick-wrap::before{width:1px;height:100%;left:50%;top:0;}
  .stick-wrap::after {width:100%;height:1px;top:50%;left:0;}
  .thumb{
    position:absolute;width:40px;height:40px;border-radius:50%;
    background:radial-gradient(circle at 35% 35%,#7fd6ff,var(--thumb));
    box-shadow:0 0 12px rgba(79,195,247,.45);
    transform:translate(-50%,-50%);pointer-events:none;
    transition:box-shadow .15s;
  }
  .stick-wrap.active .thumb{box-shadow:0 0 22px rgba(79,195,247,.85);}
  .stick-readout{
    font-size:.68rem;color:var(--muted);font-variant-numeric:tabular-nums;
    letter-spacing:.03em;white-space:nowrap;
  }

  /* ── Button grid ───────────────────────────────────────────────────────── */
  .btn-section{width:100%;max-width:900px;}
  .btn-grid{
    display:grid;grid-template-columns:repeat(5,1fr);gap:8px;
  }
  @media(max-width:500px){
    .sticks-row{flex-direction:column;align-items:center;}
    .btn-grid{grid-template-columns:repeat(3,1fr);}
  }
  .ctrl-btn{
    padding:14px 6px;
    border:1px solid var(--border);border-radius:8px;
    background:#1a1d28;color:var(--text);
    font-size:.78rem;font-weight:600;
    cursor:pointer;text-align:center;word-break:break-word;
    transition:border-color .15s,background .15s,transform .1s;
    -webkit-tap-highlight-color:transparent;
    user-select:none;
  }
  .ctrl-btn.unassigned{color:var(--muted);border-style:dashed;cursor:default;}
  .ctrl-btn.pressed{background:var(--accent);color:#000;border-color:var(--accent);transform:scale(.96);}
  .ctrl-btn:not(.unassigned):hover{border-color:var(--accent);}

  /* ── Settings panel ────────────────────────────────────────────────────── */
  .settings-wrap{width:100%;max-width:900px;}
  details{width:100%;}
  summary{
    cursor:pointer;
    font-size:.72rem;font-weight:700;letter-spacing:.08em;text-transform:uppercase;
    color:var(--muted);padding:10px 14px;
    background:var(--panel);border:1px solid var(--border);border-radius:10px;
    list-style:none;display:flex;align-items:center;gap:8px;user-select:none;
  }
  summary::-webkit-details-marker{display:none;}
  summary::before{content:'▶';transition:transform .2s;font-size:.6rem;}
  details[open] summary{border-radius:10px 10px 0 0;}
  details[open] summary::before{transform:rotate(90deg);}
  .settings-body{
    background:var(--panel);border:1px solid var(--border);border-top:none;
    border-radius:0 0 10px 10px;padding:16px;display:flex;flex-direction:column;gap:16px;
  }
  .section-title{font-size:.65rem;font-weight:700;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);}

  /* axis rows */
  .axis-table{display:flex;flex-direction:column;gap:6px;}
  .axis-row{display:flex;align-items:center;gap:10px;}
  .axis-row label{font-size:.78rem;color:var(--text);width:80px;flex-shrink:0;}
  .axis-row select{
    background:var(--bg);border:1px solid var(--border);border-radius:6px;
    color:var(--text);padding:6px 8px;font-size:.78rem;flex:1;max-width:160px;
  }
  .axis-row select:focus{outline:none;border-color:var(--accent);}

  /* button config table */
  .btn-table{width:100%;border-collapse:collapse;}
  .btn-table th{
    font-size:.62rem;font-weight:700;letter-spacing:.08em;text-transform:uppercase;
    color:var(--muted);padding:4px 6px;text-align:left;border-bottom:1px solid var(--border);
  }
  .btn-table td{padding:4px 6px;vertical-align:middle;}
  .btn-table tr:nth-child(even) td{background:rgba(255,255,255,.02);}
  .btn-table input[type=text],.btn-table select,.btn-table input[type=number]{
    background:var(--bg);border:1px solid var(--border);border-radius:5px;
    color:var(--text);padding:5px 7px;font-size:.76rem;width:100%;
  }
  .btn-table input:focus,.btn-table select:focus{outline:none;border-color:var(--accent);}
  .save-btn{
    align-self:flex-end;
    padding:9px 22px;border-radius:8px;
    border:1px solid var(--accent);background:transparent;color:var(--accent);
    font-size:.8rem;font-weight:700;cursor:pointer;transition:all .15s;
  }
  .save-btn:hover{background:var(--accent);color:#000;}

  /* ── Debug channel grid ────────────────────────────────────────────────── */
  .dbg-grid{
    display:grid;
    grid-template-columns:repeat(8,1fr);
    gap:5px;
  }
  @media(max-width:600px){ .dbg-grid{grid-template-columns:repeat(4,1fr);} }
  .dbg-cell{
    background:var(--bg);border:1px solid var(--border);border-radius:5px;
    padding:5px 4px;text-align:center;transition:border-color .1s;
  }
  .dbg-cell .dcn{ font-size:.58rem;color:var(--muted);letter-spacing:.04em; }
  .dbg-cell .dcv{
    font-size:.78rem;font-variant-numeric:tabular-nums;
    color:var(--text);display:block;margin-top:1px;
  }
  .dbg-cell.lit{ border-color:var(--accent); }
  .dbg-cell.lit .dcv{ color:var(--accent); }
  .dbg-info{
    font-size:.68rem;color:var(--muted);margin-top:6px;
    display:flex;gap:16px;flex-wrap:wrap;
  }
  .dbg-info span{ white-space:nowrap; }
  .dbg-info .di-val{ color:var(--text); }
</style>
</head>
<body>

<!-- ── Header ─────────────────────────────────────────────────────────────── -->
<header>
  <div class="logo">SBUS CONTROLLER</div>
  <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;">
    <button class="hdr-btn" id="modeBtn"  onclick="toggleMode()" title="Toggle SBUS-16 / SBUS-24">SBUS-24</button>
    <button class="hdr-btn" id="debugBtn" onclick="toggleDebug()" title="Toggle Serial debug output">DEBUG OFF</button>
    <div id="connBadge">DISCONNECTED</div>
  </div>
</header>

<!-- ── Joysticks ──────────────────────────────────────────────────────────── -->
<div class="sticks-row">

  <div class="stick-card">
    <div class="stick-label" id="labelL">LEFT STICK — LX:CH1  LY:CH2</div>
    <div class="stick-wrap" id="stickL">
      <div class="thumb" id="thumbL" style="left:50%;top:50%"></div>
    </div>
    <div class="stick-readout" id="readL">LX: 992&nbsp;&nbsp;LY: 992</div>
  </div>

  <div class="stick-card">
    <div class="stick-label" id="labelR">RIGHT STICK — RX:CH3  RY:CH4</div>
    <div class="stick-wrap" id="stickR">
      <div class="thumb" id="thumbR" style="left:50%;top:50%"></div>
    </div>
    <div class="stick-readout" id="readR">RX: 992&nbsp;&nbsp;RY: 992</div>
  </div>

</div>

<!-- ── Button grid ─────────────────────────────────────────────────────────── -->
<div class="btn-section">
  <div class="btn-grid" id="btnGrid"></div>
</div>

<!-- ── Settings panel ─────────────────────────────────────────────────────── -->
<div class="settings-wrap">
  <details>
    <summary>Settings</summary>
    <div class="settings-body">

      <!-- Axis channel assignment -->
      <div class="section-title">Joystick Channels</div>
      <div class="axis-table">
        <div class="axis-row"><label>Left X</label>  <select id="selLX"></select></div>
        <div class="axis-row"><label>Left Y</label>  <select id="selLY"></select></div>
        <div class="axis-row"><label>Right X</label> <select id="selRX"></select></div>
        <div class="axis-row"><label>Right Y</label> <select id="selRY"></select></div>
      </div>

      <!-- Button config table -->
      <div class="section-title">Button Configuration</div>
      <table class="btn-table">
        <thead>
          <tr>
            <th>#</th>
            <th>Label</th>
            <th>Channel</th>
            <th>Pressed Value</th>
          </tr>
        </thead>
        <tbody id="btnConfigBody"></tbody>
      </table>

      <button class="save-btn" onclick="saveConfig()">&#128190; Save Config</button>

    </div>
  </details>
</div>

<!-- ── Debug panel (always rendered; data only flows when DEBUG ON) ─────────── -->
<div class="settings-wrap" id="debugWrap">
  <details id="debugDetails" open>
    <summary>Live Channel Monitor</summary>
    <div class="settings-body">
      <div id="dbgOffMsg" style="font-size:.75rem;color:var(--muted);text-align:center;padding:8px 0;">
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
const BTN_COUNT = 15;

let ws;
let cfg = {
  lx: 1, ly: 2, rx: 3, ry: 4,
  sbus24: true,
  b: Array.from({length: BTN_COUNT}, (_, i) => ({l: `Button ${i+1}`, c: 0, v: 1811}))
};

// Track debug state locally so the button reflects it
let g_dbgOn = false;

// =============================================================================
//  WebSocket
// =============================================================================

function connect() {
  ws = new WebSocket(`ws://${location.hostname}/ws`);

  ws.onopen = () => {
    const b = document.getElementById('connBadge');
    b.textContent = 'CONNECTED';
    b.className = 'connected';
  };

  ws.onclose = () => {
    const b = document.getElementById('connBadge');
    b.textContent = 'DISCONNECTED';
    b.className = '';
    setTimeout(connect, 3000);
  };

  ws.onerror = () => {};

  ws.onmessage = (e) => {
    try {
      const msg = JSON.parse(e.data);
      if      (msg.e === 'cfg')    applyCfg(msg);
      else if (msg.e === 'chdata') handleChData(msg);
    } catch(_) {}
  };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

// =============================================================================
//  Config handling
// =============================================================================

function applyCfg(msg) {
  cfg.lx     = msg.lx     || 1;
  cfg.ly     = msg.ly     || 2;
  cfg.rx     = msg.rx     || 3;
  cfg.ry     = msg.ry     || 4;
  cfg.sbus24 = (msg.sbus24 !== undefined) ? msg.sbus24 : true;
  g_dbgOn    = msg.dbg || false;
  if (Array.isArray(msg.b)) {
    for (let i = 0; i < BTN_COUNT; i++) {
      if (msg.b[i]) {
        cfg.b[i] = {
          l: msg.b[i].l || `Button ${i+1}`,
          c: msg.b[i].c || 0,
          v: msg.b[i].v || SBUS_MAX
        };
      }
    }
  }
  renderAll();
  updateModeBtn();
  updateDebugBtn();
}

// ─── Mode toggle ─────────────────────────────────────────────────────────────

function toggleMode() {
  cfg.sbus24 = !cfg.sbus24;
  send({t: 'mode', sbus24: cfg.sbus24});
  updateModeBtn();
}

function updateModeBtn() {
  const btn = document.getElementById('modeBtn');
  if (cfg.sbus24) {
    btn.textContent = 'SBUS-24';
    btn.className   = 'hdr-btn active-24';
    btn.title       = 'Running SBUS-24 (36-byte frame) — click to switch to SBUS-16';
  } else {
    btn.textContent = 'SBUS-16';
    btn.className   = 'hdr-btn active-16';
    btn.title       = 'Running SBUS-16 (25-byte frame) — click to switch to SBUS-24';
  }
}

// ─── Debug toggle ─────────────────────────────────────────────────────────────

function toggleDebug() {
  g_dbgOn = !g_dbgOn;
  send({t: 'dbg', on: g_dbgOn});
  updateDebugBtn();
}

function updateDebugBtn() {
  const btn    = document.getElementById('debugBtn');
  const offMsg = document.getElementById('dbgOffMsg');
  if (g_dbgOn) {
    btn.textContent = 'DEBUG ON';
    btn.className   = 'hdr-btn dbg-on';
    btn.title       = 'Serial + web debug active — click to disable';
    if (offMsg) offMsg.style.display = 'none';
  } else {
    btn.textContent = 'DEBUG OFF';
    btn.className   = 'hdr-btn';
    btn.title       = 'Debug inactive — click to enable';
    // hide the grid but keep the panel visible
    if (offMsg) offMsg.style.display = '';
    const grid = document.getElementById('dbgGrid');
    const info = document.getElementById('dbgInfo');
    if (grid) grid.style.display = 'none';
    if (info) info.style.display = 'none';
  }
}

// =============================================================================
//  Live channel data (from debug WebSocket push)
// =============================================================================

let dbgCells  = [];   // array of {cell, valEl} per channel
let dbgLitTimers = [];
let dbgLastT  = 0;
let dbgUpdateCount = 0;
let dbgRateTimer   = null;
let dbgUpdatesInWindow = 0;

function initDbgGrid(chcnt) {
  const grid = document.getElementById('dbgGrid');
  if (!grid) return;
  grid.innerHTML = '';
  dbgCells = [];
  dbgLitTimers = new Array(chcnt).fill(null);

  for (let i = 0; i < chcnt; i++) {
    const cell   = document.createElement('div');
    cell.className = 'dbg-cell';

    const cn = document.createElement('div');
    cn.className = 'dcn';
    cn.textContent = `CH${String(i + 1).padStart(2, '0')}`;

    const cv = document.createElement('span');
    cv.className = 'dcv';
    cv.textContent = '—';

    cell.appendChild(cn);
    cell.appendChild(cv);
    grid.appendChild(cell);
    dbgCells.push({cell, cv});
  }
}

function handleChData(msg) {
  const ch   = msg.ch;
  const mode = msg.mode;
  const fl   = msg.fl;
  if (!Array.isArray(ch)) return;

  // First data arrival — swap the "off" message for the live grid
  const offMsg = document.getElementById('dbgOffMsg');
  const grid   = document.getElementById('dbgGrid');
  const info   = document.getElementById('dbgInfo');
  if (offMsg) offMsg.style.display = 'none';
  if (grid)   grid.style.display   = '';
  if (info)   info.style.display   = '';

  // Rebuild grid if channel count changed
  if (dbgCells.length !== ch.length) initDbgGrid(ch.length);

  const now = performance.now();

  ch.forEach((val, i) => {
    const entry = dbgCells[i];
    if (!entry) return;
    const prev = parseInt(entry.cv.textContent) || -1;

    entry.cv.textContent = val;

    if (val !== prev) {
      // Flash the cell accent colour, then fade back after 300 ms
      entry.cell.classList.add('lit');
      if (dbgLitTimers[i]) clearTimeout(dbgLitTimers[i]);
      dbgLitTimers[i] = setTimeout(() => {
        entry.cell.classList.remove('lit');
        dbgLitTimers[i] = null;
      }, 300);
    }
  });

  // Update info bar
  const modeEl  = document.getElementById('diMode');
  const frameEl = document.getElementById('diFrame');
  const chEl    = document.getElementById('diCh');
  if (modeEl)  modeEl.textContent  = `SBUS-${mode}`;
  if (frameEl) frameEl.textContent = fl;
  if (chEl)    chEl.textContent    = ch.length;

  // Rolling update-rate counter (updates/sec over last second)
  dbgUpdatesInWindow++;
  if (!dbgRateTimer) {
    dbgRateTimer = setInterval(() => {
      const rateEl = document.getElementById('diRate');
      if (rateEl) rateEl.textContent = `${dbgUpdatesInWindow}/s`;
      dbgUpdatesInWindow = 0;
    }, 1000);
  }
}

function saveConfig() {
  // Read axis selects
  cfg.lx = parseInt(document.getElementById('selLX').value);
  cfg.ly = parseInt(document.getElementById('selLY').value);
  cfg.rx = parseInt(document.getElementById('selRX').value);
  cfg.ry = parseInt(document.getElementById('selRY').value);

  // Read button config rows
  const tbody = document.getElementById('btnConfigBody');
  for (let i = 0; i < BTN_COUNT; i++) {
    const row = tbody.rows[i];
    cfg.b[i].l = row.cells[1].querySelector('input').value;
    cfg.b[i].c = parseInt(row.cells[2].querySelector('select').value);
    cfg.b[i].v = parseInt(row.cells[3].querySelector('input').value);
  }

  send({t: 'cfg', lx: cfg.lx, ly: cfg.ly, rx: cfg.rx, ry: cfg.ry, b: cfg.b});
}

// =============================================================================
//  Render helpers
// =============================================================================

function axisToSbus(v) {
  // v is -1..+1; returns integer SBUS value
  return Math.round((v * 0.5 + 0.5) * (SBUS_MAX - SBUS_MIN) + SBUS_MIN);
}

function renderAll() {
  updateStickLabels();
  renderButtons();
  renderSettings();
  updateReadouts();
  updateModeBtn();
  updateDebugBtn();
}

function updateStickLabels() {
  document.getElementById('labelL').textContent =
    `LEFT STICK — LX:CH${cfg.lx}  LY:CH${cfg.ly}`;
  document.getElementById('labelR').textContent =
    `RIGHT STICK — RX:CH${cfg.rx}  RY:CH${cfg.ry}`;
}

function updateReadouts() {
  // Read directly from stick state — never through the channel array.
  // Channel assignments control the SBUS output mapping; they must not affect
  // which stick drives which readout.  If two axes share a channel the SBUS
  // output is intentionally overwritten, but the display stays correct.
  document.getElementById('readL').innerHTML =
    `LX:&nbsp;${axisToSbus(sticks.L.x)}&nbsp;&nbsp;LY:&nbsp;${axisToSbus(-sticks.L.y)}`;
  document.getElementById('readR').innerHTML =
    `RX:&nbsp;${axisToSbus(sticks.R.x)}&nbsp;&nbsp;RY:&nbsp;${axisToSbus(-sticks.R.y)}`;
}

// Build channel <select> options: None + CH1–CH24
function buildChSelect(selectedVal, includeNone) {
  const sel = document.createElement('select');
  if (includeNone) {
    const opt = document.createElement('option');
    opt.value = '0'; opt.textContent = 'None';
    if (selectedVal === 0) opt.selected = true;
    sel.appendChild(opt);
  }
  for (let c = 1; c <= 24; c++) {
    const opt = document.createElement('option');
    opt.value = String(c);
    opt.textContent = `CH${c}`;
    if (c === selectedVal) opt.selected = true;
    sel.appendChild(opt);
  }
  return sel;
}

// ── Button grid ───────────────────────────────────────────────────────────────

function renderButtons() {
  const grid = document.getElementById('btnGrid');
  grid.innerHTML = '';
  for (let i = 0; i < BTN_COUNT; i++) {
    const btn = cfg.b[i];
    const el = document.createElement('button');
    el.textContent = btn.l || `Button ${i+1}`;
    el.className = 'ctrl-btn' + (btn.c === 0 ? ' unassigned' : '');
    el.dataset.idx = i;

    if (btn.c !== 0) {
      // Touch events
      el.addEventListener('touchstart', (e) => { e.preventDefault(); btnPress(i, el); }, {passive: false});
      el.addEventListener('touchend',   (e) => { e.preventDefault(); btnRelease(i, el); }, {passive: false});
      // Mouse events
      el.addEventListener('mousedown',   () => btnPress(i, el));
      el.addEventListener('mouseup',     () => btnRelease(i, el));
      el.addEventListener('mouseleave',  () => btnRelease(i, el));
    }
    grid.appendChild(el);
  }
}

function btnPress(idx, el) {
  el.classList.add('pressed');
  send({t: 'b', i: idx, p: true});
}

function btnRelease(idx, el) {
  el.classList.remove('pressed');
  send({t: 'b', i: idx, p: false});
}

// ── Settings panel ────────────────────────────────────────────────────────────

function renderSettings() {
  // Axis selects
  const ids   = ['selLX','selLY','selRX','selRY'];
  const vals  = [cfg.lx, cfg.ly, cfg.rx, cfg.ry];
  ids.forEach((id, i) => {
    const container = document.getElementById(id).parentNode;
    const newSel    = buildChSelect(vals[i], false);
    newSel.id = id;
    container.replaceChild(newSel, document.getElementById(id));
  });

  // Button config table
  const tbody = document.getElementById('btnConfigBody');
  tbody.innerHTML = '';
  for (let i = 0; i < BTN_COUNT; i++) {
    const btn = cfg.b[i];
    const tr  = document.createElement('tr');

    // # column
    const tdNum = document.createElement('td');
    tdNum.textContent = i + 1;
    tdNum.style.cssText = 'font-size:.7rem;color:var(--muted);text-align:center;';
    tr.appendChild(tdNum);

    // Label column
    const tdLbl = document.createElement('td');
    const inp   = document.createElement('input');
    inp.type = 'text';
    inp.value = btn.l;
    inp.maxLength = 31;
    inp.oninput = () => { cfg.b[i].l = inp.value; renderButtons(); };
    tdLbl.appendChild(inp);
    tr.appendChild(tdLbl);

    // Channel column
    const tdCh = document.createElement('td');
    tdCh.appendChild(buildChSelect(btn.c, true));
    tr.appendChild(tdCh);

    // Value column
    const tdVal = document.createElement('td');
    const numInp = document.createElement('input');
    numInp.type = 'number';
    numInp.min = String(SBUS_MIN);
    numInp.max = String(SBUS_MAX);
    numInp.value = btn.v;
    tdVal.appendChild(numInp);
    tr.appendChild(tdVal);

    tbody.appendChild(tr);
  }
}

// =============================================================================
//  Joystick logic
// =============================================================================

const sticks = {
  L: {x: 0, y: 0, active: false, touchId: null},
  R: {x: 0, y: 0, active: false, touchId: null}
};

let joyTimer = null;

function scheduleJoySend() {
  if (joyTimer) return;
  joyTimer = setTimeout(() => {
    joyTimer = null;
    send({t: 'a', lx: sticks.L.x, ly: sticks.L.y, rx: sticks.R.x, ry: sticks.R.y});
    updateReadouts();
  }, 30);
}

function posFromClientXY(clientX, clientY, wrap) {
  const r  = wrap.getBoundingClientRect();
  const cx = r.left + r.width  / 2;
  const cy = r.top  + r.height / 2;
  let nx = (clientX - cx) / (r.width  / 2);
  let ny = (clientY - cy) / (r.height / 2);
  // Clamp to unit circle
  const mag = Math.sqrt(nx * nx + ny * ny);
  if (mag > 1) { nx /= mag; ny /= mag; }
  return [+nx.toFixed(3), +ny.toFixed(3)];
}

function applyStick(side, nx, ny) {
  sticks[side].x = nx;
  sticks[side].y = ny;
  const thumb = document.getElementById('thumb' + side);
  thumb.style.left = (50 + nx * 44) + '%';
  thumb.style.top  = (50 + ny * 44) + '%';
  scheduleJoySend();
}

function springBack(side) {
  applyStick(side, 0, 0);
}

// ── Touch ─────────────────────────────────────────────────────────────────────

function initStick(side) {
  const wrap = document.getElementById('stick' + side);

  wrap.addEventListener('touchstart', (e) => {
    e.preventDefault();
    const t = e.changedTouches[0];
    sticks[side].touchId = t.identifier;
    sticks[side].active  = true;
    wrap.classList.add('active');
    const [nx, ny] = posFromClientXY(t.clientX, t.clientY, wrap);
    applyStick(side, nx, ny);
  }, {passive: false});

  wrap.addEventListener('touchmove', (e) => {
    e.preventDefault();
    for (const t of e.changedTouches) {
      if (t.identifier === sticks[side].touchId) {
        const [nx, ny] = posFromClientXY(t.clientX, t.clientY, wrap);
        applyStick(side, nx, ny);
      }
    }
  }, {passive: false});

  wrap.addEventListener('touchend', (e) => {
    e.preventDefault();
    for (const t of e.changedTouches) {
      if (t.identifier === sticks[side].touchId) {
        sticks[side].active  = false;
        sticks[side].touchId = null;
        wrap.classList.remove('active');
        springBack(side);
      }
    }
  }, {passive: false});

  // ── Mouse ──────────────────────────────────────────────────────────────────

  wrap.addEventListener('mousedown', (e) => {
    sticks[side].active = true;
    wrap.classList.add('active');
    const [nx, ny] = posFromClientXY(e.clientX, e.clientY, wrap);
    applyStick(side, nx, ny);

    const onMove = (ev) => {
      if (!sticks[side].active) return;
      const [nx2, ny2] = posFromClientXY(ev.clientX, ev.clientY, wrap);
      applyStick(side, nx2, ny2);
    };
    const onUp = () => {
      sticks[side].active = false;
      wrap.classList.remove('active');
      springBack(side);
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup',   onUp);
    };
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup',   onUp);
  });
}

// =============================================================================
//  Boot
// =============================================================================

initStick('L');
initStick('R');
renderAll();
connect();
</script>
</body>
</html>)rawhtml";

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n[SBUS] SBUSController booting...");

  // SBUS output: 100 kbaud, 8E2, inverted signal
  // Serial1.begin(baud, config, rx_pin, tx_pin, invert)
  // rx_pin = -1 → RX not used
  Serial1.begin(SBUS_BAUD, SERIAL_8E2, -1, SBUS_TX_PIN, true /*invert*/);
  Serial.printf("[SBUS] Serial1 TX on GPIO%d  (100kbaud 8E2 inverted)\n", SBUS_TX_PIN);

  // Initialize all channels to center (always fill the full 24-channel array)
  for (int i = 0; i < SBUS_CH_COUNT_24; i++) {
    sbusChannels[i] = SBUS_CENTER;
  }

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("[SBUS] ERROR: LittleFS mount failed!");
  } else {
    loadConfig();
  }

  // Start web server BEFORE connecting to WiFi.
  // AsyncTCP allocates its TCP socket via LWIP's tcp_alloc(), which requires the
  // TCPIP core mutex.  Calling server.begin() after WiFi.begin() returns can hit a
  // timing window where the mutex is already held by the WiFi/IP stack, causing the
  // "Required to lock TCPIP core functionality!" assert.  Starting the server while
  // LWIP is initialised but idle (before any connection attempt) avoids this entirely.
  // The server binds to INADDR_ANY so it works on whichever interface comes up.
  WiFi.persistent(false);        // don't write credentials to flash on every connect
  WiFi.mode(WIFI_STA);           // initialise LWIP now, before server.begin()

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", HTML);
  });
  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("[SBUS] Web server started.");

  // WiFi — cascading: primary → fallback → AP
  // Mode is already STA; just call begin() + wait.
  auto tryConnect = [](const char* ssid, const char* pass) -> bool {
    Serial.printf("[SBUS] Trying \"%s\"...\n", ssid);
    WiFi.disconnect(false);
    WiFi.begin(ssid, pass);
    const uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < WIFI_STA_TIMEOUT_MS) {
      delay(100);
    }
    return WiFi.status() == WL_CONNECTED;
  };

  if (tryConnect(WIFI_PRIMARY_SSID, WIFI_PRIMARY_PASS)) {
    Serial.printf("[SBUS] ✓ Connected to \"%s\"  →  http://%s\n",
      WIFI_PRIMARY_SSID, WiFi.localIP().toString().c_str());

  } else if (tryConnect(WIFI_FALLBACK_SSID, WIFI_FALLBACK_PASS)) {
    Serial.printf("[SBUS] ✓ Connected to \"%s\"  →  http://%s\n",
      WIFI_FALLBACK_SSID, WiFi.localIP().toString().c_str());

  } else {
    Serial.printf("[SBUS] ✗ Both networks failed — starting AP \"%s\"\n", WIFI_AP_SSID);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.printf("[SBUS] AP ready  →  http://%s\n", WiFi.softAPIP().toString().c_str());
  }

  Serial.printf("[SBUS] Mode: SBUS-%d  (%d ch, %d bytes/frame)\n",
                sbusChCount(), sbusChCount(), sbusFrameLen());
  Serial.println("[SBUS] Serial cmds:  m=toggle mode  d=toggle debug  ?=status");
  Serial.println("[SBUS] Ready.");
}

// =============================================================================
//  LOOP
// =============================================================================

// Handle single-character Serial commands:
//   m  →  toggle SBUS mode (SBUS-16 ↔ SBUS-24)
//   d  →  toggle debug output on/off
//   ?  →  print current status
static void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;

    if (c == 'm') {
      g_sbus24   = !g_sbus24;
      cfg.sbus24 = g_sbus24;
      saveConfig();
      for (int i = 0; i < SBUS_CH_COUNT_24; i++) sbusChannels[i] = SBUS_CENTER;
      Serial.printf("[SBUS] Mode → SBUS-%d (%d ch, %d bytes/frame)\n",
                    sbusChCount(), sbusChCount(), sbusFrameLen());
      ws.textAll(buildCfgJson());

    } else if (c == 'd') {
#ifdef SBUS_DEBUG
      g_sbusDebug = !g_sbusDebug;
      Serial.printf("[SBUS] Debug %s\n", g_sbusDebug ? "ENABLED" : "DISABLED");
      ws.textAll(buildCfgJson());
#else
      Serial.println("[SBUS] Debug not compiled in — uncomment #define SBUS_DEBUG");
#endif

    } else if (c == '?') {
      Serial.printf("[SBUS] Mode: SBUS-%d  |  Frame: %d bytes  |  Channels: %d",
                    sbusChCount(), sbusFrameLen(), sbusChCount());
#ifdef SBUS_DEBUG
      Serial.printf("  |  Debug: %s", g_sbusDebug ? "ON" : "OFF");
#endif
      Serial.println();
    }
  }
}

void loop() {
  // Handle Serial commands (m=mode, d=debug, ?=status)
  handleSerialCommands();

  // Transmit SBUS frame every 14 ms
  uint32_t now = millis();
  if (now - lastFrameMs >= SBUS_FRAME_MS) {
    lastFrameMs = now;
    transmitSbus();
  }

  // Periodically clean up stale WebSocket clients
  ws.cleanupClients();
}
