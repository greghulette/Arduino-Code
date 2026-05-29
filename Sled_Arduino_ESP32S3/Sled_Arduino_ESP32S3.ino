// Sled Controller for ESP32-S3 with WCB 3.2 Hardware
// ----------------------------------------------------------------------------
// 2026-05 MATRIX PORT (Stage 1):
//   The sled LEDs are now an 8x32 = 256-pixel WS2812B matrix (same panel as the
//   bipedal robot). Rendering is driven by the bipedal robot's LogicAnimations
//   engine (logic_animations.h/.cpp + matrix_patterns.h, copied into this
//   sketch folder), instantiated as a single strip named "SLED".
//
//   Animations available (LogicAnimations): Normal (R2 blink), Alarm, Failure,
//   Solid, Flash, FlipFlop, ColorSwap, Rainbow, RedAlert, Fire, Roaming/Cylon
//   scanner, LightsOut — each with speed / brightness / density / 4-colour
//   palette. Stage 2 will add the sled's own Twinkle/Pulse/Wipe/Chase/Sparkle/
//   Heartbeat/Matrix-Rain into the same engine.
//
// WCB 3.2 Pin Mapping for ESP32-S3:
//   Serial1 TX (GPIO 4)  → Matrix Data (WS2812B)
//   Serial2 TX (GPIO 6)  → Push Button Input (cycles animations)
//   Serial1 RX (GPIO 5)  → status LED 1     Serial2 RX (GPIO 7)  → status LED 2
//   Serial3 TX (GPIO 15) → status LED 3     Serial3 RX (GPIO 16) → status LED 4
//   Serial4 TX (GPIO 17) → status LED 5     Serial4 RX (GPIO 18) → status LED 6
//   Serial5 TX (GPIO 9)  → status LED 7     Serial5 RX (GPIO 10) → status LED 8
//   GPIO 21 (direct)     → status LED 9
//   Serial (USB-CDC)     → Debug Output

#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#define ETM_MY_BOARD_INDEX ETM_BOARD_SL
#include <ETM_Droid.h>
#include "logic_animations.h"     // bipedal robot matrix animation engine
#include <Preferences.h>          // NVS preset storage
#include <ArduinoJson.h>          // WebSerial preset-editor protocol
#include "sled_presets.h"         // SledPreset struct (in a header so IDE auto-prototypes see it)

// ==================== ESP-NOW / ETM ====================
// Must match ESPNOWPASSWORD on the Droid Gateway and Body Controller.
String ESPNOWPASSWORD = "GregsAstromech";

// ==================== PIN / MATRIX DEFINITIONS ====================
#define BUTTON_PIN          6     // Serial2 TX (repurposed as input) - WCB S2 TX
#define PIXEL_PIN           4     // Serial1 TX (matrix data line) - WCB S1 TX
#define MATRIX_ROWS         8
#define MATRIX_COLS         32
#define MATRIX_LED_COUNT    (MATRIX_ROWS * MATRIX_COLS)   // 256

// 9 individual status LEDs — driven by WCB 3.2 serial headers + 1 direct.
#define LEFT_BLUE           5    // WCB S1 RX
#define LEFT_WHITE          7    // WCB S2 RX
#define LEFT_GREEN          15   // WCB S3 TX
#define RIGHT_BLUE          16   // WCB S3 RX
#define RIGHT_WHITE         17   // WCB S4 TX
#define RIGHT_GREEN         18   // WCB S4 RX
#define LED7                9    // WCB S5 TX
#define LED8                10   // WCB S5 RX
#define LED9                21   // Direct to ESP32-S3 (not via WCB)

#define LED_COUNT           9
int LEDS[LED_COUNT] = {LEFT_BLUE, LEFT_WHITE, LEFT_GREEN, RIGHT_BLUE, RIGHT_WHITE, RIGHT_GREEN, LED7, LED8, LED9};

// ==================== MATRIX ANIMATION ENGINE ====================
// One LogicAnimations strip for the whole sled matrix, named "SLED".
LogicAnimations sled(PIXEL_PIN, MATRIX_LED_COUNT, "SLED");

// Animations the local button / :NEXT / :PREV cycle through (implemented ones
// only — reserved enum slots are skipped).
const LogicAnimation CYCLE_ANIMS[] = {
  LogicAnimation::NORMAL,
  LogicAnimation::ALARM,
  LogicAnimation::FAILURE,
  LogicAnimation::SOLIDCOLOR,
  LogicAnimation::FLASHCOLOR,
  LogicAnimation::FLIPFLOPCOLOR,
  LogicAnimation::COLORSWAP,
  LogicAnimation::RAINBOW,
  LogicAnimation::REDALERT,
  LogicAnimation::FIRE,
  LogicAnimation::ROAMINGPIXEL,
  // sled-ported
  LogicAnimation::TWINKLE,
  LogicAnimation::PULSE,
  LogicAnimation::WIPE,
  LogicAnimation::CHASE,
  LogicAnimation::SPARKLE,
  LogicAnimation::HEARTBEAT,
  LogicAnimation::MATRIXRAIN,
  LogicAnimation::LIGHTSOUT,
};
const int CYCLE_COUNT = sizeof(CYCLE_ANIMS) / sizeof(CYCLE_ANIMS[0]);
int animIndex = 0;   // index into CYCLE_ANIMS for the button cycle

// ==================== USER PRESETS (NVS) ====================
// A "preset" is a saved animation = a base animation TYPE plus its settings.
// Authored over USB by the standalone WebSerial editor (sled_animation_studio.html),
// stored in NVS as a packed blob, and used at runtime. Creating a "new animation"
// just means saving another preset of an existing type with different settings.
#define MAX_PRESETS 32
#define TEXT_MAX_COLS 256        // max width of a scroll-text bitmap (1 byte/col)

static inline uint8_t hexNib(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return 0;
}

// struct SledPreset is defined in sled_presets.h (included above).

SledPreset presets[MAX_PRESETS];
uint8_t    presetCount      = 0;
int        presetIndex      = 0;       // current preset in the cycle
bool       cycleUsesPresets = true;    // button cycles presets (true) or raw types (false)
Preferences sledPrefs;

// Live matrix mirror to the WebSerial editor (on-screen preview, no LEDs needed)
bool          mirrorOn       = false;
unsigned long mirrorLastMs   = 0;
const uint16_t MIRROR_INTERVAL_MS = 100;   // ~10 fps (fits the 115200 baud budget)

// Active matrix mask: false = SLED bordered region, true = FULL 8x32 panel.
bool maskFull = false;
// Panel mounting orientation — mirror logical coords to match the physical panel.
bool flipX = false, flipY = false;

// Push a preset's settings into the engine and start it.
void applyPreset(const SledPreset& p) {
  sled.setAnimation((LogicAnimation)p.anim);   // resets per-anim state
  sled.setSpeed(p.speed);
  sled.setBrightness(p.bright);
  sled.setDensity(p.density);
  for (uint8_t i = 0; i < 4; i++) {
    sled.setColor(i + 1, p.rgb[i][0], p.rgb[i][1], p.rgb[i][2]);  // slots are 1-based
    sled.setWeight(i + 1, p.weight[i]);
    sled.setColorBrightness(i + 1, p.cbright[i]);
  }
  sled.enable(true);
}

// ==================== STATE VARIABLES ====================
unsigned long currentMillis;
unsigned long blinkpreviousMillis = 0;
const long blinkinterval = 400;   // status-LED random blink interval (ms)

int randomNumberArray;
int lastButtonState = HIGH;
int stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool buttonPressed = false;

// ── Button multi-tap + auto-cycle ──────────────────────────────────────────
// Taps are counted within a short window so single vs double can be told apart:
//   single tap  → advance to next animation
//   double tap  → toggle AUTO-CYCLE (each animation runs AUTO_CYCLE_INTERVAL_MS,
//                 then advances). On enable, the panel flashes light-blue 3x.
uint8_t       tapCount = 0;
unsigned long lastTapMs = 0;
const unsigned long MULTITAP_WINDOW_MS = 600;   // 2 taps within this = double
bool          autoCycle = false;
unsigned long autoCycleLastMs = 0;
const unsigned long AUTO_CYCLE_INTERVAL_MS = 30000;   // 30s per animation

// ==================== ANIMATION HELPERS ====================
// Apply a cycle-list animation by index and keep animIndex in sync.
void applyAnimIndex(int n)
{
    if (n < 0 || n >= CYCLE_COUNT) return;
    animIndex = n;
    sled.setAnimation(CYCLE_ANIMS[n]);
    sled.enable(true);
    Serial.printf("[SLED] anim[%d] = %s\n", n, LogicAnimations::getAnimationName(CYCLE_ANIMS[n]));
}

void runPresetIndex(int n)
{
    if (presetCount == 0) return;
    if (n < 0) n = presetCount - 1;
    if (n >= presetCount) n = 0;
    presetIndex = n;
    applyPreset(presets[presetIndex]);
    Serial.printf("[SLED] preset[%d] = %s\n", presetIndex, presets[presetIndex].name);
}

// Button / :NEXT — step the active cycle source (presets, else raw types).
void cycleNext()
{
    if (cycleUsesPresets && presetCount > 0) runPresetIndex(presetIndex + 1);
    else                                     applyAnimIndex((animIndex + 1) % CYCLE_COUNT);
}

void cyclePrev()
{
    if (cycleUsesPresets && presetCount > 0) runPresetIndex(presetIndex - 1);
    else                                     applyAnimIndex((animIndex + CYCLE_COUNT - 1) % CYCLE_COUNT);
}

// Enable/disable auto-cycle. On enable, flash light-blue 3x and (re)start the
// per-animation timer; on disable, keep whatever animation is running.
void setAutoCycle(bool on)
{
    autoCycle = on;
    if (on) {
        Serial.println("[SLED] AUTO-CYCLE ON");
        sled.confirmPulse(80, 160, 255, 3);    // light-blue flash x3
        autoCycleLastMs = millis();
    } else {
        Serial.println("[SLED] AUTO-CYCLE OFF (keeping current animation)");
    }
}

// NOTE: LogicAnimations colour/weight slots are 1-BASED (1..4); colors_[0] is
// "COLOR1" == slot 1. Always pass 1/2 here, never 0.

// Force a single solid colour: pin COLOR1, zero the other weights so the
// weighted picker always lands on COLOR1, then run SOLIDCOLOR.
void sledSolid(const char* name)
{
    sled.setColor(1, name);
    sled.setWeight(1, 100); sled.setWeight(2, 0); sled.setWeight(3, 0); sled.setWeight(4, 0);
    sled.setAnimation(LogicAnimation::SOLIDCOLOR);
    sled.enable(true);
}

// Set COLOR1 (if given) and run a specific animation (SCAN / FLASH / etc.).
void sledColorAnim(LogicAnimation anim, const char* name)
{
    if (name && name[0]) sled.setColor(1, name);
    sled.setAnimation(anim);
    sled.enable(true);
}

// Map a speed token to a frame interval (ms), relative to a per-animation
// default. Keywords scale the default; a number is taken literally. The engine
// clamps to 10..1000 ms in setSpeed().
uint16_t sledSpeed(const char* tok, uint16_t def)
{
    if (!tok || !tok[0]) return def;
    if (strcasecmp(tok, "SLOW")   == 0) return def * 2;
    if (strcasecmp(tok, "MED")    == 0 || strcasecmp(tok, "NORMAL") == 0) return def;
    if (strcasecmp(tok, "FAST")   == 0) return def / 2;
    if (strcasecmp(tok, "TURBO")  == 0) return def / 4;
    int v = atoi(tok);
    if (v >= 5 && v <= 2000) return (uint16_t)v;
    return def;
}

// ==================== PRESET NVS PERSISTENCE ====================
void presetsLoadNVS()
{
    sledPrefs.begin("sledfx", true);   // read-only
    presetCount = sledPrefs.getUChar("count", 0);
    if (presetCount > MAX_PRESETS) presetCount = 0;
    if (presetCount > 0) {
        size_t want = (size_t)presetCount * sizeof(SledPreset);
        sledPrefs.getBytes("presets", presets, want);
    }
    cycleUsesPresets = sledPrefs.getUChar("cyclesrc", 1) != 0;
    maskFull = sledPrefs.getUChar("maskfull", 0) != 0;
    flipX = sledPrefs.getUChar("flipx", 0) != 0;
    flipY = sledPrefs.getUChar("flipy", 0) != 0;
    sledPrefs.end();
    sledSetFlip(flipX, flipY);    // before mask so the rebuild keeps flips
    sledSetMask(maskFull);
    Serial.printf("[SLED] NVS: %u presets, cycle=%s, mask=%s, flip=%c%c\n",
                  presetCount, cycleUsesPresets ? "presets" : "types",
                  maskFull ? "FULL" : "SLED", flipX ? 'X' : '-', flipY ? 'Y' : '-');
}

void presetsSaveNVS()
{
    sledPrefs.begin("sledfx", false);
    sledPrefs.putUChar("count", presetCount);
    sledPrefs.putBytes("presets", presets, (size_t)presetCount * sizeof(SledPreset));
    sledPrefs.putUChar("cyclesrc", cycleUsesPresets ? 1 : 0);
    sledPrefs.putUChar("maskfull", maskFull ? 1 : 0);
    sledPrefs.putUChar("flipx", flipX ? 1 : 0);
    sledPrefs.putUChar("flipy", flipY ? 1 : 0);
    sledPrefs.end();
}

// ---- scroll-text bitmap persistence ----
void textSaveNVS(const uint8_t* bmp, uint16_t cols, uint32_t color, bool scroll, uint16_t speed)
{
    sledPrefs.begin("sledfx", false);
    sledPrefs.putUShort("txtcols",   cols);
    sledPrefs.putUInt  ("txtcolor",  color);
    sledPrefs.putUChar ("txtscroll", scroll ? 1 : 0);
    sledPrefs.putUShort("txtspeed",  speed);
    if (cols > 0) sledPrefs.putBytes("txtbmp", bmp, cols);
    sledPrefs.end();
}

void textLoadNVS()
{
    sledPrefs.begin("sledfx", true);
    uint16_t cols = sledPrefs.getUShort("txtcols", 0);
    if (cols > 0 && cols <= TEXT_MAX_COLS) {
        static uint8_t buf[TEXT_MAX_COLS];
        sledPrefs.getBytes("txtbmp", buf, cols);
        uint32_t color = sledPrefs.getUInt("txtcolor", 0xFFFFFF);
        bool scroll = sledPrefs.getUChar("txtscroll", 1) != 0;
        sled.setText(buf, cols, color, scroll);   // bitmap ready; runs when SCROLLTEXT is selected
    }
    sledPrefs.end();
}

// ==================== WEBSERIAL PRESET-EDITOR PROTOCOL ====================
// JSON request/response over USB serial (direct connect from the standalone
// editor page). One JSON object per line; replies are one JSON object per line.
static void presetFromJson(JsonObjectConst o, SledPreset& p)
{
    memset(&p, 0, sizeof(p));
    strlcpy(p.name, o["name"] | "preset", sizeof(p.name));
    p.anim    = o["anim"]    | (uint8_t)LogicAnimation::TWINKLE;
    p.speed   = o["speed"]   | 60;
    p.bright  = o["bright"]  | 120;
    p.density = o["density"] | 15;
    for (uint8_t i = 0; i < 4; i++) {
        p.rgb[i][0] = o["colors"][i][0] | 0;
        p.rgb[i][1] = o["colors"][i][1] | 0;
        p.rgb[i][2] = o["colors"][i][2] | 0;
        p.weight[i]  = o["weights"][i] | 0;
        p.cbright[i] = o["cbright"][i] | 255;
    }
}

static void presetToJson(const SledPreset& p, JsonObject o, int idx)
{
    o["i"]       = idx;
    o["name"]    = p.name;
    o["anim"]    = p.anim;
    o["speed"]   = p.speed;
    o["bright"]  = p.bright;
    o["density"] = p.density;
    JsonArray colors = o.createNestedArray("colors");
    for (uint8_t i = 0; i < 4; i++) {
        JsonArray c = colors.createNestedArray();
        c.add(p.rgb[i][0]); c.add(p.rgb[i][1]); c.add(p.rgb[i][2]);
    }
    JsonArray w = o.createNestedArray("weights");
    for (uint8_t i = 0; i < 4; i++) w.add(p.weight[i]);
    JsonArray cb = o.createNestedArray("cbright");
    for (uint8_t i = 0; i < 4; i++) cb.add(p.cbright[i]);
}

// Stream the preset list one object per line (BEGIN header, one PRESET per
// line, END) so we never build one giant JSON doc — 32 presets would blow a
// single document's memory pool.
static void sendPresetList()
{
    {
        DynamicJsonDocument head(320);
        head["type"]     = "PRESETS_BEGIN";
        head["count"]    = presetCount;
        head["max"]      = MAX_PRESETS;
        head["cycleSrc"] = cycleUsesPresets ? "presets" : "types";
        head["maskFull"] = maskFull;
        head["flipX"]    = flipX;
        head["flipY"]    = flipY;
        head["auto"]     = autoCycle;
        serializeJson(head, Serial);
        Serial.println();
    }
    for (int i = 0; i < presetCount; i++) {
        DynamicJsonDocument doc(1024);
        JsonObject o = doc.to<JsonObject>();
        o["type"] = "PRESET";
        presetToJson(presets[i], o, i);
        serializeJson(doc, Serial);
        Serial.println();
    }
    Serial.println("{\"type\":\"PRESETS_END\"}");
}

// Base animation types the editor can build presets from.
static void sendBases()
{
    static const LogicAnimation list[] = {
        LogicAnimation::NORMAL, LogicAnimation::ALARM, LogicAnimation::FAILURE,
        LogicAnimation::SOLIDCOLOR, LogicAnimation::FLASHCOLOR, LogicAnimation::FLIPFLOPCOLOR,
        LogicAnimation::COLORSWAP, LogicAnimation::RAINBOW, LogicAnimation::REDALERT,
        LogicAnimation::FIRE, LogicAnimation::ROAMINGPIXEL,
        LogicAnimation::TWINKLE, LogicAnimation::PULSE, LogicAnimation::WIPE,
        LogicAnimation::CHASE, LogicAnimation::SPARKLE, LogicAnimation::HEARTBEAT,
        LogicAnimation::MATRIXRAIN, LogicAnimation::SCROLLTEXT, LogicAnimation::LIGHTSOUT
    };
    DynamicJsonDocument doc(2048);
    doc["type"] = "BASES";
    JsonArray arr = doc.createNestedArray("data");
    for (uint8_t k = 0; k < sizeof(list)/sizeof(list[0]); k++) {
        JsonObject o = arr.createNestedObject();
        o["id"]   = (uint8_t)list[k];
        o["name"] = LogicAnimations::getAnimationName(list[k]);
    }
    serializeJson(doc, Serial);
    Serial.println();
}

// Parse + dispatch one JSON line from the editor.
void handleStudioJson(const String& line)
{
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, line)) { Serial.println("{\"type\":\"ERR\",\"msg\":\"parse\"}"); return; }
    const char* cmd = doc["cmd"] | "";

    if (!strcmp(cmd, "LIST"))  { sendPresetList(); return; }
    if (!strcmp(cmd, "BASES")) { sendBases();      return; }

    if (!strcmp(cmd, "PREVIEW")) {                          // apply live, no save
        SledPreset p; presetFromJson(doc["preset"], p); applyPreset(p);
        Serial.println("{\"type\":\"ACK\",\"cmd\":\"PREVIEW\"}");
        return;
    }
    if (!strcmp(cmd, "SAVE")) {
        int i = doc["i"] | -1;
        SledPreset p; presetFromJson(doc["preset"], p);
        if (i < 0 || i >= presetCount) {                    // new / append
            if (presetCount >= MAX_PRESETS) { Serial.println("{\"type\":\"ERR\",\"msg\":\"full\"}"); return; }
            i = presetCount++;
        }
        presets[i] = p;
        presetsSaveNVS();
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"SAVE\",\"i\":%d}\n", i);
        return;
    }
    if (!strcmp(cmd, "DELETE")) {
        int i = doc["i"] | -1;
        if (i >= 0 && i < presetCount) {
            for (int k = i; k < presetCount - 1; k++) presets[k] = presets[k + 1];
            presetCount--;
            presetsSaveNVS();
        }
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"DELETE\",\"i\":%d}\n", i);
        return;
    }
    if (!strcmp(cmd, "RUN")) {
        int i = doc["i"] | -1;
        if (i >= 0 && i < presetCount) runPresetIndex(i);
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"RUN\",\"i\":%d}\n", i);
        return;
    }
    if (!strcmp(cmd, "CYCLESRC")) {
        const char* s = doc["src"] | "presets";
        cycleUsesPresets = (strcmp(s, "types") != 0);
        presetsSaveNVS();
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"CYCLESRC\",\"src\":\"%s\"}\n",
                      cycleUsesPresets ? "presets" : "types");
        return;
    }
    if (!strcmp(cmd, "MIRROR")) {                 // start/stop the live preview stream
        mirrorOn = doc["on"] | false;
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"MIRROR\",\"on\":%s}\n", mirrorOn ? "true" : "false");
        return;
    }
    if (!strcmp(cmd, "CLEAR")) {                   // wipe all presets (used by Import-replace)
        presetCount = 0;
        presetsSaveNVS();
        Serial.println("{\"type\":\"ACK\",\"cmd\":\"CLEAR\"}");
        return;
    }
    if (!strcmp(cmd, "MASK")) {                    // switch active matrix region
        maskFull = doc["full"] | false;
        sledSetMask(maskFull);
        presetsSaveNVS();
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"MASK\",\"full\":%s}\n", maskFull ? "true" : "false");
        return;
    }
    if (!strcmp(cmd, "FLIP")) {                     // mirror to match panel mounting
        flipX = doc["x"] | flipX;
        flipY = doc["y"] | flipY;
        sledSetFlip(flipX, flipY);
        presetsSaveNVS();
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"FLIP\",\"x\":%s,\"y\":%s}\n",
                      flipX ? "true" : "false", flipY ? "true" : "false");
        return;
    }
    if (!strcmp(cmd, "AUTO")) {                     // auto-cycle on/off (30s each)
        setAutoCycle(doc["on"] | false);
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"AUTO\",\"on\":%s}\n", autoCycle ? "true" : "false");
        return;
    }
    if (!strcmp(cmd, "TEXT")) {                    // set + store a scroll-text bitmap and run it
        uint16_t cols = doc["cols"] | 0;
        if (cols > TEXT_MAX_COLS) cols = TEXT_MAX_COLS;
        const char* hex = doc["bitmap"] | "";
        size_t hlen = strlen(hex);
        static uint8_t buf[TEXT_MAX_COLS];
        for (uint16_t i = 0; i < cols; i++) {
            size_t o = (size_t)i * 2;
            buf[i] = (o + 1 < hlen) ? ((hexNib(hex[o]) << 4) | hexNib(hex[o + 1])) : 0;
        }
        uint32_t color = (((uint32_t)(uint8_t)(doc["color"][0] | 255)) << 16)
                       | (((uint32_t)(uint8_t)(doc["color"][1] | 255)) << 8)
                       |  ((uint32_t)(uint8_t)(doc["color"][2] | 255));
        bool     scroll = doc["scroll"] | true;
        uint16_t speed  = doc["speed"]  | 40;
        sled.setText(buf, cols, color, scroll);
        sled.setSpeed(speed);
        sled.setAnimation(LogicAnimation::SCROLLTEXT);
        sled.enable(true);
        textSaveNVS(buf, cols, color, scroll, speed);
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"TEXT\",\"cols\":%u}\n", cols);
        return;
    }
    Serial.println("{\"type\":\"ERR\",\"msg\":\"unknown cmd\"}");
}

// ==================== COMMAND DISPATCH ====================
// Parses ":VERB" or ":VERB,ARG1[,ARG2]" and drives the matrix. Returns true if
// recognised. Mirrors the old sled verbs onto the LogicAnimations engine, plus
// a generic :ANIM,<name> passthrough and :SPEED / :BRIGHT / :DENSITY tuning.
//
// Supported:
//   :OFF                      :COLOR,<name>
//   :NORMAL  :ALARM  :FAILURE  :RAINBOW  :FIRE
//   :ALERT (RedAlert)         :FLASH,<name>   :SWAP / :COLORSWAP   :FLIPFLOP
//   :SCAN,<name>              (Roaming/Cylon scanner)
//   :ANIM,<name>              (any LogicAnimations name; SCANNER, ROAMING, etc.)
//   :SPEED,<ms>  :BRIGHT,<0-255>  :DENSITY,<1-100>
//   :NEXT  :PREV  :MODE,<0..N>
//   Sled-ported animations (real, on the 8x32 matrix):
//     :TWINKLE[,speed]            (multicolour shimmer — speed: FAST/SLOW/TURBO/ms)
//     :PULSE,<color>[,speed]      :WIPE,<color>[,speed]
//     :CHASE,<c1>,<c2>[,speed]    :SPARKLE,<color>[,speed]
//     :HEARTBEAT,<color>[,speed]  :RAIN,<color>[,speed]
bool runSledCommand(const char* cmdIn)
{
    if (cmdIn == nullptr) return false;
    const char* src = (cmdIn[0] == ':') ? cmdIn + 1 : cmdIn;

    char buf[100];
    strncpy(buf, src, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* verb = strtok(buf, ",");
    if (verb == nullptr) return false;
    char* a1 = strtok(nullptr, ",");
    char* a2 = strtok(nullptr, ",");
    char* a3 = strtok(nullptr, ",");

    Serial.printf("[SLED] verb=%s a1=%s a2=%s a3=%s\n",
                  verb, a1 ? a1 : "-", a2 ? a2 : "-", a3 ? a3 : "-");

    // ---- direct animation verbs ----
    if      (strcasecmp(verb, "OFF")      == 0) { sled.setAnimation(LogicAnimation::LIGHTSOUT); return true; }
    else if (strcasecmp(verb, "COLOR")    == 0) { sledSolid(a1 ? a1 : "WHITE"); return true; }
    else if (strcasecmp(verb, "NORMAL")   == 0) { sled.setAnimation(LogicAnimation::NORMAL);   sled.enable(true); return true; }
    else if (strcasecmp(verb, "ALARM")    == 0) { sled.setAnimation(LogicAnimation::ALARM);    sled.enable(true); return true; }
    else if (strcasecmp(verb, "FAILURE")  == 0) { sled.setAnimation(LogicAnimation::FAILURE);  sled.enable(true); return true; }
    else if (strcasecmp(verb, "RAINBOW")  == 0) { sled.setAnimation(LogicAnimation::RAINBOW);  sled.enable(true); return true; }
    else if (strcasecmp(verb, "FIRE")     == 0) { sled.setAnimation(LogicAnimation::FIRE);     sled.enable(true); return true; }
    else if (strcasecmp(verb, "ALERT")    == 0) { sled.setAnimation(LogicAnimation::REDALERT); sled.enable(true); return true; }
    else if (strcasecmp(verb, "FLASH")    == 0) { sledColorAnim(LogicAnimation::FLASHCOLOR, a1); return true; }
    else if (strcasecmp(verb, "SWAP")     == 0 ||
             strcasecmp(verb, "COLORSWAP")== 0) { sled.setAnimation(LogicAnimation::COLORSWAP);    sled.enable(true); return true; }
    else if (strcasecmp(verb, "FLIPFLOP") == 0) { sled.setAnimation(LogicAnimation::FLIPFLOPCOLOR); sled.enable(true); return true; }
    else if (strcasecmp(verb, "SCAN")     == 0) { sledColorAnim(LogicAnimation::ROAMINGPIXEL, a1 ? a1 : "RED"); return true; }

    // ---- sled-ported animations (now real, remapped to 8x32) ----
    // TWINKLE is multicolour (ignores colour arg); a1 is an optional speed token.
    else if (strcasecmp(verb, "TWINKLE")  == 0) { sled.setSpeed(sledSpeed(a1, 60));
                                                   sled.setAnimation(LogicAnimation::TWINKLE);   sled.enable(true); return true; }
    else if (strcasecmp(verb, "PULSE")    == 0) { sled.setColor(1, a1 ? a1 : "BLUE");  sled.setSpeed(sledSpeed(a2, 20));
                                                   sled.setAnimation(LogicAnimation::PULSE);      sled.enable(true); return true; }
    else if (strcasecmp(verb, "WIPE")     == 0) { sled.setColor(1, a1 ? a1 : "WHITE"); sled.setSpeed(sledSpeed(a2, 60));
                                                   sled.setAnimation(LogicAnimation::WIPE);       sled.enable(true); return true; }
    else if (strcasecmp(verb, "CHASE")    == 0) { sled.setColor(1, a1 ? a1 : "BLUE");  sled.setColor(2, a2 ? a2 : "RED");
                                                   sled.setSpeed(sledSpeed(a3, 120));
                                                   sled.setAnimation(LogicAnimation::CHASE);      sled.enable(true); return true; }
    else if (strcasecmp(verb, "SPARKLE")  == 0) { sled.setColor(1, a1 ? a1 : "WHITE"); sled.setSpeed(sledSpeed(a2, 40));
                                                   sled.setAnimation(LogicAnimation::SPARKLE);    sled.enable(true); return true; }
    else if (strcasecmp(verb, "HEARTBEAT")== 0) { sled.setColor(1, a1 ? a1 : "RED");   sled.setSpeed(sledSpeed(a2, 10));
                                                   sled.setAnimation(LogicAnimation::HEARTBEAT);  sled.enable(true); return true; }
    else if (strcasecmp(verb, "RAIN")     == 0) { sled.setColor(1, a1 ? a1 : "GREEN"); sled.setSpeed(sledSpeed(a2, 90));
                                                   sled.setAnimation(LogicAnimation::MATRIXRAIN); sled.enable(true); return true; }

    // ---- generic passthrough + tuning ----
    else if (strcasecmp(verb, "ANIM") == 0) {
        LogicAnimation a;
        // parseAnimationName() is the free inline helper in logic_animations.h.
        if (a1 && parseAnimationName(a1, a)) { sled.setAnimation(a); sled.enable(true); return true; }
        Serial.printf("[SLED] unknown anim name: %s\n", a1 ? a1 : "-");
        return false;
    }
    else if (strcasecmp(verb, "SPEED")   == 0) { if (a1) sled.setSpeed((uint16_t)atoi(a1)); return true; }
    else if (strcasecmp(verb, "BRIGHT")  == 0) { if (a1) sled.setBrightness((uint8_t)atoi(a1)); return true; }
    else if (strcasecmp(verb, "DENSITY") == 0) { if (a1) sled.setDensity((uint8_t)atoi(a1)); return true; }
    else if (strcasecmp(verb, "NEXT")    == 0) { cycleNext(); return true; }
    else if (strcasecmp(verb, "PREV")    == 0) { cyclePrev(); return true; }
    else if (strcasecmp(verb, "MODE")    == 0) {
        int n = a1 ? atoi(a1) : 0;
        if (n < 0 || n >= CYCLE_COUNT) return false;
        applyAnimIndex(n);
        return true;
    }
    // Run a saved preset by index (e.g. ":RUN,3"), reachable over ESP-NOW too.
    else if (strcasecmp(verb, "RUN")     == 0) {
        int n = a1 ? atoi(a1) : -1;
        if (n < 0 || n >= presetCount) return false;
        runPresetIndex(n);
        return true;
    }
    // Switch what the button / :NEXT cycles: ":CYCLESRC,presets" or ":CYCLESRC,types".
    else if (strcasecmp(verb, "CYCLESRC") == 0) {
        cycleUsesPresets = !(a1 && strcasecmp(a1, "types") == 0);
        presetsSaveNVS();
        return true;
    }
    // Switch the active matrix region: ":MASK,FULL" or ":MASK,SLED".
    else if (strcasecmp(verb, "MASK") == 0) {
        maskFull = (a1 && strcasecmp(a1, "full") == 0);
        sledSetMask(maskFull);
        presetsSaveNVS();
        return true;
    }
    // Auto-cycle: ":AUTO,ON" / ":AUTO,OFF" / ":AUTO" (toggle).
    else if (strcasecmp(verb, "AUTO") == 0) {
        bool on = !autoCycle;
        if (a1 && strcasecmp(a1, "on")  == 0) on = true;
        if (a1 && strcasecmp(a1, "off") == 0) on = false;
        setAutoCycle(on);
        return true;
    }

    Serial.printf("[SLED] Unknown verb: %s\n", verb);
    return false;
}

// ==================== ESP-NOW RECEIVE CALLBACK ====================
void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len)
{
    if (len < (int)sizeof(espnow_struct_message)) return;

    espnow_struct_message msg;
    memcpy(&msg, incomingData, sizeof(msg));

    // Password gate — silently drop foreign-network packets
    if (strncmp(msg.structPassword, ESPNOWPASSWORD.c_str(), sizeof(msg.structPassword)) != 0) return;

    int senderIdx = etmBoardIndexFromMAC(info->src_addr);
    if (senderIdx < 0) return;   // unknown peer

    // Any inbound packet refreshes the sender's heartbeat timestamp
    etmHandleHeartbeat(senderIdx);

    switch (msg.structPacketType)
    {
        case PACKET_TYPE_HEARTBEAT:
            return;

        case PACKET_TYPE_ACK:
            etmProcessAck(senderIdx, msg.structSequenceNumber);
            return;

        case PACKET_TYPE_COMMAND:
        {
            bool forUs       = (strncmp(msg.structTargetID, "SL", 2) == 0);
            bool isBroadcast = (strncmp(msg.structTargetID, "BR", 2) == 0);
            if (!forUs && !isBroadcast) return;

            if (msg.structCommandIncluded)
            {
                Serial.printf("[ESP-NOW] %s -> %s : %s (seq %u)\n",
                              ETM_BOARD_IDS[senderIdx],
                              msg.structTargetID,
                              msg.structCommand,
                              msg.structSequenceNumber);
                runSledCommand(msg.structCommand);
            }

            if (forUs)
            {
                etmSendAck(senderIdx, msg.structSequenceNumber);
            }
            return;
        }
    }
}

// ==================== SETUP ====================
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== Sled Controller (ESP32-S3 / 8x32 Matrix) Starting ===");

    randomSeed(esp_random());   // vary random-based animations between boots

    // Matrix engine + saved presets.
    sled.begin();
    sled.setBrightness(120);
    presetsLoadNVS();
    textLoadNVS();              // restore any saved scroll-text bitmap
    if (cycleUsesPresets && presetCount > 0) runPresetIndex(0);  // boot to first saved preset
    else                                     applyAnimIndex(0);   // else NORMAL (R2 blink)

    // Button + status LEDs
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    for (int i = 0; i < LED_COUNT; i++)
    {
        pinMode(LEDS[i], OUTPUT);
        digitalWrite(LEDS[i], LOW);
    }

    // ===== ESP-NOW (ETM_Droid) =====
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();   // Pure ESP-NOW — no AP/STA join needed
    esp_wifi_set_mac(WIFI_IF_STA, (uint8_t*)ETM_BOARD_MACS[ETM_BOARD_SL]);
    Serial.print("[ESP-NOW] MAC: "); Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESP-NOW] init FAILED");
    }
    else
    {
        esp_now_register_recv_cb(OnDataRecv);

        esp_now_peer_info_t peerInfo = {};
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        memcpy(peerInfo.peer_addr, ETM_BROADCAST_MAC, 6);
        esp_now_add_peer(&peerInfo);

        for (int i = 0; i < ETM_NUM_BOARDS; i++)
        {
            if (i == ETM_BOARD_SL) continue;
            memcpy(peerInfo.peer_addr, ETM_BOARD_MACS[i], 6);
            esp_now_add_peer(&peerInfo);
        }

        etmInit(ESPNOWPASSWORD.c_str(), ETM_BOARD_SL);
        Serial.println("[ESP-NOW] Ready as SL");
    }

    Serial.println("Setup complete. Press button to cycle, or send :E SL <cmd> from the webpage.");
}

// ==================== USB SERIAL COMMAND INPUT ====================
// Drive the sled directly from a USB serial monitor at 115200, e.g.:
//   :RAINBOW    :FIRE    :SCAN,RED    :COLOR,BLUE    :MODE,3    :SPEED,40
// Sized for the editor JSON — a TEXT command with a 256-col bitmap is ~600 chars.
static char     serialCmdBuf[1200];
static uint16_t serialCmdLen = 0;

static void serialPoll()
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n')
        {
            if (serialCmdLen > 0)
            {
                serialCmdBuf[serialCmdLen] = '\0';
                if (serialCmdBuf[0] == '{') {
                    // JSON line — preset-editor protocol.
                    handleStudioJson(String(serialCmdBuf));
                } else {
                    Serial.printf("[USB] received: %s\n", serialCmdBuf);
                    bool ok = runSledCommand(serialCmdBuf);
                    if (!ok) Serial.printf("[USB] command rejected\n");
                }
                serialCmdLen = 0;
            }
        }
        else if (serialCmdLen < sizeof(serialCmdBuf) - 1)
        {
            serialCmdBuf[serialCmdLen++] = c;
        }
        else
        {
            Serial.println("[USB] line too long, discarded");
            serialCmdLen = 0;
        }
    }
}

// ==================== MAIN LOOP ====================
void loop()
{
    // Drive ETM heartbeats / retries / offline detection
    etmProcess();

    // Drain any pending USB serial commands
    serialPoll();

    // Update the matrix animation (non-blocking, self-paced)
    sled.update();

    // Stream the frame to the editor's on-screen preview (when enabled).
    if (mirrorOn && (millis() - mirrorLastMs) >= MIRROR_INTERVAL_MS) {
        mirrorLastMs = millis();
        sled.emitMirrorFrame(Serial);
    }

    // ===== AUTO-CYCLE ADVANCE =====
    if (autoCycle && (millis() - autoCycleLastMs) >= AUTO_CYCLE_INTERVAL_MS) {
        autoCycleLastMs = millis();
        cycleNext();
    }

    // ===== BUTTON DEBOUNCING (count taps) =====
    int reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState)
    {
        lastDebounceTime = millis();
        lastButtonState = reading;
    }
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        if (reading != stableButtonState)
        {
            stableButtonState = reading;
            if (stableButtonState == LOW && !buttonPressed)   // press detected
            {
                buttonPressed = true;
                tapCount++;
                lastTapMs = millis();
            }
            else if (stableButtonState == HIGH)
            {
                buttonPressed = false;
            }
        }
    }

    // ===== RESOLVE TAPS once the multi-tap window closes =====
    if (tapCount > 0 && (millis() - lastTapMs) > MULTITAP_WINDOW_MS)
    {
        if (tapCount >= 2) {
            setAutoCycle(!autoCycle);   // double tap → toggle auto-cycle
        } else {
            // Single tap → manual advance (and reset the auto timer if running)
            Serial.printf("Button: advancing from anim %d\n", animIndex);
            cycleNext();
            if (autoCycle) autoCycleLastMs = millis();
        }
        tapCount = 0;
    }

    // ===== RANDOM STATUS-LED BLINKING =====
    currentMillis = millis();
    if (currentMillis - blinkpreviousMillis >= blinkinterval)
    {
        blinkpreviousMillis = currentMillis;
        randomNumberArray = random(0, LED_COUNT);
        digitalWrite(LEDS[randomNumberArray], HIGH);
        randomNumberArray = random(0, LED_COUNT);
        digitalWrite(LEDS[randomNumberArray], LOW);
    }
}
