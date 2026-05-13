#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  rc_config.h — RC button mapping configuration
//  Structs, defaults pre-populated from current Body Controller mappings,
//  NVS load/save, and JSON serialisation helpers.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Action types
// ─────────────────────────────────────────────────────────────────────────────
enum RcActionType : uint8_t {
  RA_NONE   = 0,
  RA_ESPNOW = 1,   // sendESPNOWCommand(target, cmd)
  RA_HCR    = 2,   // HCRFunction(fn, chan, track)
  RA_ANIM   = 3,   // callAnimation(fn)   — fn matches :A## animation table
};

struct RcAction {
  RcActionType type;
  char         target[6];    // RA_ESPNOW: board ID (BS/DC/DP/HP/DG)
  char         cmd[32];      // RA_ESPNOW: command string (longest default is 22 chars)
  int8_t       hcrFn;        // RA_HCR: HCRFunction() case number
  int8_t       hcrChan;      // RA_HCR: channel (= emotion for Stimulate/Trigger/SetEmotion)
  int16_t      hcrTrack;     // RA_HCR: track/value
  uint8_t      animFn;       // RA_ANIM: animation function index (1–40)
  uint16_t     delayMs;      // optional delay before this specific action
  char         note[20];     // Optional human-readable comment (GUI-displayed,
                             // 19 visible chars + null) — kept small to fit
                             // the ESP32 DRAM budget across all action slots.
};

#define RC_ACTIONS_PER_TIER  5   // max simultaneous actions per tap tier
                                 // (capped to fit ESP32 DRAM — 5 actions × 3 tiers
                                 //  × 57 mappings × ~60 B = ~50 KB)

struct RcTier {
  uint8_t  count;
  RcAction a[RC_ACTIONS_PER_TIER];
};

struct RcMapping {
  bool    exclusive;  // true = EXCLUSIVE mode (tap2 replaces tap1 within window)
  RcTier  t[3];       // t[0]=1-tap  t[1]=2-tap  t[2]=3-tap
};

// ─────────────────────────────────────────────────────────────────────────────
//  PWM threshold for each of the 19 virtual channel-6 buttons
// ─────────────────────────────────────────────────────────────────────────────
struct RcThreshold {
  int  id;          // 1–19
  char label[24];
  int  minPwm;
  int  maxPwm;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Configurable toggle switches SA-SH
//    index: 0=SA 1=SB 2=SC 3=SD 4=SE 5=SF 6=SG 7=SH
//    positions = 2 (down/up) or 3 (down/mid/up)
//    On position change, the matching RcTier fires through rcExecuteAction.
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_SWITCHES 8

struct RcSwitch {
  int     channel;     // SBUS channel 1-24; 0 = disabled
  uint8_t positions;   // 2 or 3
  RcTier  t[3];        // [down, mid, up] — for 2-pos switches t[1] unused
};

static const char* RC_SWITCH_LABELS[RC_NUM_SWITCHES]   = {"SA","SB","SC","SD","SE","SF","SG","SH"};
//  New default channel layout (post-Kyber alignment): SA-SH on CH8-CH15 sequentially.
static const int   RC_SWITCH_DEFAULT_CH[RC_NUM_SWITCHES] = {  8,   9,  10,  11,  12,  13,  14,  15 };
static const uint8_t RC_SWITCH_DEFAULT_POS[RC_NUM_SWITCHES] = { 3,   3,   3,   3,   3,   2,   3,   2 };

// Switch index constants for clarity in code that binds functions to switches
#define SW_SA 0
#define SW_SB 1
#define SW_SC 2
#define SW_SD 3
#define SW_SE 4
#define SW_SF 5
#define SW_SG 6
#define SW_SH 7

// ─────────────────────────────────────────────────────────────────────────────
//  Configurable analog knobs S1 / S2
//  Each knob reads a continuous SBUS value (172-1811) and applies it to a
//  function (volume, etc.).  Defaults match the original BC firmware:
//    S1 → CH4 → HCR Vocalizer Volume
//    S2 → CH5 → HCR WAV Volume
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_KNOBS 2

enum RcKnobFunction : uint8_t {
  KF_NONE          = 0,
  KF_HCR_VOC_VOL   = 1,   // HCR Vocalizer volume (chan 0)
  KF_HCR_WAV_VOL   = 2,   // HCR WAV volume       (chan 1)
};

struct RcKnob {
  int     channel;       // SBUS channel 1-24; 0 = disabled
  uint8_t function;      // RcKnobFunction
};

static const char* RC_KNOB_LABELS[RC_NUM_KNOBS]   = {"S1","S2"};
//  New default channel layout: S1 on CH5, S2 on CH6 (CH4 is now empty/reserved).
static const int   RC_KNOB_DEFAULT_CH[RC_NUM_KNOBS] = {  5,   6 };
static const uint8_t RC_KNOB_DEFAULT_FN[RC_NUM_KNOBS] = {KF_HCR_VOC_VOL, KF_HCR_WAV_VOL};

// ─────────────────────────────────────────────────────────────────────────────
//  Legacy function bindings — which SWITCH (by index) drives each hardcoded
//  helper.  This lets switches move channels freely without breaking the
//  function calls.  Set to -1 to disable a function.
//  Defaults preserve the original X18 wiring:
//     SE → mode/function selector
//     SB → RADH Auto toggle
//     SF → Maintenance lights
//     SC → Lights mode
//     SD → HCR Muse mode
// ─────────────────────────────────────────────────────────────────────────────
struct RcFuncBindings {
  int8_t modeSwitch;        // which switch drives the 1/2/3 mode selector
  int8_t radhAutoSwitch;    // which switch drives RC_RADH_Auto()
  int8_t maintLightsSwitch; // which switch drives maintLights()
  int8_t lightsModeSwitch;  // which switch drives lightsMode()
  int8_t museModeSwitch;    // which switch drives updateMuse()
};

// ─────────────────────────────────────────────────────────────────────────────
//  Full runtime config block
//  buttonId  =  (mode * 100) + btn   e.g. mode=1 btn=5 → 105
//  flat index = (mode-1)*19 + (btn-1)  → 0–56
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_THRESHOLDS 19
#define RC_NUM_MAPPINGS   57   // 3 modes × 19 buttons

inline int rcMapIndex(int mode, int btn) { return (mode - 1) * 19 + (btn - 1); }

struct RcConfig {
  int            tapWindowMs;
  int            matrixChannel;   // SBUS channel carrying the multiplexed button matrix (default 7)
  RcThreshold    thresholds[RC_NUM_THRESHOLDS];
  RcMapping      mappings[RC_NUM_MAPPINGS];
  RcSwitch       switches[RC_NUM_SWITCHES];
  RcKnob         knobs[RC_NUM_KNOBS];
  RcFuncBindings funcBindings;     // function-to-switch bindings
};

// Defined in the .ino
extern RcConfig rcConfig;

// ─────────────────────────────────────────────────────────────────────────────
//  Construction helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline RcAction makeESPNOW(const char* target, const char* cmd,
                                  uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_ESPNOW;
  strlcpy(a.target, target, sizeof(a.target));
  strlcpy(a.cmd,    cmd,    sizeof(a.cmd));
  strlcpy(a.note,   note,   sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeHCR(int8_t fn, int8_t chan, int16_t track,
                               uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type     = RA_HCR;
  a.hcrFn    = fn;
  a.hcrChan  = chan;
  a.hcrTrack = track;
  a.delayMs  = delayMs;
  strlcpy(a.note, note, sizeof(a.note));
  return a;
}
static inline RcAction makeAnim(uint8_t fn, uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type    = RA_ANIM;
  a.animFn  = fn;
  a.delayMs = delayMs;
  strlcpy(a.note, note, sizeof(a.note));
  return a;
}
static inline void setTier1(RcTier& tier, RcAction a0) {
  tier.count = 1;  tier.a[0] = a0;
}
static inline void setTier2(RcTier& tier, RcAction a0, RcAction a1) {
  tier.count = 2;  tier.a[0] = a0;  tier.a[1] = a1;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Load factory defaults (all current Body Controller mappings)
// ─────────────────────────────────────────────────────────────────────────────
void rcConfigLoadDefaults() {
  rcConfig.tapWindowMs   = 500;
  rcConfig.matrixChannel = 7;   // Kyber-aligned: button matrix lives on CH7

  // Default function-to-switch bindings (preserves original X18 semantics
  // regardless of which channels the switches are actually wired to).
  rcConfig.funcBindings.modeSwitch        = SW_SE;
  rcConfig.funcBindings.radhAutoSwitch    = SW_SB;
  rcConfig.funcBindings.maintLightsSwitch = SW_SF;
  rcConfig.funcBindings.lightsModeSwitch  = SW_SC;
  rcConfig.funcBindings.museModeSwitch    = SW_SD;

  // Default PWM threshold bands (button 1 = highest PWM, button 19 = lowest)
  // Order matches the FrSky X18 model: function buttons B1-B6 first, then
  // trim pairs T4/T5/T3/T2/T6/T1, each as Left/Right.  T1 keeps narrowed bands
  // around SBUS_CENTER 992 so the "no button pressed" state lands in a dead zone.
  struct { const char* label; int mn; int mx; } bands[19] = {
    { "B1",          1800, 1850 },
    { "B2",          1750, 1799 },
    { "B3",          1700, 1749 },
    { "B4",          1650, 1699 },
    { "B5",          1600, 1649 },
    { "B6",          1550, 1599 },
    { "T4 Left",     1500, 1549 },
    { "T4 Right",    1450, 1499 },
    { "T5 Left",     1400, 1449 },
    { "T5 Right",    1350, 1399 },
    { "T3 Left",     1300, 1349 },
    { "T3 Right",    1250, 1299 },
    { "T2 Left",     1200, 1249 },
    { "T2 Right",    1150, 1199 },
    { "T6 Left",     1100, 1149 },
    { "T6 Right",    1050, 1099 },
    { "T1 Left",     1023, 1049 },   // narrowed so SBUS_CENTER 992 ± 30 = dead zone
    { "T1 Right",     850,  960 },   // narrowed so SBUS_CENTER 992 ± 30 = dead zone
    { "Unassigned",   800,  849 },
  };
  for (int i = 0; i < 19; i++) {
    rcConfig.thresholds[i].id = i + 1;
    strlcpy(rcConfig.thresholds[i].label, bands[i].label, 24);
    rcConfig.thresholds[i].minPwm = bands[i].mn;
    rcConfig.thresholds[i].maxPwm = bands[i].mx;
  }

  memset(rcConfig.mappings, 0, sizeof(rcConfig.mappings));

  // ── MODE 1 — switch DOWN ──────────────────────────────────────────────────
  setTier2(rcConfig.mappings[rcMapIndex(1,1)].t[0],
           makeHCR(16,1,0, 0,"Stop WAV"),
           makeHCR(9,0,0,  0,"Stop emote"));                                                // B1
  setTier1(rcConfig.mappings[rcMapIndex(1,2)].t[0],  makeHCR(4,0,1, 0,"Happy"));            // B2
  setTier1(rcConfig.mappings[rcMapIndex(1,3)].t[0],  makeHCR(4,1,1, 0,"Sad"));              // B3
  setTier1(rcConfig.mappings[rcMapIndex(1,4)].t[0],  makeHCR(4,2,1, 0,"Mad"));              // B4
  setTier1(rcConfig.mappings[rcMapIndex(1,5)].t[0],  makeHCR(4,3,1, 0,"Scared"));           // B5
  setTier1(rcConfig.mappings[rcMapIndex(1,6)].t[0],  makeHCR(14,1,4, 0,"Fart"));            // B6
  setTier1(rcConfig.mappings[rcMapIndex(1,7)].t[0],  makeAnim(11, 0,"All Open"));           // T4 Left
  setTier2(rcConfig.mappings[rcMapIndex(1,8)].t[0],
           makeAnim(7, 0,"All Close"),
           makeESPNOW("DP",":SUS:PH", 0,"Periscope home"));                                 // T4 Right
  setTier1(rcConfig.mappings[rcMapIndex(1,9)].t[0],  makeAnim(22, 0,"Wave Utility Arm"));   // T5 Left
  setTier1(rcConfig.mappings[rcMapIndex(1,10)].t[0], makeAnim(12, 0,"Drawer Wave"));        // T5 Right
  setTier1(rcConfig.mappings[rcMapIndex(1,11)].t[0], makeAnim(6,  0,"Harlem Shake"));       // T3 Left
  setTier1(rcConfig.mappings[rcMapIndex(1,12)].t[0], makeAnim(2,  0,"Panel Wave"));         // T3 Right
  setTier1(rcConfig.mappings[rcMapIndex(1,13)].t[0], makeESPNOW("DP",":SUS:PS4", 0,"Periscope Seq 4")); // T2 Left
  setTier1(rcConfig.mappings[rcMapIndex(1,14)].t[0], makeAnim(14, 0,"Short Circuit"));      // T2 Right
  setTier1(rcConfig.mappings[rcMapIndex(1,15)].t[0], makeAnim(13, 0,"Open/Close Easing"));  // T6 Left
  setTier1(rcConfig.mappings[rcMapIndex(1,16)].t[0], makeAnim(3,  0,"Panel Wave Fast"));    // T6 Right
  setTier2(rcConfig.mappings[rcMapIndex(1,17)].t[0],
           makeESPNOW("DP",":SUS:PP100:L0", 0,"Peri up + lights"),
           makeESPNOW("BS",":D320",         0,"Display mode"));                             // T1 Left
  setTier1(rcConfig.mappings[rcMapIndex(1,18)].t[0], makeESPNOW("BS",":D312", 0,"Open/Close Wave")); // T1 Right

  // ── MODE 2 — switch MIDDLE ────────────────────────────────────────────────
  setTier2(rcConfig.mappings[rcMapIndex(2,1)].t[0],
           makeHCR(16,1,0, 0,"Stop WAV"),
           makeHCR(9,0,0,  0,"Stop emote"));                                                // B1
  setTier1(rcConfig.mappings[rcMapIndex(2,2)].t[0],  makeHCR(14,1,204, 0,"SW Dance"));      // B2
  setTier1(rcConfig.mappings[rcMapIndex(2,3)].t[0],  makeHCR(14,1,3,   0,"Leia long"));     // B3
  setTier1(rcConfig.mappings[rcMapIndex(2,4)].t[0],  makeHCR(14,1,1,   0,"SW theme"));      // B4
  setTier1(rcConfig.mappings[rcMapIndex(2,5)].t[0],  makeHCR(14,1,2,   0,"Vader theme"));   // B5
  setTier1(rcConfig.mappings[rcMapIndex(2,6)].t[0],  makeHCR(14,1,203, 0,"Cantina"));       // B6
  setTier1(rcConfig.mappings[rcMapIndex(2,7)].t[0],  makeAnim(8,  0,"All Flutter"));        // T4 Left
  setTier1(rcConfig.mappings[rcMapIndex(2,8)].t[0],  makeAnim(25, 0,"Smoke Sequence"));     // T4 Right
  setTier1(rcConfig.mappings[rcMapIndex(2,9)].t[0],  makeESPNOW("BS",":D126", 0,"CPU sequence"));            // T5 Left
  // slot 10 (T5 Right) — empty in original
  setTier1(rcConfig.mappings[rcMapIndex(2,11)].t[0], makeESPNOW("DP",":SUS:PP100:L0", 0,"Peri up + lights")); // T3 Left
  setTier1(rcConfig.mappings[rcMapIndex(2,12)].t[0], makeESPNOW("DP",":SUS:PH",       0,"Periscope home"));   // T3 Right
  setTier1(rcConfig.mappings[rcMapIndex(2,13)].t[0], makeESPNOW("DP",":SUS:PAR,50",   0,"Periscope rotate")); // T2 Left
  setTier1(rcConfig.mappings[rcMapIndex(2,14)].t[0], makeESPNOW("DP",":SUS:PS10",     0,"Periscope Seq 10")); // T2 Right
  setTier1(rcConfig.mappings[rcMapIndex(2,15)].t[0], makeESPNOW("BS",":D315",         0,"Long dance"));       // T6 Left
  // slot 16 (T6 Right) — empty in original
  setTier1(rcConfig.mappings[rcMapIndex(2,17)].t[0], makeESPNOW("DP",":A70",          0,"Fire extinguisher")); // T1 Left
  setTier1(rcConfig.mappings[rcMapIndex(2,18)].t[0], makeESPNOW("DP",":A61",          0,"Arm saber"));         // T1 Right

  // ── MODE 3 — switch UP ────────────────────────────────────────────────────
  setTier2(rcConfig.mappings[rcMapIndex(3,1)].t[0],
           makeHCR(16,1,0, 0,"Stop WAV"),
           makeHCR(9,0,0,  0,"Stop emote"));                                                // B1
  setTier1(rcConfig.mappings[rcMapIndex(3,2)].t[0],  makeHCR(14,1,8,   0,"Whistle"));       // B2
  setTier1(rcConfig.mappings[rcMapIndex(3,3)].t[0],  makeHCR(14,1,201, 0,"Staying Alive")); // B3
  setTier1(rcConfig.mappings[rcMapIndex(3,4)].t[0],  makeHCR(14,1,100, 0,"WAV 100"));       // B4
  setTier1(rcConfig.mappings[rcMapIndex(3,5)].t[0],  makeHCR(14,1,101, 0,"WAV 101"));       // B5
  setTier1(rcConfig.mappings[rcMapIndex(3,6)].t[0],  makeHCR(14,1,202, 0,"WAV 202"));       // B6
  setTier2(rcConfig.mappings[rcMapIndex(3,7)].t[0],
           makeESPNOW("BS",":D10103", 0,    "CPU raise start"),
           makeESPNOW("BS",":D121",   500,  "CPU raise body"));                             // T4 Left
  setTier2(rcConfig.mappings[rcMapIndex(3,8)].t[0],
           makeESPNOW("BS",":D122",   0,    "CPU lower start"),
           makeESPNOW("BS",":D10203", 1000, "CPU lower body"));                             // T4 Right
  setTier1(rcConfig.mappings[rcMapIndex(3,9)].t[0],  makeESPNOW("BS",":D123",   0,"CPU extend"));    // T5 Left
  setTier1(rcConfig.mappings[rcMapIndex(3,10)].t[0], makeESPNOW("BS",":D124",   0,"CPU retract"));   // T5 Right
  setTier1(rcConfig.mappings[rcMapIndex(3,11)].t[0], makeESPNOW("BS",":D126",   0,"CPU sequence"));  // T3 Left
  setTier1(rcConfig.mappings[rcMapIndex(3,12)].t[0], makeESPNOW("BS",":D125",   0,"CPU rotate"));    // T3 Right
  setTier2(rcConfig.mappings[rcMapIndex(3,17)].t[0],
           makeESPNOW("BS",":D10104", 0,    "Stow saber start"),
           makeESPNOW("BS",":D10204", 7050, "Stow saber finish"));                          // T1 Left
  setTier1(rcConfig.mappings[rcMapIndex(3,18)].t[0], makeESPNOW("DP",":A60",    0,"Launch saber")); // T1 Right

  // ── Default switch channel / position assignments (all actions empty) ──
  memset(rcConfig.switches, 0, sizeof(rcConfig.switches));
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    rcConfig.switches[i].channel   = RC_SWITCH_DEFAULT_CH[i];
    rcConfig.switches[i].positions = RC_SWITCH_DEFAULT_POS[i];
  }

  // ── Default knob assignments  (S1→CH4 HCR Voc Vol, S2→CH5 HCR WAV Vol) ──
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    rcConfig.knobs[i].channel  = RC_KNOB_DEFAULT_CH[i];
    rcConfig.knobs[i].function = RC_KNOB_DEFAULT_FN[i];
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  JSON helpers — action ↔ JSON object
// ─────────────────────────────────────────────────────────────────────────────
static void actionToJson(const RcAction& a, JsonObject obj) {
  switch (a.type) {
    case RA_ESPNOW:
      obj["type"]   = "espnow";
      obj["target"] = a.target;
      obj["cmd"]    = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_HCR:
      obj["type"]  = "hcr";
      obj["fn"]    = a.hcrFn;
      obj["chan"]  = a.hcrChan;
      obj["track"] = a.hcrTrack;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_ANIM:
      obj["type"] = "anim";
      obj["fn"]   = a.animFn;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    default: break;
  }
  // Note is shared across all action types — only emit if non-empty to keep
  // JSON payload small.
  if (a.note[0]) obj["note"] = a.note;
}

static bool actionFromJson(const JsonObject& obj, RcAction& a) {
  memset(&a, 0, sizeof(a));
  const char* type = obj["type"] | "";
  bool ok = false;
  if (strcmp(type, "espnow") == 0) {
    a.type = RA_ESPNOW;
    strlcpy(a.target, obj["target"] | "", sizeof(a.target));
    strlcpy(a.cmd,    obj["cmd"]    | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "hcr") == 0) {
    a.type     = RA_HCR;
    a.hcrFn    = obj["fn"]    | 0;
    a.hcrChan  = obj["chan"]  | 0;
    a.hcrTrack = obj["track"] | 0;
    a.delayMs  = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "anim") == 0) {
    a.type    = RA_ANIM;
    a.animFn  = obj["fn"] | 0;
    a.delayMs = obj["delay"] | 0;
    ok = true;
  }
  if (ok) strlcpy(a.note, obj["note"] | "", sizeof(a.note));
  return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Serialise full config to JSON string (for GET_CONFIG WebSerial response)
//  Sparse — only non-empty mappings are included.
// ─────────────────────────────────────────────────────────────────────────────
String rcConfigToJSON() {
  // Use DynamicJsonDocument — generous size for 57 possible mappings
  // 10 KB is comfortable for a fully-populated default config
  DynamicJsonDocument doc(20480);

  doc["tapWindowMs"]   = rcConfig.tapWindowMs;
  doc["matrixChannel"] = rcConfig.matrixChannel;

  // Function-to-switch bindings
  JsonObject fb = doc.createNestedObject("funcBindings");
  fb["mode"]       = rcConfig.funcBindings.modeSwitch;
  fb["radhAuto"]   = rcConfig.funcBindings.radhAutoSwitch;
  fb["maintLights"]= rcConfig.funcBindings.maintLightsSwitch;
  fb["lightsMode"] = rcConfig.funcBindings.lightsModeSwitch;
  fb["museMode"]   = rcConfig.funcBindings.museModeSwitch;

  JsonArray thArr = doc.createNestedArray("thresholds");
  for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
    JsonObject th = thArr.createNestedObject();
    th["id"]     = rcConfig.thresholds[i].id;
    th["label"]  = rcConfig.thresholds[i].label;
    th["minPwm"] = rcConfig.thresholds[i].minPwm;
    th["maxPwm"] = rcConfig.thresholds[i].maxPwm;
  }

  JsonObject mapObj = doc.createNestedObject("mappings");
  for (int mode = 1; mode <= 3; mode++) {
    for (int btn = 1; btn <= 19; btn++) {
      int idx = rcMapIndex(mode, btn);
      const RcMapping& m = rcConfig.mappings[idx];

      // Skip completely empty mappings
      bool hasAny = false;
      for (int ti = 0; ti < 3 && !hasAny; ti++)
        if (m.t[ti].count > 0) hasAny = true;
      if (!hasAny && !m.exclusive) continue;

      String key = String(mode * 100 + btn);
      JsonObject mObj = mapObj.createNestedObject(key);
      mObj["exclusive"] = m.exclusive;

      for (int ti = 0; ti < 3; ti++) {
        if (m.t[ti].count == 0) continue;
        String tierKey = String("t") + (ti + 1);
        JsonArray acts = mObj.createNestedArray(tierKey);
        for (int ai = 0; ai < m.t[ti].count; ai++) {
          actionToJson(m.t[ti].a[ai], acts.createNestedObject());
        }
      }
    }
  }

  // ── Switches SA-SH ──
  JsonObject swObj = doc.createNestedObject("switches");
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    const RcSwitch& s = rcConfig.switches[i];
    JsonObject sObj = swObj.createNestedObject(RC_SWITCH_LABELS[i]);
    sObj["channel"]   = s.channel;
    sObj["positions"] = s.positions;
    for (int pi = 0; pi < 3; pi++) {
      if (s.t[pi].count == 0) continue;
      String tierKey = String("p") + pi;   // p0=down p1=mid p2=up
      JsonArray acts = sObj.createNestedArray(tierKey);
      for (int ai = 0; ai < s.t[pi].count; ai++) {
        actionToJson(s.t[pi].a[ai], acts.createNestedObject());
      }
    }
  }

  // ── Knobs S1, S2 ──
  JsonObject knObj = doc.createNestedObject("knobs");
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    JsonObject kObj = knObj.createNestedObject(RC_KNOB_LABELS[i]);
    kObj["channel"]  = rcConfig.knobs[i].channel;
    kObj["function"] = rcConfig.knobs[i].function;
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Load config from JSON string (from SET_CONFIG WebSerial message)
// ─────────────────────────────────────────────────────────────────────────────
bool rcConfigFromJSON(const String& json) {
  DynamicJsonDocument doc(16384);
  if (deserializeJson(doc, json) != DeserializationError::Ok) return false;

  if (doc.containsKey("tapWindowMs"))
    rcConfig.tapWindowMs = doc["tapWindowMs"];

  if (doc.containsKey("matrixChannel"))
    rcConfig.matrixChannel = doc["matrixChannel"];

  if (doc.containsKey("funcBindings")) {
    JsonObject fb = doc["funcBindings"];
    rcConfig.funcBindings.modeSwitch        = fb["mode"]        | rcConfig.funcBindings.modeSwitch;
    rcConfig.funcBindings.radhAutoSwitch    = fb["radhAuto"]    | rcConfig.funcBindings.radhAutoSwitch;
    rcConfig.funcBindings.maintLightsSwitch = fb["maintLights"] | rcConfig.funcBindings.maintLightsSwitch;
    rcConfig.funcBindings.lightsModeSwitch  = fb["lightsMode"]  | rcConfig.funcBindings.lightsModeSwitch;
    rcConfig.funcBindings.museModeSwitch    = fb["museMode"]    | rcConfig.funcBindings.museModeSwitch;
  }

  if (doc.containsKey("thresholds")) {
    JsonArray thArr = doc["thresholds"];
    int i = 0;
    for (JsonObject th : thArr) {
      if (i >= RC_NUM_THRESHOLDS) break;
      rcConfig.thresholds[i].id = th["id"] | (i + 1);
      strlcpy(rcConfig.thresholds[i].label, th["label"] | "", 24);
      rcConfig.thresholds[i].minPwm = th["minPwm"] | 0;
      rcConfig.thresholds[i].maxPwm = th["maxPwm"] | 0;
      i++;
    }
  }

  if (doc.containsKey("mappings")) {
    JsonObject mapObj = doc["mappings"];
    for (JsonPair kv : mapObj) {
      int buttonId = String(kv.key().c_str()).toInt();
      int mode = buttonId / 100;
      int btn  = buttonId % 100;
      if (mode < 1 || mode > 3 || btn < 1 || btn > 19) continue;
      int idx = rcMapIndex(mode, btn);

      RcMapping& m = rcConfig.mappings[idx];
      memset(&m, 0, sizeof(m));

      JsonObject mObj = kv.value().as<JsonObject>();
      m.exclusive = mObj["exclusive"] | false;

      for (int ti = 0; ti < 3; ti++) {
        String tierKey = String("t") + (ti + 1);
        if (!mObj.containsKey(tierKey)) continue;
        JsonArray acts = mObj[tierKey];
        int ai = 0;
        for (JsonObject aObj : acts) {
          if (ai >= RC_ACTIONS_PER_TIER) break;
          if (actionFromJson(aObj, m.t[ti].a[ai])) {
            m.t[ti].count = ++ai;
          }
        }
      }
    }
  }

  // ── Switches ──
  if (doc.containsKey("switches")) {
    JsonObject swObj = doc["switches"];
    for (int i = 0; i < RC_NUM_SWITCHES; i++) {
      if (!swObj.containsKey(RC_SWITCH_LABELS[i])) continue;
      JsonObject sObj = swObj[RC_SWITCH_LABELS[i]];
      RcSwitch& s = rcConfig.switches[i];
      memset(s.t, 0, sizeof(s.t));
      s.channel   = sObj["channel"]   | RC_SWITCH_DEFAULT_CH[i];
      s.positions = sObj["positions"] | RC_SWITCH_DEFAULT_POS[i];
      for (int pi = 0; pi < 3; pi++) {
        String tierKey = String("p") + pi;
        if (!sObj.containsKey(tierKey)) continue;
        JsonArray acts = sObj[tierKey];
        int ai = 0;
        for (JsonObject aObj : acts) {
          if (ai >= RC_ACTIONS_PER_TIER) break;
          if (actionFromJson(aObj, s.t[pi].a[ai])) s.t[pi].count = ++ai;
        }
      }
    }
  }

  // ── Knobs ──
  if (doc.containsKey("knobs")) {
    JsonObject knObj = doc["knobs"];
    for (int i = 0; i < RC_NUM_KNOBS; i++) {
      if (!knObj.containsKey(RC_KNOB_LABELS[i])) continue;
      JsonObject kObj = knObj[RC_KNOB_LABELS[i]];
      rcConfig.knobs[i].channel  = kObj["channel"]  | RC_KNOB_DEFAULT_CH[i];
      rcConfig.knobs[i].function = kObj["function"] | RC_KNOB_DEFAULT_FN[i];
    }
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  NVS persistence  (namespace "rcfg")
//  Split across keys to stay within NVS 4000-byte per-value limit:
//    "cfg"  — tapWindowMs
//    "th"   — thresholds JSON
//    "m1"   — mode-1 mappings JSON  (keys "101"–"119")
//    "m2"   — mode-2 mappings JSON  (keys "201"–"219")
//    "m3"   — mode-3 mappings JSON  (keys "301"–"319")
// ─────────────────────────────────────────────────────────────────────────────
void rcConfigSaveNVS() {
  Preferences prefs;
  prefs.begin("rcfg", false);

  // tap window + matrix channel + function bindings (compact small fields)
  prefs.putInt("cfg",      rcConfig.tapWindowMs);
  prefs.putInt("matrixCh", rcConfig.matrixChannel);
  prefs.putChar("fbMode",  rcConfig.funcBindings.modeSwitch);
  prefs.putChar("fbRADH",  rcConfig.funcBindings.radhAutoSwitch);
  prefs.putChar("fbMaint", rcConfig.funcBindings.maintLightsSwitch);
  prefs.putChar("fbLight", rcConfig.funcBindings.lightsModeSwitch);
  prefs.putChar("fbMuse",  rcConfig.funcBindings.museModeSwitch);

  // thresholds
  {
    DynamicJsonDocument doc(1024);
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
      JsonObject o = arr.createNestedObject();
      o["id"]     = rcConfig.thresholds[i].id;
      o["label"]  = rcConfig.thresholds[i].label;
      o["minPwm"] = rcConfig.thresholds[i].minPwm;
      o["maxPwm"] = rcConfig.thresholds[i].maxPwm;
    }
    String s; serializeJson(doc, s);
    prefs.putString("th", s);
  }

  // mode mappings (one key per mode so each stays small)
  const char* modeKeys[3] = { "m1", "m2", "m3" };
  for (int mode = 1; mode <= 3; mode++) {
    DynamicJsonDocument doc(4096);
    JsonObject root = doc.to<JsonObject>();
    for (int btn = 1; btn <= 19; btn++) {
      int idx = rcMapIndex(mode, btn);
      const RcMapping& m = rcConfig.mappings[idx];
      bool hasAny = false;
      for (int ti = 0; ti < 3 && !hasAny; ti++) if (m.t[ti].count > 0) hasAny = true;
      if (!hasAny && !m.exclusive) continue;

      String key = String(mode * 100 + btn);
      JsonObject mObj = root.createNestedObject(key);
      mObj["exclusive"] = m.exclusive;
      for (int ti = 0; ti < 3; ti++) {
        if (m.t[ti].count == 0) continue;
        String tierKey = String("t") + (ti + 1);
        JsonArray acts = mObj.createNestedArray(tierKey);
        for (int ai = 0; ai < m.t[ti].count; ai++) {
          actionToJson(m.t[ti].a[ai], acts.createNestedObject());
        }
      }
    }
    String s; serializeJson(doc, s);
    prefs.putString(modeKeys[mode - 1], s);
  }

  // switches — single key "sw"
  {
    DynamicJsonDocument doc(4096);
    JsonObject root = doc.to<JsonObject>();
    for (int i = 0; i < RC_NUM_SWITCHES; i++) {
      const RcSwitch& s = rcConfig.switches[i];
      JsonObject sObj = root.createNestedObject(RC_SWITCH_LABELS[i]);
      sObj["channel"]   = s.channel;
      sObj["positions"] = s.positions;
      for (int pi = 0; pi < 3; pi++) {
        if (s.t[pi].count == 0) continue;
        String tierKey = String("p") + pi;
        JsonArray acts = sObj.createNestedArray(tierKey);
        for (int ai = 0; ai < s.t[pi].count; ai++) {
          actionToJson(s.t[pi].a[ai], acts.createNestedObject());
        }
      }
    }
    String s; serializeJson(doc, s);
    prefs.putString("sw", s);
  }

  // knobs — single key "kn"
  {
    DynamicJsonDocument doc(512);
    JsonObject root = doc.to<JsonObject>();
    for (int i = 0; i < RC_NUM_KNOBS; i++) {
      JsonObject kObj = root.createNestedObject(RC_KNOB_LABELS[i]);
      kObj["channel"]  = rcConfig.knobs[i].channel;
      kObj["function"] = rcConfig.knobs[i].function;
    }
    String s; serializeJson(doc, s);
    prefs.putString("kn", s);
  }

  prefs.end();
  Serial.println("RC config saved to NVS");
}

void rcConfigLoadNVS() {
  Preferences prefs;
  prefs.begin("rcfg", true);  // read-only

  // tap window + matrix channel + function bindings
  if (prefs.isKey("cfg"))      rcConfig.tapWindowMs   = prefs.getInt("cfg",      rcConfig.tapWindowMs);
  if (prefs.isKey("matrixCh")) rcConfig.matrixChannel = prefs.getInt("matrixCh", rcConfig.matrixChannel);
  if (prefs.isKey("fbMode"))   rcConfig.funcBindings.modeSwitch        = prefs.getChar("fbMode",  rcConfig.funcBindings.modeSwitch);
  if (prefs.isKey("fbRADH"))   rcConfig.funcBindings.radhAutoSwitch    = prefs.getChar("fbRADH",  rcConfig.funcBindings.radhAutoSwitch);
  if (prefs.isKey("fbMaint"))  rcConfig.funcBindings.maintLightsSwitch = prefs.getChar("fbMaint", rcConfig.funcBindings.maintLightsSwitch);
  if (prefs.isKey("fbLight"))  rcConfig.funcBindings.lightsModeSwitch  = prefs.getChar("fbLight", rcConfig.funcBindings.lightsModeSwitch);
  if (prefs.isKey("fbMuse"))   rcConfig.funcBindings.museModeSwitch    = prefs.getChar("fbMuse",  rcConfig.funcBindings.museModeSwitch);

  // thresholds
  if (prefs.isKey("th")) {
    String s = prefs.getString("th", "");
    DynamicJsonDocument doc(1024);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonArray arr = doc.as<JsonArray>();
      int i = 0;
      for (JsonObject o : arr) {
        if (i >= RC_NUM_THRESHOLDS) break;
        rcConfig.thresholds[i].id = o["id"] | (i + 1);
        strlcpy(rcConfig.thresholds[i].label, o["label"] | "", 24);
        rcConfig.thresholds[i].minPwm = o["minPwm"] | rcConfig.thresholds[i].minPwm;
        rcConfig.thresholds[i].maxPwm = o["maxPwm"] | rcConfig.thresholds[i].maxPwm;
        i++;
      }
    }
  }

  // mode mappings
  const char* modeKeys[3] = { "m1", "m2", "m3" };
  for (int mode = 1; mode <= 3; mode++) {
    if (!prefs.isKey(modeKeys[mode - 1])) continue;
    String s = prefs.getString(modeKeys[mode - 1], "");
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, s) != DeserializationError::Ok) continue;
    JsonObject root = doc.as<JsonObject>();
    for (JsonPair kv : root) {
      int buttonId = String(kv.key().c_str()).toInt();
      int m_ = buttonId / 100, b_ = buttonId % 100;
      if (m_ < 1 || m_ > 3 || b_ < 1 || b_ > 19) continue;
      int idx = rcMapIndex(m_, b_);
      RcMapping& m = rcConfig.mappings[idx];
      memset(&m, 0, sizeof(m));
      JsonObject mObj = kv.value().as<JsonObject>();
      m.exclusive = mObj["exclusive"] | false;
      for (int ti = 0; ti < 3; ti++) {
        String tierKey = String("t") + (ti + 1);
        if (!mObj.containsKey(tierKey)) continue;
        JsonArray acts = mObj[tierKey];
        int ai = 0;
        for (JsonObject aObj : acts) {
          if (ai >= RC_ACTIONS_PER_TIER) break;
          if (actionFromJson(aObj, m.t[ti].a[ai])) m.t[ti].count = ++ai;
        }
      }
    }
  }

  // switches
  if (prefs.isKey("sw")) {
    String s = prefs.getString("sw", "");
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      for (int i = 0; i < RC_NUM_SWITCHES; i++) {
        if (!root.containsKey(RC_SWITCH_LABELS[i])) continue;
        JsonObject sObj = root[RC_SWITCH_LABELS[i]];
        RcSwitch& sw = rcConfig.switches[i];
        memset(sw.t, 0, sizeof(sw.t));
        sw.channel   = sObj["channel"]   | RC_SWITCH_DEFAULT_CH[i];
        sw.positions = sObj["positions"] | RC_SWITCH_DEFAULT_POS[i];
        for (int pi = 0; pi < 3; pi++) {
          String tierKey = String("p") + pi;
          if (!sObj.containsKey(tierKey)) continue;
          JsonArray acts = sObj[tierKey];
          int ai = 0;
          for (JsonObject aObj : acts) {
            if (ai >= RC_ACTIONS_PER_TIER) break;
            if (actionFromJson(aObj, sw.t[pi].a[ai])) sw.t[pi].count = ++ai;
          }
        }
      }
    }
  }

  // knobs
  if (prefs.isKey("kn")) {
    String s = prefs.getString("kn", "");
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      for (int i = 0; i < RC_NUM_KNOBS; i++) {
        if (!root.containsKey(RC_KNOB_LABELS[i])) continue;
        JsonObject kObj = root[RC_KNOB_LABELS[i]];
        rcConfig.knobs[i].channel  = kObj["channel"]  | RC_KNOB_DEFAULT_CH[i];
        rcConfig.knobs[i].function = kObj["function"] | RC_KNOB_DEFAULT_FN[i];
      }
    }
  }

  prefs.end();
  Serial.println("RC config loaded from NVS");
}
