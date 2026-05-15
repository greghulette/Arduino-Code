#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  rc_config.h — RC button / switch / knob mapping configuration
//
//  Adapted from Body_Controller_ESP32_GUI for the RC_Controller project.
//  Key changes vs. BC version:
//    • Action types: RA_ESPNOW/RA_HCR/RA_ANIM → RA_WCB_UNICAST/RA_WCB_BROADCAST/
//                   RA_MAESTRO_LOCAL/RA_MAESTRO_REMOTE
//    • RA_SERIAL ports: S3/S4 (Serial3-4 SoftwareSerial on WCB HW 3.2 —
//      Serial5 is now reserved for SBUS OUT passthrough)
//    • Knob functions: removed HCR volume; added KF_MAESTRO_LOCAL
//    • Default mappings: empty (all actions user-configured via GUI)
//    • Same NVS namespace "rcfg" and same JSON shape for GET_CONFIG/SET_CONFIG
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Action types
// ─────────────────────────────────────────────────────────────────────────────
enum RcActionType : uint8_t {
  RA_NONE              = 0,
  RA_WCB_UNICAST       = 1,   // wcb.send(boardId, cmd)   — boardId in target[]
  RA_WCB_BROADCAST     = 2,   // wcb.broadcast(cmd)
  RA_MAESTRO_LOCAL     = 3,   // local Maestro on Serial2 — cmd = "setTarget,ch,pos"
                               //   or "goHome" / "stopScript" / "restartScript,n"
  RA_MAESTRO_REMOTE    = 4,   // remote Maestro via WCBStream — target[] = "1"-"4",
                               //   cmd = same format as RA_MAESTRO_LOCAL
  RA_SERIAL            = 5,   // write cmd to S3/S4 (target[] = port label).
                               // S5 is reserved for SBUS OUT — not selectable.
  RA_HCR               = 6,   // Human-Cyborg Relations vocalizer command —
                               //   fn/chan/track describe the HCR call.
                               //   Destination (serial port OR WCB ID+port) is
                               //   a GLOBAL setting in RcConfig::hcrDest.
};

// Serial port labels for RA_SERIAL dispatch.
// Keep in sync with rcExecuteAction() in the .ino.
#define RA_SERIAL_PORTS_LIST   "S3,S4"

// ─────────────────────────────────────────────────────────────────────────────
//  RcAction — a single atomic action fired by a button press or switch change
// ─────────────────────────────────────────────────────────────────────────────
struct RcAction {
  RcActionType type;
  // target[]:
  //   RA_WCB_UNICAST     = WCB board ID string ("1"–"20")
  //   RA_MAESTRO_REMOTE  = remote Maestro slot ("1"–"8")
  //   RA_SERIAL          = port label ("S3"/"S4")
  //   others             = unused
  char    target[6];
  // cmd[]:
  //   RA_WCB_UNICAST/BROADCAST = command string (e.g. ":PP100")
  //   RA_MAESTRO_LOCAL/REMOTE  = "setTarget,ch,pos" | "goHome" | "stopScript" | "restartScript,n"
  //   RA_SERIAL                = command string (terminated with \r by dispatcher)
  //   RA_HCR                   = unused (use fn/chan/track instead)
  char    cmd[32];
  uint16_t delayMs;       // optional pre-fire delay (ms)
  char    note[20];       // human-readable label shown in GUI (19 chars + null)

  // RA_HCR-specific fields (zero for other action types). The HCR destination
  // (transport, serial port or WCB ID/port) is a GLOBAL setting stored in
  // RcConfig::hcrDest — every HCR action shares it.  See RcHcrDest.
  uint8_t fn;             // HCR function number (2=SetEmotion, 3=Trigger, 4=Stimulate,
                          //   5=Overload, 6=Muse, 8=Stop, 9=StopEmote, 11=ResetEmotions,
                          //   14=PlayWAV, 16=StopWAV, 17=SetVolume)
  int8_t  chan;           // emotion (0=H,1=S,2=M,3=C,4=Overload) or audio chan (0=V,1=A,2=B)
  int16_t track;          // PlayWAV track number, SetVolume value, Trigger level, etc.
};

#define RC_ACTIONS_PER_TIER  5   // max simultaneous actions per tap tier

struct RcTier {
  uint8_t  count;
  RcAction a[RC_ACTIONS_PER_TIER];
};

struct RcMapping {
  bool    exclusive;  // true = tap2 replaces tap1 within the tap window
  RcTier  t[3];       // t[0]=1-tap  t[1]=2-tap  t[2]=3-tap
};

// ─────────────────────────────────────────────────────────────────────────────
//  PWM threshold bands for the 19 virtual matrix-channel buttons
// ─────────────────────────────────────────────────────────────────────────────
struct RcThreshold {
  int  id;          // 1–19
  char label[24];
  int  minPwm;
  int  maxPwm;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Configurable toggle switches SA–SJ (10 total)
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_SWITCHES 10

struct RcSwitch {
  int     channel;     // SBUS channel 1–24; 0 = disabled
  uint8_t positions;   // 2 or 3
  RcTier  t[3];        // [down, mid, up] — for 2-pos switches t[1] unused
};

static const char*   RC_SWITCH_LABELS[RC_NUM_SWITCHES]     = {"SA","SB","SC","SD","SE","SF","SG","SH","SI","SJ"};
static const int     RC_SWITCH_DEFAULT_CH[RC_NUM_SWITCHES]  = {  8,   9,  10,  11,  12,  13,  14,  15,   0,   0 };
static const uint8_t RC_SWITCH_DEFAULT_POS[RC_NUM_SWITCHES] = {  3,   3,   3,   3,   3,   2,   3,   2,   2,   2 };

#define SW_SA 0
#define SW_SB 1
#define SW_SC 2
#define SW_SD 3
#define SW_SE 4
#define SW_SF 5
#define SW_SG 6
#define SW_SH 7
#define SW_SI 8
#define SW_SJ 9

// ─────────────────────────────────────────────────────────────────────────────
//  Configurable analog sources (8 total — 2 rotary knobs + 2 sliders + 4 gimbal axes)
//    Panel:    S1, S2 (rotary knobs) + LS, RS (side sliders)
//    Sticks:   J1, J2 (right gimbal X/Y axes — AIL/ELE)
//              J3, J4 (left  gimbal Y/X axes — THR/RUD)
//
//  The C struct is still called RcKnob (umbrella term) but the GUI labels
//  J1-J4 as joystick axes, LS/RS as sliders, and S1/S2 as knobs.
//
//  Each source samples one SBUS channel and dispatches its value through one
//  of two functions:
//
//    KF_MAESTRO_PASSTHROUGH  → outputs[] entries each drive a Maestro servo
//                              channel (Maestro ID 1-8 + maestroCh + qus min/max).
//    KF_HCR_VOLUME           → outputs[] entries each drive an HCR audio
//                              channel volume (audio chan + vol min/max 0-99).
//
//  Maestro Pass-Through and HCR Volume are PARALLEL choices — a given knob
//  does one or the other (not both at once). All outputs[] share the parent
//  knob's function.
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_KNOBS         8
#define RC_KNOB_MAX_OUTPUTS  8

enum RcKnobFunction : uint8_t {
  KF_NONE               = 0,
  KF_MAESTRO_PASSTHROUGH = 1,   // outputs[] = Maestro servo passthroughs
  KF_HCR_VOLUME         = 2,    // outputs[] = HCR audio-channel volumes
};

struct RcKnobOutput {
  // Interpreted by the parent knob's function:
  //   KF_MAESTRO_PASSTHROUGH: target = Maestro slot ID (1-8 → rcConfig.maestros[id-1])
  //   KF_HCR_VOLUME:          target = HCR audio chan (0=V vocalizer, 1=A, 2=B)
  uint8_t  target;
  // KF_MAESTRO_PASSTHROUGH only: Maestro servo channel (0-31). Unused for HCR.
  uint8_t  maestroCh;
  // KF_MAESTRO_PASSTHROUGH: Maestro qus position at SBUS min/max (e.g. 4000-8000)
  // KF_HCR_VOLUME:          HCR volume value at SBUS min/max (0-99 clamped)
  uint16_t posMin;
  uint16_t posMax;
};

struct RcKnob {
  int     channel;     // SBUS channel 1–24; 0 = disabled
  uint8_t function;    // RcKnobFunction
  uint8_t outputCount; // number of valid entries in outputs[]
  RcKnobOutput outputs[RC_KNOB_MAX_OUTPUTS];
};

// Default SBUS channels assume typical X18 Mode-2 layout:
//   J1 = CH1 (AIL = R-stick X)   J3 = CH3 (THR = L-stick Y)
//   J2 = CH2 (ELE = R-stick Y)   J4 = CH4 (RUD = L-stick X)
//   S1 = CH5, S2 = CH6  (Kyber-aligned knobs)
//   LS/RS sliders disabled by default (channel 0); user assigns in tool.
static const char*   RC_KNOB_LABELS[RC_NUM_KNOBS]    = {"S1","S2","LS","RS","J1","J2","J3","J4"};
static const int     RC_KNOB_DEFAULT_CH[RC_NUM_KNOBS] = {  5,   6,   0,   0,   1,   2,   3,   4 };
static const uint8_t RC_KNOB_DEFAULT_FN[RC_NUM_KNOBS] = {
  KF_NONE, KF_NONE, KF_NONE, KF_NONE,
  KF_NONE, KF_NONE, KF_NONE, KF_NONE
};

// ─────────────────────────────────────────────────────────────────────────────
//  Mode selector binding — which switch drives the 1/2/3 mode that gates the
//  button matrix action set.  Set to -1 to disable the mode system.
// ─────────────────────────────────────────────────────────────────────────────
struct RcFuncBindings {
  int8_t modeSwitch;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Global HCR destination — one vocalizer wiring shared by all RA_HCR actions.
//  The user configures this once in the GUI; individual HCR actions only carry
//  fn/chan/track.
//
//    transport = 0  : local SoftwareSerial port (target = "S3"/"S4")
//    transport = 1  : WCB ESP-NOW raw forward (target = WCB ID "0"-"20",
//                      0 = broadcast; wcbPort = serial port on receiver 1-5)
// ─────────────────────────────────────────────────────────────────────────────
struct RcHcrDest {
  uint8_t transport;
  char    target[6];
  uint8_t wcbPort;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Maestro locations (8 slots, ID 1-8)
//
//  Buttons / knobs / actions reference Maestros by their slot ID (1-8) only —
//  they don't care whether a given Maestro is wired locally to Serial2 or
//  routed via ESP-NOW broadcast. This struct holds the runtime wiring for
//  each slot, so the dispatcher knows where to send the Pololu bytes and
//  which device number to embed in the Pololu protocol header.
//
//    type = 0  : disabled (slot ignored by the dispatcher)
//    type = 1  : local — wired to Serial2 on this board
//    type = 2  : remote — broadcast via ESP-NOW to all WCBs; any WCB with
//                Kyber_Remote enabled forwards the bytes to its Maestro port
//
//  device: Pololu protocol device number (0-127).  Every Maestro on the
//  network MUST have a unique device # set in Maestro Control Center —
//  this firmware always uses Pololu protocol (not compact), because the
//  broadcast model puts multiple Maestros on a shared logical bus and the
//  device-number filter is what keeps them from all responding to every
//  command.
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_MAESTROS 8

struct RcMaestroSlot {
  uint8_t type;       // 0 disabled · 1 local (Serial2) · 2 remote (broadcast)
  uint8_t device;     // 0-127, Pololu protocol device # (no compact support)
};

// ─────────────────────────────────────────────────────────────────────────────
//  Full runtime config block
//  buttonId  = (mode * 100) + btn  →  e.g. mode=1, btn=5 → 105
//  flat index = (mode-1)*19 + (btn-1)  → 0–56
// ─────────────────────────────────────────────────────────────────────────────
#define RC_NUM_THRESHOLDS 19
#define RC_NUM_MAPPINGS   57   // 3 modes × 19 buttons

inline int rcMapIndex(int mode, int btn) { return (mode - 1) * 19 + (btn - 1); }

struct RcConfig {
  int            tapWindowMs;
  int            matrixChannel;   // SBUS channel carrying the multiplexed button matrix
  RcThreshold    thresholds[RC_NUM_THRESHOLDS];
  RcMapping      mappings[RC_NUM_MAPPINGS];
  RcSwitch       switches[RC_NUM_SWITCHES];
  RcKnob         knobs[RC_NUM_KNOBS];
  RcFuncBindings funcBindings;
  RcHcrDest      hcrDest;
  RcMaestroSlot  maestros[RC_NUM_MAESTROS];  // ID 1-8 → maestros[0..7]
};

extern RcConfig rcConfig;

// ─────────────────────────────────────────────────────────────────────────────
//  Construction helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline RcAction makeWCBUnicast(const char* wcbId, const char* cmd,
                                      uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_WCB_UNICAST;
  strlcpy(a.target, wcbId, sizeof(a.target));
  strlcpy(a.cmd,    cmd,   sizeof(a.cmd));
  strlcpy(a.note,   note,  sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeWCBBroadcast(const char* cmd,
                                         uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_WCB_BROADCAST;
  strlcpy(a.cmd,  cmd,  sizeof(a.cmd));
  strlcpy(a.note, note, sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeMaestroLocal(const char* cmd,
                                         uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_MAESTRO_LOCAL;
  strlcpy(a.cmd,  cmd,  sizeof(a.cmd));
  strlcpy(a.note, note, sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeMaestroRemote(const char* slot, const char* cmd,
                                          uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_MAESTRO_REMOTE;
  strlcpy(a.target, slot, sizeof(a.target));
  strlcpy(a.cmd,    cmd,  sizeof(a.cmd));
  strlcpy(a.note,   note, sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline RcAction makeSerial(const char* port, const char* cmd,
                                   uint16_t delayMs = 0, const char* note = "") {
  RcAction a = {};
  a.type = RA_SERIAL;
  strlcpy(a.target, port, sizeof(a.target));
  strlcpy(a.cmd,    cmd,  sizeof(a.cmd));
  strlcpy(a.note,   note, sizeof(a.note));
  a.delayMs = delayMs;
  return a;
}
static inline void setTier1(RcTier& tier, RcAction a0) {
  tier.count = 1; tier.a[0] = a0;
}
static inline void setTier2(RcTier& tier, RcAction a0, RcAction a1) {
  tier.count = 2; tier.a[0] = a0; tier.a[1] = a1;
}
static inline void setTier3(RcTier& tier, RcAction a0, RcAction a1, RcAction a2) {
  tier.count = 3; tier.a[0] = a0; tier.a[1] = a1; tier.a[2] = a2;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Factory defaults
//  All button/matrix mappings start empty — configure via GUI.
//  Switch channel assignments mirror the Kyber ELRS layout (SA-SH on CH8-CH15).
// ─────────────────────────────────────────────────────────────────────────────
void rcConfigLoadDefaults() {
  rcConfig.tapWindowMs   = 500;
  rcConfig.matrixChannel = 7;            // button matrix on CH7
  rcConfig.funcBindings.modeSwitch = SW_SE;  // SE (CH12) drives mode 1/2/3

  // Default PWM threshold bands (matches Body Controller layout for compatibility)
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
    { "T1 Left",     1023, 1049 },
    { "T1 Right",     850,  960 },
    { "Unassigned",   800,  849 },
  };
  for (int i = 0; i < 19; i++) {
    rcConfig.thresholds[i].id = i + 1;
    strlcpy(rcConfig.thresholds[i].label, bands[i].label, 24);
    rcConfig.thresholds[i].minPwm = bands[i].mn;
    rcConfig.thresholds[i].maxPwm = bands[i].mx;
  }

  // All matrix button mappings start empty
  memset(rcConfig.mappings, 0, sizeof(rcConfig.mappings));

  // Default switch channel / position assignments (no actions — user configures)
  memset(rcConfig.switches, 0, sizeof(rcConfig.switches));
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    rcConfig.switches[i].channel   = RC_SWITCH_DEFAULT_CH[i];
    rcConfig.switches[i].positions = RC_SWITCH_DEFAULT_POS[i];
  }

  // Default knob assignments (no outputs assigned — user configures in GUI)
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    rcConfig.knobs[i].channel     = RC_KNOB_DEFAULT_CH[i];
    rcConfig.knobs[i].function    = RC_KNOB_DEFAULT_FN[i];
    rcConfig.knobs[i].outputCount = 0;
    memset(rcConfig.knobs[i].outputs, 0, sizeof(rcConfig.knobs[i].outputs));
  }

  // Default global HCR destination — local Serial S3, disabled until user
  // changes it (no transport effect when no HCR actions are configured).
  rcConfig.hcrDest.transport = 0;
  strlcpy(rcConfig.hcrDest.target, "S3", sizeof(rcConfig.hcrDest.target));
  rcConfig.hcrDest.wcbPort   = 1;

  // Default Maestro slots — all 8 disabled until user enables them in the
  // GUI Maestro Locations panel.  Device numbers default to match the slot
  // ID (slot 1 → device 1, ..., slot 8 → device 8) so they're easy to
  // remember.  The user must set the matching device # in Maestro Control
  // Center on each physical Maestro for the dispatcher's address filter to
  // route correctly.
  for (int i = 0; i < RC_NUM_MAESTROS; i++) {
    rcConfig.maestros[i].type   = 0;
    rcConfig.maestros[i].device = (uint8_t)(1 + i);   // 1, 2, ..., 8
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  JSON helpers — action ↔ JSON object
// ─────────────────────────────────────────────────────────────────────────────
static void actionToJson(const RcAction& a, JsonObject obj) {
  switch (a.type) {
    case RA_WCB_UNICAST:
      obj["type"]   = "wcb_unicast";
      obj["target"] = a.target;   // WCB board ID string
      obj["cmd"]    = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_WCB_BROADCAST:
      obj["type"] = "wcb_broadcast";
      obj["cmd"]  = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_MAESTRO_LOCAL:
      obj["type"] = "maestro_local";
      obj["cmd"]  = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_MAESTRO_REMOTE:
      // Unified Maestro action — target = Maestro slot ID ("1"-"8") from the
      // GUI Maestro Locations panel. The enum value is still
      // RA_MAESTRO_REMOTE for backward compat with the dispatcher switch, but
      // the JSON name is the user-facing "maestro".
      obj["type"]   = "maestro";
      obj["target"] = a.target;
      obj["cmd"]    = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_SERIAL:
      obj["type"]  = "serial";
      obj["port"]  = a.target;    // "S3"/"S4"
      obj["cmd"]   = a.cmd;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    case RA_HCR:
      // Destination (transport / target / wcbPort) lives in rcConfig.hcrDest
      // as a global setting — see RcHcrDest.  Each action only carries the
      // HCR command itself.
      obj["type"]  = "hcr";
      obj["fn"]    = a.fn;
      obj["chan"]  = a.chan;
      obj["track"] = a.track;
      if (a.delayMs) obj["delay"] = a.delayMs;
      break;
    default: break;
  }
  if (a.note[0]) obj["note"] = a.note;
}

static bool actionFromJson(const JsonObject& obj, RcAction& a) {
  memset(&a, 0, sizeof(a));
  const char* type = obj["type"] | "";
  bool ok = false;
  if (strcmp(type, "wcb_unicast") == 0) {
    a.type = RA_WCB_UNICAST;
    strlcpy(a.target, obj["target"] | "", sizeof(a.target));
    strlcpy(a.cmd,    obj["cmd"]    | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "wcb_broadcast") == 0) {
    a.type = RA_WCB_BROADCAST;
    strlcpy(a.cmd, obj["cmd"] | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "maestro_local") == 0) {
    a.type = RA_MAESTRO_LOCAL;
    strlcpy(a.cmd, obj["cmd"] | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "maestro_remote") == 0 || strcmp(type, "maestro") == 0) {
    // "maestro" is the unified action emitted by the new config tool.
    // "maestro_remote" is accepted for backward compat with older saved configs.
    // Both store target = Maestro slot ID 1-8 (firmware uses rcConfig.maestros[]
    // to decide where the bytes actually go).
    a.type = RA_MAESTRO_REMOTE;
    strlcpy(a.target, obj["target"] | "", sizeof(a.target));
    strlcpy(a.cmd,    obj["cmd"]    | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "serial") == 0) {
    a.type = RA_SERIAL;
    strlcpy(a.target, obj["port"] | "", sizeof(a.target));
    strlcpy(a.cmd,    obj["cmd"]  | "", sizeof(a.cmd));
    a.delayMs = obj["delay"] | 0;
    ok = true;
  } else if (strcmp(type, "hcr") == 0) {
    // HCR destination is now a global config (RcHcrDest in RcConfig).
    // Each action only carries the HCR command parameters.
    a.type    = RA_HCR;
    a.fn      = (uint8_t)(obj["fn"]    | 0);
    a.chan    = (int8_t) (obj["chan"]  | 0);
    a.track   = (int16_t)(obj["track"] | 0);
    a.delayMs = obj["delay"] | 0;
    ok = true;
  }
  if (ok) strlcpy(a.note, obj["note"] | "", sizeof(a.note));
  return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Serialise full config to JSON string (for GET_CONFIG WebSocket response)
// ─────────────────────────────────────────────────────────────────────────────
String rcConfigToJSON() {
  DynamicJsonDocument doc(32768);

  doc["tapWindowMs"]   = rcConfig.tapWindowMs;
  doc["matrixChannel"] = rcConfig.matrixChannel;

  JsonObject fb = doc.createNestedObject("funcBindings");
  fb["mode"] = rcConfig.funcBindings.modeSwitch;

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
      bool hasAny = false;
      for (int ti = 0; ti < 3 && !hasAny; ti++) if (m.t[ti].count > 0) hasAny = true;
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

  JsonObject swObj = doc.createNestedObject("switches");
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    const RcSwitch& s = rcConfig.switches[i];
    JsonObject sObj = swObj.createNestedObject(RC_SWITCH_LABELS[i]);
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

  JsonObject knObj = doc.createNestedObject("knobs");
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    const RcKnob& kn = rcConfig.knobs[i];
    JsonObject kObj = knObj.createNestedObject(RC_KNOB_LABELS[i]);
    kObj["channel"]  = kn.channel;
    kObj["function"] = kn.function;
    JsonArray outsArr = kObj.createNestedArray("outputs");
    for (uint8_t o = 0; o < kn.outputCount && o < RC_KNOB_MAX_OUTPUTS; o++) {
      JsonObject oObj = outsArr.createNestedObject();
      oObj["target"]    = kn.outputs[o].target;        // Maestro ID 1-8 OR audio chan
      oObj["maestroCh"] = kn.outputs[o].maestroCh;
      oObj["posMin"]    = kn.outputs[o].posMin;
      oObj["posMax"]    = kn.outputs[o].posMax;
    }
  }

  // Global HCR destination — every RA_HCR action reads from here.
  JsonObject hcrObj = doc.createNestedObject("hcrDest");
  hcrObj["transport"] = (rcConfig.hcrDest.transport == 1) ? "wcb" : "serial";
  if (rcConfig.hcrDest.transport == 1) {
    hcrObj["target"]  = rcConfig.hcrDest.target;       // WCB ID string
    hcrObj["wcbPort"] = rcConfig.hcrDest.wcbPort;
  } else {
    hcrObj["port"]    = rcConfig.hcrDest.target;       // "S3"/"S4"
  }

  // Maestro locations — describes where each of the 8 logical Maestros is wired.
  JsonArray maeArr = doc.createNestedArray("maestros");
  for (int i = 0; i < RC_NUM_MAESTROS; i++) {
    JsonObject mObj = maeArr.createNestedObject();
    mObj["type"]   = rcConfig.maestros[i].type;        // 0 disabled / 1 local / 2 remote
    mObj["device"] = rcConfig.maestros[i].device;      // 0-127 protocol, 255 compact
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Load config from JSON object (from SET_CONFIG WebSocket message)
// ─────────────────────────────────────────────────────────────────────────────
bool rcConfigFromJSON(const JsonObject& doc) {
  if (doc.containsKey("tapWindowMs"))   rcConfig.tapWindowMs   = doc["tapWindowMs"];
  if (doc.containsKey("matrixChannel")) rcConfig.matrixChannel = doc["matrixChannel"];

  if (doc.containsKey("funcBindings")) {
    JsonObject fb = doc["funcBindings"];
    rcConfig.funcBindings.modeSwitch = fb["mode"] | rcConfig.funcBindings.modeSwitch;
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
      int mode = buttonId / 100, btn = buttonId % 100;
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
          if (actionFromJson(aObj, m.t[ti].a[ai])) m.t[ti].count = ++ai;
        }
      }
    }
  }

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

  if (doc.containsKey("knobs")) {
    JsonObject knObj = doc["knobs"];
    for (int i = 0; i < RC_NUM_KNOBS; i++) {
      if (!knObj.containsKey(RC_KNOB_LABELS[i])) continue;
      JsonObject kObj = knObj[RC_KNOB_LABELS[i]];
      RcKnob& kn = rcConfig.knobs[i];
      kn.channel  = kObj["channel"]  | RC_KNOB_DEFAULT_CH[i];
      kn.function = kObj["function"] | RC_KNOB_DEFAULT_FN[i];
      kn.outputCount = 0;
      memset(kn.outputs, 0, sizeof(kn.outputs));
      if (kObj.containsKey("outputs")) {
        JsonArray outsArr = kObj["outputs"];
        for (JsonObject oObj : outsArr) {
          if (kn.outputCount >= RC_KNOB_MAX_OUTPUTS) break;
          RcKnobOutput& out = kn.outputs[kn.outputCount];
          // New schema: "target" holds Maestro ID 1-8 or audio chan 0-2.
          // Legacy schema: "slot" with 0 = local, 1-8 = remote slot — map
          // slot 0 → Maestro ID 1 and the others 1:1.
          if (oObj.containsKey("target")) {
            out.target = (uint8_t)(oObj["target"] | 0);
          } else {
            uint8_t legacy = (uint8_t)(oObj["slot"] | 0);
            out.target = (legacy == 0) ? 1 : legacy;
          }
          out.maestroCh = (uint8_t) (oObj["maestroCh"] | 0);
          out.posMin    = (uint16_t)(oObj["posMin"]    | 4000);
          out.posMax    = (uint16_t)(oObj["posMax"]    | 8000);
          kn.outputCount++;
        }
      }
    }
  }

  if (doc.containsKey("maestros")) {
    JsonArray maeArr = doc["maestros"];
    int i = 0;
    for (JsonObject mObj : maeArr) {
      if (i >= RC_NUM_MAESTROS) break;
      rcConfig.maestros[i].type   = (uint8_t)(mObj["type"]   | 0);
      rcConfig.maestros[i].device = (uint8_t)(mObj["device"] | (1 + i));
      i++;
    }
  }

  if (doc.containsKey("hcrDest")) {
    JsonObject hcrObj = doc["hcrDest"];
    const char* tp = hcrObj["transport"] | "serial";
    rcConfig.hcrDest.transport = (strcmp(tp, "wcb") == 0) ? 1 : 0;
    if (rcConfig.hcrDest.transport == 1) {
      strlcpy(rcConfig.hcrDest.target, hcrObj["target"] | "0", sizeof(rcConfig.hcrDest.target));
      rcConfig.hcrDest.wcbPort = (uint8_t)(hcrObj["wcbPort"] | 1);
    } else {
      strlcpy(rcConfig.hcrDest.target, hcrObj["port"]   | "S3", sizeof(rcConfig.hcrDest.target));
      rcConfig.hcrDest.wcbPort = 0;
    }
  }
  return true;
}

bool rcConfigFromJSON(const String& json) {
  DynamicJsonDocument doc(98304);
  DeserializationError err = deserializeJson(doc, json);
  if (err != DeserializationError::Ok) {
    Serial.printf("rcConfigFromJSON parse failed: %s\n", err.c_str());
    return false;
  }
  return rcConfigFromJSON(doc.as<JsonObject>());
}

// ─────────────────────────────────────────────────────────────────────────────
//  NVS persistence (namespace "rcfg")
//  Same key layout as Body Controller for familiar tooling.
//  Split across keys to stay within the NVS 4000-byte per-value limit.
// ─────────────────────────────────────────────────────────────────────────────
void rcConfigSaveNVS() {
  Preferences prefs;
  prefs.begin("rcfg", false);

  prefs.putInt("cfg",      rcConfig.tapWindowMs);
  prefs.putInt("matrixCh", rcConfig.matrixChannel);
  prefs.putChar("fbMode",  rcConfig.funcBindings.modeSwitch);

  // Thresholds
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

  // Mode mappings (one key per mode)
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

  // Switches
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

  // Knobs — 8 knobs × up to 8 outputs × ~50 bytes + overhead.
  // Worst case ~3.5 KB; use 4 KB buffer to stay under NVS 4000-byte value cap.
  {
    DynamicJsonDocument doc(4096);
    JsonObject root = doc.to<JsonObject>();
    for (int i = 0; i < RC_NUM_KNOBS; i++) {
      const RcKnob& kn = rcConfig.knobs[i];
      JsonObject kObj = root.createNestedObject(RC_KNOB_LABELS[i]);
      kObj["channel"]  = kn.channel;
      kObj["function"] = kn.function;
      JsonArray outsArr = kObj.createNestedArray("outputs");
      for (uint8_t o = 0; o < kn.outputCount && o < RC_KNOB_MAX_OUTPUTS; o++) {
        JsonObject oObj = outsArr.createNestedObject();
        oObj["target"]    = kn.outputs[o].target;
        oObj["maestroCh"] = kn.outputs[o].maestroCh;
        oObj["posMin"]    = kn.outputs[o].posMin;
        oObj["posMax"]    = kn.outputs[o].posMax;
      }
    }
    String s; serializeJson(doc, s);
    prefs.putString("kn", s);
  }

  // Global HCR destination
  {
    DynamicJsonDocument doc(256);
    JsonObject root = doc.to<JsonObject>();
    root["transport"] = (rcConfig.hcrDest.transport == 1) ? "wcb" : "serial";
    if (rcConfig.hcrDest.transport == 1) {
      root["target"]  = rcConfig.hcrDest.target;
      root["wcbPort"] = rcConfig.hcrDest.wcbPort;
    } else {
      root["port"]    = rcConfig.hcrDest.target;
    }
    String s; serializeJson(doc, s);
    prefs.putString("hcr", s);
  }

  // Maestro locations
  {
    DynamicJsonDocument doc(512);
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < RC_NUM_MAESTROS; i++) {
      JsonObject mObj = arr.createNestedObject();
      mObj["type"]   = rcConfig.maestros[i].type;
      mObj["device"] = rcConfig.maestros[i].device;
    }
    String s; serializeJson(doc, s);
    prefs.putString("mae", s);
  }

  prefs.end();
  Serial.println("RC config saved to NVS.");
}

void rcConfigLoadNVS() {
  Preferences prefs;
  prefs.begin("rcfg", true);

  if (prefs.isKey("cfg"))      rcConfig.tapWindowMs   = prefs.getInt("cfg",      rcConfig.tapWindowMs);
  if (prefs.isKey("matrixCh")) rcConfig.matrixChannel = prefs.getInt("matrixCh", rcConfig.matrixChannel);
  if (prefs.isKey("fbMode"))   rcConfig.funcBindings.modeSwitch = prefs.getChar("fbMode", rcConfig.funcBindings.modeSwitch);

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

  if (prefs.isKey("kn")) {
    String s = prefs.getString("kn", "");
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      for (int i = 0; i < RC_NUM_KNOBS; i++) {
        if (!root.containsKey(RC_KNOB_LABELS[i])) continue;
        JsonObject kObj = root[RC_KNOB_LABELS[i]];
        RcKnob& kn = rcConfig.knobs[i];
        kn.channel  = kObj["channel"]  | RC_KNOB_DEFAULT_CH[i];
        kn.function = kObj["function"] | RC_KNOB_DEFAULT_FN[i];
        kn.outputCount = 0;
        memset(kn.outputs, 0, sizeof(kn.outputs));
        if (kObj.containsKey("outputs")) {
          JsonArray outsArr = kObj["outputs"];
          for (JsonObject oObj : outsArr) {
            if (kn.outputCount >= RC_KNOB_MAX_OUTPUTS) break;
            RcKnobOutput& out = kn.outputs[kn.outputCount];
            if (oObj.containsKey("target")) {
              out.target = (uint8_t)(oObj["target"] | 0);
            } else {
              uint8_t legacy = (uint8_t)(oObj["slot"] | 0);
              out.target = (legacy == 0) ? 1 : legacy;
            }
            out.maestroCh = (uint8_t) (oObj["maestroCh"] | 0);
            out.posMin    = (uint16_t)(oObj["posMin"]    | 4000);
            out.posMax    = (uint16_t)(oObj["posMax"]    | 8000);
            kn.outputCount++;
          }
        }
      }
    }
  }

  if (prefs.isKey("mae")) {
    String s = prefs.getString("mae", "");
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonArray arr = doc.as<JsonArray>();
      int i = 0;
      for (JsonObject mObj : arr) {
        if (i >= RC_NUM_MAESTROS) break;
        rcConfig.maestros[i].type   = (uint8_t)(mObj["type"]   | 0);
        rcConfig.maestros[i].device = (uint8_t)(mObj["device"] | (1 + i));
        i++;
      }
    }
  }

  if (prefs.isKey("hcr")) {
    String s = prefs.getString("hcr", "");
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      JsonObject root = doc.as<JsonObject>();
      const char* tp = root["transport"] | "serial";
      rcConfig.hcrDest.transport = (strcmp(tp, "wcb") == 0) ? 1 : 0;
      if (rcConfig.hcrDest.transport == 1) {
        strlcpy(rcConfig.hcrDest.target, root["target"] | "0", sizeof(rcConfig.hcrDest.target));
        rcConfig.hcrDest.wcbPort = (uint8_t)(root["wcbPort"] | 1);
      } else {
        strlcpy(rcConfig.hcrDest.target, root["port"]   | "S3", sizeof(rcConfig.hcrDest.target));
        rcConfig.hcrDest.wcbPort = 0;
      }
    }
  }

  prefs.end();
  Serial.println("RC config loaded from NVS.");
}
