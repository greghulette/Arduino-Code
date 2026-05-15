// =============================================================================
//  RC_Controller.ino — WCB-based RC Controller
//  Target hardware: WCB HW 3.2 (ESP32-S3)
//
//  Features:
//    • SBUS input (16 or 24 channel, auto-detected) on Serial1 RX (GPIO5)
//    • SBUS output passthrough on GPIO9 (byte-streaming, ~110 µs latency) —
//      lets a downstream device consume the same SBUS stream
//    • Local Pololu Maestro on Serial2 @ 115200 baud (GPIO6 TX)
//    • Up to 8 remote Maestros via WCBStream broadcast over ESP-NOW
//    • WCB unicast and broadcast command dispatch
//    • Serial3, Serial4 available as general-purpose SoftwareSerial ports
//    • Multi-mode RC button/switch/knob mapping (NVS-backed, GUI-configurable)
//    • USB Serial JSON protocol for config tool and CLI debugging
//      (open config_tool/index.html on your PC, connect via Web Serial API)
//
//  USB Serial JSON Protocol (newline-delimited):
//    PING                        → {"type":"PONG"}
//    GET_CONFIG                  → {"type":"CONFIG","data":{...full config JSON...}}
//    {"type":"SET_CONFIG","data":{...}} → {"type":"ACK","ok":true}
//    {"type":"START_MONITOR"}    → streams PWM_UPDATE every 50 ms until STOP_MONITOR
//    {"type":"STOP_MONITOR"}     → {"type":"ACK","ok":true}
//    {"type":"RESET_DEFAULTS"}   → reloads factory defaults, replies ACK
//    {"type":"TRIGGER","mode":1,"btn":3,"tap":1} → fires virtual button press
//    {"type":"WCB_SEND","target":2,"cmd":":PP100"} → manually fires WCB command
//
//  Required libraries (Library Manager):
//    • ArduinoJson        (Benoit Blanchon) v6.x
//    • Adafruit NeoPixel
//
//  Local libraries (in libraries/ folder):
//    • WCBClient + WCBStream
//    • PololuMaestro
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <WCBClient.h>
#include <WCBStream.h>
#include <hcr.h>
#include "sbus_reader.h"
#include "rc_config.h"
#include "wcb_config.h"

// =============================================================================
//  WCB HW 3.2 — Pin Definitions
//  TX pins from wcb_pin_map.cpp v3.2 (matches SBUSController.ino comments).
//  RX pins: GPIO5 confirmed by user; others assumed — verify against schematic.
// =============================================================================
#define SBUS_RX_PIN      5    // Serial1 RX — SBUS from RC receiver (confirmed)
#define SBUS_TX_PIN      4    // Serial1 TX — unused (SBUS is RX-only here)

#define MAESTRO_TX_PIN   6    // Serial2 TX — local Maestro command bus
#define MAESTRO_RX_PIN   7    // Serial2 RX — optional Maestro feedback (verify pin)

#define S3_TX_PIN       15    // Aux serial S3 TX (SoftwareSerial)
#define S3_RX_PIN       16    // Aux serial S3 RX
#define S4_TX_PIN       17    // Aux serial S4 TX (SoftwareSerial)
#define S4_RX_PIN       18    // Aux serial S4 RX

// SBUS OUT — pure passthrough re-emission of the SBUS RX stream so downstream
// devices can consume the same channel data. Bit-banged via SoftwareSerial at
// 100k 8E2 inverted (TX-only); each byte is forwarded ~110 µs after it
// arrives on Serial1 RX. See SbusReader::setPassthroughSink() for the tee.
#define SBUS_OUT_PIN     9    // SoftwareSerial TX → downstream SBUS consumer

#define STATUS_LED_PIN  48    // WCB 3.2 onboard NeoPixel — verify against schematic
#define STATUS_LED_COUNT 1

// =============================================================================
//  WCB Client + Streams
//
//  Single shared broadcast stream:  All remote Maestro bytes go out on one
//  WCBStream targeting `broadcast` (target_wcb=0). The Kyber path delivers
//  them to every WCB on the network. Each receiving WCB with Kyber_Remote
//  forwards the raw bytes to its configured Maestro port — no per-slot
//  WCB/port routing needed on the sender.
//
//  No compile-time Maestro instances: Maestro slots (IDs 1-8) are configured
//  at RUNTIME via the GUI's Maestro Locations panel (rcConfig.maestros[]).
//  Each slot decides whether bytes go to Serial2 (local) or the broadcast
//  stream (remote), and which Pololu device # to embed.  See maestroWrite()
//  below for the dispatch.
// =============================================================================
WCBClient wcb(WCB_MAC_OCT2, WCB_MAC_OCT3, WCB_PASSWORD, WCB_QUANTITY, WCB_DEVICE_ID);

// One stream, broadcast to all WCBs.  target_port is ignored for broadcast.
WCBStream maestroBroadcast(/*target_wcb=*/0, /*target_port=*/0);

// =============================================================================
//  Aux serial ports S3, S4  +  dedicated SBUS OUT
//
//  ESP32-S3 has only 3 hardware UARTs (Serial / Serial1 / Serial2), all
//  claimed: USB-CDC, SBUS RX, local Maestro TX. Everything else is
//  bit-banged via SoftwareSerial. Aux ports stay reserved for ≤57600 baud
//  command lines; SBUS OUT runs at 100k 8E2 inverted TX-only.
// =============================================================================
SoftwareSerial Serial3;    // Aux command-line port, bound in setup()
SoftwareSerial Serial4;    // Aux command-line port, bound in setup()
SoftwareSerial sbusOut;    // SBUS OUT passthrough, bound in setup()

// =============================================================================
//  HCR Vocalizer — one instance per local aux serial port (S3, S4)
//
//  The HCRVocalizer library writes its own framed command strings (e.g.
//  "<SH3,QEH,QT>\n") to the bound Stream. For WCB transport we don't use
//  HCRVocalizer at all; we format the same byte string ourselves and ship it
//  via wcb.sendRaw() / wcb.broadcast() — the receiving WCB forwards the bytes
//  to its serial port unmodified, with no Kyber_Remote config required.
//
//  S5 is no longer available for HCR (it's reserved for SBUS OUT).
// =============================================================================
#define HCR_BAUD_RATE 9600
HCRVocalizer hcrS3(&Serial3, HCR_BAUD_RATE);
HCRVocalizer hcrS4(&Serial4, HCR_BAUD_RATE);

// =============================================================================
//  SBUS + RC Config
// =============================================================================
SbusReader sbusRx;
RcConfig   rcConfig;

// =============================================================================
//  Status LED
// =============================================================================
Adafruit_NeoPixel statusLed(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

void setStatusLed(uint32_t color, uint8_t brightness = 20) {
  statusLed.setBrightness(brightness);
  statusLed.setPixelColor(0, color);
  statusLed.show();
}

static const uint32_t C_RED    = 0xFF0000;
static const uint32_t C_GREEN  = 0x00FF00;
static const uint32_t C_BLUE   = 0x0000FF;
static const uint32_t C_YELLOW = 0xFFFF00;
static const uint32_t C_ORANGE = 0xFF8000;
static const uint32_t C_CYAN   = 0x00FFFF;
static const uint32_t C_OFF    = 0x000000;

// =============================================================================
//  SBUS state
// =============================================================================
int           sbusValues[24]        = {0};
unsigned long sbusFrameCount        = 0;
unsigned long sbusLastFrameMs       = 0;
unsigned long sbusFpsLastSecond     = 0;
unsigned long sbusFpsCounter        = 0;
int           sbusFps               = 0;
bool          sbusFailsafe          = false;
bool          lostFrameOld          = false;
bool          sbusLiveDump          = false;
unsigned long sbusLiveDumpLastMs    = 0;

int oldValueMatrix  = 100;
int oldValueMode    = 100;

// =============================================================================
//  Mode state
// =============================================================================
int FunctionSwState = 1;   // 1=down, 2=mid, 3=up

// =============================================================================
//  Pending action queue
// =============================================================================
struct PendingAction {
  bool          active;
  unsigned long fireAt;
  RcAction      action;
};
#define PENDING_ACTION_SLOTS 8
PendingAction pendingActions[PENDING_ACTION_SLOTS];

// =============================================================================
//  USB Serial WebSerial monitor state
// =============================================================================
bool          wsMonitorActive   = false;
unsigned long wsMonitorLastSent = 0;
#define WS_MONITOR_INTERVAL_MS  50

// =============================================================================
//  Tap detection state
// =============================================================================
struct TapState {
  int           lastBtn         = 0;   // most recent button that was tapped (sticky across release)
  int           prevPollBtn     = 0;   // btn seen on the previous poll — for press-edge detection
  uint8_t       tapCount        = 0;
  unsigned long lastTapMs       = 0;
  bool          deferredPending = false;
  unsigned long deferredFireAt  = 0;
  int           deferredBtn     = 0;
  uint8_t       deferredTaps    = 0;
};
TapState tapState;

// Last-seen switch positions for change detection
int switchPrevPos[RC_NUM_SWITCHES];

// =============================================================================
//  Helpers
// =============================================================================
static inline int readSwitchPos(int sbusVal, uint8_t positions) {
  if (positions == 2) return (sbusVal > 900) ? 2 : 0;
  if (sbusVal < 340)  return 0;
  if (sbusVal > 680)  return 2;
  return 1;
}

static inline int readBoundSwitchSbus(int8_t swIdx) {
  if (swIdx < 0 || swIdx >= RC_NUM_SWITCHES) return -1;
  int ch = rcConfig.switches[swIdx].channel;
  if (ch < 1 || ch > 24) return -1;
  return sbusValues[ch - 1];
}

int pwmToButton(int val) {
  for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
    if (val >= rcConfig.thresholds[i].minPwm && val <= rcConfig.thresholds[i].maxPwm)
      return i + 1;
  }
  return 0;
}

static inline uint16_t sbusToRange(int sbusVal, uint16_t outMin, uint16_t outMax) {
  long mapped = (long)(sbusVal - 172) * (outMax - outMin) / (1811 - 172) + outMin;
  if (mapped < (long)outMin) mapped = outMin;
  if (mapped > (long)outMax) mapped = outMax;
  return (uint16_t)mapped;
}

// =============================================================================
//  Maestro byte-level dispatcher
//
//  Writes a Pololu Maestro command to the destination stream for slot
//  `id` (1-8).  The slot's runtime location (type + device #) is looked up
//  in rcConfig.maestros[].
//
//    type = 1 → bytes go to Serial2 (local wired Maestro)
//    type = 2 → bytes go to the broadcast WCBStream (every WCB forwards)
//    type = 0 → slot disabled, command silently dropped
//
//  This firmware ALWAYS uses Pololu protocol — every Maestro on the network
//  is assumed to have its own device number (0-127), set in Maestro Control
//  Center. Compact protocol is not supported because in our broadcast model
//  multiple Maestros can share the same physical bus; the device-number
//  filter is what keeps them from all responding to every command.
//
//  Pololu protocol frame: 0xAA <device> <cmd_compact & 0x7F> <payload...>
// =============================================================================
static void maestroWrite(uint8_t id, uint8_t cmd_compact,
                         const uint8_t* payload, size_t plen) {
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  const RcMaestroSlot& slot = rcConfig.maestros[id - 1];
  if (slot.type == 0) return;          // disabled
  if (slot.device > 127) return;       // invalid Pololu device #

  Stream* dest = (slot.type == 1) ? (Stream*)&Serial2 : (Stream*)&maestroBroadcast;
  uint8_t hdr[3] = { 0xAA, slot.device, (uint8_t)(cmd_compact & 0x7F) };
  dest->write(hdr, 3);
  if (payload && plen) dest->write(payload, plen);
}

// Pololu Maestro command byte values (compact protocol).
//   0x84 SET_TARGET   · 0x87 SET_SPEED  · 0x89 SET_ACCEL
//   0xA2 GO_HOME      · 0xA4 STOP_SCRIPT · 0xA7 RESTART_SCRIPT_AT_SUB
static void maestroSetTarget(uint8_t id, uint8_t ch, uint16_t pos) {
  uint8_t p[3] = { ch, (uint8_t)(pos & 0x7F), (uint8_t)((pos >> 7) & 0x7F) };
  maestroWrite(id, 0x84, p, 3);
}
static void maestroSetSpeed(uint8_t id, uint8_t ch, uint16_t spd) {
  uint8_t p[3] = { ch, (uint8_t)(spd & 0x7F), (uint8_t)((spd >> 7) & 0x7F) };
  maestroWrite(id, 0x87, p, 3);
}
static void maestroSetAccel(uint8_t id, uint8_t ch, uint8_t accel) {
  uint8_t p[2] = { ch, accel };
  maestroWrite(id, 0x89, p, 2);
}
static void maestroGoHome(uint8_t id)        { maestroWrite(id, 0xA2, nullptr, 0); }
static void maestroStopScript(uint8_t id)    { maestroWrite(id, 0xA4, nullptr, 0); }
static void maestroRestartScript(uint8_t id, uint8_t sub) {
  maestroWrite(id, 0xA7, &sub, 1);
}

// Parse and execute a Maestro action command string against slot `id` (1-8).
// cmd: "setTarget,ch,pos" | "goHome" | "stopScript" | "restartScript,n"
//      | "setSpeed,ch,spd" | "setAccel,ch,acc"
static void executeMaestroCmd(uint8_t id, const char* cmd) {
  char buf[32];
  strlcpy(buf, cmd, sizeof(buf));
  char* tok = strtok(buf, ",");
  if (!tok) return;
  if      (strcmp(tok, "goHome")        == 0) maestroGoHome(id);
  else if (strcmp(tok, "stopScript")    == 0) maestroStopScript(id);
  else if (strcmp(tok, "setTarget")     == 0) {
    char* sCh = strtok(nullptr, ","); char* sPos = strtok(nullptr, ",");
    if (sCh && sPos) maestroSetTarget(id, (uint8_t)atoi(sCh), (uint16_t)atoi(sPos));
  }
  else if (strcmp(tok, "setSpeed")      == 0) {
    char* sCh = strtok(nullptr, ","); char* sSpd = strtok(nullptr, ",");
    if (sCh && sSpd) maestroSetSpeed(id, (uint8_t)atoi(sCh), (uint16_t)atoi(sSpd));
  }
  else if (strcmp(tok, "setAccel")      == 0) {
    char* sCh = strtok(nullptr, ","); char* sAcc = strtok(nullptr, ",");
    if (sCh && sAcc) maestroSetAccel(id, (uint8_t)atoi(sCh), (uint8_t)atoi(sAcc));
  }
  else if (strcmp(tok, "restartScript") == 0) {
    char* sN = strtok(nullptr, ",");
    maestroRestartScript(id, sN ? (uint8_t)atoi(sN) : 0);
  }
}

// =============================================================================
//  Serial port write helpers (S3, S4 only — S5 is now SBUS OUT)
// =============================================================================
void writeS3(const String& s) { for (char c : (s + '\r')) Serial3.write(c); }
void writeS4(const String& s) { for (char c : (s + '\r')) Serial4.write(c); }

// =============================================================================
//  HCR command formatter
//
//  Reimplements the byte-string format used by HCRVocalizer::sendCommand() so
//  we can ship the same payload through wcb.sendRaw() for WCB transport. Keep
//  this in sync with the HCR library if its protocol ever changes.
//
//  fn / chan / track follow the same convention as BC firmware's HCRFunction()
//  dispatcher:
//     2  SetEmotion(e, v)
//     3  Trigger(e, v)         (same payload as Stimulate)
//     4  Stimulate(e, v)
//     5  Overload()
//     6  Muse()
//     8  Stop all              (Stops V/A/B audio + emotes)
//     9  StopEmote()
//    11  ResetEmotions()
//    14  PlayWAV(ch, track)
//    16  StopWAV(ch)
//    17  SetVolume(ch, vol)
//
//  Returns an empty String for unknown fn or bad parameters.
// =============================================================================
static String hcrFormatCommand(uint8_t fn, int chan, int track) {
  static const char emoteprefix[] = "HSMC";   // HAPPY / SAD / MAD / sCared
  static const char audioprefix[] = "VAB";    // Vocalizer / A / B
  String inner;
  switch (fn) {
    case 2: {  // SetEmotion(e, v)
      if (chan < 0 || chan > 3 || track < 0 || track > 99) return "";
      inner = String("O") + emoteprefix[chan] + String(track) + ",QE" + emoteprefix[chan];
      break;
    }
    case 3:    // Trigger — same payload as Stimulate
    case 4: {  // Stimulate(e, v)
      if (chan < 0 || chan > 4 || track < 0 || track > 99) return "";
      if (chan == 4) {  // emotion 4 is Overload shortcut
        inner = "SE,QT";
      } else {
        inner = String("S") + emoteprefix[chan] + String(track) + ",QE" + emoteprefix[chan] + ",QT";
      }
      break;
    }
    case 5:  inner = "SE,QT";          break;  // Overload
    case 6:  inner = "MM";             break;  // single Muse
    case 8:  inner = "PSV,PSA,PSB,QT"; break;  // Stop (all audio + emote)
    case 9:  inner = "PSV,QT";         break;  // StopEmote
    case 11: inner = "OR,QE";          break;  // ResetEmotions
    case 14: {  // PlayWAV(ch, track) — file number is 0-padded to 4 digits
      if (chan < 0 || chan > 2 || track < 0 || track > 9999) return "";
      char file[8]; snprintf(file, sizeof(file), "%04d", track);
      inner = String("C") + audioprefix[chan] + file + ",QP" + audioprefix[chan];
      break;
    }
    case 16: {  // StopWAV(ch)
      if (chan < 0 || chan > 2) return "";
      inner = String("PS") + audioprefix[chan] + ",QP" + audioprefix[chan];
      break;
    }
    case 17: {  // SetVolume(ch, vol)
      if (chan < 0 || chan > 2 || track < 0 || track > 99) return "";
      inner = String("PV") + audioprefix[chan] + String(track);
      break;
    }
    default: return "";
  }
  return String("<") + inner + ">\n";
}

// Dispatch an HCR action.
//
// The destination is GLOBAL — pulled from rcConfig.hcrDest rather than from
// the action itself. This lets every HCR action share one configured
// vocalizer wiring; the action only carries fn/chan/track.
static void executeHcrAction(const RcAction& a) {
  const RcHcrDest& dest = rcConfig.hcrDest;

  if (dest.transport == 1) {
    // ── WCB transport (raw forward via ESP-NOW) ────────────────────────────
    String payload = hcrFormatCommand(a.fn, a.chan, a.track);
    if (payload.length() == 0) {
      Serial.printf("[DISPATCH] HCR-WCB: bad fn=%u chan=%d track=%d — skipped\n",
                    a.fn, a.chan, a.track);
      return;
    }
    uint8_t target = (uint8_t)atoi(dest.target);
    Serial.printf("[DISPATCH] HCR→WCB%u:port%u  %s",
                  target, dest.wcbPort, payload.c_str());
    if (target == 0) {
      wcb.broadcast(payload.c_str());
    } else if (target >= 1 && target <= WCB_MAX_BOARDS) {
      wcb.sendRaw(target, dest.wcbPort,
                  (const uint8_t*)payload.c_str(), payload.length());
    }
    return;
  }

  // ── Local serial transport (HCRVocalizer handles the framing) ─────────────
  HCRVocalizer* hcr = nullptr;
  if      (!strcmp(dest.target, "S3")) hcr = &hcrS3;
  else if (!strcmp(dest.target, "S4")) hcr = &hcrS4;
  // S5 is now SBUS OUT — no HCR routing available there. Legacy configs
  // that point HCR at S5 will fall through to the "unknown port" log below.
  if (!hcr) {
    Serial.printf("[DISPATCH] HCR: unknown serial port '%s' — skipped\n", dest.target);
    return;
  }

  Serial.printf("[DISPATCH] HCR→%s  fn=%u chan=%d track=%d\n",
                dest.target, a.fn, a.chan, a.track);
  switch (a.fn) {
    case 2:  hcr->SetEmotion(a.chan, a.track); hcr->update(); break;
    case 3:  hcr->Trigger   (a.chan, a.track);                break;
    case 4:  hcr->Stimulate (a.chan, a.track);                break;
    case 5:  hcr->Overload  ();                               break;
    case 6:  hcr->Muse      ();                               break;
    case 8:  hcr->Stop      ();                               break;
    case 9:  hcr->StopEmote ();                               break;
    case 11: hcr->ResetEmotions();                            break;
    case 14: hcr->PlayWAV   (a.chan, a.track); hcr->update(); break;
    case 16: hcr->StopWAV   (a.chan);                         break;
    case 17: hcr->SetVolume (a.chan, a.track);                break;
    default: Serial.printf("[DISPATCH] HCR: unsupported fn=%u\n", a.fn); break;
  }
}

// =============================================================================
//  rcExecuteAction — single-action dispatcher
// =============================================================================
static void scheduleAction(const RcAction& action, unsigned long delayMs) {
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (!pendingActions[i].active) {
      pendingActions[i] = { true, millis() + delayMs, action };
      return;
    }
  }
  Serial.println("WARN: pendingActions full — executing immediately");
}

void rcExecuteAction(const RcAction& a) {
  if (a.delayMs > 0) { scheduleAction(a, a.delayMs); return; }

  switch (a.type) {
    case RA_WCB_UNICAST: {
      uint8_t boardId = (uint8_t)atoi(a.target);
      if (boardId >= 1 && boardId <= WCB_MAX_BOARDS) {
        Serial.printf("[DISPATCH] WCB→%d  %s\n", boardId, a.cmd);
        wcb.send(boardId, a.cmd);
      }
      break;
    }
    case RA_WCB_BROADCAST:
      Serial.printf("[DISPATCH] WCB broadcast  %s\n", a.cmd);
      wcb.broadcast(a.cmd);
      break;

    case RA_MAESTRO_LOCAL:
      // Legacy "local Maestro" — treat as Maestro ID 1 for backward compat
      // with old configs.  The location of Maestro 1 (and whether it's
      // actually wired locally) is now defined in the Maestro Locations panel.
      Serial.printf("[DISPATCH] Maestro (legacy local → ID 1)  %s\n", a.cmd);
      executeMaestroCmd(1, a.cmd);
      break;

    case RA_MAESTRO_REMOTE: {
      // RA_MAESTRO_REMOTE is now the unified "Maestro" action — target holds
      // the Maestro slot ID (1-8). Wiring per ID is in rcConfig.maestros[].
      int id = atoi(a.target);
      if (id < 1 || id > RC_NUM_MAESTROS) {
        Serial.printf("WARN: Maestro action with invalid ID %d (target='%s')\n", id, a.target);
        break;
      }
      Serial.printf("[DISPATCH] Maestro %d  %s\n", id, a.cmd);
      executeMaestroCmd((uint8_t)id, a.cmd);
      break;
    }
    case RA_SERIAL: {
      String s(a.cmd);
      Serial.printf("[DISPATCH] Serial %s  %s\n", a.target, a.cmd);
      if      (!strcmp(a.target, "S3")) writeS3(s);
      else if (!strcmp(a.target, "S4")) writeS4(s);
      // S5 is reserved for SBUS OUT — actions targeting S5 are silently
      // ignored (legacy configs may still reference it).
      break;
    }
    case RA_HCR:
      executeHcrAction(a);
      break;
    default: break;
  }
}

void checkPendingActions() {
  unsigned long now = millis();
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (pendingActions[i].active && now >= pendingActions[i].fireAt) {
      pendingActions[i].active = false;
      rcExecuteAction(pendingActions[i].action);
    }
  }
}

// =============================================================================
//  RC dispatch by buttonId + tap count
// =============================================================================
void rcDispatch(int buttonId, uint8_t tapCount) {
  int mode = buttonId / 100, btn = buttonId % 100;
  if (mode < 1 || mode > 3 || btn < 1 || btn > 19) return;
  if (tapCount < 1 || tapCount > 3) return;
  const RcMapping& mapping = rcConfig.mappings[rcMapIndex(mode, btn)];
  if (mapping.exclusive) {
    // Exclusive: only the matched tier fires (e.g. double-tap fires t2 alone).
    const RcTier& tier = mapping.t[tapCount - 1];
    for (int i = 0; i < tier.count; i++) rcExecuteAction(tier.a[i]);
  } else {
    // Non-exclusive: cumulative — every tier up to and including the matched
    // one fires (e.g. double-tap fires t1 then t2; triple-tap fires t1+t2+t3).
    for (int ti = 0; ti < tapCount; ti++) {
      const RcTier& tier = mapping.t[ti];
      for (int i = 0; i < tier.count; i++) rcExecuteAction(tier.a[i]);
    }
  }
}

// =============================================================================
//  Matrix button tap detection
// =============================================================================
void RCRadio_Matrix_Buttons(int val) {
  int btn = pwmToButton(val);
  unsigned long now = millis();
  int prev = tapState.prevPollBtn;
  tapState.prevPollBtn = btn;

  // Only act on transitions. A continuous hold (prev == btn) is not a new tap,
  // and a release edge (btn == 0) is handled by checkDeferredTap() — we don't
  // count "release" as a tap event.
  if (btn == prev) return;
  if (btn == 0)    return;

  // Press edge: prev was 0 or a different button, now btn is non-zero.
  if (btn != tapState.lastBtn) {
    // Different button — commit any pending fire for the previous button now
    // before we start counting taps on the new one.
    if (tapState.deferredPending) {
      tapState.deferredPending = false;
      rcDispatch(tapState.deferredBtn, tapState.deferredTaps);
    }
    tapState.lastBtn   = btn;
    tapState.tapCount  = 1;
    tapState.lastTapMs = now;
  } else {
    // Same button as the last tap — within window counts as another tap.
    if ((now - tapState.lastTapMs) < (unsigned long)rcConfig.tapWindowMs) {
      tapState.tapCount++;
      if (tapState.tapCount > 3) tapState.tapCount = 3;
    } else {
      tapState.tapCount = 1;
    }
    tapState.lastTapMs = now;
  }

  // Arm (or refresh) the deferred fire — both exclusive and non-exclusive modes
  // wait the tap window before dispatching so multi-taps have time to register.
  tapState.deferredPending = true;
  tapState.deferredFireAt  = now + rcConfig.tapWindowMs;
  tapState.deferredBtn     = FunctionSwState * 100 + btn;
  tapState.deferredTaps    = tapState.tapCount;
}

void checkDeferredTap() {
  if (!tapState.deferredPending) return;
  if (millis() >= tapState.deferredFireAt) {
    tapState.deferredPending = false;
    rcDispatch(tapState.deferredBtn, tapState.deferredTaps);
    tapState.tapCount = 0;
    tapState.lastBtn  = 0;
  }
}

// =============================================================================
//  Switch processing
// =============================================================================
void processSwitches() {
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    RcSwitch& sw = rcConfig.switches[i];
    if (sw.channel < 1 || sw.channel > 24) continue;
    int pos = readSwitchPos(sbusValues[sw.channel - 1], sw.positions);
    if (pos == switchPrevPos[i]) continue;
    switchPrevPos[i] = pos;
    RcTier& tier = sw.t[pos];
    for (int ai = 0; ai < tier.count; ai++) rcExecuteAction(tier.a[ai]);
  }
}

// =============================================================================
//  Knob processing
//
//  Each knob/joystick-axis samples its SBUS channel once and fans the value
//  out to every configured output. The dispatch path is chosen by the knob's
//  function:
//
//    KF_MAESTRO_PASSTHROUGH  → each output.target is a Maestro ID (1-8) and
//                              maestroCh/posMin/posMax describe the servo.
//    KF_HCR_VOLUME           → each output.target is an HCR audio chan
//                              (0=V, 1=A, 2=B); posMin/posMax are volume
//                              endpoints (0-99). Rate-limited to avoid
//                              saturating ESP-NOW / HCR serial input.
//
//  ⚠ At 8 sources × 8 outputs × ~70Hz SBUS rate this could emit up to 4480
//  Maestro.setTarget() calls/sec. Keep active outputs reasonable.
// =============================================================================

// HCR volume rate-limiter — only re-emit when value changes AND a minimum
// interval has elapsed.  One slot per audio channel (V/A/B).
struct HcrVolCache {
  int8_t        lastVol[3] = { -1, -1, -1 };
  unsigned long lastSent[3] = { 0, 0, 0 };
};
static HcrVolCache hcrVolCache;
static const unsigned long HCR_VOLUME_MIN_INTERVAL_MS = 80;  // ~12 Hz max per chan

static void dispatchHcrVolume(uint8_t audioChan, uint8_t vol) {
  if (audioChan > 2) return;
  if (vol > 99) vol = 99;
  if ((int8_t)vol == hcrVolCache.lastVol[audioChan]) return;
  unsigned long now = millis();
  if (now - hcrVolCache.lastSent[audioChan] < HCR_VOLUME_MIN_INTERVAL_MS) return;
  hcrVolCache.lastVol[audioChan]  = vol;
  hcrVolCache.lastSent[audioChan] = now;
  // Build a synthetic RA_HCR action and reuse the existing dispatcher so
  // it automatically honors rcConfig.hcrDest (serial vs WCB transport).
  RcAction a{};
  a.type  = RA_HCR;
  a.fn    = 17;        // SetVolume
  a.chan  = (int8_t)audioChan;
  a.track = (int16_t)vol;
  executeHcrAction(a);
}

void processKnobs() {
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    RcKnob& kn = rcConfig.knobs[i];
    if (kn.channel < 1 || kn.channel > 24) continue;
    if (kn.function == KF_NONE || kn.outputCount == 0) continue;
    uint16_t raw = sbusValues[kn.channel - 1];

    for (uint8_t o = 0; o < kn.outputCount && o < RC_KNOB_MAX_OUTPUTS; o++) {
      const RcKnobOutput& out = kn.outputs[o];
      uint16_t mapped = sbusToRange(raw, out.posMin, out.posMax);
      if (kn.function == KF_MAESTRO_PASSTHROUGH) {
        // out.target is the Maestro slot ID (1-8)
        maestroSetTarget(out.target, out.maestroCh, mapped);
      } else if (kn.function == KF_HCR_VOLUME) {
        // out.target is the HCR audio chan (0/1/2); mapped is volume 0-99
        dispatchHcrVolume(out.target, (uint8_t)mapped);
      }
    }
  }
}

// =============================================================================
//  SBUS frame processing
// =============================================================================
void processSbus() {
  if (!sbusRx.read()) return;

  sbusFrameCount++;
  sbusLastFrameMs = millis();
  sbusFpsCounter++;
  sbusFailsafe  = sbusRx.failsafe;
  lostFrameOld  = sbusRx.lostFrame;
  for (int i = 0; i < 24; i++) sbusValues[i] = sbusRx.channels[i];

  // Mode selector
  int modeVal = readBoundSwitchSbus(rcConfig.funcBindings.modeSwitch);
  if (modeVal >= 0 && abs(modeVal - oldValueMode) > 5) {
    oldValueMode = modeVal;
    if      (modeVal < 340) FunctionSwState = 1;
    else if (modeVal < 680) FunctionSwState = 2;
    else                    FunctionSwState = 3;
  }

  // Button matrix
  int mxCh = rcConfig.matrixChannel;
  if (mxCh >= 1 && mxCh <= 24) {
    int mxVal = sbusValues[mxCh - 1];
    if (abs(mxVal - oldValueMatrix) >= 5) {
      oldValueMatrix = mxVal;
      RCRadio_Matrix_Buttons(mxVal);
    }
  }

  processSwitches();
  processKnobs();
}

// =============================================================================
//  SBUS FPS tracking
// =============================================================================
void trackSbusFps() {
  unsigned long now = millis();
  if (now - sbusFpsLastSecond >= 1000) {
    sbusFps           = (int)sbusFpsCounter;
    sbusFpsCounter    = 0;
    sbusFpsLastSecond = now;
  }
}

// =============================================================================
//  SBUS state dump (#L09)
// =============================================================================
void dumpSbusState() {
  unsigned long ageMs = (sbusLastFrameMs == 0) ? 0 : (millis() - sbusLastFrameMs);
  Serial.println("---- SBUS STATE ----");
  Serial.printf("  variant=%s (%d ch, %d-byte frame)\n",
                sbusRx.detectedChCount == 24 ? "SBUS-24" :
                sbusRx.detectedChCount == 16 ? "SBUS-16" : "(none yet)",
                sbusRx.detectedChCount, sbusRx.detectedFrameLen);
  Serial.printf("  frames=%lu  fps=%d  ageMs=%lu  lost=%s  failsafe=%s\n",
                sbusFrameCount, sbusFps, ageMs,
                lostFrameOld ? "YES" : "no", sbusFailsafe ? "YES" : "no");
  for (int r = 0; r < 3; r++) {
    int base = r * 8;
    if (base >= sbusRx.detectedChCount) break;
    Serial.printf("  CH%d-%d: ", base + 1, base + 8);
    for (int i = base; i < base + 8 && i < 24; i++)
      Serial.printf("%4d ", sbusValues[i]);
    Serial.println();
  }
}

// =============================================================================
//  WCB receive callback
// =============================================================================
void onWCBCommand(uint8_t senderID, const char* command) {
  Serial.printf("[WCB RX] from WCB%d: %s\n", senderID, command);
  // Extend here to route inbound WCB commands to local actions if needed
}

// =============================================================================
//  PWM monitor — streams to USB Serial while wsMonitorActive
// =============================================================================
void sendPWMUpdate() {
  if (!wsMonitorActive) return;
  unsigned long now = millis();
  if (now - wsMonitorLastSent < WS_MONITOR_INTERVAL_MS) return;
  wsMonitorLastSent = now;

  unsigned long ageMs = (sbusLastFrameMs == 0) ? 99999 : (now - sbusLastFrameMs);
  bool sbusOk = (sbusFps > 0) && !lostFrameOld && (ageMs < 500);

  char channelBuf[300] = "";
  int chCount = sbusRx.detectedChCount > 24 ? 24 : sbusRx.detectedChCount;
  for (int i = 0; i < chCount; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d%s", sbusValues[i], (i + 1 < chCount) ? "," : "");
    strncat(channelBuf, tmp, sizeof(channelBuf) - strlen(channelBuf) - 1);
  }

  int modeCh = 0;
  if (rcConfig.funcBindings.modeSwitch >= 0 &&
      rcConfig.funcBindings.modeSwitch < RC_NUM_SWITCHES)
    modeCh = rcConfig.switches[rcConfig.funcBindings.modeSwitch].channel;

  char buf[640];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"PWM_UPDATE\",\"matrixCh\":%d,\"modeCh\":%d,\"matrixVal\":%d,"
    "\"modeVal\":%d,\"btn\":%d,\"mode\":%d,"
    "\"sbus\":{\"ok\":%s,\"fps\":%d,\"frames\":%lu,\"ageMs\":%lu,"
    "\"lost\":%s,\"failsafe\":%s,\"chCount\":%d,\"frameLen\":%d,\"channels\":[%s]}}",
    rcConfig.matrixChannel, modeCh, oldValueMatrix, oldValueMode,
    pwmToButton(oldValueMatrix), FunctionSwState,
    sbusOk ? "true" : "false", sbusFps, sbusFrameCount, ageMs,
    lostFrameOld ? "true" : "false", sbusFailsafe ? "true" : "false",
    sbusRx.detectedChCount, sbusRx.detectedFrameLen, channelBuf);
  Serial.println(buf);
}

// =============================================================================
//  USB Serial WebSerial protocol handler
//  Same JSON protocol as Body Controller so the config_tool HTML is compatible.
// =============================================================================
String serialInputBuf;

void handleSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialInputBuf.trim();
      if (serialInputBuf.length() == 0) { serialInputBuf = ""; return; }

      if (serialInputBuf[0] == '{') {
        // ── JSON WebSerial protocol ──────────────────────────────────────────
        // Only extract the top-level "type" field here. SET_CONFIG payloads
        // can be tens of KB once mappings/knobs/maestros are populated, so
        // the header doc has to use a Filter — otherwise it tries to fit the
        // whole payload into the small buffer and returns NoMemory, which
        // surfaces as a generic "parse failed" error and breaks SET_CONFIG.
        StaticJsonDocument<32> filter;
        filter["type"] = true;
        DynamicJsonDocument hdr(192);
        DeserializationError perr = deserializeJson(
            hdr, serialInputBuf,
            DeserializationOption::Filter(filter));
        if (perr != DeserializationError::Ok) {
          // Include the received-buffer length so the host can spot truncation
          // (host-sent length vs. received length mismatch → USB RX overflow).
          Serial.printf("{\"type\":\"ERROR\",\"msg\":\"JSON parse failed (%s)\",\"rxLen\":%u}\n",
                        perr.c_str(), (unsigned)serialInputBuf.length());
          serialInputBuf = ""; return;
        }
        const char* type = hdr["type"] | "";

        if (strcmp(type,"PING")==0 || strcmp(type,"ping")==0) {
          Serial.println("{\"type\":\"PONG\"}");

        } else if (strcmp(type,"GET_CONFIG")==0) {
          Serial.print("{\"type\":\"CONFIG\",\"data\":");
          Serial.print(rcConfigToJSON());
          Serial.println("}");

        } else if (strcmp(type,"SET_CONFIG")==0) {
          DynamicJsonDocument bigDoc(98304);
          if (deserializeJson(bigDoc, serialInputBuf) != DeserializationError::Ok) {
            Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"parse failed\"}");
          } else if (!bigDoc.containsKey("data")) {
            Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"missing data\"}");
          } else {
            bool ok = rcConfigFromJSON(bigDoc["data"].as<JsonObject>());
            if (ok) {
              rcConfigSaveNVS();
              Serial.println("{\"type\":\"ACK\",\"ok\":true}");
            } else {
              Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"config apply failed\"}");
            }
          }

        } else if (strcmp(type,"START_MONITOR")==0) {
          wsMonitorActive   = true;
          wsMonitorLastSent = 0;
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"STOP_MONITOR")==0) {
          wsMonitorActive = false;
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"RESET_DEFAULTS")==0) {
          rcConfigLoadDefaults();
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"TRIGGER")==0) {
          int mode = hdr["mode"] | 1;
          int btn  = hdr["btn"]  | 0;
          int tap  = hdr["tap"]  | 1;
          if (btn < 1 || btn > 19 || mode < 1 || mode > 3 || tap < 1 || tap > 3) {
            Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"bad mode/btn/tap\"}");
          } else {
            Serial.printf("[TRIGGER] mode=%d btn=%d tap=%d\n", mode, btn, tap);
            rcDispatch(mode * 100 + btn, (uint8_t)tap);
            Serial.println("{\"type\":\"ACK\",\"ok\":true}");
          }

        } else if (strcmp(type,"WCB_SEND")==0) {
          int target     = hdr["target"] | 0;
          const char* cmd = hdr["cmd"]   | "";
          if (target == 0)                               wcb.broadcast(cmd);
          else if (target >= 1 && target <= WCB_MAX_BOARDS) wcb.send((uint8_t)target, cmd);
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else {
          Serial.println("{\"type\":\"ERROR\",\"msg\":\"unknown type\"}");
        }

      } else if (serialInputBuf[0] == '#') {
        // ── CLI commands ─────────────────────────────────────────────────────
        if (serialInputBuf.length() >= 3 &&
            (serialInputBuf[1] == 'L' || serialInputBuf[1] == 'l')) {
          int fn = 0;
          if (serialInputBuf.length() >= 4)
            fn = (serialInputBuf[2]-'0')*10 + (serialInputBuf[3]-'0');
          else
            fn = serialInputBuf[2] - '0';
          switch (fn) {
            case 1:  Serial.println("RC_Controller — WCB HW 3.2"); break;
            case 2:  ESP.restart(); break;
            case 9:  dumpSbusState(); break;
            case 10: sbusLiveDump = !sbusLiveDump;
                     Serial.printf("SBUS live dump %s\n", sbusLiveDump ? "ON (1Hz)" : "OFF"); break;
            case 11: {
              Serial.printf("WCB device ID: %d  quantity: %d\n", WCB_DEVICE_ID, WCB_QUANTITY);
              Serial.print("  Board status: ");
              for (int i = 1; i <= WCB_QUANTITY; i++)
                Serial.printf("WCB%d=%s ", i, wcb.isOnline(i) ? "UP" : "dn");
              Serial.println();
              break;
            }
            case 12: Serial.printf("Mode=%d  matrixBtn=%d  matrixVal=%d\n",
                                   FunctionSwState, pwmToButton(oldValueMatrix), oldValueMatrix); break;
          }
        }
      }
      serialInputBuf = "";
    } else {
      if (serialInputBuf.length() < 8192) serialInputBuf += c;
    }
  }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  // Bump the USB-CDC RX buffer well above the 256-byte default. The SBUS OUT
  // byte-streaming tee blocks the main loop for ~2.75 ms per SBUS frame while
  // bit-banging SoftwareSerial; during that window the host can shove ~1-2 KB
  // into us at USB-CDC speed. A 4 KB buffer comfortably absorbs the worst case
  // (e.g. a 3-4 KB SET_CONFIG payload arriving in one shot) without dropping
  // bytes that would otherwise corrupt the JSON. Must be set BEFORE begin().
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n\n=== RC_Controller (WCB HW 3.2) ===");

  // Status LED
  statusLed.begin();
  setStatusLed(C_RED, 255);

  // Serial2 — local Maestro
  Serial2.begin(LOCAL_MAESTRO_BAUD_RATE, SERIAL_8N1, MAESTRO_RX_PIN, MAESTRO_TX_PIN);
  Serial.printf("[Serial2] Local Maestro @ %d baud  TX=GPIO%d\n",
                LOCAL_MAESTRO_BAUD_RATE, MAESTRO_TX_PIN);

  // Serial3-4 — general purpose, bit-banged via SoftwareSerial.
  // 9600 8N1 by default; raise per-port baud if your peripheral needs it,
  // but keep under ~57600 — higher rates choke on ESP32-S3.
  Serial3.begin(9600, SWSERIAL_8N1, S3_RX_PIN, S3_TX_PIN, false, 95);
  Serial4.begin(9600, SWSERIAL_8N1, S4_RX_PIN, S4_TX_PIN, false, 95);

  // SBUS OUT — TX-only SoftwareSerial mirroring the SBUS RX line speed/format.
  // RX pin set to -1 (no RX), invert=true for SBUS line polarity.  The
  // SbusReader feeds each received byte into this sink ~110 µs after it
  // arrives on Serial1, giving downstream a one-byte-delayed copy of the
  // upstream SBUS stream.
  sbusOut.begin(100000, SWSERIAL_8E2, /*rxPin=*/-1, SBUS_OUT_PIN,
                /*invert=*/true, /*bufSize=*/64);

  // SBUS reader on Serial1 — wire sbusOut as the byte-streaming passthrough sink
  sbusRx.begin(&Serial1, SBUS_RX_PIN, SBUS_TX_PIN);
  sbusRx.setPassthroughSink(&sbusOut);
  Serial.printf("[SBUS] IN  on Serial1 RX (GPIO%d)\n", SBUS_RX_PIN);
  Serial.printf("[SBUS] OUT on SoftwareSerial TX (GPIO%d) — byte-streaming passthrough\n",
                SBUS_OUT_PIN);

  // WCB Client — sets STA mode + custom MAC + inits ESP-NOW
  // No WiFi AP or web server — ESP-NOW only.
  if (!wcb.begin()) {
    Serial.println("[WCB] ERROR: wcb.begin() failed — check wcb_config.h");
    setStatusLed(C_ORANGE, 200);
  } else {
    wcb.onCommand(onWCBCommand);
    Serial.printf("[WCB] Joined network as device ID %d (quantity=%d)\n",
                  WCB_DEVICE_ID, WCB_QUANTITY);
  }

  // RC Config: defaults then NVS overrides
  rcConfigLoadDefaults();
  rcConfigLoadNVS();

  // Clear state
  memset(pendingActions, 0, sizeof(pendingActions));
  memset(switchPrevPos, -1, sizeof(switchPrevPos));
  serialInputBuf.reserve(256);

  setStatusLed(C_BLUE, 10);
  Serial.println("[RC_Controller] Setup complete.");
  Serial.println("  Connect config_tool/index.html via Web Serial for configuration.");
  Serial.println("  CLI: #L01=info  #L09=SBUS dump  #L10=live  #L11=WCB status  #L12=RC state");
  Serial.println("  Send PING to test. Send GET_CONFIG to read mappings.");
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  // WCB — heartbeats, ACKs, WCBStream flushes
  wcb.update();

  // SBUS
  processSbus();
  checkDeferredTap();

  // Pending delayed actions
  checkPendingActions();

  // USB Serial WebSerial monitor stream
  sendPWMUpdate();

  // USB Serial input
  handleSerialInput();

  // FPS counter
  trackSbusFps();

  // 1Hz SBUS live dump (#L10)
  if (sbusLiveDump && (millis() - sbusLiveDumpLastMs >= 1000)) {
    sbusLiveDumpLastMs = millis();
    dumpSbusState();
  }
}
