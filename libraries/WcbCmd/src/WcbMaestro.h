#pragma once
#include <Arduino.h>

// ── Maestro (Pololu) command translator ────────────────────────────────────────
// Turns WCB ";M…" text commands into native Pololu (device-addressed) wire frames,
// identically on any board — so the SAME string drives a Maestro whether it is
// reached over ESP-NOW (a WCB parses it) or out a host-local serial port (NaviCore
// parses it). Shared by the WCB firmware and NaviCore via <WcbCmd.h>.
//
// ── ";M" grammar (backward compatible) ─────────────────────────────────────────
//   SUBROUTINE trigger — "restart script at subroutine", two equivalent spellings:
//        ;M<id><seq>            short form (no comma). id = the FIRST digit, subroutine =
//                               the REST of the digits; device is 0-9 only. ;M11 = dev 1
//                               sub 1 ; ;M25 = dev 2 sub 5.
//        ;M<dev>,<n>[,<param>]  comma form. dev = full Pololu range 0-127, n = subroutine
//                               0-255. ;M1,1 is IDENTICAL to ;M11 ; ;M12,5 reaches device
//                               12 (past the short form's range). Optional <param> uses
//                               "restart w/ parameter" (0x28).
//   VERB  (new):         ;M<dev>,<verb>[,args]  — full Pololu servo / script control
//        e.g. ;M2,setTarget,0,6000   ;M2,goHome   ;M2,setSpeed,0,10
//        device# here is the full Pololu range 0-127 (parsed up to the first comma).
//   The token AFTER the first comma decides the comma form: all-digits → subroutine,
//   a verb name → servo/query. No comma → the legacy short subroutine form.
//   Both forms address the Maestro by its Pololu device# (the frame embeds it), so a
//   built frame can be broadcast and only the matching Maestro obeys — location-
//   independent, exactly like the legacy path.
//   WCB/mesh convention: device IDs 1-8 address a Maestro; id 0 = ALL Maestros and id 9 =
//   all LOCAL Maestros are RESERVED routing targets (the WCB firmware routes them; a host
//   should not assign a Maestro to id 0 or 9). build() itself still accepts any 0-127
//   device# — the reservation is a routing convention, not a frame-construction limit.
//
// Verbs (v1):
//   setTarget,<ch>,<target¼µs>     0xAA d 0x04 ch tl th
//   setSpeed,<ch>,<speed>          0xAA d 0x07 ch sl sh
//   setAccel,<ch>,<accel 0-255>    0xAA d 0x09 ch al ah
//   goHome                         0xAA d 0x22
//   stopScript                     0xAA d 0x24         — stop the running Maestro script
//   sub,<n>[,<param>]              0xAA d 0x27 n   (or 0x28 n pl ph when <param> given)
//   getPosition,<ch>               0xAA d 0x10 ch      → Maestro REPLIES 2 bytes
//   getMovingState                 0xAA d 0x13         → REPLIES 1 byte
//   getErrors                      0xAA d 0x21         → REPLIES 2 bytes
// The get* verbs build ONLY the request frame; the Maestro then answers with raw bytes.
// Reading those reply bytes off serial is still the firmware's job (WcbCmd owns no RX
// pump), but the reply helpers below (replyInfo / decodeReply / formatReply) turn them
// into the ONE shared ":MQR" text a host parses — so the WCB relay and the host never
// disagree on the reply wire format.
//
// NOT yet included (add later with a VERIFIED command byte): bare restartScript (confirm
// the byte against the Maestro user guide first), Set Multiple Targets / Set PWM
// (Mini-Maestro-only, variable length). setSpeedAccel = just send setSpeed then setAccel.
// setEasing is not a native serial command (do it in a script).
//
// Reference (byte-identical to both):
//   WCB firmware  — WCB.ino processMaestroCommand + WCB_Maestro.cpp
//   NaviCore      — NaviCore.ino Maestro emit
namespace WcbMaestro {

// ── Pololu (device-addressed) protocol bytes (compact byte − 0x80) ──────────────
constexpr uint8_t POLOLU_LEAD       = 0xAA;
constexpr uint8_t CMD_SET_TARGET    = 0x04;
constexpr uint8_t CMD_SET_SPEED     = 0x07;
constexpr uint8_t CMD_SET_ACCEL     = 0x09;
constexpr uint8_t CMD_GET_POSITION  = 0x10;
constexpr uint8_t CMD_GET_MOVING    = 0x13;
constexpr uint8_t CMD_GET_ERRORS    = 0x21;
constexpr uint8_t CMD_GO_HOME       = 0x22;
constexpr uint8_t CMD_STOP_SCRIPT   = 0x24;   // Stop Script (Pololu compact 0xA4 − 0x80)
constexpr uint8_t CMD_RESTART_SUB   = 0x27;   // Restart Script at Subroutine (the legacy ;M)
constexpr uint8_t CMD_RESTART_SUB_P = 0x28;   // …with Parameter

constexpr size_t  MAX_FRAME = 6;   // largest v1 frame (setTarget/Speed/Accel/subParam)

// ── Legacy subroutine trigger (UNCHANGED — keeps ;M<id><seq> a valid command) ──
bool   parse(const char* payload, uint8_t& id, uint16_t& seq);
size_t buildSubroutineFrame(uint8_t id, uint8_t seq, uint8_t out[4]);

// ── Individual Pololu frame builders (dev = Pololu device# 0-127) ──────────────
// Each writes a device-addressed frame into out[] and returns its length (0 = bad args).
size_t buildSetTarget     (uint8_t dev, uint8_t ch, uint16_t target, uint8_t out[6]);  // ¼µs
size_t buildSetSpeed      (uint8_t dev, uint8_t ch, uint16_t speed,  uint8_t out[6]);
size_t buildSetAccel      (uint8_t dev, uint8_t ch, uint8_t  accel,  uint8_t out[6]);  // 0-255
size_t buildGoHome        (uint8_t dev,                               uint8_t out[3]);
size_t buildStopScript    (uint8_t dev,                               uint8_t out[3]);
size_t buildSubParam      (uint8_t dev, uint8_t sub, uint16_t param, uint8_t out[6]);
size_t buildGetPosition   (uint8_t dev, uint8_t ch,                   uint8_t out[4]);
size_t buildGetMovingState(uint8_t dev,                               uint8_t out[3]);
size_t buildGetErrors     (uint8_t dev,                               uint8_t out[3]);

// ── Text → frame (the core both hosts share) ───────────────────────────────────
// Parse a ";M" payload (legacy subroutine OR verb form; a leading 'M'/'m' is
// tolerated) and build its Pololu frame into out[] (need MAX_FRAME bytes). Returns
// the frame length, or 0 on parse error / unknown verb / bad args / too-small buffer.
size_t build(const char* payload, uint8_t* out, size_t outMax);

// build() + write the frame to a Stream (the local Maestro serial). Returns false on
// a parse error or a short write. For get* verbs this writes only the REQUEST frame.
bool emit(Stream& out, const char* payload);

// ── Query reply readback (POS / MOV / ERR) ─────────────────────────────────────
// The get* verbs above build only the REQUEST; the Maestro answers with raw little-endian
// bytes on the same serial line. Reading them (with a timeout) stays the firmware's job —
// WcbCmd owns no RX pump — but these turn the request byte + reply bytes into the ONE
// shared line   :MQR,<id>,<chan>,<KIND>,<value>   (KIND = POS | MOV | ERR)   that a host
// (NaviCore's maeConsumeRemoteReply) parses. Emitting it from the WCB relay guarantees the
// format never drifts from the parser. <id> = mesh Maestro id (1-8); <chan> = queried
// channel (0 for MOV/ERR); <value> decimal: POS = position ¼µs (u16), MOV = 0/1,
// ERR = error bitmask (u16).
enum class ReplyKind : uint8_t { POS, MOV, ERR };

constexpr size_t REPLY_TEXT_MAX = 28;   // ":MQR,255,255,ERR,65535" + NUL, with headroom

// Map a get* REQUEST command byte (the out[2] of a built get frame: 0x10 POS / 0x13 MOV /
// 0x21 ERR) to the reply it produces. false (kind/len untouched) if cmd expects no reply.
bool    replyInfo(uint8_t cmd, ReplyKind& kind, uint8_t& len);

// Bytes the Maestro sends back for a reply kind (POS=2, MOV=1, ERR=2).
uint8_t replyLen(ReplyKind kind);

// Decode the raw little-endian reply bytes into one value. false if bytes==nullptr or
// len < replyLen(kind). POS/ERR = bytes[0] | bytes[1]<<8 ; MOV = bytes[0] ? 1 : 0.
bool    decodeReply(ReplyKind kind, const uint8_t* bytes, size_t len, uint16_t& value);

// Format ":MQR,<id>,<chan>,<KIND>,<value>" into out[cap] (need REPLY_TEXT_MAX bytes).
// Returns the length written (excluding NUL), or 0 on bad args / short buffer.
size_t  formatReply(char* out, size_t cap, uint8_t id, uint8_t chan,
                    ReplyKind kind, const uint8_t* bytes, size_t len);

}  // namespace WcbMaestro
