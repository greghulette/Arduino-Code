#pragma once
// ═══════════════════════════════════════════════════════════════════════════════
//  WcbCmd — shared WCB command translators
//
//  ONE command vocabulary for the whole ecosystem. Each module turns a WCB
//  ";"-command verb into that device's NATIVE serial wire protocol, on an injected
//  Stream — nothing more. Compiled by BOTH the WCB firmware and a WCB_Client host
//  (NaviCore), so the SAME command string produces the SAME device bytes whether it
//  is reached across the mesh (a WCB parses it) or out a host-local serial port (the
//  host parses it). That is the whole point: one source of commands, two transports.
//
//  The library is PURE translation (verb parse -> native bytes). Everything else —
//  local-vs-remote routing, config slots, NVS, ESP-NOW relay, port reservation —
//  stays in each firmware. Devices are added a module at a time:
//     Phase 1: Maestro (;M)   ← shipping
//     Phase 2: MP3 (;A)       ← shipping
//     Phase 3: WLED (;L)      ← shipping
//     Phase 4: HCR (;H)       ← shipping  (device-wire formatter; no lib dependency)
//
//  TWO-COPY HAZARD: like WCB_Client, this library lives as two physical copies (the
//  Arduino-Code/libraries copy each sketch compiles, and the standalone repo). If
//  they drift, a WCB and the NaviCore would emit DIFFERENT bytes for the same
//  command — the exact opposite of the goal. Both sketches should assert
//  WCBCMD_VERSION so a stale copy fails loudly instead of drifting silently.
// ═══════════════════════════════════════════════════════════════════════════════

#define WCBCMD_VERSION "0.7.0"

#include "WcbMaestro.h"   // ;M  Maestro
#include "WcbMp3.h"       // ;A  MP3 Trigger
#include "WcbWled.h"      // ;L  WLED
#include "WcbHcr.h"       // ;H  HCR (numeric fn/chan/track -> device wire)
#include "WcbHcrFade.h"   // ;H  HCR non-blocking volume fade (shared ramp)
