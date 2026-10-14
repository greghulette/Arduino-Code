#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
//  WcbWled — ;L WLED verb -> WLED /json/state document   (WcbCmd Phase 3)
//
//  Stateless translator. Turns one ;L runtime verb (the body, WITHOUT the leading
//  L / id) into the exact WLED JSON the WCB emits today and writes it
//  newline-terminated to an injected Stream (WLED's serial parser frames on '\n').
//
//    ON | OFF | TOGGLE      -> {"on":true} / {"on":false} / {"on":"t"}
//    BRI,<0-255>            -> {"bri":N}
//    PS,<n>                 -> {"ps":N}                     (primary workflow)
//    COL,<RRGGBB[WW]>       -> {"seg":[{"col":[[r,g,b(,w)]]}]}   (segment 0)
//    FX,<n>[,<sx>,<ix>]     -> {"seg":[{"fx":N(,"sx":S)(,"ix":I)}]}
//    PAL,<n>                -> {"seg":[{"pal":N}]}
//    JSON,{...}             -> raw /json/state passthrough (verbatim, escape hatch)
//
//  Routing (the ;L<id> parse, slot lookup, ESP-NOW forward to a remote host),
//  ?WLED config, NVS and port reservation all stay in the firmware. This is the
//  byte-generating core of the WCB's wledDispatchLocal() — nothing more — so the
//  same ;L token drives a WLED identically whether a WCB parses it off the mesh or
//  a WCB_Client host (NaviCore) parses it out a local UART.
// ─────────────────────────────────────────────────────────────────────────────

namespace WcbWled {

// Build the WLED document for one verb body — the body WITHOUT the leading "L"/"L<id>,"
// (routing owns and strips the id). Returns "" (and logs to diag if given) on an
// empty/unknown verb or bad args — matching the WCB, which emits nothing to the WLED
// in those cases. Does NOT strip a leading 'L' (the WCB doesn't, so stripping would
// diverge on a malformed double-L token). Pure: no I/O.
String build(const String &cmd, Print *diag = nullptr);

// build() + write it to `out`, newline-terminated. Returns true iff a document was
// written. Byte-identical to the WCB's wledSend(out, json).
bool emit(Stream &out, const String &cmd, Print *diag = nullptr);

}  // namespace WcbWled
