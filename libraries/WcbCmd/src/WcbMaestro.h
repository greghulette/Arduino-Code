#pragma once
#include <Arduino.h>

// ── Maestro (Pololu) command translator ────────────────────────────────────────
// The WCB ";M<id><seq>" command triggers a Maestro script subroutine. This module
// turns it into the native Pololu wire frame, identically on any board, so the same
// string drives a Maestro whether it is reached over ESP-NOW (a WCB parses it) or
// out a host-local serial port (the host parses it).
//
// SCOPE: the ";M" verb is subroutine-trigger ONLY. Full Pololu control (Set Target /
// Speed / Accel) is NOT part of ";M" and stays in each firmware's own Maestro path.
//
// Reference (byte-identical to both):
//   WCB firmware  — WCB.ino processMaestroCommand + WCB_Maestro.cpp frame {0xAA,id,0x27,seq}
//   NaviCore      — NaviCore.ino local Maestro emit {0xAA, device, 0x27, seq}
namespace WcbMaestro {

// Parse a ";M" argument body into (maestroId, sequence). Tolerates an optional
// leading 'M'/'m', so the SAME token works locally and when relayed to a WCB:
//   "M25" or "25"  ->  id = 2, seq = 5.
// Matches the WCB firmware exactly: id is the FIRST digit; sequence is the remaining
// digits (empty -> 0, like String::toInt()). Returns false if there is no id digit
// or the sequence is out of the single-byte range (0-255).
bool parse(const char* payload, uint8_t& id, uint16_t& seq);

// Build the native Pololu "Restart Script at Subroutine" frame {0xAA, id, 0x27, seq}
// into out[] (exactly 4 bytes). Returns 4.
size_t buildSubroutineFrame(uint8_t id, uint8_t seq, uint8_t out[4]);

// Parse `payload` and write its frame to `out`. Returns false on a parse error, or
// if the stream dropped a byte. The one call a host needs to "run this ;M locally".
bool emit(Stream& out, const char* payload);

}  // namespace WcbMaestro
