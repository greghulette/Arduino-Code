/*
  MaestroForward.ino — WCBClient + WCBStream Example

  Demonstrates how to use WCBStream to transparently forward Pololu Maestro
  servo commands over the WCB ESP-NOW network — no serial wire, no hardware
  changes, no modifications to the Pololu library.

  ── How it works ─────────────────────────────────────────────────────────────
  The Pololu MiniMaestro (or MicroMaestro) library normally writes binary servo
  command packets to a serial port connected to the Maestro controller. Here,
  WCBStream is passed instead of a serial port. The Pololu library writes bytes
  to WCBStream exactly as it would to Serial1 — it doesn't know the difference.

  WCBStream buffers those bytes. When wcb.update() detects a gap (no new bytes
  for 2 ms), it knows the Pololu library has finished writing a complete command
  and calls sendRaw() to forward the buffered bytes wirelessly to the target WCB.

  The receiving WCB must have KYBER,REMOTE configured on the serial port
  connected to the physical Maestro controller. It forwards the raw bytes
  directly to the Maestro, which executes the command as if wired directly.

  ── Wiring ───────────────────────────────────────────────────────────────────
  This ESP32 (Kyber):   No serial wiring to Maestro needed on this end.
  Remote WCB:           Serial TX connected to Maestro RX, KYBER,REMOTE active.

  ── Network credentials ───────────────────────────────────────────────────────
  Fill in the values below to match your WCB system. Find them in the WCB
  Config Tool or by querying a WCB over serial (?WCBM, ?WCBP, ?WCBQ).
*/

#include <WCBClient.h>
#include <WCBStream.h>
#include <PololuMaestro.h>  // Provides MiniMaestro and MicroMaestro classes

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;   // ?WCBM second octet
const uint8_t MAC_OCT3     = 0x00;   // ?WCBM third octet
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 4;      // ?WCBQ — total WCBs in the system
const uint8_t DEVICE_ID    = 3;      // Unique ID for this device (must be <= WCB_QUANTITY)

// ─────────────────────────────────────────────────────────────────────────────
// Target WCBs, ports, and Maestro device numbers
//
// MAESTRO_WCB_x    : which WCB the Maestro is physically wired to
// MAESTRO_PORT_x   : which serial port on that WCB (1–5) is connected to the
//                    Maestro. Check your WCB wiring or the WCB Config Tool.
// MAESTRO_DEVICE_x : device number set in Maestro Control Center (Serial tab).
//                    This enables the Pololu protocol so commands include the
//                    0xAA start byte and device number — required when multiple
//                    Maestros share a serial bus or when your show control
//                    software addresses Maestros by device number.
//                    Set to Maestro::deviceNumberDefault (255) to use the
//                    compact protocol instead (no device addressing).
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAESTRO_WCB_1    = 2;  // WCB number connected to Maestro A
const uint8_t MAESTRO_PORT_1   = 1;  // Serial port on that WCB wired to Maestro A (1–5)
const uint8_t MAESTRO_DEVICE_1 = 1;  // Maestro A device number (from Maestro Control Center)

const uint8_t MAESTRO_WCB_2    = 2;  // WCB number connected to Maestro B
const uint8_t MAESTRO_PORT_2   = 2;  // Serial port on that WCB wired to Maestro B (1–5)
const uint8_t MAESTRO_DEVICE_2 = 2;  // Maestro B device number (from Maestro Control Center)

// ─────────────────────────────────────────────────────────────────────────────
// WCBClient and WCBStream instances
//
// Each WCBStream targets a different WCB+port pair and self-registers with wcb
// during construction. wcb.update() automatically flushes all streams — no
// extra calls needed in loop(). Up to 4 streams are supported simultaneously.
// ─────────────────────────────────────────────────────────────────────────────
WCBClient wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

WCBStream maestroStream1(wcb, MAESTRO_WCB_1, MAESTRO_PORT_1);  // bytes → WCBx serial y → Maestro A
WCBStream maestroStream2(wcb, MAESTRO_WCB_2, MAESTRO_PORT_2);  // bytes → WCBx serial y → Maestro B

// MiniMaestro 12/18/24 channel. The device number enables the Pololu protocol
// so each packet includes the 0xAA start byte and device number.
// Maestro::noResetPin = 255 means no hardware reset pin is used.
MiniMaestro maestro1(maestroStream1, Maestro::noResetPin, MAESTRO_DEVICE_1);
MiniMaestro maestro2(maestroStream2, Maestro::noResetPin, MAESTRO_DEVICE_2);

// ─────────────────────────────────────────────────────────────────────────────
// Optional: react to commands sent back from the WCB network to this device
// ─────────────────────────────────────────────────────────────────────────────
void onCommandReceived(uint8_t senderID, const char* command) {
    Serial.printf("[RX] Command from WCB%d: %s\n", senderID, command);
    // Parse the command here and call maestro methods if needed
}

unsigned long lastMoveMs = 0;

void setup() {
    Serial.begin(115200);
    wcb.begin();
    wcb.onCommand(onCommandReceived);
}

void loop() {
    // update() drives heartbeats, offline detection, AND the WCBStream flush.
    // Keep loop() free of blocking delays so this runs frequently.
    wcb.update();

    // Move servos on both Maestros back and forth every 2 seconds.
    // Each maestro.setTarget() writes bytes to its own WCBStream —
    // wcb.update() forwards each stream's buffer independently.
    if (millis() - lastMoveMs >= 2000) {
        lastMoveMs = millis();
        static bool toggle = false;
        toggle = !toggle;

        // setTarget uses quarter-microseconds: 6000 = 1500µs (centre)
        uint16_t pos = toggle ? 4000 : 8000;

        maestro1.setTarget(0, pos);
        Serial.printf("[TX] Maestro1 ch0 → %d\n", pos);

        maestro2.setTarget(0, pos);
        Serial.printf("[TX] Maestro2 ch0 → %d\n", pos);
    }
}
