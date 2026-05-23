/*
  MaestroUnicast.ino — WCB_Client + WCBStream Example

  Demonstrates forwarding Pololu Maestro servo commands to a SPECIFIC WCB's
  serial port over the WCB ESP-NOW network — no serial wire, no hardware
  changes, no modifications to the Pololu library.

  Use this when you know exactly which WCB has the Maestro wired to which port.
  For broadcasting to all WCBs with Maestros at once, see MaestroBroadcast.

  ── How it works ─────────────────────────────────────────────────────────────
  WCBStream is passed to the Pololu library instead of a serial port. The
  library writes binary command bytes to WCBStream exactly as it would to
  Serial1. WCBStream buffers those bytes and, when wcb.update() detects a
  2ms gap (end of packet), forwards the buffer wirelessly to the target WCB.
  The receiving WCB writes the bytes directly to its serial port → Maestro
  executes the command as if it were wired directly to this ESP32.

  ── Network credentials ───────────────────────────────────────────────────────
  Fill in the values below to match your WCB system. Find them via the WCB
  Config Tool or by querying a WCB over serial (?WCBM, ?WCBP, ?WCBQ).
*/

#include <WCB_Client.h>
#include <WCBStream.h>
#include <PololuMaestro.h>  // Pololu Maestro library by Pololu

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;   // ?WCBM second octet
const uint8_t MAC_OCT3     = 0x00;   // ?WCBM third octet
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 4;      // ?WCBQ — total WCBs in the system
const uint8_t DEVICE_ID    = 20;     // Unique ID for this device (20 = special slot)

// ─────────────────────────────────────────────────────────────────────────────
// Target WCBs and ports
//
// MAESTRO_WCB_x  : WCB number the Maestro is physically wired to
// MAESTRO_PORT_x : serial port on that WCB (1–5) connected to the Maestro RX
// MAESTRO_ID_x   : device number set in Maestro Control Center (Serial tab)
//                  Use Maestro::deviceNumberDefault (255) for compact protocol
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAESTRO_WCB_1  = 2;   // WCB2 has the first Maestro
const uint8_t MAESTRO_PORT_1 = 1;   // wired to Serial1 on WCB2
const uint8_t MAESTRO_ID_1   = 1;   // Maestro device number 1

const uint8_t MAESTRO_WCB_2  = 2;   // WCB2 also has the second Maestro
const uint8_t MAESTRO_PORT_2 = 2;   // wired to Serial2 on WCB2
const uint8_t MAESTRO_ID_2   = 2;   // Maestro device number 2

// ─────────────────────────────────────────────────────────────────────────────
// WCB_Client and WCBStream instances
//
// WCB_Client must be declared BEFORE WCBStream at global scope. Its constructor
// sets the singleton so WCBStream can find it automatically — no need to pass
// wcb as a parameter.
//
// Each WCBStream self-registers with WCB_Client so wcb.update() flushes them
// automatically. Up to 4 streams are supported simultaneously.
// ─────────────────────────────────────────────────────────────────────────────
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

WCBStream maestroStream1(MAESTRO_WCB_1, MAESTRO_PORT_1);  // → WCB2 Serial1 → Maestro 1
WCBStream maestroStream2(MAESTRO_WCB_2, MAESTRO_PORT_2);  // → WCB2 Serial2 → Maestro 2

// Pass WCBStream to the Pololu library instead of a hardware serial port
MiniMaestro maestro1(maestroStream1, Maestro::noResetPin, MAESTRO_ID_1);
MiniMaestro maestro2(maestroStream2, Maestro::noResetPin, MAESTRO_ID_2);

unsigned long lastMoveMs = 0;

void setup() {
    Serial.begin(115200);
    wcb.begin();
}

void loop() {
    // update() drives heartbeats and flushes all WCBStream buffers.
    // Keep loop() free of blocking delays so WCBStream flushes on time.
    wcb.update();

    // Move servos every 2 seconds
    if (millis() - lastMoveMs >= 2000) {
        lastMoveMs = millis();
        static bool toggle = false;
        toggle = !toggle;

        // setTarget uses quarter-microseconds: 4000=1000µs  6000=1500µs  8000=2000µs
        uint16_t pos = toggle ? 4000 : 8000;

        maestro1.setTarget(0, pos);
        maestro2.setTarget(0, pos);
        Serial.printf("[TX] Both Maestros ch0 → %d\n", pos);
    }
}
