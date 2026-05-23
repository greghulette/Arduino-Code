/*
  MaestroBroadcast.ino — WCB_Client + WCBStream Broadcast Example

  Demonstrates broadcasting Pololu Maestro servo commands to ALL WCBs on the
  network simultaneously using the  broadcast  constant. Any WCB that has a
  Maestro wired up and Kyber_Remote configured will execute the command.
  WCBs without a Maestro ignore it.

  ── Broadcast is the recommended approach for servo passthrough ───────────────
  Broadcast uses a single ESP-NOW packet that reaches every WCB at once,
  keeping latency low and the network uncongested regardless of how many
  Maestros are in the system. Unicast (MaestroUnicast) is a fine choice for
  low-frequency operations like triggering a RestartScript, but for continuous
  position updates or high-rate servo streaming, broadcast will give you
  noticeably better performance.

  Use this when:
    - You have Maestros on multiple WCBs and want to address all of them
    - You don't know (or don't care) which specific WCB has the Maestro
    - You want one command to ripple out to every servo controller at once
    - You need the best possible throughput for continuous servo updates

  For sending to one specific WCB:port, see MaestroUnicast.

  ── WCB setup required ───────────────────────────────────────────────────────
  On each WCB that has a Maestro physically wired to it:
    1. Configure the serial port:   ?MAESTRO,M1:W<n>S<port>:<baud>
    2. Enable Kyber remote mode:    ?KYBER,REMOTE,S<port>
  The WCB will then automatically forward any incoming Kyber broadcast to that
  serial port.

  ── Network credentials ───────────────────────────────────────────────────────
  Fill in the four values below to match your WCB system. Find them via the
  WCB Config Tool or by querying a WCB over serial (?WCBM, ?WCBP, ?WCBQ).
*/

#include <WCB_Client.h>
#include <WCBStream.h>
#include <PololuMaestro.h>  // Pololu Maestro library by Pololu

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 4;
const uint8_t DEVICE_ID    = 20;  // 20 = special out-of-band slot

// ─────────────────────────────────────────────────────────────────────────────
// Maestro device numbers
//
// Set each to match the device number configured in Maestro Control Center
// (Serial Settings tab). This enables the Pololu protocol so each packet
// includes the 0xAA start byte and the device number — the correct Maestro
// responds to its own address and ignores packets addressed to others.
//
// Use Maestro::deviceNumberDefault (255) for the compact protocol instead
// (only suitable when there is exactly one Maestro on the network).
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAESTRO_ID_1 = 1;
const uint8_t MAESTRO_ID_2 = 2;

// ─────────────────────────────────────────────────────────────────────────────
// WCB_Client and WCBStream — broadcast mode
//
// WCB_Client must be declared BEFORE WCBStream. Passing  broadcast  as the
// target tells WCBStream to use sendKyber() — one ESP-NOW packet reaches
// every WCB; those with Kyber_Remote forward the bytes to their Maestro.
//
// Each stream carries commands for a different Maestro device number.
// Because each MiniMaestro embeds its own device address in every packet,
// only the Maestro with the matching device number will act on the command.
// ─────────────────────────────────────────────────────────────────────────────
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

WCBStream maestroStream1(broadcast);
WCBStream maestroStream2(broadcast);

// Pass WCBStream to the Pololu library instead of a hardware serial port.
// The Pololu library writes bytes here; WCBStream forwards them wirelessly.
MiniMaestro maestro1(maestroStream1, Maestro::noResetPin, MAESTRO_ID_1);
MiniMaestro maestro2(maestroStream2, Maestro::noResetPin, MAESTRO_ID_2);

unsigned long lastMoveMs = 0;

void setup() {
    Serial.begin(115200);
    wcb.begin();
}

void loop() {
    // update() drives heartbeats and flushes both WCBStreams automatically.
    wcb.update();

    // Sweep both servo channel 0s back and forth every 2 seconds.
    // Each command reaches every WCB with a Maestro simultaneously;
    // the device number in each packet ensures the right Maestro responds.
    if (millis() - lastMoveMs >= 2000) {
        lastMoveMs = millis();
        static bool toggle = false;
        toggle = !toggle;

        // Quarter-microseconds: 4000=1000µs  6000=1500µs  8000=2000µs
        uint16_t pos = toggle ? 4000 : 8000;
        maestro1.setTarget(0, pos);
        maestro2.setTarget(0, pos);
        Serial.printf("[TX] Broadcast Maestros ch0 → %d (%s)\n",
                      pos, toggle ? "left" : "right");
    }
}
