/*
  CombinedUsage.ino — WCB_Client + WCBStream Combined Example

  Demonstrates text command sending/receiving AND Pololu Maestro servo
  forwarding running simultaneously on the same ESP32.

  ── What this sketch does ────────────────────────────────────────────────────
  Every 2 seconds : moves servos on two Maestros back and forth using
                    WCBStream broadcast — no serial wire required.
  Every 5 seconds : broadcasts a text command to all WCBs, and sends a
                    unicast text command to WCB1 (if it's online).
  Any time        : prints commands received from the WCB network.

  ── Broadcast is the recommended approach for servo passthrough ───────────────
  Broadcast uses a single ESP-NOW packet that reaches every WCB at once,
  keeping latency low and the network uncongested. Unicast is a fine choice
  for low-frequency operations like triggering a RestartScript, but for
  continuous position updates or high-rate servo streaming, broadcast gives
  noticeably better performance. This sketch uses broadcast for Maestro
  control and unicast only for the text command example.

  ── Network credentials ──────────────────────────────────────────────────────
  Fill in the values below to match your WCB system. Find them via the WCB
  Config Tool or by querying a WCB over serial (?WCBM, ?WCBP, ?WCBQ).

  ── WCB setup required for Maestro broadcast ─────────────────────────────────
  On each WCB that has a Maestro physically wired to it:
    1. Configure the serial port:   ?MAESTRO,M1:W<n>S<port>:<baud>
    2. Enable Kyber remote mode:    ?KYBER,REMOTE,S<port>
*/

#include <WCB_Client.h>
#include <WCBStream.h>
#include <PololuMaestro.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 4;
const uint8_t DEVICE_ID    = 20;  // 20 = special out-of-band slot

// ─────────────────────────────────────────────────────────────────────────────
// Maestro device numbers — must match the device number configured in
// Maestro Control Center (Serial Settings tab). Each Maestro responds only
// to packets addressed to its own device number.
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAESTRO_ID_1 = 1;
const uint8_t MAESTRO_ID_2 = 2;

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void onCommandReceived(uint8_t senderID, const char* command) {
    Serial.printf("[RX] From WCB%d: %s\n", senderID, command);
}

void onStatusChanged(uint8_t wcbID, bool online) {
    Serial.printf("[STATUS] WCB%d is now %s\n", wcbID, online ? "ONLINE" : "OFFLINE");
}

// ─────────────────────────────────────────────────────────────────────────────
// WCB_Client must be declared before any WCBStream objects.
// WCBStream self-registers with WCB_Client on construction so wcb.update()
// flushes all streams automatically — no extra calls needed in loop().
// ─────────────────────────────────────────────────────────────────────────────
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
              onCommandReceived, onStatusChanged);

// Broadcast each stream — one ESP-NOW packet reaches every WCB simultaneously.
// Each MiniMaestro embeds its device number in every packet so only the
// Maestro with the matching address acts on the command.
WCBStream maestroStream1(broadcast);
WCBStream maestroStream2(broadcast);

MiniMaestro maestro1(maestroStream1, Maestro::noResetPin, MAESTRO_ID_1);
MiniMaestro maestro2(maestroStream2, Maestro::noResetPin, MAESTRO_ID_2);

unsigned long lastMaestroMs = 0;
unsigned long lastCommandMs = 0;
bool          servoToggle   = false;

void setup() {
    Serial.begin(115200);
    delay(2000);
    wcb.begin();
}

void loop() {
    // update() drives heartbeats, offline detection, and all WCBStream flushes
    wcb.update();

    // ── Maestro servo movement (every 2 seconds) ──────────────────────────
    if (millis() - lastMaestroMs >= 2000) {
        lastMaestroMs = millis();
        servoToggle = !servoToggle;
        uint16_t pos = servoToggle ? 4000 : 8000;

        maestro1.setTarget(0, pos);
        maestro2.setTarget(0, pos);
        Serial.printf("[TX] Broadcast Maestros ch0 → %d (%s)\n", pos,
                      servoToggle ? "left" : "right");
    }

    // ── Text command sending (every 5 seconds) ────────────────────────────
    if (millis() - lastCommandMs >= 5000) {
        lastCommandMs = millis();

        wcb.broadcast(":PP100");
        Serial.println("[TX] Broadcast :PP100");

        if (wcb.isOnline(1)) {
            wcb.send(1, ":LEDON");
            Serial.println("[TX] Unicast to WCB1: :LEDON");
        } else {
            Serial.println("[TX] WCB1 offline — skipping");
        }
    }
}
