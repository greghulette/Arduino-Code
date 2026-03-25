/*
  CombinedUsage.ino — WCBClient + WCBStream Combined Example

  Demonstrates both WCB text command sending/receiving AND Pololu Maestro
  servo forwarding running simultaneously on the same ESP32.

  ── What this sketch does ────────────────────────────────────────────────────
  Every 2 seconds : moves a servo on Maestro A (port 1) and Maestro B (port 2)
                    back and forth using WCBStream — no serial wire required.

  Every 5 seconds : broadcasts a text command to all WCBs on the network,
                    and sends a unicast command to WCB1 specifically.

  Any time        : prints commands received from the WCB network.

  ── Network credentials ──────────────────────────────────────────────────────
  Fill in the values below to match your WCB system. Find them in the WCB
  Config Tool or by querying a WCB over serial (?WCBM, ?WCBP, ?WCBQ).

  ── WCB setup required ───────────────────────────────────────────────────────
  The WCB connected to your Maestro(s) must have the correct serial port
  configured and wired to the Maestro RX line. Set MAESTRO_WCB, MAESTRO_PORT_1
  and MAESTRO_PORT_2 below to match your physical wiring.
*/

#include <WCBClient.h>
#include <WCBStream.h>
#include <PololuMaestro.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;              // ?WCBM second octet
const uint8_t MAC_OCT3     = 0x00;              // ?WCBM third octet
const char*   PASSWORD     = "change_me_or_risk_takeover";  // ?WCBP
const uint8_t WCB_QUANTITY = 4;                 // ?WCBQ — total WCBs in system
const uint8_t DEVICE_ID    = 20;                 // Unique ID for this device

// ─────────────────────────────────────────────────────────────────────────────
// Maestro routing — set these to match your physical wiring
//
// MAESTRO_DEVICE_x : device number configured in Maestro Control Center
//                    (Serial Settings tab). Enables the Pololu protocol so
//                    each packet includes the 0xAA start byte and device
//                    number — required when your show control software
//                    addresses Maestros by device number, or when multiple
//                    Maestros share a serial bus.
//                    Set to Maestro::deviceNumberDefault (255) to use compact
//                    protocol instead (no device addressing).
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAESTRO_WCB      = 2;  // WCB number the Maestros are connected to
const uint8_t MAESTRO_PORT_1   = 1;  // Serial port on that WCB wired to Maestro A
const uint8_t MAESTRO_DEVICE_1 = 1;  // Maestro A device number (Maestro Control Center)
const uint8_t MAESTRO_PORT_2   = 2;  // Serial port on that WCB wired to Maestro B
const uint8_t MAESTRO_DEVICE_2 = 2;  // Maestro B device number (Maestro Control Center)

// ─────────────────────────────────────────────────────────────────────────────
// WCBClient — manages the ESP-NOW network connection
// ─────────────────────────────────────────────────────────────────────────────
WCBClient wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// ─────────────────────────────────────────────────────────────────────────────
// WCBStream — intercepts Pololu library writes and forwards them wirelessly.
// Each stream targets a specific WCB + serial port. Self-registers with wcb
// so wcb.update() flushes them automatically — no extra calls needed in loop().
// ─────────────────────────────────────────────────────────────────────────────
WCBStream maestroStream1(wcb, MAESTRO_WCB, MAESTRO_PORT_1);
WCBStream maestroStream2(wcb, MAESTRO_WCB, MAESTRO_PORT_2);

// Pass WCBStream to the Pololu library instead of a serial port.
// The device number enables the Pololu protocol (0xAA start byte + device ID).
// Maestro::noResetPin = 255 means no hardware reset pin is used on this ESP32.
MiniMaestro maestro1(maestroStream1, Maestro::noResetPin, MAESTRO_DEVICE_1);
MiniMaestro maestro2(maestroStream2, Maestro::noResetPin, MAESTRO_DEVICE_2);

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────

// Called when a text command arrives from the WCB network addressed to this
// device or broadcast to all. senderID is the WCB number that sent it.
void onCommandReceived(uint8_t senderID, const char* command) {
    Serial.printf("[RX] Command from WCB%d: %s\n", senderID, command);
    // Add your command handling here — e.g. parse the string and call
    // maestro methods, trigger outputs, update state, etc.
}

// Called when a WCB comes online or goes offline. Useful for knowing
// whether your target board is reachable before sending critical commands.
void onStatusChanged(uint8_t wcbID, bool online) {
    Serial.printf("[STATUS] WCB%d is now %s\n", wcbID, online ? "ONLINE" : "OFFLINE");
}

// ─────────────────────────────────────────────────────────────────────────────
// Timers — all non-blocking using millis()
// ─────────────────────────────────────────────────────────────────────────────
unsigned long lastMaestroMs = 0;   // Maestro servo movement interval
unsigned long lastCommandMs = 0;   // Text command send interval
bool          servoToggle   = false;

void setup() {
    Serial.begin(115200);
    delay(2000);  // Allow serial monitor time to connect before boot messages

    wcb.begin();

    // Callbacks can also be passed directly to the WCBClient constructor —
    // they are registered here after begin() as an alternative approach.
    wcb.onCommand(onCommandReceived);
    wcb.onStatusChange(onStatusChanged);
}

void loop() {
    // update() must be called every loop iteration — it drives:
    //   - Heartbeat broadcasts (keeps this device visible to the network)
    //   - Offline detection (fires onStatusChanged when a WCB drops out)
    //   - WCBStream flush (forwards buffered Maestro bytes via sendRaw)
    // Do not block loop() with delays or heartbeats will be missed.
    wcb.update();

    // ── Maestro servo movement (every 2 seconds) ──────────────────────────
    // setTarget() writes binary bytes to WCBStream. wcb.update() above
    // detects the 2ms inter-frame gap and forwards them to MAESTRO_WCB.
    // The target WCB writes the bytes to its serial port → Maestro executes.
    if (millis() - lastMaestroMs >= 2000) {
        lastMaestroMs = millis();
        servoToggle = !servoToggle;

        // Servo positions in quarter-microseconds:
        //   4000 = 1000µs (full left)   6000 = 1500µs (centre)
        //   8000 = 2000µs (full right)
        uint16_t pos = servoToggle ? 4000 : 8000;

        maestro1.setTarget(0, pos);
        maestro1.setTarget(1, pos + 20);
        maestro1.setTarget(2, pos + 40);
        maestro1.setTarget(3, pos + 60);
        maestro1.setTarget(4, pos + 80);
        maestro1.setTarget(5, pos + 90);
        Serial.printf("[TX] Maestro A ch0 → %d (%s)\n", pos,
                      servoToggle ? "left" : "right");

        maestro2.setTarget(0, pos);
        maestro2.restartScript(1);
        Serial.printf("[TX] Maestro B ch0 → %d (%s)\n", pos,
                      servoToggle ? "left" : "right");
    }

    // ── Text command sending (every 5 seconds) ────────────────────────────
    // Demonstrates both broadcast and unicast text commands running
    // alongside the Maestro stream without interfering.
    if (millis() - lastCommandMs >= 5000) {
        lastCommandMs = millis();

        // Broadcast to all WCBs simultaneously
        wcb.broadcast(":PP100");
        Serial.println("[TX] Broadcast :PP100");

        // Unicast to WCB1 only — checks isOnline() first so we don't
        // fire into the void if WCB1 hasn't been seen yet
        if (wcb.isOnline(1)) {
            wcb.send(1, ":LEDON");
            Serial.println("[TX] Unicast to WCB1: :LEDON");
        } else {
            Serial.println("[TX] WCB1 offline — skipping unicast");
        }
    }
}
