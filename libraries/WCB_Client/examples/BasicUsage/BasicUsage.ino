/*
  BasicUsage.ino — WCB_Client Library Example

  Demonstrates how to join a WCB ESP-NOW network as a custom ESP32 device,
  send commands to individual WCBs or broadcast to all of them, and receive
  commands that any WCB sends to this device.

  Quick start:
    1. Fill in the four network credentials below to match your WCB system.
    2. Choose a DEVICE_ID that isn't already used by a real WCB on the network.
    3. Flash to any ESP32 board — no wiring required, everything is wireless.
*/

#include <WCB_Client.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials
//
// These four values must match the settings on your WCB system exactly.
// You can find them in the WCB Config Tool or by querying a WCB over serial.
//
//   MAC_OCT2 / MAC_OCT3 : the shared MAC octets that identify your network
//                          (set via ?WCBM on each WCB)
//   PASSWORD             : the ESP-NOW network password (set via ?WCBP)
//   WCB_QUANTITY         : total number of WCBs in the system (set via ?WCBQ)
//   DEVICE_ID            : a unique ID for this device on the network
//                          Must be 1–WCB_QUANTITY, or 20 for the special slot.
//                          Do not reuse a number already assigned to a real WCB.
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 12;
const uint8_t DEVICE_ID    = 4;

// ─────────────────────────────────────────────────────────────────────────────
// Command callback  (optional)
//
// This function is called automatically by the library whenever another device
// on the WCB network sends a command to this device or broadcasts to everyone.
//
// senderID : the WCB number that sent the command (1–20)
// command  : the command string, e.g. ":PP100" or "GO"
//
// You only need this if your device needs to RECEIVE commands from WCBs.
// If this device only sends and never needs to receive, remove this function
// and remove onCommandReceived from the WCB_Client constructor below.
// ─────────────────────────────────────────────────────────────────────────────
void onCommandReceived(uint8_t senderID, const char* command) {
    Serial.printf("[RX] Command from WCB%d: %s\n", senderID, command);
    // Act on the command here — trigger an output, update state, relay to another
    // controller, etc.
}

// ─────────────────────────────────────────────────────────────────────────────
// Status callback  (optional)
//
// This function is called automatically by the library whenever a WCB on the
// network comes online (first heartbeat received) or goes offline (heartbeats
// stopped). The library tracks this internally regardless — this callback is
// just a notification so your code can react to it.
//
// Useful if you want to hold commands until a target WCB is confirmed online,
// or alert the operator when a board drops off the network.
//
// Remove this function and remove onStatusChanged from the constructor below
// if your sketch doesn't need to react to board online/offline events.
// ─────────────────────────────────────────────────────────────────────────────
void onStatusChanged(uint8_t wcbID, bool online) {
    Serial.printf("[STATUS] WCB%d is now %s\n", wcbID, online ? "ONLINE" : "OFFLINE");
}

// ─────────────────────────────────────────────────────────────────────────────
// Client instance
//
// Pass your credentials and callbacks here. The callbacks are optional —
// omit either one (or both) if you don't need them.
// ─────────────────────────────────────────────────────────────────────────────
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
               onCommandReceived, onStatusChanged);

unsigned long lastSendMs = 0;

void setup() {
    Serial.begin(115200);

    // Initialise ESP-NOW and join the WCB network.
    // The library prints status and error messages to Serial automatically.
    wcb.begin();
}

void loop() {
    // Keep the library running — handles outgoing heartbeats and detects when
    // WCBs go offline. Call this every loop iteration without blocking delays.
    wcb.update();

    // Send commands on a 5-second non-blocking timer
    if (millis() - lastSendMs >= 5000) {
        lastSendMs = millis();

        // broadcast() sends to ALL WCBs on the network simultaneously.
        // Use this when every board should act on the same command.
        wcb.broadcast(":PP100");
        Serial.println("[TX] Broadcast :PP100");

        // send() sends to ONE specific WCB by its ID.
        // Use this when only a particular board should act on the command.
        wcb.send(1, "Testing234");
        Serial.println("[TX] Unicast to WCB1: Testing234");
    }
}
