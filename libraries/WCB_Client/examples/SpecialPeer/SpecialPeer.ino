/*
  SpecialPeer.ino — WCB_Client Library Example

  Demonstrates talking to the out-of-band "special peer" on a WCB network —
  e.g. NaviCore, which lives at ID 20 OUTSIDE the numbered 1..WCB_QUANTITY slots
  (so it doesn't consume a board slot).

  Out of the box the library can already RECEIVE from the special peer. This
  example shows the two things that need enableSpecialPeer() to work:
    1. SENDING commands to it       — sendToSpecialPeer() / send(20, ...)
    2. TRACKING its ETM heartbeat   — isSpecialPeerOnline() + the status callback

  Quick start:
    1. Fill in the four network credentials below to match your WCB system.
    2. Choose a DEVICE_ID that isn't already used by a real WCB on the network.
    3. Make sure your WCBs have the special peer enabled (?SPECIAL,ON,20).
    4. Flash to any ESP32 board — no wiring required, everything is wireless.
*/

#include <WCB_Client.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly.
//   MAC_OCT2 / MAC_OCT3 : shared MAC octets that identify your network
//   PASSWORD            : the ESP-NOW network password
//   WCB_QUANTITY        : total number of WCBs in the system (?WCBQ)
//   DEVICE_ID           : a unique ID for THIS device (1–WCB_QUANTITY)
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 12;
const uint8_t DEVICE_ID    = 4;

// The special peer's ID. NaviCore defaults to WCB_SPECIAL_ID (20). Change this
// only if your NaviCore is configured for a different ID — it must match.
const uint8_t NAVICORE_ID  = WCB_SPECIAL_ID;   // = 20

// ─────────────────────────────────────────────────────────────────────────────
// Command callback (optional) — fires when any device on the network sends a
// command to this device or broadcasts to everyone. senderID 20 = the special
// peer (NaviCore).
// ─────────────────────────────────────────────────────────────────────────────
void onCommandReceived(uint8_t senderID, const char* command) {
    if (senderID == NAVICORE_ID)
        Serial.printf("[RX] From NaviCore: %s\n", command);
    else
        Serial.printf("[RX] From WCB%d: %s\n", senderID, command);
}

// ─────────────────────────────────────────────────────────────────────────────
// Status callback (optional) — fires when a peer comes online / goes offline.
// Once enableSpecialPeer() has been called, this also fires for the special peer
// (NaviCore) when its ETM heartbeat starts/stops.
// ─────────────────────────────────────────────────────────────────────────────
void onStatusChanged(uint8_t wcbID, bool online) {
    if (wcbID == NAVICORE_ID)
        Serial.printf("[STATUS] NaviCore is now %s\n", online ? "ONLINE" : "OFFLINE");
    else
        Serial.printf("[STATUS] WCB%d is now %s\n", wcbID, online ? "ONLINE" : "OFFLINE");
}

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
               onCommandReceived, onStatusChanged);

unsigned long lastSendMs = 0;

void setup() {
    Serial.begin(115200);

    // Join the WCB network.
    wcb.begin();

    // Enable two-way comms with the special peer (NaviCore). This registers its
    // MAC so we can send to it, and starts tracking its heartbeat for status.
    // Safe to call before or after begin(); pass an ID to override the default 20.
    wcb.enableSpecialPeer(NAVICORE_ID);
}

void loop() {
    // Keep the library running (heartbeats out, offline detection in).
    wcb.update();

    // Every 5 seconds, send a command to NaviCore — but only if it's online.
    if (millis() - lastSendMs >= 5000) {
        lastSendMs = millis();

        if (wcb.isSpecialPeerOnline()) {
            wcb.sendToSpecialPeer("GO");           // → NaviCore (ensured delivery)
            Serial.println("[TX] Sent 'GO' to NaviCore");
        } else {
            Serial.println("[..] NaviCore offline — holding command");
        }
    }
}
