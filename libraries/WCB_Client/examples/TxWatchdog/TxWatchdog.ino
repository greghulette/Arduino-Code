/*
  TxWatchdog.ino — WCB_Client Library Example (1.18.0)

  A device can go on HEARING the mesh while nothing it sends arrives: every WCB
  lists it offline while its own roster shows them all online. Broadcasts are
  never acknowledged, so nothing on the device notices. The transmit watchdog
  does: every 30 s it broadcasts a WDP SOLICIT (the ?WDP,POLL probe) and expects
  a fresh advert from a board it hears. After 3 unanswered probes it puts the
  mesh MAC back and restarts ESP-NOW (level 1); if that does not help it reports
  level 2, and this sketch reboots - the cure that worked in the field.

  Every level change is printed by the library with the radio's MAC, channel,
  send errors and heap. Type 's' in the Serial Monitor for the watchdog's state.

  Quick start:
    1. Fill in the network credentials below to match your WCB system.
    2. Choose a DEVICE_ID no real WCB uses.
    3. The WCBs need WDP on (the default): a board that never advertises is
       never asked, so the watchdog stays quiet instead of guessing.
*/

#include <WCB_Client.h>

const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 12;
const uint8_t DEVICE_ID    = 4;

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// Fires on the LOOP task (from wcb.update()), so a restart here is safe.
//   level 1 : the library restarted ESP-NOW; nothing to do yet
//   level 2 : still unanswered after that - reboot
//   level 0 : answered again
void onTxStall(uint8_t level, uint32_t stalledMs) {
    Serial.printf("[APP] transmit watchdog level %u after %lu s\n", level, (unsigned long)(stalledMs / 1000UL));
    if (level == 2) {
        // A real device should count these across restarts (RTC memory) and stop
        // after a few, so a wedge a reboot cannot cure does not become a boot loop -
        // NaviCore allows two in a row.
        Serial.println("[APP] restarting");
        Serial.flush();
        delay(100);
        ESP.restart();
    }
}

void setup() {
    Serial.begin(115200);
    if (!wcb.begin()) {
        Serial.println("[WCB] begin() FAILED (see error above) — halting.");
        while (true) delay(1000);
    }
    // The watchdog judges by the boards' WDP adverts, and a device that advertises
    // itself is also what a WCB's ?WDP,LIST shows - so give it an identity.
    wcb.setIdentity("TxWatchdogDemo", "1.0", "ESP32", "");
    wcb.onTxStall(onTxStall);
    wcb.setTxWatchdog(true);   // a probe every WCB_TXWD_PROBE_MS (30 s)
}

void loop() {
    wcb.update();
    if (Serial.available() && Serial.read() == 's') {
        const WCBTxWatchStatus st = wcb.getTxWatchStatus();
        Serial.printf("watchdog %s, level %u, misses %u, probes %lu, ESP-NOW restarts %lu, send errors %lu, "
                      "last answered %s\n",
                      st.enabled ? "on" : "off", st.level, st.misses, (unsigned long)st.probes,
                      (unsigned long)st.restarts, (unsigned long)st.sendErrors,
                      st.sinceProofMs == 0xFFFFFFFFUL ? "never" : (String(st.sinceProofMs / 1000UL) + " s ago").c_str());
    }
}
