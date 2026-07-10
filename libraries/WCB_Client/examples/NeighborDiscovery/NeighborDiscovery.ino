/*
  NeighborDiscovery.ino — WCB_Client Library Example

  Demonstrates the WDP consumer: this device advertises its own identity with
  setIdentity() AND listens for the adverts every other board broadcasts, so it
  learns the whole mesh automatically — each WCB and each identity-enabled
  client — with no manual configuration.

  Two ways to read the mesh are shown:
    • onNeighbor()  — an event fires the moment a board is learned or ages out.
    • getNeighbor() / neighborCount() — poll the table whenever you like.

  WDP discovery answers "who is out there and what can they do". It is separate
  from ETM heartbeats, which answer "who is reachable right now" (isOnline()).
  A neighbor can be in the WDP table (recently advertised) yet not currently
  online, or vice-versa — this example prints both so the difference is clear.

  Flash to any ESP32 board — no wiring required, everything is wireless.
*/

#include <WCB_Client.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly (see BasicUsage).
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 12;
const uint8_t DEVICE_ID    = 4;

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// ─────────────────────────────────────────────────────────────────────────────
// Decode the WCB capability bitmap (capFlags) into a human-readable list.
// These bits are set on WCBs (boards). WCB_Client devices instead advertise
// their capabilities as free-form text in capTags — see printNeighbor() below.
// ─────────────────────────────────────────────────────────────────────────────
void printCapFlags(uint16_t caps) {
    struct { uint16_t bit; const char* name; } kCaps[] = {
        { WCB_CAP_HCR,            "HCR"           },
        { WCB_CAP_MP3,            "MP3"           },
        { WCB_CAP_WLED,           "WLED"          },
        { WCB_CAP_KYBER_LOCAL,    "KyberLocal"    },
        { WCB_CAP_MAESTRO_REMOTE, "MaestroRemote" },
        { WCB_CAP_PWM,            "PWM"           },
        { WCB_CAP_CONTROLLER,     "Controller"    },
        { WCB_CAP_MAESTRO_HOST,   "MaestroHost"   },
    };
    bool any = false;
    for (auto& c : kCaps) {
        if (caps & c.bit) {
            Serial.printf("%s%s", any ? "," : "", c.name);
            any = true;
        }
    }
    if (!any) Serial.print("none");
}

// ─────────────────────────────────────────────────────────────────────────────
// Pretty-print one learned neighbor. Called from loop() (never from the
// callback) so it is safe to do as much work / Serial output as we like.
// ─────────────────────────────────────────────────────────────────────────────
void printNeighbor(const WCBNeighbor& nb) {
    // Board vs. client: a WCB advertises a numeric hwVer and a capFlags bitmap;
    // a WCB_Client device advertises a hwRev string and free-text capTags.
    Serial.printf("  WCB%-2u %-16s %-8s fw=%s",
                  nb.wcbNumber, nb.name,
                  nb.isClient ? "(client)" : "(board)", nb.fw);

    if (nb.isClient) {
        if (nb.hwRev[0])   Serial.printf("  hwRev=%s", nb.hwRev);
        if (nb.capTags[0]) Serial.printf("  caps=[%s]", nb.capTags);
    } else {
        Serial.printf("  hwVer=%u  caps=[", nb.hwVer);
        printCapFlags(nb.capFlags);
        Serial.print("]");
    }

    // Controller (special-peer) link, if this board reported one.
    if (nb.ctrlId) Serial.printf("  ctrl=%u", nb.ctrlId);

    // Local Maestro IDs, if any.
    if (nb.maestroCount) {
        Serial.print("  maestro=");
        for (uint8_t m = 0; m < nb.maestroCount; m++)
            Serial.printf("%s%u", m ? "," : "", nb.maestroIds[m]);
    }

    // Advertised serial-port labels (skip the blank ones).
    for (uint8_t p = 0; p < 5; p++) {
        if (nb.portLabels[p][0])
            Serial.printf("  P%u=%s", p + 1, nb.portLabels[p]);
    }

    // Freshness — how long since we last heard an advert (TTL is
    // WCB_WDP_NEIGHBOR_TTL_MS). Also show current ETM heartbeat liveness,
    // which is a DIFFERENT question than WDP presence.
    Serial.printf("  seen=%lus  %s",
                  (millis() - nb.lastSeenMs) / 1000,
                  wcb.isOnline(nb.wcbNumber) ? "[online]" : "[no HB]");
    Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// Neighbor callback
//
// Fires when a board's advert is first learned, refreshed, or ages out. It runs
// on the ESP-NOW (WiFi) task, so keep it SHORT: just record that something
// changed and let loop() do the printing / heavy work via getNeighbor(). We do
// NOT print the full roster here — we only flag it, then act from loop().
// ─────────────────────────────────────────────────────────────────────────────
volatile bool rosterDirty = false;   // set on the WiFi task, read in loop()

void onNeighborEvent(const WCBNeighbor& nb) {
    if (!nb.valid) {
        // TTL age-out: no advert within WCB_WDP_NEIGHBOR_TTL_MS.
        Serial.printf("[WDP] WCB%u aged out\n", nb.wcbNumber);
    } else {
        // Learned or refreshed. A one-line note is cheap; save the detail
        // for loop() where getNeighbor() gives us the full record safely.
        Serial.printf("[WDP] learned %s WCB%u = \"%s\"%s\n",
                      nb.isClient ? "client" : "board",
                      nb.wcbNumber, nb.name,
                      (nb.capFlags & WCB_CAP_MAESTRO_HOST) ? "  [Maestro host]" : "");
    }
    rosterDirty = true;   // ask loop() to reprint the roster
}

unsigned long lastDumpMs = 0;

// Print the whole learned table by polling getNeighbor() across every slot.
void dumpRoster() {
    Serial.printf("\n=== Mesh roster: %u neighbor(s) ===\n", wcb.neighborCount());
    for (uint8_t id = 1; id <= WCB_MAX_BOARDS; id++) {
        const WCBNeighbor* nb = wcb.getNeighbor(id);
        if (!nb) continue;          // nothing learned for this slot
        printNeighbor(*nb);         // pointer is valid until the next update()
    }
}

void setup() {
    Serial.begin(115200);

    // Advertise ourselves so other boards (and their config tools) see us by
    // name. The optional 3rd/4th args set the hwRev and capTags that peers
    // learning US will show — the client side of the board/client split above.
    wcb.setIdentity("MeshMonitor", "v1.0.0", "revA", "mesh.monitor");

    // Register the consumer callback, then join the network.
    wcb.onNeighbor(onNeighborEvent);
    wcb.begin();
}

void loop() {
    // Drives heartbeats, our own advert cadence, and neighbor aging.
    wcb.update();

    // Reprint promptly when the callback flagged a change, and also on a 10 s
    // heartbeat so freshness/online columns stay current. Doing the printing
    // HERE (not in the callback) keeps the WiFi task fast.
    bool timeToDump = (millis() - lastDumpMs >= 10000);
    if (rosterDirty || timeToDump) {
        rosterDirty = false;
        lastDumpMs  = millis();
        dumpRoster();
    }
}
