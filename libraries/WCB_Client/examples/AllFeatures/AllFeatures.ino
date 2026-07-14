/*
  AllFeatures.ino — every WCB_Client feature in ONE sketch (kitchen sink)

  A single, interactive demo that exercises the entire public API so you can see
  each feature in one place. Flash it to any ESP32; open the Serial Monitor at
  115200 and press the listed keys to fire each action. Background activity
  (heartbeats, WDP adverts, a gentle Maestro sweep, and a mesh-roster dump) runs
  automatically.

  ── Features demonstrated ─────────────────────────────────────────────────────
    • Construction + begin()/update() lifecycle (with a checked begin()).
    • All four callbacks: onCommand, onStatusChange, onNeighbor, onRawPacket.
    • Text commands:      send() (unicast) and broadcast().
    • Raw servo bytes:    sendRaw() (unicast WCB:port) and sendKyber() (broadcast).
    • Custom protocol:    sendRawPacket() + onRawPacket() (the OTA/relay seam).
    • Maestro streaming:  WCBStream (broadcast AND unicast) driving MiniMaestro,
                          plus flushNow()/bytesFree() (new in 1.9.3).
    • Special peer:       enableSpecialPeer(), isSpecialPeerOnline(),
                          sendToSpecialPeer()  — e.g. NaviCore at ID 20.
    • Liveness:           isOnline().
    • WDP discovery:      setIdentity() (advertise us) + onNeighbor(),
                          getNeighbor(), neighborCount() (learn the mesh).
    • Auto-join:          setAutoJoin(), autoJoinEnabled(), forgetPeer(),
                          clearLearnedPeers().
    • Serial passthrough: monitorRaw() / monitorSerial() (optional, needs wiring).
    • Config:             setChecksum().
    • Singleton:          WCB_Client::instance().

  ── Setup ─────────────────────────────────────────────────────────────────────
    1. Fill in the four network credentials to match your WCB system.
    2. Give this device a DEVICE_ID not used by a real WCB on the network.
    3. For the Maestro demo, configure Kyber remote on the WCBs that host a
       Maestro (?KYBER,REMOTE,S<port>) and set each Maestro's device number.
    4. (Optional) set ENABLE_SERIAL_MONITORS to 1 and wire a UART to demo the
       serial-tap forwarders.
*/

#include <WCB_Client.h>
#include <WCBStream.h>
#include <PololuMaestro.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly (see BasicUsage).
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 4;
const uint8_t DEVICE_ID    = 19;   // this device's unique ID on the mesh (1–20)

// The out-of-band special peer (a controller that lives OUTSIDE 1..WCB_QUANTITY,
// e.g. NaviCore). Defaults to WCB_SPECIAL_ID (20); change only to match yours.
const uint8_t SPECIAL_ID   = WCB_SPECIAL_ID;   // = 20

// Maestro device numbers (set in Maestro Control Center → Serial Settings). Each
// Maestro acts only on Pololu-protocol packets addressed to its own number.
const uint8_t MAESTRO_ID_1 = 1;    // driven over the BROADCAST stream
const uint8_t MAESTRO_ID_2 = 2;    // driven over the UNICAST stream (WCB1:port1)

// forgetPeer() demo target — a no-op unless this id is actually a learned peer.
const uint8_t FORGET_DEMO_ID = 12;

// Optional serial-tap forwarders. OFF by default so the sketch runs with no extra
// hardware. Set to 1 AND wire a device's TX pin to MONITOR_RX_PIN to enable.
#define ENABLE_SERIAL_MONITORS 0
#if ENABLE_SERIAL_MONITORS
  const int MONITOR_RX_PIN = 16;   // wire the source device's TX here (we read)
  const int MONITOR_TX_PIN = 17;   // unused by the monitors (read-only tap)
#endif

// A custom struct for the sendRawPacket()/onRawPacket() demo. Its size (8 B) is
// != 252, so the receiver delivers it through onRawPacket() rather than onCommand().
struct __attribute__((packed)) DemoPacket { char tag[4]; uint32_t counter; };
uint32_t demoCounter = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks — all four the library offers. Each runs on the ESP-NOW (WiFi) task,
// so keep them SHORT: a short print is fine (matching the library's other
// examples), but defer any heavy/blocking work to loop() via a flag or a queue.
// ─────────────────────────────────────────────────────────────────────────────
void onCommandRx(uint8_t senderID, const char* command) {
    Serial.printf("[RX cmd]  from %s%u: %s\n",
                  senderID == SPECIAL_ID ? "special-peer " : "WCB",
                  senderID, command);
}

void onStatusRx(uint8_t wcbID, bool online) {
    Serial.printf("[STATUS]  %s%u is now %s\n",
                  wcbID == SPECIAL_ID ? "special-peer " : "WCB",
                  wcbID, online ? "ONLINE" : "OFFLINE");
}

volatile bool rosterDirty = false;   // set on the WiFi task, consumed in loop()
void onNeighborRx(const WCBNeighbor& nb) {
    if (!nb.valid) Serial.printf("[WDP]     WCB%u aged out\n", nb.wcbNumber);
    else           Serial.printf("[WDP]     learned %s WCB%u = \"%s\"\n",
                                 nb.isClient ? "client" : "board", nb.wcbNumber, nb.name);
    rosterDirty = true;   // ask loop() to reprint the roster
}

void onRawPacketRx(const uint8_t* mac, const uint8_t* data, int len) {
    // Any received packet that is NOT the standard 252-byte WCB packet lands here
    // (e.g. our DemoPacket below, or a real app's OTA control/data structs).
    Serial.printf("[RAW pkt] %d bytes from %02X:%02X:%02X:%02X:%02X:%02X",
                  len, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if (len == (int)sizeof(DemoPacket)) Serial.print("  (looks like a DemoPacket)");
    Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// The client. WCB_Client MUST be declared before any WCBStream — each WCBStream
// self-registers with the singleton so wcb.update() flushes it automatically.
// (This 5-arg form registers no callbacks; we add all four via the on*() methods
//  in setup(). The constructor can also take a command + status callback.)
// ─────────────────────────────────────────────────────────────────────────────
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// Two Maestro streams: one BROADCAST (reaches every Kyber-enabled WCB at once —
// the recommended path for servo passthrough) and one UNICAST to WCB1 port 1.
WCBStream maestroBcast(broadcast);   // broadcast == 0 (the Kyber path)
WCBStream maestroUni(1, 1);          // unicast: WCB 1, serial port 1

MiniMaestro maestroB(maestroBcast, Maestro::noResetPin, MAESTRO_ID_1);
MiniMaestro maestroU(maestroUni,   Maestro::noResetPin, MAESTRO_ID_2);

unsigned long lastSweepMs = 0, lastRosterMs = 0;
bool servoToggle = false;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== WCB_Client — ALL FEATURES demo ===");

    // 1) Advertise our identity on the mesh (WDP) so other boards + the config
    //    tool discover us by name/firmware. hwRev + capTags are the client-side
    //    fields peers see when they learn US.
    wcb.setIdentity("AllFeaturesDemo", "v1.0.0", "revA", "demo kitchen-sink");

    // 2) Network-consistent config. setChecksum() MUST match ?ETM,CHKSM on the
    //    WCBs. setAutoJoin() is ON by default; set explicitly for clarity.
    wcb.setChecksum(true);
    wcb.setAutoJoin(true);

    // 3) Register all four callbacks BEFORE begin() so nothing is missed.
    wcb.onCommand(onCommandRx);
    wcb.onStatusChange(onStatusRx);
    wcb.onNeighbor(onNeighborRx);
    wcb.onRawPacket(onRawPacketRx);

    // 4) Join the mesh. ALWAYS check begin(): false = ESP-NOW did not start
    //    (e.g. WiFi didn't come up); running update()/send() then would crash.
    if (!wcb.begin()) {
        Serial.println("[WCB] begin() FAILED (see error above) — halting.");
        while (true) delay(1000);
    }

    // 5) Two-way comms with the out-of-band special peer (e.g. NaviCore at 20):
    //    registers its MAC so we can send to it AND tracks its heartbeat.
    wcb.enableSpecialPeer(SPECIAL_ID);

    // 6) OPTIONAL: forward an attached UART onto the mesh (needs wiring). Pick ONE
    //    of these per UART — a raw binary tap OR a newline-text-command tap.
    #if ENABLE_SERIAL_MONITORS
        Serial1.begin(115200, SERIAL_8N1, MONITOR_RX_PIN, MONITOR_TX_PIN);
        // Raw tap: a device's Maestro/Pololu TX bytes → WCB1 serial port 1.
        wcb.monitorRaw(Serial1, /*target_wcb=*/1, /*target_port=*/1, /*gap_ms=*/2);
        // Text tap (alternative): each '\n'-terminated line → broadcast to all WCBs.
        // wcb.monitorSerial(Serial1, WCB_TARGET_BROADCAST);
    #endif

    Serial.printf("Joined as device %u. Auto-join %s. Singleton instance()=%p.\n",
                  DEVICE_ID, wcb.autoJoinEnabled() ? "ON" : "OFF",
                  (void*)WCB_Client::instance());
    printMenu();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // The one required call: heartbeats out, offline detection in, WDP adverts,
    // WCBStream flushes, neighbor aging, and any serial monitors — all driven here.
    wcb.update();

    // Automatic gentle servo sweep every 2 s (broadcast + unicast streams).
    if (millis() - lastSweepMs >= 2000) { lastSweepMs = millis(); sweepServos(); }

    // Reprint the learned roster on a change, and on a 15 s heartbeat.
    if (rosterDirty || millis() - lastRosterMs >= 15000) {
        rosterDirty = false; lastRosterMs = millis(); dumpRoster();
    }

    // Interactive menu.
    if (Serial.available()) handleKey((char)Serial.read());
}

// ─────────────────────────────────────────────────────────────────────────────
// Maestro sweep — demonstrates WCBStream (broadcast + unicast) plus the 1.9.3
// flushNow()/bytesFree() helpers.
// ─────────────────────────────────────────────────────────────────────────────
void sweepServos() {
    servoToggle = !servoToggle;
    uint16_t pos = servoToggle ? 4000 : 8000;   // ¼µs (1000µs ↔ 2000µs)

    maestroB.setTarget(0, pos);   // → every Kyber WCB's Maestro (broadcast)
    maestroU.setTarget(0, pos);   // → WCB1 port 1's Maestro (unicast)

    // WCBStream normally flushes after gap_ms of silence. bytesFree() reports the
    // buffer headroom; flushNow() forces an immediate frame-boundary flush (also
    // what maestroWrite-style producers use to split a big multi-channel burst
    // into multiple packets instead of truncating it). Both are new in 1.9.3.
    Serial.printf("[SERVO]   ch0 -> %u  (bcast buf free before flush = %u B)\n",
                  pos, (unsigned)maestroBcast.bytesFree());
    maestroBcast.flushNow();
    maestroUni.flushNow();
}

// ─────────────────────────────────────────────────────────────────────────────
// Roster dump — neighborCount() + getNeighbor() (WDP: "who is out there") next to
// isOnline() (ETM heartbeat: "who is reachable right now"). They are different
// questions — a node can be in one and not the other.
// ─────────────────────────────────────────────────────────────────────────────
void dumpRoster() {
    Serial.printf("\n--- Mesh: %u WDP neighbor(s) | online 1..%u = ",
                  wcb.neighborCount(), WCB_QUANTITY);
    for (uint8_t id = 1; id <= WCB_QUANTITY; id++) Serial.print(wcb.isOnline(id) ? '1' : '0');
    Serial.printf(" | special-peer(%u) %s ---\n",
                  SPECIAL_ID, wcb.isSpecialPeerOnline() ? "online" : "offline");

    for (uint8_t id = 1; id <= WCB_MAX_BOARDS; id++) {
        const WCBNeighbor* nb = wcb.getNeighbor(id);   // do NOT retain past update()
        if (!nb) continue;
        Serial.printf("   WCB%-2u %-16s %-8s fw=%-10s %s  seen=%lus\n",
                      nb->wcbNumber, nb->name, nb->isClient ? "(client)" : "(board)",
                      nb->fw, wcb.isOnline(nb->wcbNumber) ? "[online]" : "[no HB]",
                      (millis() - nb->lastSeenMs) / 1000);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// One-off senders wired to the serial menu.
// ─────────────────────────────────────────────────────────────────────────────
void sendDemoRaw() {
    // Raw Pololu-protocol "Set Target" frame → WCB1 port 1 (unicast). This is the
    // hand-rolled equivalent of what WCBStream + MiniMaestro build for you.
    //   0xAA <device> <cmd&0x7F> <channel> <target lo7> <target hi7>
    //   Set Target = 0x84; target 6000 ¼µs (1500µs) = 0x1770 → lo7 0x70, hi7 0x2E.
    const uint8_t frame[] = { 0xAA, MAESTRO_ID_2, 0x04, 0x00, 0x70, 0x2E };
    bool ok = wcb.sendRaw(1, 1, frame, sizeof(frame));
    Serial.printf("[TX] sendRaw WCB1:port1 %u bytes -> %s\n",
                  (unsigned)sizeof(frame), ok ? "OK" : "FAIL");
}

void sendDemoKyber() {
    // Broadcast the same raw bytes to EVERY WCB with Kyber_Remote — each forwards
    // to its own local Maestro. No per-WCB targeting.
    const uint8_t frame[] = { 0xAA, MAESTRO_ID_1, 0x04, 0x00, 0x70, 0x2E };
    bool ok = wcb.sendKyber(frame, sizeof(frame));
    Serial.printf("[TX] sendKyber broadcast %u bytes -> %s\n",
                  (unsigned)sizeof(frame), ok ? "OK" : "FAIL");
}

void sendDemoRawPacket() {
    // Unicast an ARBITRARY struct (not a wcb_packet_etm_t) to WCB1's MAC — the seam
    // custom protocols like firmware OTA ride on. Registers the peer on demand; the
    // receiver sees it via onRawPacket() because its size != 252 B.
    DemoPacket pkt; memcpy(pkt.tag, "DEMO", 4); pkt.counter = ++demoCounter;
    bool ok = wcb.sendRawPacket(1, (const uint8_t*)&pkt, sizeof(pkt));
    Serial.printf("[TX] sendRawPacket WCB1 tag=DEMO counter=%lu (%u B) -> %s\n",
                  (unsigned long)pkt.counter, (unsigned)sizeof(pkt), ok ? "OK" : "FAIL");
}

// ─────────────────────────────────────────────────────────────────────────────
// Serial menu.
// ─────────────────────────────────────────────────────────────────────────────
void printMenu() {
    Serial.println(
        "\nKeys:\n"
        "  b = broadcast text (:PP100)      u = unicast WCB1 (:LEDON)\n"
        "  s = send special-peer (GO)       m = move servos now\n"
        "  r = sendRaw WCB1:port1           k = sendKyber (broadcast)\n"
        "  p = sendRawPacket WCB1           n = dump roster    o = online status\n"
        "  a = toggle auto-join             f = forgetPeer(demo id)   c = clearLearnedPeers\n"
        "  x = toggle checksum              h = this help");
}

void handleKey(char c) {
    static bool checksumOn = true;
    switch (c) {
        case 'b': wcb.broadcast(":PP100"); Serial.println("[TX] broadcast :PP100"); break;

        case 'u':
            if (wcb.isOnline(1)) { wcb.send(1, ":LEDON"); Serial.println("[TX] unicast WCB1 :LEDON"); }
            else                   Serial.println("[..] WCB1 offline — skipped");
            break;

        case 's':
            if (wcb.isSpecialPeerOnline()) { wcb.sendToSpecialPeer("GO"); Serial.println("[TX] special-peer GO"); }
            else                            Serial.println("[..] special-peer offline — skipped");
            break;

        case 'm': sweepServos();       break;
        case 'r': sendDemoRaw();       break;
        case 'k': sendDemoKyber();     break;
        case 'p': sendDemoRawPacket(); break;
        case 'n': dumpRoster();        break;

        case 'o':
            Serial.print("[online] ");
            for (uint8_t id = 1; id <= WCB_QUANTITY; id++)
                Serial.printf("WCB%u=%s ", id, wcb.isOnline(id) ? "up" : "down");
            Serial.printf("special-peer=%s\n", wcb.isSpecialPeerOnline() ? "up" : "down");
            break;

        case 'a': {
            bool on = !wcb.autoJoinEnabled();
            wcb.setAutoJoin(on);
            Serial.printf("[cfg] auto-join now %s\n", on ? "ON" : "OFF");
            break;
        }

        case 'f':
            wcb.forgetPeer(FORGET_DEMO_ID);   // no-op unless it's a learned peer
            Serial.printf("[cfg] forgetPeer(%u) (no-op if it wasn't auto-joined)\n", FORGET_DEMO_ID);
            break;

        case 'c':
            wcb.clearLearnedPeers();
            Serial.println("[cfg] clearLearnedPeers() — dropped all auto-joined peers");
            break;

        case 'x':
            checksumOn = !checksumOn;
            wcb.setChecksum(checksumOn);   // MUST match ?ETM,CHKSM on the WCBs
            Serial.printf("[cfg] checksum now %s (match ?ETM,CHKSM on the WCBs)\n",
                          checksumOn ? "ON" : "OFF");
            break;

        case 'h': case '?': printMenu(); break;
        case '\r': case '\n': break;   // ignore line endings
        default: Serial.printf("[?] unknown key '%c' — press h for help\n", c); break;
    }
}
