/*
  DeliveryStats.ino — WCB_Client Library Example

  Per-peer delivery statistics: which mesh links are healthy, which are retrying,
  which are failing. This is the diagnostic you reach for when a droid "mostly
  works" and you need to know whether the radio is the problem.

  What you get:
    getPeerStats(id)      sent / ackd / retries / failed / noSlot for one peer
    getAggregateStats()   the same, totalled across all peers
    getBroadcastSent()    broadcast COMMAND frames put on the air
    resetStats()          zero everything and start counting from here

  Quick start:
    1. Fill in the four network credentials below to match your WCB system.
    2. Set TARGET_WCB to a real board you can physically power on and off.
    3. Flash to any ESP32 — no wiring required.
    4. Open Serial at 115200 and press 'h'.

  ── The bench procedure this sketch exists for ───────────────────────────────
  A wrong counter compiles perfectly, so the only real verification is two
  boards and a power switch. Press 'g' for the guided walk-through; the short
  version is:

    1. TARGET on, send some traffic     -> sent and ackd rise together
    2. TARGET off, send within ~50 s    -> retries climb, then failed
    3. TARGET off, wait past ~50 s      -> failed climbs with NO retries

  Step 3 is the one that matters. The offline window is 50 s (10 s heartbeat x
  5 missed) against a retry budget of only ~2 s, so once a board has been off
  for a while the library already knows it is gone and stops waiting without
  ever retrying. That path is the common one in real life, and it is the
  counter nobody would notice was broken.
*/

#include <WCB_Client.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly.
//   MAC_OCT2 / MAC_OCT3 : shared MAC octets identifying your network (?WCBM)
//   PASSWORD            : ESP-NOW network password (?WCBP)
//   WCB_QUANTITY        : total number of WCBs in the system (?WCBQ)
//   DEVICE_ID           : unique ID for THIS device (1–WCB_QUANTITY, or 20)
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 12;
const uint8_t DEVICE_ID    = 4;

// The board this sketch exercises. Pick one you can physically power off —
// steps 2 and 3 of the bench procedure depend on it.
const uint8_t TARGET_WCB   = 1;

// A harmless command to send. Anything the target will accept is fine; the
// counters do not care what the payload is, only whether it was acknowledged.
const char*   TEST_COMMAND = ":PP100";

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// ─────────────────────────────────────────────────────────────────────────────
// Sketch state
// ─────────────────────────────────────────────────────────────────────────────
bool          autoSend    = false;          // 'a' toggles a steady 1 Hz trickle
unsigned long lastSendMs  = 0;
unsigned long lastDumpMs  = 0;
const unsigned long SEND_INTERVAL_MS = 1000;
const unsigned long DUMP_INTERVAL_MS = 5000;

// Previous snapshot for the target, so the table can show DELTAS. Watching the
// change between dumps is far more readable on a bench than watching running
// totals climb.
WCBPeerStats prevTarget = {};
bool         havePrev   = false;

void setup() {
    Serial.begin(115200);
    delay(300);

    // If your fleet runs on a non-default ESP-NOW channel, match it BEFORE
    // begin() — one radio, one channel, and a mismatch is silent.
    // wcb.setMeshChannel(1);
    if (!wcb.begin()) {
        Serial.println("[WCB] begin() FAILED (see error above) — halting.");
        while (true) delay(1000);
    }

    Serial.printf("\n[WCB] Delivery statistics demo. This device is id %u, "
                  "exercising WCB%u.\n", DEVICE_ID, TARGET_WCB);
    printMenu();
}

void loop() {
    // Required every iteration. Besides heartbeats and offline detection, this
    // is what drives the ETM retry engine — which is where `retries` and most
    // of `failed` are counted. Block loop() and the counters stop making sense.
    wcb.update();

    if (autoSend && millis() - lastSendMs >= SEND_INTERVAL_MS) {
        lastSendMs = millis();
        wcb.send(TARGET_WCB, TEST_COMMAND);        // ensured by default
    }

    if (millis() - lastDumpMs >= DUMP_INTERVAL_MS) {
        lastDumpMs = millis();
        dumpStats();
    }

    if (Serial.available()) handleKey((char)Serial.read());
}

// ─────────────────────────────────────────────────────────────────────────────
// The stats table.
//
// Peers with no traffic are skipped so the table stays readable — on a 20-board
// mesh you are usually looking at one or two links.
// ─────────────────────────────────────────────────────────────────────────────
void dumpStats() {
    Serial.printf("\n--- Delivery stats @ %lus | WCB%u %s | auto-send %s ---\n",
                  millis() / 1000, TARGET_WCB,
                  wcb.isOnline(TARGET_WCB) ? "ONLINE" : "OFFLINE",
                  autoSend ? "ON" : "off");
    Serial.println("  peer    sent    ackd   retry    fail  noSlot");

    for (uint8_t id = 1; id <= WCB_MAX_BOARDS; id++) {
        WCBPeerStats s = wcb.getPeerStats(id);
        if (s.sent == 0 && s.ackd == 0 && s.failed == 0 && s.noSlot == 0) continue;
        Serial.printf("  %4u %7lu %7lu %7lu %7lu %7lu%s\n", id,
                      (unsigned long)s.sent,   (unsigned long)s.ackd,
                      (unsigned long)s.retries,(unsigned long)s.failed,
                      (unsigned long)s.noSlot,
                      id == TARGET_WCB ? "   <- target" : "");
    }

    // Deltas for the target since the last dump — this is the line you actually
    // read while flipping the target's power.
    WCBPeerStats t = wcb.getPeerStats(TARGET_WCB);
    if (havePrev) {
        Serial.printf("   +/- %7ld %7ld %7ld %7ld %7ld   (last %lus)\n",
                      (long)t.sent    - (long)prevTarget.sent,
                      (long)t.ackd    - (long)prevTarget.ackd,
                      (long)t.retries - (long)prevTarget.retries,
                      (long)t.failed  - (long)prevTarget.failed,
                      (long)t.noSlot  - (long)prevTarget.noSlot,
                      DUMP_INTERVAL_MS / 1000);
    }
    prevTarget = t;
    havePrev   = true;

    WCBPeerStats agg = wcb.getAggregateStats();
    Serial.printf("   ALL %7lu %7lu %7lu %7lu %7lu\n",
                  (unsigned long)agg.sent,   (unsigned long)agg.ackd,
                  (unsigned long)agg.retries,(unsigned long)agg.failed,
                  (unsigned long)agg.noSlot);
    Serial.printf("  broadcast frames on the air: %lu\n",
                  (unsigned long)wcb.getBroadcastSent());

    checkInvariant();
}

// ─────────────────────────────────────────────────────────────────────────────
// Continuous self-check.
//
// ackd + failed + noSlot <= sent must hold for every peer AND in the total, the
// difference being commands still in the air. A violation is not a lost packet —
// it means a pending slot was settled twice, which is a library bug. Checking it
// automatically is the whole point: nobody eyeballs five columns every 5 s.
// ─────────────────────────────────────────────────────────────────────────────
void checkInvariant() {
    bool ok = true;

    for (uint8_t id = 1; id <= WCB_MAX_BOARDS; id++) {
        WCBPeerStats s = wcb.getPeerStats(id);
        uint32_t settled = s.ackd + s.failed + s.noSlot;
        if (settled > s.sent) {
            Serial.printf("  *** INVARIANT VIOLATED on WCB%u: "
                          "ackd+failed+noSlot (%lu) > sent (%lu) ***\n", id,
                          (unsigned long)settled, (unsigned long)s.sent);
            ok = false;
        }
    }

    WCBPeerStats a = wcb.getAggregateStats();
    long inFlight = (long)a.sent - (long)a.ackd - (long)a.failed - (long)a.noSlot;
    if (inFlight < 0) {
        Serial.printf("  *** INVARIANT VIOLATED in aggregate: in-flight = %ld ***\n",
                      inFlight);
        ok = false;
    }

    if (ok) Serial.printf("  invariant OK | in flight: %ld\n", inFlight);
}

// ─────────────────────────────────────────────────────────────────────────────
// Guided bench walk-through — the sequence that actually proves the counters.
// ─────────────────────────────────────────────────────────────────────────────
void benchGuide() {
    Serial.printf(
        "\n=== Bench procedure for WCB%u ===\n"
        "  0. Press 'z' to zero the counters.\n"
        "\n"
        "  1. TARGET POWERED ON. Press '1' (10 ensured unicasts).\n"
        "     EXPECT  sent +10, ackd +10, failed +0.\n"
        "     Non-zero retries just means a marginal link — real signal, not a bug.\n"
        "\n"
        "  2. POWER THE TARGET OFF, then within ~50 s press '1' again.\n"
        "     The board is still believed online, so ETM retries it.\n"
        "     EXPECT  sent +10, retries ~+30 (3 per command), failed +10.\n"
        "\n"
        "  3. LEAVE IT OFF and wait past ~50 s (watch the header flip to OFFLINE),\n"
        "     then press '1' again.\n"
        "     EXPECT  sent +10, retries +0, failed +10.\n"
        "     >>> This is the important one. Zero retries, and failed MUST still\n"
        "     >>> move. If failed stays flat here, the counter is broken in the\n"
        "     >>> exact way that would never show up in normal use.\n"
        "\n"
        "  4. POWER IT BACK ON, wait for ONLINE, press '3' (ensured broadcast).\n"
        "     EXPECT  every online peer's sent +1, broadcast frames +1.\n"
        "     An ensured broadcast is a per-board guaranteed delivery, so it is\n"
        "     counted against each board — the frame counter is separate, not a\n"
        "     subset of it.\n"
        "\n"
        "  5. Press '5' (raw/Maestro traffic).\n"
        "     EXPECT  NOTHING to move. Raw and Maestro bytes carry no sequence\n"
        "     number and are never ACKd, so there is no delivery to count. This\n"
        "     is correct behaviour, not a bug — check it so nobody reports it.\n"
        "\n"
        "  Throughout: the invariant line must never say VIOLATED.\n"
        "=========================================\n", TARGET_WCB);
}

void printMenu() {
    Serial.println(
        "\nKeys:\n"
        "  1 = 10 ensured unicasts to target    2 = 10 best-effort unicasts\n"
        "  3 = ensured broadcast                4 = best-effort broadcast\n"
        "  5 = raw/Maestro burst (counts NOTHING — that is the point)\n"
        "  6 = 15 rapid ensured sends (tries to saturate the table -> noSlot)\n"
        "  a = toggle 1 Hz auto-send            t = dump stats now\n"
        "  z = reset stats                      g = guided bench procedure\n"
        "  h = this help");
}

void handleKey(char c) {
    switch (c) {
        case '1':
            for (int i = 0; i < 10; i++) wcb.send(TARGET_WCB, TEST_COMMAND);
            Serial.printf("[TX] 10 ensured unicasts -> WCB%u\n", TARGET_WCB);
            break;

        case '2':
            // ensured=false: sent once, no retries. Still ACK-tracked, so it can
            // still resolve to ackd — or to failed on the 1 s slot timeout.
            for (int i = 0; i < 10; i++) wcb.send(TARGET_WCB, TEST_COMMAND, false);
            Serial.printf("[TX] 10 best-effort unicasts -> WCB%u\n", TARGET_WCB);
            break;

        case '3':
            wcb.broadcast(TEST_COMMAND);
            Serial.println("[TX] ensured broadcast (credits every online peer)");
            break;

        case '4':
            wcb.broadcast(TEST_COMMAND, false);
            Serial.println("[TX] best-effort broadcast (frame counter only)");
            break;

        case '5': {
            // sendRaw bypasses the COMMAND layer entirely — its own packet type,
            // no sequence number, no ACK. Deliberately invisible to the counters.
            uint8_t maestro[] = { 0x84, 0x00, 0x70, 0x2E };   // Maestro set-target
            for (int i = 0; i < 10; i++)
                wcb.sendRaw(TARGET_WCB, 1, maestro, sizeof(maestro));
            Serial.println("[TX] 10 raw packets — expect NO counter movement");
            break;
        }

        case '6':
            // The pending table holds WCB_PENDING_MAX (10) in-flight COMMANDS.
            // Firing more than that in one loop pass, with no chance for update()
            // to service ACKs in between, is how a send gets denied a slot: it
            // goes out once, untracked, and lands in noSlot. Most visible when
            // the target is OFF, since then nothing is ACKing slots free.
            for (int i = 0; i < 15; i++) wcb.send(TARGET_WCB, TEST_COMMAND);
            Serial.println("[TX] 15 rapid ensured sends — watch noSlot");
            break;

        case 'a':
            autoSend = !autoSend;
            Serial.printf("[--] auto-send %s\n", autoSend ? "ON (1 Hz)" : "off");
            break;

        case 't': dumpStats(); break;

        case 'z':
            // Safe from loop(); it takes the pending-table lock, so never call it
            // from inside a receive callback. In-flight commands are re-credited
            // so a reset cannot leave ackd landing against a zeroed sent.
            wcb.resetStats();
            havePrev = false;
            Serial.println("[--] stats reset");
            break;

        case 'g': benchGuide(); break;
        case 'h': printMenu();  break;
        default: break;
    }
}
