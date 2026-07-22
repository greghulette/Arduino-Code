/*
  MgmtRelay.ino — turn any ESP32 into a WCB management relay (USB serial ↔ ESP-NOW)

  Flash this to ANY ESP32, plug it into USB, and it becomes a transparent doorway
  into your WCB network. Whatever you (or a host tool like the NaviCore Config
  Tool in "Via WCB" mode) send on USB serial is relayed onto the mesh, and every
  reply from the mesh is printed straight back to USB. That is ALL it does — it
  hosts no servos, audio, LEDs, or PWM; its sole purpose is to be the relay.

  Use it to:
    • Run the NaviCore Config Tool over a spare ESP32 instead of a real WCB —
      "Via WCB" mode talks to this relay and manages every WCB *and* the NaviCore
      through this one USB port.
    • Fire ad-hoc commands at any board from a plain Serial Monitor.
    • Watch the mesh's JSON telemetry / replies scroll by.

  ── Serial protocol (matches the WCB firmware's own serial port) ───────────────
    ;w<id>,<command>    Relay <command> to WCB <id> over ESP-NOW (unicast).
                        <id> 1–19 = a WCB; 20 = the NaviCore (special peer).
                        Two-digit ids REQUIRE the comma:  ;w20,{"type":"PING"}
    ;w<alias>,<command> Same, but address a board by its advertised WDP alias
                        (e.g. ;wdome,:PP100) — resolved from adverts we've heard.
    <plain text, no ';' prefix>
                        BROADCAST to every board, exactly like typing a bare
                        command into a WCB's serial port. (A bare JSON line —
                        one starting with '{' — is IGNORED; see the note below.)

  ── Using it with the NaviCore Config Tool ────────────────────────────────────
  Just Connect. The tool pings for a DIRECT reply first, gets none through the
  relay, and auto-switches to "Via WCB" mode — wrapping every command as
  ;w20,{...} and fragmenting large configs — with no manual toggle. That
  auto-detect is exactly why a bare JSON line is ignored above: relaying raw JSON
  would let the tool latch onto Direct-USB mode, where big configs are sent
  UNFRAGMENTED and would overflow this relay. In "Via WCB" mode everything is
  wrapped as ;w20,{...}, so it flows correctly.

  Inbound: any command/reply a board sends to this relay (or broadcasts) is
  printed to USB verbatim — so a NaviCore JSON reply reaches the Config Tool's
  parser unchanged (the sender's id travels *inside* the JSON). That includes the
  5 Hz rc_ch channel stream, which drives the Config Tool's live joystick / channel
  display; it's low bandwidth and, because every USB write is done from loop() (see
  the RelayLine note), it doesn't flood or corrupt the stream.

  ── Setup ─────────────────────────────────────────────────────────────────────
    1. Fill in the four network credentials below to match your WCB system.
    2. Give this relay a DEVICE_ID no real WCB uses. A high id like the default 19
       is the natural pick — it won't collide with a real board's slot. It's above
       WCB_QUANTITY, so begin() logs a harmless one-line WARNING; that's fine — the
       NaviCore/WCBs register this relay to reply the moment it first talks to them
       (needs WCB_Client 1.9.6+), and it also advertises over WDP so they remember
       it across reboots.
    2b. Set WCB_QUANTITY to your real number of WCBs. The relay pre-registers those
       (1..WCB_QUANTITY), so ;w<id>, reaches each of them right away; boards above
       that number are reached once the relay auto-joins them from their adverts.
    3. To talk to the NaviCore, the WCBs must have the special peer enabled:
       ?SPECIAL,ON,20  (and the NaviCore must be at that id — 20 by default).
    4. Flash to any ESP32 — no wiring, everything is wireless.
*/

#include <WCB_Client.h>

// ── Network credentials — must match your WCB system exactly (see BasicUsage) ──
//   MAC_OCT2 / MAC_OCT3 : shared MAC octets that identify your network (?WCBM)
//   PASSWORD            : the ESP-NOW network password (?WCBP)
//   WCB_QUANTITY        : total number of WCBs in the system (?WCBQ)
//   DEVICE_ID           : a unique id for THIS relay — see setup note #2
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 4;
const uint8_t DEVICE_ID    = 19;

// The NaviCore / out-of-band controller lives at the special-peer id (default 20).
const uint8_t NAVICORE_ID  = WCB_SPECIAL_ID;   // = 20

// This relay's firmware string, advertised over WDP so it shows up by name.
const char*   RELAY_FW     = "1.0";

// Name this relay advertises (WDP) and reports to the WCB Wizard as its alias.
const char*   RELAY_ALIAS  = "Mgmt Relay";

// 1 = also echo each relayed line + board online/offline to USB (handy from a
// plain Serial Monitor). Leave 0 when a tool is driving the port so the only
// non-JSON noise is startup text.
#define VERBOSE 0

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// Inbound mesh lines are queued here on the receive callback and PRINTED from
// loop() (Core 1) — never printed straight from onMeshData. That callback runs on
// the ESP-NOW receive task (Core 0); doing a multi-USB-packet Serial write there
// races Core 1's Serial use and corrupts any line longer than one 64-byte packet
// (CONFIG fragments, rc_hb) while small ones (PONG) sneak through. This is the same
// defer-to-loop discipline the WCB firmware uses for its JSON relay.
struct RelayLine { char buf[208]; };
QueueHandle_t relayOutQueue = nullptr;

// ── WCB Wizard remote management — Phase 2a: FRAG forwarding ───────────────────
// The Wizard manages a REMOTE board THROUGH this relay by sending, on USB serial:
//   ?MGMT,FRAG,<targetWCB>,<sessionHex>,<chunkIdx>,<totalChunks>,<payload>
// Each FRAG is forwarded to the target EXACTLY like the WCB firmware relay
// (WCB.ino handleMgmtFrag ~2394-2426), so a real WCB and this relay are interchangeable:
//   • single-chunk (totalChunks==1): one ETM unicast with a '\x01' (SOH) wizard-origin
//     marker prepended, so the target resets lastReceivedViaESPNOW and the relayed
//     command propagates just like a locally-typed one (Push-small / Test / IDENTIFY /
//     WLED / VAR / sequence-test / remote-terminal start-stop-keystrokes).
//   • multi-chunk (a big config Push): each chunk is forwarded VERBATIM as a raw type-3
//     wcb_packet_mgmt_t (WCB_MGMT_PACKET_TYPE_FRAG); the TARGET reassembles per session
//     and runs it. So there is NO relay-side reassembly, no relay-side size cap, and the
//     target keeps wizard-origin (fan-out) semantics. (wcb.send() must NOT be used for
//     multi-chunk: it caps at WCB_MGMT_MAX_COMMAND_LEN and fragments as type-5 UNICAST,
//     which silently drops oversized configs and loses fan-out.)
//
// Phase 2b: ?MGMT,PULL / STATS / ETM,CHAR + the remote-terminal OUTPUT stream. These need
// firmware ESP-NOW structs that WCB_Client does NOT expose, so they are replicated here
// byte-for-byte. Flow: the relay unicasts a 43-byte config-request to the target; the target
// broadcasts back 230-byte config-fragments (config / stats / etm — distinguished by
// packetType) which we reassemble per session and print as [MGMT:CONFIG|STATS|ETM,<src>]<data>;
// a target running a remote terminal unicasts 204-byte remote-term packets which we print as
// [TERM:<src>]<line>. The raw receive callback runs on the WiFi task, so it ONLY copies the
// packet into a queue — all decode/reassembly/Serial output happens in loop() (single writer).

// Firmware packet types (WCB.ino ~223-233, WCB_RemoteTerm.h:42) — mirror exactly.
#define PT_MGMT_FRAG    3
#define PT_CONFIG_REQ   5
#define PT_CONFIG_FRAG  6
#define PT_STATS_REQ    7
#define PT_ETM_REQ      8
#define PT_STATS_FRAG   9
#define PT_ETM_FRAG     10
#define PT_REMOTE_TERM  7      // in the 204-byte struct — SIZE disambiguates from PT_STATS_REQ (43B)
#define CFG_PAYLOAD     183    // firmware CONFIG_PAYLOAD_SIZE
#define MGMT_CHUNKS     16     // firmware MGMT_MAX_CHUNKS (uint16 receivedMask)

// 43 B — relay → target request (config / stats / etm; only packetType differs).
typedef struct __attribute__((packed)) {
  char    structPassword[40];
  uint8_t packetType;
  uint8_t targetWCB;
  uint8_t requesterWCB;
} relay_config_req;

// 230 B — target → relay reply fragment (config / stats / etm).
typedef struct __attribute__((packed)) {
  char     structPassword[40];
  uint8_t  packetType;
  uint8_t  sourceWCB;
  uint8_t  requesterWCB;
  uint16_t sessionId;
  uint8_t  chunkIdx;
  uint8_t  totalChunks;
  char     payload[CFG_PAYLOAD];
} relay_config_frag;

// 204 B — target → relay remote-terminal line.
typedef struct __attribute__((packed)) {
  char    structPassword[40];
  uint8_t packetType;
  uint8_t sourceWCB;
  uint8_t destWCB;
  uint8_t textLen;
  char    text[160];
} relay_remote_term;

// Raw inbound packets are copied on the WiFi task and processed from loop(). 232 B covers
// the larger of the two structs (config-frag 230 / remote-term 204).
struct RawPkt { uint8_t len; uint8_t data[232]; };
QueueHandle_t rawInQueue = nullptr;

// Reassembly state for one config/stats/etm reply at a time (the Wizard requests serially).
struct FragReasm {
  bool     active      = false;
  uint8_t  packetType  = 0;
  uint8_t  sourceWCB   = 0;
  uint16_t sessionId   = 0;
  uint8_t  totalChunks = 0;
  uint16_t receivedMask= 0;
  unsigned long lastMs = 0;
  char     chunks[MGMT_CHUNKS][CFG_PAYLOAD + 1];
};
static FragReasm fragReasm;
static uint16_t  lastDeliveredSession = 0;   // suppress the CONFIG second pass (target sends x2)

// ── Inbound: mesh → USB ───────────────────────────────────────────────────────
// Fires for every command a board sends to this relay OR broadcasts. Queue it for
// loop() to print verbatim so a JSON reply reaches a host tool's parser unmodified.
// Everything is forwarded — including the 5 Hz rc_ch channel stream, which is what
// drives the Config Tool's live joystick / channel display. It's low bandwidth
// (~640 B/s) and every Serial write happens on the loop task, so it neither floods
// nor corrupts.
void onMeshData(uint8_t senderID, const char* command) {
    (void)senderID;                          // the id is carried inside JSON replies
    if (!relayOutQueue) return;
    RelayLine line;                          // copy — `command` is valid only for this call
    size_t n = strlen(command);
    if (n >= sizeof(line.buf)) n = sizeof(line.buf) - 1;
    memcpy(line.buf, command, n);
    line.buf[n] = '\0';
    xQueueSend(relayOutQueue, &line, 0);     // non-blocking; a dropped line is re-sent by the sender's ETM retry
}

// Optional: report boards coming/going (only when VERBOSE).
void onMeshStatus(uint8_t wcbID, bool online) {
    (void)wcbID; (void)online;
#if VERBOSE
    Serial.printf("[relay] %s%d %s\n",
                  wcbID == NAVICORE_ID ? "NaviCore/id" : "WCB",
                  wcbID, online ? "online" : "offline");
#endif
}

// Case-insensitive string equality (avoids depending on strcasecmp being visible).
static bool ieq(const char* a, const char* b) {
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca += 32;
        if (cb >= 'A' && cb <= 'Z') cb += 32;
        if (ca != cb) return false;
        a++; b++;
    }
    return *a == *b;
}

// Case-insensitive: does `s` begin with `prefix`?
static bool iStarts(const char* s, const char* prefix) {
    while (*prefix) {
        char cs = *s, cp = *prefix;
        if (cs >= 'A' && cs <= 'Z') cs += 32;
        if (cp >= 'A' && cp <= 'Z') cp += 32;
        if (cs != cp) return false;
        s++; prefix++;
    }
    return true;
}

// Resolve a board by its advertised WDP alias. Returns 0 if none is known.
uint8_t idForAlias(const char* alias) {
    for (uint8_t id = 1; id <= WCB_MAX_BOARDS; id++) {
        const WCBNeighbor* nb = wcb.getNeighbor(id);
        if (nb && nb->name[0] && ieq(nb->name, alias)) return id;
    }
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  WCB Wizard support (the WCB config web tool)
//
//  The Wizard drives a WCB over USB with WCB-FIRMWARE commands (?backup / ?version
//  / ?WDP,DUMP / ?MGMT,...), NOT the ;w / JSON the NaviCore tool uses — both coexist
//  here (dispatched by prefix in relaySerialLine). These replies are byte-matched to
//  the real WCB firmware so the Wizard's parser accepts them, and all print from
//  loop() (Core 1) — single-writer Serial, no cross-core corruption.
//  Phase 1  = the connect handshake + mesh discovery (this device answers as itself).
//  Phase 2a = FRAG forwarding (handleMgmtFrag): remote Push / Test / IDENTIFY / WLED /
//             VAR / sequence-test + remote-terminal INPUT flow to any board through here.
//  Phase 2b = remote PULL / STATS / ETM,CHAR + remote-terminal OUTPUT (still TODO — needs
//             the raw config-req / config-frag / remote-term structs, see onMeshData note).
// ═══════════════════════════════════════════════════════════════════════════════

// Replace WDP-grammar-breaking chars in a free-text field with '_', like the WCB's
// wdpScrub(): a ',' or ']' or control char inside a [WDP:...] token would corrupt it.
static void wdpScrub(char* s) {
    for (; *s; s++)
        if (*s == ',' || *s == ']' || (unsigned char)*s < 0x20) *s = '_';
}

// ?backup / WCB_WEBTOOL_CONFIG_PULL → a minimal WCB-style config backup. The Wizard
// only requires the "WCB Configuration Backup" header + an "End of Backup" line to
// accept the device; the ?TOKEN lines identify this relay's slot (id/MAC/quantity).
void wcbPrintBackup() {
    Serial.println();
    Serial.println("*** ========================================");
    Serial.println("*** WCB Configuration Backup");
    Serial.println("*** Copy and paste these commands to restore");
    Serial.println("*** ========================================");
    Serial.println();
    Serial.println("?HW,32");
    Serial.printf ("?MAC,2,%02X\n", MAC_OCT2);
    Serial.printf ("?MAC,3,%02X\n", MAC_OCT3);
    Serial.printf ("?WCB,%d\n",     DEVICE_ID);
    Serial.println("?RELAY,1");     // marks this device a management relay → dedicated Wizard card
    Serial.printf ("?ALIAS,%s\n",   RELAY_ALIAS);
    Serial.printf ("?WCBQ,%d\n",    WCB_QUANTITY);
    Serial.printf ("?EPASS,%s\n",   PASSWORD);
    Serial.println("?CMDCHAR,;");
    Serial.println("--------- End of Backup ---------");
    Serial.println();
}

// ?version → the Wizard reads "Software Version: X", terminated by "End of Version".
void wcbPrintVersion() {
    Serial.printf("Software Version: %s\n", RELAY_FW);
    Serial.println("End of Version");
}

// ?WDP,DUMP → the mesh inventory the Wizard's discovery panel shows. Built from this
// relay's WDP neighbor table (getNeighbor), formatted byte-for-byte like the WCB's
// printWdpDump(): a SELF row, one [WDP:...] per neighbor (+ [WDPIF:...] port labels),
// a [WDPCFG:...] summary, and a terminating [WDP:END,count=N].
void wcbPrintWdpDump() {
    unsigned long now = millis();
    // SELF row (this relay). CLIENT=0 → shown as the tethered board; PEER=3 = "this board".
    Serial.printf("[WDP:N=%d,CLIENT=0,ALIAS=%s,HW=32,HWREV=,FW=%s,CAP=0000,CTRL=0,CAPTAGS=,MAESTRO=-,AGE=0,SEEN=1,PEER=3]\n",
                  DEVICE_ID, RELAY_ALIAS, RELAY_FW);
    int count = 0, peers = 0;
    for (uint8_t id = 1; id <= WCB_MAX_BOARDS; id++) {
        if (id <= WCB_QUANTITY || wcb.isLearnedPeer(id)) peers++;
        const WCBNeighbor* nb = wcb.getNeighbor(id);
        if (!nb) continue;
        count++;
        char alias[25];   strncpy(alias,   nb->name,    sizeof(alias));   alias[sizeof(alias)-1]     = '\0'; wdpScrub(alias);
        char fw[28];      strncpy(fw,      nb->fw,      sizeof(fw));       fw[sizeof(fw)-1]           = '\0'; wdpScrub(fw);
        char hwrev[16];   strncpy(hwrev,   nb->hwRev,   sizeof(hwrev));    hwrev[sizeof(hwrev)-1]     = '\0'; wdpScrub(hwrev);
        char captags[49]; strncpy(captags, nb->capTags, sizeof(captags)); captags[sizeof(captags)-1] = '\0'; wdpScrub(captags);
        char maestro[40];
        if (nb->maestroCount == 0) { maestro[0] = '-'; maestro[1] = '\0'; }
        else {
            int o = 0; maestro[0] = '\0';
            for (int m = 0; m < nb->maestroCount && m < 9; m++)
                o += snprintf(maestro + o, sizeof(maestro) - o, "%s%d", m ? "." : "", nb->maestroIds[m]);
        }
        int peerFlag = wcb.isLearnedPeer(id) ? 2 : (id <= WCB_QUANTITY ? 1 : 0);
        Serial.printf("[WDP:N=%d,CLIENT=%d,ALIAS=%s,HW=%d,HWREV=%s,FW=%s,CAP=%04X,CTRL=%d,CAPTAGS=%s,MAESTRO=%s,AGE=%lu,SEEN=1,PEER=%d]\n",
                      id, nb->isClient ? 1 : 0, alias, nb->hwVer, hwrev, fw, (unsigned)nb->capFlags,
                      nb->ctrlId, captags, maestro, (now - nb->lastSeenMs) / 1000UL, peerFlag);
        for (int p = 0; p < 5; p++) {
            if (!nb->portLabels[p][0]) continue;
            char lbl[25]; strncpy(lbl, nb->portLabels[p], sizeof(lbl)); lbl[sizeof(lbl)-1] = '\0'; wdpScrub(lbl);
            Serial.printf("[WDPIF:N=%d,S=%d,DEV=%s]\n", id, p + 1, lbl);
        }
    }
    Serial.printf("[WDPCFG:EN=1,AUTOJOIN=%d,PEERS=%d]\n", wcb.autoJoinEnabled() ? 1 : 0, peers);
    Serial.printf("[WDP:END,count=%d]\n", count);
}

// Forward a Wizard "?MGMT,FRAG,..." line to the target board (Phase 2a).
// `frag` points AFTER "?MGMT,FRAG,", i.e. "<targetWCB>,<sessionHex>,<idx>,<total>,<payload>".
// The payload may itself contain commas (e.g. ?SEQ,SAVE,key,val), so only the first
// FOUR fields are split off and everything after the 4th comma is the payload.
void handleMgmtFrag(char* frag) {
  char* c1 = strchr(frag,   ','); if (!c1) return;
  char* c2 = strchr(c1 + 1, ','); if (!c2) return;
  char* c3 = strchr(c2 + 1, ','); if (!c3) return;
  char* c4 = strchr(c3 + 1, ','); if (!c4) return;
  *c1 = *c2 = *c3 = *c4 = '\0';
  uint8_t     target  = (uint8_t)atoi(frag);
  uint16_t    sid     = (uint16_t)strtoul(c1 + 1, nullptr, 16);   // session id is hex
  uint8_t     idx     = (uint8_t)atoi(c2 + 1);
  uint8_t     total   = (uint8_t)atoi(c3 + 1);
  const char* payload = c4 + 1;                                   // remainder (may contain commas)

  if (target < 1 || target > WCB_MAX_BOARDS || total == 0) return;

  // Single-chunk: one ETM unicast, '\x01' wizard-origin marker prepended so the
  // target treats it like a locally-typed command (mirrors the WCB firmware relay).
  if (total == 1) {
    String marked = String("\x01") + payload;
    wcb.send(target, marked.c_str(), true);          // ensured (ETM + CRC); auto-frags if long
#if VERBOSE
    Serial.printf("[relay] MGMT -> WCB%d (1/1): %s\n", target, payload);
#endif
    return;
  }

  // Multi-chunk (big config push): forward THIS chunk verbatim as a raw type-3 MGMT_FRAG
  // packet. The TARGET reassembles per (targetWCB, sessionId) and runs the whole command —
  // mirroring the WCB firmware relay exactly (WCB.ino ~2415-2426). No relay-side reassembly
  // or size cap (the target enforces its own MGMT_MAX_CHUNKS ceiling, same as a real WCB),
  // and wizard-origin (fan-out) semantics are preserved by packetType 3.
  wcb_packet_mgmt_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  strncpy(pkt.structPassword, PASSWORD, sizeof(pkt.structPassword) - 1);
  pkt.packetType  = WCB_MGMT_PACKET_TYPE_FRAG;       // 3 = wizard-origin (target re-broadcasts allowed)
  pkt.targetWCB   = target;
  pkt.sessionId   = sid;
  pkt.chunkIdx    = idx;
  pkt.totalChunks = total;
  strncpy(pkt.payload, payload, sizeof(pkt.payload) - 1);
  pkt.payload[sizeof(pkt.payload) - 1] = '\0';
  wcb.sendRawPacket(target, (const uint8_t*)&pkt, sizeof(pkt));
#if VERBOSE
  Serial.printf("[relay] MGMT -> WCB%d frag %u/%u (session %04X)\n", target, idx + 1, total, sid);
#endif
}

// ── Phase 2b: PULL / STATS / ETM,CHAR requests + reply reassembly + remote terminal ──────

// Send a config/stats/etm request to the target (relay → target). PULL is sent x3 for loss
// resilience (a first frame to a board is often dropped on a busy mesh); the target dedups.
void sendConfigReq(uint8_t target, uint8_t packetType, uint8_t times) {
  if (target < 1 || target > WCB_MAX_BOARDS) return;
  relay_config_req req;
  memset(&req, 0, sizeof(req));
  strncpy(req.structPassword, PASSWORD, sizeof(req.structPassword) - 1);
  req.packetType   = packetType;
  req.targetWCB    = target;
  req.requesterWCB = DEVICE_ID;                // replies come back addressed to us
  for (uint8_t i = 0; i < times; i++) {
    wcb.sendRawPacket(target, (const uint8_t*)&req, sizeof(req));
    if (i + 1 < times) delay(15);
  }
}

// WiFi-task callback: copy config-frag(230) / remote-term(204) packets for loop() to process.
// MUST NOT print — a multi-packet Serial write here races Core 1 and corrupts long lines.
void onMeshRaw(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (!rawInQueue) return;
  if (len != (int)sizeof(relay_config_frag) && len != (int)sizeof(relay_remote_term)) return;
  RawPkt r; r.len = (uint8_t)len; memcpy(r.data, data, len);
  xQueueSend(rawInQueue, &r, 0);               // non-blocking; a dropped frag is covered by the 2nd pass
}

// loop(): reassemble a config/stats/etm reply fragment; emit [MGMT:CONFIG|STATS|ETM,<src>]<data>
// when the session is complete. Mirrors WCB.ino handleConfigFragPacket / handleStats/ETMFragPacket.
void processConfigFrag(const uint8_t* data) {
  relay_config_frag pkt;
  memcpy(&pkt, data, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (strcmp(pkt.structPassword, PASSWORD) != 0) return;
  if (pkt.requesterWCB != DEVICE_ID) return;
  if (pkt.totalChunks == 0 || pkt.totalChunks > MGMT_CHUNKS) return;
  if (pkt.chunkIdx >= MGMT_CHUNKS || pkt.chunkIdx >= pkt.totalChunks) return;
  const char* tag = pkt.packetType == PT_CONFIG_FRAG ? "CONFIG"
                  : pkt.packetType == PT_STATS_FRAG  ? "STATS"
                  : pkt.packetType == PT_ETM_FRAG    ? "ETM" : nullptr;
  if (!tag) return;
  if (lastDeliveredSession != 0 && pkt.sessionId == lastDeliveredSession) return;  // CONFIG 2nd pass

  if (!fragReasm.active || fragReasm.sessionId != pkt.sessionId) {
    memset(&fragReasm, 0, sizeof(fragReasm));
    fragReasm.active     = true;
    fragReasm.packetType = pkt.packetType;
    fragReasm.sourceWCB  = pkt.sourceWCB;
    fragReasm.sessionId  = pkt.sessionId;
    fragReasm.totalChunks= pkt.totalChunks;
  }
  if (!(fragReasm.receivedMask & (1 << pkt.chunkIdx))) {
    strncpy(fragReasm.chunks[pkt.chunkIdx], pkt.payload, CFG_PAYLOAD);
    fragReasm.chunks[pkt.chunkIdx][CFG_PAYLOAD] = '\0';
    fragReasm.receivedMask |= (1 << pkt.chunkIdx);
  }
  fragReasm.lastMs = millis();

  uint16_t expected = (uint16_t)((1 << pkt.totalChunks) - 1);
  if (fragReasm.receivedMask == expected) {
    String full = "";
    for (int i = 0; i < pkt.totalChunks; i++) full += fragReasm.chunks[i];
    uint8_t src = fragReasm.sourceWCB;
    lastDeliveredSession = pkt.sessionId;
    memset(&fragReasm, 0, sizeof(fragReasm));
    Serial.printf("[MGMT:%s,%d]%s\n", tag, src, full.c_str());
  }
}

// loop(): a remote-terminal line from a target → [TERM:<src>]<line> (trailing CR/LF stripped),
// which the Wizard demuxes into per-board panes. Mirrors WCB_RemoteTerm.cpp:161-176.
void processRemoteTerm(const uint8_t* data) {
  relay_remote_term pkt;
  memcpy(&pkt, data, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (strcmp(pkt.structPassword, PASSWORD) != 0) return;
  if (pkt.destWCB != DEVICE_ID) return;
  if (pkt.textLen == 0 || pkt.textLen > sizeof(pkt.text)) return;   // firmware drops these (WCB_RemoteTerm.cpp:142)
  uint8_t len = pkt.textLen;
  char line[sizeof(pkt.text) + 1];
  memcpy(line, pkt.text, len);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) len--;
  if (len == 0) return;                                            // blank line: firmware prints nothing (line 171)
  line[len] = '\0';
  Serial.printf("[TERM:%d]%s\n", pkt.sourceWCB, line);
}

// ── Serial → mesh: parse one line and relay it ────────────────────────────────
// Mirrors the WCB firmware's serial routing:
//   ;w<id>,<cmd> / ;w<alias>,<cmd>  → unicast to that board
//   <no ';' prefix>                 → broadcast to the whole mesh
// The `line` buffer is modified in place (temporary NUL terminators).
void relaySerialLine(char* line) {
    // ── WCB Wizard: answer its local (?) queries so it accepts this relay and can
    // drive the mesh through it. These coexist with the NaviCore tool's ;w / JSON.
    if (!strcmp(line, "WCB_WEBTOOL_CONFIG_PULL")) { wcbPrintBackup(); return; }
    if (line[0] == '?') {
        const char* c = line + 1;                // command after the '?' funcChar
        if (ieq(c, "backup"))  { wcbPrintBackup();  return; }
        if (ieq(c, "version")) { wcbPrintVersion(); return; }
        if (iStarts(c, "WDP,")) {
            if (ieq(c + 4, "DUMP")) wcbPrintWdpDump();
            // ?WDP,AUTOJOIN / FORGET / CLEAR — a later phase.
            return;
        }
        if (iStarts(c, "MGMT,")) {
            // Route the Wizard's remote-management verbs to the target board.
            //   FRAG      → forward config/command chunks (Phase 2a)
            //   PULL      → request the target's config     (reply: [MGMT:CONFIG,<n>])
            //   STATS     → request ESP-NOW stats            (reply: [MGMT:STATS,<n>])
            //   ETM,CHAR  → request ETM characterization     (reply: [MGMT:ETM,<n>])
            // Remote-terminal START/STOP/keystrokes ride inside a FRAG; its OUTPUT comes back
            // as [TERM:<n>] via processRemoteTerm.
            char* m = line + 6;                  // skip "?MGMT,"
            if      (iStarts(m, "FRAG,"))     handleMgmtFrag(m + 5);
            else if (iStarts(m, "PULL,"))     sendConfigReq((uint8_t)atoi(m + 5), PT_CONFIG_REQ, 3);
            else if (iStarts(m, "STATS,"))    sendConfigReq((uint8_t)atoi(m + 6), PT_STATS_REQ, 1);
            else if (iStarts(m, "ETM,CHAR,")) sendConfigReq((uint8_t)atoi(m + 9), PT_ETM_REQ,   1);
            return;
        }
        return;                                  // other ? queries: answer nothing, don't broadcast to the mesh
    }

    // No ';' command prefix:
    if (line[0] != ';') {
        // A bare JSON line means a host tool is (wrongly) in Direct-USB mode.
        // Ignore it so the Config Tool's transport probe gets no reply and
        // auto-switches to "Via WCB" (JSON wrapped as ;w20,{...}, big configs
        // fragmented). Relaying it would strand the tool in Direct-USB mode.
        if (line[0] == '{') return;
        // Plain text → broadcast to all boards (like a bare line on a WCB).
        wcb.broadcast(line);
#if VERBOSE
        Serial.printf("[relay] broadcast: %s\n", line);
#endif
        return;
    }

    // Only the ;w relay verb is handled — this device is a relay, not a full WCB.
    char verb = line[1];
    if (verb != 'w' && verb != 'W') {
        Serial.printf("[relay] unsupported ;%c command — this relay only handles ;w<id>,<cmd>\n",
                      verb ? verb : '?');
        return;
    }

    char* p = line + 2;                      // first char after ";w"
    if (*p == '\0') { Serial.println("[relay] usage: ;w<id>,<command>"); return; }

    uint8_t     target  = 0;
    const char* payload = nullptr;

    if (*p >= '0' && *p <= '9') {
        char* d = p;
        while (*d >= '0' && *d <= '9') d++;
        if (*d == ',') {                     // ;w<digits>,<cmd> → whole run is the id (10–20 ok)
            *d = '\0';
            target  = (uint8_t)atoi(p);
            payload = d + 1;
        } else {                             // ;w<d><rest> → single-digit id, rest is payload
            target  = (uint8_t)(*p - '0');
            payload = p + 1;
        }
    } else {                                 // ;w<alias>,<cmd>
        char* comma = strchr(p, ',');
        if (!comma) { Serial.println("[relay] usage: ;w<alias>,<command>"); return; }
        *comma  = '\0';
        target  = idForAlias(p);
        payload = comma + 1;
        if (!target) { Serial.printf("[relay] no board known as \"%s\"\n", p); return; }
    }

    if (target < 1 || target > WCB_MAX_BOARDS) {
        Serial.printf("[relay] bad target id %d (use 1-%d, or %d for NaviCore)\n",
                      target, WCB_MAX_BOARDS, NAVICORE_ID);
        return;
    }

    wcb.send(target, payload);               // ensured (ETM + CRC); auto-fragments if long
#if VERBOSE
    Serial.printf("[relay] -> WCB%d: %s\n", target, payload);
#endif
}

void setup() {
    Serial.begin(115200);
    // Queue for inbound mesh lines, printed from loop() (see the RelayLine note).
    // Created before begin() so it exists before the receive callback can fire.
    relayOutQueue = xQueueCreate(24, sizeof(RelayLine));
    rawInQueue    = xQueueCreate(24, sizeof(RawPkt));   // Phase 2b: config/stats/etm/term replies
    wcb.onCommand(onMeshData);
    wcb.onStatusChange(onMeshStatus);
    wcb.onRawPacket(onMeshRaw);                          // Phase 2b: raw reply/terminal packets

    // Join the WCB network. false = ESP-NOW did not start; halt rather than run
    // update()/send against an uninitialised driver (which would crash).
    if (!wcb.begin()) {
        Serial.println("[relay] begin() FAILED (see error above) — halting.");
        while (true) delay(1000);
    }

    // Reach the NaviCore (special peer) so ;w20,... works, and advertise this
    // relay over WDP so boards discover and auto-join it (and can reply to it).
    wcb.enableSpecialPeer(NAVICORE_ID);
    // Advertise as a TEMPORARY peer: WCBs adopt this relay only while it's actively
    // advertising and drop it on silence (~3 min) and on reboot — instead of remembering
    // it as a permanent peer like other clients. Ideal for a management relay you only
    // connect now and then. Remove this line to auto-join permanently instead.
    wcb.setTemporary(true);
    wcb.setIdentity(RELAY_ALIAS, RELAY_FW);

    // Boot lines the WCB Wizard's sniffer reads to learn this device's command
    // chars (funcChar '?', cmdChar ';') + version. Also satisfies the Wizard's
    // auto-detect, which triggers on ANY bytes emitted after reset.
    Serial.println("Delimeter Character: ^");
    Serial.println("Local Function Identifier: ?");
    Serial.println("Command Character: ;");
    Serial.printf ("Software Version: %s\n", RELAY_FW);

    Serial.println("[relay] WCB management relay ready.");
    Serial.println("[relay]   ;w<id>,<cmd>    relay to WCB <id> (20 = NaviCore)");
    Serial.println("[relay]   ;w<alias>,<cmd> relay by advertised name");
    Serial.println("[relay]   <text>          broadcast to all boards");
}

char   inBuf[256];
size_t inLen = 0;

void loop() {
    wcb.update();                            // heartbeats out, offline detection, WDP, retries

    // Print inbound mesh lines the receive callback queued. Doing ALL Serial
    // writes from this one task (Core 1) is what keeps large lines (CONFIG
    // fragments, rc_hb) from being corrupted by a cross-core write race.
    RelayLine out;
    while (relayOutQueue && xQueueReceive(relayOutQueue, &out, 0) == pdTRUE)
        Serial.println(out.buf);

    // Decode raw config/stats/etm/terminal packets queued by onMeshRaw (Phase 2b).
    RawPkt rp;
    while (rawInQueue && xQueueReceive(rawInQueue, &rp, 0) == pdTRUE) {
        if      (rp.len == (uint8_t)sizeof(relay_config_frag)) processConfigFrag(rp.data);
        else if (rp.len == (uint8_t)sizeof(relay_remote_term)) processRemoteTerm(rp.data);
    }
    // Drop a stalled reassembly (lost fragment) after 6 s so a later pull isn't wedged.
    if (fragReasm.active && (millis() - fragReasm.lastMs) > 6000) fragReasm.active = false;

    // Read USB serial one line at a time (CR/LF terminated) and relay it.
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (inLen) { inBuf[inLen] = '\0'; relaySerialLine(inBuf); inLen = 0; }
        } else if (inLen < sizeof(inBuf) - 1) {
            inBuf[inLen++] = c;
        } else {
            inLen = 0;                        // overrun — drop the oversized line
            Serial.println("[relay] line too long — dropped (use the Config Tool for big configs)");
        }
    }
}
