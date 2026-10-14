#pragma once
// =============================================================================
//  WCB_Mgmt.h — the WCB Wizard's management surface, for any WCB_Client host
// =============================================================================
//
//  Lets a device act as the Wizard's doorway into the mesh: the Wizard talks to
//  it over serial or a WebSocket, and it forwards config pulls, pushes, tests and
//  remote-terminal traffic to every WCB over ESP-NOW.
//
//  ── WHY THIS IS A LIBRARY MODULE AND NOT A COPY ─────────────────────────────
//  It began life inside examples/MgmtRelay. NaviCore needs the same surface — it
//  already links WCB_Client, already dispatches "?" commands through one handler
//  shared by USB and its WebSocket, and already relays firmware OTA to WCBs — so
//  the obvious move was to copy those ~250 lines across.
//
//  That would have been a mistake of a very specific kind. This surface is a WIRE
//  PROTOCOL with three parties: the Wizard, this device, and the WCB firmware.
//  Every struct here is byte-matched to WCB.ino, and every reply string is
//  byte-matched to what the Wizard's parser expects. Two copies in two firmwares
//  drift the first time one side is fixed and the other is not — and the failure
//  is silent, because a stale copy still answers, just wrongly.
//
//  The ecosystem already has the answer to exactly this problem: WcbCmd is one
//  repo compiled by two firmwares precisely so a command cannot mean two things.
//  This is the same shape.
//
//  ── THE HOST KEEPS ITS OWN CALLBACKS ────────────────────────────────────────
//  This module registers NOTHING with WCB_Client. It does not call onRawPacket(),
//  onCommand(), or touch the mesh channel. The host calls in:
//
//      WcbMgmt::begin(&wcb, &out, identity);   // once, from setup()
//      WcbMgmt::handleLine(line)               // from your "?" dispatcher
//      WcbMgmt::onRawPacket(data, len)         // from your raw-packet hook
//      WcbMgmt::service()                      // from loop()
//
//  That is not politeness, it is the only safe design. WCB_Client::onRawPacket()
//  takes ONE callback, and a host may already own it — NaviCore registers
//  naviota::otaRawPacketHook there for firmware OTA. Registering ours would have
//  silently replaced it and broken OTA, with nothing to see until an update
//  failed. Feeding this module from the host's existing hook composes instead.
//
//  ── THE CONCURRENCY RULE ────────────────────────────────────────────────────
//  onRawPacket() runs on the WiFi/ESP-NOW task. It ONLY copies into a queue and
//  returns. Every decode, reassembly and print happens in service(), on the loop
//  task, so there is exactly one writer to the output Print. A multi-packet write
//  from the receive callback races the loop task and corrupts any line longer
//  than one USB packet — measured on the relay before it was split this way.
// =============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "WCB_Client.h"

namespace WcbMgmt {

// ── Firmware packet types (WCB.ino PACKET_TYPE_* defines, WCB_RemoteTerm.h:42) ─
// Mirror these EXACTLY. They are the wire, not an implementation detail.
static const uint8_t PT_MGMT_FRAG   = 3;
static const uint8_t PT_CONFIG_REQ  = 5;
static const uint8_t PT_CONFIG_FRAG = 6;
static const uint8_t PT_STATS_REQ   = 7;
static const uint8_t PT_ETM_REQ     = 8;
static const uint8_t PT_STATS_FRAG  = 9;
static const uint8_t PT_ETM_FRAG    = 10;
// A config over 2912 characters (16 chunks x 182) does not fit one reply. A target
// asked with PT_CONFIG_REQ_PARTS sends it as several messages instead, each its own
// session of 230-byte PT_CONFIG_PART frags, "P<id>,<k>,<K>:<data>~" per part. A
// target that cannot answer a config request (either type) sends one PT_CONFIG_PART
// message "E<CODE>,<detail>" instead: NOPARTS to a plain request over the limit,
// NOMEM, CHANGED, TOOBIG.
// NEW TYPES, NEVER TYPE 6: every older relay prints a completed type-6 session as
// [MGMT:CONFIG,<n>], and every older Wizard (including the copy frozen inside each
// Intellex install) stores any non-empty body under that tag as the board's whole
// config, with no CRC check, so a part sent as type 6 would be saved as the config.
// An older relay drops a 230-byte frag of a type it does not know before it touches
// its session: every revision of this module and of MgmtRelay did
// (`if (!tag) return`), and so does the WCB firmware's 230-byte dispatch. So an old
// relay passes on neither parts nor errors, and an old Wizard behind it sees none.
static const uint8_t PT_CONFIG_PART      = 18;  // target → requester, 230 B
static const uint8_t PT_CONFIG_REQ_PARTS = 19;  // requester → target, 43 B: "I take parts"
// In the 204-byte struct. SIZE disambiguates it from PT_STATS_REQ (43 B).
static const uint8_t PT_REMOTE_TERM = 7;

static const size_t  CFG_PAYLOAD  = 183;   // firmware CONFIG_PAYLOAD_SIZE
static const uint8_t MGMT_CHUNKS  = 16;    // firmware MGMT_MAX_CHUNKS (uint16 receivedMask)

// 43 B — requester → target (config / stats / etm; only packetType differs).
typedef struct __attribute__((packed)) {
  char    structPassword[40];
  uint8_t packetType;
  uint8_t targetWCB;
  uint8_t requesterWCB;
} config_req;

// 230 B — target → requester reply fragment (config / stats / etm).
typedef struct __attribute__((packed)) {
  char     structPassword[40];
  uint8_t  packetType;
  uint8_t  sourceWCB;
  uint8_t  requesterWCB;
  uint16_t sessionId;
  uint8_t  chunkIdx;
  uint8_t  totalChunks;
  char     payload[CFG_PAYLOAD];
} config_frag;

// 204 B — target → requester remote-terminal line.
typedef struct __attribute__((packed)) {
  char    structPassword[40];
  uint8_t packetType;
  uint8_t sourceWCB;
  uint8_t destWCB;
  uint8_t textLen;
  char    text[160];
} remote_term;

// ── Who this device is, as the Wizard should see it ─────────────────────────
struct Identity {
  uint8_t     deviceId     = 0;      // our mesh id; replies come back addressed here
  const char* alias        = "";     // ?ALIAS and the WDP SELF row
  const char* fw           = "";     // ?version and the WDP SELF row
  const char* meshPassword = "";     // must match the fleet, or every packet is dropped
  uint8_t     hwVer        = 32;     // ?HW. 32 = "not a real WCB" (a relay or a NaviCore)
  uint8_t     macOct2      = 0;      // ?MAC,2 / ?MAC,3 — the Wizard shows them, nothing routes on them
  uint8_t     macOct3      = 0;
  uint8_t     wcbQuantity  = 1;      // ?WCBQ
  // Emit "?RELAY,1" in the backup, which makes the Wizard file this device as a
  // dedicated RELAY CARD instead of a configurable board in the numbered grid.
  // Correct for anything that is a doorway rather than a WCB.
  bool        advertiseRelay = true;
};

// ── Internals ───────────────────────────────────────────────────────────────
namespace detail {

// Raw inbound packets, copied on the WiFi task and processed from loop(). 232 B
// covers the larger of the two structs (config-frag 230 / remote-term 204).
struct RawPkt { uint8_t len; uint8_t data[232]; };

// ONE reassembly slot, shared by CONFIG, STATS and ETM replies and config parts.
// A session is keyed on (sessionId, packetType, sourceWCB): a frag that differs in
// any of them is another reply, never a continuation, so two boards whose random
// sessionIds collide cannot have their bodies spliced into one line. A new key
// normally wipes the slot; the one exception, which keeps a legacy reply ahead of
// a part stream, is in processConfigFrag(). Overlapping pulls still thrash the one
// slot, which is why the Wizard sends its pulls through a relay one at a time.
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

inline WCB_Client*  wcb   = nullptr;
inline Print*       out   = nullptr;
inline Identity     me;
inline QueueHandle_t rawQ = nullptr;
inline FragReasm    reasm;
inline uint16_t     lastDeliveredSession = 0;   // suppress the CONFIG second pass

inline bool ieq(const char* a, const char* b)     { return strcasecmp(a, b) == 0; }
inline bool iStarts(const char* s, const char* p) { return strncasecmp(s, p, strlen(p)) == 0; }

// Replace WDP-grammar-breaking characters in free text. A ',' or ']' or control
// char inside a [WDP:...] token corrupts the whole record for the parser.
inline void wdpScrub(char* s) {
  for (; *s; s++)
    if (*s == ',' || *s == ']' || (unsigned char)*s < 0x20) *s = '_';
}

}  // namespace detail

// ── ?backup / WCB_WEBTOOL_CONFIG_PULL ───────────────────────────────────────
// A minimal WCB-style backup. The Wizard only needs the "WCB Configuration
// Backup" header and an "End of Backup" line to accept the device; the ?TOKEN
// lines tell it which slot this is.
inline void printBackup() {
  Print& o = *detail::out;
  const Identity& id = detail::me;
  o.println();
  o.println("*** ========================================");
  o.println("*** WCB Configuration Backup");
  o.println("*** Copy and paste these commands to restore");
  o.println("*** ========================================");
  o.println();
  o.printf ("?HW,%u\n", (unsigned)id.hwVer);
  o.printf ("?MAC,2,%02X\n", id.macOct2);
  o.printf ("?MAC,3,%02X\n", id.macOct3);
  o.printf ("?WCB,%d\n", id.deviceId);
  if (id.advertiseRelay) o.println("?RELAY,1");
  o.printf ("?ALIAS,%s\n", id.alias);
  o.printf ("?WCBQ,%d\n", id.wcbQuantity);
  // The Wizard reads the mesh password from here. It is NOT a secret to the
  // Wizard — it needs it to talk to the fleet — but it does travel over whatever
  // transport this console is on, exactly as it does from a real WCB.
  o.printf ("?EPASS,%s\n", id.meshPassword);
  o.println("?CMDCHAR,;");
  o.println("--------- End of Backup ---------");
  o.println();
}

// ── ?version ────────────────────────────────────────────────────────────────
// The Wizard reads "Software Version: X", terminated by "End of Version".
inline void printVersion() {
  detail::out->printf("Software Version: %s\n", detail::me.fw);
  detail::out->println("End of Version");
}

// ── ?WDP,DUMP ───────────────────────────────────────────────────────────────
// The mesh inventory the Wizard's discovery panel and relay card are built from.
// Formatted byte-for-byte like the WCB firmware's printWdpDump(): a SELF row, one
// [WDP:...] per neighbour (+ [WDPIF:...] port labels), a [WDPCFG:...] summary and
// a terminating [WDP:END,count=N] which the Wizard uses as its sentinel.
inline void printWdpDump() {
  Print& o = *detail::out;
  const Identity& id = detail::me;
  WCB_Client* w = detail::wcb;
  const unsigned long now = millis();

  // SELF row. CLIENT=0 → shown as the tethered board; PEER=3 = "this board".
  o.printf("[WDP:N=%d,CLIENT=0,ALIAS=%s,HW=%u,HWREV=,FW=%s,CAP=0000,CTRL=0,"
           "CAPTAGS=,MAESTRO=-,AGE=0,SEEN=1,PEER=3]\n",
           id.deviceId, id.alias, (unsigned)id.hwVer, id.fw);

  int count = 0, peers = 0;
  for (uint8_t n = 1; n <= WCB_MAX_BOARDS; n++) {
    if (n <= id.wcbQuantity || w->isLearnedPeer(n)) peers++;
    const WCBNeighbor* nb = w->getNeighbor(n);
    if (!nb) continue;
    count++;
    char alias[25];   strncpy(alias,   nb->name,    sizeof(alias));   alias[sizeof(alias) - 1]     = '\0'; detail::wdpScrub(alias);
    char fw[28];      strncpy(fw,      nb->fw,      sizeof(fw));      fw[sizeof(fw) - 1]           = '\0'; detail::wdpScrub(fw);
    char hwrev[16];   strncpy(hwrev,   nb->hwRev,   sizeof(hwrev));   hwrev[sizeof(hwrev) - 1]     = '\0'; detail::wdpScrub(hwrev);
    char captags[49]; strncpy(captags, nb->capTags, sizeof(captags)); captags[sizeof(captags) - 1] = '\0'; detail::wdpScrub(captags);
    char maestro[40];
    if (nb->maestroCount == 0) { maestro[0] = '-'; maestro[1] = '\0'; }
    else {
      int off = 0; maestro[0] = '\0';
      for (int m = 0; m < nb->maestroCount && m < 9; m++)
        off += snprintf(maestro + off, sizeof(maestro) - off, "%s%d", m ? "." : "", nb->maestroIds[m]);
    }
    const int peerFlag = w->isLearnedPeer(n) ? 2 : (n <= id.wcbQuantity ? 1 : 0);
    o.printf("[WDP:N=%d,CLIENT=%d,ALIAS=%s,HW=%d,HWREV=%s,FW=%s,CAP=%04X,CTRL=%d,"
             "CAPTAGS=%s,MAESTRO=%s,AGE=%lu,SEEN=1,PEER=%d]\n",
             n, nb->isClient ? 1 : 0, alias, nb->hwVer, hwrev, fw,
             (unsigned)nb->capFlags, nb->ctrlId, captags, maestro,
             (now - nb->lastSeenMs) / 1000UL, peerFlag);
    for (int p = 0; p < 5; p++) {
      if (!nb->portLabels[p][0]) continue;
      char lbl[25]; strncpy(lbl, nb->portLabels[p], sizeof(lbl)); lbl[sizeof(lbl) - 1] = '\0';
      detail::wdpScrub(lbl);
      o.printf("[WDPIF:N=%d,S=%d,DEV=%s]\n", n, p + 1, lbl);
    }
  }
  o.printf("[WDPCFG:EN=1,AUTOJOIN=%d,PEERS=%d]\n", w->autoJoinEnabled() ? 1 : 0, peers);
  o.printf("[WDP:END,count=%d]\n", count);
}

// ── ?MGMT,PULL / STATS / ETM,CHAR ───────────────────────────────────────────
// PULL is sent x3 for loss resilience — a first frame to a board is often dropped
// on a busy mesh, and the target dedups by session.
inline void sendConfigReq(uint8_t target, uint8_t packetType, uint8_t times) {
  if (target < 1 || target > WCB_MAX_BOARDS) return;
  config_req req;
  memset(&req, 0, sizeof(req));
  strncpy(req.structPassword, detail::me.meshPassword, sizeof(req.structPassword) - 1);
  req.packetType   = packetType;
  req.targetWCB    = target;
  req.requesterWCB = detail::me.deviceId;      // replies come back addressed to us
  for (uint8_t i = 0; i < times; i++) {
    detail::wcb->sendRawPacket(target, (const uint8_t*)&req, sizeof(req));
    if (i + 1 < times) delay(15);
  }
}

namespace detail {

// True when `s` (what follows "<n>,") is the parts flag: a 'P', then nothing but
// whitespace. Whitespace is tolerated because not every host trims a line before
// handing it over (MgmtRelay's reader strips only CR/LF). Anything else is not the
// flag, and the pull goes out plain, just as every older relay sends it: they read
// "<n>,P" through atoi() as "<n>".
inline bool isPartsFlag(const char* s) {
  if (*s != 'P' && *s != 'p') return false;
  for (s++; *s; s++)
    if (*s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') return false;
  return true;
}

// ?MGMT,PULL,<n>    CONFIG_REQ (5) x3, 15 ms apart. Unchanged.
// ?MGMT,PULL,<n>,P  CONFIG_REQ_PARTS (19) x3, THEN CONFIG_REQ (5) x3, 15 ms apart.
//   The type-5 copies keep an OLDER target answering: it drops type 19 at its
//   receive filter and replies to the 5s as it always has. A new target accepts the
//   first request it hears from us and ignores the rest as duplicates, so the 19s
//   go first. Parts are only ever sent to a requester that asked with a 19, so a
//   plain pull never gets any.
//   This blocks the loop task ~75 ms (a plain pull ~30 ms). NaviCore services SBUS
//   on that task; one pull per Wizard request keeps it to one stall.
inline void sendPull(const char* args) {
  const uint8_t target = (uint8_t)atoi(args);
  const char*   comma  = strchr(args, ',');
  if (comma && isPartsFlag(comma + 1)) {
    if (target < 1 || target > WCB_MAX_BOARDS) return;
    sendConfigReq(target, PT_CONFIG_REQ_PARTS, 3);
    delay(15);
  }
  sendConfigReq(target, PT_CONFIG_REQ, 3);
}

}  // namespace detail

// ── ?MGMT,FRAG ──────────────────────────────────────────────────────────────
// Forward one Wizard fragment to the target, exactly as the WCB firmware's own
// relay does. `frag` points AFTER "?MGMT,FRAG,", i.e.
// "<targetWCB>,<sessionHex>,<idx>,<total>,<payload>". The payload may itself
// contain commas (?SEQ,SAVE,key,val), so only the first FOUR fields are split off
// and everything after the 4th comma is payload.
inline void handleMgmtFrag(char* frag) {
  char* c1 = strchr(frag,   ','); if (!c1) return;
  char* c2 = strchr(c1 + 1, ','); if (!c2) return;
  char* c3 = strchr(c2 + 1, ','); if (!c3) return;
  char* c4 = strchr(c3 + 1, ','); if (!c4) return;
  *c1 = *c2 = *c3 = *c4 = '\0';
  const uint8_t  target  = (uint8_t)atoi(frag);
  const uint16_t sid     = (uint16_t)strtoul(c1 + 1, nullptr, 16);
  const uint8_t  idx     = (uint8_t)atoi(c2 + 1);
  const uint8_t  total   = (uint8_t)atoi(c3 + 1);
  char*          payload = c4 + 1;
  if (target < 1 || target > WCB_MAX_BOARDS) return;

  if (total <= 1) {
    // Single chunk: send it as an ordinary text command so the target's normal
    // parser handles it. No reassembly, no session state on either side.
    detail::wcb->send(target, payload);
    detail::out->printf("[relay] MGMT -> WCB%d (1/1): %s\n", target, payload);
    return;
  }

  // Multi-chunk (a big config push): forward THIS chunk verbatim as a raw type-3
  // MGMT_FRAG so the target reassembles per session, exactly as it would from a
  // WCB relay. Going through send() would re-fragment it against a different size
  // cap and the target would reassemble garbage.
  // The frame is the firmware's own push struct (espnow_struct_mgmt, 226 B), as
  // examples/MgmtRelay sends it. A WCB routes a raw frame by its SIZE: this used to
  // be a config_frag, the 230 B REPLY layout, which never reached the target's
  // reassembly, so every push of more than one chunk through a WcbMgmt host was
  // lost (NaviCore D-NC48, HIL ncmesh.mgmt_frag_multichunk).
  wcb_packet_mgmt_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  strncpy(pkt.structPassword, detail::me.meshPassword, sizeof(pkt.structPassword) - 1);
  pkt.packetType  = WCB_MGMT_PACKET_TYPE_FRAG;   // 3 = wizard-origin (target re-broadcast allowed)
  pkt.targetWCB   = target;
  pkt.sessionId   = sid;
  pkt.chunkIdx    = idx;
  pkt.totalChunks = total;
  strncpy(pkt.payload, payload, sizeof(pkt.payload) - 1);
  detail::wcb->sendRawPacket(target, (const uint8_t*)&pkt, sizeof(pkt));
  detail::out->printf("[relay] MGMT -> WCB%d frag %u/%u (session %04X)\n",
                      target, idx + 1, total, sid);
}

// ── The command surface ─────────────────────────────────────────────────────
// Returns true when the line was OURS and has been handled. A host calls this
// from its own "?" dispatcher and falls through to its own commands on false, so
// adding this module cannot shadow anything the host already answers.
inline bool handleLine(const char* line) {
  if (!detail::wcb || !detail::out) return false;
  if (!strcmp(line, "WCB_WEBTOOL_CONFIG_PULL")) { printBackup(); return true; }
  if (line[0] != '?') return false;
  const char* c = line + 1;
  if (detail::ieq(c, "backup"))  { printBackup();  return true; }
  if (detail::ieq(c, "version")) { printVersion(); return true; }
  if (detail::iStarts(c, "WDP,")) {
    if (detail::ieq(c + 4, "DUMP")) { printWdpDump(); return true; }
    return false;                      // other ?WDP verbs are not ours
  }
  if (detail::iStarts(c, "MGMT,")) {
    // A copy, because handleMgmtFrag writes NULs into the buffer and the caller's
    // string may be const or reused.
    //
    // REFUSE AN OVERLONG LINE, DO NOT TRUNCATE IT. strncpy would silently cut a
    // FRAG's payload and we would forward the short version as a complete chunk —
    // the target reassembles it, writes it, and nothing notices until the config
    // is wrong. The longest legitimate line here is a FRAG carrying CFG_PAYLOAD
    // (183) plus its five header fields, so anything approaching this cap is
    // already malformed. Fail loudly, the way the relay's own reader drops an
    // oversized line rather than relaying its tail.
    char buf[400];
    const size_t n = strlen(c + 5);
    if (n >= sizeof(buf)) {
      detail::out->printf("[mgmt] line too long (%u B) - dropped\n", (unsigned)n);
      return true;
    }
    memcpy(buf, c + 5, n + 1);
    if      (detail::iStarts(buf, "FRAG,"))     handleMgmtFrag(buf + 5);
    else if (detail::iStarts(buf, "PULL,"))     detail::sendPull(buf + 5);
    else if (detail::iStarts(buf, "STATS,"))    sendConfigReq((uint8_t)atoi(buf + 6), PT_STATS_REQ,  1);
    else if (detail::iStarts(buf, "ETM,CHAR,")) sendConfigReq((uint8_t)atoi(buf + 9), PT_ETM_REQ,    1);
    return true;
  }
  return false;
}

// ── Raw packets ─────────────────────────────────────────────────────────────
// Call from the host's onRawPacket hook. Returns true when the packet was ours.
// RUNS ON THE WiFi TASK: copy and return, nothing else. See the concurrency note
// at the top of this file.
inline bool onRawPacket(const uint8_t* data, int len) {
  if (!detail::rawQ) return false;
  if (len != (int)sizeof(config_frag) && len != (int)sizeof(remote_term)) return false;
  detail::RawPkt r;
  r.len = (uint8_t)len;
  memcpy(r.data, data, len);
  // Non-blocking. A dropped fragment is covered by the sender's second pass, and
  // blocking the ESP-NOW receive task is far worse than losing one.
  xQueueSend(detail::rawQ, &r, 0);
  return true;
}

namespace detail {

// How long a live CONFIG/STATS/ETM session holds the slot against config parts.
// Its frags arrive 20 ms apart, so 200 ms without one means it has stalled.
static const unsigned long PART_YIELD_MS = 200;

// Reassemble one reply and print it as ONE line once the session is complete:
//   type 6  (CONFIG)              -> [MGMT:CONFIG,<src>]<body>
//   type 9 / 10  (STATS / ETM)    -> [MGMT:STATS,<src>]<body> / [MGMT:ETM,<src>]<body>
//   type 18, body "P..." (a part) -> [MGMT:CFGPART,<src>]<body>
//   type 18, body "E..." (error)  -> [MGMT:CFGERR,<src>]<body without the E>
//   type 18, anything else        -> dropped
// The relay does not join parts: the requester collects the K parts of one id and
// checks the CRC of the joined text. Mirrors WCB.ino handleConfigFragPacket.
inline void processConfigFrag(const uint8_t* data) {
  config_frag pkt;
  memcpy(&pkt, data, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (strcmp(pkt.structPassword, me.meshPassword) != 0) return;
  if (pkt.requesterWCB != me.deviceId) return;
  if (pkt.totalChunks == 0 || pkt.totalChunks > MGMT_CHUNKS) return;
  if (pkt.chunkIdx >= MGMT_CHUNKS || pkt.chunkIdx >= pkt.totalChunks) return;
  if (pkt.packetType != PT_CONFIG_FRAG && pkt.packetType != PT_STATS_FRAG &&
      pkt.packetType != PT_ETM_FRAG    && pkt.packetType != PT_CONFIG_PART) return;
  // The 2nd pass. One value covers every reply type: a target that sends parts
  // never gives two consecutive sessions (each part, each error) the same id.
  if (lastDeliveredSession != 0 && pkt.sessionId == lastDeliveredSession) return;

  const bool sameSession = reasm.active && reasm.sessionId == pkt.sessionId &&
                           reasm.packetType == pkt.packetType &&
                           reasm.sourceWCB  == pkt.sourceWCB;
  if (!sameSession) {
    // A LEGACY REPLY KEEPS THE SLOT. A board over 2912 characters streams parts for
    // hundreds of ms, and every part frag would wipe a CONFIG/STATS/ETM reply from
    // ANOTHER board that is still arriving. That reply may be for an old Wizard,
    // which never asked for parts: it fires pulls on every ETM ONLINE edge with
    // nothing coordinating them, and its fixed retry timers would meet the stream
    // again on every attempt. So a part frag yields to a live non-part session and
    // is dropped; the part's second pass or the new Wizard's retry recovers it. A
    // non-part frag always takes the slot, exactly as it did before parts existed.
    if (pkt.packetType == PT_CONFIG_PART && reasm.active &&
        reasm.packetType != PT_CONFIG_PART &&
        (millis() - reasm.lastMs) < PART_YIELD_MS) return;
    memset(&reasm, 0, sizeof(reasm));
    reasm.active      = true;
    reasm.packetType  = pkt.packetType;
    reasm.sourceWCB   = pkt.sourceWCB;
    reasm.sessionId   = pkt.sessionId;
    reasm.totalChunks = pkt.totalChunks;
  }
  if (!(reasm.receivedMask & (1 << pkt.chunkIdx))) {
    strncpy(reasm.chunks[pkt.chunkIdx], pkt.payload, CFG_PAYLOAD);
    reasm.chunks[pkt.chunkIdx][CFG_PAYLOAD] = '\0';
    reasm.receivedMask |= (1 << pkt.chunkIdx);
  }
  reasm.lastMs = millis();

  const uint16_t expected = (uint16_t)((1 << pkt.totalChunks) - 1);
  if (reasm.receivedMask != expected) return;

  // NO String. `full += chunk` skips a chunk SILENTLY when its allocation fails
  // (String::concat returns false, and operator+= ignores it: esp32 core 3.3.4
  // WString.cpp:330-331, WString.h:143-146). A heap-starved relay would then print
  // a short but well-formed [MGMT:CONFIG] line, and a Wizard stores that as the
  // board's config. Instead the chunks are compacted in place into one C string:
  // each row holds at most CFG_PAYLOAD characters in CFG_PAYLOAD + 1 bytes, so the
  // text moved down so far never reaches the next row before that row is read.
  char*  body = reinterpret_cast<char*>(reasm.chunks);
  size_t len  = 0;
  for (int i = 0; i < pkt.totalChunks; i++) {
    const size_t n = strlen(reasm.chunks[i]);
    memmove(body + len, reasm.chunks[i], n);
    len += n;
  }
  body[len] = '\0';

  const char* tag  = nullptr;
  const char* text = body;
  if      (reasm.packetType == PT_CONFIG_FRAG) tag = "CONFIG";
  else if (reasm.packetType == PT_STATS_FRAG)  tag = "STATS";
  else if (reasm.packetType == PT_ETM_FRAG)    tag = "ETM";
  else if (body[0] == 'P')                     tag = "CFGPART";
  else if (body[0] == 'E')                   { tag = "CFGERR"; text = body + 1; }

  lastDeliveredSession = reasm.sessionId;
  // ONE printf, tag through newline. Print::printf formats into one buffer and makes
  // ONE write() (esp32 core 3.3.4 Print.cpp:46-71), so a print from another task can
  // land only between two lines, never inside one or ahead of its newline. That
  // matters for parts: the Wizard drops a part line with anything after its closing
  // '~'. If printf cannot allocate that buffer it prints nothing at all, and a
  // missing line costs the requester a retry, never a wrong config.
  if (tag) out->printf("[MGMT:%s,%d]%s\n", tag, (int)reasm.sourceWCB, text);
  memset(&reasm, 0, sizeof(reasm));
}

// A remote-terminal line → [TERM:<src>]<line>, which the Wizard demuxes into
// per-board panes. Mirrors WCB_RemoteTerm.cpp:161-176.
inline void processRemoteTerm(const uint8_t* data) {
  remote_term pkt;
  memcpy(&pkt, data, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (strcmp(pkt.structPassword, me.meshPassword) != 0) return;
  if (pkt.destWCB != me.deviceId) return;
  if (pkt.textLen == 0 || pkt.textLen > sizeof(pkt.text)) return;   // firmware drops these
  uint8_t len = pkt.textLen;
  char line[sizeof(pkt.text) + 1];
  memcpy(line, pkt.text, len);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) len--;
  if (len == 0) return;                                             // firmware prints nothing
  line[len] = '\0';
  out->printf("[TERM:%d]%s\n", pkt.sourceWCB, line);
}

}  // namespace detail

// ── Call from loop() ────────────────────────────────────────────────────────
// Every decode and print happens here, on the loop task, so there is one writer.
inline void service() {
  if (!detail::rawQ) return;
  detail::RawPkt rp;
  while (xQueueReceive(detail::rawQ, &rp, 0) == pdTRUE) {
    if      (rp.len == (uint8_t)sizeof(config_frag)) detail::processConfigFrag(rp.data);
    else if (rp.len == (uint8_t)sizeof(remote_term)) detail::processRemoteTerm(rp.data);
  }
}

// ── Setup ───────────────────────────────────────────────────────────────────
// Create the queue BEFORE the host's raw-packet hook can fire, or the first
// fragment is lost to a create/publish race.
inline bool begin(WCB_Client* client, Print* output, const Identity& identity) {
  detail::wcb = client;
  detail::out = output;
  detail::me  = identity;
  if (!detail::rawQ) detail::rawQ = xQueueCreate(24, sizeof(detail::RawPkt));
  return detail::rawQ != nullptr;
}

}  // namespace WcbMgmt
