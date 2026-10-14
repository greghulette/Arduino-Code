/*
  MgmtRelay.ino — turn any ESP32 into a WCB management relay
                  (USB serial and/or WiFi ↔ ESP-NOW)

  Flash this to ANY ESP32, plug it into USB, and it becomes a transparent doorway
  into your WCB network. Whatever you (or a host tool like the NaviCore Config
  Tool in "Via WCB" mode) send on USB serial is relayed onto the mesh, and every
  reply from the mesh is printed straight back to USB. That is ALL it does — it
  hosts no servos, audio, LEDs, or PWM; its sole purpose is to be the relay.

  Optionally (RELAY_WIFI_MODE, OFF by default) it carries that exact same line
  protocol over a WebSocket at ws://<ip>/ws — from its own SoftAP, or by joining an
  existing one such as NaviCore's — so a phone or desktop app reaches the whole mesh
  with no USB cable. USB keeps working at the same time: the two are peers, and USB
  stays the fallback when WiFi is misbehaving.

  ── WHY THIS BOARD CARRIES THE WiFi AND THE WCBs DO NOT ───────────────────────
  Putting WiFi on every WCB is the obvious idea and the wrong one. The ESP32 has ONE
  radio: an associated STA follows its AP's channel, so an AP you do not control can
  move a board off the mesh channel and split the network silently, and a STA that
  cannot find its AP rescans across every channel, going deaf to the mesh on each
  sweep. On a WCB that costs real damage too — S3/S4/S5 are bit-banged software
  UARTs whose TX timing is already sensitive to radio interrupt load. Concentrating
  WiFi on ONE board that hosts nothing puts all of that where it costs nothing, and
  gives a host app one address instead of N.

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

  ── Changing the WiFi mode without a reflash ──────────────────────────────────
    ?RELAY,WIFI                     report the mode, SSID and address
    ?RELAY,WIFI,OFF                 ESP-NOW only
    ?RELAY,WIFI,AP[,<pass>]         host our own AP (optionally set its password)
    ?RELAY,WIFI,JOIN,<ssid>,<pass>  join an existing AP — NaviCore's, typically
    ?RELAY,WIFI,DEFAULTS            forget NVS, return to the compiled-in values

  The mode and credentials live in NVS, so this survives a reboot and the sketch
  never has to be edited to move a relay between a bench AP and a droid. Each of
  these saves and then REBOOTS (after the console goes quiet) — see the note above
  RELAY_WIFI_MODE for why the change cannot safely be applied to a live radio.
  Passwords can be set but are never printed back: this console is mirrored to
  every connected WebSocket client.

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
    5. OPTIONAL, for WiFi access: set RELAY_WIFI_MODE and fill in the credentials
       in the block below. MESH_CHANNEL must match every WCB (?WCBCH) — one radio
       means the AP and the mesh share a channel, and a mismatch is a silent
       blackout, not an error. The AP password is mandatory (>= 8 chars): this
       WebSocket relays ";w<id>,<cmd>" to every board with no credential of its
       own, so an open AP would hand the droid to anyone in range.

  ── WebSocket protocol (RELAY_WIFI_MODE != OFF) ───────────────────────────────
    ADDRESS: with RELAY_STATIC_IP (on by default) this relay is always at
             192.168.4.<DEVICE_ID> — 192.168.4.19 out of the box — whether it is
             hosting the AP itself or has joined NaviCore's. The app therefore needs
             no discovery and does not care which of the two happened. Needs
             DEVICE_ID >= 13; lower ids fall inside the AP's DHCP pool and the build
             refuses them rather than letting it fail in the field.

    ws://192.168.4.19/ws
                   Newline-delimited UTF-8. Lines are EXACTLY what you would type
                   on the USB port above (";w3,:PP100", or bare text to broadcast),
                   because both mouths feed the same relaySerialLine(). A line may
                   span several frames; frames are reassembled per socket.
    Replies are a CONSOLE MIRROR, not strict request/response: everything the relay
    would print to USB — command replies, board online/offline, mesh telemetry — is
    streamed to every connected client.

    This is deliberately the same transport NaviCore exposes (its docs/PROTOCOLS.md;
    same URI, same framing, same mirror semantics), so a host app writes ONE client
    and points it at either box. Only the payload grammar differs: NaviCore speaks
    JSON verbs, this speaks the WCB serial grammar.
*/

#include <WCB_Client.h>
#include <mbedtls/base64.h>   // Phase 3: decode the Wizard's base64 OTA_DATA fragments

// ── Network credentials — must match your WCB system exactly (see BasicUsage) ──
//   MAC_OCT2 / MAC_OCT3 : shared MAC octets that identify your network (?WCBM)
//   PASSWORD            : the ESP-NOW network password (?WCBP)
//   WCB_QUANTITY        : total number of WCBs in the system (?WCBQ)
//   DEVICE_ID           : a unique id for THIS relay — see setup note #2
const uint8_t MAC_OCT2     = 0x05;
const uint8_t MAC_OCT3     = 0x4B;
const char*   PASSWORD     = "khEdzNZNh9rMFP";
const uint8_t WCB_QUANTITY = 1;
const uint8_t DEVICE_ID    = 19;

// The NaviCore / out-of-band controller lives at the special-peer id (default 20).
const uint8_t NAVICORE_ID  = WCB_SPECIAL_ID;   // = 20

// This relay's firmware string, advertised over WDP so it shows up by name.
const char*   RELAY_FW     = "1.2";

// Name this relay advertises (WDP) and reports to the WCB Wizard as its alias.
const char*   RELAY_ALIAS  = "Mgmt Relay";

// 1 = also echo each relayed line + board online/offline to USB (handy from a
// plain Serial Monitor). Leave 0 when a tool is driving the port so the only
// non-JSON noise is startup text.
#define VERBOSE 0

// ── Remote access over WiFi (optional) ─────────────────────────────────────────
// Gives any host app a WebSocket into the mesh at ws://<ip>/ws carrying
// the SAME newline-delimited line protocol this sketch's USB port already speaks —
// deliberately the same transport NaviCore exposes (docs/PROTOCOLS.md), so one
// client library talks to either box. USB keeps working at the same time: the two
// are peers, not alternatives, and USB stays the fallback when WiFi misbehaves.
//
//   RELAY_WIFI_OFF   ESP-NOW only. The original behaviour, and the default.
//   RELAY_WIFI_AP    Host our own SoftAP. Self-contained — no infrastructure
//                    needed, and nothing external can move us off the mesh channel.
//   RELAY_WIFI_JOIN  Join an existing AP (e.g. NaviCore's) so ONE WiFi network
//                    reaches both boxes. Falls back to RELAY_WIFI_AP if that AP is
//                    absent or is not on the mesh channel.
#define RELAY_WIFI_OFF   0
#define RELAY_WIFI_AP    1
#define RELAY_WIFI_JOIN  2

// ── The mode is a RUNTIME setting; this is only its factory default ────────────
// It used to be compile-time, which meant that moving between hosting an AP and
// joining NaviCore's took an edit, a rebuild and a reflash — with the board
// usually already installed in a droid. Now it lives in NVS and is set from the
// console (USB or the WebSocket, same command surface):
//
//   ?RELAY,WIFI                       report the current mode (never a password)
//   ?RELAY,WIFI,OFF|AP|JOIN           switch mode, then reboot to apply
//   ?RELAY,WIFI,AP,<pass>             switch to our own AP, setting its password
//   ?RELAY,WIFI,JOIN,<ssid>,<pass>    switch to joining <ssid>
//   ?RELAY,WIFI,DEFAULTS              forget NVS and go back to the values below
//
// WHY A REBOOT RATHER THAN RECONFIGURING LIVE. The ordering rules above
// relayStartSoftAP() are not advice: a SoftAP MUST be up BEFORE wcb.begin() and a
// STA MUST be joined AFTER it, because that begin() inspects WiFi.getMode() and
// calls WiFi.disconnect() unconditionally. Re-running the dance on a live radio
// means tearing ESP-NOW down underneath a mesh that is mid-conversation. A reboot
// re-runs setup() in the one order known to work, costs ~2 s, and cannot leave the
// radio in a state no code path was written for.
//
// The credentials are NVS-backed for the same reason: they were compile-time
// constants, so changing a password was also a rebuild — and an empty AP_PASS makes
// the relay refuse to start WiFi at all (deliberately; see AP_PASS below), which is
// a bad thing to discover after installing the board.
#define RELAY_WIFI_MODE  RELAY_WIFI_OFF

// Compile the WiFi code in at all. Separate from the mode, because the mode is now
// chosen at runtime: every path has to be PRESENT in the image for a console
// command to be able to select it. Set to 0 only to reclaim flash on a build that
// will never use WiFi — ?RELAY,WIFI then says it was compiled out rather than
// silently doing nothing.
#define RELAY_WIFI_BUILD 1

// The ESP-NOW mesh channel. MUST match every WCB (?WCBCH, default 1) and every
// other client. The ESP32 has ONE radio, so this is also the channel any AP here
// runs on and the ONLY channel we will accept an association on. That is not a
// preference: a radio on another channel is a total, silent mesh blackout — every
// packet dropped, nothing logged on the boards, no fault indication anywhere.
const uint8_t MESH_CHANNEL = 1;

// Our own SoftAP — used by RELAY_WIFI_AP, and as the RELAY_WIFI_JOIN fallback.
//
// THE PASSWORD IS MANDATORY (>= 8 chars) AND WE FAIL CLOSED. This WebSocket is an
// unauthenticated command channel to every board on the mesh — ";w1,?RESTART" is a
// bare string with no credential — so an open AP hands the whole droid to anyone in
// range. WiFi.softAP() with an empty passphrase cheerfully creates an OPEN network,
// so refusing has to be explicit. Same reasoning, and the same refusal, as NaviCore.
// These four are FACTORY DEFAULTS. Whatever ?RELAY,WIFI last stored in NVS wins;
// ?RELAY,WIFI,DEFAULTS forgets NVS and comes back to exactly these.
const char* AP_SSID = "";                 // "" = derive "MgmtRelay-<DEVICE_ID>"
const char* AP_PASS = "";                 // REQUIRED — AP will not start without it

// The AP to join in RELAY_WIFI_JOIN mode (NaviCore's SoftAP, typically).
const char* JOIN_SSID = "";
const char* JOIN_PASS = "";
// Retry cadence, and how long to keep trying before giving up and hosting our own AP.
// The give-up window is generous ON PURPOSE: powered from one switch, this relay is
// ready long before NaviCore raises its SoftAP (LittleFS mount, config load, ~3.1 MB
// PSRAM alloc all come first), so a short window would lose that race on every cold
// boot and strand the app on a fallback AP. Retries cost nothing — see the note above
// relayJoinService() on why a channel-pinned attempt does not sweep the band.
const uint32_t JOIN_RETRY_MS  =  5000;    // between attempts
const uint32_t JOIN_GIVEUP_MS = 90000;    // since boot, then host our own AP

// ── Fixed address ──────────────────────────────────────────────────────────────
// 1 = take <SUBNET>.DEVICE_ID (e.g. 192.168.4.19) instead of the .1 SoftAP default /
// a DHCP lease. The point is that the app then knows where this relay lives WITHOUT
// discovery, and the address is the SAME whether we host the AP or joined NaviCore's.
//
// Why this cannot collide with the AP's DHCP pool: the ESP32 core leases exactly
// eleven addresses starting one above the AP's own IP — start = AP+1, end = start+10
// (NetworkInterface.cpp:451-453). NaviCore's AP is the default 192.168.4.1, so its
// pool is 192.168.4.2 - 192.168.4.12. DEVICE_ID 19 sits clear of it.
//
// SO KEEP DEVICE_ID >= 13 IF YOU ENABLE THIS. Ids 2-12 land inside that pool and the
// DHCP server, which knows nothing of our static claim, will eventually hand the same
// address to a phone. Id 1 is the AP/gateway itself. The default id of 19 is fine;
// the check below refuses the unsafe ones rather than letting them fail in the field.
#define RELAY_STATIC_IP  1

// The /24 the AP lives on. 192.168.4.x is the ESP32 SoftAP default, which is what
// NaviCore uses (it never calls softAPConfig).
const uint8_t SUBNET_A = 192, SUBNET_B = 168, SUBNET_C = 4;

#if RELAY_STATIC_IP && RELAY_WIFI_MODE != RELAY_WIFI_OFF
  static_assert(DEVICE_ID >= 13 && DEVICE_ID <= 254,
                "RELAY_STATIC_IP needs DEVICE_ID >= 13: ids 2-12 fall inside the AP's "
                "DHCP pool (AP+1 .. AP+11) and id 1 is the AP itself. Raise DEVICE_ID "
                "or set RELAY_STATIC_IP to 0.");
#endif

#if RELAY_WIFI_BUILD
  #include "mgmt_wsserver.h"
#endif

// ── Host-facing output ─────────────────────────────────────────────────────────
// Everything the relay says to its host goes through here: USB serial ALWAYS, plus
// every connected WebSocket client. The handlers stay transport-unaware and keep
// printing exactly as they always have, so the two mouths cannot drift.
// Serial.begin/available/read stay direct — this is an OUTPUT tee only.
class HostOut : public Print {
 public:
  size_t write(uint8_t c) override {
    Serial.write(c);
#if RELAY_WIFI_BUILD
    mgmtws::sink.write(c);
#endif
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
#if RELAY_WIFI_BUILD
    mgmtws::sink.write(b, n);
#endif
    return n;
  }
  using Print::write;
};
HostOut host;

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// ── Deferred reboot ────────────────────────────────────────────────────────────
// NEVER ESP.restart() FROM A COMMAND HANDLER. A host tool sends commands as a
// STREAM, so restarting inside one destroys every command still queued behind it —
// and the sender cannot tell, because the boot banner satisfies its "did the board
// answer" test, so nothing retries and the batch is scored as fully delivered.
// That is rule 11 in the WCB firmware, learned there the hard way; this relay feeds
// the same tools through the same queues and has exactly the same exposure.
//
// So set a deadline, let loop() keep draining, and restart only after the console
// has been QUIET for a moment. Quiet matters on its own: an ACK-paced push empties
// the queue between every pair of commands, so "the queue is empty" is not the same
// as "the sender has finished talking".
static uint32_t relayRebootAtMs = 0;             // 0 = nothing pending
static const uint32_t REBOOT_QUIET_MS = 1200;

static void relayRequestReboot(const char* why) {
  host.printf("[relay] %s - rebooting to apply.\n", why);
  relayRebootAtMs = millis() + REBOOT_QUIET_MS;
}

// Called whenever a line arrives, to push the reboot out past the rest of a burst.
static void relayRebootDefer() {
  if (relayRebootAtMs) relayRebootAtMs = millis() + REBOOT_QUIET_MS;
}

static void relayRebootService() {
  if (relayRebootAtMs && (int32_t)(millis() - relayRebootAtMs) >= 0) {
    Serial.flush();          // the WebSocket sink cannot survive the restart; USB can
    ESP.restart();
  }
}


#if RELAY_WIFI_BUILD
#include <esp_wifi.h>   // esp_wifi_get_channel() — verify where an association left us
#include <Preferences.h>

// ── Runtime WiFi settings (NVS) ────────────────────────────────────────────────
// Read ONCE at boot into these, so every later reader sees one consistent snapshot
// and nothing has to touch NVS on a hot path. ?RELAY,WIFI writes NVS and asks for a
// reboot; it deliberately does NOT mutate the live values, because half-applied
// settings are exactly the state the reboot exists to avoid.
//
// PASSWORDS ARE WRITE-ONLY FROM THE CONSOLE. They can be set and cleared but never
// printed back — not in ?RELAY,WIFI, not in the backup, not in a log line. The
// mesh password already gets that treatment (?EPASS is emitted in the backup but
// a host app's discovery drops the line), and a WiFi password reaching a
// console that is itself mirrored to every WebSocket client would be worse: it
// would travel over the very network it protects.
static Preferences relayPrefs;
static const char* RELAY_NVS_NS = "mgmtrelay";

static uint8_t relayWifiMode = RELAY_WIFI_MODE;
static String  relayApPass   = AP_PASS;
static String  relayJoinSsid = JOIN_SSID;
static String  relayJoinPass = JOIN_PASS;

static const char* relayModeName(uint8_t m) {
  return m == RELAY_WIFI_AP ? "AP" : m == RELAY_WIFI_JOIN ? "JOIN" : "OFF";
}

static void relayLoadWifiSettings() {
  if (!relayPrefs.begin(RELAY_NVS_NS, /*readOnly=*/true)) return;   // never written yet
  relayWifiMode = relayPrefs.getUChar("wifiMode", relayWifiMode);
  relayApPass   = relayPrefs.getString("apPass",   relayApPass);
  relayJoinSsid = relayPrefs.getString("joinSsid", relayJoinSsid);
  relayJoinPass = relayPrefs.getString("joinPass", relayJoinPass);
  relayPrefs.end();
  // Anything outside the three known modes is a corrupt or downgraded NVS entry.
  // Fall back rather than run with a mode no branch handles.
  if (relayWifiMode > RELAY_WIFI_JOIN) relayWifiMode = RELAY_WIFI_MODE;
}

// ── WiFi bring-up ──────────────────────────────────────────────────────────────
// ORDERING IS LOAD-BEARING, AND THE TWO MODES PULL IN OPPOSITE DIRECTIONS:
//
//   SoftAP  MUST come up BEFORE wcb.begin(). WCB_Client::begin() reads
//           WiFi.getMode() on entry and, finding an AP already up, selects
//           WIFI_AP_STA and KEEPS it rather than forcing WIFI_STA and tearing the
//           AP down (WCB_Client.cpp:95-112). Raise it afterwards and ESP-NOW has
//           already pinned the radio.
//   STA     MUST come up AFTER wcb.begin(). That same begin() calls
//           WiFi.disconnect() unconditionally (WCB_Client.cpp:103), which drops any
//           association already made. Join first and it is silently undone.
//
// Either way the radio ends on MESH_CHANNEL or the interface does not come up.
static bool wifiUp = false;

static void relayStartSoftAP() {
  const size_t pwLen = relayApPass.length();
  if (pwLen < 8) {
    host.printf("[relay] WIFI REFUSED: AP password is %u character(s); WPA2 needs 8.\n",
                (unsigned)pwLen);
    host.println("[relay] Refusing to start an OPEN access point - this WebSocket relays");
    host.println("[relay] ;w<id>,<cmd> to every board with no credential of its own.");
    return;
  }
  char ssid[33];
  if (AP_SSID[0]) strlcpy(ssid, AP_SSID, sizeof(ssid));
  else            snprintf(ssid, sizeof(ssid), "MgmtRelay-%u", (unsigned)DEVICE_ID);
#if RELAY_STATIC_IP
  // Host the AP at <SUBNET>.DEVICE_ID rather than the default .1, so this relay has
  // the SAME address whether it hosts the AP or joins NaviCore's. See the note on
  // RELAY_STATIC_IP for why the DHCP pool cannot collide with it.
  const IPAddress apIp(SUBNET_A, SUBNET_B, SUBNET_C, DEVICE_ID);
  if (!WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0)))
    host.printf("[relay] softAPConfig(%s) FAILED - falling back to the default address.\n",
                apIp.toString().c_str());
#endif
  // Channel is softAP()'s 3rd parameter and DEFAULTS TO 1 — pass MESH_CHANNEL
  // explicitly. Once an AP owns the radio, WCB_Client stops calling
  // esp_wifi_set_channel and only WARNS on a mismatch (WCB_Client.cpp:121-130), so a
  // defaulted channel against a mesh on any other channel is a silent, total
  // blackout: every packet dropped, one log line nobody reads.
  if (WiFi.softAP(ssid, relayApPass.c_str(), MESH_CHANNEL, /*hidden=*/0, /*max_conn=*/4)) {
    host.printf("[relay] SoftAP \"%s\" up on channel %u - ws://%s/ws\n",
                ssid, MESH_CHANNEL, WiFi.softAPIP().toString().c_str());
    wifiUp = true;
  } else {
    host.printf("[relay] SoftAP \"%s\" FAILED to start on channel %u.\n", ssid, MESH_CHANNEL);
  }
}

// ── Joining an existing AP ─────────────────────────────────────────────────────
// ALWAYS COMPILED, run only when relayWifiMode == RELAY_WIFI_JOIN. It used to be
// #if'd out of an AP-mode build, which is exactly why changing mode needed a
// reflash — the other mode's code was not in the image to switch to.
// NON-BLOCKING, RETRIED FROM loop(), AND THAT IS THE WHOLE POINT.
//
// On a shared power switch this relay and NaviCore boot together, and the relay wins
// that race every time: its setup() is a few queues and wcb.begin(), while NaviCore
// raises its SoftAP only at the END of its own setup — after mounting LittleFS,
// loading config, and allocating a ~3.1 MB PSRAM record buffer. A one-shot wait in
// setup() therefore expires against an AP that had simply not appeared yet, and the
// fallback then leaves TWO APs up with the app on the wrong one. That is worse than
// no AP at all, and it would be the COMMON case, not the edge case. So we keep
// retrying for JOIN_GIVEUP_MS with the mesh fully alive throughout, and only host our
// own AP if the other one genuinely never shows.
//
// Two things make an association dangerous to a mesh device. Both are handled here
// rather than hoped away:
//
//   SCANNING  A WiFi.begin() with no channel probes EVERY channel looking for the
//             SSID, and for the length of each sweep this board is off the mesh.
//             Passing MESH_CHANNEL confines the probe to the one channel we are
//             already on, which is what makes retrying cheap enough to do at all;
//             auto-reconnect stays OFF so a vanished AP cannot start its own sweeps.
//   CHANNEL   The AP owns the radio channel once associated. NaviCore raises its
//             SoftAP on its own mesh channel (NaviCore.ino:4647), so joining it is a
//             no-op channel-wise — but if we ever land somewhere else we hang up
//             rather than sit silently off-mesh.
static uint32_t joinNextAttemptMs = 0;
static uint8_t  joinAttempts      = 0;
static bool     joinSettled       = false;   // joined, or gave up and raised our own AP

static void relayJoinAttempt() {
  WiFi.setAutoReconnect(false);
#if RELAY_STATIC_IP
  // Ask for <SUBNET>.DEVICE_ID instead of taking whatever DHCP offers, so the app
  // always knows where to find us. Safe against the AP's DHCP pool — see the note on
  // RELAY_STATIC_IP. Gateway/DNS are the AP itself at .1.
  const IPAddress me(SUBNET_A, SUBNET_B, SUBNET_C, DEVICE_ID);
  const IPAddress gw(SUBNET_A, SUBNET_B, SUBNET_C, 1);
  if (!WiFi.config(me, gw, IPAddress(255, 255, 255, 0), gw))
    host.println("[relay] static IP config FAILED - falling back to DHCP.");
#endif
  WiFi.begin(relayJoinSsid.c_str(), relayJoinPass.c_str(), MESH_CHANNEL);
  joinAttempts++;
}

// Serviced every loop() pass until it settles. Never blocks.
static void relayJoinService() {
  if (joinSettled) return;
  const uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    // Verify where the association actually left the radio. WCB_Client::update() also
    // notices a drift and warns every 30 s (WCB_Client.cpp:215, :1549) — but it only
    // WARNS, and a relay that cannot reach the mesh is useless. Act on it.
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &second);
    if (primary != MESH_CHANNEL) {
      host.printf("[relay] joined \"%s\" but the radio is on channel %u, not %u - "
                  "that is off-mesh. Disconnecting.\n", relayJoinSsid.c_str(), primary, MESH_CHANNEL);
      WiFi.disconnect(true);
      joinSettled = true;
      relayStartSoftAP();
      return;
    }
    host.printf("[relay] joined \"%s\" on channel %u after %u attempt(s) - ws://%s/ws\n",
                relayJoinSsid.c_str(), primary, joinAttempts, WiFi.localIP().toString().c_str());
    wifiUp      = true;
    joinSettled = true;
    return;
  }

  if (now < joinNextAttemptMs) return;

  if (now >= JOIN_GIVEUP_MS) {
    host.printf("[relay] \"%s\" did not appear within %lu s (%u attempts) - "
                "hosting our own SoftAP instead.\n",
                relayJoinSsid.c_str(), (unsigned long)(JOIN_GIVEUP_MS / 1000), joinAttempts);
    WiFi.disconnect(true);
    joinSettled = true;
    relayStartSoftAP();
    return;
  }

  relayJoinAttempt();
  joinNextAttemptMs = now + JOIN_RETRY_MS;
}

// Bring the WebSocket up the moment an interface exists, whichever way it arrived.
// Started from loop(), not setup(), because in JOIN mode the interface may not be up
// until well after setup() has returned.
static bool wsStarted = false;

static void relayWifiService() {
  if (relayWifiMode == RELAY_WIFI_JOIN) relayJoinService();
  if (wifiUp && !wsStarted) {
    wsStarted = mgmtws::begin();
    if (wsStarted) host.println("[relay] WebSocket up at /ws - same line protocol as this port.");
  }
}
#endif  // RELAY_WIFI_BUILD

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
// a config over 2912 characters comes back as type-18 messages, printed as [MGMT:CFGPART,<src>]
// (one line per part, only ever to a "?MGMT,PULL,<n>,P"), and a target that cannot answer a pull
// sends one printed as [MGMT:CFGERR,<src>] (see processConfigFrag);
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
// A config over 2912 characters (16 chunks x 182) does not fit one reply. A target asked with
// PT_CONFIG_REQ_PARTS sends it as several messages, each its own session of 230-byte
// PT_CONFIG_PART frags, "P<id>,<k>,<K>:<data>~" per part. A target that cannot answer a config
// request (either type) sends one PT_CONFIG_PART message "E<CODE>,<detail>" instead: NOPARTS to a
// plain request over the limit, NOMEM, CHANGED, TOOBIG. NEW TYPES, NEVER TYPE 6: an older relay
// prints a completed type-6 session as [MGMT:CONFIG,<n>], and an older Wizard stores any non-empty
// body under that tag as the board's whole config, with no CRC check. An older relay (every
// revision of this sketch did) drops a 230-byte frag of a type it does not know before touching
// its session, so it never passes one on.
#define PT_CONFIG_PART      18   // target → relay, 230 B
#define PT_CONFIG_REQ_PARTS 19   // relay → target, 43 B: "this requester takes parts"
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

// ── Phase 3: wireless OTA relay (?OTA,* ↔ [OTA:ACK,…]) ─────────────────────────
// The WCB Wizard drives a firmware update THROUGH this relay: it sends, on USB serial,
//   ?OTA,BEGIN,<target>,<session>,<imageSize>,<family>
//   ?OTA,DATA,<target>,<session>,<offset>,<base64(<=192 B)>
//   ?OTA,END,<target>,<session>        (?OTA,ABORT,<target>,<session> to tear down)
// We unicast the matching ESP-NOW packet to the target; the target unicasts a 55-byte
// OTA_ACK ctrl packet back to us (addressed to DEVICE_ID) which we surface as
//   [OTA:ACK,<target>,<session>,<offset>,<status>]
// for the Wizard's _otaRelayAwaitAck sentinel. The wire format (packet types + struct
// bytes) is IDENTICAL to WCB_OTA.cpp / navicore_ota.h, so a WCB, a NaviCore, or this
// relay are interchangeable as the relay. This relay only RECEIVES the 55-byte ACK
// (BEGIN/DATA/END all go relay→target), so RawPkt's 232-byte buffer already covers it.
#define PT_OTA_BEGIN  20
#define PT_OTA_DATA   21
#define PT_OTA_ACK    22
#define PT_OTA_END    23
#define PT_OTA_ABORT  24
#define OTA_PAYLOAD   192      // firmware bytes per OTA_DATA packet (== OTA_ESPNOW_PAYLOAD)

// 55 B — BEGIN / END / ABORT / ACK control packet (this size is unique on the mesh).
typedef struct __attribute__((packed)) {
  char     structPassword[40];
  uint8_t  packetType;
  uint8_t  targetWCB;
  uint8_t  sourceWCB;
  uint8_t  chipFamily;
  uint8_t  status;
  uint16_t sessionId;
  uint32_t imageSize;
  uint32_t ackedOffset;
} relay_ota_ctrl;

// 243 B — one firmware fragment (relay → target ONLY; never received here).
typedef struct __attribute__((packed)) {
  char     structPassword[40];
  uint8_t  packetType;
  uint8_t  targetWCB;
  uint8_t  sourceWCB;
  uint16_t sessionId;
  uint16_t dataLen;
  uint32_t fragOffset;
  uint8_t  data[OTA_PAYLOAD];
} relay_ota_data;

static_assert(sizeof(relay_ota_ctrl) == 55,  "relay_ota_ctrl must be 55 B to match WCB_OTA / navicore_ota");
static_assert(sizeof(relay_ota_data) == 243, "relay_ota_data must be 243 B to match WCB_OTA / navicore_ota");

// ONE reassembly slot, shared by config/stats/etm replies and config parts. A session is keyed
// on (sessionId, packetType, sourceWCB): a frag differing in any of them is another reply, never a
// continuation, so two boards whose random sessionIds collide cannot be spliced into one line. A
// new key normally wipes the slot; the one exception (a legacy reply keeps it against a part
// stream) is in processConfigFrag(). The Wizard sends its pulls through a relay one at a time.
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
    host.printf("[relay] %s%d %s\n",
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
    host.println();
    host.println("*** ========================================");
    host.println("*** WCB Configuration Backup");
    host.println("*** Copy and paste these commands to restore");
    host.println("*** ========================================");
    host.println();
    host.println("?HW,32");
    host.printf ("?MAC,2,%02X\n", MAC_OCT2);
    host.printf ("?MAC,3,%02X\n", MAC_OCT3);
    host.printf ("?WCB,%d\n",     DEVICE_ID);
    host.println("?RELAY,1");     // marks this device a management relay → dedicated Wizard card
    host.printf ("?ALIAS,%s\n",   RELAY_ALIAS);
    host.printf ("?WCBQ,%d\n",    WCB_QUANTITY);
    host.printf ("?EPASS,%s\n",   PASSWORD);
    host.println("?CMDCHAR,;");
    host.println("--------- End of Backup ---------");
    host.println();
}

// ?version → the Wizard reads "Software Version: X", terminated by "End of Version".
void wcbPrintVersion() {
    host.printf("Software Version: %s\n", RELAY_FW);
    host.println("End of Version");
}

// ?WDP,DUMP → the mesh inventory the Wizard's discovery panel shows. Built from this
// relay's WDP neighbor table (getNeighbor), formatted byte-for-byte like the WCB's
// printWdpDump(): a SELF row, one [WDP:...] per neighbor (+ [WDPIF:...] port labels),
// a [WDPCFG:...] summary, and a terminating [WDP:END,count=N].
void wcbPrintWdpDump() {
    unsigned long now = millis();
    // SELF row (this relay). CLIENT=0 → shown as the tethered board; PEER=3 = "this board".
    host.printf("[WDP:N=%d,CLIENT=0,ALIAS=%s,HW=32,HWREV=,FW=%s,CAP=0000,CTRL=0,CAPTAGS=,MAESTRO=-,AGE=0,SEEN=1,PEER=3]\n",
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
        host.printf("[WDP:N=%d,CLIENT=%d,ALIAS=%s,HW=%d,HWREV=%s,FW=%s,CAP=%04X,CTRL=%d,CAPTAGS=%s,MAESTRO=%s,AGE=%lu,SEEN=1,PEER=%d]\n",
                      id, nb->isClient ? 1 : 0, alias, nb->hwVer, hwrev, fw, (unsigned)nb->capFlags,
                      nb->ctrlId, captags, maestro, (now - nb->lastSeenMs) / 1000UL, peerFlag);
        for (int p = 0; p < 5; p++) {
            if (!nb->portLabels[p][0]) continue;
            char lbl[25]; strncpy(lbl, nb->portLabels[p], sizeof(lbl)); lbl[sizeof(lbl)-1] = '\0'; wdpScrub(lbl);
            host.printf("[WDPIF:N=%d,S=%d,DEV=%s]\n", id, p + 1, lbl);
        }
    }
    host.printf("[WDPCFG:EN=1,AUTOJOIN=%d,PEERS=%d]\n", wcb.autoJoinEnabled() ? 1 : 0, peers);
    host.printf("[WDP:END,count=%d]\n", count);
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
    host.printf("[relay] MGMT -> WCB%d (1/1): %s\n", target, payload);
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
  host.printf("[relay] MGMT -> WCB%d frag %u/%u (session %04X)\n", target, idx + 1, total, sid);
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

// True when `s` (what follows "<n>,") is the parts flag: a 'P', then nothing but whitespace.
// The reader below strips only CR/LF, so trailing spaces are tolerated here. Anything else is not
// the flag and the pull goes out plain, as every older relay sends it (atoi reads "<n>,P" as <n>).
static bool relayIsPartsFlag(const char* s) {
  if (*s != 'P' && *s != 'p') return false;
  for (s++; *s; s++)
    if (*s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') return false;
  return true;
}

// ?MGMT,PULL,<n>    CONFIG_REQ (5) x3, 15 ms apart. Unchanged.
// ?MGMT,PULL,<n>,P  CONFIG_REQ_PARTS (19) x3, THEN CONFIG_REQ (5) x3, 15 ms apart. The type-5
//   copies keep an OLDER target answering (it drops type 19 at its receive filter); a new target
//   accepts the first request it hears from us and ignores the rest as duplicates, so the 19s go
//   first. Parts only ever go to a requester that sent a 19. Blocks loop() ~75 ms.
static void sendPull(const char* args) {
  const uint8_t target = (uint8_t)atoi(args);
  const char*   comma  = strchr(args, ',');
  if (comma && relayIsPartsFlag(comma + 1)) {
    if (target < 1 || target > WCB_MAX_BOARDS) return;
    sendConfigReq(target, PT_CONFIG_REQ_PARTS, 3);
    delay(15);
  }
  sendConfigReq(target, PT_CONFIG_REQ, 3);
}

// WiFi-task callback: copy config-frag(230) / remote-term(204) packets for loop() to process.
// MUST NOT print — a multi-packet Serial write here races Core 1 and corrupts long lines.
void onMeshRaw(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (!rawInQueue) return;
  if (len != (int)sizeof(relay_config_frag) && len != (int)sizeof(relay_remote_term) &&
      len != (int)sizeof(relay_ota_ctrl)) return;   // Phase 3: 55-byte OTA ACK
  RawPkt r; r.len = (uint8_t)len; memcpy(r.data, data, len);
  xQueueSend(rawInQueue, &r, 0);               // non-blocking; a dropped frag is covered by the 2nd pass
}

// How long a live config/stats/etm session holds the slot against config parts. Its frags arrive
// 20 ms apart, so 200 ms without one means it has stalled.
#define PART_YIELD_MS 200UL

// loop(): reassemble one reply and print it as ONE line once the session is complete:
//   type 6  (CONFIG)              -> [MGMT:CONFIG,<src>]<body>
//   type 9 / 10  (STATS / ETM)    -> [MGMT:STATS,<src>]<body> / [MGMT:ETM,<src>]<body>
//   type 18, body "P..." (a part) -> [MGMT:CFGPART,<src>]<body>
//   type 18, body "E..." (error)  -> [MGMT:CFGERR,<src>]<body without the E>
//   type 18, anything else        -> dropped
// The relay does not join parts: the Wizard collects the K parts of one id and checks the CRC of
// the joined text. Mirrors WCB.ino handleConfigFragPacket and WCB_Mgmt.h processConfigFrag.
void processConfigFrag(const uint8_t* data) {
  relay_config_frag pkt;
  memcpy(&pkt, data, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (strcmp(pkt.structPassword, PASSWORD) != 0) return;
  if (pkt.requesterWCB != DEVICE_ID) return;
  if (pkt.totalChunks == 0 || pkt.totalChunks > MGMT_CHUNKS) return;
  if (pkt.chunkIdx >= MGMT_CHUNKS || pkt.chunkIdx >= pkt.totalChunks) return;
  if (pkt.packetType != PT_CONFIG_FRAG && pkt.packetType != PT_STATS_FRAG &&
      pkt.packetType != PT_ETM_FRAG    && pkt.packetType != PT_CONFIG_PART) return;
  // The 2nd pass. One value covers every reply type: a target that sends parts never gives two
  // consecutive sessions (each part, each error) the same id.
  if (lastDeliveredSession != 0 && pkt.sessionId == lastDeliveredSession) return;

  const bool sameSession = fragReasm.active && fragReasm.sessionId == pkt.sessionId &&
                           fragReasm.packetType == pkt.packetType &&
                           fragReasm.sourceWCB  == pkt.sourceWCB;
  if (!sameSession) {
    // A LEGACY REPLY KEEPS THE SLOT. A board over 2912 characters streams parts for hundreds of
    // ms, and each part frag would wipe a config/stats/etm reply from ANOTHER board that is still
    // arriving - possibly for an old Wizard, which never asks for parts and whose fixed retry
    // timers would meet the stream again on every attempt. So a part frag yields to a live
    // non-part session and is dropped; the part's second pass or the new Wizard's retry recovers
    // it. A non-part frag always takes the slot, exactly as before parts existed.
    if (pkt.packetType == PT_CONFIG_PART && fragReasm.active &&
        fragReasm.packetType != PT_CONFIG_PART &&
        (millis() - fragReasm.lastMs) < PART_YIELD_MS) return;
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
  if (fragReasm.receivedMask != expected) return;

  // NO String: `full += chunk` skips a chunk SILENTLY when its allocation fails (String::concat
  // returns false and operator+= ignores it), and a heap-starved relay would print a short but
  // well-formed [MGMT:CONFIG] line that a Wizard stores as the board's config. Compact the chunks
  // in place instead: each row holds at most CFG_PAYLOAD chars in CFG_PAYLOAD + 1 bytes, so the
  // text moved down so far never reaches the next row before that row is read.
  char*  body = reinterpret_cast<char*>(fragReasm.chunks);
  size_t len  = 0;
  for (int i = 0; i < pkt.totalChunks; i++) {
    const size_t n = strlen(fragReasm.chunks[i]);
    memmove(body + len, fragReasm.chunks[i], n);
    len += n;
  }
  body[len] = '\0';

  const char* tag  = nullptr;
  const char* text = body;
  if      (fragReasm.packetType == PT_CONFIG_FRAG) tag = "CONFIG";
  else if (fragReasm.packetType == PT_STATS_FRAG)  tag = "STATS";
  else if (fragReasm.packetType == PT_ETM_FRAG)    tag = "ETM";
  else if (body[0] == 'P')                         tag = "CFGPART";
  else if (body[0] == 'E')                       { tag = "CFGERR"; text = body + 1; }

  lastDeliveredSession = fragReasm.sessionId;
  // ONE printf, tag through newline: Print::printf formats into one buffer and makes one write(),
  // so HostOut hands USB and the WebSocket sink each the whole line in one call and nothing another
  // task prints can land inside it or ahead of its newline (the Wizard drops a part line with
  // anything after its '~'). If printf cannot allocate that buffer it prints nothing, and a missing
  // line is a retry, never a wrong config.
  if (tag) host.printf("[MGMT:%s,%d]%s\n", tag, (int)fragReasm.sourceWCB, text);
  memset(&fragReasm, 0, sizeof(fragReasm));
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
  host.printf("[TERM:%d]%s\n", pkt.sourceWCB, line);
}

// CRC-32 (reflected, poly 0xEDB88320) — byte-for-byte the same function as
// WCB.ino's calculateCRC32(), navicore_ota.h's otaCrc32() and the config tool's
// _crc32Hex(). All four must agree: the tool computes it and whichever relay is
// in the path verifies it.
static uint32_t relayCrc32(const String &data) {
  const uint8_t *b = (const uint8_t *)data.c_str();
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < data.length(); i++) {
    crc ^= b[i];
    for (int j = 0; j < 8; j++) crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

// ── Phase 3: OTA relay ────────────────────────────────────────────────────────
// Forward a Wizard "?OTA,<sub>,<target>,<session>,…" line to the target as the matching
// ESP-NOW OTA packet. Mirrors navicore_ota.h processOtaRelayCommand / WCB_OTA.cpp exactly:
// BEGIN/END/ABORT ride the 55-byte ctrl packet, DATA the 243-byte data packet.
// wcb.sendRawPacket() registers the target as an ESP-NOW peer on demand, so OTA does not
// depend on ambient WDP peering (the classic "worked, then silently stopped" relay-OTA bug).
void relayOtaCommand(const char* argsC) {
  String args(argsC);
  int c1 = args.indexOf(',');
  String sub  = (c1 < 0) ? args : args.substring(0, c1);
  String rest = (c1 < 0) ? ""   : args.substring(c1 + 1);
  sub.trim(); sub.toUpperCase();

  int c2 = rest.indexOf(',');
  uint8_t  target  = (uint8_t)((c2 < 0 ? rest : rest.substring(0, c2)).toInt());
  String   r2      = (c2 < 0) ? "" : rest.substring(c2 + 1);
  int c3 = r2.indexOf(',');
  uint16_t session = (uint16_t)((c3 < 0 ? r2 : r2.substring(0, c3)).toInt());
  String   r3      = (c3 < 0) ? "" : r2.substring(c3 + 1);

  if (target < 1 || target > WCB_MAX_BOARDS) { host.printf("[OTA] relay: invalid target %u\n", target); return; }

  // Send and SAY SO IF IT FAILED. sendRawPacket() returns false when the target
  // could not be registered as an ESP-NOW peer — the usual cause being a full
  // peer table (ESP-NOW caps at ~20, and auto-join makes learned peers permanent
  // in NVS, so the set only grows). Discarding that bool made a failed BEGIN
  // indistinguishable from a target that never answered: the Wizard and the
  // NaviCore config tool both just time out and report "no response via relay" /
  // "target rejected BEGIN", with nothing whatsoever on this console. The WCB
  // firmware surfaces the same failure on its own leg (WCB_OTA.cpp otaUnicast).
  auto otaSend = [&](const void* p, size_t n, const char* label) {
    if (wcb.sendRawPacket(target, (const uint8_t*)p, n)) return;
    host.printf("[OTA] relay %s -> WCB%u FAILED to send — could not register the "
                  "peer (ESP-NOW table full? cap ~20; ?WDP,DUMP reports PEERS=n)\n",
                  label, target);
  };

  if (sub == "BEGIN") {
    int p = r3.indexOf(',');
    uint32_t size   = (uint32_t)((p < 0 ? r3 : r3.substring(0, p)).toInt());
    uint8_t  family = (uint8_t) (p < 0 ? 0 : r3.substring(p + 1).toInt());
    relay_ota_ctrl pkt; memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, PASSWORD, sizeof(pkt.structPassword) - 1);
    pkt.packetType = PT_OTA_BEGIN; pkt.targetWCB = target; pkt.sourceWCB = DEVICE_ID;
    pkt.chipFamily = family; pkt.sessionId = session; pkt.imageSize = size;
    otaSend(&pkt, sizeof(pkt), "BEGIN");
    return;
  }

  if (sub == "DATA") {
    int p = r3.indexOf(',');
    if (p < 0) { host.println("[OTA] relay DATA usage: ?OTA,DATA,<t>,<s>,<offset>[:<crc32>],<b64>"); return; }
    String offField = r3.substring(0, p);
    String   b64    = r3.substring(p + 1); b64.trim();
    // Optional integrity suffix on the OFFSET field: "<offset>:<crc32hex>" over
    // "<offset>,<b64>" as transmitted. It rides on the offset because toInt()
    // stops at the ':', so a relay or a sender predating it still interoperates —
    // appending a trailing field would have been swept into the base64 and failed
    // EVERY packet. Dropping a failed line turns silent serial corruption into
    // ordinary loss, which the target's cursor/ACK protocol already recovers from:
    // it does not advance, and the sender rewinds and resends. The ESP-NOW hop
    // needs no cover — 802.11 CRCs every frame in hardware.
    int cpos = offField.indexOf(':');
    String crcHex;
    if (cpos >= 0) { crcHex = offField.substring(cpos + 1); offField = offField.substring(0, cpos); }
    uint32_t offset = (uint32_t)offField.toInt();
    if (crcHex.length()) {
      const uint32_t want = (uint32_t)strtoul(crcHex.c_str(), nullptr, 16);
      const uint32_t have = relayCrc32(offField + "," + b64);
      if (want != have) {
        host.printf("[OTA] relay DATA @%lu DROPPED: crc %08X != %08X (b64 %u chars)\n",
                      (unsigned long)offset, (unsigned)have, (unsigned)want, (unsigned)b64.length());
        return;   // target cursor stalls -> sender rewinds and resends
      }
    }
    relay_ota_data pkt; memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, PASSWORD, sizeof(pkt.structPassword) - 1);
    pkt.packetType = PT_OTA_DATA; pkt.targetWCB = target; pkt.sourceWCB = DEVICE_ID;
    pkt.sessionId = session; pkt.fragOffset = offset;
    size_t outLen = 0;
    int rc = mbedtls_base64_decode(pkt.data, sizeof(pkt.data), &outLen,
                                   (const unsigned char*)b64.c_str(), b64.length());
    if (rc != 0) { host.printf("[OTA] relay DATA base64 error %d\n", rc); return; }
    pkt.dataLen = (uint16_t)outLen;
    otaSend(&pkt, sizeof(pkt), "DATA");
    return;
  }

  if (sub == "END" || sub == "ABORT") {
    relay_ota_ctrl pkt; memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, PASSWORD, sizeof(pkt.structPassword) - 1);
    pkt.packetType = (sub == "END") ? PT_OTA_END : PT_OTA_ABORT;
    pkt.targetWCB  = target; pkt.sourceWCB = DEVICE_ID; pkt.sessionId = session;
    otaSend(&pkt, sizeof(pkt), (sub == "END") ? "END" : "ABORT");
    return;
  }

  host.printf("[OTA] relay: unknown subcommand '%s'\n", sub.c_str());
}

// loop(): a target's 55-byte OTA_ACK → [OTA:ACK,<src>,<session>,<offset>,<status>] on USB,
// the sentinel the Wizard's _otaRelayAwaitAck waits for. Mirrors navicore_ota.h handleOtaAckRelay.
void processOtaAck(const uint8_t* data) {
  relay_ota_ctrl pkt;
  memcpy(&pkt, data, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (strcmp(pkt.structPassword, PASSWORD) != 0) return;
  if (pkt.packetType != PT_OTA_ACK) return;          // BEGIN/END/ABORT are ours→target; ignore any echo
  if (pkt.targetWCB  != DEVICE_ID)  return;           // this ACK is addressed to us (the relay)
  host.printf("[OTA:ACK,%u,%u,%lu,%u]\n",
                pkt.sourceWCB, pkt.sessionId, (unsigned long)pkt.ackedOffset, pkt.status);
}

// ── ?RELAY,WIFI — read and change the WiFi mode without a reflash ─────────────
//   ?RELAY,WIFI                     report mode / SSID / address
//   ?RELAY,WIFI,OFF                 ESP-NOW only
//   ?RELAY,WIFI,AP[,<pass>]         host our own AP (optionally set its password)
//   ?RELAY,WIFI,JOIN,<ssid>,<pass>  join an existing AP (NaviCore's, typically)
//   ?RELAY,WIFI,DEFAULTS            forget NVS, return to the compiled-in values
//
// A PASSWORD IS THE LAST FIELD AND IS TAKEN VERBATIM, commas and all. WPA2 allows
// them and splitting on every comma would silently truncate such a password to its
// first field — which fails as "wrong password" against an AP that is working fine.
//
// Nothing here is echoed back. See the note on relayApPass: this console is
// mirrored to every connected WebSocket client, so printing a password would send
// it over the network it exists to protect.
static void relayWifiCommand(const char* arg) {
#if !RELAY_WIFI_BUILD
    (void)arg;
    host.println("[relay] WiFi was compiled out of this build (RELAY_WIFI_BUILD 0).");
#else
    // Bare "?RELAY" or "?RELAY,WIFI" → report.
    if (!arg[0] || ieq(arg, "WIFI")) {
        host.printf("[relay] WiFi mode: %s%s\n", relayModeName(relayWifiMode),
                    relayWifiMode == RELAY_WIFI_OFF ? "" :
                    (wifiUp ? " (up)" : " (not up yet)"));
        if (relayWifiMode == RELAY_WIFI_JOIN)
            host.printf("[relay]   joining: \"%s\"%s\n",
                        relayJoinSsid.length() ? relayJoinSsid.c_str() : "(unset)",
                        relayJoinPass.length() ? "" : "  (no password set)");
        if (relayWifiMode == RELAY_WIFI_AP)
            host.printf("[relay]   password: %s\n",
                        relayApPass.length() >= 8 ? "set" : "NOT SET - the AP will refuse to start");
        if (wifiUp)
            host.printf("[relay]   ws://%s/ws on channel %u\n",
                        (relayWifiMode == RELAY_WIFI_JOIN && WiFi.status() == WL_CONNECTED
                           ? WiFi.localIP() : WiFi.softAPIP()).toString().c_str(),
                        MESH_CHANNEL);
        host.println("[relay]   ?RELAY,WIFI,OFF | AP[,<pass>] | JOIN,<ssid>,<pass> | DEFAULTS");
        return;
    }
    if (!iStarts(arg, "WIFI,")) {
        host.printf("[relay] unknown ?RELAY command \"%s\" - try ?RELAY,WIFI\n", arg);
        return;
    }
    const char* v = arg + 5;                       // past "WIFI,"

    if (ieq(v, "DEFAULTS")) {
        // Checked, like the write path below. clear() on a namespace that failed to
        // open is a silent no-op, and rebooting while announcing "cleared" would
        // send someone hunting for why the old settings came back.
        if (!relayPrefs.begin(RELAY_NVS_NS, false)) {
            host.println("[relay] could not open NVS - nothing was cleared.");
            return;
        }
        relayPrefs.clear();
        relayPrefs.end();
        relayRequestReboot("WiFi settings cleared");
        return;
    }

    uint8_t mode;
    String  ssid, pass;
    if (ieq(v, "OFF")) {
        mode = RELAY_WIFI_OFF;
    } else if (ieq(v, "AP") || iStarts(v, "AP,")) {
        mode = RELAY_WIFI_AP;
        if (v[2] == ',') pass = v + 3;             // rest of the line, commas included
        else             pass = relayApPass;       // keep whatever is stored
        if (pass.length() < 8) {
            host.printf("[relay] REFUSED: an AP password needs 8+ characters (got %u).\n",
                        (unsigned)pass.length());
            host.println("[relay] This WebSocket relays ;w<id>,<cmd> to every board with no");
            host.println("[relay] credential of its own - an open AP hands over the droid.");
            return;
        }
    } else if (iStarts(v, "JOIN,")) {
        mode = RELAY_WIFI_JOIN;
        const char* s = v + 5;
        const char* comma = strchr(s, ',');
        if (!comma) {
            host.println("[relay] usage: ?RELAY,WIFI,JOIN,<ssid>,<password>");
            return;
        }
        ssid = String(s).substring(0, comma - s);
        pass = comma + 1;                          // verbatim: a password may contain commas
        if (!ssid.length()) {
            host.println("[relay] REFUSED: the SSID to join cannot be empty.");
            return;
        }
    } else {
        host.printf("[relay] unknown WiFi mode \"%s\" - use OFF, AP or JOIN.\n", v);
        return;
    }

    // Write, then reboot. Deliberately NOT applied to the live variables: the
    // ordering rules above relayStartSoftAP() mean a mode only makes sense when
    // setup() runs it in the right order relative to wcb.begin(), and a half-applied
    // pair (new mode, old radio) is the state the reboot exists to avoid.
    if (!relayPrefs.begin(RELAY_NVS_NS, false)) {
        host.println("[relay] could not open NVS - setting NOT saved.");
        return;
    }
    relayPrefs.putUChar("wifiMode", mode);
    if (mode == RELAY_WIFI_AP)   relayPrefs.putString("apPass", pass);
    if (mode == RELAY_WIFI_JOIN) { relayPrefs.putString("joinSsid", ssid);
                                   relayPrefs.putString("joinPass", pass); }
    relayPrefs.end();

    if (mode == RELAY_WIFI_JOIN)
        host.printf("[relay] WiFi mode -> JOIN \"%s\"\n", ssid.c_str());
    else
        host.printf("[relay] WiFi mode -> %s\n", relayModeName(mode));
    relayRequestReboot("WiFi settings saved");
#endif
}

// ── Serial → mesh: parse one line and relay it ────────────────────────────────
// Mirrors the WCB firmware's serial routing:
//   ;w<id>,<cmd> / ;w<alias>,<cmd>  → unicast to that board
//   <no ';' prefix>                 → broadcast to the whole mesh
// The `line` buffer is modified in place (temporary NUL terminators).
void relaySerialLine(char* line) {
    // A pending reboot waits out the rest of whatever burst this line belongs to.
    relayRebootDefer();
    // ── WCB Wizard: answer its local (?) queries so it accepts this relay and can
    // drive the mesh through it. These coexist with the NaviCore tool's ;w / JSON.
    if (!strcmp(line, "WCB_WEBTOOL_CONFIG_PULL")) { wcbPrintBackup(); return; }
    if (line[0] == '?') {
        const char* c = line + 1;                // command after the '?' funcChar
        if (ieq(c, "backup"))  { wcbPrintBackup();  return; }
        if (ieq(c, "version")) { wcbPrintVersion(); return; }
        if (iStarts(c, "RELAY,") || ieq(c, "RELAY")) {
            relayWifiCommand(ieq(c, "RELAY") ? "" : c + 6);
            return;
        }
        if (iStarts(c, "WDP,")) {
            if (ieq(c + 4, "DUMP")) wcbPrintWdpDump();
            // ?WDP,AUTOJOIN / FORGET / CLEAR — a later phase.
            return;
        }
        if (iStarts(c, "MGMT,")) {
            // Route the Wizard's remote-management verbs to the target board.
            //   FRAG      → forward config/command chunks (Phase 2a)
            //   PULL      → request the target's config     (reply: [MGMT:CONFIG,<n>], or
            //               [MGMT:CFGERR,<n>] when the target cannot send it)
            //   PULL,<n>,P → the same, and accept parts: a config over 2912 chars comes back as
            //               [MGMT:CFGPART,<n>] lines, one per part
            //   STATS     → request ESP-NOW stats            (reply: [MGMT:STATS,<n>])
            //   ETM,CHAR  → request ETM characterization     (reply: [MGMT:ETM,<n>])
            // Remote-terminal START/STOP/keystrokes ride inside a FRAG; its OUTPUT comes back
            // as [TERM:<n>] via processRemoteTerm.
            char* m = line + 6;                  // skip "?MGMT,"
            if      (iStarts(m, "FRAG,"))     handleMgmtFrag(m + 5);
            else if (iStarts(m, "PULL,"))     sendPull(m + 5);
            else if (iStarts(m, "STATS,"))    sendConfigReq((uint8_t)atoi(m + 6), PT_STATS_REQ, 1);
            else if (iStarts(m, "ETM,CHAR,")) sendConfigReq((uint8_t)atoi(m + 9), PT_ETM_REQ,   1);
            return;
        }
        if (iStarts(c, "OTA,")) { relayOtaCommand(c + 4); return; }   // Phase 3: wireless OTA relay
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
        host.printf("[relay] broadcast: %s\n", line);
#endif
        return;
    }

    // Only the ;w relay verb is handled — this device is a relay, not a full WCB.
    char verb = line[1];
    if (verb != 'w' && verb != 'W') {
        host.printf("[relay] unsupported ;%c command — this relay only handles ;w<id>,<cmd>\n",
                      verb ? verb : '?');
        return;
    }

    char* p = line + 2;                      // first char after ";w"
    if (*p == '\0') { host.println("[relay] usage: ;w<id>,<command>"); return; }

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
        if (!comma) { host.println("[relay] usage: ;w<alias>,<command>"); return; }
        *comma  = '\0';
        target  = idForAlias(p);
        payload = comma + 1;
        if (!target) { host.printf("[relay] no board known as \"%s\"\n", p); return; }
    }

    if (target < 1 || target > WCB_MAX_BOARDS) {
        host.printf("[relay] bad target id %d (use 1-%d, or %d for NaviCore)\n",
                      target, WCB_MAX_BOARDS, NAVICORE_ID);
        return;
    }

    wcb.send(target, payload);               // ensured (ETM + CRC); auto-fragments if long
#if VERBOSE
    host.printf("[relay] -> WCB%d: %s\n", target, payload);
#endif
}

void setup() {
    // RX headroom BEFORE begin(). The default ring is 256 B, and a relayed OTA
    // streams a window of 8 ?OTA,DATA lines of ~284 B = 2272 B — so the ring only
    // survives while this loop keeps draining it, and one stall (an inline
    // ESP-NOW send is enough) overflows it. That does not merely LOSE a line: it
    // drops a contiguous run of bytes from the MIDDLE of one, and if the run
    // falls inside the base64 field with the newline intact, the remainder is
    // STILL valid base64 and STILL decodes to a shorter, re-phased byte string.
    // We would forward it as a normal fragment at the offset the sender named,
    // the target would write it and advance its cursor, and nothing would notice
    // until the SHA at 100%. 8 KB holds ~28 such lines. Works for both a
    // HardwareSerial UART0 and a native-USB HWCDC Serial.
    Serial.setRxBufferSize(8192);
    Serial.begin(115200);
    // Queue for inbound mesh lines, printed from loop() (see the RelayLine note).
    // Created before begin() so it exists before the receive callback can fire.
    relayOutQueue = xQueueCreate(24, sizeof(RelayLine));
    rawInQueue    = xQueueCreate(24, sizeof(RawPkt));   // Phase 2b: config/stats/etm/term replies
    wcb.onCommand(onMeshData);
    wcb.onStatusChange(onMeshStatus);
    wcb.onRawPacket(onMeshRaw);                          // Phase 2b: raw reply/terminal packets

#if RELAY_WIFI_BUILD
    // BEFORE the mode is used for anything: the stored mode and credentials decide
    // both branches below, and reading NVS here means one snapshot serves the whole
    // boot rather than each site re-reading and possibly disagreeing.
    relayLoadWifiSettings();
    if (relayWifiMode == RELAY_WIFI_AP) {
        // BEFORE wcb.begin() — see the ordering note above relayStartSoftAP().
        relayStartSoftAP();
    }
#endif
    // Pin the expected mesh channel before begin(). With no AP up, WCB_Client sets
    // the radio to it; with our AP already up, the AP owns the channel and this is
    // what update() compares against to catch a drift.
    wcb.setMeshChannel(MESH_CHANNEL);

    // Join the WCB network. false = ESP-NOW did not start; halt rather than run
    // update()/send against an uninitialised driver (which would crash).
    if (!wcb.begin()) {
        host.println("[relay] begin() FAILED (see error above) — halting.");
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

#if RELAY_WIFI_BUILD
    if (relayWifiMode == RELAY_WIFI_JOIN) {
    // AFTER wcb.begin() — begin() calls WiFi.disconnect() and would undo the join.
    // Only KICKS OFF the first attempt; relayJoinService() in loop() carries it from
    // here, so a NaviCore that is still booting costs us no blocked time at all. If
    // it never appears we host our own AP instead — raising it late is safe because
    // it goes up on MESH_CHANNEL, and the conflict that ordering normally causes is a
    // CHANNEL conflict, which softAP() on the pinned channel cannot create.
    if (!relayJoinSsid.length()) {
        host.println("[relay] JOIN mode selected but no SSID is set - hosting our own AP.");
        host.println("[relay]   set one with: ?RELAY,WIFI,JOIN,<ssid>,<pass>");
        joinSettled = true;
        relayStartSoftAP();
    } else {
        host.printf("[relay] looking for \"%s\" on channel %u (up to %lu s, then our own AP)\n",
                    relayJoinSsid.c_str(), MESH_CHANNEL,
                    (unsigned long)(JOIN_GIVEUP_MS / 1000));
        relayJoinAttempt();
        joinNextAttemptMs = millis() + JOIN_RETRY_MS;
    }
    }
#endif

    // Boot lines the WCB Wizard's sniffer reads to learn this device's command
    // chars (funcChar '?', cmdChar ';') + version. Also satisfies the Wizard's
    // auto-detect, which triggers on ANY bytes emitted after reset.
    host.println("Delimeter Character: ^");
    host.println("Local Function Identifier: ?");
    host.println("Command Character: ;");
    host.printf ("Software Version: %s\n", RELAY_FW);

    host.println("[relay] WCB management relay ready.");
    host.println("[relay]   ;w<id>,<cmd>    relay to WCB <id> (20 = NaviCore)");
    host.println("[relay]   ;w<alias>,<cmd> relay by advertised name");
    host.println("[relay]   <text>          broadcast to all boards");
}

// 384 B: a relay OTA "?OTA,DATA,<t>,<s>,<offset>,<base64>" line is the longest
// input we take — a 192-byte firmware chunk base64-encodes to 256 chars, plus the
// ~16-27 char prefix = up to 283 chars. inBuf[256] would truncate EVERY DATA frame
// at the "line too long" guard below, silently dropping the whole OTA stream (BEGIN
// acks, then nothing transfers). Sized with headroom so DATA lines pass intact.
char   inBuf[384];
size_t inLen = 0;
// Set when a line overruns inBuf, cleared at the next newline. Without it, zeroing
// inLen mid-line does not DROP the line — it restarts accumulation, so the tail of
// an oversized line is treated as a fresh command at the newline. An unprefixed
// tail falls through relaySerialLine() to wcb.broadcast(), putting an arbitrary
// mid-line fragment on EVERY board on the mesh moments after the operator was told
// the line was dropped. Suppress to end-of-line instead.
bool   inOverrun = false;

void loop() {
    wcb.update();                            // heartbeats out, offline detection, WDP, retries

    // Print inbound mesh lines the receive callback queued. Doing ALL Serial
    // writes from this one task (Core 1) is what keeps large lines (CONFIG
    // fragments, rc_hb) from being corrupted by a cross-core write race.
    RelayLine out;
    while (relayOutQueue && xQueueReceive(relayOutQueue, &out, 0) == pdTRUE)
        host.println(out.buf);

    // Decode raw config/stats/etm/terminal packets queued by onMeshRaw (Phase 2b).
    RawPkt rp;
    while (rawInQueue && xQueueReceive(rawInQueue, &rp, 0) == pdTRUE) {
        if      (rp.len == (uint8_t)sizeof(relay_config_frag)) processConfigFrag(rp.data);
        else if (rp.len == (uint8_t)sizeof(relay_remote_term)) processRemoteTerm(rp.data);
        else if (rp.len == (uint8_t)sizeof(relay_ota_ctrl))    processOtaAck(rp.data);   // Phase 3
    }
    // Drop a stalled reassembly (lost fragment) after 6 s so a later pull isn't wedged.
    if (fragReasm.active && (millis() - fragReasm.lastMs) > 6000) fragReasm.active = false;

    // Read USB serial one line at a time (CR/LF terminated) and relay it.
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            // End of line: relay it, unless the line overran — then drop the WHOLE
            // line (nothing has been relayed) and re-arm for the next one.
            if (inOverrun) { inOverrun = false; inLen = 0; }
            else if (inLen) { inBuf[inLen] = '\0'; relaySerialLine(inBuf); inLen = 0; }
        } else if (inOverrun) {
            continue;                         // still swallowing an oversized line
        } else if (inLen < sizeof(inBuf) - 1) {
            inBuf[inLen++] = c;
        } else {
            inOverrun = true;                 // overrun — swallow to end-of-line
            inLen     = 0;
            host.println("[relay] line too long — dropped (use the Config Tool for big configs)");
        }
    }

#if RELAY_WIFI_BUILD
    // Carry a pending join forward (JOIN mode) and start the WebSocket once an
    // interface exists. Non-blocking: the mesh keeps running the whole time.
    relayWifiService();
    // Run at most ONE queued WebSocket command per pass, on THIS task — the httpd
    // handler that received it runs on Core 0 beside the ESP-NOW callback and may
    // only copy-enqueue-return (see mgmt_wsserver.h). Then flush host output to the
    // sockets at this one known point, never from inside a print.
    mgmtws::drain();
    mgmtws::pump();
#endif

    // LAST, and after the pump above, so the "rebooting to apply" line has actually
    // reached the WebSocket clients before the socket dies with the restart.
    relayRebootService();
}
