#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <atomic>

// Forward declaration — WCBStream is defined in WCBStream.h.
// Keeps this header self-contained; users include WCBStream.h only when needed.
class WCBStream;

// =============================================================================
// WCB_Client Library
//
// Allows any ESP32-based device to join a Wireless Communication Board (WCB)
// ESP-NOW network as a first-class peer — sending commands, receiving commands,
// and participating in the Ensured Transmission Mode (ETM) heartbeat system.
//
// ── How the WCB network works ────────────────────────────────────────────────
// All devices on a WCB network share two MAC octets (oct2, oct3) and a
// password. Each board is assigned a unique ID (1–20) and its WiFi MAC is
// set to the scheme:  02:oct2:oct3:00:00:ID
// This lets every board compute every other board's MAC address without any
// discovery — they all know the scheme.
//
// ── Ensured Transmission Mode (ETM) ──────────────────────────────────────────
// ETM adds reliability on top of ESP-NOW's best-effort delivery:
//   Heartbeats : each board broadcasts a heartbeat packet every N seconds so
//                all peers can track who is online.
//   ACKs       : every COMMAND packet is acknowledged. If the sender doesn't
//                receive an ACK it can retry (retry logic is in WCB firmware;
//                this library sends and tracks ACKs but does not auto-retry).
//   Checksum   : a CRC32 is appended to every command string so corrupt
//                packets are rejected before reaching the application.
//
// ── Packet format ────────────────────────────────────────────────────────────
// All packets use wcb_packet_etm_t (252 bytes, packed).
// The structCommand field carries the command string. When checksum is enabled
// the format is:  <command>|CRC<8 uppercase hex digits>
// e.g.  :PP100|CRC1A2B3C4D
// =============================================================================


// ─────────────────────────────────────────────────────────────────────────────
// Packet type constants
// Stored in wcb_packet_etm_t::structPacketType.
// Must stay in sync with WCB firmware.
// ─────────────────────────────────────────────────────────────────────────────
#define WCB_PACKET_COMMAND    0   // A text command (or raw binary) directed at a target
#define WCB_PACKET_ACK        1   // Acknowledgement that a COMMAND was received
#define WCB_PACKET_HEARTBEAT  2   // Periodic keepalive broadcast — no command payload

// ─────────────────────────────────────────────────────────────────────────────
// Special target IDs
// Used in wcb_packet_etm_t::structTargetID.
// Must stay in sync with WCB firmware.
// ─────────────────────────────────────────────────────────────────────────────
#define WCB_TARGET_BROADCAST   0   // Packet is addressed to all WCBs simultaneously

// Pass as target_wcb in WCBStream / monitorRaw to broadcast via the Kyber path.
//   WCBStream maestro(broadcast);       // all WCBs with Kyber_Remote
//   WCBStream maestro(2, 1);            // unicast to WCB2 port 1
constexpr uint8_t broadcast = 0;
#define WCB_TARGET_RAW_SERIAL  97  // Unicast: target WCB writes payload as raw bytes
                                   // to the specified serial port (3-byte header: port,
                                   // len_lo, len_hi). No KYBER config required on the
                                   // receiving WCB — the port just needs to be wired.
#define WCB_TARGET_KYBER       98  // Broadcast: ALL WCBs with Kyber_Remote configured
                                   // write the payload to their local Maestro ports.
                                   // Use sendKyber() to build and send this packet type.
                                   // Firmware header: len_lo, len_hi, then data bytes.

// ─────────────────────────────────────────────────────────────────────────────
// Library limits
// ─────────────────────────────────────────────────────────────────────────────
#define WCB_MAX_BOARDS   20   // Maximum WCB IDs supported (1–20)
#define WCB_PENDING_MAX  10   // In-flight COMMAND slots tracked for ACK.
                              // Matches the WCB firmware's ETM_PENDING_MAX so
                              // client ensured traffic has the same depth as
                              // WCB-to-WCB ETM. When full, the oldest slot is
                              // evicted (see _findFreePending) — same policy as
                              // the firmware — rather than dropping the new send.
#define WCB_SPECIAL_ID   20   // Device ID 20 is an out-of-band slot for third-party
                              // devices that don't consume a WCB slot in the system.
                              // Requires specialPeerEnabled = true on the WCBs.

// ── Ensured-delivery (ETM) retransmit tuning ─────────────────────────────────
// Applies ONLY to send()/broadcast() calls made with ensured=true. These values
// MIRROR the WCB firmware's own ETM retry engine (processETMAcksAndRetries) so
// client-originated ensured traffic behaves identically to WCB-to-WCB ETM:
//   • initial send goes out as-is (unicast, or a single broadcast),
//   • then PER-BOARD UNICAST retries (reusing the original sequence number) to
//     each expected board that hasn't ACK'd, every ETM_RETRY_INTERVAL_MS,
//   • up to ETM_MAX_RETRIES per board; a board that drops offline is dropped
//     from the expected set rather than retried forever.
// Matches the firmware defaults (etmTimeoutMs = 500, 3 retries). Default
// (ensured=false) sends are single-transmit with NO retry of any kind —
// fire-and-forget for broadcast, send-once for unicast. Guaranteed delivery
// comes ONLY from ETM (ensured=true); there is no other reliability layer.
#define ETM_RETRY_INTERVAL_MS  500  // ms between retry passes (matches firmware etmTimeoutMs)
#define ETM_MAX_RETRIES         3   // per-board unicast retries before giving up (matches firmware)

// ─────────────────────────────────────────────────────────────────────────────
// Packet structs
//
// Both structs are __attribute__((packed)) so the compiler doesn't add any
// padding bytes. The layout must match the WCB firmware exactly — do not
// reorder or resize any fields.
// ─────────────────────────────────────────────────────────────────────────────

// Basic packet — used by WCBs running without ETM.
// This library always uses wcb_packet_etm_t, but the struct is provided here
// for reference and for parsing legacy packets if needed.
typedef struct __attribute__((packed)) {
    char    structPassword[40];       // Network password — must match on all peers
    char    structSenderID[4];        // ASCII decimal ID of the sending device
    char    structTargetID[4];        // ASCII decimal ID of the target (0 = broadcast)
    uint8_t structCommandIncluded;    // 1 if structCommand carries data, 0 otherwise
    char    structCommand[200];       // Command string payload (null-terminated)
} wcb_packet_t;

// ETM-extended packet — used for all packets when ETM is active.
// Extends wcb_packet_t with packet type and sequence number fields appended
// at the end so the struct remains backward-compatible with the basic layout.
typedef struct __attribute__((packed)) {
    char     structPassword[40];      // Network password — must match on all peers
    char     structSenderID[4];       // ASCII decimal ID of the sending device
    char     structTargetID[4];       // ASCII decimal ID of the target (0 = broadcast)
    uint8_t  structCommandIncluded;   // 1 if structCommand carries data, 0 otherwise
    char     structCommand[200];      // Command string payload (null-terminated).
                                      // When checksum is enabled the format is:
                                      //   <command>|CRC<8 uppercase hex digits>
                                      // e.g.  :PP100|CRC1A2B3C4D
                                      // The library strips the suffix before delivering
                                      // to the application callback.
    uint8_t  structPacketType;        // WCB_PACKET_COMMAND / ACK / HEARTBEAT
    uint16_t structSequenceNumber;    // Monotonically increasing per sender; used to
                                      // match ACKs back to their originating COMMAND.
} wcb_packet_etm_t;

// ─────────────────────────────────────────────────────────────────────────────
// MGMT fragmentation packet (226 bytes, packed) — used for AUTOMATIC
// fragmentation of unicast commands longer than the single-packet limit.
// Must stay byte-identical to the WCB firmware's espnow_struct_mgmt: the
// firmware dispatches incoming ESP-NOW packets BY SIZE (226 → MGMT reassembly),
// reassembles the chunks per sessionId, and executes the full command through
// its normal parser. Duplicate chunks are idempotent; completed sessions are
// remembered, so retransmitted passes are safely discarded.
// ─────────────────────────────────────────────────────────────────────────────
#define WCB_MGMT_PACKET_TYPE_FRAG  3     // firmware PACKET_TYPE_MGMT_FRAG (wizard relay)
#define WCB_MGMT_PACKET_TYPE_FRAG_UNICAST 5  // firmware PACKET_TYPE_MGMT_FRAG_UNICAST:
                                         // chunk sent directly by this library. On
                                         // firmware that knows the type (6.1.1+), the
                                         // reassembled command keeps UNICAST semantics
                                         // (no re-broadcast of broadcastable tokens).
                                         // Older firmware ignores packetType and treats
                                         // it as a normal FRAG — command still executes,
                                         // but broadcastable tokens may propagate.
#define WCB_MGMT_MAX_CHUNKS        16    // firmware MGMT_MAX_CHUNKS (uint16 mask)
#define WCB_MGMT_CHUNK_LEN         179   // payload[180] minus NUL (firmware strncpy)
#define WCB_MGMT_MAX_COMMAND_LEN   ((size_t)WCB_MGMT_CHUNK_LEN * WCB_MGMT_MAX_CHUNKS)

typedef struct __attribute__((packed)) {
    char     structPassword[40];   // network password — must match all peers
    uint8_t  packetType;           // WCB_MGMT_PACKET_TYPE_FRAG
    uint8_t  targetWCB;            // board this session is addressed to
    uint16_t sessionId;            // ties all chunks of one command together
    uint8_t  chunkIdx;             // 0-based chunk index
    uint8_t  totalChunks;          // total chunks in this session
    char     payload[180];         // command-string fragment
} wcb_packet_mgmt_t;

// ─────────────────────────────────────────────────────────────────────────────
// Internal state types
// ─────────────────────────────────────────────────────────────────────────────

// Tracks whether a specific WCB is currently online and when it was last seen.
// A WCB is considered online as long as its heartbeats arrive within the
// expected window (heartbeatInterval * missedBeforeOffline seconds).
struct WCBBoardStatus {
    bool          online;       // true = board has sent a recent heartbeat
    unsigned long lastSeenMs;   // millis() timestamp of the last heartbeat received
};

// Tracks an in-flight COMMAND that is waiting for an ACK.
// When an ACK arrives with a matching seqNum the slot is updated; a unicast
// slot is freed on its target's ACK, a broadcast slot when all expected
// recipients have ACK'd.
//
// Best-effort sends (ensured=false): the slot is informational only — there
// is NO retransmit of any kind (a best-effort unicast is sent once; best-effort
// broadcasts aren't even tracked).  ENSURED sends (ensured=true) ARE
// retransmitted by update() via the ETM ACK/retry below until complete.
struct WCBPending {
    bool          active;                     // true = this slot is in use
    uint16_t      seqNum;                     // sequence number of the tracked packet
    char          command[200];               // copy of the command string (resent on retry)
    unsigned long sentMs;                     // millis() of the last (re)transmit
    uint8_t       targetID;                   // target WCB ID (or WCB_TARGET_BROADCAST)
    bool          ackReceived[WCB_MAX_BOARDS];// which boards have ACK'd this sequence
    // ── Ensured-delivery state (only meaningful when `ensured` is true) ──
    bool          ensured;                    // retransmit until every expected board ACKs
    bool          expected[WCB_MAX_BOARDS];   // boards that must ACK (snapshot of online set at send)
    uint8_t       retryCount[WCB_MAX_BOARDS]; // PER-BOARD unicast retries done (mirrors firmware)
};

// ─────────────────────────────────────────────────────────────────────────────
// Callback signatures
// ─────────────────────────────────────────────────────────────────────────────

// Called when a COMMAND packet arrives from the WCB network addressed to this
// device or broadcast to all devices.
//   senderID : WCB number that sent the command (1–20)
//   command  : null-terminated command string with the |CRC suffix already stripped
typedef void (*WCBCommandCallback)(uint8_t senderID, const char* command);

// Called when a WCB transitions between online and offline.
//   wcbID  : WCB number (1–20)
//   online : true = board just came online, false = board just went offline
// The library tracks online/offline state internally regardless of whether this
// callback is registered — use isOnline() to poll status at any time.
typedef void (*WCBStatusCallback)(uint8_t wcbID, bool online);


// =============================================================================
// WCB_Client
//
// One instance per sketch. Declare it at global scope so it persists for the
// lifetime of the program. Pass all configuration in the constructor, then
// call begin() once from setup() to start ESP-NOW.
// =============================================================================
class WCB_Client {
public:

    // ── Construction ─────────────────────────────────────────────────────────

    // Stores network credentials and optional callbacks.
    // Does NOT touch WiFi or ESP-NOW hardware — safe to call at global scope
    // before the Arduino runtime has initialised the hardware.
    //
    // mac_oct2    : 2nd octet of the shared WCB MAC scheme  (e.g. 0x22)
    // mac_oct3    : 3rd octet of the shared WCB MAC scheme  (e.g. 0x33)
    // password    : network password — must match all WCBs  (max 39 chars)
    // wcb_quantity: total WCBs in the system (?WCBQ value)
    // device_id   : this device's unique ID on the network
    //               1–19 : must be <= wcb_quantity; WCBs pre-register this MAC
    //               20   : special out-of-band slot (requires specialPeerEnabled
    //                       on the WCBs, does not consume a WCB slot)
    // commandCb   : optional — called when a command is received from the network
    // statusCb    : optional — called when a WCB comes online or goes offline
    WCB_Client(uint8_t mac_oct2, uint8_t mac_oct3,
              const char* password, uint8_t wcb_quantity, uint8_t device_id,
              WCBCommandCallback commandCb = nullptr,
              WCBStatusCallback  statusCb  = nullptr);

    // ── Initialisation ───────────────────────────────────────────────────────

    // Initialise WiFi (STA mode), set the custom MAC address, start ESP-NOW,
    // register all WCB peers, and begin the heartbeat timer.
    // Call once from setup(). Returns true on success.
    // On failure the library prints a descriptive error to Serial.
    bool begin();

    // ── Main loop ────────────────────────────────────────────────────────────

    // Must be called every iteration of loop().
    // Drives two background tasks:
    //   1. Heartbeat timer  — broadcasts a heartbeat packet every N seconds so
    //                         other WCBs know this device is still alive.
    //   2. Offline detection — marks WCBs as offline if their heartbeats stop
    //                          arriving within the expected window and fires the
    //                          status callback.
    // Do not block loop() with delays or this will miss heartbeat windows and
    // the network will think this device went offline.
    void update();

    // ── Sending ──────────────────────────────────────────────────────────────

    // Send a text command to one specific WCB.
    // target_wcb : WCB number to address (1–WCB_MAX_BOARDS)
    // command    : null-terminated command string. Fits-in-one-packet limit is
    //              199 chars (187 with checksum enabled — the |CRCxxxxxxxx
    //              suffix shares the same 200-byte field).
    //              LONGER commands are fragmented AUTOMATICALLY (since 1.3.0):
    //              the library splits them into MGMT chunks that the target WCB
    //              reassembles and executes whole — long ?SEQ,SAVE sequences
    //              just work, up to ~2.8 KB (16 chunks × 179 chars).
    //              Fragmented delivery has NO per-chunk ACK at the firmware
    //              layer; the library compensates by transmitting each chunk
    //              multiple times (3 passes when ensured, 2 otherwise —
    //              duplicates are harmless, the target dedups by session).
    //              Since 1.3.1 fragmented sends are NON-BLOCKING: the chunks
    //              are queued and transmitted from update() (~10 ms apart), so
    //              send() returns immediately — true means QUEUED, and the
    //              transmission result is logged when the job completes. Keep
    //              calling update() regularly (you must anyway). Only ONE
    //              fragmented send may be in flight at a time; a second one
    //              returns false until the first finishes (~0.5 s max).
    // ensured    : true (default) → application-layer ETM ensured delivery:
    //              retransmit (reusing the sequence number) until the target
    //              ACKs at the ETM layer, up to ETM_MAX_RETRIES. This matches
    //              the WCB firmware, which sends with ETM ON by default — so
    //              client commands are reliable by default, just like WCB-to-WCB.
    //              false → sent ONCE, no retry. Use for high-rate traffic where
    //              the next update supersedes a lost one (you generally want the
    //              raw streaming helpers — sendRaw/WCBStream — for that instead).
    // Returns true if ESP-NOW accepted the (first) packet for transmission.
    bool send(uint8_t target_wcb, const char* command, bool ensured = true);

    // Broadcast a text command to ALL WCBs on the network simultaneously.
    // Sends one ESP-NOW packet to the shared broadcast MAC; every WCB receives it.
    // SIZE LIMIT: broadcast does NOT fragment — commands longer than the
    // single-packet limit (199 chars / 187 with checksum) FAIL with an error
    // log and return false. Fragmentation is unicast-only (the reassembly
    // session on the target is per-board): send() the long command to each
    // board individually instead.
    // ensured : true (default) → ENSURED broadcast: the packet is retransmitted
    //           (per-board unicast, reusing the sequence number) until every
    //           board that was online at send time has ACK'd at the ETM layer
    //           (or dropped offline), up to ETM_MAX_RETRIES. Mirrors the WCB
    //           firmware, which ensures broadcasts to every online board when
    //           ETM is on — so "command all boards" actions land by default.
    //           false → fire-and-forget (no ACK tracking, no retry). Correct for
    //           periodic telemetry / status spam where reliability isn't needed
    //           and you don't want to spend a pending slot + retransmits on it.
    // Returns true if ESP-NOW accepted the (first) packet for transmission.
    bool broadcast(const char* command, bool ensured = true);

    // Send raw binary data to a specific WCB for forwarding out one of its
    // serial ports. Use this to deliver Pololu / Maestro binary protocol packets
    // without text encoding.
    //
    // This is a UNICAST to one specific WCB:port pair. The receiving WCB writes
    // the bytes directly to the specified serial port — no Kyber configuration
    // is required on the receiving board.
    //
    // The WCB firmware expects a 3-byte header inside structCommand:
    //   [0]   target_port  — serial port on the WCB to write to (1–5)
    //   [1–2] data length  — little-endian uint16
    //   [3..] data bytes
    // sendRaw() builds this header automatically from target_port and len.
    //
    // target_wcb  : WCB number to route through (1–WCB_MAX_BOARDS)
    // target_port : serial port on that WCB (1–5) connected to the Maestro
    // data        : pointer to the binary byte array
    // len         : number of bytes (max 177 — firmware limit for raw chunks)
    // Returns true if ESP-NOW accepted the packet for transmission.
    bool sendRaw(uint8_t target_wcb, uint8_t target_port,
                 const uint8_t* data, size_t len);

    // Broadcast raw binary data to ALL WCBs simultaneously via the Kyber path.
    //
    // This is a BROADCAST — every WCB on the network receives the packet. Any
    // WCB that has Kyber_Remote configured will forward the bytes to its local
    // Maestro serial port(s) automatically. WCBs without Kyber_Remote ignore it.
    //
    // Use this instead of sendRaw() when:
    //   - You have Maestros on multiple WCBs and want to address all of them.
    //   - You don't know which WCB has the Maestro (broadcast, let each decide).
    //   - You want to mirror Kyber servo-passthrough traffic across the network.
    //
    // The packet uses WCB_TARGET_KYBER (98) with the ETM packet format.
    // No CRC is added — the data is treated as opaque binary.
    //
    // data : pointer to the binary byte array (Maestro/Pololu command bytes)
    // len  : number of bytes (max 178 — 2-byte firmware header leaves 178 usable
    //        bytes out of structCommand's 180-byte usable space)
    // Returns true if ESP-NOW accepted the packet for transmission.
    bool sendKyber(const uint8_t* data, size_t len);

    // ── Status ───────────────────────────────────────────────────────────────

    // Returns true if the specified WCB has sent a heartbeat recently enough
    // to be considered online. Use this before send() if you need to confirm
    // the target is reachable before dispatching a critical command.
    bool isOnline(uint8_t wcb_number);

    // ── Special peer (NaviCore) ────────────────────────────────────────────────

    // Enable two-way communication with the out-of-band "special peer" — e.g.
    // NaviCore, which normally lives at ID 20 OUTSIDE the 1..wcb_quantity range
    // (so it does not consume a WCB slot). This:
    //   • registers the special peer's MAC as an ESP-NOW peer (ESP-NOW requires
    //     this before send()/sendToSpecialPeer() can reach it), and
    //   • includes it in online/offline (ETM heartbeat) tracking, so
    //     isSpecialPeerOnline() / isOnline(id) and the status callback work for it.
    //
    // Call from setup(); works before OR after begin() (if begin() already ran,
    // the peer is registered immediately). The matching WCBs must have the
    // special peer enabled too (?SPECIAL,ON,<id>).
    //
    // id : the special peer's ID (1–20). Defaults to WCB_SPECIAL_ID (20).
    void enableSpecialPeer(uint8_t id = WCB_SPECIAL_ID);

    // True if the special peer has sent a heartbeat within the offline threshold.
    // Always false until enableSpecialPeer() has been called.
    bool isSpecialPeerOnline();

    // Send a text command to the special peer (e.g. NaviCore). Convenience wrapper
    // around send(specialPeerID, ...). Returns false if enableSpecialPeer() was
    // never called.
    bool sendToSpecialPeer(const char* command, bool ensured = true);

    // ── Callbacks ────────────────────────────────────────────────────────────

    // Register or replace the command callback after construction.
    // Can be called at any time; the new callback takes effect immediately.
    void onCommand(WCBCommandCallback callback);

    // Register or replace the status callback after construction.
    // Can be called at any time; the new callback takes effect immediately.
    void onStatusChange(WCBStatusCallback callback);

    // ── Serial monitoring ─────────────────────────────────────────────────────

    // Monitor a UART for raw binary packets output by an attached device
    // (e.g. a Pololu/Maestro/Kyber writing servo commands out its TX pin).
    //
    // How to wire it:
    //   The device outputs bytes on its TX pin. Connect that TX pin to an
    //   unused UART's RX pin on this ESP32 (a hardware tap). Pass that UART
    //   here. The library reads bytes as they arrive, buffers them, and sends
    //   the buffer via sendRaw() to target_wcb when the inter-frame gap
    //   (gap_ms milliseconds of silence) signals the end of a packet.
    //
    //   Default gap_ms of 2 ms works well for small Pololu packets at typical
    //   baud rates (9600–115200). Increase it if packets are large or slow.
    //
    // Call once from setup() after begin(). Driven automatically by update().
    //
    // target_wcb : WCB number that will forward the bytes to the servo controller
    // gap_ms     : milliseconds of silence that marks the end of a packet
    void monitorRaw(HardwareSerial& port, uint8_t target_wcb, uint8_t target_port = 0,
                    uint16_t gap_ms = 2);

    // Monitor a UART for newline-terminated text commands output by an attached
    // device (e.g. a host controller writing WCB command strings).
    //
    // Each complete line (terminated by 'terminator') is forwarded via send()
    // or broadcast() to target_wcb. Use WCB_TARGET_BROADCAST (0) as target_wcb
    // to send each captured line to all WCBs simultaneously.
    //
    // Call once from setup() after begin(). Driven automatically by update().
    //
    // target_wcb : WCB number to forward commands to, or WCB_TARGET_BROADCAST
    // terminator : character that marks end of a command line (default '\n')
    void monitorSerial(HardwareSerial& port, uint8_t target_wcb, char terminator = '\n');

    // ── Configuration ────────────────────────────────────────────────────────

    // Enable or disable CRC32 checksum on outgoing commands and verification
    // of checksums on incoming commands.
    //
    // This MUST match the ?ETM,CHKSM setting on all WCBs in the network:
    //   setChecksum(true)  → WCBs must have CHKSM ON  (the default)
    //   setChecksum(false) → WCBs must have CHKSM OFF
    //
    // When enabled:
    //   - Outgoing: "|CRC{8 hex}" is appended to every command string before sending.
    //   - Incoming: the CRC suffix is verified and then stripped before the command
    //               is delivered to your callback. Packets with a bad or missing
    //               checksum are silently rejected.
    //
    // When disabled:
    //   - Commands are sent and received as plain strings with no checksum processing.
    //
    // Default: true  (matches WCB factory default of CHKSM ON)
    // Note: enabling checksum reduces the usable command length from 200 to ~188 chars.
    void setChecksum(bool enabled);


private:

    // ── Stored configuration ─────────────────────────────────────────────────
    uint8_t  _oct2, _oct3;       // Shared MAC octets identifying this WCB network
    char     _password[40];      // Network password
    uint8_t  _quantity;          // Total WCBs in the system
    uint8_t  _deviceID;          // This device's ID (1–20)
    uint8_t  _specialPeerID = 0; // Out-of-band special peer (NaviCore) ID; 0 = not enabled
    bool     _started       = false; // True once begin() has registered ESP-NOW peers

    // ── MAC address tables ───────────────────────────────────────────────────
    // Pre-computed MAC addresses for every possible WCB slot and the broadcast
    // address. Built by _buildMACs() and used by _registerPeers() and send methods.
    uint8_t        _wcbMACs[WCB_MAX_BOARDS][6];  // _wcbMACs[i] = MAC for WCB (i+1)
    uint8_t        _broadcastMAC[6];             // Broadcast MAC for the network

    // ── Board tracking ───────────────────────────────────────────────────────
    WCBBoardStatus _boards[WCB_MAX_BOARDS];   // Online/offline state per WCB slot
    WCBPending     _pending[WCB_PENDING_MAX]; // In-flight commands awaiting ACK

    // ── ETM state ────────────────────────────────────────────────────────────
    // Atomic: the sequence number is incremented from BOTH cores — the main
    // loop (send()/broadcast() via the application) AND the ESP-NOW receive
    // callback (WiFi task), since the command callback may reply with send().
    // A non-atomic ++ races and can hand the same sequence number to two
    // packets, causing the receiver's duplicate-detection to silently drop
    // one of them.  std::atomic<uint16_t> is lock-free on Xtensa.
    std::atomic<uint16_t> _seqCounter;  // monotonic per-COMMAND sequence number
    unsigned long _nextHeartbeatMs;  // millis() when the next heartbeat is due

    // ETM timing — defaults match WCB firmware factory defaults.
    // Changing these here without matching changes on the WCBs will cause
    // incorrect online/offline detection timing.
    uint16_t _heartbeatIntervalSec = 10;  // How often to send a heartbeat (seconds)
    uint8_t  _missedBeforeOffline  = 3;   // Missed heartbeats before marking offline

    bool _checksumEnabled = true;  // CRC32 on/off — must match ?ETM,CHKSM on WCBs

    // ── Callbacks ────────────────────────────────────────────────────────────
    WCBCommandCallback _commandCallback = nullptr;
    WCBStatusCallback  _statusCallback  = nullptr;

    // ── WCBStream registry ───────────────────────────────────────────────────
    // WCBStream instances self-register here during construction so update()
    // can call tick() on all of them automatically. Supports up to
    // WCB_STREAM_MAX simultaneous streams (e.g. one per Maestro controller).
    static constexpr uint8_t WCB_STREAM_MAX = 4;
    WCBStream* _wcbStreams[WCB_STREAM_MAX] = {};
    uint8_t    _wcbStreamCount = 0;

    // Called by WCBStream's constructor — not intended for direct use.
    // Registers the stream so update() drives it via _processMonitors().
    void _registerWCBStream(WCBStream* stream) {
        if (_wcbStreamCount < WCB_STREAM_MAX)
            _wcbStreams[_wcbStreamCount++] = stream;
    }

    // ── Monitor state ────────────────────────────────────────────────────────

    // Raw binary monitor — buffers bytes from a tapped UART and flushes via
    // sendRaw() when the inter-frame gap elapses.
    HardwareSerial* _monitorRawPort   = nullptr;
    uint8_t         _monitorRawTarget = 0;
    uint8_t         _monitorRawTPort  = 1;   // serial port on the target WCB (1–5)
    uint16_t        _monitorRawGapMs  = 2;
    uint8_t         _monitorRawBuf[177];     // capped at firmware raw chunk limit
    size_t          _monitorRawLen    = 0;
    unsigned long   _monitorRawLastMs = 0;

    // Text serial monitor — accumulates characters and flushes via send() or
    // broadcast() when the terminator character is received.
    HardwareSerial* _monitorSerialPort   = nullptr;
    uint8_t         _monitorSerialTarget = 0;
    char            _monitorSerialTerm   = '\n';
    char            _monitorSerialBuf[200];
    size_t          _monitorSerialLen    = 0;

    // ── Private helpers ──────────────────────────────────────────────────────

    // Populate _wcbMACs[] and _broadcastMAC using the stored oct2/oct3 values.
    // MAC scheme: 02:oct2:oct3:00:00:ID  (ID is 1-based WCB number)
    void _buildMACs();

    // Register each WCB (1–quantity) as an ESP-NOW peer using its pre-computed
    // MAC address, plus the shared broadcast MAC as a peer.
    void _registerPeers();
    void _registerSpecialPeer();   // registers the special peer MAC if enabled + out-of-band

    // Build and broadcast a HEARTBEAT packet so all WCBs know this device is alive.
    void _sendHeartbeat();

    // Send an ACK packet back to the device that sent us a COMMAND.
    // targetID : sender's WCB ID to reply to
    // seqNum   : sequence number from the COMMAND being acknowledged
    void _sendAck(uint8_t targetID, uint16_t seqNum);

    // Build and send a COMMAND packet to targetID (or broadcast if 0).
    // Allocates a sequence number, optionally records the packet in the pending
    // table (always for ensured sends and unicast; never for best-effort
    // broadcast), snapshots the expected-recipient set for ensured sends, then
    // transmits via _transmit().
    bool _sendPacket(uint8_t targetID, const char* command, bool ensured);

    // ── Fragmented send (non-blocking, drained from update()) ───────────────
    // Oversized unicast commands are QUEUED here and transmitted one chunk at
    // a time from update() (~10 ms pacing), never blocking the caller — safe
    // to call from loop() or even the ESP-NOW receive callback. One
    // fragmented send may be in flight at a time; a second call while busy
    // returns false. The full chunk set is repeated (3 passes when ensured,
    // 2 otherwise) to compensate for the lack of per-chunk ACKs; the target
    // dedups chunks and completed sessions, so repeats are harmless.
    struct FragJob {
        bool      active     = false;
        uint8_t   targetWCB  = 0;
        uint16_t  sessionId  = 0;
        uint8_t   totalChunks = 0;
        uint8_t   passes     = 0;     // total passes to transmit
        uint8_t   pass       = 0;     // current pass (0-based)
        uint8_t   chunk      = 0;     // next chunk index within the pass
        uint16_t  acceptedMask = 0;   // bit N set once chunk N was accepted by
                                      // ESP-NOW at least once (any pass)
        unsigned long nextSendMs = 0; // pacing
        char*     cmd        = nullptr; // heap copy of the command
        size_t    len        = 0;
    };
    FragJob _fragJob;

    // Queue an oversized unicast for fragmented transmission. Returns false
    // if the command exceeds WCB_MGMT_MAX_COMMAND_LEN or a fragmented send
    // is already in flight. Returns true = queued (transmission happens in
    // update(); completion/result is reported on Serial).
    bool _sendFragmented(uint8_t target_wcb, const char* command, bool ensured);

    // Transmit the next pending fragment (called once per update()).
    void _processFragJob();

    // Max command chars that fit a single packet given the checksum setting.
    size_t _maxSingleCommandLen() const;

    // Build + transmit a COMMAND packet with an EXPLICIT sequence number — no
    // counter increment, no pending bookkeeping. Used for the initial send and
    // for ensured retransmits, which MUST reuse the original seq so receivers
    // dedup it and returning ACKs still match the pending slot. Appends the
    // CRC32 suffix when _checksumEnabled is true.
    bool _transmit(uint8_t targetID, const char* command, uint16_t seqNum);

    // True when an ensured packet is fully delivered: every board in p.expected[]
    // has either ACK'd or dropped offline (so we no longer wait on it).
    bool _ensuredComplete(const WCBPending& p) const;

    // Scan _boards[] and mark any WCB as offline if its last heartbeat is older
    // than (heartbeatInterval * missedBeforeOffline) seconds.
    void _checkOfflineBoards();

    // Poll both serial monitors on every update() call. Reads available bytes,
    // buffers them, and flushes to the WCB network when a packet boundary is
    // detected (gap for raw, terminator for text).
    void _processMonitors();

    // Process an incoming ESP-NOW packet. Routes to heartbeat, ACK, or command
    // handling based on structPacketType.
    void _handleReceive(const uint8_t* mac, const uint8_t* data, int len);

    // Find an empty slot in _pending[]. Returns index, or -1 if all slots are
    // occupied. Packets are still sent even when -1 is returned — they just
    // won't be tracked for ACK.
    int _findFreePending();

    // CRC32 implementation that matches the WCB firmware calculateCRC32() exactly.
    // Init=0xFFFFFFFF, poly=0xEDB88320 (reflected), final XOR=~crc.
    // Must be identical to the firmware version or checksums won't match.
    uint32_t _crc32(const char* data, size_t len);

    // WCBStream needs access to _registerWCBStream() and sendRaw().
    friend class WCBStream;

    // Singleton pointer — set in the constructor so WCBStream can reach the
    // client without needing an explicit reference passed in by the user.
    // Only one WCB_Client instance is supported per sketch.
    static WCB_Client* _instance;

public:
    // Returns the single WCB_Client instance. Used internally by WCBStream.
    static WCB_Client* instance() { return _instance; }

private:

    // Static bridge required because ESP-NOW's receive callback must be a plain
    // C function (no 'this' pointer). Delegates to _instance->_handleReceive().
    static void _espNowReceiveCB(const esp_now_recv_info_t* info,
                                 const uint8_t* data, int len);
};
