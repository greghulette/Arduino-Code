#include "WCB_Client.h"
#include "WCBStream.h"
#include <esp_wifi.h>

// Singleton instance pointer — allows the static ESP-NOW callback to route
// received packets back to the active WCB_Client object. Only one WCB_Client
// instance is supported per sketch.
WCB_Client* WCB_Client::_instance = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
//
// Stores all network credentials and optional callbacks into private members.
// No hardware is touched here — ESP-NOW and WiFi are not initialised until
// begin() is called. This makes it safe to declare a WCB_Client at global scope
// before the Arduino runtime has started.
// ─────────────────────────────────────────────────────────────────────────────
WCB_Client::WCB_Client(uint8_t mac_oct2, uint8_t mac_oct3,
                     const char* password, uint8_t wcb_quantity, uint8_t device_id,
                     WCBCommandCallback commandCb, WCBStatusCallback statusCb) {
    _oct2     = mac_oct2;
    _oct3     = mac_oct3;
    _quantity = wcb_quantity;
    _deviceID = device_id;
    strncpy(_password, password, sizeof(_password) - 1);
    _password[sizeof(_password) - 1] = '\0';
    _commandCallback = commandCb;
    _statusCallback  = statusCb;
    // Set singleton immediately so WCBStream objects declared at global scope
    // (after this WCB_Client) can self-register without needing a reference passed in.
    _instance = this;
}

// ─────────────────────────────────────────────────────────────────────────────
// begin
//
// Validates configuration, sets the custom WiFi MAC address, initialises
// ESP-NOW, registers all WCB peers, and arms the heartbeat timer.
//
// Must be called once from setup() after Serial.begin().
// Returns true on success, false if validation or ESP-NOW init fails.
//
// Why we set a custom MAC:
//   Every WCB derives every peer's MAC from the shared oct2/oct3 and their ID.
//   If this device uses a random factory MAC the WCBs won't have it registered
//   and will silently drop all packets from it. Setting the MAC to the expected
//   scheme (02:oct2:oct3:00:00:ID) makes this device look identical to a real
//   WCB at the hardware level.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::begin() {

    // ── Validate device_id ───────────────────────────────────────────────────
    // device_id must be 1–WCB_MAX_BOARDS. If it's not the special slot (20)
    // it must also be within the declared network size so the WCBs already have
    // this MAC pre-registered in their peer tables.
    if (_deviceID == 0 || _deviceID > WCB_MAX_BOARDS) {
        Serial.printf("[WCB_Client] ERROR: device_id %d is out of range (1–%d)\n",
                      _deviceID, WCB_MAX_BOARDS);
        return false;
    }
    if (_deviceID != WCB_SPECIAL_ID && _deviceID > _quantity) {
        Serial.printf("[WCB_Client] ERROR: device_id %d exceeds wcb_quantity %d. "
                      "Use device_id <= quantity or device_id = %d for the special slot.\n",
                      _deviceID, _quantity, WCB_SPECIAL_ID);
        return false;
    }

    // ── Reset internal state ─────────────────────────────────────────────────
    _seqCounter      = 0;
    _nextHeartbeatMs = 0;                          // Triggers heartbeat immediately on
                                                   // first update() call
    memset(_boards,  0, sizeof(_boards));
    memset(_pending, 0, sizeof(_pending));

    // ── WiFi setup ───────────────────────────────────────────────────────────
    // ESP-NOW requires WiFi to be in station mode. We disconnect from any AP
    // so no association overhead interferes with ESP-NOW timing.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // ── Build MAC table ──────────────────────────────────────────────────────
    // Must happen BEFORE esp_wifi_set_mac() so _wcbMACs[] is populated.
    _buildMACs();

    // ── Set custom MAC address ───────────────────────────────────────────────
    // Set this ESP32's WiFi MAC to the WCB network scheme so other WCBs
    // recognise it as a valid peer. Without this, every packet sent by this
    // device arrives from an unknown MAC and is dropped.
    if (esp_wifi_set_mac(WIFI_IF_STA, _wcbMACs[_deviceID - 1]) != ESP_OK) {
        // Non-fatal — some ESP32 variants restrict MAC changes. Packets may
        // still be received, but WCBs will not accept incoming packets from us.
        Serial.printf("[WCB_Client] WARNING: could not set custom MAC for device ID %d\n",
                      _deviceID);
    } else {
        Serial.printf("[WCB_Client] MAC set to %02X:%02X:%02X:%02X:%02X:%02X\n",
                      _wcbMACs[_deviceID-1][0], _wcbMACs[_deviceID-1][1],
                      _wcbMACs[_deviceID-1][2], _wcbMACs[_deviceID-1][3],
                      _wcbMACs[_deviceID-1][4], _wcbMACs[_deviceID-1][5]);
    }

    // ── Initialise ESP-NOW ───────────────────────────────────────────────────
    if (esp_now_init() != ESP_OK) {
        Serial.println("[WCB_Client] ERROR: esp_now_init() failed");
        return false;
    }

    // Store the instance pointer and register the receive callback.
    // The callback is a static function; it uses _instance to call the
    // non-static _handleReceive() on the correct object.
    _instance = this;
    esp_now_register_recv_cb(_espNowReceiveCB);

    // ── Register peers ───────────────────────────────────────────────────────
    // Pre-register every WCB MAC and the broadcast MAC as ESP-NOW peers.
    // ESP-NOW requires a peer to be registered before you can send to it.
    _registerPeers();

    Serial.printf("[WCB_Client] Joined WCB network as device ID %d "
                  "(quantity=%d, oct2=0x%02X, oct3=0x%02X)\n",
                  _deviceID, _quantity, _oct2, _oct3);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// update
//
// Call every iteration of loop(). Performs two tasks:
//
//   1. Heartbeat  — fires a broadcast HEARTBEAT packet at the configured
//                   interval so all WCBs know this device is still alive.
//                   WCBs that stop seeing heartbeats will mark this device
//                   offline and may stop routing messages to it.
//
//   2. Offline detection — scans the board status table and marks any WCB
//                          offline if its last heartbeat is older than
//                          (heartbeatIntervalSec * missedBeforeOffline) seconds.
//                          Fires the status callback when a transition occurs.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::update() {
    unsigned long now = millis();

    if (now >= _nextHeartbeatMs) {
        _sendHeartbeat();
        _nextHeartbeatMs = now + (_heartbeatIntervalSec * 1000UL);
    }

    _checkOfflineBoards();
    _processMonitors();

    // Service the pending table.
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        WCBPending& p = _pending[i];
        if (!p.active) continue;

        if (p.ensured) {
            // ── Ensured delivery: PER-BOARD UNICAST retries ──────────────────
            // Mirrors the WCB firmware's processETMAcksAndRetries() exactly:
            // after the initial send, retry as a UNICAST to each expected board
            // that hasn't ACK'd (reusing the original seq so it's deduped),
            // up to ETM_MAX_RETRIES per board; drop a board that goes offline.
            // (An ensured UNICAST is normally freed by the ACK handler the
            //  instant its target ACKs; reaching here means it hasn't yet.)
            if (_ensuredComplete(p)) {            // every expected board acked / offline
                p.active = false;
                continue;
            }
            if ((now - p.sentMs) >= ETM_RETRY_INTERVAL_MS) {
                for (int b = 0; b < WCB_MAX_BOARDS; b++) {
                    if (!p.expected[b] || p.ackReceived[b]) continue;
                    if (!_boards[b].online) {     // board gone — stop waiting on it
                        p.expected[b] = false;
                        continue;
                    }
                    if (p.retryCount[b] < ETM_MAX_RETRIES) {
                        _transmit((uint8_t)(b + 1), p.command, p.seqNum);  // unicast retry
                        p.retryCount[b]++;
                    } else {
                        p.expected[b] = false;    // exhausted retries for this board
                    }
                }
                p.sentMs = now;                   // reset the retry window
                if (_ensuredComplete(p)) p.active = false;   // nothing left outstanding
            }
            continue;
        }

        // ── Best-effort unicast: reclaim backstop ────────────────────────────
        // A best-effort (ensured=false) unicast is sent ONCE — there is no
        // retransmit. The slot is normally freed by the ACK handler on the
        // target's ACK; this just frees one whose ACK never arrived so the
        // table can't leak. 1 s is well beyond a normal ETM ACK round-trip.
        if ((now - p.sentMs) > 1000UL) p.active = false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// send
//
// Send a text command to one specific WCB.
// Internally calls _sendPacket() which handles CRC appending and ACK tracking.
// Commands longer than one packet are fragmented automatically (1.3.0) —
// previously they were silently truncated by strncpy, so a long ?SEQ,SAVE
// arrived cut short and a corrupt sequence got stored on the board.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::send(uint8_t target_wcb, const char* command, bool ensured) {
    if (target_wcb < 1 || target_wcb > WCB_MAX_BOARDS) return false;
    if (!command) return false;
    if (strlen(command) > _maxSingleCommandLen()) {
        return _sendFragmented(target_wcb, command, ensured);
    }
    return _sendPacket(target_wcb, command, ensured);
}

// ─────────────────────────────────────────────────────────────────────────────
// broadcast
//
// Send a text command to ALL WCBs simultaneously via the shared broadcast MAC.
// Every WCB on the network receives and processes the same packet.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::broadcast(const char* command, bool ensured) {
    if (!command) return false;
    // Fragmentation is unicast-only (the target-side reassembly session is
    // per-board) — an oversized broadcast must fail LOUDLY, never truncate.
    if (strlen(command) > _maxSingleCommandLen()) {
        Serial.printf("[WCB_Client] broadcast: command too long (%u > %u chars) — "
                      "fragmentation is unicast-only. send() it to each board instead.\n",
                      (unsigned)strlen(command), (unsigned)_maxSingleCommandLen());
        return false;
    }
    return _sendPacket(WCB_TARGET_BROADCAST, command, ensured);
}

// ─────────────────────────────────────────────────────────────────────────────
// _maxSingleCommandLen
//
// Longest command that fits ONE packet: structCommand is 200 bytes (199 + NUL);
// when checksum is enabled the "|CRCxxxxxxxx" suffix (12 chars) shares it.
// ─────────────────────────────────────────────────────────────────────────────
size_t WCB_Client::_maxSingleCommandLen() const {
    return _checksumEnabled ? (199 - 12) : 199;
}

// ─────────────────────────────────────────────────────────────────────────────
// _sendFragmented
//
// Deliver an oversized UNICAST command via the WCB firmware's MGMT
// fragmentation protocol (the same one the web config tool uses):
//   • split into WCB_MGMT_CHUNK_LEN-char chunks (max WCB_MGMT_MAX_CHUNKS)
//   • broadcast each chunk as a wcb_packet_mgmt_t (226 bytes — the firmware
//     dispatches by size); only the addressed targetWCB reassembles it
//   • when all chunks arrive, the target executes the WHOLE command through
//     its normal parser (chained ^ commands split correctly)
//
// Reliability: the MGMT layer has NO per-chunk ACK, so the full chunk set is
// transmitted in multiple passes (3 when ensured, 2 otherwise). Duplicates
// are harmless — the target stores each chunk once and remembers completed
// sessionIds, so repeat passes after completion are silently discarded.
//
// Blocking: ~10 ms per chunk per pass (a full 16-chunk ensured send ≈ 0.5 s).
// The target's reassembly window is 15 s — passes land well inside it.
//
// Returns true if every chunk of at least one full pass was accepted by
// ESP-NOW for transmission. NOTE: this is acceptance, not delivery — there
// is no ACK at this layer (use a short command for anything you must confirm).
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_sendFragmented(uint8_t target_wcb, const char* command, bool ensured) {
    size_t len = strlen(command);
    uint8_t totalChunks = (uint8_t)((len + WCB_MGMT_CHUNK_LEN - 1) / WCB_MGMT_CHUNK_LEN);
    if (totalChunks > WCB_MGMT_MAX_CHUNKS) {
        Serial.printf("[WCB_Client] send: command too long even for fragmentation "
                      "(%u chars > %u max). Not sent.\n",
                      (unsigned)len, (unsigned)(WCB_MGMT_CHUNK_LEN * WCB_MGMT_MAX_CHUNKS));
        return false;
    }

    // Fresh sessionId per command. 0xFFFF is reserved (the firmware's
    // completed-session ring buffer initialises to 0xFFFF, which would make
    // the very first session look like a duplicate and be discarded).
    uint16_t sessionId = (uint16_t)esp_random();
    if (sessionId == 0xFFFF) sessionId = 0xFFFE;

    wcb_packet_mgmt_t pkt;
    const int passes = ensured ? 3 : 2;
    bool anyPassClean = false;

    Serial.printf("[WCB_Client] send: %u chars > single-packet limit — fragmenting "
                  "to WCB%d (%d chunks, session %04X, %d passes)\n",
                  (unsigned)len, target_wcb, totalChunks, sessionId, passes);

    for (int pass = 0; pass < passes; pass++) {
        bool passClean = true;
        for (uint8_t i = 0; i < totalChunks; i++) {
            memset(&pkt, 0, sizeof(pkt));
            strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
            pkt.packetType  = WCB_MGMT_PACKET_TYPE_FRAG;
            pkt.targetWCB   = target_wcb;
            pkt.sessionId   = sessionId;
            pkt.chunkIdx    = i;
            pkt.totalChunks = totalChunks;

            size_t off = (size_t)i * WCB_MGMT_CHUNK_LEN;
            size_t n   = len - off;
            if (n > WCB_MGMT_CHUNK_LEN) n = WCB_MGMT_CHUNK_LEN;
            memcpy(pkt.payload, command + off, n);   // payload[180] zeroed above → NUL-safe

            if (esp_now_send(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt)) != ESP_OK) {
                passClean = false;
            }
            delay(10);   // pacing: don't flood the ESP-NOW TX queue
        }
        if (passClean) anyPassClean = true;
    }

    if (!anyPassClean) {
        Serial.printf("[WCB_Client] send: fragmented send to WCB%d failed — "
                      "no complete pass was accepted by ESP-NOW\n", target_wcb);
    }
    return anyPassClean;
}

// ─────────────────────────────────────────────────────────────────────────────
// sendRaw
//
// Unicast raw binary data to one specific WCB for forwarding to its serial port.
//
// How it works:
//   The target ID in the packet is set to WCB_TARGET_RAW_SERIAL (97).
//   When the receiving WCB sees this target ID it writes the raw bytes directly
//   to the specified serial port. No text parsing, no CRC — byte-for-byte.
//
// Use case — Pololu / Maestro (known target):
//   You know exactly which WCB has the servo controller wired to which port.
//   Build the binary command (e.g. a Maestro set-target packet) and call
//   sendRaw() to deliver it wirelessly as if this ESP32 were wired directly.
//
// Requirements:
//   - The target WCB just needs the servo controller wired to target_port.
//   - No KYBER,REMOTE config is required on the receiving WCB.
//   - data length is capped at 177 bytes (firmware raw-chunk limit).
//   - No CRC is added — the data is treated as opaque binary.
//
// See also: sendKyber() for broadcasting to ALL WCBs with Maestros at once.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::sendRaw(uint8_t target_wcb, uint8_t target_port,
                        const uint8_t* data, size_t len) {
    if (target_wcb < 1 || target_wcb > WCB_MAX_BOARDS) return false;
    if (target_port < 1 || target_port > 5) return false;
    if (len == 0) return false;

    // WCB firmware caps raw chunks at 177 bytes — clip silently
    if (len > 177) len = 177;

    // Raw serial packets MUST use the non-ETM struct (wcb_packet_t, 249 bytes).
    // The WCB firmware routes incoming packets by size: 252 bytes → ETM path,
    // 249 bytes → standard path. The raw serial handler (targetID=97) only
    // exists in the standard path — ETM-sized raw packets are dropped.
    // This matches exactly how WCB firmware sends raw serial via sendESPNowRawSerial().
    wcb_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);

    // Target ID 97 tells the receiving WCB to forward raw bytes to a serial port
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "%d", WCB_TARGET_RAW_SERIAL);

    pkt.structCommandIncluded = 1;

    // WCB firmware expects a 3-byte header before the data:
    //   [0]   target_port  — which serial port on the WCB (1–5)
    //   [1]   len low byte — little-endian uint16 length
    //   [2]   len high byte
    //   [3..] data bytes
    pkt.structCommand[0] = target_port;
    pkt.structCommand[1] = (uint8_t)(len & 0xFF);
    pkt.structCommand[2] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(pkt.structCommand + 3, data, len);

    return esp_now_send(_wcbMACs[target_wcb - 1],
                        (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// sendKyber
//
// Broadcast raw binary data to ALL WCBs on the network via the Kyber path.
//
// How it works:
//   The packet uses WCB_TARGET_KYBER (98) with the NON-ETM packet struct
//   (wcb_packet_t / 249 bytes) — see the long note in the function body for
//   why the ETM struct does not work here.
//   Any WCB that has Kyber_Remote enabled receives the broadcast and writes the
//   bytes to its locally wired Maestro serial port(s). WCBs without Kyber_Remote
//   ignore it. This mirrors exactly what the WCB firmware does when a physical
//   Kyber device is connected — no targeted addressing needed.
//
// Header format inside structCommand:
//   [0]   len low byte   — little-endian uint16 length of the data
//   [1]   len high byte
//   [2..] data bytes
// sendKyber() builds this header automatically.
//
// Use case — Maestro broadcast (multiple WCBs with Maestros):
//   You want the same Maestro command to reach every board with a servo
//   controller, or you don't know which WCB has the Maestro. One broadcast
//   packet reaches all of them in one ESP-NOW send.
//
// data : pointer to the binary byte array (Maestro/Pololu command bytes)
// len  : number of bytes (max 178 — 180 structCommand bytes minus 2-byte header)
// Returns true if ESP-NOW accepted the packet for transmission.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::sendKyber(const uint8_t* data, size_t len) {
    if (len == 0) return false;
    // Firmware caps Kyber chunks at 178 bytes (structCommand[180] - 2-byte header)
    if (len > 178) len = 178;

    // Kyber broadcast MUST use the non-ETM struct (wcb_packet_t, 249 bytes) —
    // NOT the ETM struct.  The WCB firmware routes incoming ESP-NOW packets by
    // size: 252-byte packets go down the ETM path, 249-byte packets down the
    // standard path.  The Kyber-broadcast handler (targetID==98) only exists in
    // the *standard* path, and the standard path has an explicit Kyber bypass
    // so a 249-byte targetID-98 frame falls straight through to it and is
    // written to every WCB's local Maestro port(s) — true one-send broadcast.
    //
    // An ETM-sized (252-byte) Kyber packet never reaches that handler: the ETM
    // path treats it as a PACKET_TYPE_COMMAND and drops it at the target gate
    // (targetID 98 is neither 0/broadcast nor this board's ID) before any Kyber
    // processing.  That made Kyber broadcast silently fail on every ETM-enabled
    // WCB (the default).  Using wcb_packet_t here is what the WCB firmware and
    // this library's own WCB_TARGET_KYBER header doc actually expect.
    wcb_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "%d", WCB_TARGET_KYBER);
    pkt.structCommandIncluded = 1;

    // 2-byte little-endian length header, then data
    pkt.structCommand[0] = (uint8_t)(len & 0xFF);
    pkt.structCommand[1] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(pkt.structCommand + 2, data, len);

    return esp_now_send(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// monitorRaw
//
// Registers a UART to watch for raw binary output from an attached device
// (e.g. a Pololu/Maestro servo controller driven by another library writing
// to a serial port on this same ESP32).
//
// Wiring: connect the source UART's TX pin to an unused UART's RX pin on
// this ESP32. Pass the receiving UART here. The library reads bytes as they
// arrive, buffers them, and sends via sendRaw() when the inter-frame gap
// (gap_ms of silence) signals a packet boundary.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::monitorRaw(HardwareSerial& port, uint8_t target_wcb,
                           uint8_t target_port, uint16_t gap_ms) {
    _monitorRawPort   = &port;
    _monitorRawTarget = target_wcb;
    _monitorRawTPort  = target_port;
    _monitorRawGapMs  = gap_ms;
    _monitorRawLen    = 0;
    _monitorRawLastMs = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// monitorSerial
//
// Registers a UART to watch for text command lines output by an attached
// device. Each line terminated by 'terminator' is forwarded to target_wcb
// via send() (or broadcast() if target_wcb == WCB_TARGET_BROADCAST).
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::monitorSerial(HardwareSerial& port, uint8_t target_wcb, char terminator) {
    _monitorSerialPort   = &port;
    _monitorSerialTarget = target_wcb;
    _monitorSerialTerm   = terminator;
    _monitorSerialLen    = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// _processMonitors
//
// Called by update() every loop iteration. Polls both serial monitors.
//
// Raw monitor:
//   Reads all available bytes into the buffer. When no new bytes have arrived
//   for _monitorRawGapMs milliseconds the buffer is considered a complete
//   packet and flushed via sendRaw(). The gap threshold is the key: Pololu
//   Mini SSC packets are 3 bytes; at 9600 baud that's ~0.3ms per packet.
//   A 2ms gap reliably falls between packets, not mid-packet.
//
// Text monitor:
//   Reads characters one by one. When the terminator is received the
//   accumulated string is dispatched via send() or broadcast() and the
//   buffer resets. Lines longer than 199 chars are silently truncated.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_processMonitors() {
    unsigned long now = millis();

    // ── WCBStream flush ───────────────────────────────────────────────────────
    // Tick all registered WCBStream instances (one per Maestro controller).
    // tick() checks the inter-frame gap and calls sendRaw() if a complete
    // packet has accumulated since the last write().
    for (uint8_t i = 0; i < _wcbStreamCount; i++)
        _wcbStreams[i]->tick();

    // ── Raw binary monitor ────────────────────────────────────────────────────
    if (_monitorRawPort) {
        // Drain all available bytes into the buffer
        while (_monitorRawPort->available() && _monitorRawLen < sizeof(_monitorRawBuf)) {
            _monitorRawBuf[_monitorRawLen++] = (uint8_t)_monitorRawPort->read();
            _monitorRawLastMs = now;
        }
        // Flush when we have data and the inter-frame gap has elapsed.
        // target == 0 means Kyber broadcast to all WCBs; otherwise unicast.
        if (_monitorRawLen > 0 && (now - _monitorRawLastMs) >= _monitorRawGapMs) {
            if (_monitorRawTarget == 0)
                sendKyber(_monitorRawBuf, _monitorRawLen);
            else
                sendRaw(_monitorRawTarget, _monitorRawTPort, _monitorRawBuf, _monitorRawLen);
            _monitorRawLen = 0;
        }
    }

    // ── Text serial monitor ───────────────────────────────────────────────────
    if (_monitorSerialPort) {
        while (_monitorSerialPort->available()) {
            char c = (char)_monitorSerialPort->read();
            if (c == _monitorSerialTerm) {
                // Terminator received — dispatch whatever is in the buffer
                if (_monitorSerialLen > 0) {
                    _monitorSerialBuf[_monitorSerialLen] = '\0';
                    if (_monitorSerialTarget == WCB_TARGET_BROADCAST)
                        broadcast(_monitorSerialBuf);
                    else
                        send(_monitorSerialTarget, _monitorSerialBuf);
                    _monitorSerialLen = 0;
                }
            } else if (_monitorSerialLen < sizeof(_monitorSerialBuf) - 1) {
                _monitorSerialBuf[_monitorSerialLen++] = c;
            }
            // Bytes past the buffer limit are silently dropped
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// isOnline
//
// Returns true if the specified WCB has sent a heartbeat recently enough to
// be considered online. Useful for guarding send() calls so you don't fire
// commands at a board that has dropped off the network.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::isOnline(uint8_t wcb_number) {
    if (wcb_number < 1 || wcb_number > WCB_MAX_BOARDS) return false;
    return _boards[wcb_number - 1].online;
}

// ─────────────────────────────────────────────────────────────────────────────
// onCommand / onStatusChange / setChecksum
// ─────────────────────────────────────────────────────────────────────────────

void WCB_Client::onCommand(WCBCommandCallback callback) {
    _commandCallback = callback;
}

void WCB_Client::onStatusChange(WCBStatusCallback callback) {
    _statusCallback = callback;
}

// Enable or disable CRC32 checksum. Must match the WCB network's ?ETM,CHKSM setting.
void WCB_Client::setChecksum(bool enabled) {
    _checksumEnabled = enabled;
}

// =============================================================================
// Private helpers
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// _buildMACs
//
// Computes the MAC address for every possible WCB slot (1–WCB_MAX_BOARDS)
// and for the network broadcast address using the stored oct2/oct3 values.
//
// WCB MAC scheme:
//   02 : locally administered, unicast bit set — required for custom MACs
//   oct2, oct3 : shared octets that identify this WCB network
//   00, 00 : reserved / always zero
//   ID : WCB number (1-based, stored in the last octet)
//
// Broadcast MAC:
//   FF:oct2:oct3:FF:FF:FF
//   The non-standard broadcast (not FF:FF:FF:FF:FF:FF) scopes the broadcast
//   to this network's oct2/oct3, so different WCB networks on the same channel
//   don't interfere with each other.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_buildMACs() {
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        _wcbMACs[i][0] = 0x02;
        _wcbMACs[i][1] = _oct2;
        _wcbMACs[i][2] = _oct3;
        _wcbMACs[i][3] = 0x00;
        _wcbMACs[i][4] = 0x00;
        _wcbMACs[i][5] = (uint8_t)(i + 1);  // WCB number, 1-based
    }

    _broadcastMAC[0] = 0xFF;
    _broadcastMAC[1] = _oct2;
    _broadcastMAC[2] = _oct3;
    _broadcastMAC[3] = 0xFF;
    _broadcastMAC[4] = 0xFF;
    _broadcastMAC[5] = 0xFF;
}

// ─────────────────────────────────────────────────────────────────────────────
// _registerPeers
//
// Adds each WCB (1 through wcb_quantity) and the broadcast MAC as ESP-NOW
// peers. ESP-NOW requires peers to be pre-registered before you can send to
// them; receiving does not require peer registration.
//
// This device's own slot is skipped (you can't send to yourself).
// The special slot (ID 20) outside the 1–quantity range is not registered here
// because WCBs with specialPeerEnabled already have that MAC registered on
// their side, allowing bidirectional communication.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_registerPeers() {
    for (int i = 0; i < _quantity; i++) {
        if ((i + 1) == _deviceID) continue;  // Skip our own slot

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, _wcbMACs[i], 6);
        peer.channel = 0;       // 0 = use the current WiFi channel
        peer.encrypt = false;   // WCB network uses password auth, not ESP-NOW encryption
        esp_now_add_peer(&peer);
    }

    // Register the broadcast MAC so broadcast() can send via esp_now_send()
    esp_now_peer_info_t bcast = {};
    memcpy(bcast.peer_addr, _broadcastMAC, 6);
    bcast.channel = 0;
    bcast.encrypt = false;
    esp_now_add_peer(&bcast);
}

// ─────────────────────────────────────────────────────────────────────────────
// _sendHeartbeat
//
// Broadcasts a HEARTBEAT packet to the network. Heartbeats carry no command
// payload (structCommandIncluded = 0); they exist purely to tell every other
// WCB "I am alive." WCBs that don't see a heartbeat within the expected window
// will mark this device offline and may stop routing messages to it.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_sendHeartbeat() {
    wcb_packet_etm_t hb;
    memset(&hb, 0, sizeof(hb));

    strncpy(hb.structPassword, _password, sizeof(hb.structPassword) - 1);
    snprintf(hb.structSenderID, sizeof(hb.structSenderID), "%d", _deviceID);
    snprintf(hb.structTargetID, sizeof(hb.structTargetID), "0");  // 0 = broadcast
    hb.structCommandIncluded = 0;         // No command payload
    hb.structPacketType      = WCB_PACKET_HEARTBEAT;
    hb.structSequenceNumber  = 0;         // Heartbeats don't use sequence numbers

    esp_now_send(_broadcastMAC, (uint8_t*)&hb, sizeof(hb));
}

// ─────────────────────────────────────────────────────────────────────────────
// _sendAck
//
// Sends a WCB_PACKET_ACK back to the device that sent us a COMMAND.
// The ACK carries the same sequence number as the original COMMAND so the
// sender can match it to the correct pending slot and stop retrying.
//
// ACKs have no command payload (structCommandIncluded = 0).
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_sendAck(uint8_t targetID, uint16_t seqNum) {
    if (targetID < 1 || targetID > WCB_MAX_BOARDS) return;

    wcb_packet_etm_t ack;
    memset(&ack, 0, sizeof(ack));

    strncpy(ack.structPassword, _password, sizeof(ack.structPassword) - 1);
    snprintf(ack.structSenderID, sizeof(ack.structSenderID), "%d", _deviceID);
    snprintf(ack.structTargetID, sizeof(ack.structTargetID), "%d", targetID);
    ack.structCommandIncluded = 0;
    ack.structPacketType      = WCB_PACKET_ACK;
    ack.structSequenceNumber  = seqNum;  // Echo the original sequence number

    esp_now_send(_wcbMACs[targetID - 1], (uint8_t*)&ack, sizeof(ack));
}

// ─────────────────────────────────────────────────────────────────────────────
// _sendPacket
//
// Entry point for send() and broadcast(). Allocates the sequence number, sets
// up pending-table tracking (and, for ensured sends, snapshots the expected
// recipient set), then hands the actual packet build + transmit to _transmit().
// The CRC suffix and wire framing live in _transmit() so retransmits reuse it.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_sendPacket(uint8_t targetID, const char* command, bool ensured) {
    uint16_t seq = ++_seqCounter;   // atomic; never reused by a retransmit

    // Decide whether to record this packet in the pending table:
    //   - unicast            → tracked (ACK info; freed on the target's ACK).
    //   - ensured broadcast  → tracked (update() retransmits until complete).
    //   - best-effort bcast  → NOT tracked. Its ACK slot would never free (the
    //                          ACK handler keeps broadcast slots open to collect
    //                          multi-board ACKs), so tracking an un-ensured one
    //                          would just leak the small WCB_PENDING_MAX table.
    bool track = ensured || (targetID != WCB_TARGET_BROADCAST);
    int  slot  = track ? _findFreePending() : -1;
    if (slot >= 0) {
        WCBPending& p = _pending[slot];
        p.active   = true;
        p.seqNum   = seq;
        p.sentMs   = millis();
        p.targetID = targetID;
        p.ensured  = ensured;
        strncpy(p.command, command, sizeof(p.command) - 1);
        p.command[sizeof(p.command) - 1] = '\0';   // strncpy may not NUL-terminate
        memset(p.ackReceived, 0, sizeof(p.ackReceived));
        memset(p.expected,    0, sizeof(p.expected));
        memset(p.retryCount,  0, sizeof(p.retryCount));
        if (ensured) {
            // Snapshot who we'll wait for. For a broadcast that's every board we
            // currently believe is online (except ourself); for a unicast it's
            // just the target. A board that later drops offline stops blocking
            // completion (see _ensuredComplete), so we never retry forever to a
            // board that's simply gone.
            if (targetID == WCB_TARGET_BROADCAST) {
                for (int b = 0; b < WCB_MAX_BOARDS; b++)
                    if (_boards[b].online && (b + 1) != _deviceID)
                        p.expected[b] = true;
            } else {
                p.expected[targetID - 1] = true;
            }
        }
    }

    return _transmit(targetID, command, seq);
}

// ─────────────────────────────────────────────────────────────────────────────
// _transmit
//
// Build a COMMAND packet with the GIVEN sequence number and hand it to ESP-NOW.
// No _seqCounter increment and no pending bookkeeping — this is the shared
// transmit used by both the initial send (_sendPacket) and ensured retransmits
// (update()), which must reuse the original seq so receivers dedup the resend
// and their ACKs still match the pending slot.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_transmit(uint8_t targetID, const char* command, uint16_t seqNum) {
    wcb_packet_etm_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "%d", targetID);
    pkt.structCommandIncluded = 1;

    // Append CRC32 checksum if enabled — must match ?ETM,CHKSM setting on WCBs.
    if (_checksumEnabled) {
        uint32_t crc = _crc32(command, strlen(command));
        snprintf(pkt.structCommand, sizeof(pkt.structCommand),
                 "%s|CRC%08X", command, crc);
    } else {
        strncpy(pkt.structCommand, command, sizeof(pkt.structCommand) - 1);
    }
    pkt.structCommand[sizeof(pkt.structCommand) - 1] = '\0';

    pkt.structPacketType     = WCB_PACKET_COMMAND;
    pkt.structSequenceNumber = seqNum;

    uint8_t* mac = (targetID == WCB_TARGET_BROADCAST)
                   ? _broadcastMAC
                   : _wcbMACs[targetID - 1];

    return esp_now_send(mac, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// _ensuredComplete
//
// An ensured packet is "done" once nothing in its expected set is still
// outstanding — i.e. every expected board has either ACK'd this sequence or
// gone offline (so we stop waiting on a board that's simply no longer there).
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_ensuredComplete(const WCBPending& p) const {
    for (int b = 0; b < WCB_MAX_BOARDS; b++) {
        if (p.expected[b] && !p.ackReceived[b] && _boards[b].online)
            return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// _checkOfflineBoards
//
// Scans the board status table on every update() call. If a WCB's last
// heartbeat timestamp is older than (heartbeatIntervalSec * missedBeforeOffline)
// seconds it is marked offline and the status callback is fired.
//
// The threshold matches the WCB firmware's own ETM offline detection logic so
// both sides agree on when a board should be considered gone.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_checkOfflineBoards() {
    unsigned long now       = millis();
    unsigned long threshold = (unsigned long)_heartbeatIntervalSec
                              * _missedBeforeOffline * 1000UL;

    for (int i = 0; i < _quantity; i++) {
        if (!_boards[i].online) continue;
        if (now - _boards[i].lastSeenMs > threshold) {
            _boards[i].online = false;
            if (_statusCallback) _statusCallback(i + 1, false);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// _findFreePending
//
// Returns the index of a usable slot in _pending[]. Prefers the first inactive
// slot. If all WCB_PENDING_MAX slots are occupied, evicts the OLDEST one (by
// sentMs) and returns it — same policy as the WCB firmware's pending table.
//
// Eviction (rather than the old "return -1, send untracked") matters now that
// ensured delivery is the default: a brand-new ensured command must get a slot
// so it can be retried, and the oldest outstanding entry is the safest to drop
// (it has had the most time to be delivered + ACK'd at the MAC layer already).
// Always returns a valid index in [0, WCB_PENDING_MAX).
// ─────────────────────────────────────────────────────────────────────────────
int WCB_Client::_findFreePending() {
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        if (!_pending[i].active) return i;
    }
    // All slots busy — evict the one with the greatest age. Age is computed as
    // (now - sentMs) with unsigned arithmetic, so it stays correct across a
    // millis() rollover (entries only live for ms, far under the ~49.7-day
    // wrap period). The oldest entry is the safest to drop: it has had the most
    // time to be delivered and MAC-layer-retried already.
    unsigned long now    = millis();
    int           oldest = 0;
    unsigned long maxAge = now - _pending[0].sentMs;
    for (int i = 1; i < WCB_PENDING_MAX; i++) {
        unsigned long age = now - _pending[i].sentMs;
        if (age > maxAge) { maxAge = age; oldest = i; }
    }
    _pending[oldest].active = false;
    return oldest;
}

// ─────────────────────────────────────────────────────────────────────────────
// _handleReceive
//
// Central receive handler, called by the static ESP-NOW callback for every
// incoming packet. Routes by packet type:
//
//   HEARTBEAT : update the sender's online status and timestamp
//   ACK       : find the matching pending slot and mark it acknowledged
//   COMMAND   : verify checksum, send ACK back, deliver to user callback
//
// Packets are silently dropped if:
//   - They are shorter than wcb_packet_etm_t (incomplete packet)
//   - The structPassword doesn't match our network password
//   - They are not addressed to this device or broadcast
//   - The CRC32 is wrong or missing (when _checksumEnabled is true)
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_handleReceive(const uint8_t* mac, const uint8_t* data, int len) {

    // Ignore undersized packets — they can't be a valid wcb_packet_etm_t
    if (len < (int)sizeof(wcb_packet_etm_t)) return;

    const wcb_packet_etm_t* pkt = (const wcb_packet_etm_t*)data;

    // Password check — reject packets from other WCB networks on the same channel
    if (strncmp(pkt->structPassword, _password, sizeof(pkt->structPassword) - 1) != 0)
        return;

    int senderID = atoi(pkt->structSenderID);
    int targetID = atoi(pkt->structTargetID);

    // Only process packets addressed to us or to everyone
    if (targetID != WCB_TARGET_BROADCAST && targetID != _deviceID) return;

    switch (pkt->structPacketType) {

        // ── Heartbeat ────────────────────────────────────────────────────────
        // Update the sender's last-seen timestamp. If the board was previously
        // offline, mark it online and fire the status callback.
        case WCB_PACKET_HEARTBEAT:
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS) {
                int  idx        = senderID - 1;
                bool wasOffline = !_boards[idx].online;
                _boards[idx].online     = true;
                _boards[idx].lastSeenMs = millis();
                if (wasOffline && _statusCallback)
                    _statusCallback((uint8_t)senderID, true);
            }
            break;

        // ── ACK ──────────────────────────────────────────────────────────────
        // Find the pending slot whose sequence number matches this ACK and
        // record that the sender has acknowledged it. For unicast packets the
        // slot is freed immediately; for broadcasts the slot stays open because
        // ACKs may arrive from multiple boards.
        case WCB_PACKET_ACK:
            for (int i = 0; i < WCB_PENDING_MAX; i++) {
                if (!_pending[i].active) continue;
                if (_pending[i].seqNum != pkt->structSequenceNumber) continue;

                if (senderID >= 1 && senderID <= WCB_MAX_BOARDS)
                    _pending[i].ackReceived[senderID - 1] = true;

                if (_pending[i].targetID != WCB_TARGET_BROADCAST)
                    _pending[i].active = false;

                break;
            }
            break;

        // ── Command ──────────────────────────────────────────────────────────
        // 1. Send an ACK so the sender stops retrying.
        // 2. Verify the CRC32 checksum (if enabled); reject on mismatch or missing.
        // 3. Strip the "|CRC..." suffix so the application sees a clean string.
        // 4. Deliver the command string to the user callback.
        case WCB_PACKET_COMMAND:

            // Always ACK, even if we end up rejecting the packet for a bad CRC.
            // This prevents the sender from flooding retries at us.
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS)
                _sendAck((uint8_t)senderID, pkt->structSequenceNumber);

            if (pkt->structCommandIncluded && _commandCallback) {
                char cmd[201];
                strncpy(cmd, pkt->structCommand, sizeof(cmd) - 1);
                cmd[sizeof(cmd) - 1] = '\0';

                char* crcTag = strstr(cmd, "|CRC");
                if (crcTag) {
                    // CRC suffix found — verify it before delivering
                    uint32_t rxCRC   = (uint32_t)strtoul(crcTag + 4, nullptr, 16);
                    uint32_t calcCRC = _crc32(cmd, (size_t)(crcTag - cmd));
                    if (rxCRC != calcCRC) {
                        Serial.printf("[WCB_Client] CRC mismatch from WCB%d — packet rejected\n",
                                      senderID);
                        break;
                    }
                    *crcTag = '\0';  // Strip "|CRC..." — deliver clean command only
                } else if (_checksumEnabled) {
                    // We expect a checksum but none was present — likely a mismatch
                    // in ?ETM,CHKSM settings between this device and the sender.
                    Serial.printf("[WCB_Client] Missing CRC from WCB%d — packet rejected\n",
                                  senderID);
                    break;
                }

                _commandCallback((uint8_t)senderID, cmd);
            }

            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// _crc32
//
// CRC32 implementation that exactly matches the WCB firmware's calculateCRC32().
// The algorithm parameters must be identical or checksums will never verify:
//
//   Initial value : 0xFFFFFFFF
//   Polynomial    : 0xEDB88320  (bit-reversed / reflected CRC-32/ISO-HDLC)
//   Final XOR     : ~crc  (complement of the accumulated value)
//
// This is the same algorithm used by zlib, PNG, and most Ethernet stacks.
// ─────────────────────────────────────────────────────────────────────────────
uint32_t WCB_Client::_crc32(const char* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (int j = 0; j < 8; j++) {
            // If LSB is set, shift right and XOR with polynomial; else just shift
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// _espNowReceiveCB  (static)
//
// ESP-NOW's receive callback must be a plain C function with no implicit 'this'
// pointer, so it is declared static. It simply delegates to the non-static
// _handleReceive() on the stored _instance pointer.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_espNowReceiveCB(const esp_now_recv_info_t* info,
                                  const uint8_t* data, int len) {
    if (_instance) _instance->_handleReceive(info->src_addr, data, len);
}
