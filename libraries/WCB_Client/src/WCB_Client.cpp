#include "WCB_Client.h"
#include "WCBStream.h"
#include <esp_wifi.h>
#include <Preferences.h>   // learned-peer persistence (NVS)

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
    // device_id must be 1–WCB_MAX_BOARDS (hard-fail otherwise). A value ABOVE
    // wcb_quantity (and not the special slot 20) is ALLOWED but non-standard: the
    // floor boards (1..quantity) haven't pre-registered this MAC, so the device is
    // reachable INBOUND only once they auto-join it from its WDP advert — call
    // setIdentity() and keep auto-join on. begin() only WARNS in that case.
    if (_deviceID == 0 || _deviceID > WCB_MAX_BOARDS) {
        Serial.printf("[WCB_Client] ERROR: device_id %d is out of range (1–%d)\n",
                      _deviceID, WCB_MAX_BOARDS);
        return false;
    }
    // NON-FATAL: an id above wcb_quantity (and not the special slot 20) is allowed.
    // Index safety is already guaranteed by the out-of-range hard-fail above, and
    // every board array is sized WCB_MAX_BOARDS. The only consequence is inbound
    // reachability, which auto-join resolves once the floor boards hear this
    // device's WDP advert (setIdentity()).
    if (_deviceID != WCB_SPECIAL_ID && _deviceID > _quantity) {
        Serial.printf("[WCB_Client] WARNING: device_id %d is above wcb_quantity %d - "
                      "allowed, but the floor boards can't reach it until they auto-join "
                      "it from its WDP advert; call setIdentity() and keep auto-join on.\n",
                      _deviceID, _quantity);
    }

    // ── Reset internal state ─────────────────────────────────────────────────
    _seqCounter      = 0;
    _nextHeartbeatMs = 0;                          // Triggers heartbeat immediately on
                                                   // first update() call
    memset(_boards,  0, sizeof(_boards));
    memset(_pending, 0, sizeof(_pending));

    // ── WiFi setup ───────────────────────────────────────────────────────────
    // ESP-NOW requires WiFi to have station mode enabled. esp_now_init() fails
    // outright if the WiFi driver is not running, so verify the mode change
    // took before we get there — that failure is the usual root cause of a
    // crash on the first heartbeat.
    //
    // If the sketch has already brought up a SoftAP (e.g. hosting a web UI),
    // preserve it — AP and STA share a single radio channel on the ESP32, so
    // ESP-NOW simply rides whatever channel the AP is already on instead of
    // forcing STA-only and tearing the AP down. WiFi.disconnect() only drops
    // a STA association (default wifioff=false), so it's already AP-safe on
    // its own.
    wifi_mode_t priorMode = WiFi.getMode();
    bool apActive = (priorMode == WIFI_MODE_AP || priorMode == WIFI_MODE_APSTA);
    wifi_mode_t targetMode = apActive ? WIFI_AP_STA : WIFI_STA;
    if (!WiFi.mode(targetMode)) {
        Serial.printf("[WCB_Client] WARNING: WiFi.mode(%s) reported failure "
                      "— ESP-NOW init will likely fail below\n",
                      apActive ? "WIFI_AP_STA" : "WIFI_STA");
    }
    WiFi.disconnect();

    if (apActive) {
        // Modem sleep is far more likely to bite once the AP is actively
        // serving clients — a missed ESP-NOW heartbeat here reads as this
        // device going offline to the rest of the mesh.
        esp_wifi_set_ps(WIFI_PS_NONE);
        Serial.println("[WCB_Client] SoftAP detected — running WIFI_AP_STA, "
                       "ESP-NOW sharing the AP's radio channel");
    }

    // ── Mesh channel ─────────────────────────────────────────────────────────
    // ESP-NOW peers are channel-0 ("current channel"), so this device is only
    // heard if its radio sits on the mesh channel (_meshChannel, settable via
    // setMeshChannel()). With no SoftAP we pin it deterministically; with a
    // SoftAP active the AP owns the channel, so we can't move it without yanking
    // the AP — we only WARN on a mismatch, and re-check in update() to catch a
    // SoftAP that's brought up after begin().
    if (!apActive) {
        esp_err_t chErr = esp_wifi_set_channel(_meshChannel, WIFI_SECOND_CHAN_NONE);
        if (chErr != ESP_OK) {
            Serial.printf("[WCB_Client] WARNING: esp_wifi_set_channel(%u) failed (err %d) - "
                          "the radio may be on the wrong channel; the mesh uses 1-11.\n",
                          _meshChannel, chErr);
        }
    } else {
        _checkMeshChannel();
    }

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
    // If this fails, _started stays false and update() becomes a safe no-op
    // (see the guard there) instead of crashing on the first heartbeat. Print
    // the exact error name so the cause is obvious from the boot log.
    esp_err_t enErr = esp_now_init();
    if (enErr != ESP_OK) {
        Serial.printf("[WCB_Client] ERROR: esp_now_init() failed: %s (0x%x). "
                      "Device will NOT join the mesh; check that WiFi started.\n",
                      esp_err_to_name(enErr), enErr);
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

    // Restore auto-joined peers persisted by a previous session — they are
    // permanent members until the app forgets them (forgetPeer/clearLearnedPeers).
    _loadLearnedPeers();
    _started = true;             // enableSpecialPeer() after begin() registers immediately

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
    // If begin() never succeeded (e.g. esp_now_init() failed), ESP-NOW is not
    // initialised. Calling esp_now_send() in that state dereferences a NULL
    // driver global and hard-crashes the chip (Guru Meditation LoadProhibited)
    // — an S3 with a failed init would boot-loop here. Do nothing until begin()
    // has actually brought ESP-NOW up.
    if (!_started) return;

    unsigned long now = millis();

    if (now >= _nextHeartbeatMs) {
        _sendHeartbeat();
        _nextHeartbeatMs = now + (_heartbeatIntervalSec * 1000UL);
        _checkMeshChannel();   // catch a SoftAP that moved the radio off the mesh channel
    }

    _checkOfflineBoards();
    _processMonitors();
    _processFragJob();   // drain a pending fragmented send, one chunk per tick
    _wdpTick();          // WDP device-identity advert cadence (boot burst + periodic)
    _ageNeighbors(now);  // expire WDP neighbors we haven't heard from recently

    // Drain deferred auto-joins on the LOOP task. _handleWdpAdvert (which flags
    // these) runs in the ESP-NOW receive callback (WiFi task), where
    // _addLearnedPeer()'s esp_now_add_peer + NVS flash write would overflow the
    // small callback stack and crash. Doing them here makes them safe.
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        if (_pendingJoin[i]) { _pendingJoin[i] = false; _addLearnedPeer((uint8_t)(i + 1)); }
    }

    // Register transient reply-peers flagged on the RX callback (an authenticated
    // above-floor sender we couldn't unicast an ACK/reply to yet). Same discipline
    // as auto-join above — the bool is set on Core 0, the esp_now_add_peer runs
    // HERE on the loop task — but with NO NVS write: this is not a learned member,
    // just enough of a peer to answer whoever just talked to us. Auto-join still
    // persists genuine members separately. Lands within ~1 loop, so the sender's
    // next command (the config tool re-pings 6x/500ms on connect) gets its reply.
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        if (!_pendingReplyPeer[i]) continue;
        _pendingReplyPeer[i] = false;
        if (esp_now_is_peer_exist(_wcbMACs[i])) continue;
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, _wcbMACs[i], 6);
        peer.channel = 0;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    // Service the pending table. _pending[] is mutated cross-core (the ACK handler
    // and a Core-0 _sendPacket claim BOTH take _pendingMux), so every read/write of
    // a slot here runs UNDER the lock too — otherwise this Core-1 loop would tear a
    // slot the RX task is concurrently claiming/acking. The blocking _transmit()
    // (esp_now_send) must NOT run inside the spinlock, so we collect the boards to
    // retransmit + a snapshot of the command/seq under the lock and transmit AFTER
    // releasing it (a stale retransmit is harmless — receivers dedup by seq).
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        uint8_t  retryBoards[WCB_MAX_BOARDS];
        int      nRetry  = 0;
        char     cmdCopy[sizeof(_pending[0].command)];
        uint16_t seqCopy = 0;

        portENTER_CRITICAL(&_pendingMux);
        WCBPending& p = _pending[i];
        if (p.active) {
            if (p.ensured) {
                // ── Ensured delivery: PER-BOARD UNICAST retries ──────────────
                // Mirrors the WCB firmware's processETMAcksAndRetries(): retry as a
                // UNICAST to each expected board that hasn't ACK'd (reusing the seq
                // so it's deduped), up to ETM_MAX_RETRIES per board.
                if (_ensuredComplete(p)) {            // every expected board acked / gone
                    p.active = false;
                } else if ((now - p.sentMs) >= ETM_RETRY_INTERVAL_MS) {
                    for (int b = 0; b < WCB_MAX_BOARDS; b++) {
                        if (!p.expected[b] || p.ackReceived[b]) continue;
                        if (!_boards[b].online) {
                            // Dropped after being online (lastSeenMs!=0) -> give up.
                            // Never-online learned peer (lastSeenMs==0, advert-only
                            // client) -> keep retrying up to ETM_MAX_RETRIES.
                            if (_boards[b].lastSeenMs != 0) { p.expected[b] = false; continue; }
                        }
                        if (p.retryCount[b] < ETM_MAX_RETRIES) {
                            retryBoards[nRetry++] = (uint8_t)(b + 1);   // _transmit after unlock
                            p.retryCount[b]++;
                        } else {
                            p.expected[b] = false;    // exhausted retries for this board
                        }
                    }
                    p.sentMs = now;                   // reset the retry window
                    if (_ensuredComplete(p)) p.active = false;
                    if (nRetry) { memcpy(cmdCopy, p.command, sizeof(cmdCopy)); seqCopy = p.seqNum; }
                }
            } else {
                // Best-effort unicast: sent ONCE, no retransmit — free a slot whose
                // ACK never arrived so the table can't leak (1 s >> ETM round-trip).
                if ((now - p.sentMs) > 1000UL) p.active = false;
            }
        }
        portEXIT_CRITICAL(&_pendingMux);

        for (int k = 0; k < nRetry; k++) _transmit(retryBoards[k], cmdCopy, seqCopy);
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
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
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
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
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
// Longest command that fits ONE packet. Derived from the actual struct field
// (not magic numbers) so this threshold can never drift from what _sendPacket
// physically packs: structCommand holds N-1 chars + NUL; when checksum is
// enabled the "|CRC%08X" suffix (12 chars) shares the same field.
// ─────────────────────────────────────────────────────────────────────────────
static const size_t kCmdFieldChars  = sizeof(((wcb_packet_etm_t*)0)->structCommand) - 1; // 199
static const size_t kCrcSuffixChars = 12;  // strlen("|CRC") + 8 hex digits

size_t WCB_Client::_maxSingleCommandLen() const {
    return _checksumEnabled ? (kCmdFieldChars - kCrcSuffixChars) : kCmdFieldChars;
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
// NON-BLOCKING (1.3.1): this only QUEUES the job — chunks are transmitted one
// at a time from update() with ~10 ms pacing, so it is safe to call from
// loop() or even the ESP-NOW receive callback (the old implementation ran
// delay(10) loops in the caller, which could stall the WiFi task for ~0.5 s).
// The target's 15 s reassembly window comfortably covers the spread passes.
//
// Returns true = queued. The transmission result (every chunk accepted by
// ESP-NOW at least once across the passes, or not) is logged on completion.
// NOTE: acceptance is not delivery — there is no ACK at this layer.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_sendFragmented(uint8_t target_wcb, const char* command, bool ensured) {
    size_t len = strlen(command);

    // Bound-check on LENGTH before any narrow cast: computing a uint8_t chunk
    // count first wraps at 256 chunks (~45.8 KB) and would bypass this guard —
    // silently truncating or sending nothing.
    if (len > WCB_MGMT_MAX_COMMAND_LEN) {
        Serial.printf("[WCB_Client] send: command too long even for fragmentation "
                      "(%u chars > %u max). Not sent.\n",
                      (unsigned)len, (unsigned)WCB_MGMT_MAX_COMMAND_LEN);
        return false;
    }
    if (_fragJob.active) {
        Serial.println("[WCB_Client] send: a fragmented send is already in flight — "
                       "wait for it to finish (max ~0.5 s) and retry.");
        return false;
    }

    char* copy = (char*)malloc(len + 1);
    if (!copy) {
        Serial.println("[WCB_Client] send: out of memory for fragmented command.");
        return false;
    }
    memcpy(copy, command, len + 1);

    // Fresh sessionId per command. 0xFFFF is reserved (the firmware's
    // completed-session ring buffer initialises to 0xFFFF, which would make
    // the very first session look like a duplicate and be discarded).
    uint16_t sessionId = (uint16_t)esp_random();
    if (sessionId == 0xFFFF) sessionId = 0xFFFE;

    _fragJob.targetWCB    = target_wcb;
    _fragJob.sessionId    = sessionId;
    _fragJob.totalChunks  = (uint8_t)((len + WCB_MGMT_CHUNK_LEN - 1) / WCB_MGMT_CHUNK_LEN);
    _fragJob.passes       = ensured ? 3 : 2;
    _fragJob.pass         = 0;
    _fragJob.chunk        = 0;
    _fragJob.acceptedMask = 0;
    _fragJob.nextSendMs   = 0;       // first chunk goes out on the next update()
    _fragJob.cmd          = copy;
    _fragJob.len          = len;
    _fragJob.active       = true;    // set LAST — update() may run on the other core

    Serial.printf("[WCB_Client] send: %u chars > single-packet limit — fragmenting "
                  "to WCB%d (%d chunks, session %04X, %d passes, via update())\n",
                  (unsigned)len, target_wcb, _fragJob.totalChunks, sessionId,
                  _fragJob.passes);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// _processFragJob — transmit the next pending fragment. Called from update().
// One chunk per call, ~10 ms apart: a maximal ensured job (16 chunks × 3
// passes) completes in ~0.5 s of wall time without ever blocking loop().
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_processFragJob() {
    if (!_fragJob.active) return;
    unsigned long now = millis();
    if (now < _fragJob.nextSendMs) return;

    wcb_packet_mgmt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    pkt.packetType  = WCB_MGMT_PACKET_TYPE_FRAG_UNICAST;  // unicast semantics on 6.1.1+
    pkt.targetWCB   = _fragJob.targetWCB;
    pkt.sessionId   = _fragJob.sessionId;
    pkt.chunkIdx    = _fragJob.chunk;
    pkt.totalChunks = _fragJob.totalChunks;

    size_t off = (size_t)_fragJob.chunk * WCB_MGMT_CHUNK_LEN;
    size_t n   = _fragJob.len - off;
    if (n > WCB_MGMT_CHUNK_LEN) n = WCB_MGMT_CHUNK_LEN;
    memcpy(pkt.payload, _fragJob.cmd + off, n);   // payload[180] zeroed above → NUL-safe

    if (esp_now_send(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK) {
        _fragJob.acceptedMask |= (uint16_t)(1u << _fragJob.chunk);
    }
    _fragJob.nextSendMs = now + 10;   // pacing: don't flood the ESP-NOW TX queue

    // Advance chunk → pass → done.
    if (++_fragJob.chunk >= _fragJob.totalChunks) {
        _fragJob.chunk = 0;
        if (++_fragJob.pass >= _fragJob.passes) {
            // Success = every chunk accepted by ESP-NOW at least once across
            // the passes (per-chunk, NOT per-pass: the target dedups chunks,
            // so a union of clean chunks completes the session).
            uint16_t fullMask = (uint16_t)((1u << _fragJob.totalChunks) - 1);
            if (_fragJob.acceptedMask == fullMask) {
                Serial.printf("[WCB_Client] fragmented send to WCB%d complete "
                              "(session %04X, %d chunks x %d passes)\n",
                              _fragJob.targetWCB, _fragJob.sessionId,
                              _fragJob.totalChunks, _fragJob.passes);
            } else {
                Serial.printf("[WCB_Client] fragmented send to WCB%d INCOMPLETE — "
                              "chunk mask %04X of %04X accepted; the command may "
                              "not have executed\n",
                              _fragJob.targetWCB, _fragJob.acceptedMask, fullMask);
            }
            free(_fragJob.cmd);
            _fragJob.cmd    = nullptr;
            _fragJob.active = false;
        }
    }
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
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
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
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
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
//   buffer resets. Lines longer than the single-packet limit (199 chars,
//   187 with checksum) are truncated to that limit before dispatch.
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
                // Terminator received — dispatch whatever is in the buffer.
                // CLAMP to the single-packet limit first: monitor lines are
                // streaming spam, so an over-limit line is truncated (and
                // delivered) rather than dropped by broadcast()'s oversize
                // rejection or queued through the fragmented path — both of
                // which are wrong for high-rate monitor traffic.
                if (_monitorSerialLen > 0) {
                    size_t maxLen = _maxSingleCommandLen();
                    if (_monitorSerialLen > maxLen) _monitorSerialLen = maxLen;
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
// enableSpecialPeer / isSpecialPeerOnline / sendToSpecialPeer
//
// The "special peer" (NaviCore) is an out-of-band device (default ID 20) that
// lives outside the 1..wcb_quantity range. Receiving heartbeats/commands/ACKs
// from it already works; these helpers add the two things that DON'T work until
// the peer is explicitly enabled: (1) registering its MAC so we can SEND to it,
// and (2) including it in offline detection so its ETM status is tracked.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::enableSpecialPeer(uint8_t id) {
    if (id < 1 || id > WCB_MAX_BOARDS) {
        Serial.printf("[WCB_Client] enableSpecialPeer: id %d out of range (1–%d)\n",
                      id, WCB_MAX_BOARDS);
        return;
    }
    _specialPeerID = id;
    if (_started) _registerSpecialPeer();   // begin() already ran — register now
    Serial.printf("[WCB_Client] Special peer enabled at ID %d\n", id);
}

bool WCB_Client::isSpecialPeerOnline() {
    if (_specialPeerID < 1 || _specialPeerID > WCB_MAX_BOARDS) return false;
    return _boards[_specialPeerID - 1].online;
}

bool WCB_Client::sendToSpecialPeer(const char* command, bool ensured) {
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
    if (_specialPeerID < 1) {
        Serial.println("[WCB_Client] sendToSpecialPeer: special peer not enabled "
                       "(call enableSpecialPeer() first)");
        return false;
    }
    return send(_specialPeerID, command, ensured);
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

void WCB_Client::onRawPacket(WCBRawPacketCallback callback) {
    _rawPacketCallback = callback;
}

// ─────────────────────────────────────────────────────────────────────────────
// onNeighbor / getNeighbor / neighborCount — WDP consumer API
//
// A neighbor is any board on the mesh whose WDP advert we've decoded: another
// WCB, or a WCB_Client device (isClient). Adverts arrive roughly every 30s and
// age out after WCB_WDP_NEIGHBOR_TTL_MS of silence.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::onNeighbor(WCBNeighborCallback callback) {
    _neighborCallback = callback;
}

const WCBNeighbor* WCB_Client::getNeighbor(uint8_t wcbNumber) {
    if (wcbNumber < 1 || wcbNumber > WCB_MAX_BOARDS) return nullptr;
    const WCBNeighbor& nb = _neighbors[wcbNumber - 1];
    return nb.valid ? &nb : nullptr;
}

uint8_t WCB_Client::neighborCount() {
    uint8_t n = 0;
    for (int i = 0; i < WCB_MAX_BOARDS; i++)
        if (_neighbors[i].valid) n++;
    return n;
}

// Enable/disable auto-join. Turning it OFF stops NEW boards from being learned;
// already-registered learned peers stay until they age out or the device
// restarts. Turning it ON lets subsequently-heard boards join.
void WCB_Client::setAutoJoin(bool enabled) {
    _autoJoin = enabled;
}

// Expire neighbors we haven't heard an advert from within the TTL. Fires
// onNeighbor once more with valid=false so the app can drop it from any UI.
void WCB_Client::_ageNeighbors(unsigned long now) {
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        WCBNeighbor& nb = _neighbors[i];
        if (!nb.valid) continue;
        if ((long)(now - nb.lastSeenMs) >= (long)WCB_WDP_NEIGHBOR_TTL_MS) {
            nb.valid = false;
            if (_neighborCallback) _neighborCallback(nb);
        }
    }
}

// Unicast a raw buffer to a WCB's MAC (computed scheme). Registers the peer on
// demand so a custom protocol (e.g. OTA) can reach any WCB without relying on
// _registerPeers() having already added it. Returns true if ESP-NOW accepted it.
bool WCB_Client::sendRawPacket(uint8_t target_wcb, const uint8_t* data, size_t len) {
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
    if (target_wcb < 1 || target_wcb > WCB_MAX_BOARDS || !data || len == 0) return false;
    const uint8_t* mac = _wcbMACs[target_wcb - 1];
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) return false;
    }
    return esp_now_send(mac, data, len) == ESP_OK;
}

// Enable or disable CRC32 checksum. Must match the WCB network's ?ETM,CHKSM setting.
void WCB_Client::setChecksum(bool enabled) {
    _checksumEnabled = enabled;
}

// Set the ESP-NOW mesh channel this device expects (1–13). See the header for the
// full contract. Out-of-range values are ignored (the current value is kept).
void WCB_Client::setMeshChannel(uint8_t channel) {
    if (channel < 1 || channel > 11) {
        Serial.printf("[WCB_Client] setMeshChannel: %u out of range (1-11), ignored\n", channel);
        return;
    }
    _meshChannel = channel;
    // If begin() already ran and we own the radio (no SoftAP), re-pin it live so a
    // runtime change actually takes effect — otherwise this would only update the
    // expected value while the radio stayed put. With a SoftAP active the AP owns
    // the channel; we don't move it (update()'s _checkMeshChannel warns instead).
    if (_started) {
        wifi_mode_t mode = WiFi.getMode();
        bool apActive = (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
        if (!apActive) esp_wifi_set_channel(_meshChannel, WIFI_SECOND_CHAN_NONE);
    }
}

// Warn (rate-limited) if the radio isn't on the expected mesh channel — this device
// can't reach the mesh when off-channel, so surface it loudly instead of failing
// silently. The advice depends on WHY the radio moved: a hosted SoftAP owns the
// channel (bring the AP up on the mesh channel), otherwise a STA association or a
// channel change moved it. We only warn (never force) so we don't yank a
// deliberately-chosen SoftAP channel.
//
// NOTE: this can only catch a radio that drifted OFF _meshChannel. It cannot detect
// a _meshChannel that is itself wrong (the fleet moved via ?WCBCH but this sketch was
// never updated) — an ESP-NOW device on the wrong channel hears nothing, so it has no
// way to learn the fleet's real channel. Keep _meshChannel in sync via setMeshChannel().
void WCB_Client::_checkMeshChannel() {
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&primary, &second) != ESP_OK) return;
    if (primary == _meshChannel) { _lastWarnedChannel = 0; return; }  // on-channel: reset so a later drift re-warns

    unsigned long now = millis();
    // Warn on first detection / a change of wrong channel; otherwise throttle to 30s.
    if (primary == _lastWarnedChannel && (now - _lastChannelWarnMs) < 30000UL) return;
    _lastWarnedChannel = primary;
    _lastChannelWarnMs = now;
    wifi_mode_t mode = WiFi.getMode();
    bool apActive = (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
    if (apActive) {
        Serial.printf("[WCB_Client] WARNING: radio is on channel %u but the WCB mesh is on "
                      "channel %u - this device will NOT reach the mesh. A hosted SoftAP owns "
                      "the radio channel; bring the AP up on the mesh channel: "
                      "WiFi.softAP(ssid, pass, %u).\n",
                      primary, _meshChannel, _meshChannel);
    } else {
        Serial.printf("[WCB_Client] WARNING: radio is on channel %u but the WCB mesh is on "
                      "channel %u - this device will NOT reach the mesh. A WiFi STA association "
                      "or a channel change may have moved it; call setMeshChannel(%u) or reboot.\n",
                      primary, _meshChannel, _meshChannel);
    }
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

    // Register the out-of-band special peer (NaviCore) if one was enabled.
    _registerSpecialPeer();
}

// ─────────────────────────────────────────────────────────────────────────────
// _registerSpecialPeer
//
// Registers the special peer's MAC as an ESP-NOW peer so send()/sendToSpecialPeer()
// can reach it. Only needed when the special peer sits OUTSIDE the 1..quantity
// range (IDs within that range are already registered by _registerPeers()).
// No-op when no special peer is enabled, when it's this device's own ID, or when
// it's already a normal peer.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_registerSpecialPeer() {
    if (_specialPeerID < 1 || _specialPeerID > WCB_MAX_BOARDS) return;
    if (_specialPeerID == _deviceID)  return;   // that's us
    if (_specialPeerID <= _quantity)  return;   // already registered as a normal peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, _wcbMACs[_specialPeerID - 1], 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

// ─────────────────────────────────────────────────────────────────────────────
// _addLearnedPeer
//
// Register a regular WCB (learned via a WDP advert) as an ESP-NOW peer, live,
// and remember it in NVS. Uses the same derived MAC scheme as _buildMACs(), so
// no address is learned — only the FACT that the board exists. Idempotent, and
// guarded against self, the special peer, ids already covered by the
// 1..quantity floor, and the ~20-peer ESP-NOW cap. A learned peer is PERMANENT:
// it is restored on every begin() and is always expected to be on and ready.
// Offline-detection tracks its heartbeat liveness, but membership never
// self-evicts — only forgetPeer()/clearLearnedPeers() remove it. If the peer
// table gets crowded, clearing it up is deliberately the user's call.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_addLearnedPeer(uint8_t id) {
    if (id < 1 || id > WCB_MAX_BOARDS)  return false;
    if (id == _deviceID)                return false;   // that's us
    if (id == _specialPeerID || id == WCB_SPECIAL_ID)
                                        return false;   // controller slot — never learn/persist it
    if (id <= _quantity)                return false;   // already a floor peer
    int idx = id - 1;
    if (_learnedPeer[idx])              return true;    // already joined

    if (!esp_now_is_peer_exist(_wcbMACs[idx])) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, _wcbMACs[idx], 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.printf("[WCB_Client] auto-join: could not add WCB%d (peer table full?)\n", id);
            return false;
        }
    }
    _learnedPeer[idx] = true;
    _saveLearnedPeers();     // joining is a rare, one-time event — persist immediately
    Serial.printf("[WCB_Client] auto-joined WCB%d\n", id);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Learned-peer persistence (NVS namespace "wcb_peers")
//
// Only MEMBERSHIP is stored, as a 20-bit mask — identity re-learns from adverts
// and MACs are always derived, so there is nothing else to keep. The blob is
// fingerprinted with the network octets: change oct2/oct3 (a different mesh)
// and the old membership is discarded instead of deriving wrong-group MACs.
// Mirrors the WCB firmware's "learned_peers" scheme.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_saveLearnedPeers() {
    uint32_t mask = 0;
    for (int i = 0; i < WCB_MAX_BOARDS; i++)
        if (_learnedPeer[i]) mask |= (1UL << i);
    Preferences prefs;
    prefs.begin("wcb_peers", false);
    prefs.putUChar("ver", 1);
    prefs.putUChar("o2", _oct2);
    prefs.putUChar("o3", _oct3);
    prefs.putUInt("mask", mask);
    prefs.end();
}

void WCB_Client::_loadLearnedPeers() {
    Preferences prefs;
    prefs.begin("wcb_peers", true);
    uint8_t  ver  = prefs.getUChar("ver", 0);
    uint8_t  o2   = prefs.getUChar("o2", 0);
    uint8_t  o3   = prefs.getUChar("o3", 0);
    uint32_t mask = prefs.getUInt("mask", 0);
    prefs.end();

    if (ver != 1) return;                       // nothing saved (or unknown schema)
    if (o2 != _oct2 || o3 != _oct3) {           // saved under a different network
        Preferences wipe;
        wipe.begin("wcb_peers", false);
        wipe.clear();
        wipe.end();
        return;
    }

    int restored = 0;
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        if (!(mask & (1UL << i))) continue;
        uint8_t id = i + 1;
        // Ids now covered by the floor/special/self are skipped (e.g. the app
        // raised wcb_quantity since the save) — _addLearnedPeer re-derives the
        // guards, so a stale bit simply doesn't re-join.
        if (_addLearnedPeer(id)) restored++;
    }
    if (restored > 0)
        Serial.printf("[WCB_Client] restored %d learned peer(s)\n", restored);
}

// Forget one auto-joined peer: deregister it, drop it from NVS, mark it offline.
// Floor peers (1..wcb_quantity) and the special peer are unaffected.
void WCB_Client::forgetPeer(uint8_t id) {
    if (id < 1 || id > WCB_MAX_BOARDS) return;
    // Never deregister the special peer's ESP-NOW MAC. clearLearnedPeers()
    // funnels through here, and _specialPeerID can still be 0 when the special
    // peer is enabled AFTER begin(), so guard the reserved controller slot
    // unconditionally — otherwise a routine table cleanup would silently break
    // sendToSpecialPeer() by deleting the peer whose MAC it shares.
    if (id == _specialPeerID || id == WCB_SPECIAL_ID) return;
    int idx = id - 1;
    if (!_learnedPeer[idx]) return;
    _learnedPeer[idx] = false;
    _advertCount[idx] = 0;   // reset so it must be heard >=2 times again before it can re-join
    if (esp_now_is_peer_exist(_wcbMACs[idx])) esp_now_del_peer(_wcbMACs[idx]);
    _boards[idx].online = false;
    // Also drop the WDP discovery record so getNeighbor(id) goes null immediately —
    // otherwise a just-forgotten peer keeps showing (via getNeighbor) for up to
    // WCB_WDP_NEIGHBOR_TTL_MS until the advert ages out. It repopulates on its next
    // advert (and re-joins via auto-join after >=2 adverts), which is intended.
    _neighbors[idx] = WCBNeighbor{};
    _saveLearnedPeers();
    Serial.printf("[WCB_Client] forgot WCB%d\n", id);
}

// Forget ALL auto-joined peers (the user-cleanup handle for a crowded table).
void WCB_Client::clearLearnedPeers() {
    for (int i = 0; i < WCB_MAX_BOARDS; i++)
        if (_learnedPeer[i]) forgetPeer(i + 1);
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
    if (!_started) return;   // ESP-NOW not up — never touch esp_now_send()

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
// WDP device-identity advert
//
// Broadcasts this device's identity (set via setIdentity) on the mesh so every
// WCB discovers it through its Wireless Discovery Protocol neighbor table. The
// wire format MUST match the WCB firmware's WCB_WDP: a 2-byte header (magic 'W',
// proto 0x01) then [type][len][value] TLVs, carried raw in the ETM packet's
// structCommand with structPacketType = WCB_PACKET_WDP. No CRC — WDP carries TLV
// bytes, not a text command.
// ─────────────────────────────────────────────────────────────────────────────
#define WCB_WDP_MAGIC        'W'
#define WCB_WDP_PROTO        0x01
#define WCB_WDP_TLV_END      0x00
#define WCB_WDP_TLV_FWVER    0x03   // string (shared with WCB adverts)
#define WCB_WDP_TLV_DEVTYPE  0x0B   // string — canonical device type name
#define WCB_WDP_TLV_HWREV    0x0C   // string — hardware revision
#define WCB_WDP_TLV_CAPTAGS  0x0D   // string — space-separated capability tags
// Decode-only TLVs (WCBs advertise these; the consumer below reads them).
#define WCB_WDP_TLV_ALIAS     0x01  // string — WCB alias
#define WCB_WDP_TLV_HWVER     0x04  // uint8  — WCB numeric hardware version
#define WCB_WDP_TLV_CAPFLAGS  0x05  // uint16 LE — WCB capability bitmap
#define WCB_WDP_TLV_MAESTRO   0x06  // bytes  — local Maestro IDs
#define WCB_WDP_TLV_PORTLABEL 0x09  // [port][label] — one per labeled serial port
#define WCB_WDP_TLV_CTRLID    0x0A  // uint8  — controller (special-peer) ID

// Append one TLV; returns the new offset (unchanged if it wouldn't fit).
static int wcbPutTLV(uint8_t* buf, int o, int max, uint8_t type,
                     const uint8_t* val, int len) {
    if (len < 0)   len = 0;
    if (len > 255) len = 255;
    if (o + 2 + len > max) return o;   // no room — skip this TLV
    buf[o++] = type;
    buf[o++] = (uint8_t)len;
    for (int i = 0; i < len; i++) buf[o++] = val[i];
    return o;
}

void WCB_Client::setIdentity(const char* type, const char* fw,
                             const char* hwRev, const char* caps) {
    strncpy(_wdpType,  type  ? type  : "", sizeof(_wdpType)  - 1);  _wdpType[sizeof(_wdpType)  - 1] = '\0';
    strncpy(_wdpFw,    fw    ? fw    : "", sizeof(_wdpFw)    - 1);  _wdpFw[sizeof(_wdpFw)      - 1] = '\0';
    strncpy(_wdpHwRev, hwRev ? hwRev : "", sizeof(_wdpHwRev) - 1);  _wdpHwRev[sizeof(_wdpHwRev) - 1] = '\0';
    strncpy(_wdpCaps,  caps  ? caps  : "", sizeof(_wdpCaps)  - 1);  _wdpCaps[sizeof(_wdpCaps)  - 1] = '\0';

    // Arm the cadence so the new identity goes out promptly (short boot burst),
    // then settles to a staggered periodic backstop. Works whether called before
    // or after begin(): the send happens from update() once ESP-NOW is up.
    _wdpBootLeft     = (_wdpType[0]) ? 3 : 0;
    _wdpNextBootMs   = millis() + 300;
    _wdpNextAdvertMs = millis() + 60000UL + (unsigned long)((_deviceID % 16) * 500);

    if (_wdpType[0])
        Serial.printf("[WCB_Client] WDP identity set: type=\"%s\" fw=\"%s\"\n", _wdpType, _wdpFw);
}

int WCB_Client::_buildWdpPayload(uint8_t* buf, int max) {
    int o = 0;
    if (max < 3) return 0;
    buf[o++] = WCB_WDP_MAGIC;
    buf[o++] = WCB_WDP_PROTO;
    o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_DEVTYPE, (const uint8_t*)_wdpType, strlen(_wdpType));
    if (_wdpFw[0])    o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_FWVER,   (const uint8_t*)_wdpFw,    strlen(_wdpFw));
    if (_wdpHwRev[0]) o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_HWREV,   (const uint8_t*)_wdpHwRev, strlen(_wdpHwRev));
    if (_wdpCaps[0])  o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_CAPTAGS, (const uint8_t*)_wdpCaps,  strlen(_wdpCaps));
    if (o < max) buf[o++] = WCB_WDP_TLV_END;
    return o;
}

void WCB_Client::_sendWdpAdvert() {
    wcb_packet_etm_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "0");   // 0 = broadcast
    pkt.structCommandIncluded = 1;
    // Raw TLV bytes (may contain 0x00) go straight into structCommand — no CRC.
    _buildWdpPayload((uint8_t*)pkt.structCommand, sizeof(pkt.structCommand));
    pkt.structPacketType     = WCB_PACKET_WDP;
    pkt.structSequenceNumber = 0;
    esp_now_send(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt));
}

void WCB_Client::_wdpTick() {
    if (!_started || !_wdpType[0]) return;   // not running, or nothing to advertise
    unsigned long now = millis();
    if (_wdpBootLeft > 0 && now >= _wdpNextBootMs) {
        _sendWdpAdvert();
        _wdpBootLeft--;
        _wdpNextBootMs = now + 1300;
    }
    if ((long)(now - _wdpNextAdvertMs) >= 0) {   // rollover-safe
        _sendWdpAdvert();
        _wdpNextAdvertMs = now + 60000UL;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// _handleWdpAdvert — WDP consumer
//
// Decode a neighbor's advert payload (magic 'W' + proto + [type][len][value]
// TLVs, riding structCommand) into _neighbors[senderWCB-1] and fire onNeighbor.
// Mirrors the WCB firmware's decoder; a client's DEVTYPE doubles as the name and
// flags isClient. In-RAM parse only — safe to run inline in the receive callback.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_handleWdpAdvert(uint8_t senderWCB, const uint8_t* cmd) {
    if (senderWCB < 1 || senderWCB > WCB_MAX_BOARDS) return;
    if (cmd[0] != (uint8_t)WCB_WDP_MAGIC || cmd[1] != WCB_WDP_PROTO) return;

    WCBNeighbor& nb = _neighbors[senderWCB - 1];
    memset(&nb, 0, sizeof(nb));            // a board is the sole authority for its facts — replace wholesale
    nb.valid      = true;
    nb.wcbNumber  = senderWCB;
    nb.lastSeenMs = millis();

    int o = 2;                             // skip magic + version
    while (o + 2 <= 200) {
        uint8_t type = cmd[o];
        if (type == WCB_WDP_TLV_END) break;
        int len = cmd[o + 1];
        const uint8_t* val = &cmd[o + 2];
        if (o + 2 + len > 200) break;      // truncated / malformed
        switch (type) {
            case WCB_WDP_TLV_ALIAS: {
                int L = len > 24 ? 24 : len; memcpy(nb.name, val, L); nb.name[L] = '\0'; break;
            }
            case WCB_WDP_TLV_DEVTYPE: {    // client device type doubles as the name
                int L = len > 24 ? 24 : len; memcpy(nb.name, val, L); nb.name[L] = '\0';
                nb.isClient = true; break;
            }
            case WCB_WDP_TLV_FWVER: {
                int L = len > 27 ? 27 : len; memcpy(nb.fw, val, L); nb.fw[L] = '\0'; break;
            }
            case WCB_WDP_TLV_HWVER:  if (len >= 1) nb.hwVer = val[0]; break;
            case WCB_WDP_TLV_HWREV: {
                int L = len > 15 ? 15 : len; memcpy(nb.hwRev, val, L); nb.hwRev[L] = '\0'; break;
            }
            case WCB_WDP_TLV_CAPFLAGS:
                if (len >= 2) nb.capFlags = (uint16_t)val[0] | ((uint16_t)val[1] << 8);
                break;
            case WCB_WDP_TLV_CAPTAGS: {
                int L = len > 48 ? 48 : len; memcpy(nb.capTags, val, L); nb.capTags[L] = '\0'; break;
            }
            case WCB_WDP_TLV_CTRLID:  if (len >= 1) nb.ctrlId = val[0]; break;
            case WCB_WDP_TLV_MAESTRO: {
                int L = len > 9 ? 9 : len; memcpy(nb.maestroIds, val, L); nb.maestroCount = (uint8_t)L; break;
            }
            case WCB_WDP_TLV_PORTLABEL: {
                if (len >= 1) {
                    int port = val[0];
                    if (port >= 1 && port <= 5) {
                        int L = len - 1; if (L > 24) L = 24;
                        memcpy(nb.portLabels[port - 1], val + 1, L);
                        nb.portLabels[port - 1][L] = '\0';
                    }
                }
                break;
            }
            default: break;                // unknown TLV — skipped by length (forward-compatible)
        }
        o += 2 + len;
    }

    // Auto-join: register a regular WCB (not a client device, not the special
    // peer) as an ESP-NOW peer once we've heard it advertise at least twice, so
    // a single stray/spoofed advert can't add a peer. The sender was bound to its
    // source MAC in _handleReceive, so senderWCB is trustworthy here.
    //
    // CRITICAL: this decode runs in the ESP-NOW RECEIVE CALLBACK (WiFi task) with
    // a small stack. _addLearnedPeer() does esp_now_add_peer() + an NVS flash
    // write, which must NEVER run here (overflows the callback stack → crash /
    // boot loop on the 2nd advert). Only FLAG the join; update() performs it on
    // the loop task.
    // Auto-join learns WCBs AND client devices (mesh monitors, other controllers,
    // command-accepting clients) so anything seen on the mesh becomes a PERSISTENT
    // peer you can send() to — registered on the loop task, saved to NVS, restored
    // every begin(). (_addLearnedPeer still skips self, the special peer, and the
    // 1..wcb_quantity floor.) A learned peer never self-evicts; forgetPeer() /
    // clearLearnedPeers() to drop one.
    if (_autoJoin) {
        if (_advertCount[senderWCB - 1] < 255) _advertCount[senderWCB - 1]++;
        if (_advertCount[senderWCB - 1] >= 2 && !_learnedPeer[senderWCB - 1])
            _pendingJoin[senderWCB - 1] = true;
    }

    if (_neighborCallback) _neighborCallback(nb);
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

    // Claim + fill the pending slot atomically vs the cross-core ACK handler.
    // _findFreePending assumes _pendingMux is held; keep only array work inside
    // the critical section (no esp_now/Serial) — the transmit happens after.
    portENTER_CRITICAL(&_pendingMux);
    int slot = track ? _findFreePending() : -1;
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
    portEXIT_CRITICAL(&_pendingMux);

    if (slot < 0 && ensured) {
        // Pending table saturated with still-outstanding ensured deliveries. We
        // couldn't get a slot to track/retry this one, and must NOT evict a
        // guaranteed in-flight command to make room (see _findFreePending). Send
        // it best-effort once and report non-guaranteed delivery so the caller
        // can retry rather than assume ensured semantics it didn't get.
        _transmit(targetID, command, seq);
        return false;
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
        if (!p.expected[b] || p.ackReceived[b]) continue;
        // Still outstanding if the board is online, OR it is a peer that has
        // NEVER been online (lastSeenMs==0) and still has retries left — e.g. a
        // learned client that only adverts (WDP) and never heartbeats. Without
        // this, an ensured send to such a target would look instantly complete
        // and never retry. A board that WAS online and then dropped
        // (lastSeenMs!=0, !online) is NOT outstanding — stop waiting on a board
        // that's simply gone.
        if (_boards[b].online ||
            (_boards[b].lastSeenMs == 0 && p.retryCount[b] < ETM_MAX_RETRIES))
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

    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        // Sweep floor peers (1..quantity) AND auto-joined learned peers; a learned
        // board above the floor must still transition offline when its heartbeats
        // stop, so ensured retries to it stop and the status callback fires.
        if (i >= _quantity && !_learnedPeer[i]) continue;
        if (!_boards[i].online) continue;
        if (now - _boards[i].lastSeenMs > threshold) {
            _boards[i].online = false;
            if (_statusCallback) _statusCallback(i + 1, false);
        }
    }

    // The special peer (NaviCore) sits outside 1..quantity, so the loop above
    // skips it — check it here so its ETM offline transition fires too (and so
    // ensured retries to it stop once it's gone).
    if (_specialPeerID > _quantity && _specialPeerID <= WCB_MAX_BOARDS) {
        int sp = _specialPeerID - 1;
        if (_boards[sp].online && (now - _boards[sp].lastSeenMs > threshold)) {
            _boards[sp].online = false;
            if (_statusCallback) _statusCallback(_specialPeerID, false);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// _findFreePending
//
// Returns the index of a usable slot in _pending[], or -1 when the table is
// saturated with in-flight ensured deliveries. Prefers the first inactive slot.
// If all WCB_PENDING_MAX slots are occupied, reclaims only a slot that is SAFE
// to drop — a best-effort (un-ensured) slot, or an ensured slot that has already
// completed (every expected board acked or gone). A still-outstanding ensured
// slot is NEVER evicted, because dropping it would silently lose a guaranteed
// command that hasn't been delivered yet. Among reclaimable slots, the oldest is
// dropped. If every slot is an outstanding ensured delivery, returns -1 and the
// caller decides how to degrade (see _sendPacket).
//
// MUST be called with _pendingMux held (it is only invoked from _sendPacket,
// which takes the lock across the find + fill so the claim is atomic vs the
// cross-core ACK handler).
// ─────────────────────────────────────────────────────────────────────────────
int WCB_Client::_findFreePending() {
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        if (!_pending[i].active) return i;
    }
    // All slots busy. Reclaim the oldest slot that is safe to drop. Age is
    // (now - sentMs) with unsigned arithmetic, correct across a millis() rollover
    // (entries live for ms, far under the ~49.7-day wrap period).
    unsigned long now    = millis();
    int           victim = -1;
    unsigned long maxAge = 0;
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        bool reclaimable = !_pending[i].ensured || _ensuredComplete(_pending[i]);
        if (!reclaimable) continue;                 // outstanding ensured — must not drop
        unsigned long age = now - _pending[i].sentMs;
        if (victim < 0 || age > maxAge) { maxAge = age; victim = i; }
    }
    if (victim >= 0) _pending[victim].active = false;
    return victim;   // -1 => every slot is an outstanding ensured delivery
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

    // Network-namespace check FIRST — the sender's MAC must belong to THIS
    // network's octet scheme (02:oct2:oct3:00:00:ID, see _buildMACs). The radio
    // delivers ESP-NOW broadcasts/action-frames from foreign networks sharing the
    // channel regardless of their scoped FF:oct2:oct3:FF:FF:FF group, so without
    // this the password was the ONLY gate — and a mis-set local oct2/oct3 would
    // still "see" other networks' boards (false-positive online status) even
    // though it could never address them. Drop anything whose source octets
    // aren't ours. Done before the size routing below so it also gates raw
    // (non-etm) packets handed to _rawPacketCallback. (mac == info->src_addr.)
    if (!mac ||
        mac[0] != 0x02 || mac[1] != _oct2 || mac[2] != _oct3 ||
        mac[3] != 0x00 || mac[4] != 0x00)
        return;

    // Anything that is NOT the standard 252-byte WCB packet (e.g. an
    // application's OTA control/data structs, 55/243 B) goes to the raw-packet
    // hook, if registered, and stops here. This lets a custom protocol share the
    // mesh without forking the receive path. Undersized junk with no hook
    // registered is simply dropped (same as the previous `< sizeof` guard).
    if (len != (int)sizeof(wcb_packet_etm_t)) {
        if (_rawPacketCallback) _rawPacketCallback(mac, data, len);
        return;
    }

    const wcb_packet_etm_t* pkt = (const wcb_packet_etm_t*)data;

    // Password check — reject packets from other WCB networks on the same channel
    if (strncmp(pkt->structPassword, _password, sizeof(pkt->structPassword) - 1) != 0)
        return;

    int senderID = atoi(pkt->structSenderID);
    int targetID = atoi(pkt->structTargetID);

    // Bind the claimed sender id to its real MAC. The octet check above already
    // proved octets 0-4; every WCB and WCB_Client forces its STA MAC's LAST octet
    // to its board number (02:oct2:oct3:00:00:<id>, see _buildMACs), so a packet
    // whose structSenderID disagrees with mac[5] is spoofed or misconfigured.
    // Drop it before it can mark a board online, satisfy an ACK, or auto-join.
    if (senderID >= 1 && senderID <= WCB_MAX_BOARDS && (uint8_t)senderID != mac[5]) return;

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
            // Runs on Core 0 (WiFi/RX task). Serialize against the loop task's
            // slot claim+fill in _sendPacket so we never match/mutate a slot that
            // is being half-written (see _pendingMux). Pure array work — no
            // esp_now/Serial inside the critical section.
            portENTER_CRITICAL(&_pendingMux);
            for (int i = 0; i < WCB_PENDING_MAX; i++) {
                if (!_pending[i].active) continue;
                if (_pending[i].seqNum != pkt->structSequenceNumber) continue;

                if (senderID >= 1 && senderID <= WCB_MAX_BOARDS)
                    _pending[i].ackReceived[senderID - 1] = true;

                if (_pending[i].targetID != WCB_TARGET_BROADCAST)
                    _pending[i].active = false;

                break;
            }
            portEXIT_CRITICAL(&_pendingMux);
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

            // A command from an authenticated sender means we may need to unicast
            // back to it — the ACK above, and any reply the callback sends. If it's
            // an ABOVE-FLOOR sender that isn't a registered peer yet (e.g. a client
            // / relay before auto-join's >=2-advert threshold), esp_now_send to it
            // fails (NOT_FOUND) and the reply is silently lost. Flag it (cheap bool,
            // same discipline as _pendingJoin) so update() registers it as a
            // transient peer on the LOOP task — NEVER add_peer on this RX callback
            // (the documented boot-loop hazard). Floor/special/self and already-
            // learned ids are skipped: they're registered already.
            if (senderID > _quantity && senderID <= WCB_MAX_BOARDS &&
                senderID != _deviceID && senderID != _specialPeerID &&
                senderID != WCB_SPECIAL_ID && !_learnedPeer[senderID - 1])
                _pendingReplyPeer[senderID - 1] = true;

            if (pkt->structCommandIncluded && _commandCallback) {
                char cmd[201];
                strncpy(cmd, pkt->structCommand, sizeof(cmd) - 1);
                cmd[sizeof(cmd) - 1] = '\0';

                // Only strip/verify a CRC suffix when checksums are ENABLED —
                // otherwise a legitimate payload that merely contains the literal
                // "|CRC" (e.g. a command-library or freetext command) would be
                // wrongly truncated or rejected. When enabled, the sender appends
                // the CRC as the tail "|CRCxxxxxxxx" (exactly 8 hex digits), so
                // anchor to the END of the string rather than the first match.
                if (_checksumEnabled) {
                    char*  crcTag = nullptr;
                    size_t len    = strlen(cmd);
                    if (len >= 12) {                        // "|CRC" + 8 hex digits
                        char* cand = cmd + len - 12;
                        if (strncmp(cand, "|CRC", 4) == 0) {
                            bool hex8 = true;
                            for (int h = 0; h < 8 && hex8; h++) {
                                char c = cand[4 + h];
                                if (!((c >= '0' && c <= '9') ||
                                      (c >= 'a' && c <= 'f') ||
                                      (c >= 'A' && c <= 'F'))) hex8 = false;
                            }
                            if (hex8) crcTag = cand;
                        }
                    }
                    if (!crcTag) {
                        // Expected a checksum but the tail isn't a valid one —
                        // likely a ?ETM,CHKSM mismatch between us and the sender.
                        Serial.printf("[WCB_Client] Missing CRC from WCB%d — packet rejected\n",
                                      senderID);
                        break;
                    }
                    uint32_t rxCRC   = (uint32_t)strtoul(crcTag + 4, nullptr, 16);
                    uint32_t calcCRC = _crc32(cmd, (size_t)(crcTag - cmd));
                    if (rxCRC != calcCRC) {
                        Serial.printf("[WCB_Client] CRC mismatch from WCB%d — packet rejected\n",
                                      senderID);
                        break;
                    }
                    *crcTag = '\0';  // Strip the verified "|CRC..." — deliver clean command
                }
                // checksum disabled → deliver exactly as received (no stripping)

                _commandCallback((uint8_t)senderID, cmd);
            }

            break;

        // ── WDP advert ─────────────────────────────────────────────────────────
        // A neighbor announcing itself. Decode its TLVs into the neighbor table
        // and fire onNeighbor. No ACK — adverts are fire-and-forget.
        case WCB_PACKET_WDP:
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS && pkt->structCommandIncluded)
                _handleWdpAdvert((uint8_t)senderID, (const uint8_t*)pkt->structCommand);
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
