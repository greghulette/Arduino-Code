#pragma once

#include <Arduino.h>
#include <Stream.h>

// =============================================================================
// WCBStream
//
// A Stream adapter that intercepts bytes written by the Pololu Maestro library
// (or any other library that writes to a Stream) and forwards them wirelessly
// over the WCB ESP-NOW network via sendRaw() instead of a physical serial port.
//
// ── How it works ─────────────────────────────────────────────────────────────
// The Pololu Maestro library accepts any Stream object in its constructor — it
// doesn't care whether that stream is a UART, SoftwareSerial, or a custom class.
// WCBStream presents itself as a Stream, so the Maestro library writes bytes to
// it exactly as it would to Serial1. WCBStream buffers those bytes and, once the
// inter-frame gap (gap_ms milliseconds of silence) signals the end of a packet,
// calls wcb.sendRaw() to forward the complete packet wirelessly.
//
// ── No hardware changes required ─────────────────────────────────────────────
// This eliminates the need for a physical TX→RX loopback wire. The interception
// happens entirely in software before bytes ever reach a UART pin.
//
// ── Usage ────────────────────────────────────────────────────────────────────
//   // Replace:  MiniMaestro maestro(Serial1);
//   // With:
//   WCBStream maestroStream(wcb, /*target_wcb=*/1);
//   MiniMaestro maestro(maestroStream);
//
//   void setup() { wcb.begin(); }
//   void loop()  { wcb.update(); maestro.setTarget(0, 6000); }
//
// WCBStream self-registers with the WCB_Client instance so wcb.update() drives
// the flush automatically — no extra calls needed in loop().
//
// ── gap_ms ───────────────────────────────────────────────────────────────────
// The Maestro library writes all bytes for one command in a tight synchronous
// burst (typically 2–6 bytes). A gap_ms of 2 ms reliably falls between commands
// rather than mid-command. Increase it only if you observe partial packets.
// =============================================================================

// Forward declaration — avoids a circular include with WCB_Client.h
class WCB_Client;

class WCBStream : public Stream {
public:

    // ── Construction ──────────────────────────────────────────────────────────

    // Create a WCBStream. Automatically registers with the WCB_Client singleton
    // so wcb.update() drives the flush — no reference to wcb needed here.
    //
    // target_wcb  : WCB number to unicast to (1–WCB_MAX_BOARDS), OR
    //               the constant  broadcast  (0) to send to all WCBs at once —
    //               every WCB with Kyber_Remote will forward bytes to its Maestro.
    // target_port : serial port on the target WCB (1–5). Ignored for broadcast.
    // gap_ms      : silence in ms that signals end of a packet (default 2 ms).
    //
    // Examples:
    //   WCBStream maestro(broadcast);     // all WCBs with Kyber_Remote
    //   WCBStream maestro(2, 1);          // unicast to WCB2 Serial1
    WCBStream(uint8_t target_wcb, uint8_t target_port = 0, uint16_t gap_ms = 2);

    // ── Stream interface ──────────────────────────────────────────────────────

    // Called by the Pololu library (or any Stream user) for every byte it wants
    // to transmit. Bytes are appended to the internal buffer. If the buffer is
    // full (200 bytes) the byte is silently dropped — Maestro commands are at
    // most ~20 bytes so this limit is never reached in normal use.
    size_t write(uint8_t byte) override;

    // WCBStream is write-only — reading is not supported.
    // These return 0 / -1 to satisfy the pure-virtual Stream interface.
    int available() override { return 0;  }
    int read()      override { return -1; }
    int peek()      override { return -1; }

    // ── Internal flush ────────────────────────────────────────────────────────

    // Called automatically by WCB_Client::update() on every loop iteration.
    // Checks whether the inter-frame gap has elapsed since the last write().
    // If so, sends the buffered bytes via wcb.sendRaw() and resets the buffer.
    // You do not need to call this manually.
    void tick();

private:
    uint8_t       _target;          // Target WCB number for sendRaw()
    uint8_t       _port;            // Target serial port on the WCB (1–5)
    uint16_t      _gapMs;           // Inter-frame gap threshold in milliseconds
    uint8_t       _buf[177];        // Byte buffer — capped at firmware raw chunk limit
    size_t        _len;             // Number of bytes currently in the buffer
    unsigned long _lastWriteMs;     // millis() timestamp of the most recent write()
};
