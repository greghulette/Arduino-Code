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
// WCBStream self-registers with the WCBClient instance so wcb.update() drives
// the flush automatically — no extra calls needed in loop().
//
// ── gap_ms ───────────────────────────────────────────────────────────────────
// The Maestro library writes all bytes for one command in a tight synchronous
// burst (typically 2–6 bytes). A gap_ms of 2 ms reliably falls between commands
// rather than mid-command. Increase it only if you observe partial packets.
// =============================================================================

// Forward declaration — avoids a circular include with WCBClient.h
class WCBClient;

class WCBStream : public Stream {
public:

    // ── Construction ──────────────────────────────────────────────────────────

    // Create a WCBStream and register it with the given WCBClient.
    // After construction, wcb.update() automatically checks this stream for
    // buffered bytes and forwards them via sendRaw() when the gap elapses.
    //
    // client     : the WCBClient instance managing the ESP-NOW network
    // target_wcb : WCB number that will receive and forward the raw bytes
    //              (must have KYBER,REMOTE configured on its serial port)
    // gap_ms     : inter-frame gap in milliseconds — time of silence after the
    //              last written byte before the buffer is sent as one packet
    // client      : the WCBClient instance managing the ESP-NOW network
    // target_wcb  : WCB number that will receive and forward the raw bytes
    // target_port : serial port on that WCB (1–5) connected to the Maestro
    // gap_ms      : inter-frame gap in ms before the buffer is flushed
    WCBStream(WCBClient& client, uint8_t target_wcb, uint8_t target_port,
              uint16_t gap_ms = 2);

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

    // Called automatically by WCBClient::update() on every loop iteration.
    // Checks whether the inter-frame gap has elapsed since the last write().
    // If so, sends the buffered bytes via wcb.sendRaw() and resets the buffer.
    // You do not need to call this manually.
    void tick();

private:
    WCBClient&    _client;          // Reference to the owning WCBClient
    uint8_t       _target;          // Target WCB number for sendRaw()
    uint8_t       _port;            // Target serial port on the WCB (1–5)
    uint16_t      _gapMs;           // Inter-frame gap threshold in milliseconds
    uint8_t       _buf[177];        // Byte buffer — capped at firmware raw chunk limit
    size_t        _len;             // Number of bytes currently in the buffer
    unsigned long _lastWriteMs;     // millis() timestamp of the most recent write()
};
