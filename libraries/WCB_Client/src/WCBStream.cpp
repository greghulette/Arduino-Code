#include "WCBStream.h"
#include "WCB_Client.h"
#include <stdarg.h>

// Non-blocking debug line. tick() runs from WCB_Client::update() on the caller's
// hot path; a plain Serial.printf() there freezes the entire sketch whenever
// no host is draining the USB serial port (the controller stops sending ESP-NOW
// heartbeats and the WCBs declare it offline). This formats into a small stack
// buffer and writes ONLY if the TX buffer has room — otherwise the line is
// dropped. It can never block, so a missing or stalled host cannot stall tick().
static void wcbStreamLog(const char* fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    if (Serial.availableForWrite() >= n) Serial.write((const uint8_t*)buf, (size_t)n);
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
//
// Stores configuration and self-registers with the WCB_Client so that
// wcb.update() will automatically call tick() on every loop iteration.
// ─────────────────────────────────────────────────────────────────────────────
WCBStream::WCBStream(uint8_t target_wcb, uint8_t target_port, uint16_t gap_ms)
    : _target(target_wcb),
      _port(target_port),
      _gapMs(gap_ms),
      _len(0),
      _lastWriteMs(0)
{
    // Self-register with the singleton so WCB_Client::update() drives tick().
    // WCB_Client must be declared before WCBStream at global scope — its
    // constructor sets _instance, which is guaranteed to be non-null here.
    if (WCB_Client::instance())
        WCB_Client::instance()->_registerWCBStream(this);
}

// ─────────────────────────────────────────────────────────────────────────────
// write
//
// Called by the Pololu Maestro library (or any Stream consumer) for each byte
// it wants to transmit. Bytes accumulate in _buf[]. The buffer is never flushed
// here — flushing happens in tick() driven by the inter-frame gap timer so that
// a complete multi-byte command is always sent as one atomic sendRaw() call.
// ─────────────────────────────────────────────────────────────────────────────
size_t WCBStream::write(uint8_t byte) {
    if (_len < sizeof(_buf)) {
        _buf[_len++] = byte;
    }
    // Reset the gap timer on every byte — flush happens only after silence
    _lastWriteMs = millis();
    return 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// tick
//
// Called by WCB_Client::update() on every loop iteration. Checks whether the
// inter-frame gap has elapsed since the last byte was written. If so, the
// accumulated buffer represents a complete Maestro command (or sequence of
// commands) and is forwarded to the target WCB via sendRaw().
//
// Why gap detection works here:
//   The Pololu library writes all bytes for one command in a tight synchronous
//   burst — for example, setTarget() writes 2–4 bytes with no delay between
//   them. After the burst there is silence until the next command is called.
//   A gap_ms of 2 ms is far longer than the intra-burst byte spacing (which
//   is zero — all writes happen in the same CPU instruction sequence) but far
//   shorter than the time between consecutive commands in a typical sketch.
// ─────────────────────────────────────────────────────────────────────────────
void WCBStream::tick() {
    if (_len > 0 && (millis() - _lastWriteMs) >= _gapMs) {
        WCB_Client* wcb = WCB_Client::instance();
        if (!wcb) { _len = 0; return; }
        bool ok;
        if (_target == 0) {
            ok = wcb->sendKyber(_buf, _len);
            wcbStreamLog("[WCBStream] Broadcast (Kyber) %d bytes — %s\n", _len, ok ? "OK" : "FAIL");
        } else {
            ok = wcb->sendRaw(_target, _port, _buf, _len);
            wcbStreamLog("[WCBStream] Flushing %d bytes to WCB%d port %d — %s\n", _len, _target, _port, ok ? "OK" : "FAIL");
        }
        _len = 0;
    }
}
