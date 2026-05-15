// ─────────────────────────────────────────────────────────────────────────────
//  sbus_reader.h — Custom SBUS frame parser supporting BOTH standard 16-channel
//  AND FrSky's 24-channel "SBUS-24" extended frame format.
//
//  The two variants share:
//    • Header byte 0x0F
//    • Footer byte 0x00
//    • 11-bit channel encoding (same bit-packing for both)
//    • 100 kbaud, 8E2, inverted UART
//
//  They differ only in TOTAL FRAME LENGTH:
//    • SBUS-16:  25 bytes  = 0x0F + 22 data + 1 flags + 0x00
//    • SBUS-24:  36 bytes  = 0x0F + 33 data + 1 flags + 0x00
//
//  Reference: SBUSController.ino buildSbusFrame() / decodeSbusFrame()
//
//  Auto-detection method: timestamp-based gap detection.  After every byte
//  the inter-byte time is checked.  If the gap exceeds 1ms (frames are sent
//  every ~7-14ms with a much shorter inter-byte time within a frame), we
//  consider the buffer complete and parse based on its accumulated length.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

class SbusReader {
public:
  static constexpr uint8_t SBUS_HEADER       = 0x0F;
  static constexpr uint8_t SBUS_FOOTER       = 0x00;
  static constexpr int     FRAME_LEN_16      = 25;
  static constexpr int     FRAME_LEN_24      = 36;
  static constexpr int     MAX_CHANNELS      = 24;
  static constexpr unsigned long INTER_FRAME_GAP_US = 1500;   // 1.5 ms — comfortably less than typical 3 ms gap, more than intra-frame byte spacing

  uint16_t channels[MAX_CHANNELS];
  uint8_t  detectedChCount = 0;   // 16 or 24 after first valid frame; 0 if none yet
  uint8_t  detectedFrameLen = 0;  // 25 or 36
  bool     failsafe = false;
  bool     lostFrame = false;
  unsigned long lastValidFrameMs = 0;

  void begin(HardwareSerial* uart, int rxPin, int txPin) {
    uart_ = uart;
    // Standard SBUS: 100k baud, 8E2, inverted.  ESP32 HardwareSerial::begin
    // accepts the invert flag as the 6th argument.
    uart_->begin(100000, SERIAL_8E2, rxPin, txPin, true);
    bufIdx_ = 0;
    inFrame_ = false;
    lastByteUs_ = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) channels[i] = 992;  // center
  }

  // ── SBUS OUT passthrough sink ────────────────────────────────────────────
  // Pass a Stream pointer (e.g. SoftwareSerial configured for 100k 8E2 inverted
  // TX) here to enable byte-streaming SBUS passthrough. Every byte read from
  // the SBUS RX UART will be written to this sink as it arrives — typical
  // added latency is ~110 µs (one byte time) end-to-end.
  //
  // No-op if sink is null. Call before/after begin(); takes effect immediately.
  void setPassthroughSink(Stream* sink) { sink_ = sink; }

  // Call frequently (e.g. each main loop iteration).
  // Returns true exactly once per successfully-parsed frame.
  bool read() {
    if (!uart_) return false;
    unsigned long nowUs = micros();
    bool gotFrame = false;

    while (uart_->available()) {
      uint8_t b = uart_->read();

      // Byte-streaming passthrough — tee the byte to the SBUS OUT sink BEFORE
      // any parsing work, so downstream sees it as close to real-time as the
      // sink allows. SoftwareSerial.write() is blocking at 100k baud (~110 µs
      // per byte) which naturally rate-limits us to the SBUS line speed.
      if (sink_) sink_->write(b);

      unsigned long sinceLastByte = nowUs - lastByteUs_;
      lastByteUs_ = nowUs;

      // Gap-based delimiter: if too much time elapsed since the previous byte,
      // the previous frame must have ended.  Try to parse it before consuming
      // the new byte as the start of a new frame.
      if (inFrame_ && sinceLastByte > INTER_FRAME_GAP_US && bufIdx_ > 0) {
        gotFrame = tryParseAndReset() || gotFrame;
      }

      // Frame must start with the SBUS header byte.
      if (!inFrame_) {
        if (b == SBUS_HEADER) {
          buf_[0] = b;
          bufIdx_ = 1;
          inFrame_ = true;
        }
        continue;
      }

      buf_[bufIdx_++] = b;

      // Buffer-overflow guard — should never hit if framing works.
      if (bufIdx_ >= FRAME_LEN_24 + 4) {
        inFrame_ = false;
        bufIdx_ = 0;
        continue;
      }
    }

    // Also flush at end if a frame is sitting complete in the buffer and we
    // haven't seen more bytes within the gap window.
    if (inFrame_ && bufIdx_ > 0 && (micros() - lastByteUs_) > INTER_FRAME_GAP_US) {
      gotFrame = tryParseAndReset() || gotFrame;
    }

    return gotFrame;
  }

private:
  HardwareSerial* uart_ = nullptr;
  Stream*  sink_ = nullptr;             // optional SBUS OUT passthrough sink
  uint8_t  buf_[FRAME_LEN_24 + 4];
  int      bufIdx_ = 0;
  bool     inFrame_ = false;
  unsigned long lastByteUs_ = 0;

  // Returns true on a successful parse, resets state regardless.
  bool tryParseAndReset() {
    bool ok = false;
    if (bufIdx_ == FRAME_LEN_16 && buf_[0] == SBUS_HEADER && buf_[FRAME_LEN_16 - 1] == SBUS_FOOTER) {
      decodeFrame(buf_, 16);
      detectedChCount   = 16;
      detectedFrameLen  = 25;
      lastValidFrameMs  = millis();
      ok = true;
    } else if (bufIdx_ == FRAME_LEN_24 && buf_[0] == SBUS_HEADER && buf_[FRAME_LEN_24 - 1] == SBUS_FOOTER) {
      decodeFrame(buf_, 24);
      detectedChCount   = 24;
      detectedFrameLen  = 36;
      lastValidFrameMs  = millis();
      ok = true;
    }
    // Otherwise: malformed / partial — drop silently
    inFrame_ = false;
    bufIdx_ = 0;
    return ok;
  }

  // Bit-unpacking is identical for both frame lengths, just iterate
  // chcnt channels through the same 11-bit shift pattern.
  void decodeFrame(const uint8_t* frame, int chcnt) {
    for (int i = 0; i < chcnt; i++) {
      int b = i * 11;
      uint16_t raw = 0;
      raw  = (uint16_t)(frame[1 + b / 8])     >> (b % 8);
      raw |= (uint16_t)(frame[1 + b / 8 + 1]) << (8 - b % 8);
      if ((b % 8) > 5) {
        raw |= (uint16_t)(frame[1 + b / 8 + 2]) << (16 - b % 8);
      }
      channels[i] = raw & 0x07FF;   // 11-bit mask
    }
    // Zero out any channels above the detected count for safety
    for (int i = chcnt; i < MAX_CHANNELS; i++) channels[i] = 992;
    // Flags byte sits just before the footer
    uint8_t flags = frame[(chcnt == 16 ? FRAME_LEN_16 : FRAME_LEN_24) - 2];
    lostFrame = (flags & 0x04) != 0;
    failsafe  = (flags & 0x08) != 0;
  }
};
