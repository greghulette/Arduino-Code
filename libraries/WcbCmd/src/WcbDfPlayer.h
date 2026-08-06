#pragma once
#include <Arduino.h>

// ── DFPlayer Mini command translator ───────────────────────────────────────────
// The WCB ";D,..." commands drive a DFRobot DFPlayer Mini (and the many YX5300 /
// MH2024K clones) over serial. This module turns them into the player's native
// 10-byte frame protocol, identically on any board, so the same ";D" string drives
// a DFPlayer whether reached over ESP-NOW (a WCB parses it) or out a host-local
// serial port (the host parses it).
//
// Wire format — every command is exactly 10 bytes:
//     7E FF 06 <CMD> <ACK> <PARAM_HI> <PARAM_LO> <CK_HI> <CK_LO> EF
//   checksum = -(sum of bytes[1..6]) as uint16, big-endian.
// ACK is always 0 here: the reply frame is not awaited, matching the fire-and-
// forget style of the other WcbCmd modules (the caller is on a real-time path and
// cannot block). Reference implementation: DFRobotDFPlayerMini.cpp sendStack() /
// calculateCheckSum().
//
// Why this is NOT the DFRobot library: that library's sendStack() calls delay(10)
// after every frame with ACK off, and blocks outright with ACK on. Neither is
// survivable on a board whose loop() also re-emits SBUS every ~9 ms. Frames here
// are written and forgotten — at the DFPlayer's fixed 9600 baud a 10-byte frame
// takes ~10.4 ms to shift out, so two commands issued back-to-back in one loop
// pass are naturally spaced by the UART itself and need no software delay.
//
// TWO DEVICE DIFFERENCES from Mp3Codec that are deliberate, not oversights:
//   1. Volume is NOT re-sent before every play. The DFPlayer keeps its volume
//      across tracks (the MP3 Trigger does not), so a redundant volume frame would
//      only add a second frame to pace behind.
//   2. The volume scale is the DEVICE's: 0 = silent .. 30 = loudest. That is the
//      INVERSE of the MP3 Trigger's 0 = loudest .. 64 = inaudible. Values are never
//      translated between the two — each action type carries its own device's
//      native scale.
//
// A DFPlayer needs ~1.5-3 s after power-on before it accepts commands. Nothing here
// gates on that (the codec has no clock); a host firing sound at boot should hold
// off itself.
//
// The host keeps its OWN "is a DFPlayer configured / which port / local-vs-remote"
// decision and calls handle() only when the command should run locally on `out`.
class DfPlayerCodec {
public:
    // Bind the output stream (the UART the DFPlayer is wired to) + optional diag.
    // Call again if the resolved port changes.
    void begin(Stream& out, Print* diag = nullptr) { _out = &out; _diag = diag; }

    // Parse + emit one ";D" command body:
    //   "PLAY,5[,ONFIN,key|,key]"   "FOLDER,1,5[,...]"   "MP3FOLDER,3[,...]"
    //   "STOP" "NEXT" "PREV" "PAUSE" "RESUME" "RANDOM" "RESET"
    //   "VOL,20"  "VOLUP"  "VOLDN"   "EQ,2"   "DEVICE,2"
    //   "LOOP,5"  "LOOPALL,1"  "LOOPFOLDER,1"   "STATUS"
    // Tolerates an optional leading 'D'/'d' (+ comma). Returns false on an unknown
    // or out-of-range command (nothing emitted).
    bool handle(const char* body);

    // Pump the DFPlayer's RX responses (call once per loop tick). Fires onFinished
    // (0x3D track-done, with the pending ONFIN key) and onError (0x40). Harmless on
    // a write-only stream (available() == 0).
    void poll();

    uint8_t volume() const       { return _volume; }
    void    setVolume(uint8_t v) { _volume = (v > DFP_VOL_MAX) ? DFP_VOL_MAX : v; }  // shadow only, emits nothing (config load)

    static const uint8_t DFP_VOL_MAX = 30;   // device ceiling; 30 = loudest

    // Hooks — the host wires these; a host that doesn't need them leaves them null.
    void (*onVolumeChanged)(uint8_t vol) = nullptr;  // WCB -> saveDFPSettings()
    void (*onFinished)(const char* key)  = nullptr;  // WCB -> recallCommandSlot()  (ONFIN)
    void (*onError)()                    = nullptr;  // WCB -> onErrCmd

private:
    void frame(uint8_t cmd, uint16_t param = 0);
    void setPending(const char* key);

    Stream* _out  = nullptr;
    Print*  _diag = nullptr;
    uint8_t _volume = 20;              // sane mid-loud default on a 0..30 scale
    char    _pendingCb[40] = {0};      // ONFIN callback key, held until 0x3D

    // RX-parse state — collect whole 10-byte frames, resyncing on 0x7E
    uint8_t _rx[10];
    uint8_t _rxLen = 0;
};
