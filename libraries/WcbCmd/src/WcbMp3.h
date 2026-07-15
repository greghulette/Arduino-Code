#pragma once
#include <Arduino.h>

// ── MP3 Trigger (SparkFun v2.x) command translator ─────────────────────────────
// The WCB ";A,..." commands drive a SparkFun MP3 Trigger over serial. This module
// turns them into the trigger's native single-/two-byte serial protocol, identically
// on any board, so the same ";A" string drives an MP3 Trigger whether reached over
// ESP-NOW (a WCB parses it) or out a host-local serial port (the host parses it).
//
// Stateful: it holds the volume shadow (sent before every play, 0=loudest..64=quiet)
// and the pending ONFIN callback key, and it can pump the trigger's RX responses.
//
// Reference (byte-identical): WCB WCB_MP3.cpp (processMP3AudioCommand + sendMP3Raw +
// processMP3Responses); NaviCore mp3SendLocal/mp3Raw.
//
// The host keeps its OWN "is MP3 configured / which port / local-vs-remote" decision
// and calls handle() only when the command should run locally on `out`.
class Mp3Codec {
public:
    // Bind the output stream (the UART the MP3 Trigger is wired to) + optional diag.
    // Call again if the resolved port changes.
    void begin(Stream& out, Print* diag = nullptr) { _out = &out; _diag = diag; }

    // Parse + emit one ";A" command body:
    //   "PLAY,5[,ONFIN,key|,key]"  "PLAYFS,3[...]"  "STOP" "NEXT" "PREV"
    //   "VOL,25"  "VOLUP"  "VOLDN"  "COUNT"  "VER"
    // Tolerates an optional leading 'A'/'a' (+ comma). Returns false on an unknown or
    // out-of-range command (nothing emitted).
    bool handle(const char* body);

    // Pump the MP3 Trigger's RX responses (call once per loop tick). Fires onFinished
    // (track done, with the pending ONFIN key) and onError. Harmless on a write-only
    // stream (available() == 0).
    void poll();

    uint8_t volume() const       { return _volume; }
    void    setVolume(uint8_t v) { _volume = v; }   // set the shadow WITHOUT emitting (config load)

    // Hooks — the host wires these; a host that doesn't need them leaves them null.
    void (*onVolumeChanged)(uint8_t vol) = nullptr;  // WCB -> saveMP3Settings()
    void (*onFinished)(const char* key)  = nullptr;  // WCB -> recallCommandSlot()  (ONFIN)
    void (*onError)()                    = nullptr;  // WCB -> onErrCmd

private:
    void raw(uint8_t b1, int b2 = -1);
    void setPending(const char* key);

    Stream* _out  = nullptr;
    Print*  _diag = nullptr;
    uint8_t _volume = 20;              // WCB default
    char    _pendingCb[40] = {0};      // ONFIN callback key, held until 'X'

    // RX-parse state (mirrors WCB processMP3Responses)
    bool    _inTrigger   = false;
    uint8_t _triggerBytes = 0;
    bool    _inStatus    = false;
    char    _statusBuf[48];
    uint8_t _statusLen   = 0;
};
