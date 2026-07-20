#pragma once
#include <Arduino.h>
#include "WcbHcr.h"

// ─────────────────────────────────────────────────────────────────────────────
//  WcbHcrFade — non-blocking per-channel volume fade for the HCR audio channels
//                                                        (WcbCmd Phase 4, shared)
//
//  The HCR device has no native fade command. This synthesizes a smooth ramp by
//  stepping SetVolume over time — the SAME code the WCB firmware and a WCB_Client
//  host (NaviCore) run, so an HCR fades identically whichever board drives it.
//
//  Every write goes THROUGH the injected HcrCodec (fn 17 SetVolume, fn 16 StopWAV),
//  so the codec's per-channel volume shadow stays the single source of truth and the
//  bytes are identical to a normal SetVolume. Byte-identical to the WCB's former
//  inline fade (WCB_HCR.cpp hcrStartFade/hcrStepFades): 150 ms step, linear interp.
//
//  Drive it:  start() on a ;H,FADEIN / ;H,FADEOUT (or a play-with-fadein);
//             tick()  every loop() pass. Both use millis() internally.
//
//  Audio ch: 0=V(ocalizer) 1=A 2=B. Fade only applies to the audio channels.
// ─────────────────────────────────────────────────────────────────────────────
class HcrFade {
public:
  // Ramp audio `ch` (0-2) from->to over durSec, writing through `codec` to `out`.
  //  durSec <= 0 : instant — one SetVolume (+ StopWAV/restore if stopAtEnd).
  //  stopAtEnd   : fade-out — on reaching the end, StopWAV(ch) then restore to
  //                restoreTo. Supersedes any in-flight fade on the same channel.
  void start(HcrCodec &codec, Stream &out, int ch, int from, int to,
             int durSec, bool stopAtEnd, int restoreTo);

  // Advance all active fades; emit any step that is due. Call once per loop().
  void tick(HcrCodec &codec, Stream &out);

  void cancel(int ch)       { if (ch >= 0 && ch <= 2) _f[ch].active = false; }
  bool active(int ch) const { return (ch >= 0 && ch <= 2) && _f[ch].active; }

private:
  struct Fade {
    bool     active    = false;
    int      from      = 0, to = 0;
    uint32_t startMs   = 0, durMs = 0, nextStep = 0;
    int      lastSent  = -1;
    bool     stopAtEnd = false;
    int      restoreTo = 0;
  };
  Fade _f[3];                              // by audio channel (0=V,1=A,2=B)
  static const uint16_t STEP_MS = 150;     // ramp granularity — matches the WCB
};
