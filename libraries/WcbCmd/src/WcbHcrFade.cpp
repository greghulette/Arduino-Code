#include "WcbHcrFade.h"

// Mirror of the WCB firmware's hcrStartFade()/hcrStepFades() (WCB_HCR.cpp), but every
// write goes through HcrCodec so the codec's volume shadow stays authoritative and the
// bytes match a normal SetVolume. Volume range is the shared 0-99 convention (the codec's).
//
// NOTE vs the old WCB inline fade: codec.emit(17,...) updates the codec's per-channel
// shadow on every step, so mid-fade the shadow reflects the CURRENT ramp level (the old
// firmware left its shadow at the pre-fade value until the ramp finished). The emitted
// step sequence is identical either way; only a command that arrives mid-fade sees the
// (arguably more correct) live level. The ramp math uses f.from/f.to, not the shadow.

void HcrFade::start(HcrCodec &codec, Stream &out, int ch, int from, int to,
                    int durSec, bool stopAtEnd, int restoreTo) {
  if (ch < 0 || ch > 2) return;
  from      = constrain(from,      0, 99);
  to        = constrain(to,        0, 99);
  restoreTo = constrain(restoreTo, 0, 99);
  Fade &f = _f[ch];
  f.active = false;                        // supersede any in-flight fade on this channel

  if (durSec <= 0) {                       // instant — one write, no anchor
    codec.emit(out, 17, ch, to);           // SetVolume(ch, to)
    if (stopAtEnd) {
      codec.emit(out, 16, ch, 0);          // StopWAV(ch)
      codec.emit(out, 17, ch, restoreTo);  // restore
    }
    return;
  }

  // Anchor the start point only if it differs from the shadow — avoids an extra
  // packet when `from` already matches what the codec last commanded.
  if (from != codec.getVol(ch)) codec.emit(out, 17, ch, from);

  f.from = from; f.to = to;
  f.startMs = millis(); f.durMs = (uint32_t)durSec * 1000UL;
  f.nextStep = 0; f.lastSent = from;
  f.stopAtEnd = stopAtEnd; f.restoreTo = restoreTo;
  f.active = true;
}

void HcrFade::tick(HcrCodec &codec, Stream &out) {
  uint32_t now = millis();
  for (int ch = 0; ch < 3; ch++) {
    Fade &f = _f[ch];
    if (!f.active) continue;

    uint32_t elapsed = now - f.startMs;
    if (elapsed >= f.durMs) {              // done
      codec.emit(out, 17, ch, f.to);
      if (f.stopAtEnd) {
        codec.emit(out, 16, ch, 0);        // StopWAV
        codec.emit(out, 17, ch, f.restoreTo);
      }
      f.active = false;
      continue;
    }

    if (now < f.nextStep) continue;
    f.nextStep = now + STEP_MS;
    int v = f.from + (int)((long)(f.to - f.from) * (long)elapsed / (long)f.durMs);
    v = constrain(v, 0, 99);
    if (v != f.lastSent) { codec.emit(out, 17, ch, v); f.lastSent = v; }
  }
}
