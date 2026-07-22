#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
//  WcbHcr — HCR (Human-Cyborg Relations) Vocalizer device-wire formatter
//                                                              (WcbCmd Phase 4)
//
//  Turns a numeric (fn, chan, track) action into the HCR's native serial wire
//  string — short ASCII frames like  <SH50,QEH,QT>\n  ,  <CA0003,QPA>\n ,
//  <PVV50>\n  — the exact bytes the HCR vocalizer consumes. This is the ONE
//  place the wire format lives, so an HCR behaves identically whether:
//    • a WCB_Client host (NaviCore) formats it and writes a local UART, or
//    • the command rides the mesh as ;H,FN,<fn>,<chan>,<track> and a WCB formats it.
//
//  fn / chan / track match the Body-Controller HCRFunction() convention (shared
//  rc_config.h), so a saved action means the same thing on every board:
//     2  SetEmotion(e,v)      3  Trigger(e,v)     4  Stimulate(e,v)
//     5  Overload()           6  Muse()           7  Muse(min,max)  (min=chan,max=track)
//     8  Stop (all A/V/B+emote)                   9  StopEmote()
//    10  OverrideEmotions(v)  (v=chan)           11  ResetEmotions()
//    13  SetMuse(v)           (v=track)          14  PlayWAV(ch,track)
//    16  StopWAV(ch)                             17  SetVolume(ch,vol)  (chan 3 = ALL)
//    18  VolumeUpAll(step)    19  VolumeDownAll(step)   (track = step; 0 = default 5)
//
//  Emotion chan: 0=H 1=S 2=M 3=C (chan 4 is rejected — use fn 5 for Overload). Audio ch:
//  0=V 1=A 2=B. Query/poll fns (1, status reads) are NOT here — this is the
//  fire-and-forget WRITE path. Status RX parsing stays in each firmware (the WCB
//  keeps the HCRVocalizer library for that; NaviCore is fire-and-forget).
//
//  Extracted verbatim from NaviCore's hcrFormatCommand()/hcrNormalizeAction(),
//  which is byte-identical to the WcbCmd golden vectors and to the WCB's
//  HCRVocalizer output for every fn EXCEPT fn 8 (Stop) — see docs/WCB-INTEGRATION.md.
// ─────────────────────────────────────────────────────────────────────────────

class HcrCodec {
public:
  // Readable names for the numeric codes (thin layer — these ARE the integers, so
  // they pass straight into format()/emit() with no cast and no wire-byte change):
  //   hcr.emit(out, HcrCodec::Stimulate, HcrCodec::Happy, 50);   // == emit(out, 4, 0, 50)
  enum Fn : uint8_t {
    SetEmotion = 2, Trigger = 3, Stimulate = 4, Overload = 5, Muse = 6, MuseGap = 7,
    StopAll = 8, StopEmote = 9, OverrideEmotions = 10, ResetEmotions = 11, SetMuse = 13,
    PlayWAV = 14, StopWAV = 16, SetVolume = 17, VolumeUpAll = 18, VolumeDownAll = 19,
  };
  enum Emotion : uint8_t { Happy = 0, Sad = 1, Mad = 2, Scared = 3 };   // emotion chan
  enum Audio   : uint8_t { Vocalizer = 0, ChA = 1, ChB = 2, AllChannels = 3 };  // audio ch (17 uses 3=ALL)

  // Validate + normalize a (fn,chan,track) triplet IN PLACE: range-checks per fn.
  // Returns false for an unknown fn or out-of-range params (caller skips it).
  // Static — the same one source of truth for both transports.
  static bool normalize(uint8_t &fn, int &chan, int &track);

  // Format the HCR device wire string for one action: "<...>\n" (some fns emit
  // several frames, e.g. VolumeAll). Returns "" for a rejected/unknown action.
  // fn 17 (absolute) and 18/19 (relative step) update the per-channel volume
  // shadow — see setVol()/getVol().
  String format(uint8_t fn, int chan, int track);

  // format() + write the bytes to `out`. Returns true iff bytes were written.
  bool emit(Stream &out, uint8_t fn, int chan, int track);

  // Per-channel commanded-volume shadow (0=V 1=A 2=B). The relative VolumeUp/Down
  // path (fn 18/19) has no readback, so it steps this shadow and emits absolute
  // <PVx..> sets from it. Seed/reset it to keep a firmware's state — and the two
  // library copies — in agreement. Clamped 0..99.
  void setVol(int ch, int v);
  int  getVol(int ch) const;

private:
  int8_t _vol[3] = { 50, 50, 50 };   // V, A, B — matches NaviCore's hcrLocalVol seed
};
