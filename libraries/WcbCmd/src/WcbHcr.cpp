#include "WcbHcr.h"

// Extracted verbatim from NaviCore's hcrNormalizeAction()/hcrFormatCommand()
// (NaviCore.ino), the canonical hand-rolled reimplementation of the HCR wire
// protocol. Keep this byte-identical to that source AND to the golden vectors.

bool HcrCodec::normalize(uint8_t &fn, int &chan, int &track) {
  switch (fn) {
    case 2:   // SetEmotion(e, v)
      return (chan >= 0 && chan <= 3 && track >= 0 && track <= 99);
    case 3:   // Trigger — same payload as Stimulate
    case 4:   // Stimulate(e, v)
      if (chan < 0 || chan > 4 || track < 0 || track > 99) return false;
      if (chan == 4) { fn = 5; chan = 0; track = 0; }   // emotion 4 = Overload shortcut
      return true;
    case 5: case 6: case 8: case 9: case 11:             // no-param functions
      return true;
    case 7:   // Muse(min, max) — auto-muse gap in seconds (min=chan, max=track)
      return (chan >= 0 && chan <= 99 && track >= 0 && track <= 99);
    case 10:  // OverrideEmotions(v) — v=chan (0/1)
      return (chan >= 0 && chan <= 1);
    case 13:  // SetMuse(v) — v=track (0/1)
      return (track >= 0 && track <= 1);
    case 14:  // PlayWAV(ch, track)
      return (chan >= 0 && chan <= 2 && track >= 0 && track <= 9999);
    case 16:  // StopWAV(ch)
      return (chan >= 0 && chan <= 2);
    case 17:  // SetVolume(ch, vol). chan 3 = ALL (V+A+B).
      return (chan >= 0 && chan <= 3 && track >= 0 && track <= 99);
    case 18:  // VolumeUpAll(step)   — track = step (0 = default 5)
    case 19:  // VolumeDownAll(step) — chan unused
      return (track >= 0 && track <= 99);
    default:
      return false;
  }
}

void HcrCodec::setVol(int ch, int v) {
  if (ch < 0 || ch > 2) return;
  if (v < 0) v = 0; else if (v > 99) v = 99;
  _vol[ch] = (int8_t)v;
}

int HcrCodec::getVol(int ch) const {
  return (ch >= 0 && ch <= 2) ? _vol[ch] : -1;
}

String HcrCodec::format(uint8_t fn, int chan, int track) {
  static const char emoteprefix[] = "HSMC";   // HAPPY / SAD / MAD / sCared
  static const char audioprefix[] = "VAB";     // Vocalizer / A / B
  if (!normalize(fn, chan, track)) return "";  // ranges + Overload shortcut

  String inner;
  switch (fn) {
    case 2:   // SetEmotion(e, v)
      inner = String("O") + emoteprefix[chan] + String(track) + ",QE" + emoteprefix[chan];
      break;
    case 3:   // Trigger — same payload as Stimulate
    case 4:   // Stimulate(e, v)  (chan==4 already normalized to fn 5)
      inner = String("S") + emoteprefix[chan] + String(track) + ",QE" + emoteprefix[chan] + ",QT";
      break;
    case 5:  inner = "SE,QT";          break;  // Overload
    case 6:  inner = "MM";             break;  // single Muse
    case 7:  inner = String("MN") + chan + ",MX" + track; break;  // Muse(min,max) gap
    case 8:  // Stop (all audio + emote) — 4 frames = StopEmote + StopWAV V/A/B, matching
             // HCRVocalizer::Stop() exactly (the canonical form; see docs/WCB-INTEGRATION.md).
      return "<PSV,QT>\n<PSV,QPV>\n<PSA,QPA>\n<PSB,QPB>\n";
    case 9:  inner = "PSV,QT";         break;  // StopEmote
    case 10: inner = String("O") + chan + ",QO"; break;  // OverrideEmotions(v) — "O<v>" (digit)
    case 11: inner = "OR,QE";          break;  // ResetEmotions
    case 13: inner = String("M") + track + ",QM"; break;  // SetMuse(v) — "M<v>" (digit)
    case 14: {  // PlayWAV(ch, track) — file number 0-padded to 4 digits
      char file[8]; snprintf(file, sizeof(file), "%04d", track);
      inner = String("C") + audioprefix[chan] + file + ",QP" + audioprefix[chan];
      break;
    }
    case 16:  // StopWAV(ch)
      inner = String("PS") + audioprefix[chan] + ",QP" + audioprefix[chan];
      break;
    case 17:  // SetVolume(ch, vol). chan 3 = ALL = V, A, B in one write.
      if (chan == 3) {
        _vol[0] = _vol[1] = _vol[2] = (int8_t)track;   // keep the shadow in sync
        return String("<PVV") + track + ">\n<PVA" + track + ">\n<PVB" + track + ">\n";
      }
      _vol[chan] = (int8_t)track;                       // keep the shadow in sync
      inner = String("PV") + audioprefix[chan] + String(track);
      break;
    case 18:  // VolumeUpAll(step) / VolumeDownAll(step) — RELATIVE across V+A+B.
    case 19:  // No native step command: step the per-channel shadow, emit absolute sets.
    {
      int step = (track > 0) ? track : 5;               // track = step; 0 = default 5
      if (fn == 19) step = -step;
      String allCh;
      for (int c = 0; c < 3; c++) {
        int v = _vol[c] + step;
        if (v < 0) v = 0; else if (v > 99) v = 99;
        _vol[c] = (int8_t)v;
        allCh += String("<PV") + audioprefix[c] + v + ">\n";
      }
      return allCh;
    }
    default: return "";   // unreachable — normalize() rejects unknown fn
  }
  return String("<") + inner + ">\n";
}

bool HcrCodec::emit(Stream &out, uint8_t fn, int chan, int track) {
  String s = format(fn, chan, track);
  if (s.length() == 0) return false;
  out.print(s);
  return true;
}
