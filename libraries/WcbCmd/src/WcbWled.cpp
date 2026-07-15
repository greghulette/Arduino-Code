#include "WcbWled.h"

namespace {

// Split a comma list, return field idx (0-based), trimmed. "" if absent.
// Verbatim from WCB_WLED.cpp's wledField() so field parsing can't drift.
String wledField(const String &s, int idx) {
  int start = 0;
  for (int i = 0; i < idx; i++) {
    int c = s.indexOf(',', start);
    if (c < 0) return "";
    start = c + 1;
  }
  int end = s.indexOf(',', start);
  String f = (end < 0) ? s.substring(start) : s.substring(start, end);
  f.trim();
  return f;
}

}  // namespace

namespace WcbWled {

String build(const String &cmd, Print *diag) {
  String body = cmd; body.trim();

  // Tolerate an optional leading "L" + numeric id (";L3,PS,2" / "L,ON") so the exact
  // token the mesh routes can be handed straight in. No WLED verb starts with 'L' or
  // a digit, so this is unambiguous. Routing still owns the id — it's discarded here.
  if (body.length() && (body[0] == 'L' || body[0] == 'l')) {
    int i = 1;
    while (i < (int)body.length() && isDigit(body[i])) i++;
    if (i < (int)body.length() && body[i] == ',') i++;
    body = body.substring(i);
    body.trim();
  }

  if (body.length() == 0) { if (diag) diag->println("[WLED] Empty ;L command"); return ""; }

  String verb = wledField(body, 0);
  String vU   = verb; vU.toUpperCase();

  // ---- JSON: raw passthrough — everything after "JSON," verbatim -----------
  // (may contain commas / braces, so do NOT field-split it).
  if (vU == "JSON") {
    int firstComma = body.indexOf(',');
    String payload = (firstComma < 0) ? "" : body.substring(firstComma + 1);
    payload.trim();
    if (payload.length() == 0) { if (diag) diag->println("[WLED] JSON needs a payload"); return ""; }
    return payload;
  }

  if (vU == "ON")     return "{\"on\":true}";
  if (vU == "OFF")    return "{\"on\":false}";
  if (vU == "TOGGLE") return "{\"on\":\"t\"}";

  if (vU == "BRI") {
    int bri = constrain(wledField(body, 1).toInt(), 0, 255);
    return "{\"bri\":" + String(bri) + "}";
  }

  if (vU == "PS") {                     // recall preset (primary workflow)
    String n = wledField(body, 1);
    if (n.length() == 0) { if (diag) diag->println("[WLED] Usage: ;L[id],PS,<preset#>"); return ""; }
    return "{\"ps\":" + String(n.toInt()) + "}";
  }

  if (vU == "COL") {                    // solid colour on segment 0: RRGGBB or RRGGBBWW
    String hex = wledField(body, 1);
    if (hex.startsWith("#")) hex = hex.substring(1);
    if (hex.length() != 6 && hex.length() != 8) {
      if (diag) diag->println("[WLED] Usage: ;L[id],COL,RRGGBB  (or RRGGBBWW for RGBW)");
      return "";
    }
    unsigned long v = strtoul(hex.c_str(), nullptr, 16);
    int r, g, b, w = -1;
    if (hex.length() == 8) { r = (v >> 24) & 0xFF; g = (v >> 16) & 0xFF; b = (v >> 8) & 0xFF; w = v & 0xFF; }
    else                   { r = (v >> 16) & 0xFF; g = (v >> 8)  & 0xFF; b = v & 0xFF; }
    String col = "[" + String(r) + "," + String(g) + "," + String(b);
    if (w >= 0) col += "," + String(w);
    col += "]";
    return "{\"seg\":[{\"col\":[" + col + "]}]}";
  }

  if (vU == "FX") {                     // effect by index, optional speed + intensity
    String n = wledField(body, 1);
    if (n.length() == 0) { if (diag) diag->println("[WLED] Usage: ;L[id],FX,<index>[,<speed>,<intensity>]"); return ""; }
    String seg = "{\"fx\":" + String(n.toInt());
    String sx = wledField(body, 2);
    String ix = wledField(body, 3);
    if (sx.length()) seg += ",\"sx\":" + String(constrain(sx.toInt(), 0, 255));
    if (ix.length()) seg += ",\"ix\":" + String(constrain(ix.toInt(), 0, 255));
    seg += "}";
    return "{\"seg\":[" + seg + "]}";
  }

  if (vU == "PAL") {                    // palette by index
    String n = wledField(body, 1);
    if (n.length() == 0) { if (diag) diag->println("[WLED] Usage: ;L[id],PAL,<index>"); return ""; }
    return "{\"seg\":[{\"pal\":" + String(n.toInt()) + "}]}";
  }

  if (diag) diag->printf("[WLED] Unknown ;L command: %s  (ON/OFF/TOGGLE/BRI/PS/COL/FX/PAL/JSON)\n",
                         verb.c_str());
  return "";
}

bool emit(Stream &out, const String &cmd, Print *diag) {
  String json = build(cmd, diag);
  if (json.length() == 0) return false;
  out.print(json);
  out.print('\n');            // WLED's serial parser frames on newline
  return true;
}

}  // namespace WcbWled
