#include "WcbMaestro.h"

namespace WcbMaestro {

// ── Legacy subroutine (UNCHANGED — ;M<id><seq>) ────────────────────────────────
bool parse(const char* payload, uint8_t& id, uint16_t& seq) {
    if (!payload) return false;
    const char* p = payload;
    if (*p == 'M' || *p == 'm') p++;            // optional leading verb letter
    if (*p < '0' || *p > '9') return false;     // need an id digit (WCB: substring(1,2))
    id = (uint8_t)(*p - '0');                   // id = single digit
    p++;
    long s = 0;                                 // sequence = remaining digits (WCB: substring(2).toInt())
    while (*p >= '0' && *p <= '9') {
        s = s * 10 + (*p - '0');
        if (s > 255) return false;              // out of single-byte range → reject
        p++;
    }
    seq = (uint16_t)s;                          // empty run → 0, like String::toInt()
    return true;
}

size_t buildSubroutineFrame(uint8_t id, uint8_t seq, uint8_t out[4]) {
    out[0] = POLOLU_LEAD;      // Pololu protocol lead byte
    out[1] = id;               // device number
    out[2] = CMD_RESTART_SUB;  // "Restart Script at Subroutine"
    out[3] = seq;              // subroutine number
    return 4;
}

// ── Pololu frame builders ──────────────────────────────────────────────────────
// A 14-bit value goes on the wire as two 7-bit bytes: low first, then high.
static inline void put14(uint8_t* p, uint16_t v) { p[0] = v & 0x7F; p[1] = (v >> 7) & 0x7F; }

size_t buildSetTarget(uint8_t dev, uint8_t ch, uint16_t target, uint8_t out[6]) {
    if (ch > 127 || target > 16383) return 0;
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_SET_TARGET; out[3]=ch; put14(out+4, target);
    return 6;
}
size_t buildSetSpeed(uint8_t dev, uint8_t ch, uint16_t speed, uint8_t out[6]) {
    if (ch > 127 || speed > 16383) return 0;
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_SET_SPEED; out[3]=ch; put14(out+4, speed);
    return 6;
}
size_t buildSetAccel(uint8_t dev, uint8_t ch, uint8_t accel, uint8_t out[6]) {
    if (ch > 127) return 0;
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_SET_ACCEL; out[3]=ch; put14(out+4, accel);
    return 6;
}
size_t buildGoHome(uint8_t dev, uint8_t out[3]) {
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_GO_HOME; return 3;
}
size_t buildStopScript(uint8_t dev, uint8_t out[3]) {
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_STOP_SCRIPT; return 3;
}
size_t buildSubParam(uint8_t dev, uint8_t sub, uint16_t param, uint8_t out[6]) {
    if (param > 16383) return 0;
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_RESTART_SUB_P; out[3]=sub; put14(out+4, param);
    return 6;
}
size_t buildGetPosition(uint8_t dev, uint8_t ch, uint8_t out[4]) {
    if (ch > 127) return 0;
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_GET_POSITION; out[3]=ch; return 4;
}
size_t buildGetMovingState(uint8_t dev, uint8_t out[3]) {
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_GET_MOVING; return 3;
}
size_t buildGetErrors(uint8_t dev, uint8_t out[3]) {
    out[0]=POLOLU_LEAD; out[1]=dev; out[2]=CMD_GET_ERRORS; return 3;
}

// ── Verb-form text parsing helpers (no String, no malloc — runs on any board) ──
static bool cStrHasComma(const char* p) { for (; *p; ++p) if (*p == ',') return true; return false; }
static const char* nextField(const char* p) {   // advance to just past the next comma
    while (*p && *p != ',') ++p;
    if (*p == ',') ++p;
    return p;
}
static bool uintField(const char* p, long& out) {   // parse the uint at p (stops at ','/'\0')
    while (*p == ' ') ++p;
    if (*p < '0' || *p > '9') return false;
    long v = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10 + (*p - '0'); if (v > 1000000L) return false; ++p; }
    out = v;
    return true;
}
static bool verbIs(const char* p, const char* name) {   // case-insensitive whole-token compare
    while (*name) {
        char a = *p++, b = *name++;
        if (a >= 'A' && a <= 'Z') a += 32;
        if (b >= 'A' && b <= 'Z') b += 32;
        if (a != b) return false;
    }
    return (*p == ',' || *p == '\0' || *p == ' ');   // token must END here (no prefix match)
}

// ── Text → frame ───────────────────────────────────────────────────────────────
size_t build(const char* payload, uint8_t* out, size_t outMax) {
    if (!payload || !out) return 0;
    const char* p = payload;
    if (*p == 'M' || *p == 'm') ++p;

    // LEGACY: no comma → ;M<id><seq> subroutine (device 0-9). Keeps ;M11 valid.
    if (!cStrHasComma(p)) {
        uint8_t id; uint16_t seq;
        if (!parse(payload, id, seq)) return 0;
        if (outMax < 4) return 0;
        return buildSubroutineFrame(id, (uint8_t)seq, out);
    }

    // VERB: ;M<dev>,<verb>[,args]  (device 0-127)
    long dev;
    if (!uintField(p, dev) || dev > 127) return 0;
    const char* v = nextField(p);
    uint8_t d = (uint8_t)dev;

    // NUMERIC token → subroutine trigger: ;M<dev>,<n>[,<param>]. This makes the comma
    // form ;M1,1 behave EXACTLY like the legacy no-comma ;M11 (same 0xAA d 0x27 n frame),
    // and lets it address the full device range 0-127 with an explicit subroutine (the
    // no-comma form only reaches device 0-9). An optional <param> upgrades it to
    // "Restart Script at Subroutine w/ Parameter" (0x28), same as the "sub" verb.
    // Alphabetic tokens fall through to the servo/query verbs below.
    {
        long n;
        if (uintField(v, n)) {          // numeric token (uintField skips leading spaces) → subroutine
            if (n > 255) return 0;      // (a non-numeric OR overflowing token → false → fall to verbs)
            const char* a2 = nextField(v); long param;
            if (uintField(a2, param)) {
                if (param > 16383 || outMax < 6) return 0;
                return buildSubParam(d, (uint8_t)n, (uint16_t)param, out);
            }
            if (outMax < 4) return 0;
            return buildSubroutineFrame(d, (uint8_t)n, out);
        }
    }

    if (verbIs(v, "setTarget")) {
        const char* a1 = nextField(v);  long ch;  if (!uintField(a1, ch))  return 0;
        const char* a2 = nextField(a1); long pos; if (!uintField(a2, pos)) return 0;
        if (outMax < 6) return 0;
        return buildSetTarget(d, (uint8_t)ch, (uint16_t)pos, out);
    }
    if (verbIs(v, "setSpeed")) {
        const char* a1 = nextField(v);  long ch; if (!uintField(a1, ch)) return 0;
        const char* a2 = nextField(a1); long sp; if (!uintField(a2, sp)) return 0;
        if (outMax < 6) return 0;
        return buildSetSpeed(d, (uint8_t)ch, (uint16_t)sp, out);
    }
    if (verbIs(v, "setAccel")) {
        const char* a1 = nextField(v);  long ch; if (!uintField(a1, ch))            return 0;
        const char* a2 = nextField(a1); long ac; if (!uintField(a2, ac) || ac > 255) return 0;
        if (outMax < 6) return 0;
        return buildSetAccel(d, (uint8_t)ch, (uint8_t)ac, out);
    }
    if (verbIs(v, "goHome")) {
        if (outMax < 3) return 0;
        return buildGoHome(d, out);
    }
    if (verbIs(v, "stopScript")) {
        if (outMax < 3) return 0;
        return buildStopScript(d, out);
    }
    if (verbIs(v, "sub")) {   // ;M<dev>,sub,<n>[,<param>] — explicit subroutine, device 0-127
        const char* a1 = nextField(v); long n; if (!uintField(a1, n) || n > 255) return 0;
        const char* a2 = nextField(a1); long param;
        if (uintField(a2, param)) {
            if (param > 16383 || outMax < 6) return 0;
            return buildSubParam(d, (uint8_t)n, (uint16_t)param, out);
        }
        if (outMax < 4) return 0;
        return buildSubroutineFrame(d, (uint8_t)n, out);
    }
    if (verbIs(v, "getPosition")) {
        const char* a1 = nextField(v); long ch; if (!uintField(a1, ch)) return 0;
        if (outMax < 4) return 0;
        return buildGetPosition(d, (uint8_t)ch, out);
    }
    if (verbIs(v, "getMovingState")) { if (outMax < 3) return 0; return buildGetMovingState(d, out); }
    if (verbIs(v, "getErrors"))      { if (outMax < 3) return 0; return buildGetErrors(d, out); }

    return 0;   // unknown verb
}

bool emit(Stream& out, const char* payload) {
    uint8_t buf[MAX_FRAME];
    size_t n = build(payload, buf, sizeof(buf));
    if (n == 0) return false;
    return out.write(buf, n) == n;
}

// ── Query reply readback (POS / MOV / ERR) ─────────────────────────────────────
bool replyInfo(uint8_t cmd, ReplyKind& kind, uint8_t& len) {
    switch (cmd) {
        case CMD_GET_POSITION: kind = ReplyKind::POS; len = 2; return true;
        case CMD_GET_MOVING:   kind = ReplyKind::MOV; len = 1; return true;
        case CMD_GET_ERRORS:   kind = ReplyKind::ERR; len = 2; return true;
        default:               return false;   // not a get* command → no reply expected
    }
}

uint8_t replyLen(ReplyKind kind) {
    return (kind == ReplyKind::MOV) ? 1 : 2;   // POS=2, MOV=1, ERR=2
}

bool decodeReply(ReplyKind kind, const uint8_t* bytes, size_t len, uint16_t& value) {
    if (!bytes || len < replyLen(kind)) return false;
    if (kind == ReplyKind::MOV) value = bytes[0] ? 1 : 0;                      // 1-byte 0/1
    else                        value = (uint16_t)(bytes[0] | (bytes[1] << 8)); // 2-byte LE
    return true;
}

size_t formatReply(char* out, size_t cap, uint8_t id, uint8_t chan,
                   ReplyKind kind, const uint8_t* bytes, size_t len) {
    if (!out || cap == 0) return 0;
    uint16_t value;
    if (!decodeReply(kind, bytes, len, value)) return 0;
    const char* k = (kind == ReplyKind::POS) ? "POS"
                  : (kind == ReplyKind::MOV) ? "MOV" : "ERR";
    int n = snprintf(out, cap, ":MQR,%u,%u,%s,%u",
                     (unsigned)id, (unsigned)chan, k, (unsigned)value);
    if (n <= 0 || (size_t)n >= cap) return 0;   // encoding error or truncated → nothing usable
    return (size_t)n;
}

}  // namespace WcbMaestro
