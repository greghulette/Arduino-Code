#include "WcbMaestro.h"

namespace WcbMaestro {

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
        if (s > 255) return false;              // out of single-byte range → reject (WCB returns w/o send)
        p++;
    }
    seq = (uint16_t)s;                          // empty run → 0, like String::toInt()
    return true;
}

size_t buildSubroutineFrame(uint8_t id, uint8_t seq, uint8_t out[4]) {
    out[0] = 0xAA;   // Pololu Compact-protocol lead byte
    out[1] = id;     // device number
    out[2] = 0x27;   // "Restart Script at Subroutine"
    out[3] = seq;    // subroutine number
    return 4;
}

bool emit(Stream& out, const char* payload) {
    uint8_t  id;
    uint16_t seq;
    if (!parse(payload, id, seq)) return false;
    uint8_t frame[4];
    buildSubroutineFrame(id, (uint8_t)seq, frame);
    return out.write(frame, 4) == 4;
}

}  // namespace WcbMaestro
