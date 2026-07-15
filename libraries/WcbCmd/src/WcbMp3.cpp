#include "WcbMp3.h"
#include <string.h>
#include <ctype.h>

// case-insensitive: does s begin with pre?
static bool ip(const char* s, const char* pre) {
    while (*pre) { if (toupper((unsigned char)*s) != toupper((unsigned char)*pre)) return false; s++; pre++; }
    return true;
}
// case-insensitive full equality
static bool ieq(const char* a, const char* b) {
    while (*a && *b) { if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return false; a++; b++; }
    return *a == *b;
}

void Mp3Codec::raw(uint8_t b1, int b2) {
    if (!_out) return;
    _out->write(b1);                 // MP3 Trigger command byte
    if (b2 >= 0) _out->write((uint8_t)b2);   // optional argument byte (0-255)
}

void Mp3Codec::setPending(const char* key) {
    if (!key) { _pendingCb[0] = '\0'; return; }
    while (*key == ' ') key++;                       // trim leading spaces
    strncpy(_pendingCb, key, sizeof(_pendingCb) - 1);
    _pendingCb[sizeof(_pendingCb) - 1] = '\0';
    for (int i = (int)strlen(_pendingCb) - 1;         // trim trailing whitespace
         i >= 0 && (_pendingCb[i] == ' ' || _pendingCb[i] == '\r' || _pendingCb[i] == '\n'); i--)
        _pendingCb[i] = '\0';
}

bool Mp3Codec::handle(const char* body) {
    if (!body) return false;
    const char* r = body;
    if (*r == 'A' || *r == 'a') r++;     // optional leading verb letter
    if (*r == ',') r++;
    while (*r == ' ') r++;

    // ── PLAY,n[,ONFIN,key|,key] — set volume, then play track by number (1-255) ──
    if (ip(r, "PLAY,")) {
        const char* args  = r + 5;
        int         track = atoi(args);
        if (track < 1 || track > 255) return false;   // reject BEFORE touching pending state,
        const char* comma = strchr(args, ',');        // else a rejected PLAY clobbers an
        setPending(comma ? (ip(comma + 1, "ONFIN,") ? comma + 7 : comma + 1) : nullptr);  // in-flight track's ONFIN key
        raw('v', _volume);
        raw('t', track);
        return true;
    }

    // ── PLAYFS,n[,...] — set volume, then play by file index (0-255) ─────────────
    if (ip(r, "PLAYFS,")) {
        const char* args = r + 7;
        int         idx  = atoi(args);
        if (idx < 0 || idx > 255) return false;        // reject before setPending (see PLAY)
        const char* comma = strchr(args, ',');
        setPending(comma ? (ip(comma + 1, "ONFIN,") ? comma + 7 : comma + 1) : nullptr);
        raw('v', _volume);
        raw('p', idx);
        return true;
    }

    if (ieq(r, "STOP")) { raw('O'); return true; }   // start/stop toggle
    if (ieq(r, "NEXT")) { raw('F'); return true; }
    if (ieq(r, "PREV")) { raw('R'); return true; }

    // ── VOL,n (0=loudest .. 64=inaudible) ───────────────────────────────────────
    if (ip(r, "VOL,")) {
        int n = atoi(r + 4);
        if (n < 0 || n > 64) return false;
        _volume = (uint8_t)n;
        raw('v', _volume);
        if (onVolumeChanged) onVolumeChanged(_volume);
        return true;
    }
    if (ieq(r, "VOLUP")) {                             // louder — decrease value, floor 0
        _volume = (_volume <= 5) ? 0 : _volume - 5;
        raw('v', _volume);
        if (onVolumeChanged) onVolumeChanged(_volume);
        return true;
    }
    if (ieq(r, "VOLDN")) {                             // quieter — increase value, ceiling 64
        _volume = (_volume >= 59) ? 64 : _volume + 5;
        raw('v', _volume);
        if (onVolumeChanged) onVolumeChanged(_volume);
        return true;
    }

    if (ieq(r, "COUNT")) { raw('S', '1'); return true; }   // request track count
    if (ieq(r, "VER"))   { raw('S', '0'); return true; }   // request firmware version

    return false;                                     // unknown command
}

void Mp3Codec::poll() {
    if (!_out) return;
    while (_out->available()) {
        int c = _out->read();
        if (c < 0) break;
        uint8_t b = (uint8_t)c;

        if (_inTrigger) {                             // 'M' + 3 bytes: quiet-mode trigger report — discard
            if (++_triggerBytes >= 3) { _inTrigger = false; _triggerBytes = 0; }
            continue;
        }
        if (_inStatus) {                              // '=' + ASCII + CR: version / track-count string
            if (b == '\r' || b == '\n') {
                _statusBuf[_statusLen] = '\0';
                if (_diag) _diag->printf("[MP3] Status: %s\n", _statusBuf);
                _statusLen = 0; _inStatus = false;
            } else if (_statusLen < sizeof(_statusBuf) - 1) {
                _statusBuf[_statusLen++] = (char)b;
            }
            continue;
        }
        switch (b) {
            case 'X':   // track finished normally
                if (_diag) _diag->println("[MP3] Track finished");
                if (_pendingCb[0] && onFinished) onFinished(_pendingCb);
                _pendingCb[0] = '\0';
                break;
            case 'x':   // track cancelled — do NOT fire the callback
                if (_diag) _diag->println("[MP3] Track cancelled");
                _pendingCb[0] = '\0';
                break;
            case 'E':   // error — track not found / device fault
                if (_diag) _diag->println("[MP3] Error: track not found or device error");
                _pendingCb[0] = '\0';
                if (onError) onError();
                break;
            case '=':   _statusLen = 0; _inStatus = true; break;
            case 'M':   _inTrigger = true; _triggerBytes = 0; break;
            default:    break;                         // unknown byte — ignore
        }
    }
}
