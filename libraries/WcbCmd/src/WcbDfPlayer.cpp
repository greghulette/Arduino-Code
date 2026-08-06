#include "WcbDfPlayer.h"
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

// Emit one 10-byte DFPlayer frame. ACK byte is 0 — see the header for why we never
// wait for the reply.
void DfPlayerCodec::frame(uint8_t cmd, uint16_t param) {
    if (!_out) return;
    uint8_t f[10] = { 0x7E, 0xFF, 0x06, cmd, 0x00,
                      (uint8_t)(param >> 8), (uint8_t)(param & 0xFF),
                      0x00, 0x00, 0xEF };
    // Checksum spans bytes[1..6] (Version .. ParamLo) and is the two's complement
    // of their sum — matches DFRobotDFPlayerMini::calculateCheckSum().
    uint16_t sum = 0;
    for (int i = 1; i <= 6; i++) sum += f[i];
    uint16_t ck = (uint16_t)(-(int16_t)sum);
    f[7] = (uint8_t)(ck >> 8);
    f[8] = (uint8_t)(ck & 0xFF);
    _out->write(f, 10);
    if (_diag) _diag->printf("[DFP] cmd=0x%02X param=%u\n", cmd, param);
}

void DfPlayerCodec::setPending(const char* key) {
    if (!key) { _pendingCb[0] = '\0'; return; }
    while (*key == ' ') key++;                       // trim leading spaces
    strncpy(_pendingCb, key, sizeof(_pendingCb) - 1);
    _pendingCb[sizeof(_pendingCb) - 1] = '\0';
    for (int i = (int)strlen(_pendingCb) - 1;         // trim trailing whitespace
         i >= 0 && (_pendingCb[i] == ' ' || _pendingCb[i] == '\r' || _pendingCb[i] == '\n'); i--)
        _pendingCb[i] = '\0';
}

// Pull the optional trailing callback key out of an arg list, given the comma that
// ends the LAST positional argument. Accepts both ",ONFIN,key" and a bare ",key".
static const char* cbFrom(const char* comma) {
    if (!comma) return nullptr;
    return ip(comma + 1, "ONFIN,") ? comma + 7 : comma + 1;
}

bool DfPlayerCodec::handle(const char* body) {
    if (!body) return false;
    const char* r = body;
    if (*r == 'D' || *r == 'd') r++;     // optional leading verb letter
    if (*r == ',') r++;
    while (*r == ' ') r++;

    // ── Ordering note ───────────────────────────────────────────────────────────
    // Every prefix test below includes the trailing comma ("LOOP," not "LOOP"), so
    // LOOP / LOOPALL / LOOPFOLDER cannot shadow one another and the order of the
    // tests is not load-bearing. Keep the commas if you add a verb.

    // ── PLAY,n[,ONFIN,key|,key] — play global track index (1-2999) ──────────────
    if (ip(r, "PLAY,")) {
        const char* args  = r + 5;
        int         track = atoi(args);
        if (track < 1 || track > 2999) return false;  // reject BEFORE touching pending state,
        setPending(cbFrom(strchr(args, ',')));        // else a rejected PLAY clobbers an
        frame(0x03, (uint16_t)track);                 // in-flight track's ONFIN key
        return true;
    }

    // ── FOLDER,f,t[,...] — folder 1-99, track 1-255 (the /01/002.mp3 layout) ────
    if (ip(r, "FOLDER,")) {
        const char* args   = r + 7;
        int         folder = atoi(args);
        const char* c1     = strchr(args, ',');
        if (!c1) return false;
        int         track  = atoi(c1 + 1);
        if (folder < 1 || folder > 99 || track < 1 || track > 255) return false;
        setPending(cbFrom(strchr(c1 + 1, ',')));      // callback follows the 2nd comma
        frame(0x0F, (uint16_t)((folder << 8) | track));
        return true;
    }

    // ── MP3FOLDER,n[,...] — track n in the reserved /MP3 folder (1-9999) ────────
    if (ip(r, "MP3FOLDER,")) {
        const char* args = r + 10;
        int         idx  = atoi(args);
        if (idx < 1 || idx > 9999) return false;
        setPending(cbFrom(strchr(args, ',')));
        frame(0x12, (uint16_t)idx);
        return true;
    }

    if (ieq(r, "STOP"))   { frame(0x16); _pendingCb[0] = '\0'; return true; }  // an explicit stop cancels ONFIN
    if (ieq(r, "NEXT"))   { frame(0x01); return true; }
    if (ieq(r, "PREV"))   { frame(0x02); return true; }
    if (ieq(r, "PAUSE"))  { frame(0x0E); return true; }
    if (ieq(r, "RESUME")) { frame(0x0D); return true; }
    if (ieq(r, "RANDOM")) { frame(0x18); return true; }
    if (ieq(r, "RESET"))  { frame(0x0C); _pendingCb[0] = '\0'; return true; }

    // ── VOL,n (0 = silent .. 30 = loudest — the DEVICE's scale, see header) ─────
    if (ip(r, "VOL,")) {
        int n = atoi(r + 4);
        if (n < 0 || n > DFP_VOL_MAX) return false;
        _volume = (uint8_t)n;
        frame(0x06, _volume);
        if (onVolumeChanged) onVolumeChanged(_volume);
        return true;
    }
    // VOLUP/VOLDN send an ABSOLUTE volume frame rather than the device's own
    // relative 0x04/0x05. The shadow is what the host persists and what volume()
    // reports, and a relative step would drift it out of sync with the device.
    if (ieq(r, "VOLUP")) {                             // louder — step up, ceiling 30
        _volume = (_volume >= DFP_VOL_MAX - 2) ? DFP_VOL_MAX : _volume + 2;
        frame(0x06, _volume);
        if (onVolumeChanged) onVolumeChanged(_volume);
        return true;
    }
    if (ieq(r, "VOLDN")) {                             // quieter — step down, floor 0
        _volume = (_volume <= 2) ? 0 : _volume - 2;
        frame(0x06, _volume);
        if (onVolumeChanged) onVolumeChanged(_volume);
        return true;
    }

    // ── LOOP,n — repeat one track forever; LOOPALL,0|1; LOOPFOLDER,f ───────────
    if (ip(r, "LOOPALL,")) {
        int on = atoi(r + 8);
        if (on < 0 || on > 1) return false;
        frame(0x11, (uint16_t)on);
        return true;
    }
    if (ip(r, "LOOPFOLDER,")) {
        int folder = atoi(r + 11);
        if (folder < 1 || folder > 99) return false;
        frame(0x17, (uint16_t)folder);
        return true;
    }
    if (ip(r, "LOOP,")) {
        int track = atoi(r + 5);
        if (track < 1 || track > 2999) return false;
        frame(0x08, (uint16_t)track);
        return true;
    }

    // ── EQ,0-5  (0 Normal · 1 Pop · 2 Rock · 3 Jazz · 4 Classic · 5 Bass) ──────
    if (ip(r, "EQ,")) {
        int eq = atoi(r + 3);
        if (eq < 0 || eq > 5) return false;
        frame(0x07, (uint16_t)eq);
        return true;
    }

    // ── DEVICE,n  (1 USB · 2 SD · 3 AUX · 4 sleep · 5 flash) ───────────────────
    if (ip(r, "DEVICE,")) {
        int dev = atoi(r + 7);
        if (dev < 1 || dev > 5) return false;
        frame(0x09, (uint16_t)dev);
        return true;
    }

    if (ieq(r, "STATUS")) { frame(0x42); return true; }   // query play status (diag)

    return false;                                     // unknown command
}

void DfPlayerCodec::poll() {
    if (!_out) return;
    while (_out->available()) {
        int c = _out->read();
        if (c < 0) break;
        uint8_t b = (uint8_t)c;

        // Resync on the start byte: a partial or corrupt frame must not swallow the
        // next good one (a DFPlayer emits unsolicited init/finish frames at will).
        if (_rxLen == 0 && b != 0x7E) continue;
        _rx[_rxLen++] = b;
        if (_rxLen < 10) continue;
        _rxLen = 0;

        if (_rx[9] != 0xEF) continue;                  // not a frame after all — drop
        uint8_t  cmd   = _rx[3];
        uint16_t param = ((uint16_t)_rx[5] << 8) | _rx[6];

        switch (cmd) {
            case 0x3D:   // track finished (SD)
            case 0x3C:   // track finished (USB)
                if (_diag) _diag->printf("[DFP] Track %u finished\n", param);
                if (_pendingCb[0] && onFinished) onFinished(_pendingCb);
                _pendingCb[0] = '\0';
                break;
            case 0x40:   // module error — param is the error code
                if (_diag) _diag->printf("[DFP] Error 0x%02X\n", param);
                _pendingCb[0] = '\0';
                if (onError) onError();
                break;
            case 0x3F:   // init complete — param is the storage bitmask
                if (_diag) _diag->printf("[DFP] Online (storage 0x%02X)\n", param);
                break;
            default:     break;                        // ACK/query replies — ignore
        }
    }
}
