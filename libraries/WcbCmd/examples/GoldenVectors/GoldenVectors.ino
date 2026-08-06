/*
  GoldenVectors.ino — WcbCmd self-test

  Asserts each device module reproduces the exact native wire bytes the WCB firmware
  and NaviCore emit today. Flash to any ESP32, open Serial @115200: every line should
  read OK and the last line PASSED. These vectors are the contract that keeps the two
  library copies from drifting — if a copy changes the bytes, this test fails.
*/
#include <WcbCmd.h>

static int failures = 0;

// A Stream that just captures whatever is written to it (for byte-exact checks).
class CaptureStream : public Stream {
public:
    uint8_t buf[128];
    size_t  len = 0;
    void reset() { len = 0; }
    size_t write(uint8_t b) override { if (len < sizeof(buf)) buf[len++] = b; return 1; }
    size_t write(const uint8_t* d, size_t n) override { size_t w = 0; for (size_t i = 0; i < n; i++) w += write(d[i]); return w; }
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
};

static void report(bool pass, const char* label, const uint8_t* got, size_t n) {
    if (!pass) failures++;
    Serial.printf("%s  %-14s -> [", pass ? "OK  " : "FAIL", label);
    for (size_t i = 0; i < n; i++) Serial.printf("%s%02X", i ? " " : "", got[i]);
    Serial.println("]");
}

// ── Maestro ;M<id><seq> -> {0xAA, id, 0x27, seq} ────────────────────────────────
static void mMaestro(const char* payload, uint8_t e0, uint8_t e1, uint8_t e2, uint8_t e3) {
    uint8_t id; uint16_t seq;
    bool ok = WcbMaestro::parse(payload, id, seq);
    uint8_t f[4] = {0,0,0,0};
    if (ok) WcbMaestro::buildSubroutineFrame(id, (uint8_t)seq, f);
    report(ok && f[0]==e0 && f[1]==e1 && f[2]==e2 && f[3]==e3, payload, f, 4);
}
static void mReject(const char* payload) {
    uint8_t id; uint16_t seq;
    bool ok = WcbMaestro::parse(payload, id, seq);
    if (ok) failures++;
    Serial.printf("%s  %-14s -> rejected\n", ok ? "FAIL" : "OK  ", payload);
}

// ── Maestro query readback: reply bytes -> ":MQR,<id>,<chan>,<KIND>,<value>" ─────
static void mReply(uint8_t id, uint8_t chan, WcbMaestro::ReplyKind kind,
                   const uint8_t* bytes, size_t n, const char* exp) {
    char out[WcbMaestro::REPLY_TEXT_MAX];
    size_t w = WcbMaestro::formatReply(out, sizeof(out), id, chan, kind, bytes, n);
    bool pass = w == strlen(exp) && strcmp(out, exp) == 0;
    if (!pass) failures++;
    Serial.printf("%s  %-18s -> %s\n", pass ? "OK  " : "FAIL", exp, w ? out : "(none)");
}
static void mReplyInfo(uint8_t cmd, WcbMaestro::ReplyKind expKind, uint8_t expLen) {
    WcbMaestro::ReplyKind k; uint8_t ln = 0;
    bool ok = WcbMaestro::replyInfo(cmd, k, ln) && k == expKind && ln == expLen;
    if (!ok) failures++;
    Serial.printf("%s  replyInfo 0x%02X     -> %s\n", ok ? "OK  " : "FAIL", cmd, ok ? "kind/len match" : "MISMATCH");
}
static void mReplyInfoReject(uint8_t cmd) {   // a non-get command has no reply
    WcbMaestro::ReplyKind k; uint8_t ln = 0;
    bool rejected = !WcbMaestro::replyInfo(cmd, k, ln);
    if (!rejected) failures++;
    Serial.printf("%s  replyInfo 0x%02X     -> %s\n", rejected ? "OK  " : "FAIL", cmd, rejected ? "no reply (rejected)" : "UNEXPECTED reply");
}
static void mReplyTruncate() {   // too-small buffer -> formatReply returns 0, nothing usable
    uint8_t b[] = {0xFF, 0xFF};
    char out[10];                 // ":MQR,4,0,POS,65535" is 18 chars -> must truncate into out[10]
    size_t w = WcbMaestro::formatReply(out, sizeof(out), 4, 0, WcbMaestro::ReplyKind::POS, b, 2);
    bool pass = (w == 0);
    if (!pass) failures++;
    Serial.printf("%s  formatReply cap10  -> %s\n", pass ? "OK  " : "FAIL", pass ? "0 (truncated, safe)" : "OVERRAN");
}

// ── MP3 ;A,... -> SparkFun MP3 Trigger native bytes ─────────────────────────────
static Mp3Codec     mp3;
static CaptureStream cap;
static void mMp3(const char* body, const uint8_t* exp, size_t n) {
    cap.reset();
    bool ok = mp3.handle(body);
    bool pass = ok && cap.len == n && memcmp(cap.buf, exp, n) == 0;
    report(pass, body, cap.buf, cap.len);
}

// ── DFPlayer ;D,... -> DFPlayer Mini native 10-byte frames ──────────────────────
// Every frame is  7E FF 06 <CMD> 00 <PH> <PL> <CKH> <CKL> EF  with
// checksum = -(sum of bytes[1..6]). The expected bytes below are written out in
// full ON PURPOSE — recomputing the checksum here would test nothing.
static DfPlayerCodec dfp;
static void mDfp(const char* body, const uint8_t* exp, size_t n) {
    cap.reset();
    bool ok = dfp.handle(body);
    bool pass = ok && cap.len == n && memcmp(cap.buf, exp, n) == 0;
    report(pass, body, cap.buf, cap.len);
}
static void mDfpReject(const char* body) {
    cap.reset();
    bool ok = dfp.handle(body);
    if (ok || cap.len) failures++;
    Serial.printf("%s  %-14s -> (nothing emitted)\n", (!ok && !cap.len) ? "OK  " : "FAIL", body);
}

// ── WLED ;L,... -> WLED /json/state document + '\n' ─────────────────────────────
static void mWled(const char* body, const char* expJson) {
    cap.reset();
    bool ok = WcbWled::emit(cap, body);
    String exp = String(expJson) + "\n";
    bool pass = ok && cap.len == exp.length() && memcmp(cap.buf, exp.c_str(), exp.length()) == 0;
    if (!pass) failures++;
    // print the captured document minus the trailing '\n'
    Serial.printf("%s  %-14s -> %.*s\n", pass ? "OK  " : "FAIL", body,
                  (int)(cap.len ? cap.len - 1 : 0), cap.buf);
}
static void mWledReject(const char* body) {
    cap.reset();
    bool ok = WcbWled::emit(cap, body);
    if (ok || cap.len) failures++;
    Serial.printf("%s  %-14s -> (nothing emitted)\n", (!ok && !cap.len) ? "OK  " : "FAIL", body);
}

// ── HCR fn/chan/track -> "<...>\n" device frames ────────────────────────────────
// expWire is the FULL expected byte string, newlines included (some fns emit
// several frames, each '\n'-terminated).
static HcrCodec hcr;
static void mHcr(uint8_t fn, int chan, int track, const char* expWire) {
    cap.reset();
    bool ok = hcr.emit(cap, fn, chan, track);
    size_t n = strlen(expWire);
    bool pass = ok && cap.len == n && memcmp(cap.buf, expWire, n) == 0;
    if (!pass) failures++;
    char lbl[24]; snprintf(lbl, sizeof(lbl), "fn%d,%d,%d", fn, chan, track);
    String shown; for (size_t i=0;i<cap.len;i++) shown += (cap.buf[i]=='\n')?String("\\n"):String((char)cap.buf[i]);
    Serial.printf("%s  %-14s -> %s\n", pass ? "OK  " : "FAIL", lbl, shown.c_str());
}
static void mHcrReject(uint8_t fn, int chan, int track) {
    cap.reset();
    bool ok = hcr.emit(cap, fn, chan, track);
    if (ok || cap.len) failures++;
    char lbl[24]; snprintf(lbl, sizeof(lbl), "fn%d,%d,%d", fn, chan, track);
    Serial.printf("%s  %-14s -> (rejected)\n", (!ok && !cap.len) ? "OK  " : "FAIL", lbl);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("WcbCmd %s — golden vectors\n\n", WCBCMD_VERSION);

    Serial.println("-- Maestro --");
    mMaestro("M11",   0xAA, 1, 0x27, 1);
    mMaestro("M25",   0xAA, 2, 0x27, 5);
    mMaestro("M2100", 0xAA, 2, 0x27, 100);
    mMaestro("M0255", 0xAA, 0, 0x27, 255);
    mMaestro("11",    0xAA, 1, 0x27, 1);      // no leading verb letter
    mMaestro("M2",    0xAA, 2, 0x27, 0);      // empty sequence -> 0
    mReject("M2256");                          // sequence out of 0-255
    mReject("MX1");                            // no id digit

    Serial.println("\n-- Maestro query readback --");
    mReplyInfo(WcbMaestro::CMD_GET_POSITION, WcbMaestro::ReplyKind::POS, 2);
    mReplyInfo(WcbMaestro::CMD_GET_MOVING,   WcbMaestro::ReplyKind::MOV, 1);
    mReplyInfo(WcbMaestro::CMD_GET_ERRORS,   WcbMaestro::ReplyKind::ERR, 2);
    mReplyInfoReject(WcbMaestro::CMD_SET_TARGET);   // 0x04 — not a query, no reply expected
    { uint8_t b[] = {0x70,0x17}; mReply(3, 5, WcbMaestro::ReplyKind::POS, b, 2, ":MQR,3,5,POS,6000"); } // 0x1770 = 6000 ¼µs
    { uint8_t b[] = {0x01};      mReply(2, 0, WcbMaestro::ReplyKind::MOV, b, 1, ":MQR,2,0,MOV,1"); }
    { uint8_t b[] = {0x00};      mReply(2, 0, WcbMaestro::ReplyKind::MOV, b, 1, ":MQR,2,0,MOV,0"); }
    { uint8_t b[] = {0x00,0x00}; mReply(1, 0, WcbMaestro::ReplyKind::ERR, b, 2, ":MQR,1,0,ERR,0"); }
    { uint8_t b[] = {0x21,0x00}; mReply(1, 0, WcbMaestro::ReplyKind::ERR, b, 2, ":MQR,1,0,ERR,33"); } // 0x0021 = 33
    mReplyTruncate();

    Serial.println("\n-- MP3 --");
    mp3.begin(cap);
    mp3.setVolume(20);                         // WCB default
    { uint8_t e[] = {0x76,0x14,0x74,0x05}; mMp3("PLAY,5",   e, 4); }  // v20, t5
    { uint8_t e[] = {0x76,0x14,0x70,0x03}; mMp3("PLAYFS,3", e, 4); }  // v20, p3
    { uint8_t e[] = {0x4F};                mMp3("STOP",     e, 1); }
    { uint8_t e[] = {0x46};                mMp3("NEXT",     e, 1); }
    { uint8_t e[] = {0x52};                mMp3("PREV",     e, 1); }
    { uint8_t e[] = {0x76,0x19};           mMp3("VOL,25",   e, 2); }  // sets shadow -> 25
    { uint8_t e[] = {0x76,0x19,0x74,0x01}; mMp3("PLAY,1",   e, 4); }  // now v25
    { uint8_t e[] = {0x76,0x14};           mMp3("VOLUP",    e, 2); }  // 25 -> 20
    { uint8_t e[] = {0x76,0x19};           mMp3("VOLDN",    e, 2); }  // 20 -> 25
    { uint8_t e[] = {0x53,0x31};           mMp3("COUNT",    e, 2); }
    { uint8_t e[] = {0x53,0x30};           mMp3("VER",      e, 2); }
    { uint8_t e[] = {0x4F};                mMp3("A,STOP",   e, 1); }  // leading 'A' tolerated

    Serial.println("\n-- DFPlayer --");
    dfp.begin(cap);
    dfp.setVolume(20);                         // shadow only, emits nothing
    // PLAY is ONE frame — unlike the MP3 Trigger above, volume is deliberately NOT
    // re-sent before each play (the module remembers it). A 20-byte capture here
    // would mean that regression came back.
    { uint8_t e[] = {0x7E,0xFF,0x06,0x03,0x00,0x00,0x05,0xFE,0xF3,0xEF}; mDfp("PLAY,5",        e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x0F,0x00,0x01,0x02,0xFE,0xE9,0xEF}; mDfp("FOLDER,1,2",    e, 10); } // /01/002.mp3
    { uint8_t e[] = {0x7E,0xFF,0x06,0x12,0x00,0x00,0x03,0xFE,0xE6,0xEF}; mDfp("MP3FOLDER,3",   e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x16,0x00,0x00,0x00,0xFE,0xE5,0xEF}; mDfp("STOP",          e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x01,0x00,0x00,0x00,0xFE,0xFA,0xEF}; mDfp("NEXT",          e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x02,0x00,0x00,0x00,0xFE,0xF9,0xEF}; mDfp("PREV",          e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x0E,0x00,0x00,0x00,0xFE,0xED,0xEF}; mDfp("PAUSE",         e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x0D,0x00,0x00,0x00,0xFE,0xEE,0xEF}; mDfp("RESUME",        e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x06,0x00,0x00,0x19,0xFE,0xDC,0xEF}; mDfp("VOL,25",        e, 10); } // shadow -> 25
    { uint8_t e[] = {0x7E,0xFF,0x06,0x06,0x00,0x00,0x1B,0xFE,0xDA,0xEF}; mDfp("VOLUP",         e, 10); } // 25 -> 27, ABSOLUTE frame
    { uint8_t e[] = {0x7E,0xFF,0x06,0x06,0x00,0x00,0x19,0xFE,0xDC,0xEF}; mDfp("VOLDN",         e, 10); } // 27 -> 25
    { uint8_t e[] = {0x7E,0xFF,0x06,0x08,0x00,0x00,0x07,0xFE,0xEC,0xEF}; mDfp("LOOP,7",        e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x11,0x00,0x00,0x01,0xFE,0xE9,0xEF}; mDfp("LOOPALL,1",     e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x17,0x00,0x00,0x02,0xFE,0xE2,0xEF}; mDfp("LOOPFOLDER,2",  e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x18,0x00,0x00,0x00,0xFE,0xE3,0xEF}; mDfp("RANDOM",        e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x07,0x00,0x00,0x02,0xFE,0xF2,0xEF}; mDfp("EQ,2",          e, 10); } // Rock
    { uint8_t e[] = {0x7E,0xFF,0x06,0x09,0x00,0x00,0x02,0xFE,0xF0,0xEF}; mDfp("DEVICE,2",      e, 10); } // SD card
    { uint8_t e[] = {0x7E,0xFF,0x06,0x0C,0x00,0x00,0x00,0xFE,0xEF,0xEF}; mDfp("RESET",         e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x42,0x00,0x00,0x00,0xFE,0xB9,0xEF}; mDfp("STATUS",        e, 10); }
    { uint8_t e[] = {0x7E,0xFF,0x06,0x16,0x00,0x00,0x00,0xFE,0xE5,0xEF}; mDfp("D,STOP",        e, 10); } // leading 'D' tolerated
    mDfpReject("PLAY,0");                       // track below 1
    mDfpReject("PLAY,3000");                    // track above 2999
    mDfpReject("FOLDER,0,1");                   // folder below 1
    mDfpReject("FOLDER,1,256");                 // track above 255
    mDfpReject("FOLDER,1");                     // missing the track argument
    mDfpReject("VOL,31");                       // above the device ceiling of 30
    mDfpReject("EQ,6");                         // only 0-5 exist
    mDfpReject("BOGUS");                        // unknown verb

    Serial.println("\n-- WLED --");
    mWled("ON",            "{\"on\":true}");
    mWled("OFF",           "{\"on\":false}");
    mWled("TOGGLE",        "{\"on\":\"t\"}");
    mWled("BRI,128",       "{\"bri\":128}");
    mWled("BRI,300",       "{\"bri\":255}");                 // constrained to 255
    mWled("PS,2",          "{\"ps\":2}");
    mWled("COL,FF0000",    "{\"seg\":[{\"col\":[[255,0,0]]}]}");
    mWled("COL,#00FF00",   "{\"seg\":[{\"col\":[[0,255,0]]}]}"); // '#' tolerated
    mWled("COL,010203FF",  "{\"seg\":[{\"col\":[[1,2,3,255]]}]}"); // RGBW
    mWled("FX,9",          "{\"seg\":[{\"fx\":9}]}");
    mWled("FX,9,200,128",  "{\"seg\":[{\"fx\":9,\"sx\":200,\"ix\":128}]}");
    mWled("PAL,3",         "{\"seg\":[{\"pal\":3}]}");
    mWled("JSON,{\"bri\":10}", "{\"bri\":10}");              // raw passthrough
    mWledReject("L,ON");                                    // leading 'L' NOT stripped (matches WCB)
    mWledReject("L3,PS,2");                                 // leading 'L<id>,' NOT stripped (matches WCB)
    mWledReject("COL,FFF");                                  // bad hex length -> nothing
    mWledReject("BOGUS");                                    // unknown verb -> nothing

    Serial.println("\n-- HCR --");
    hcr.setVol(0, 50); hcr.setVol(1, 50); hcr.setVol(2, 50);   // known shadow for fn 18/19
    mHcr(2,  0, 50, "<OH50,QEH>\n");            // SetEmotion HAPPY 50
    mHcr(4,  2, 30, "<SM30,QEM,QT>\n");         // Stimulate MAD 30
    mHcrReject(4,  4,  0);                       // chan 4 rejected (no Overload shortcut) — matches WCB
    mHcr(5,  0,  0, "<SE,QT>\n");               // Overload
    mHcr(6,  0,  0, "<MM>\n");                  // single Muse
    mHcr(7,  3, 10, "<MN3,MX10>\n");            // Muse gap 3..10s
    mHcr(8,  0,  0, "<PSV,QT>\n<PSV,QPV>\n<PSA,QPA>\n<PSB,QPB>\n");  // Stop all: 4 frames = HCRVocalizer::Stop()
    mHcr(9,  0,  0, "<PSV,QT>\n");              // StopEmote
    mHcr(10, 1,  0, "<O1,QO>\n");               // OverrideEmotions ON
    mHcr(11, 0,  0, "<OR,QE>\n");               // ResetEmotions
    mHcr(13, 0,  1, "<M1,QM>\n");               // SetMuse ON
    mHcr(14, 1,  3, "<CA0003,QPA>\n");          // PlayWAV A, file 3 (0-padded)
    mHcr(16, 2,  0, "<PSB,QPB>\n");             // StopWAV B
    mHcr(17, 0, 50, "<PVV50>\n");               // SetVolume V=50
    mHcr(17, 3, 40, "<PVV40>\n<PVA40>\n<PVB40>\n");   // SetVolume ALL=40 (3 frames)
    mHcr(18, 0, 10, "<PVV50>\n<PVA50>\n<PVB50>\n");   // VolumeUp step 10: 40->50 (shadow was 40 from prev)
    mHcr(19, 0,  0, "<PVV45>\n<PVA45>\n<PVB45>\n");   // VolumeDown default step 5: 50->45
    mHcrReject(2, 5, 0);                        // emotion out of range
    mHcrReject(99, 0, 0);                       // unknown fn

    Serial.printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
}

void loop() {}
