// =============================================================================
//  mgmt_wsserver.h — WebSocket command endpoint for the WCB management relay
// =============================================================================
//
//  A second mouth for the SAME protocol MgmtRelay's USB serial port speaks. Every
//  line that arrives here goes to relaySerialLine(), the identical dispatcher the
//  USB reader in loop() feeds, so there is exactly one implementation of the relay
//  command surface and the two transports cannot drift.
//
//  ── DELIBERATELY THE SAME WIRE PROTOCOL AS NaviCore ─────────────────────────
//  NaviCore exposes ws://<softAPIP>/ws carrying newline-delimited UTF-8, reassembled
//  per socket, with the reply stream being a CONSOLE MIRROR rather than strict
//  request/response (NaviCore docs/PROTOCOLS.md:19, navicore_wsserver.h). This
//  endpoint is the same URI, the same framing and the same mirror semantics, so a
//  host app writes ONE client and points it at either box. The payloads
//  differ — NaviCore speaks JSON verbs, this speaks the WCB serial grammar
//  (";w<id>,<cmd>", bare text = broadcast) — but the transport does not.
//
//  ── THE CONCURRENCY RULE THIS FILE EXISTS TO OBEY ───────────────────────────
//  The httpd task is NOT the loop task. Its handler runs on Core 0 alongside the
//  ESP-NOW receive callback, and the same prohibition applies there as everywhere
//  else in this sketch: the WiFi/httpd side may only copy, enqueue and return.
//  relaySerialLine() calls wcb.send(), which blocks on ETM retries and touches the
//  ESP-NOW driver — exactly what must not happen beside the receive callback. So
//  the handler enqueues and drain() runs the command from loop() on Core 1. That is
//  the same split relayOutQueue already uses for inbound mesh lines, for the same
//  reason, and it is why replies go out through httpd_ws_send_frame_async() (the
//  API documented for sends "out of the scope of current request") rather than the
//  in-request send.
//
//  ── WHY A VALUE QUEUE, NOT A POINTER QUEUE ──────────────────────────────────
//  NaviCore queues by pointer out of PSRAM because a SET_CONFIG line approaches
//  98 KB. This relay has no such payload: its longest input is a relayed OTA
//  "?OTA,DATA,..." line at ~283 chars, which is why inBuf is 384 B in the .ino.
//  WS_LINE_MAX tracks that same cap, so a value queue costs 8 x 388 B and the
//  sketch keeps working on a plain ESP32 with no PSRAM. Keep the two in step: a
//  line this transport accepts but the USB path would reject is a silent
//  divergence between two mouths that are supposed to be identical.
// =============================================================================
#pragma once

#include <esp_http_server.h>
#include <WiFi.h>
#include <lwip/sockets.h>   // close() — the session-close hook must close the socket

// Defined in MgmtRelay.ino. Declared here because this header is included near the
// top of the sketch, long before the definition, and Arduino's auto-prototyping
// only covers the .ino itself.
void relaySerialLine(char* line);

namespace mgmtws {

// Keep in step with httpd_config_t::max_open_sockets in begin(). More clients than
// the server will accept is just dead slots; fewer means a connected client the
// sink never writes to, which is the silent-deafness bug this array exists to
// prevent (the same trap NaviCore hit with a single fd).
static const uint8_t  WS_MAX_CLIENTS     = 3;
// drain() runs ONE command per loop() pass, so a client that puts several lines in
// a single frame needs room. NaviCore measured 3 of 6 commands answered at depth 3
// with a zero enqueue timeout — the rest discarded in the handler with NOTHING sent
// back, which is the worst failure available here: the client waits forever for a
// reply to a command that was already thrown away.
static const uint8_t  WS_QUEUE_DEPTH     = 8;
// How long the handler waits for room rather than discarding. This is the httpd
// task, NOT loop, so blocking here costs a little latency on other sockets and
// nothing on the mesh. It turns a silent drop into ordinary backpressure.
static const uint32_t WS_ENQUEUE_WAIT_MS = 50;
// MUST match inBuf[] in MgmtRelay.ino — see the header note above.
static const size_t   WS_LINE_MAX        = 384;
// Output staging. Big enough to hold a burst of mesh lines between pump() calls
// without flushing mid-loop; small enough that one flush is ~one TCP segment.
static const size_t   WS_SINK_BUF        = 2048;

struct WsCmd {
  int  fd;                    // client socket, for the async reply
  char line[WS_LINE_MAX];     // NUL-terminated command
};

inline QueueHandle_t  wsQueue  = nullptr;
inline httpd_handle_t wsServer = nullptr;

// ── Reply sink ──────────────────────────────────────────────────────────────
// A Print that fans host-facing output out to every connected client. MgmtRelay.ino
// prints through `host` (Serial + this), so the handlers stay transport-unaware and
// a WebSocket client sees exactly what a USB console would.
//
// BUFFER, THEN SEND FROM pump(). Never send inside write(). write() runs in the
// middle of arbitrary printf calls on the loop task, and a TCP send there would put
// network I/O between two halves of a print. Buffering here and flushing at one
// known point in loop() keeps the blocking where it can be reasoned about.
//
// OVERFLOW DROPS WHOLE LINES. Truncating mid-line would hand the host half a JSON
// object (the mesh streams rc_hb/rc_ch telemetry through here), which fails at
// JSON.parse. Dropping to the next newline loses a sample instead.
class WsSink : public Print {
 public:
  void begin(int fd) {
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] == fd) return;   // already known
    // FIRST client after a quiet period: start from a clean sheet. _dropping is set
    // when a pump() fails, which is exactly what happens as the last client leaves
    // mid-line — and without this reset it survives the disconnect and eats the
    // first whole line the NEXT client is sent. Only on the transition to live:
    // clearing while another client is connected would discard output buffered for it.
    if (!live()) { _len = 0; _dropping = false; }
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] < 0) { _fds[i] = fd; return; }
    _fds[0] = fd;   // full: evict the oldest rather than refuse the newcomer
  }
  void drop(int fd) { for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] == fd) _fds[i] = -1; }
  void end()        { for (int i = 0; i < WS_MAX_CLIENTS; i++) _fds[i] = -1; _len = 0; _dropping = false; }
  bool live() const {
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] >= 0) return true;
    return false;
  }

  size_t write(uint8_t c) override {
    if (!live()) return 1;
    if (_dropping) { if (c == '\n') _dropping = false; return 1; }
    if (_len >= WS_SINK_BUF) {
      // FULL — flush here rather than drop. Only a line longer than the whole
      // buffer reaches this branch, i.e. a big config/stats dump, not the routine
      // telemetry that motivated the buffering. Losing a config reply is a hard
      // failure; a few ms of jitter on an explicit user action is not.
      if (!pump()) { _dropping = true; return 1; }   // send failed: client is gone
    }
    _buf[_len++] = (char)c;
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
  using Print::write;

  // Called from loop() (and from write() when the buffer fills). Broadcasts to every
  // connected client; one that has gone away is dropped individually rather than
  // taking the others down with it. Returns false only when nobody is left.
  bool pump() {
    if (!_len || !wsServer) return live();
    // NEVER CUT THROUGH A UTF-8 SEQUENCE. This is a TEXT frame and RFC 6455 8.1
    // requires each one to be valid UTF-8 on its own. The buffer flushes on a BYTE
    // count, so a multi-byte character in a board alias straddles the boundary and
    // the frame ends in a bare continuation byte. A conforming client fails the
    // connection with 1007, the reconnect replays the same bytes at the same
    // alignment, and the link dies in a loop that looks like flaky WiFi. Hold the
    // partial character back for the next frame instead; it is at most 3 bytes.
    size_t send = _utf8SafeLen(_buf, _len);
    if (!send) send = _len;          // cannot improve it — send rather than stall
    httpd_ws_frame_t f = {};
    f.final   = true;
    f.type    = HTTPD_WS_TYPE_TEXT;
    f.payload = (uint8_t*)_buf;
    f.len     = send;
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
      if (_fds[i] < 0) continue;
      if (httpd_ws_send_frame_async(wsServer, _fds[i], &f) != ESP_OK) _fds[i] = -1;
    }
    const size_t left = _len - send;
    if (left) memmove(_buf, _buf + send, left);
    _len = left;
    return live();
  }

 private:
  // Longest prefix of b[0..n) that does not end part-way through a UTF-8 sequence.
  // Returns n when the tail is already complete — the overwhelmingly common case
  // (pure ASCII), so this costs a couple of compares per flush.
  static size_t _utf8SafeLen(const char* b, size_t n) {
    if (!n) return 0;
    size_t back = 0;
    while (back < 3 && back < n) {
      unsigned char c = (unsigned char)b[n - 1 - back];
      if ((c & 0xC0) == 0x80) { back++; continue; }        // continuation byte
      size_t need = (c & 0x80) == 0x00 ? 1 :
                    (c & 0xE0) == 0xC0 ? 2 :
                    (c & 0xF0) == 0xE0 ? 3 :
                    (c & 0xF8) == 0xF0 ? 4 : 1;
      return (back + 1 >= need) ? n : n - (back + 1);      // complete? keep : hold back
    }
    return n;
  }

  int    _fds[WS_MAX_CLIENTS] = { -1, -1, -1 };
  char   _buf[WS_SINK_BUF];
  size_t _len      = 0;
  bool   _dropping = false;
};

inline WsSink sink;

// ── Per-socket line accumulator ─────────────────────────────────────────────
// ONE PER SOCKET, never a shared static. A single function-level buffer fuses two
// clients' bytes into one garbage line whenever their frames interleave — and that
// is not a rare race, because a long line legitimately spans several frames, so a
// second client's command landing in that window destroys both. NaviCore reproduced
// exactly this on hardware (two valid commands, zero replies). Eviction policy is
// deliberately identical to WsSink::begin() so the two cannot drift.
struct WsAcc {
  int    fd  = -1;
  size_t len = 0;
  bool   overrun = false;
  char   buf[WS_LINE_MAX];
};
inline WsAcc accs[WS_MAX_CLIENTS];

inline WsAcc* accFor(int fd) {
  for (int i = 0; i < WS_MAX_CLIENTS; i++) if (accs[i].fd == fd) return &accs[i];
  for (int i = 0; i < WS_MAX_CLIENTS; i++)
    if (accs[i].fd < 0) { accs[i].fd = fd; accs[i].len = 0; accs[i].overrun = false; return &accs[i]; }
  accs[0].fd = fd; accs[0].len = 0; accs[0].overrun = false;   // full: evict the oldest
  return &accs[0];
}
inline void releaseAcc(int fd) {
  for (int i = 0; i < WS_MAX_CLIENTS; i++)
    if (accs[i].fd == fd) { accs[i].fd = -1; accs[i].len = 0; accs[i].overrun = false; }
}

// Feed one received byte into a client's accumulator; enqueue on newline.
inline void feed(WsAcc* a, int fd, char c) {
  if (c == '\n' || c == '\r') {
    if (a->overrun)  { a->overrun = false; a->len = 0; return; }   // drop the WHOLE line
    if (!a->len)     return;                                       // blank line
    a->buf[a->len] = '\0';
    WsCmd cmd;
    cmd.fd = fd;
    memcpy(cmd.line, a->buf, a->len + 1);
    a->len = 0;
    if (!wsQueue) return;
    // Backpressure, not silent loss — see WS_ENQUEUE_WAIT_MS.
    if (xQueueSend(wsQueue, &cmd, pdMS_TO_TICKS(WS_ENQUEUE_WAIT_MS)) != pdTRUE)
      Serial.println("[relay] ws: command queue full - dropped");
    return;
  }
  if (a->overrun) return;                                          // swallow to end-of-line
  if (a->len < WS_LINE_MAX - 1) { a->buf[a->len++] = c; return; }
  // Overrun. Suppress to end-of-line rather than restarting accumulation: zeroing
  // len mid-line would treat the TAIL as a fresh command, and an unprefixed tail
  // falls through relaySerialLine() to wcb.broadcast() — putting an arbitrary
  // mid-line fragment on EVERY board on the mesh. Same reasoning as inOverrun in
  // the .ino's USB reader; the two paths must fail identically.
  a->overrun = true;
  a->len     = 0;
  Serial.println("[relay] ws: line too long - dropped");
}

// ── httpd handler (Core 0 — copy, enqueue, return) ──────────────────────────
inline esp_err_t wsHandler(httpd_req_t* req) {
  const int fd = httpd_req_to_sockfd(req);
  if (req->method == HTTP_GET) {          // handshake completed
    sink.begin(fd);
    accFor(fd);
    return ESP_OK;
  }

  httpd_ws_frame_t frame = {};
  frame.type = HTTPD_WS_TYPE_TEXT;
  // Two-call pattern: first with max_len 0 to learn the length, then read.
  esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
  if (err != ESP_OK) return err;
  if (frame.type == HTTPD_WS_TYPE_CLOSE) { releaseAcc(fd); sink.drop(fd); return ESP_OK; }
  if (frame.len == 0) return ESP_OK;
  // Cap a single frame at four lines' worth. A client that sends more than that in
  // one frame cannot produce a valid command anyway, and this keeps the buffer
  // below bounded. SAY SO rather than returning quietly: a silently discarded frame
  // leaves the client waiting forever for a reply to a command we never read, which
  // is the same failure the enqueue timeout exists to avoid.
  if (frame.len > WS_LINE_MAX * 4) {
    Serial.printf("[relay] ws: frame of %u B exceeds %u B - dropped\n",
                  (unsigned)frame.len, (unsigned)(WS_LINE_MAX * 4));
    return ESP_OK;
  }

  // static, not a stack array: 1.5 KB would blow the httpd task stack. Safe because
  // esp_http_server services every socket from ONE task, so two handler invocations
  // can never overlap.
  static uint8_t buf[WS_LINE_MAX * 4];
  frame.payload = buf;
  err = httpd_ws_recv_frame(req, &frame, sizeof(buf));
  if (err != ESP_OK) return err;
  if (frame.type != HTTPD_WS_TYPE_TEXT) return ESP_OK;

  // Re-arm the sink on traffic: a client that was evicted while another connected
  // would otherwise stay silent.
  sink.begin(fd);
  WsAcc* a = accFor(fd);
  for (size_t i = 0; i < frame.len; i++) feed(a, fd, (char)buf[i]);
  return ESP_OK;
}

// Session close: release the accumulator and the sink slot. Without this a departed
// client holds both until some later send happens to fail on it, and the unowned
// tail of a half-accumulated line eats the first command of the NEXT client.
inline void wsClose(httpd_handle_t /*hd*/, int sockfd) {
  releaseAcc(sockfd);
  sink.drop(sockfd);
  close(sockfd);          // httpd hands ownership of the fd to this hook
}

// ── Lifecycle ───────────────────────────────────────────────────────────────
// Self-contained: on any failure it says so and leaves everything inert, so a
// WebSocket that will not start can never take the relay down with it.
inline bool begin() {
  wsQueue = xQueueCreate(WS_QUEUE_DEPTH, sizeof(WsCmd));
  if (!wsQueue) { Serial.println("[relay] ws: queue alloc FAILED - WebSocket disabled"); return false; }

  httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
  cfg.max_open_sockets = WS_MAX_CLIENTS;   // keep in step with WS_MAX_CLIENTS above
  cfg.close_fn         = wsClose;
  cfg.lru_purge_enable = true;
  if (httpd_start(&wsServer, &cfg) != ESP_OK) {
    Serial.println("[relay] ws: httpd_start FAILED - WebSocket disabled");
    vQueueDelete(wsQueue); wsQueue = nullptr; wsServer = nullptr;
    return false;
  }

  httpd_uri_t uri  = {};
  uri.uri          = "/ws";
  uri.method       = HTTP_GET;
  uri.handler      = wsHandler;
  uri.user_ctx     = nullptr;
  uri.is_websocket = true;
  if (httpd_register_uri_handler(wsServer, &uri) != ESP_OK) {
    Serial.println("[relay] ws: URI register FAILED - WebSocket disabled");
    httpd_stop(wsServer); wsServer = nullptr;
    vQueueDelete(wsQueue); wsQueue = nullptr;
    return false;
  }
  return true;
}

// Run at most ONE queued command per loop() pass, on Core 1. One at a time is the
// honest ceiling: relaySerialLine() may block on ETM retries, and running a burst
// back-to-back would stall wcb.update() (heartbeats, offline detection, WDP).
inline void drain() {
  if (!wsQueue) return;
  WsCmd cmd;
  if (xQueueReceive(wsQueue, &cmd, 0) != pdTRUE) return;
  sink.begin(cmd.fd);            // ensure the reply reaches the client that asked
  relaySerialLine(cmd.line);
}

inline void pump() { sink.pump(); }

}  // namespace mgmtws
