///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Body Controller ESP32 — GUI Config Edition
//  Based on Body_Controller_ESP32 by Greg Hulette
//
//  Additions over the base firmware:
//    • NVS-backed RC button mapping config  (rc_config.h)
//    • WebSerial JSON protocol on USB Serial (see handleWebSerialMessage())
//    • Live PWM monitor mode — streams {"type":"PWM","ch6":###,"ch16":###} at 50 ms
//    • RCRadio_Matrix_Buttons() and dispatchDeferredButton() now use NVS config
//    • callAnimation() extends the animation table to case 40
//
//  USB Serial JSON Protocol (newline-delimited):
//    PING                        → {"type":"PONG"}
//    GET_CONFIG                  → {"type":"CONFIG","data":{...full config JSON...}}
//    {"type":"SET_CONFIG","data":{...}} → {"type":"ACK","ok":true}
//    {"type":"START_MONITOR"}    → streams PWM_UPDATE every 50 ms until STOP_MONITOR
//    {"type":"STOP_MONITOR"}     → {"type":"ACK","ok":true}
//    {"type":"RESET_DEFAULTS"}   → reloads factory defaults, replies ACK
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//  Libraries
//////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
#include <ElegantOTA.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#define ETM_MY_BOARD_INDEX ETM_BOARD_BC
// 2026-05 (v6): bumped from default 8 → 16 to give more in-flight slots
// for the GET_CONFIG chunked-response burst (~98 chunks @ 1Hz). With
// only 8 slots and 500ms ETM timeout × 3 retries, the pending table
// could fill up and `etmAddToPending` would silently start returning -1
// (chunk sent but untracked → not retried if lost over the air).
#define ETM_PENDING_MAX 16
#include <ETM_Droid.h>
#include <Adafruit_NeoPixel.h>
#include "body_controller_esp32_pin_map.h"
#include <DebugR2.h>
#include <ReelTwo.h>
#include "core/DelayCall.h"
#include <SoftwareSerial.h>
#include "ArduinoJson.h"
#include <hcr.h>
#include "sbus.h"           // bfs library — kept around for any other parts of code that reference it (unused for RX now)
#include "sbus_reader.h"    // custom auto-detect 16/24 channel reader (replaces bfs::SbusRx)
#include <Preferences.h>
#include "rc_config.h"

//////////////////////////////////////////////////////////////////////
//  BENCH TEST mode
//  --------------------------------------------------------------
//  Uncomment to compile a "naked board" build for the S3 DevKitC:
//    • All peripheral Serial.begin() calls are skipped
//    • WiFi / ESP-NOW init is skipped
//    • rcExecuteAction prints "[DISPATCH] …" instead of calling
//      sendESPNOWCommand / HCRFunction / callAnimation
//    • A new WebSerial "TRIGGER" message lets the GUI fire a
//      virtual button press without needing an SBUS receiver.
//
//  Comment out for the real build that drives the hardware.
//////////////////////////////////////////////////////////////////////
// #define BENCH_TEST

//////////////////////////////////////////////////////////////////////
//  User preferences
//////////////////////////////////////////////////////////////////////
String ESPNOWPASSWORD = "GregsAstromech";
const char* ssid      = "R2D2_Control_Network";
const char* password  = "astromech";
bool STATUS_TRACKING  = 1;
int  keepAliveDuration = 4000;
int  defaultESPNOWSendDuration = 50;

#define BL_BAUD_RATE      9600
#define RD_BAUD_RATE      9600
#define ST_BAUD_RATE      9600
#define MP_BAUD_RATE      9600
#define SERIAL1_BAUD_RATE 9600
#define SERIAL2_BAUD_RATE 9600

//////////////////////////////////////////////////////////////////////
//  RC Config global (defined here — declared extern in rc_config.h)
//////////////////////////////////////////////////////////////////////
RcConfig rcConfig;

//////////////////////////////////////////////////////////////////////
//  Command variables
//////////////////////////////////////////////////////////////////////
String HOSTNAME = "Body Controller GUI";
char   inputBuffer[200];
String inputString;
volatile boolean stringComplete = false;
String autoInputString;
volatile boolean autoComplete   = false;
int commandLength;
String serialCommandString, serialPort, serialCommandSubString;
String ESPNOWCommandString, ESPNOWTarget, ESPNOWTargetCommand;
String mp3CommandString;
String eepromCommandString;
String ledCommandString, radhCommandString, controllerCommandString;
uint32_t Internal_Command[6]  = {0,0,0,0,0,0};
int internalCommandFunction = 0;
uint32_t Animation_Command[6] = {0,0,0,0,0,0};
int AnimationCommandFunction  = 0;
debugClass Debug;
String debugInputIdentifier = "";
int mp3Track;
String mp3Comm, HCRTriggerResponseString;
int* getEmotions;
int getEmotion;
float getDuration;
int getOverride;
bool isPlaying;
int getMuse, getWAVCount, getPlayingWAV, HCRVolume;
int happyEmotionValue, sadEmotionValue, madEmotionValue, scaredEmotionValue;

//////////////////////////////////////////////////////////////////////
//  Startup / loop timing
//////////////////////////////////////////////////////////////////////
boolean startUp  = true;
boolean isStartUp = true;
unsigned long mainLoopTime;
unsigned long MLMillis;
uint8_t mainLoopDelayVar = 5;

//////////////////////////////////////////////////////////////////////
//  Status variables
//////////////////////////////////////////////////////////////////////
unsigned long keepAliveMillisDuration = 15000;
unsigned long blkeepAliveAge, blkeepaliveAgeMillis, keepAliveMillis;
int BL_LDP_Bright, BL_MAINT_Bright, BL_VU_Bright, BL_CS_Bright;
int BL_vuOffsetInt, BL_vuBaselineInt, BL_vuOffsetExt, BL_vuBaselineExt;
float BL_BatteryVoltage;
int BL_BatteryPercentage;
int FunctionSwState = 1;
bool bodyLEDControllerStatus = 0;
bool remoteConnected = false;

//////////////////////////////////////////////////////////////////////
//  Serial ports
//////////////////////////////////////////////////////////////////////
// 2026-06: blSerial (Body LED Controller / ATMEGA link) moved from bit-banged
// SoftwareSerial → hardware UART1. The SoftwareSerial instance on GPIO15 was
// dropping every byte under the GUI's WiFi/ESP-NOW load — the Mega received
// nothing even though write() ran, while other SoftwareSerial peripherals on
// this board kept working. Hardware UART has dedicated timing immune to that.
// UART1 is remapped to the SAME pins (RX=16 / TX=15) in begin(), so NO PCB
// change. To free UART1, rdSerial (RADH) drops to SoftwareSerial — it's
// low-rate and tolerates it (per the working-SoftwareSerial evidence above).
HardwareSerial blSerial(1);
SoftwareSerial rdSerial, stSerial, mpSerial, s1Serial, s2Serial;
HCRVocalizer HCR(&mpSerial, MP_BAUD_RATE);

//////////////////////////////////////////////////////////////////////
//  SBUS — custom reader supports both 16-channel standard SBUS and
//  FrSky's 24-channel "SBUS-24" extended frame, auto-detected per frame.
//////////////////////////////////////////////////////////////////////
#define SBUS
#ifdef SBUS
SbusReader sbus_rx;
#endif

//////////////////////////////////////////////////////////////////////
//  WiFi / IP
//////////////////////////////////////////////////////////////////////
IPAddress local_IP(192,168,4,109);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,4,100);
const uint8_t oldLocalMACAddress[] = {0x24,0x0A,0xC4,0xED,0x30,0x12};
AsyncWebServer server(80);

//////////////////////////////////////////////////////////////////////
//  Status LED
//////////////////////////////////////////////////////////////////////
const uint32_t red=0xFF0000, orange=0xFF8000, yellow=0xFFFF00, green=0x00FF00;
const uint32_t cyan=0x00FFFF, blue=0x0000FF, magenta=0xFF00FF, white=0xFFFFFF, off=0x000000;
const uint32_t basicColors[9] = {off,red,yellow,green,cyan,blue,magenta,orange,white};
#define STATUS_LED_COUNT 1
Adafruit_NeoPixel ESP_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, STAUS_LED_PIN, NEO_GRB + NEO_KHZ800);

//////////////////////////////////////////////////////////////////////
//  WebSerial monitor state
//////////////////////////////////////////////////////////////////////
bool          wsMonitorActive   = false;
unsigned long wsMonitorLastSent = 0;
// While the config tool's calibration wizard is open the operator deliberately
// wiggles every stick/knob/switch/button. Suppress ALL action dispatch while
// this is set so nothing fires during cal. Set/cleared via
// {"type":"CALIB","on":bool}; also force-cleared by STOP_MONITOR and a fresh
// PING so a crashed/closed page can never leave the board permanently muted.
bool          calibrationActive = false;
#define WS_MONITOR_INTERVAL_MS  50

// Small queue for delayed config-driven actions (avoids lambda-capture problem)
struct PendingAction {
  bool     active;
  unsigned long fireAt;
  RcAction action;
};
#define PENDING_ACTION_SLOTS 6
PendingAction pendingActions[PENDING_ACTION_SLOTS];

//////////////////////////////////////////////////////////////////////
//  ESP-NOW
//////////////////////////////////////////////////////////////////////
String  senderID, targetID;
bool    commandIncluded;
String  command;
uint32_t SuccessCounter = 0, FailureCounter = 0;
String  incomingTargetID, incomingSenderID, incomingCommand;
bool    incomingCommandIncluded;
String  incomingPassword;
String  success;

// ── Chunked-config reassembly state ─────────────────────────────────────────
// ESP-NOW caps the command field at 100 bytes (see espnow_struct_message),
// so a 3-4 KB SET_CONFIG payload arrives as N small chunks and gets stitched
// back together here before being fed to the existing JSON SET_CONFIG path.
//
//   Wire protocol (handled in the main-loop '#' parser):
//     #CB <seq> <total>            — Begin a transaction
//     #CC <seq> <idx>:<rawBytes>   — One chunk; raw JSON after the colon
//     #CE <seq>                    — Commit; parse buffer + apply
//
// Stale transactions auto-abort after CFG_REASM_TIMEOUT_MS so a dropped
// END or lost connectivity doesn't lock the slot forever. A new #CB always
// resets state regardless of any active transaction.
#define CFG_REASM_MAX        6144     // bytes — bigger than any expected config payload
#define CFG_REASM_TIMEOUT_MS 10000    // ms of silence before stale buffer is dropped
String   cfgReasmBuf;
uint16_t cfgReasmSeq      = 0;        // 0 = no active transaction
uint16_t cfgReasmTotal    = 0;
uint16_t cfgReasmReceived = 0;
unsigned long cfgReasmLastMs = 0;
// 2026-05: when the chunked END handler hands the assembled JSON to
// handleWebSerialMessage(), cfgReasmSeq gets reset to 0 — but the
// SET_CONFIG handler still needs to echo that seq back in its ACK so
// the GUI can match the ACK to its push. Stash it here just before the
// reset; the handler reads it when emitting sendBcConfigAck().
uint16_t lastChunkSeqForAck = 0;

// 2026-05 GET_CONFIG return path. When the GUI sends GET_CONFIG, the BC
// captures rcConfigToJSON() output here and dribbles it out as 100-byte
// chunks (riding the periodic BC→DG status struct) at ~50ms cadence
// until pendingCfgIdx == pendingCfgTotal. While idle, pendingCfgTotal=0
// and the cfg-chunk fields in the outgoing struct stay zeroed so the
// chunk doesn't replay on keepalive frames.
String        pendingCfgJson    = "";
uint16_t      pendingCfgSeq     = 0;
uint16_t      pendingCfgIdx     = 0;
uint16_t      pendingCfgTotal   = 0;
unsigned long pendingCfgLastMs  = 0;
// 2026-05 (v4): chunks now ride the ETM-tracked command path
// (sendESPNOWCommand("DG", text) → outgoingMsg.structCommand[100], packed
// as "#C <seq> <idx> <total>:<payload>"). Header overhead is at most
// "#C 65535 999 999:" = 18 bytes, so payload max = 100 - 18 = 82 bytes;
// rounding down to 80 leaves comfortable margin.
#define BC_CFG_CHUNK_BYTES 80
// 2026-05 (v6): was 1000ms, tuned for the OLD slow Gateway LoRa drain — the
// four factors that forced 1000ms were (a) 600ms inter-chunk throttle, (b)
// 200ms LoRa airtime/chunk, (c) 1Hz telemetry stealing ~250ms, (d) inline
// LoRa-ACK eating ~250ms.
// 2026-06: every one of those collapsed — (a) Gateway gap now 45ms, (b) 500kHz
// → ~48ms airtime/chunk, (c) telemetry shrank 252→52B AND 500kHz → ~13ms steal
// (and telemetry is suppressed entirely during a transfer anyway), (d) already
// deferred. The Gateway now drains a chunk every ~85ms (ACK-gated), so the BC
// can feed much faster. 175ms keeps the BC comfortably SLOWER than the Gateway
// drain (~2× margin) so the Gateway's 32-slot queue stays shallow — feeding
// faster than the drain would overflow it and corrupt the config.
// Result: ~98 chunks × 175ms ≈ ~17s (was ~98s). If hardware shows the queue
// staying near-empty (no "[CFG] queue full"), this can be pushed toward ~120ms.
#define BC_CFG_CHUNK_INTERVAL_MS 175

// MUST match the Droid_Gateway.ino struct of the same name EXACTLY —
// both sides share this layout for the ESP-NOW packet between BC and DG.
//
// 2026-05 NOTE: The reverse-channel ACK and GET_CONFIG cfg-chunk fields
// USED TO live in this struct. They were briefly moved into a separate
// BcEvent_struct_message and dispatched by len, but that path used raw
// esp_now_send() with no retries — 10-20% of chunks dropped silently.
// 2026-05 (v4): ACKs and chunks now ride the ETM-tracked command path
// (sendESPNOWCommand("DG", text) packs them as "#A ..." / "#C ..." TEXT
// commands inside outgoingMsg.structCommand). ETM provides sequence
// numbers, ACK tracking, and 3 retries with 500ms timeout. The
// BcEvent_struct_message typedef is gone.
typedef struct bodyControllerStatus_struct_message {
  char   structPassword[25];
  char   structSenderID[15];
  char   structTargetID[5];
  int    structBL_LDP_Bright, structBL_MAINT_Bright, structBL_VU_Bright, structBL_CS_Bright;
  int    structBL_vuOffsetInt, structBL_vuBaselineInt, structBL_vuOffsetExt, structBL_vuBaselineExt;
  float  structBL_BatteryVoltage;
  int    structBL_BatteryPercentage;
  bool   structbodyLEDControllerStatus;
  int    structFunctionSWState;
  bool   struct_remoteConnected;
  bool   structCommandIncluded;
  uint32_t structSuccess, structFailure;
  char   structCommand[100];
  bool     etmBoardOnline[ETM_NUM_BOARDS];
  uint32_t etmBoardSent[ETM_NUM_BOARDS];
  uint32_t etmBoardAckd[ETM_NUM_BOARDS];
  uint32_t etmBoardRetries[ETM_NUM_BOARDS];
  uint32_t etmBoardFailed[ETM_NUM_BOARDS];
} bodyControllerStatus_struct_message;

// 2026-05 (v4): BcEvent_struct_message removed. ACKs and GET_CONFIG
// chunks now ride the ETM-tracked command path as text-encoded payloads
// inside outgoingMsg.structCommand. See sendBcConfigAck() /
// sendBcCfgChunk() below — they build "#A ..." / "#C ..." strings and
// hand them to sendESPNOWCommand("DG", s), which uses ETM's sequence /
// ACK / 3-retry machinery instead of raw esp_now_send().

bodyControllerStatus_struct_message commandsToSendtoDroidLoRa;
espnow_struct_message outgoingMsg, incomingMsg;
esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == 0) SuccessCounter++; else FailureCounter++;
  // 2026-05 (v4): chunk in-flight tracking removed — ETM handles
  // delivery, ACK matching, and retries for the chunk/ACK text frames
  // now. OnDataSent is back to just counting outcomes.
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  colorWipeStatus("ES", orange, 255);
  if (len < (int)sizeof(espnow_struct_message)) { colorWipeStatus("ES", blue, 10); return; }
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
  if (strncmp(incomingMsg.structPassword, ESPNOWPASSWORD.c_str(), sizeof(incomingMsg.structPassword)) != 0) {
    colorWipeStatus("ES", blue, 10); return;
  }
  int senderIdx = etmBoardIndexFromMAC(esp_now_info->src_addr);
  if (senderIdx >= 0) etmHandleHeartbeat(senderIdx);
  switch (incomingMsg.structPacketType) {
    case PACKET_TYPE_HEARTBEAT: break;
    case PACKET_TYPE_ACK:
      etmProcessAck(senderIdx, incomingMsg.structSequenceNumber); break;
    case PACKET_TYPE_COMMAND:
      if (senderIdx < 0) break;
      incomingSenderID  = incomingMsg.structSenderID;
      incomingTargetID  = incomingMsg.structTargetID;
      incomingCommandIncluded = incomingMsg.structCommandIncluded;
      incomingCommand   = incomingMsg.structCommand;
      etmSendAck(senderIdx, incomingMsg.structSequenceNumber);   // ACK FIRST so a lost-ACK resend stops retrying
      if (incomingMsg.structSequenceNumber != 0 && etmCmdSeqDup(senderIdx, incomingMsg.structSequenceNumber))
        break;   // exact retransmit of a command we already ran -- ACKed above, don't re-execute
      processESPNOWIncomingMessage();
      break;
    default: break;
  }
  colorWipeStatus("ES", blue, 10);
}

void processESPNOWIncomingMessage() {
  if (incomingTargetID == "BC" || incomingTargetID == "BR") {
    inputString    = incomingCommand;
    stringComplete = true;
  }
}

#define RST RESET_PIN_2560

///////////////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////

void colorWipeStatus(String statusled, uint32_t c, int brightness) {
  if (statusled == "ES") {
    ESP_LED.setBrightness(brightness);
    ESP_LED.setPixelColor(0, c);
    ESP_LED.show();
  }
}

void keepAlive() {
  if (STATUS_TRACKING == 1) {
    if (millis() - keepAliveMillis >= (keepAliveDuration + random(1, 1000))) {
      keepAliveMillis = millis();
      sendESPNOWCommand("DG", "");
    }
  }
}

void checkAgeofkeepAlive() {
  if (bodyLEDControllerStatus == 1) {
    if (millis() - blkeepAliveAge >= keepAliveMillisDuration) {
      bodyLEDControllerStatus = 0;
      BL_LDP_Bright = BL_MAINT_Bright = BL_VU_Bright = BL_CS_Bright = 0;
      BL_vuOffsetInt = BL_vuBaselineInt = BL_vuOffsetExt = BL_vuBaselineExt = 0;
      BL_BatteryPercentage = 0; BL_BatteryVoltage = 0.0;
    }
  }
}

//////////////////////////////////////////////////////////////////////
//  Serial events
//////////////////////////////////////////////////////////////////////
// 2026-05 fix (BUG #4): the previous implementation did `inputString += inChar`
// per byte. Arduino String reallocates on every concatenation, so a multi-KB
// line over direct USB caused O(N²) heap thrash. Accumulate into a fixed
// static buffer and hand the full line over only once it's complete.
//
// Buffer size: 2 KB is plenty for the realistic direct-USB use cases:
//   - Typed commands (`#L00`, `#DETM`, etc.) — tens of bytes
//   - Pasted single-line JSON for testing — hundreds of bytes to a couple KB
// LARGE SET_CONFIG payloads (~25 KB) DO NOT come in via direct USB; the web
// GUI sends them as a chunked sequence (`#CB`/`#CC`/`#CE`) which is
// reassembled in the separate `cfgReasmBuf` String (capacity 6 KB). So
// lineBuf doesn't need to hold a full SET_CONFIG.
//
// Sized for the chunked-config path: every config line that arrives here is a
// single #CB/#CC/#CE chunk (~90 chars) or a normal command, never a full
// multi-KB config blob — so 1 KB is plenty and overflow simply NACKs the line.
// (Was 2 KB, but DRAM `.dram0.bss` overflowed the ESP32 segment by 552 bytes
// once the rest of the GUI firmware grew; 1 KB frees ~1 KB and links cleanly.)
// If you ever need to paste a multi-KB SET_CONFIG via direct USB, use the
// chunked path instead (or move this buffer to PSRAM with `ps_malloc()`).
static char lineBuf[1024];
static size_t lineLen = 0;
void serialEvent() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\r' || inChar == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        if (lineBuf[0] == '{') {
          // WebSerial JSON protocol line — hand directly to the JSON handler
          // without paying String += cost.
          handleWebSerialMessage(String(lineBuf));
        } else {
          // Non-JSON line goes through the existing stringComplete path.
          inputString = lineBuf;
          stringComplete = true;
        }
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = inChar;
    } else {
      // Buffer full — drop the line so we don't silently truncate JSON.
      // (Realistic SET_CONFIG fits comfortably; this only fires on a
      // genuine pathological line.)
      Serial.println("{\"type\":\"NACK\",\"reason\":\"serial_line_overflow\"}");
      lineLen = 0;
      // Drain remaining chars of this line so we resync on the next \r/\n.
      while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') break;
      }
    }
  }
}

void serialRdEvent() {
  while (rdSerial.available()) {
    char inChar = (char)rdSerial.read();
    inputString += inChar;
    if (inChar == '\r') stringComplete = true;
  }
}

void serialBlEvent() {
  while (blSerial.available()) {
    StaticJsonDocument<2048> doc;
    colorWipeStatus("ES", green, 10);
    DeserializationError err = deserializeJson(doc, blSerial);
    if (err == DeserializationError::Ok) {
      bodyLEDControllerStatus = doc["bodyLEDControllerStatus"].as<bool>();
      if (bodyLEDControllerStatus == 1) blkeepAliveAge = millis();
      BL_LDP_Bright = doc["LDPBright"].as<int>();
      BL_MAINT_Bright = doc["MaintBright"].as<int>();
      BL_VU_Bright = doc["VUBright"].as<int>();
      BL_CS_Bright = doc["CoinBright"].as<int>();
      BL_vuOffsetInt = doc["VUIntOffset"].as<int>();
      BL_vuBaselineInt = doc["VUIntBaseline"].as<int>();
      BL_vuOffsetExt = doc["VUExtOffset"].as<int>();
      BL_vuBaselineExt = doc["VUExtBaseline"].as<int>();
      BL_BatteryVoltage = doc["BatteryVoltage"].as<float>();
      BL_BatteryPercentage = doc["BatteryPercent"].as<int>();
    }
  }
  colorWipeStatus("ES", blue, 10);
}

void serialStEvent() {
  while (stSerial.available()) {
    char inChar = (char)stSerial.read();
    inputString += inChar;
    if (inChar == '\r') { inputString = ":" + inputString; stringComplete = true; }
  }
}

void serialMpEvent() {
  while (mpSerial.available()) {
    char inChar = (char)mpSerial.read();
    HCRTriggerResponseString += inChar;
    if (inChar == '\r') {}
  }
  HCRTriggerResponseString = "";
}

void serial1Event() {
  while (s1Serial.available()) {
    char inChar = (char)s1Serial.read();
    inputString += inChar;
    if (inChar == '\r') stringComplete = true;
  }
}
void serial2Event() {
  while (s2Serial.available()) {
    char inChar = (char)s2Serial.read();
    inputString += inChar;
    if (inChar == '\r') stringComplete = true;
  }
}

//////////////////////////////////////////////////////////////////////
//  Serial write helpers
//////////////////////////////////////////////////////////////////////
void writeSerialString(String s) { Serial.print(s + '\r'); }
void writeRdSerial(String s) { for (char c : (s + '\r')) rdSerial.write(c); }
// 2026-06 FIX: the BC→ATMEGA LED-command TX is BIT-BANGED on GPIO15 as a plain
// GPIO — NOT a hardware UART, NOT SoftwareSerial. On this board both of those got
// silently killed: SBUS's UART2 detaches the UART1 TX pin in the GPIO matrix, and
// any active EspSoftwareSerial instance corrupts it too (write() ran but nothing
// left the pin). A direct digitalWrite bit-bang can't be detached or clobbered.
// Interrupts are held off per byte (like EspSoftwareSerial does) so the 9600-baud
// bit timing stays clean; this is on core 1, so WiFi/ESP-NOW (core 0) is unaffected.
// blSerial (UART1) is kept RX-ONLY on GPIO16 to read the LED board's status JSON.
static void blTxByte(uint8_t b) {
  const uint32_t BIT_US = 104;            // 1,000,000 / 9600
  noInterrupts();
  digitalWrite(SERIAL_TX_BL_PIN, LOW);  delayMicroseconds(BIT_US);            // start bit
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(SERIAL_TX_BL_PIN, (b >> i) & 1);
    delayMicroseconds(BIT_US);
  }
  digitalWrite(SERIAL_TX_BL_PIN, HIGH); delayMicroseconds(BIT_US);            // stop bit
  interrupts();
}
void writeBlSerial(String s) {
  String out = s + '\r';
  for (size_t i = 0; i < out.length(); i++) blTxByte((uint8_t)out[i]);
}
void writeStSerial(String s) { for (char c : (s + '\r')) stSerial.write(c); }
void writeMpSerial(String s) { for (char c : (s + '\r')) mpSerial.write(c); }
void writeS1Serial(String s) { for (char c : (s + '\r')) s1Serial.write(c); }
void writeS2Serial(String s) { for (char c : (s + '\r')) s2Serial.write(c); }

//////////////////////////////////////////////////////////////////////
//  ESP-NOW send
//////////////////////////////////////////////////////////////////////
void setupSendStatusStruct(bodyControllerStatus_struct_message* msg, String pass,
                            String sender, String targetID, bool hasCmd, String cmd) {
  snprintf(msg->structPassword, 25, "%s", pass.c_str());
  snprintf(msg->structSenderID, 15, "%s", sender.c_str());
  snprintf(msg->structTargetID, 5,  "%s", targetID.c_str());
  msg->structBL_LDP_Bright = BL_LDP_Bright; msg->structBL_MAINT_Bright = BL_MAINT_Bright;
  msg->structBL_VU_Bright  = BL_VU_Bright;  msg->structBL_CS_Bright    = BL_CS_Bright;
  msg->structBL_vuOffsetInt = BL_vuOffsetInt; msg->structBL_vuBaselineInt = BL_vuBaselineInt;
  msg->structBL_vuOffsetExt = BL_vuOffsetExt; msg->structBL_vuBaselineExt = BL_vuBaselineExt;
  msg->structBL_BatteryVoltage = BL_BatteryVoltage;
  msg->structBL_BatteryPercentage = BL_BatteryPercentage;
  msg->structbodyLEDControllerStatus = bodyLEDControllerStatus;
  msg->structFunctionSWState = FunctionSwState;
  msg->struct_remoteConnected = remoteConnected;
  msg->structCommandIncluded = hasCmd;
  msg->structSuccess = SuccessCounter; msg->structFailure = FailureCounter;
  snprintf(msg->structCommand, 100, "%s", cmd.c_str());
  for (int i = 0; i < ETM_NUM_BOARDS; i++) {
    msg->etmBoardOnline[i]  = etmBoardTable[i].online;
    msg->etmBoardSent[i]    = etmBoardTable[i].messagesSent;
    msg->etmBoardAckd[i]    = etmBoardTable[i].messagesAckd;
    msg->etmBoardRetries[i] = etmBoardTable[i].totalRetries;
    msg->etmBoardFailed[i]  = etmBoardTable[i].totalFailed;
  }
}

// 2026-05 (v4): reverse-channel ACK from BC → GUI, now routed through
// ETM. Packs the ACK as a TEXT command "#A <seq> <ok> <msg>" and hands
// it to sendESPNOWCommand("DG", s) — which carries it as an
// ETM-tracked command frame (sequence number, gateway ACK, 3 retries
// with 500ms timeout). The Gateway recognizes the "#A " prefix in
// processESPNOWIncomingMessage() and copies seq/ok/msg into the same
// BC_bcAck* globals the LoRa telemetry relay already uses.
//
// `seq`  — the chunked-push sequence number this ACK corresponds to,
//          OR 0 for non-chunked replies (direct USB SET_CONFIG, GET_CONFIG,
//          RESET_DEFAULTS). Web side filters by matching state.lastSentSeq.
// `ok`   — true for success, false for NACK.
// `msg`  — short status string (≤23 chars); examples:
//          "applied", "parse_fail", "incomplete", "out_of_order",
//          "wrong_seq", "overflow", "missing_data", "apply_failed".
void sendBcConfigAck(uint16_t seq, bool ok, const char *msg) {
  String s = "#A ";
  s += String(seq);
  s += " ";
  s += (ok ? "1" : "0");
  s += " ";
  s += (msg ? msg : "");
  sendESPNOWCommand("DG", s);
  // Also print on USB for direct-cable debugging.
  Serial.printf("{\"type\":\"%s\",\"seq\":%u,\"ok\":%s,\"msg\":\"%s\"}\n",
                ok ? "ACK" : "NACK", seq, ok ? "true" : "false", msg ? msg : "");
}

// 2026-05 (v4): GET_CONFIG chunk send, also routed through ETM. Packs the
// chunk as "#C <seq> <idx> <total>:<payload>" inside outgoingMsg.structCommand
// (max 100 bytes). Header overhead is at most "#C 65535 999 999:" = 18
// bytes, leaving room for BC_CFG_CHUNK_BYTES (80) of payload.
// ETM's sequence/ACK/retry handle reliable delivery — no in-flight flag,
// no manual retry loop, no OnDataSent inspection.
void sendBcCfgChunk(uint16_t seq, uint16_t idx, uint16_t total,
                    uint8_t len, const char *payload) {
  String s = "#C ";
  s += String(seq);
  s += " ";
  s += String(idx);
  s += " ";
  s += String(total);
  s += ":";
  for (uint8_t i = 0; i < len; i++) s += payload[i];
  sendESPNOWCommand("DG", s);
  Serial.printf("[CFG] SEND chunk seq=%u idx=%u/%u len=%u (ETM)\n",
                seq, idx, total, len);
}

void sendESPNOWCommand(String starget, String scomm) {
  bool hasCommand = (scomm.length() > 0);
  // 2026-05 (v4): only the *heartbeat* path (empty scomm) still uses the
  // big bodyControllerStatus_struct_message — that struct carries
  // periodic telemetry (battery, LED-controller status, ETM stats, etc.)
  // and is dispatched by len on the Gateway side. Non-empty commands to
  // "DG" (e.g. the "#A ..." ACKs and "#C ..." cfg chunks built by
  // sendBcConfigAck/sendBcCfgChunk) fall through to the ETM-tracked
  // path below so they get sequence numbers, gateway ACKs, and retries.
  if (starget == "DG" && !hasCommand) {
    setupSendStatusStruct(&commandsToSendtoDroidLoRa, ESPNOWPASSWORD, "BC", starget, hasCommand, scomm);
    esp_now_send(ETM_BOARD_MACS[ETM_BOARD_DG], (uint8_t*)&commandsToSendtoDroidLoRa, sizeof(commandsToSendtoDroidLoRa));
    return;
  }
  bool isBroadcast = (starget == "BR");
  int  targetIdx   = etmBoardIndexFromID(starget.c_str());
  const uint8_t* targetMAC = isBroadcast ? ETM_BROADCAST_MAC
                             : (targetIdx >= 0) ? ETM_BOARD_MACS[targetIdx] : nullptr;
  if (!targetMAC) return;
  uint16_t seqNum = etmNextSequence();
  memset(&outgoingMsg, 0, sizeof(outgoingMsg));
  snprintf(outgoingMsg.structPassword, sizeof(outgoingMsg.structPassword), "%s", ESPNOWPASSWORD.c_str());
  snprintf(outgoingMsg.structSenderID, sizeof(outgoingMsg.structSenderID), "BC");
  snprintf(outgoingMsg.structTargetID, sizeof(outgoingMsg.structTargetID), "%s", starget.c_str());
  outgoingMsg.structCommandIncluded = hasCommand;
  outgoingMsg.structSuccess = SuccessCounter; outgoingMsg.structFailure = FailureCounter;
  snprintf(outgoingMsg.structCommand, sizeof(outgoingMsg.structCommand), "%s", scomm.c_str());
  outgoingMsg.structPacketType     = PACKET_TYPE_COMMAND;
  outgoingMsg.structSequenceNumber = seqNum;
  esp_err_t result = esp_now_send(targetMAC, (uint8_t*)&outgoingMsg, sizeof(outgoingMsg));
  if (result == ESP_OK && !isBroadcast) etmAddToPending(scomm.c_str(), starget.c_str(), targetIdx, seqNum);
  Debug.ESPNOW("Send to %s (idx %d, seq %d): %s — result: %s\n", starget.c_str(), targetIdx, seqNum, scomm.c_str(), esp_err_to_name(result));
}

//////////////////////////////////////////////////////////////////////
//  Misc helpers
//////////////////////////////////////////////////////////////////////
void resetArduino(int d) { digitalWrite(RST, LOW); delay(d); digitalWrite(RST, HIGH); }

String getValue(String data, char sep, int idx) {
  int found = 0, maxI = data.length() - 1;
  int si[] = {0, -1};
  for (int i = 0; i <= maxI && found <= idx; i++) {
    if (data.charAt(i) == sep || i == maxI) {
      found++; si[0] = si[1] + 1;
      si[1] = (i == maxI) ? i + 1 : i;
    }
  }
  return found > idx ? data.substring(si[0], si[1]) : "";
}

void mp3Trigger(String comm, int track) { mpSerial.print(comm); mpSerial.write(track); }

void HCRFunction(int command=0, int chan=0, int track=0, String filename="") {
  switch (command) {
    case 1:  HCR.update();                        break;
    case 2:  HCR.SetEmotion(chan,track); HCR.update(); break;
    case 3:  HCR.Trigger(chan,track);             break;
    case 4:  HCR.Stimulate(chan,track);           break;
    case 5:  HCR.Overload();                      break;
    case 6:  HCR.Muse();                          break;
    case 7:  HCR.Muse(chan,track);                break;
    case 8:  HCR.Stop();                          break;
    case 9:  HCR.StopEmote();                     break;
    case 10: HCR.OverrideEmotions(chan);          break;
    case 11: HCR.ResetEmotions();                 break;
    case 12: HCR.SetEmotion(chan,track);          break;
    case 13: HCR.SetMuse(track);                  break;
    case 14: HCR.PlayWAV(chan,track); HCR.update(); break;
    case 15: HCR.PlayWAV(chan,filename);          break;
    case 16: HCR.StopWAV(chan);                   break;
    case 17: HCR.SetVolume(chan,track);           break;
    case 18: getEmotions = HCR.GetEmotions();     break;
    case 19: getEmotion  = HCR.GetEmotion(chan);  break;
    case 20: getDuration = HCR.GetDuration();     break;
    case 21: getOverride = HCR.GetOverride();     break;
    case 22: isPlaying   = HCR.IsPlaying();       break;
    case 24: isPlaying   = HCR.IsPlaying(chan);   break;
    case 25: getWAVCount    = HCR.GetWAVCount();  break;
    case 26: getPlayingWAV  = HCR.GetPlayingWAV(chan); break;
    case 27: HCR.getUpdate();                     break;
    case 28: HCRVolume = HCR.getVolume(chan);     break;
    case 29: HCR.getUpdate(); getHCRStatus();     break;
  }
}

void getHCRStatus() {
  happyEmotionValue  = HCR.GetEmotion(0);
  sadEmotionValue    = HCR.GetEmotion(1);
  madEmotionValue    = HCR.GetEmotion(2);
  scaredEmotionValue = HCR.GetEmotion(3);
}

// Full Body LED Controller status dump (invoked from #L08).
// Mirrors the original BC firmware — toggles debugflag_status on the first
// call so the print fires regardless of the current debug-flag state.
void printKeepaliveStatus() {
  if (Debug.debugflag_status == 0) {
    Debug.debugflag_status = 1;
    Debug.STATUS("Body LED Controller Status: %d\n", bodyLEDControllerStatus);
    Debug.STATUS("LDP Bright: %i\n", BL_LDP_Bright);
    Debug.STATUS("Maintenence Bright: %i\n", BL_MAINT_Bright);
    Debug.STATUS("VU Bright: %i\n", BL_VU_Bright);
    Debug.STATUS("Coin SLots Bright: %i\n", BL_CS_Bright);
    Debug.STATUS("vuOffsetInt: %i\n", BL_vuOffsetInt);
    Debug.STATUS("vuBaselineInt: %i\n", BL_vuBaselineInt);
    Debug.STATUS("vuOffsetExt: %i\n", BL_vuOffsetExt);
    Debug.STATUS("vuBaselineExt: %i\n", BL_vuBaselineExt);
    Debug.STATUS("BL_BatteryVoltage: %f\n", BL_BatteryVoltage);
    Debug.STATUS("BL_BatteryPercentage: %i\n", BL_BatteryPercentage);
    Debug.STATUS(("Fucntion Sw State: %i\n"), FunctionSwState);
    Debug.STATUS("Remote Connected Status: %i\n", remoteConnected);
    Debug.debugflag_status = 0;
  } else {
    Debug.STATUS("Body LED Controller Status: %d\n", bodyLEDControllerStatus);
    Debug.STATUS("LDP Bright: %i\n", BL_LDP_Bright);
    Debug.STATUS("Maintenence Bright: %i\n", BL_MAINT_Bright);
    Debug.STATUS("VU Bright: %i\n", BL_VU_Bright);
    Debug.STATUS("Coin SLots Bright: %i\n", BL_CS_Bright);
    Debug.STATUS("vuOffsetInt: %i\n", BL_vuOffsetInt);
    Debug.STATUS("vuBaselineInt: %i\n", BL_vuBaselineInt);
    Debug.STATUS("vuOffsetExt: %i\n", BL_vuOffsetExt);
    Debug.STATUS("vuBaselineExt: %i\n", BL_vuBaselineExt);
    Debug.STATUS("BL_BatteryVoltage: %f\n", BL_BatteryVoltage);
    Debug.STATUS("BL_BatteryPercentage: %i\n", BL_BatteryPercentage);
    Debug.STATUS(("Fucntion Sw State: %i\n"), FunctionSwState);
    Debug.STATUS("Remote Connected Status: %i\n", remoteConnected);
  }
  Internal_Command[0] = '\0';
}

//////////////////////////////////////////////////////////////////////
//  Animation functions  (same as base BC, plus new cases 26-40)
//////////////////////////////////////////////////////////////////////
void normalOperations() { writeBlSerial("A99"); sendESPNOWCommand("BS",":D304"); HCR.Stop(); Animation_Command[0]='\0'; }
void panelWave()        { sendESPNOWCommand("BS",":D310D03125"); Animation_Command[0]='\0'; }
void panelWaveFast()    { sendESPNOWCommand("BS",":D311D01000"); Animation_Command[0]='\0'; }
void domePeriscope()    { sendESPNOWCommand("DP",":SUS:PS4"); writeRdSerial(":DPS4"); Animation_Command[0]='\0'; }
void allOpenClose()     { sendESPNOWCommand("BS",":D306"); Animation_Command[0]='\0'; }
void HarlemShake()      { sendESPNOWCommand("BS",":D313"); HCR.PlayWAV(1,202); HCR.update(); Animation_Command[0]='\0'; }
void allClose()         { sendESPNOWCommand("BS",":D304"); writeBlSerial("J98"); Animation_Command[0]='\0'; }
void allFlutter()       { sendESPNOWCommand("DC",":R0112"); sendESPNOWCommand("BS",":D308"); DelayCall::schedule([]{sendESPNOWCommand("DC",":R0155");},1000); Animation_Command[0]='\0'; }
bool doorsOpen = false;
void toggleDoors() {
  if (doorsOpen) { sendESPNOWCommand("BS",":D304"); writeBlSerial("J98"); doorsOpen=false; }
  else           { sendESPNOWCommand("BS",":D303"); writeBlSerial("J99"); doorsOpen=true;  }
  Animation_Command[0]='\0';
}
bool lightsOn = true;
void allLightsToggle() {
  if (lightsOn) { writeBlSerial("E98"); lightsOn=false; }
  else          { writeBlSerial("E99"); lightsOn=true;  }
  Animation_Command[0]='\0';
}
void allOpen()          { sendESPNOWCommand("BS",":D303"); writeBlSerial("J99"); Animation_Command[0]='\0'; }
void drawerWave()       { sendESPNOWCommand("BS",":D118"); Animation_Command[0]='\0'; }
void OpenClosewithEasing(){ sendESPNOWCommand("BS",":D306B312000400000050"); writeBlSerial("J99"); Animation_Command[0]='\0'; }
void CompleteshortCircuit() {
  writeBlSerial("E14"); HCR.Overload();
  DelayCall::schedule([]{HCR.Stimulate(3,1);},1500);
  sendESPNOWCommand("HP",":HA014"); writeRdSerial(":DPS14");
  DelayCall::schedule([]{sendESPNOWCommand("DC",":SPS0T4");},50);
  DelayCall::schedule([]{sendESPNOWCommand("BS",":D305");},100);
  DelayCall::schedule([]{sendESPNOWCommand("DP",":A54");},150);
  DelayCall::schedule([]{sendESPNOWCommand("DC",":SDL@APLE20530");},200);
  DelayCall::schedule([]{sendESPNOWCommand("DP",":SUS:PS14");},250);
  DelayCall::schedule([]{writeBlSerial("K99"); sendESPNOWCommand("HP",":HA0025");},18000);
  Animation_Command[0]='\0';
}
void StarWarsThemeSong(){ HCR.PlayWAV(1,1);    Animation_Command[0]='\0'; }
void VaderThemeSong()   { HCR.PlayWAV(1,2);    Animation_Command[0]='\0'; }
void PeriscopeSeq2()    { sendESPNOWCommand("DP",":SUS:PS2");  Animation_Command[0]='\0'; }
void PerisopeSeq10()    { sendESPNOWCommand("DP",":SUS:PS10"); Animation_Command[0]='\0'; }
void LightsOff()        { writeBlSerial("K98"); Animation_Command[0]='\0'; }
void LightsOn()         { writeBlSerial("K99"); Animation_Command[0]='\0'; }
void FartNoise()        { HCR.PlayWAV(1,4);    Animation_Command[0]='\0'; }
void WaveUtilityArm()   { sendESPNOWCommand("BS",":D119"); Animation_Command[0]='\0'; }
void ArmSaber()         { sendESPNOWCommand("DP",":A61");  Animation_Command[0]='\0'; }
void LaunchSaber()      { sendESPNOWCommand("DP",":A60");  Animation_Command[0]='\0'; }
void StowSaber()        { sendESPNOWCommand("BS",":D10104"); DelayCall::schedule([]{sendESPNOWCommand("BS",":D10204");},7050); Animation_Command[0]='\0'; }
void SmokeSequence()    { sendESPNOWCommand("DP",":A54");   Animation_Command[0]='\0'; }
void display()          { sendESPNOWCommand("DP",":SUS:PP100:L0"); sendESPNOWCommand("BS",":D320"); Animation_Command[0]='\0'; }
void openCloseWave()    { sendESPNOWCommand("BS",":D312"); Animation_Command[0]='\0'; }
void longDance()        { sendESPNOWCommand("BS",":D315"); Animation_Command[0]='\0'; }
void cpuSequence()      { sendESPNOWCommand("BS",":D126"); Animation_Command[0]='\0'; }
void cpuRaise()         { sendESPNOWCommand("BS",":D10103"); DelayCall::schedule([]{sendESPNOWCommand("BS",":D121");},500); Animation_Command[0]='\0'; }
void cpuLower()         { sendESPNOWCommand("BS",":D122");  DelayCall::schedule([]{sendESPNOWCommand("BS",":D10203");},1000); Animation_Command[0]='\0'; }
void cpuExtend()        { sendESPNOWCommand("BS",":D123"); Animation_Command[0]='\0'; }
void cpuRetract()       { sendESPNOWCommand("BS",":D124"); Animation_Command[0]='\0'; }
void cpuRotate()        { sendESPNOWCommand("BS",":D125"); Animation_Command[0]='\0'; }
void fireExinguisher()  { sendESPNOWCommand("DP",":A70");  Animation_Command[0]='\0'; }
void stopSounds()       { HCR.StopWAV(1); HCR.StopEmote(); Animation_Command[0]='\0'; }
void stayingAliveSong() { HCR.PlayWAV(1,201); Animation_Command[0]='\0'; }
void stayingAliveFull() { HCR.PlayWAV(1,201); sendESPNOWCommand("BS",":D2327"); Animation_Command[0]='\0'; }
void periscopeHome()    { sendESPNOWCommand("DP",":SUS:PH"); Animation_Command[0]='\0'; }
void stayingAliveServo() {
  HCR.PlayWAV(1, 201);                        // start song
  sendESPNOWCommand("BS", ":D2327");          // tell body servo to run the whole 20s sequence
  Animation_Command[0] = '\0';
}
// callAnimation — used by the config dispatch (RA_ANIM) and :A## commands
// ─────────────────────────────────────────────────────────────────────────────
//  Animation table — labels exposed to the GUI via GET_ANIM_LIST.
//  Keep this in sync with the switch cases in callAnimation() below — entries
//  not present here will simply not appear in the GUI dropdown, and any case
//  in callAnimation() without a matching entry here is effectively hidden.
// ─────────────────────────────────────────────────────────────────────────────
struct AnimEntry { uint8_t fn; const char* label; };
static const AnimEntry ANIM_TABLE[] = {
  {  1, "Normal Operations"      },
  {  2, "Panel Wave"             },
  {  3, "Panel Wave Fast"        },
  {  4, "Dome Periscope"         },
  {  5, "All Open/Close"         },
  {  6, "Harlem Shake"           },
  {  7, "All Close"              },
  {  8, "All Flutter"            },
  {  9, "Toggle Doors"           },
  { 10, "All Lights Toggle"      },
  { 11, "All Open"               },
  { 12, "Drawer Wave"            },
  { 13, "Open/Close Easing"      },
  { 14, "Short Circuit"          },
  { 15, "Star Wars Theme"        },
  { 16, "Vader Theme"            },
  { 17, "Periscope Seq 2"        },
  { 18, "Periscope Seq 10"       },
  { 19, "Lights Off"             },
  { 20, "Lights On"              },
  { 21, "Fart Noise"             },
  { 22, "Wave Utility Arm"       },
  { 23, "Arm Saber"              },
  { 24, "Launch Saber"           },
  { 25, "Smoke Sequence"         },
  { 26, "Display Mode"           },
  { 27, "Open/Close Wave"        },
  { 28, "CPU Raise"              },
  { 29, "CPU Lower"              },
  { 30, "CPU Extend"             },
  { 31, "CPU Retract"            },
  { 32, "CPU Rotate"             },
  { 33, "Long Dance"             },
  { 34, "CPU Sequence"           },
  { 35, "Fire Extinguisher"      },
  { 36, "Stow Saber"             },
  { 37, "Stop Sounds"            },
  { 38, "Staying Alive (song)"   },
  { 39, "Staying Alive (Servo)"  },
  { 40, "Periscope Home"         },
};
static const size_t ANIM_TABLE_COUNT = sizeof(ANIM_TABLE) / sizeof(ANIM_TABLE[0]);

void callAnimation(int fn) {
  Animation_Command[0] = '\0';  // ensure flag is clear
  switch (fn) {
    case 1:  normalOperations();     break;
    case 2:  panelWave();            break;
    case 3:  panelWaveFast();        break;
    case 4:  domePeriscope();        break;
    case 5:  allOpenClose();         break;
    case 6:  HarlemShake();          break;
    case 7:  allClose();             break;
    case 8:  allFlutter();           break;
    case 9:  toggleDoors();          break;
    case 10: allLightsToggle();      break;
    case 11: allOpen();              break;
    case 12: drawerWave();           break;
    case 13: OpenClosewithEasing();  break;
    case 14: CompleteshortCircuit(); break;
    case 15: StarWarsThemeSong();    break;
    case 16: VaderThemeSong();       break;
    case 17: PeriscopeSeq2();        break;
    case 18: PerisopeSeq10();        break;
    case 19: LightsOff();            break;
    case 20: LightsOn();             break;
    case 21: FartNoise();            break;
    case 22: WaveUtilityArm();       break;
    case 23: ArmSaber();             break;
    case 24: LaunchSaber();          break;
    case 25: SmokeSequence();        break;
    // Extended cases (new in GUI version)
    case 26: display();              break;
    case 27: openCloseWave();        break;
    case 28: cpuRaise();             break;
    case 29: cpuLower();             break;
    case 30: cpuExtend();            break;
    case 31: cpuRetract();           break;
    case 32: cpuRotate();            break;
    case 33: longDance();            break;
    case 34: cpuSequence();          break;
    case 35: fireExinguisher();      break;
    case 36: StowSaber();            break;
    case 37: stopSounds();           break;
    case 38: stayingAliveSong();     break;
    case 39: stayingAliveFull();     break;
    case 40: periscopeHome();        break;
    default: break;
  }
}

//////////////////////////////////////////////////////////////////////
//  RC Config dispatch — execute a single RcAction
//  Actions with delayMs > 0 are queued in pendingActions[]
//////////////////////////////////////////////////////////////////////
static void scheduleAction(const RcAction& action, unsigned long delayMs) {
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (!pendingActions[i].active) {
      pendingActions[i].active  = true;
      pendingActions[i].fireAt  = millis() + delayMs;
      pendingActions[i].action  = action;
      return;
    }
  }
  // No free slot — execute immediately as fallback
  Serial.println("WARN: pendingActions full, executing immediately");
}

void rcExecuteAction(const RcAction& a) {
  // Calibration mode: the operator is intentionally moving every control to
  // set thresholds — drop every action (including delayed/scheduled ones, so
  // nothing queues up to fire the instant calibration ends).
  if (calibrationActive) return;
  if (a.delayMs > 0) {
    scheduleAction(a, a.delayMs);
    return;
  }
  switch (a.type) {
    case RA_ESPNOW:
#ifdef BENCH_TEST
      Serial.printf("[DISPATCH] ESPNOW  target=%s  cmd=%s\n", a.target, a.cmd);
#endif
      sendESPNOWCommand(a.target, a.cmd);
      break;
    case RA_HCR:
#ifdef BENCH_TEST
      Serial.printf("[DISPATCH] HCR     fn=%d  chan=%d  track=%d\n",
                    a.hcrFn, a.hcrChan, a.hcrTrack);
#endif
      HCRFunction(a.hcrFn, a.hcrChan, a.hcrTrack);
      break;
    case RA_ANIM:
#ifdef BENCH_TEST
      Serial.printf("[DISPATCH] ANIM    fn=%d\n", a.animFn);
#endif
      callAnimation(a.animFn);
      break;
    case RA_SERIAL: {
#ifdef BENCH_TEST
      Serial.printf("[DISPATCH] SERIAL  port=%s  cmd=%s\n", a.target, a.cmd);
#endif
      // Dispatch the command string to the named local serial port.
      // Keep this in sync with RA_SERIAL_PORTS_LIST in rc_config.h.
      String s(a.cmd);
      if      (!strcmp(a.target, "BL")) writeBlSerial(s);
      else if (!strcmp(a.target, "RD")) writeRdSerial(s);
      else if (!strcmp(a.target, "MP")) writeMpSerial(s);
      else if (!strcmp(a.target, "ST")) writeStSerial(s);
      else if (!strcmp(a.target, "S1")) writeS1Serial(s);
      else if (!strcmp(a.target, "S2")) writeS2Serial(s);
      // Unknown ports are silently dropped — GUI restricts the dropdown so this
      // shouldn't happen unless config was hand-edited.
      break;
    }
    default: break;
  }
}

void rcDispatch(int buttonId, uint8_t tapCount) {
  int mode = buttonId / 100;
  int btn  = buttonId % 100;
  if (mode < 1 || mode > 3 || btn < 1 || btn > 19) return;
  if (tapCount < 1 || tapCount > 3) return;
  RcMapping& m = rcConfig.mappings[rcMapIndex(mode, btn)];
  RcTier&    tier = m.t[tapCount - 1];
  for (int i = 0; i < tier.count; i++) {
    rcExecuteAction(tier.a[i]);
  }
}

void checkPendingActions() {
  unsigned long now = millis();
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (pendingActions[i].active && now >= pendingActions[i].fireAt) {
      pendingActions[i].active = false;
      rcExecuteAction(pendingActions[i].action);
    }
  }
}

//////////////////////////////////////////////////////////////////////
//  SBUS / RC
//////////////////////////////////////////////////////////////////////
#ifdef SBUS
// ─────────────────────────────────────────────────────────────────────────────
// Channel layout (Kyber-aligned, post-refactor):
//   Matrix-encoded buttons live on rcConfig.matrixChannel (default CH7).
//   Mode/function selector + RADH/Maint/Lights/Muse all read the SBUS value
//   of whichever switch they're bound to via rcConfig.funcBindings.  This
//   means moving switches around in the model just changes the channel
//   field — function bindings stay correct regardless.
// ─────────────────────────────────────────────────────────────────────────────

int newValue = 0;
// Last-seen values per logical role for change detection.
int oldValueMatrix   = 100;  // raw matrix SBUS value — kept fresh for status/diagnostics
int oldValueMode     = 100;
bool lostFrameOld    = false;
int  sbusValues[24]  = {0};  // Sized for SBUS-24 max; SBUS-16 leaves CH17-24 at 0.

// ── Matrix-button edge state machine ─────────────────────────────────────────
// Replaces the old raw-value-delta gate (abs(mxVal-oldValueMatrix) >= 5) that
// silently dropped fast re-presses of the same button (the value was identical
// across the two sampled SBUS frames so no "edge" was seen — see analysis).
//
// Three-state debounced edge machine (NEUTRAL · TRANSITION · BUTTON):
//   • A press is COMMITTED only after the decoded button is stable in-band for
//     rcConfig.matrixDebounceFrames consecutive frames — rejects single-frame
//     noise and analog resistor-ladder sweep transients.
//   • A release/re-arm is COMMITTED only after NEUTRAL (decoded == 0) is stable
//     for matrixDebounceFrames consecutive frames — so a brief 1-frame neutral
//     dip mid-press (sweep slew, contact bounce) can no longer falsely re-arm
//     and split one press into a phantom double.
//   • Anything not yet stable (short neutral blips, unsettled band) is the
//     implicit TRANSITION state: it changes nothing — no fire, no re-arm.
// matrixDebounceFrames is runtime-configurable (Config modal): 1 = fastest
// (clean digital SBUS source), 2-4 for a noisy analog transmitter matrix.
// Only a true sub-frame tap (press+release inside one ~7-14 ms frame interval)
// is unrecoverable, and that's a hard SBUS-protocol limit, not logic.
bool matrixArmed       = true; // true → a confirmed-neutral release has armed the next press
int  matrixCandidate   = 0;    // decoded button currently being debounced
int  matrixCandCount   = 0;    // consecutive frames matrixCandidate has held in-band
int  matrixNeutralCount = 0;   // consecutive NEUTRAL (decoded==0) frames — for release debounce

// Read the live SBUS value of the switch at the given switch-table index
// (0-7 = SA-SH).  Returns -1 if the index is out of range or its channel
// is unassigned.
static inline int readBoundSwitchSbus(int8_t swIdx) {
  if (swIdx < 0 || swIdx >= RC_NUM_SWITCHES) return -1;
  int ch = rcConfig.switches[swIdx].channel;
  if (ch < 1 || ch > 24) return -1;
  return sbusValues[ch - 1];
}

// SBUS frame stats (for GUI live status indicator)
unsigned long sbusFrameCount     = 0;
unsigned long sbusLastFrameMs    = 0;
unsigned long sbusFpsLastSecond  = 0;
unsigned long sbusFpsCounter     = 0;
int           sbusFps            = 0;
bool          sbusFailsafe       = false;
bool          sbusLiveDump       = false;   // #L10 toggles this
unsigned long sbusLiveDumpLastMs = 0;

// One-shot SBUS state dump to Serial — call from #L09 or anywhere
void dumpSbusState() {
#ifdef SBUS
  unsigned long now = millis();
  unsigned long age = (sbusLastFrameMs == 0) ? 0 : (now - sbusLastFrameMs);
  Serial.println("---- SBUS STATE ----");
  Serial.printf("  variant=%s (%d ch, %d-byte frame)\n",
                sbus_rx.detectedChCount == 24 ? "SBUS-24" :
                sbus_rx.detectedChCount == 16 ? "SBUS-16" : "(none yet)",
                sbus_rx.detectedChCount, sbus_rx.detectedFrameLen);
  Serial.printf("  frames=%lu  fps=%d  ageMs=%lu  lost=%s  failsafe=%s\n",
                sbusFrameCount, sbusFps, age,
                lostFrameOld ? "YES" : "no",
                sbusFailsafe ? "YES" : "no");
  Serial.printf("  CH1-8:  ");
  for (int i = 0; i < 8; i++) Serial.printf("%4d ", sbusValues[i]);
  Serial.println();
  Serial.printf("  CH9-16: ");
  for (int i = 8; i < 16; i++) Serial.printf("%4d ", sbusValues[i]);
  Serial.println();
  // Show CH17-24 only when SBUS-24 was detected (otherwise those slots are zero).
  if (sbus_rx.detectedChCount >= 24) {
    Serial.printf("  CH17-24:");
    for (int i = 16; i < 24; i++) Serial.printf("%4d ", sbusValues[i]);
    Serial.println();
  }
  Serial.printf("  btn(matrixCh=%d)=%d  funcSwState(mode<-%s)=%d\n",
                rcConfig.matrixChannel, pwmToButton(oldValueMatrix),
                (rcConfig.funcBindings.modeSwitch >= 0 && rcConfig.funcBindings.modeSwitch < RC_NUM_SWITCHES)
                  ? RC_SWITCH_LABELS[rcConfig.funcBindings.modeSwitch] : "?",
                FunctionSwState);
  Serial.println("--------------------");
#else
  Serial.println("SBUS not compiled in.");
#endif
}

// Tap detection (uses rcConfig.tapWindowMs — not the #define)
int           rcTapLastButtonId  = -1;
unsigned long rcTapLastPressTime = 0;
uint8_t       rcTapCount         = 0;
int           deferButtonId      = -1;
uint8_t       deferTapCount      = 0;
bool          deferPending       = false;
unsigned long deferFireTime      = 0;

// Convert raw SBUS PWM to button index 1-19, or 0 = neutral
// Uses configurable thresholds from rcConfig
int pwmToButton(int pwm) {
  for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
    if (pwm >= rcConfig.thresholds[i].minPwm &&
        pwm <= rcConfig.thresholds[i].maxPwm) {
      return rcConfig.thresholds[i].id;
    }
  }
  return 0;
}

uint8_t detectRCTap(int buttonId) {
  unsigned long now = millis();
  if (buttonId == rcTapLastButtonId &&
      (now - rcTapLastPressTime) < (unsigned long)rcConfig.tapWindowMs) {
    rcTapCount++;
  } else {
    rcTapCount = 1;
  }
  rcTapLastButtonId  = buttonId;
  rcTapLastPressTime = now;
  return rcTapCount;
}

// Forward declaration
void dispatchDeferredButton(int buttonId, uint8_t taps);

void queueExclusiveTap(int buttonId) {
  if (deferPending && deferButtonId != buttonId) {
    deferPending = false;
    dispatchDeferredButton(deferButtonId, deferTapCount);
  }
  deferButtonId = buttonId;
  deferTapCount = rcTapCount;
  if (!deferPending) {
    deferPending  = true;
    deferFireTime = millis() + (unsigned long)rcConfig.tapWindowMs;
  }
}

void checkDeferredTap() {
  if (!deferPending) return;
  if (millis() < deferFireTime) return;
  deferPending = false;
  dispatchDeferredButton(deferButtonId, deferTapCount);
}

void dispatchDeferredButton(int buttonId, uint8_t taps) {
  rcDispatch(buttonId, taps);
}

void RCRadio_Matrix_Buttons(int PWMvalue) {
  int btn = pwmToButton(PWMvalue);
  if (btn == 0) return;

  int switchMode = FunctionSwState;
  int buttonId   = (switchMode * 100) + btn;
  uint8_t taps   = detectRCTap(buttonId);

  int idx = rcMapIndex(switchMode, btn);
  if (idx < 0 || idx >= RC_NUM_MAPPINGS) return;
  RcMapping& m = rcConfig.mappings[idx];

  if (m.exclusive) {
    queueExclusiveTap(buttonId);
  } else {
    rcDispatch(buttonId, taps);
  }
}

void RCRadio_HCRVolChange(int chan, int sbusPos) {
  int vol = map(sbusValues[sbusPos], 170, 1820, 0, 100);
  HCR.SetVolume(chan, vol);
}

// NOTE: The old hardcoded helpers — updateMuse(), lightsMode(), maintLights(),
// RC_RADH_Auto() — have been REMOVED.  Their behaviors are now configurable
// per-switch-position via RA_SERIAL and RA_HCR actions.  Defaults in
// rcConfigLoadDefaults() pre-populate switches SB/SC/SD/SF with exactly the
// same commands they used to send, so out-of-box behavior is unchanged.

// Per-switch last-known position cache (-1 = unread).
// Index matches rcConfig.switches[] (SA, SB, SC, SD, SE, SF, SG, SH).
int8_t rcSwitchLastPos[RC_NUM_SWITCHES] = {-1,-1,-1,-1,-1,-1,-1,-1};

// Per-knob last-known raw value (used to debounce — only act on changes >=5).
int rcKnobLastVal[RC_NUM_KNOBS] = {100, 100};

// Reads each configured knob channel, applies the assigned function on change.
void processKnobs() {
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    RcKnob& k = rcConfig.knobs[i];
    if (k.channel < 1 || k.channel > 24) continue;
    if (k.function == KF_NONE) continue;
    int val = sbusValues[k.channel - 1];
    if (abs(val - rcKnobLastVal[i]) < 5) continue;
    rcKnobLastVal[i] = val;
    switch (k.function) {
      case KF_HCR_VOC_VOL:
        RCRadio_HCRVolChange(0, k.channel - 1);
        break;
      case KF_HCR_WAV_VOL:
        RCRadio_HCRVolChange(1, k.channel - 1);
        break;
      default: break;
    }
  }
}

// Reads each configured switch channel, computes position (0/1/2),
// and runs the matching RcTier when the position changes.
void processSwitches() {
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    RcSwitch& sw = rcConfig.switches[i];
    if (sw.channel < 1 || sw.channel > 24) continue;
    int val = sbusValues[sw.channel - 1];
    int pos;
    if (sw.positions == 2) {
      pos = (val < 1000) ? 0 : 2;
    } else {
      pos = (val < 400) ? 0 : (val > 1500) ? 2 : 1;
    }
    if (pos != rcSwitchLastPos[i]) {
      int8_t prev = rcSwitchLastPos[i];
      rcSwitchLastPos[i] = pos;
      if (prev < 0) continue;   // skip first read (avoid firing on boot)
      Serial.printf("[SWITCH] %s  ch%d=%d  pos=%d\n",
                    RC_SWITCH_LABELS[i], sw.channel, val, pos);
      RcTier& tier = sw.t[pos];
      for (int j = 0; j < tier.count; j++) rcExecuteAction(tier.a[j]);
    }
  }
}

void processSbus() {
  // Update fps counter once per second (runs even on dropped frames)
  unsigned long nowMs = millis();
  if (nowMs - sbusFpsLastSecond >= 1000) {
    sbusFps           = sbusFpsCounter;
    sbusFpsCounter    = 0;
    sbusFpsLastSecond = nowMs;
  }

  if (sbus_rx.read()) {
    // Copy however many channels the detected frame contains (16 or 24).
    int chCount = sbus_rx.detectedChCount;
    if (chCount > 24) chCount = 24;
    for (int i = 0; i < chCount; i++) sbusValues[i] = sbus_rx.channels[i];

    // SBUS stats
    sbusFrameCount++;
    sbusFpsCounter++;
    sbusLastFrameMs = nowMs;
    sbusFailsafe    = sbus_rx.failsafe;

    if (lostFrameOld != sbus_rx.lostFrame) {
      lostFrameOld    = sbus_rx.lostFrame;
      remoteConnected = !sbus_rx.lostFrame;
    }

    // Mode/function selector — read from whichever switch is bound to it
    int modeVal = readBoundSwitchSbus(rcConfig.funcBindings.modeSwitch);
    if (modeVal >= 0 && abs(modeVal - oldValueMode) > 5) {
      oldValueMode = modeVal;
      if      (modeVal < 200)                                FunctionSwState = 1;
      else if (modeVal > 700 && modeVal <= 1200)             FunctionSwState = 2;
      else if (modeVal > 1700)                               FunctionSwState = 3;
      sendESPNOWCommand("DG","");
    }

    // Volume knobs S1 / S2 — channel + function configurable in the GUI.
    processKnobs();

    // Button matrix on the configurable matrix channel — edge-detected with
    // an asymmetric debounce (see matrixArmed/Candidate state above). Decodes
    // EVERY frame instead of gating on raw value delta, so a fast re-press of
    // the same button is no longer dropped.
    int mxCh = rcConfig.matrixChannel;
    if (mxCh >= 1 && mxCh <= 24) {
      int mxVal = sbusValues[mxCh - 1];
      oldValueMatrix = mxVal;                 // keep fresh for status/diagnostics
      int decoded = pwmToButton(mxVal);       // 0 = neutral / between-band gap
      int debFrames = rcConfig.matrixDebounceFrames;
      if (debFrames < 1) debFrames = 1;          // safety clamp (config is 1-4)

      if (decoded == 0) {
        // NEUTRAL candidate. A real release sits at rest for many frames; a
        // sweep slew / contact bounce only dips out-of-band for 1-2 frames.
        // Only a *debounced* neutral run counts as a release → re-arm. This is
        // the key reliability fix: a transient neutral can no longer split one
        // physical press into a phantom double.
        matrixCandidate = 0;
        matrixCandCount = 0;
        if (matrixNeutralCount < debFrames) matrixNeutralCount++;
        if (matrixNeutralCount >= debFrames) matrixArmed = true;   // release confirmed
      } else {
        // BUTTON candidate. Any in-band reading breaks a neutral run, so a
        // mid-press sweep transient that briefly crosses a neighbor band does
        // not accumulate toward a release.
        matrixNeutralCount = 0;
        if (decoded == matrixCandidate) {
          if (matrixCandCount < debFrames) matrixCandCount++;
        } else {
          matrixCandidate = decoded;
          matrixCandCount = 1;
        }
        // Fire once the button has been stable in-band AND we're armed
        // (armed only by a confirmed neutral — never by a 1-frame blip).
        if (matrixArmed && matrixCandCount >= debFrames) {
          matrixArmed = false;             // consume — needs a CONFIRMED neutral to re-arm
          RCRadio_Matrix_Buttons(mxVal);
        }
      }
    }

    // All switch behavior is now driven entirely by per-position actions
    // configured in the GUI (RA_ESPNOW / RA_HCR / RA_ANIM / RA_SERIAL).
    // No more hardcoded RADH/Maint/Lights/Muse handlers — those behaviors
    // are now Serial-command actions on the corresponding switch positions
    // (set up in rcConfigLoadDefaults() to preserve original out-of-box behavior).
    processSwitches();
  }
}
#endif  // SBUS

//////////////////////////////////////////////////////////////////////
//  WebSerial protocol handler
//  Called from serialEvent() when a line starts with '{'
//////////////////////////////////////////////////////////////////////
void handleWebSerialMessage(const String& line) {
  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, line);
  if (err != DeserializationError::Ok) {
    Serial.println("{\"type\":\"ERROR\",\"msg\":\"JSON parse failed\"}");
    return;
  }

  const char* type = doc["type"] | "";

  // ── PING ──────────────────────────────────────────────────────────────────
  if (strcmp(type, "PING") == 0 || strcmp(type, "ping") == 0) {
    // A fresh page connect pings first — clear any stale calibration mute
    // left behind by a previously crashed/closed calibration page.
    calibrationActive = false;
    Serial.println("{\"type\":\"PONG\"}");
    return;
  }

  // ── GET_CONFIG ─────────────────────────────────────────────────────────────
  if (strcmp(type, "GET_CONFIG") == 0) {
    String cfg = rcConfigToJSON();
    // Send as a response envelope — print to USB for direct-cable debug.
    Serial.print("{\"type\":\"CONFIG\",\"data\":");
    Serial.print(cfg);
    Serial.println("}");
    // 2026-05 GET_CONFIG return path: stage the chunked send over the
    // BC → DG → Remote → USB → Web pipeline. The main loop dribbles
    // out one chunk per BC_CFG_CHUNK_INTERVAL_MS until done. We pack
    // the OUTER envelope so the Web side sees the same shape as a
    // direct-USB CONFIG reply, just split across chunks.
    pendingCfgJson  = String("{\"type\":\"CONFIG\",\"data\":") + cfg + String("}");
    uint32_t jsonLen = pendingCfgJson.length();
    pendingCfgTotal = (uint16_t)((jsonLen + BC_CFG_CHUNK_BYTES - 1) / BC_CFG_CHUNK_BYTES);
    pendingCfgIdx   = 0;
    // seq: never zero (BC uses 0 as "no active chunk"); wrap millis().
    uint16_t seq = (uint16_t)(millis() & 0xFFFF);
    if (seq == 0) seq = 1;
    pendingCfgSeq = seq;
    // Fire first chunk immediately on next loop tick by backdating.
    pendingCfgLastMs = millis() - BC_CFG_CHUNK_INTERVAL_MS;
    // 2026-05 unconditional print so we can see chunk staging without
    // enabling LOOP debug. Reports the seq, total chunks, and total
    // bytes about to be chunked out.
    Serial.printf("[CFG] STAGED seq=%u total=%u jsonLen=%u\n",
                  pendingCfgSeq, pendingCfgTotal, (unsigned)jsonLen);
    return;
  }

  // ── SET_CONFIG ─────────────────────────────────────────────────────────────
  if (strcmp(type, "SET_CONFIG") == 0) {
    // 2026-05 fix: was DynamicJsonDocument(98304) = 96 KB on the heap per
    // push. ESP32 typical free heap during runtime is ~150-180 KB (after
    // Wi-Fi/ESP-NOW/LoRa stacks initialise), so allocating 96 KB in one
    // shot fragmented the heap badly and risked OOM on repeated pushes.
    // Worst-case config payload is ~25 KB; 32 KB capacity plus ArduinoJson's
    // ~1.5× overhead headroom is plenty. NoMemory now returns an explicit
    // ACK so the failure is visible instead of silent.
    DynamicJsonDocument bigDoc(32768);
    DeserializationError err = deserializeJson(bigDoc, line);
    if (err != DeserializationError::Ok) {
      char buf[24];
      snprintf(buf, sizeof(buf), "parse_%s", err.c_str());
      sendBcConfigAck(lastChunkSeqForAck, false, buf);
      return;
    }
    if (!bigDoc.containsKey("data")) {
      sendBcConfigAck(lastChunkSeqForAck, false, "missing_data");
      return;
    }
    JsonObject data = bigDoc["data"].as<JsonObject>();
    bool ok = rcConfigFromJSON(data);
    if (ok) {
      rcConfigSaveNVS();
      sendBcConfigAck(lastChunkSeqForAck, true, "applied");
    } else {
      sendBcConfigAck(lastChunkSeqForAck, false, "apply_failed");
    }
    return;
  }

  // ── START_MONITOR ──────────────────────────────────────────────────────────
  if (strcmp(type, "START_MONITOR") == 0) {
    wsMonitorActive   = true;
    wsMonitorLastSent = 0;
    Serial.println("{\"type\":\"ACK\",\"ok\":true}");
    return;
  }

  // ── STOP_MONITOR ──────────────────────────────────────────────────────────
  if (strcmp(type, "STOP_MONITOR") == 0) {
    wsMonitorActive = false;
    // Calibration always runs with the monitor on; stopping it is a reliable
    // "calibration is over" signal even if CALIB-off is lost.
    calibrationActive = false;
    Serial.println("{\"type\":\"ACK\",\"ok\":true}");
    return;
  }

  // ── CALIB — mute all action dispatch while the calibration wizard runs ─────
  if (strcmp(type, "CALIB") == 0) {
    calibrationActive = doc["on"] | false;
    Serial.printf("[CALIB] action dispatch %s\n",
                  calibrationActive ? "SUPPRESSED (calibrating)" : "resumed");
    Serial.println("{\"type\":\"ACK\",\"ok\":true}");
    return;
  }

  // ── RESET_DEFAULTS ─────────────────────────────────────────────────────────
  if (strcmp(type, "RESET_DEFAULTS") == 0) {
    // 2026-05 fix: was loading defaults into RAM only — NVS still held the
    // previous config, so a reboot brought back the old config. The GUI's
    // confirm prompt explicitly says "this will overwrite the BC's NVS",
    // so save() must persist the reset.
    rcConfigLoadDefaults();
    rcConfigSaveNVS();
    sendBcConfigAck(0, true, "defaults_reset");
    return;
  }

  // ── GET_ANIM_LIST — returns the animation table for GUI dropdowns ─────────
  if (strcmp(type, "GET_ANIM_LIST") == 0) {
    Serial.print("{\"type\":\"ANIM_LIST\",\"data\":[");
    for (size_t i = 0; i < ANIM_TABLE_COUNT; i++) {
      if (i > 0) Serial.print(",");
      Serial.printf("{\"fn\":%d,\"label\":\"%s\"}", ANIM_TABLE[i].fn, ANIM_TABLE[i].label);
    }
    Serial.println("]}");
    return;
  }

  // ── TRIGGER ────────────────────────────────────────────────────────────────
  //   {"type":"TRIGGER","mode":1,"btn":3,"tap":1}
  //   Fires a virtual button-press through rcDispatch so the GUI can test
  //   the dispatch path without a real SBUS receiver.  In BENCH_TEST builds
  //   rcExecuteAction only prints [DISPATCH] lines.
  if (strcmp(type, "TRIGGER") == 0) {
    int mode = doc["mode"] | 1;
    int btn  = doc["btn"]  | 0;
    int tap  = doc["tap"]  | 1;
    if (btn < 1 || btn > 19 || mode < 1 || mode > 3 || tap < 1 || tap > 3) {
      Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"bad mode/btn/tap\"}");
      return;
    }
    Serial.printf("[TRIGGER]  mode=%d  btn=%d  tap=%d\n", mode, btn, tap);
    rcDispatch(mode * 100 + btn, (uint8_t)tap);
    Serial.println("{\"type\":\"ACK\",\"ok\":true}");
    return;
  }

  Serial.println("{\"type\":\"ERROR\",\"msg\":\"unknown type\"}");
}

// PWM monitor streaming  (called from loop when wsMonitorActive)
void sendPWMUpdate() {
#ifdef SBUS
  unsigned long now = millis();
  if (now - wsMonitorLastSent < WS_MONITOR_INTERVAL_MS) return;
  wsMonitorLastSent = now;
  unsigned long ageMs = (sbusLastFrameMs == 0) ? 99999 : (now - sbusLastFrameMs);
  bool sbusOk = (sbusFps > 0) && !lostFrameOld && (ageMs < 500);
  // Send live channel values keyed by ROLE rather than fixed channel number
  // so the GUI labels stay correct even when matrixChannel/modeChannel change.
  // Includes detected SBUS channel count (16 or 24) so the GUI can show variant.
  char channelBuf[256] = "";
  int chCount = sbus_rx.detectedChCount;
  if (chCount > 24) chCount = 24;
  for (int i = 0; i < chCount; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d%s", sbusValues[i], (i + 1 < chCount) ? "," : "");
    strncat(channelBuf, tmp, sizeof(channelBuf) - strlen(channelBuf) - 1);
  }

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"PWM_UPDATE\",\"matrixCh\":%d,\"modeCh\":%d,\"matrixVal\":%d,\"modeVal\":%d,\"btn\":%d,\"mode\":%d,"
    "\"sbus\":{\"ok\":%s,\"fps\":%d,\"frames\":%lu,\"ageMs\":%lu,\"lost\":%s,\"failsafe\":%s,\"chCount\":%d,\"frameLen\":%d,\"channels\":[%s]}}",
    rcConfig.matrixChannel,
    (rcConfig.funcBindings.modeSwitch >= 0 && rcConfig.funcBindings.modeSwitch < RC_NUM_SWITCHES)
      ? rcConfig.switches[rcConfig.funcBindings.modeSwitch].channel : 0,
    oldValueMatrix,
    oldValueMode,
    pwmToButton(oldValueMatrix),
    FunctionSwState,
    sbusOk ? "true" : "false",
    sbusFps,
    sbusFrameCount,
    ageMs,
    lostFrameOld ? "true" : "false",
    sbusFailsafe ? "true" : "false",
    sbus_rx.detectedChCount,
    sbus_rx.detectedFrameLen,
    channelBuf);
  Serial.println(buf);
#endif
}

//////////////////////////////////////////////////////////////////////
//  WiFi / OTA
//////////////////////////////////////////////////////////////////////
void connectWiFi() {
  esp_now_deinit(); WiFi.disconnect(); WiFi.mode(WIFI_OFF); delay(500);
  Serial.println(WiFi.config(local_IP, gateway, subnet) ? "Client IP OK" : "IP Failed");
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &oldLocalMACAddress[0]);
  delay(500);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(1000); Serial.println("Connecting WiFi..."); }
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200,"text/plain","OTA: goto /update"); });
  ElegantOTA.begin(&server); server.begin();
}

//////////////////////////////////////////////////////////////////////
//  SETUP
//////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  delay(2000);                              // give serial monitor time to attach
  Serial.println("\n[S0] Serial up");
  // 2026-05 diagnostic: print struct sizes at boot. The BC's periodic
  // heartbeat is sent as a `bodyControllerStatus_struct_message` to the
  // Gateway; the Gateway dispatches by len. If BC's size != Gateway's
  // size, the Gateway silently ignores the heartbeat and marks BC offline.
  // Compare this number to the matching print in Droid_Gateway.ino — they
  // MUST be identical for ESP-NOW to deliver heartbeats.
  Serial.printf("[S0] sizeof(bodyControllerStatus_struct_message) = %u bytes\n",
                (unsigned)sizeof(bodyControllerStatus_struct_message));
  Serial.flush();

  // 2026-06: blSerial = hardware UART1 (reliable), remapped to the ATMEGA pins.
  // rdSerial (RADH) demoted to SoftwareSerial to free UART1.
  // 2026-06: blSerial = hardware UART1 RX-ONLY (GPIO16) for the LED board's status
  // JSON. TX is bit-banged on GPIO15 (see writeBlSerial / blTxByte) because the
  // UART1 TX pin gets killed by SBUS / SoftwareSerial peripheral conflicts.
  blSerial.begin(BL_BAUD_RATE,  SERIAL_8N1,    SERIAL_RX_BL_PIN, -1 /* no UART TX */);
  pinMode(SERIAL_TX_BL_PIN, OUTPUT);
  digitalWrite(SERIAL_TX_BL_PIN, HIGH);          // UART line idles HIGH
  // 2026-06: ONLY start SoftwareSerial ports that are physically wired. An unused
  // port's RX pin floats and storms its EspSoftwareSerial edge interrupt, which
  // corrupts the hardware UART1 TX (the ATMEGA LED link). Connected here: RADH +
  // MP3/HCR. AUX1/AUX2 are not wired, so they stay unstarted.
  rdSerial.begin(RD_BAUD_RATE,  SWSERIAL_8N1,  SERIAL_RX_RD_PIN, SERIAL_TX_RD_PIN, false, 95);  // RADH
  mpSerial.begin(MP_BAUD_RATE,  SWSERIAL_8N1,  SERIAL_RX_MP_PIN, SERIAL_TX_MP_PIN, false, 95);  // MP3/HCR
#ifndef SBUS
  // stSerial (stealth) shares GPIO35 with SBUS RX — mutually exclusive. Only
  // started on non-SBUS builds (its predecessor role, before SBUS was added).
  stSerial.begin(ST_BAUD_RATE,  SWSERIAL_8N1,  SERIAL_RX_ST_PIN, SERIAL_TX_ST_PIN, false, 95);
#endif
  // s1Serial (AUX1) / s2Serial (AUX2) intentionally NOT started — nothing wired.
  // Re-enable here only when a device is actually connected to that port.
  // s1Serial.begin(SERIAL1_BAUD_RATE, SWSERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN,  false, 95);
  // s2Serial.begin(SERIAL2_BAUD_RATE, SWSERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN,  false, 95);

#ifdef SBUS
  // Custom SBUS reader handles both 16ch and 24ch frames on the same pins.
  // NOTE: SBUS's UART2 activity (and active SoftwareSerials) kill the UART1 TX
  // pin GPIO15 — which is why the LED-board TX is bit-banged on a plain GPIO
  // instead of using a UART. See writeBlSerial() / blTxByte().
  sbus_rx.begin(&Serial2, SERIAL_RX_SB_PIN, SERIAL_TX_SB_PIN);
#endif

  Serial.println("\n\n--- Body Controller GUI Edition booting ---");
#ifdef BENCH_TEST
  Serial.println("*** BENCH_TEST build — rcExecuteAction will also print [DISPATCH] lines ***");
#endif

  pinMode(RST, OUTPUT); digitalWrite(RST, HIGH);
  ESP_LED.begin(); ESP_LED.show(); colorWipeStatus("ES", red, 255);

  inputString.reserve(200); autoInputString.reserve(200);

  // Load RC config: factory defaults first, then NVS overrides
  rcConfigLoadDefaults();
  rcConfigLoadNVS();

  // Clear pending action slots
  memset(pendingActions, 0, sizeof(pendingActions));

  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, ETM_BOARD_MACS[ETM_MY_BOARD_INDEX]);

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0; peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, ETM_BROADCAST_MAC, 6);
  esp_now_add_peer(&peerInfo);
  for (int i = 0; i < ETM_NUM_BOARDS; i++) {
    if (i == ETM_MY_BOARD_INDEX) continue;
    memcpy(peerInfo.peer_addr, ETM_BOARD_MACS[i], 6);
    esp_now_add_peer(&peerInfo);
  }
  esp_now_register_recv_cb(OnDataRecv);
  etmInit(ESPNOWPASSWORD.c_str(), ETM_MY_BOARD_INDEX);

  resetArduino(500);

  Serial.println("Setup complete.");
  Serial.println("Send PING to test WebSerial. Send GET_CONFIG to read mappings.");
  Serial.println("Send TRIGGER {mode,btn,tap} to fire a virtual button press from the GUI.");
}

//////////////////////////////////////////////////////////////////////
//  MAIN LOOP
//////////////////////////////////////////////////////////////////////
// 2026-05 GET_CONFIG return path. Dribbles pendingCfgJson out as
// BC_CFG_CHUNK_BYTES-sized chunks over ESP-NOW at BC_CFG_CHUNK_INTERVAL_MS
// cadence. The Gateway forwards each chunk over LoRa as a separate
// CfgChunk_LoRa_Struct packet (disambiguated by size from the periodic
// telemetry struct). Resets state when finished.
void tickPendingCfgChunks() {
  if (pendingCfgTotal == 0) return;
  if (pendingCfgIdx >= pendingCfgTotal) {
    // Done — clear state.
    pendingCfgJson   = "";
    pendingCfgSeq    = 0;
    pendingCfgIdx    = 0;
    pendingCfgTotal  = 0;
    pendingCfgLastMs = 0;
    return;
  }
  // 2026-05 (v4): ETM handles per-chunk reliability now (sequence
  // numbers, gateway ACK, 3 retries with 500ms timeout). The BC's job
  // here is just to pace chunks at BC_CFG_CHUNK_INTERVAL_MS — no
  // in-flight tracking, no manual retry loop.
  if (millis() - pendingCfgLastMs < BC_CFG_CHUNK_INTERVAL_MS) return;

  // 2026-05 (v6): backpressure on ETM pending table. If pending is more
  // than half full, defer the next chunk to give ETM time to drain
  // ACKs/retries. Without this, when pending fills up,
  // `etmAddToPending` returns -1 and the chunk is sent but NEVER tracked
  // for retry — if it's lost over the air, it's gone forever, and the
  // GUI's reassembler aborts on the resulting idx gap.
  {
    int pendingCount = 0;
    for (int i = 0; i < ETM_PENDING_MAX; i++) {
      if (etmPendingTable[i].active) pendingCount++;
    }
    if (pendingCount >= ETM_PENDING_MAX / 2) {
      // ETM is busy — try again next tick. Don't advance pendingCfgLastMs.
      return;
    }
  }

  int start = (int)pendingCfgIdx * BC_CFG_CHUNK_BYTES;
  int end   = start + BC_CFG_CHUNK_BYTES;
  int total = (int)pendingCfgJson.length();
  if (end > total) end = total;
  String slice = pendingCfgJson.substring(start, end);
  uint8_t len = (uint8_t)slice.length();
  sendBcCfgChunk(pendingCfgSeq, pendingCfgIdx, pendingCfgTotal, len, slice.c_str());
  pendingCfgIdx++;
  pendingCfgLastMs = millis();
}

void loop() {
  etmProcess();
  keepAlive();
  checkAgeofkeepAlive();
  tickPendingCfgChunks();

#ifdef SBUS
  processSbus();
  checkDeferredTap();

  // #L10 live SBUS dump (1Hz)
  if (sbusLiveDump && (millis() - sbusLiveDumpLastMs >= 1000)) {
    sbusLiveDumpLastMs = millis();
    dumpSbusState();
  }
#endif
  checkPendingActions();

  // PWM monitor streaming
  if (wsMonitorActive) sendPWMUpdate();

  if (millis() - MLMillis >= mainLoopDelayVar) {
    MLMillis = millis();
    AnimatedEvent::process();

    if (startUp) {
      colorWipeStatus("ES", blue, 10);
      startUp = false;
    }

    if (Serial.available())         serialEvent();
    if (blSerial.available())       serialBlEvent();
    if (stSerial.available())       serialStEvent();
    if (mpSerial.available())       serialMpEvent();
    if (s1Serial.available())       serial1Event();
    if (s2Serial.available())       serial2Event();

    if (stringComplete) autoComplete = false;
    if (stringComplete || autoComplete) {
      if (stringComplete) { inputString.toCharArray(inputBuffer, 200); inputString = ""; }
      else { autoInputString.toCharArray(inputBuffer, 200); autoInputString = ""; }

      // 2026-05 fix (BUG #6): processESPNOWIncomingMessage puts the
      // payload directly into inputString without going through
      // serialEvent's '{'-branch routing. That meant a SET_CONFIG /
      // GET_CONFIG / RESET_DEFAULTS arriving over LoRa/ESP-NOW would fall
      // through (no branch matched) and be silently ignored. Detect a
      // leading '{' here and route to handleWebSerialMessage just like
      // direct-USB JSON does. Note: chunked-config flow uses '#CC/E/B'
      // which lands in the '#' branch below and routes through there.
      if (inputBuffer[0] == '{') {
        handleWebSerialMessage(String(inputBuffer));
      } else if (inputBuffer[0] == '#') {
        commandLength = strlen(inputBuffer);
        if (inputBuffer[1]=='D' || inputBuffer[1]=='d') {
          debugInputIdentifier = "";
          for (int i=2; i<commandLength; i++) debugInputIdentifier += inputBuffer[i];
          debugInputIdentifier.toUpperCase();
          if (debugInputIdentifier == "ETM") etmToggleDebug();
          else Debug.toggle(debugInputIdentifier);
          debugInputIdentifier = "";
        } else if (inputBuffer[1]=='L' || inputBuffer[1]=='l') {
          internalCommandFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');
          Internal_Command[0] = internalCommandFunction;
        } else if (inputBuffer[1]=='E' || inputBuffer[1]=='e') {
          // EEPROM config commands  (mirrors the original BC firmware):
          //   #EE<cmd>  — stage eeprom command string only (no transmit)
          //   #EA<cmd>  — write <cmd> straight to the Body LED Controller
          //              (used for LED-brightness EEPROM settings on the ATMEGA2560)
          Debug.LOOP("EEPROM configuration selected\n");
          if (inputBuffer[2] == 'E') {
            for (int i = 2; i <= commandLength - 2; i++) eepromCommandString += inputBuffer[i];
          }
          if (inputBuffer[2] == 'A') {
            for (int i = 3; i <= commandLength - 1; i++) eepromCommandString += inputBuffer[i];
            writeBlSerial(eepromCommandString);
            eepromCommandString = "";
          }
        } else if (inputBuffer[1]=='C' || inputBuffer[1]=='c') {
          // ── Chunked config transmission (SET_CONFIG over ESP-NOW / LoRa) ──
          //  Reassembles a multi-packet config payload that's too large to fit
          //  in a single ESP-NOW frame (100-byte command limit).  See the
          //  cfgReasm* state block for the wire protocol summary.
          //
          //  Format strings INSIDE inputBuffer (after the leading '#'):
          //    CB <seq> <total>             — Begin
          //    CC <seq> <idx>:<rawBytes>    — Chunk (raw JSON after the ':')
          //    CE <seq>                     — End: parse + apply
          char sub = inputBuffer[2];
          unsigned long nowMs = millis();
          // Drop any stale transaction that timed out before completing.
          if (cfgReasmSeq != 0 && (nowMs - cfgReasmLastMs) > CFG_REASM_TIMEOUT_MS) {
            Debug.LOOP("[CFG] timeout — abandoning seq %u\n", cfgReasmSeq);
            cfgReasmSeq = 0; cfgReasmBuf = ""; cfgReasmReceived = 0;
          }
          if (sub == 'B' || sub == 'b') {
            // "#CB <seq> <total>"
            String payload = String(inputBuffer + 4);  // skip "#CB "
            int sp = payload.indexOf(' ');
            if (sp < 0) { Debug.LOOP("[CFG] bad BEGIN: %s\n", inputBuffer); }
            else {
              uint16_t seq   = payload.substring(0, sp).toInt();
              uint16_t total = payload.substring(sp + 1).toInt();
              if (seq == 0 || total == 0) {
                Debug.LOOP("[CFG] BEGIN with zero seq/total\n");
              } else {
                cfgReasmSeq      = seq;
                cfgReasmTotal    = total;
                cfgReasmReceived = 0;
                cfgReasmBuf      = "";
                int reserve = total * 90; if (reserve > CFG_REASM_MAX) reserve = CFG_REASM_MAX;
                cfgReasmBuf.reserve(reserve);
                cfgReasmLastMs   = nowMs;
                Debug.LOOP("[CFG] BEGIN seq=%u total=%u\n", seq, total);
              }
            }
          } else if (sub == 'C' || sub == 'c') {
            // "#CC <seq> <idx>:<raw bytes...>"
            //
            // 2026-05 fix (BUG #8): previously we appended in arrival order
            // and ignored <idx>. With HTTP transport the gateway may
            // deliver chunks out of order; with LoRa transport an ESP-NOW
            // retry can also flip order. Out-of-order arrival silently
            // corrupted the reassembled JSON. Now we parse <idx>, require
            // it to match the next-expected slot, and abort the transaction
            // (with a Serial NACK so it's visible on the BC's USB monitor)
            // if anything is off.
            String payload = String(inputBuffer + 4);  // skip "#CC "
            int sp = payload.indexOf(' ');
            int co = (sp >= 0) ? payload.indexOf(':', sp + 1) : -1;
            if (sp < 0 || co < 0) {
              sendBcConfigAck(0, false, "malformed");
              Debug.LOOP("[CFG] malformed CHUNK\n");
            } else {
              uint16_t seq = payload.substring(0, sp).toInt();
              uint16_t idx = payload.substring(sp + 1, co).toInt();
              if (seq != cfgReasmSeq || cfgReasmSeq == 0) {
                sendBcConfigAck(seq, false, "wrong_seq");
                Debug.LOOP("[CFG] CHUNK seq=%u rejected (active=%u)\n", seq, cfgReasmSeq);
              } else if (idx != cfgReasmReceived) {
                sendBcConfigAck(seq, false, "out_of_order");
                Debug.LOOP("[CFG] CHUNK seq=%u idx=%u out-of-order (expected %u) — aborting\n",
                           seq, idx, cfgReasmReceived);
                cfgReasmSeq = 0; cfgReasmBuf = ""; cfgReasmReceived = 0;
              } else if (cfgReasmBuf.length() + (payload.length() - co - 1) > CFG_REASM_MAX) {
                sendBcConfigAck(seq, false, "overflow");
                Debug.LOOP("[CFG] CHUNK would overflow reassembly buffer — aborting\n");
                cfgReasmSeq = 0; cfgReasmBuf = ""; cfgReasmReceived = 0;
              } else {
                cfgReasmBuf += payload.substring(co + 1);
                cfgReasmReceived++;
                cfgReasmLastMs = nowMs;
              }
            }
          } else if (sub == 'E' || sub == 'e') {
            // "#CE <seq>"
            //
            // 2026-05 fix (BLOCKER #5): on incomplete CE we used to
            // silently log to Debug only — the JS would never know the
            // push failed and would set "synced" anyway. Now we emit a
            // Serial NACK that the GUI can see (and a USB monitor can
            // catch).
            String payload = String(inputBuffer + 4);
            uint16_t seq = payload.toInt();
            if (seq != cfgReasmSeq || cfgReasmSeq == 0) {
              sendBcConfigAck(seq, false, "wrong_seq_end");
              Debug.LOOP("[CFG] END seq=%u rejected (active=%u)\n", seq, cfgReasmSeq);
            } else if (cfgReasmReceived != cfgReasmTotal) {
              sendBcConfigAck(seq, false, "incomplete");
              Debug.LOOP("[CFG] END seq=%u incomplete (got %u of %u chunks) — discarding\n",
                         seq, cfgReasmReceived, cfgReasmTotal);
              cfgReasmSeq = 0; cfgReasmBuf = ""; cfgReasmReceived = 0;
            } else {
              Debug.LOOP("[CFG] END seq=%u — assembling %u bytes, applying...\n",
                         seq, cfgReasmBuf.length());
              // Stash the seq so handleWebSerialMessage can echo it in its
              // own ACK (it doesn't know about cfgReasmSeq otherwise).
              lastChunkSeqForAck = seq;
              handleWebSerialMessage(cfgReasmBuf);
              lastChunkSeqForAck = 0;
              cfgReasmSeq = 0; cfgReasmBuf = ""; cfgReasmReceived = 0;
            }
          }
        }
        if (Internal_Command[0]) {
          switch (Internal_Command[0]) {
            case 1: Serial.println(HOSTNAME); break;
            case 2: ESP.restart(); break;
            case 8: printKeepaliveStatus(); break;                       // #L08 — full BL status dump
            case 9: dumpSbusState(); break;                              // #L09 — one-shot SBUS dump
            case 10: sbusLiveDump = !sbusLiveDump;                       // #L10 — toggle live SBUS spam
                     Serial.printf("SBUS live dump %s\n", sbusLiveDump ? "ON (1Hz)" : "OFF"); break;
          }
          Internal_Command[0] = '\0';
        }
      } else if (inputBuffer[0] == ':') {
        commandLength = strlen(inputBuffer);
        if (commandLength >= 3) {
          if (inputBuffer[1]=='E' || inputBuffer[1]=='e') {
            // 2026-05 fix: was i<=commandLength which reads one past
            // strlen() into the NUL terminator. Arduino String += '\0'
            // corrupts internal length tracking, silently truncating
            // downstream substring() calls. Same bug fixed in Gateway.
            for (int i=2; i<commandLength; i++) ESPNOWCommandString += inputBuffer[i];
            ESPNOWTarget       = ESPNOWCommandString.substring(0,2);
            ESPNOWTargetCommand= ESPNOWCommandString.substring(2);
            sendESPNOWCommand(ESPNOWTarget, ESPNOWTargetCommand);
            ESPNOWCommandString = ESPNOWTargetCommand = ESPNOWTarget = "";
          }
          if (inputBuffer[1]=='S' || inputBuffer[1]=='s') {
            for (int i=2; i<commandLength-1; i++) serialCommandString += inputBuffer[i];
            serialPort = serialCommandString.substring(0,2);
            serialCommandSubString = serialCommandString.substring(2,commandLength);
            if (serialPort=="S1") writeS1Serial(serialCommandSubString);
            else if (serialPort=="S2") writeS2Serial(serialCommandSubString);
            else if (serialPort=="MP") writeMpSerial(serialCommandSubString);
            serialCommandString = serialPort = serialCommandSubString = "";
          }
          if (inputBuffer[1]=='L' || inputBuffer[1]=='l') {
            for (int i=2; i<commandLength; i++) ledCommandString += inputBuffer[i];
            // 2026-06 diag: the LED relay was silent, so "nothing happens" couldn't
            // be told apart from "BC parsed + forwarded but the mega/link is dead".
            // This prints exactly what's being pushed to the mega over blSerial @9600.
            // If you SEE this line but the LEDs don't move → it's the BC→mega link
            // (wiring/ground) or the mega itself. If you DON'T see it → the command
            // never reached this branch (line-ending / wrong format / serial RX).
            Serial.printf("[BL->mega @9600] forwarding: '%s'\n", ledCommandString.c_str());
            writeBlSerial(ledCommandString); ledCommandString = "";
          }
          if (inputBuffer[1]=='A' || inputBuffer[1]=='a') {
            AnimationCommandFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');
            callAnimation(AnimationCommandFunction);
          }
          if (inputBuffer[1]=='R' || inputBuffer[1]=='r') {
            for (int i=2; i<commandLength; i++) radhCommandString += inputBuffer[i];
            writeRdSerial(radhCommandString); radhCommandString = "";
          }
          if (inputBuffer[1]=='M' || inputBuffer[1]=='m') {
            for (int i=2; i<commandLength; i++) mp3CommandString += inputBuffer[i];
            String hFn = getValue(mp3CommandString,',',0), hCh = getValue(mp3CommandString,',',1);
            String hTr = getValue(mp3CommandString,',',2), hSt = getValue(mp3CommandString,',',3);
            HCRFunction(hFn.toInt(), hCh.toInt(), hTr.toInt(), hTr);
            mp3CommandString = "";
          }
          if (inputBuffer[1]=='C' || inputBuffer[1]=='c') {
            for (int i=2; i<commandLength; i++) controllerCommandString += inputBuffer[i];
            writeStSerial(controllerCommandString); controllerCommandString = "";
          }
        }
      }

      stringComplete = autoComplete = false;
      inputBuffer[0] = inputBuffer[1] = '\0';
    }

    if (isStartUp) { isStartUp = false; delay(500); }
  }
}
