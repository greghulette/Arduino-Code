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
// rdSerial = hardware UART1 (Serial1) — matches original BC firmware on
// the production WROOM-32 board.  (On PICO-MINI-02 hardware the default
// UART1 pins clash with flash; that variant must use SoftwareSerial.)
#define rdSerial Serial1
SoftwareSerial blSerial, stSerial, mpSerial, s1Serial, s2Serial;
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

bodyControllerStatus_struct_message commandsToSendtoDroidLoRa;
espnow_struct_message outgoingMsg, incomingMsg;
esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == 0) SuccessCounter++; else FailureCounter++;
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
      processESPNOWIncomingMessage();
      etmSendAck(senderIdx, incomingMsg.structSequenceNumber);
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
void serialEvent() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\r' || inChar == '\n') {
      inputString.trim();
      if (inputString.length() > 0) {
        if (inputString[0] == '{') {
          // WebSerial JSON protocol line
          handleWebSerialMessage(inputString);
          inputString = "";
        } else {
          stringComplete = true;
        }
      } else {
        inputString = "";
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
void writeBlSerial(String s) { for (char c : (s + '\r')) blSerial.write(c); }
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

void sendESPNOWCommand(String starget, String scomm) {
  bool hasCommand = (scomm.length() > 0);
  if (starget == "DG") {
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
int oldValueMatrix   = 100;
int oldValueMode     = 100;
bool lostFrameOld    = false;
int  sbusValues[24]  = {0};  // Sized for SBUS-24 max; SBUS-16 leaves CH17-24 at 0.

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

    // Button matrix on the configurable matrix channel
    int mxCh = rcConfig.matrixChannel;
    if (mxCh >= 1 && mxCh <= 24) {
      int mxVal = sbusValues[mxCh - 1];
      if (abs(mxVal - oldValueMatrix) >= 5) {
        oldValueMatrix = mxVal;
        RCRadio_Matrix_Buttons(mxVal);
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
    Serial.println("{\"type\":\"PONG\"}");
    return;
  }

  // ── GET_CONFIG ─────────────────────────────────────────────────────────────
  if (strcmp(type, "GET_CONFIG") == 0) {
    String cfg = rcConfigToJSON();
    // Send as a response envelope — split into two prints to keep RAM low
    Serial.print("{\"type\":\"CONFIG\",\"data\":");
    Serial.print(cfg);
    Serial.println("}");
    return;
  }

  // ── SET_CONFIG ─────────────────────────────────────────────────────────────
  if (strcmp(type, "SET_CONFIG") == 0) {
    // Larger doc (~96 KB) to handle the full config envelope.  Worst case is
    // 3 modes × 19 buttons × 5 actions plus 8 switches × 3 pos × 5 actions
    // plus thresholds/bindings/knobs ≈ 25 KB.  96 KB gives stronger headroom
    // for bigger editor payloads and future config additions.
    DynamicJsonDocument bigDoc(98304);
    DeserializationError err = deserializeJson(bigDoc, line);
    if (err != DeserializationError::Ok) {
      Serial.printf("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"parse failed: %s\"}\n", err.c_str());
      return;
    }
    if (!bigDoc.containsKey("data")) {
      Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"missing data\"}");
      Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"missing data\"}");
      return;
    }
    JsonObject data = bigDoc["data"].as<JsonObject>();
    bool ok = rcConfigFromJSON(data);
    if (ok) {
      rcConfigSaveNVS();
      Serial.println("{\"type\":\"ACK\",\"ok\":true}");
    } else {
      Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"config apply failed\"}");
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
    Serial.println("{\"type\":\"ACK\",\"ok\":true}");
    return;
  }

  // ── RESET_DEFAULTS ─────────────────────────────────────────────────────────
  if (strcmp(type, "RESET_DEFAULTS") == 0) {
    rcConfigLoadDefaults();
    Serial.println("{\"type\":\"ACK\",\"ok\":true}");
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
  Serial.flush();

  // rdSerial = hardware Serial1, remapped to SERIAL_RX_RD_PIN/SERIAL_TX_RD_PIN
  rdSerial.begin(RD_BAUD_RATE,  SERIAL_8N1,    SERIAL_RX_RD_PIN, SERIAL_TX_RD_PIN);
  blSerial.begin(BL_BAUD_RATE,  SWSERIAL_8N1,  SERIAL_RX_BL_PIN, SERIAL_TX_BL_PIN, false, 95);
  stSerial.begin(ST_BAUD_RATE,  SWSERIAL_8N1,  SERIAL_RX_ST_PIN, SERIAL_TX_ST_PIN, false, 95);
  mpSerial.begin(MP_BAUD_RATE,  SWSERIAL_8N1,  SERIAL_RX_MP_PIN, SERIAL_TX_MP_PIN, false, 95);
  s1Serial.begin(SERIAL1_BAUD_RATE, SWSERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN,  false, 95);
  s2Serial.begin(SERIAL2_BAUD_RATE, SWSERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN,  false, 95);

#ifdef SBUS
  // Custom SBUS reader handles both 16ch and 24ch frames on the same pins.
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
void loop() {
  etmProcess();
  keepAlive();
  checkAgeofkeepAlive();

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

      if (inputBuffer[0] == '#') {
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
            for (int i=2; i<=commandLength; i++) ESPNOWCommandString += inputBuffer[i];
            ESPNOWTarget       = ESPNOWCommandString.substring(0,2);
            ESPNOWTargetCommand= ESPNOWCommandString.substring(2,commandLength+1);
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
