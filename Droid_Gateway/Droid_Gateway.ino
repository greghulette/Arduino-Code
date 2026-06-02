///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                                                                                                       *****///
///*****                            Created by Greg Hulette.                                                   *****///
///*****                                                                                                       *****///
///*****                               So exactly what does this all do.....?                                  *****///
///*****                       - Acts as a bridge between ESP-NOW and LoRa                                     *****///
///*****                       - Controls the master relay on the flip down electronics board                  *****///
///*****                       - Monitors operational status of other ESP boards in my Droid                   *****///
///*****                                                                                                       *****///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////








//////////////////////////////////////////////////////////////////////
///*****        Libraries used in this sketch                 *****///
//////////////////////////////////////////////////////////////////////

// Standard Arduino library
#include <Arduino.h>

// Used for OTA
#include "ESPAsyncWebServer.h"
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
#include <ElegantOTA.h>
#include <AsyncTCP.h>
#include <WiFi.h>

//Used for ESP-NOW
#include "esp_wifi.h"
#include <esp_now.h>

// ETM — Ensured Transmission Mode for Droid ESP-NOW network
#define ETM_MY_BOARD_INDEX ETM_BOARD_DG
#include <ETM_Droid.h>

//Used for status LEDs
#include <Adafruit_NeoPixel.h>

//Used for pin definition
#include "droid_gateway_pin_map.h"

// Debug Functions  - Using my own library for this
#include <DebugR2.h>  //  https://github.com/greghulette/Arduino-Code/tree/main/libraries/DebugR2  Put these files in a folder called "DebugR2" in your libraries folder and restart the IDE

//ReelTwo libaries - Using my forked version of this libarary at https://github.com/greghulette/Reeltwo
#include <ReelTwo.h>   
#include "core/DelayCall.h"

//Used for LoRa
#include <SPI.h>
// #include "LoRa.h"  // Using the ReelTwo LoRa library.  Files included in this sketch's folder.
#include <LoRa.h>






//////////////////////////////////////////////////////////////////////
///*****          Preferences/Items to change                 *****///
//////////////////////////////////////////////////////////////////////
 // ESPNOW Password - This must be the same across all devices
  String ESPNOWPASSWORD = "GregsAstromech";

    // R2 Control Network Details
  const char* ssid = "R2D2_Control_Network";
  const char* password =  "astromech";

  // Keepalive timer to send status messages to the Kill Switch (Droid)
  int keepAliveDuration= 4000;  // 4 seconds
  bool sendUpdateStatus = true;
  int sendStatusFrequency = 4000;

// used to sync timing with the dome controller better, allowing time for the ESP-NOW messages to travel to the dome
// Change this to work with how your droid performs
  int defaultESPNOWSendDuration = 50;  

  // Serial Baud Rates
  #define SERIAL1_BAUD_RATE 115200







//////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
  String HOSTNAME = "Droid Gateway";

  char inputBuffer[100];
  String inputString;         // a string to hold incoming data

  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA

  int commandLength;
  
  String serialStringCommand;
  String serialPort;
  String serialSubStringCommand;

  uint32_t Local_Command[6]  = {0,0,0,0,0,0};
  int localCommandFunction     = 0;

  String ESPNOWStringCommand;
  String ESPNOWTarget;
  String ESPNOWSubStringCommand;

debugClass Debug;
String debugInputIdentifier ="";








//////////////////////////////////////////////////////////////////////
  ///*****       Startup and Loop Variables                     *****///
  //////////////////////////////////////////////////////////////////////

  boolean startUp = true;
  boolean isStartUp = true;
  
    //Main Loop Timers
  unsigned long mainLoopTime; 
  unsigned long MLMillis;
  byte mainLoopDelayVar = 5;


///////////////////////////////////////////////////////////////////////
  ///*****                Status Variables                     *****///
  /////////////////////////////////////////////////////////////////////
  unsigned long statusCurrentMillis;
  unsigned long statusPreviousMillis;
  unsigned long statusDelayInterval = 5000;

  bool droidGatewayStatus = 1;
  bool bodyControllerStatus ;
  bool bodyLEDControllerStatus;  
  bool bodyServoControllerStatus;
  bool domePlateControllerStatus;
  bool domeControllerStatus;
  bool droidRemoteStatus;
  bool hpControllerStatus;
  bool domeLogicsControllerStatus;


  int keepAliveTimeOut = 15000;
  unsigned long keepAliveMillis;
  unsigned long dckeepAliveAge;
  unsigned long dckeepaliveAgeMillis;
  unsigned long dpkeepAliveAge;
  unsigned long dpkeepaliveAgeMillis;
  unsigned long bskeepAliveAge;
  unsigned long bskeepaliveAgeMillis;
  unsigned long bckeepAliveAge;
  unsigned long bckeepaliveAgeMillis;
  unsigned long blkeepAliveAge;
  unsigned long blkeepaliveAgeMillis;
  unsigned long drkeepAliveAge;
  unsigned long hpkeepaliveAgeMillis;
  unsigned long hpkeepAliveAge;
  unsigned long sendStatusMillis;
  unsigned long sendStatusAge;
    // variables for storing status and settings from ATMEGA2560
  int BL_LDP_Bright;
  int BL_MAINT_Bright;
  int BL_VU_Bright;
  int BL_CS_Bright;
  int BL_vuOffsetInt;
  int BL_vuBaselineInt;
  int BL_vuOffsetExt;
  int BL_vuBaselineExt;
  float BL_BatteryVoltage;
  int BL_BatteryPercentage;
  int FunctionSWState;

  bool ACKBool = false;
  byte incomingMsgId;
  byte lastProcessedMsgId = 0xFF;  // 0xFF = "none yet"; used for deduplication
  uint32_t msgAckID;
  //////////////////////////////////////////////////////////////////
  ///******       Serial Ports Definitions                  *****///
  //////////////////////////////////////////////////////////////////

  #define s1Serial Serial1







  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////

//Droid Remote ESP          192.168.4.101   
//Droid Gateway ESP         192.168.4.108    ************ (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Controller ESP       192.168.4.109   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Servo ESP            192.168.4.110   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Controller ESP       192.168.4.111   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Plate Controller ESP 192.168.4.112   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//HP Controller ESP         192.168.4.113   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Droid Raspberry Pi        192.168.4.114
//Remote Raspberry Pi       192.168.4.115
//Developer Laptop          192.168.4.125
  
  // IP Address config of local ESP
  IPAddress local_IP(192,168,4,108);
  IPAddress subnet(255,255,255,0);
  IPAddress gateway(192,168,4,101);
  
  const uint8_t oldLocalMACAddress[] = {0x24, 0x0A, 0xC4, 0xED, 0x30, 0x11};    //used when connecting to WiFi for OTA

  AsyncWebServer server(80);
  

//////////////////////////////////////////////////////////////////////
///*****            Status LED Variables and settings       *****///
//////////////////////////////////////////////////////////////////////

// -------------------------------------------------
// Define some constants to help reference objects,
// pins, leds, colors etc by name instead of numbers
// -------------------------------------------------
//    CAMERA LENS LED VARIABLES
    const uint32_t red     = 0xFF0000;
    const uint32_t orange  = 0xFF8000;
    const uint32_t yellow  = 0xFFFF00;
    const uint32_t green   = 0x00FF00;
    const uint32_t cyan    = 0x00FFFF;
    const uint32_t blue    = 0x0000FF;
    const uint32_t magenta = 0xFF00FF;
    const uint32_t white   = 0xFFFFFF;
    const uint32_t off     = 0x000000;

    const uint32_t basicColors[9] = {off, red, yellow, green, cyan, blue, magenta, orange, white};

  #define STATUS_LED_COUNT 1
  
  Adafruit_NeoPixel ESP_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel RELAY_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, RELAY_LED_PIN, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel LORA_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, LORA_LED_PIN, NEO_GRB + NEO_KHZ800);



































































/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

// MAC addresses and board IDs are defined in ETM_Droid.h
// Use ETM_BOARD_MACS[ETM_BOARD_XX] and ETM_BOARD_IDS[ETM_BOARD_XX]

// Define variables to store commands to be sent
  String  senderID;
  String  targetID;
  bool    commandIncluded;
  String  command;


// Define variables to store incoming commands
  String incomingPassword;
  String incomingTargetID;  
  String incomingSenderID;
  bool incomingCommandIncluded;
  String incomingCommand;
  int incomingstructBL_LDP_Bright;
  int incomingstructBL_MAINT_Bright;
  int incomingstructBL_VU_Bright;
  int incomingstructBL_CS_Bright;
  int incomingstructBL_vuOffsetInt;
  int incomingstructBL_vuBaselineInt;
  int incomingstructBL_vuOffsetExt;
  int incomingstructBL_vuBaselineExt;
  float incomingstructBL_BatteryVoltage;
  int incomingstructBL_BatteryPercentage;
  int incomingstructFunctionSWState;
  bool remoteConnected;
  bool incomingstructbodyLEDControllerStatus;
  uint32_t DGSuccessCounter = 0;
  uint32_t DGFailureCounter = 0;
  uint32_t BSSuccessCounter;
  uint32_t BSFailureCounter;
  uint32_t BCSuccessCounter;
  uint32_t BCFailureCounter;
  uint32_t DPSuccessCounter;
  uint32_t DPFailureCounter;
  uint32_t DCSuccessCounter;
  uint32_t DCFailureCounter;
  uint32_t HPSuccessCounter;
  uint32_t HPFailureCounter;
  // BC's reverse-channel ACK fields (received from BC, forwarded to GUI).
  // See the matching block in Body_Controller_ESP32_GUI.ino. The Gateway
  // just relays — no interpretation. Web GUI matches against its
  // state.lastSentSeq to know whether the ACK is for its own push.
  uint16_t BC_bcAckSeq      = 0;
  bool     BC_bcAckOk       = false;
  char     BC_bcAckMsg[24]  = {0};
  // 2026-05 servo read-back: most-recent "#LI..."/"#LV..." reply captured from
  // a DC/BS/DP ESP-NOW frame. Sent to the Remote in its OWN small LoRa packet
  // (ServoInfo_LoRa_Struct) — NOT the telemetry struct, which is already 252 of
  // the 255-byte LoRa limit. servoInfoPending flags a buffered reply that the
  // main loop ships out when the LoRa radio is free. Loss is fine: the web GUI
  // retries each "describe" request, so a dropped packet just gets re-sent.
  volatile bool servoInfoPending = false;
  char     servoInfoPayload[96]  = {0};
  // 2026-05 GET_CONFIG return path: most-recent cfg chunk captured from
  // the BC status struct. When BC_cfgSeq>0 we immediately ship the chunk
  // out over LoRa via sendCfgChunkLoRa() and then zero these so they
  // don't replay on the next telemetry tick.
  uint16_t BC_cfgSeq        = 0;
  uint16_t BC_cfgIdx        = 0;
  uint16_t BC_cfgTotal      = 0;
  uint8_t  BC_cfgLen        = 0;
  char     BC_cfgPayload[100] = {0};
  // 2026-05 (v4): defer the LoRa ACK from inside onReceive to the main
  // loop. The old code called sendACK() (which does a full LoRa.endPacket
  // taking ~250ms) directly inside the onReceive callback path. During
  // a config burst, this starved tickCfgChunkQueue and caused the chunk
  // queue to overflow. Now onReceive just sets pendingLoRaAck=true and
  // the main loop fires sendACK AFTER tickCfgChunkQueue has had a chance
  // to drain any pending cfg chunks.
  volatile bool pendingLoRaAck   = false;
  volatile int  pendingLoRaAckId = 0;
  // BC's ETM per-board delivery stats (received via bodyControllerStatus_struct_message)
  // Indexed by ETM_BOARD_* constants
  bool     BC_etmBoardOnline[ETM_NUM_BOARDS]  = {};
  uint32_t BC_etmBoardSent[ETM_NUM_BOARDS]    = {};
  uint32_t BC_etmBoardAckd[ETM_NUM_BOARDS]    = {};
  uint32_t BC_etmBoardRetries[ETM_NUM_BOARDS] = {};
  uint32_t BC_etmBoardFailed[ETM_NUM_BOARDS]  = {};
  
// Variable to store if sending data was successful
  String success;

// espnow_struct_message is defined in ETM_Droid.h

// NOTE: must match Body_Controller_ESP32_GUI.ino exactly — both sides of the
// ESP-NOW link share this layout. If you add a field, add it in BOTH files
// in the SAME order or the receive-side memcpy will corrupt the payload.
//
// 2026-05 NOTE: The reverse-channel ACK and GET_CONFIG cfg-chunk fields
// USED TO live in this struct. They were briefly moved into a separate
// BcEvent_struct_message dispatched by len, but that path used raw
// esp_now_send() with no retries — 10-20% of chunks dropped silently.
// 2026-05 (v4): ACKs and chunks now ride the ETM-tracked command path
// from BC as TEXT commands ("#A ..." for ACKs, "#C ..." for chunks)
// inside the standard espnow_struct_message.structCommand. They are
// detected by prefix in processESPNOWIncomingMessage(). The
// BcEvent_struct_message typedef is gone.
typedef struct bodyControllerStatus_struct_message{
      char structPassword[25];
      char structSenderID[15];
      char structTargetID[5];
      int structBL_LDP_Bright;
      int structBL_MAINT_Bright;
      int structBL_VU_Bright;
      int structBL_CS_Bright;
      int structBL_vuOffsetInt;
      int structBL_vuBaselineInt;
      int structBL_vuOffsetExt;
      int structBL_vuBaselineExt;
      float structBL_BatteryVoltage;
      int structBL_BatteryPercentage;
      bool structbodyLEDControllerStatus;
      int structFunctionSWState;
      bool struct_remoteConnected;
      bool structCommandIncluded;
      uint32_t structSuccess;
      uint32_t structFailure;
      char structCommand[100];
      // ETM delivery stats — one entry per board (indexed by ETM_BOARD_*)
      bool     etmBoardOnline[ETM_NUM_BOARDS];
      uint32_t etmBoardSent[ETM_NUM_BOARDS];
      uint32_t etmBoardAckd[ETM_NUM_BOARDS];
      uint32_t etmBoardRetries[ETM_NUM_BOARDS];
      uint32_t etmBoardFailed[ETM_NUM_BOARDS];
  } bodyControllerStatus_struct_message;

// 2026-05 (v4): BcEvent_struct_message removed. ACKs and GET_CONFIG
// chunks now arrive via the ETM-tracked command path (espnow_struct_message
// PACKET_TYPE_COMMAND with the text "#A <seq> <ok> <msg>" or
// "#C <seq> <idx> <total>:<payload>" in structCommand). They're handled
// inline in processESPNOWIncomingMessage() — see the prefix dispatch
// there.

// Create a espnow_struct_message called commandsTosendto****** to hold variables that will be sent
  espnow_struct_message commandsToSendtoBroadcast;
  espnow_struct_message commandsToSendtoDroidLoRa;
  espnow_struct_message commandsToSendtoBodyController;
  espnow_struct_message commandsToSendtoBodyServoController;
  espnow_struct_message commandsToSendtoDomeController;
  espnow_struct_message commandsToSendtoDomePlateController;
  espnow_struct_message commandsToSendtoHPController;

// Create a espnow_struct_message to hold variables that will be received
  espnow_struct_message commandsToReceiveFromBroadcast;
  espnow_struct_message commandsToReceiveFromDroidLoRa;
  bodyControllerStatus_struct_message commandsToReceiveFromBodyController;
  espnow_struct_message commandsToReceiveFromBodyServoController;
  espnow_struct_message commandsToReceiveFromDomeController;
  espnow_struct_message commandsToReceiveFromDomePlateController;
  espnow_struct_message commandsToReceiveFromHPController;

  esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == 0) { DGSuccessCounter++; } else { DGFailureCounter++; }
  if (Debug.debugflag_espnow == 1) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}

// Forward decl: defined alongside LoRa_Struct below; called from OnDataRecv
// to immediately forward a BC cfg chunk over LoRa as a separate packet.
void sendCfgChunkLoRa();

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  colorWipeStatus("ES", orange, 255);

  int senderIdx = etmBoardIndexFromMAC(esp_now_info->src_addr);
  Debug.ESPNOW("Received ESP-NOW packet from board index %d, len %d\n", senderIdx, len);
  // 2026-05 (v4) diagnostic: BC now sends only two sizes —
  //   sizeof(bodyControllerStatus_struct_message) — periodic heartbeat
  //   sizeof(espnow_struct_message)               — ETM command frames
  //     (PACKET_TYPE_COMMAND text "#A ..." ACK / "#C ..." chunk arrive here)
  // Anything else is unexpected.
  if (senderIdx == ETM_BOARD_BC &&
      len != (int)sizeof(bodyControllerStatus_struct_message) &&
      len != (int)sizeof(espnow_struct_message)) {
    Serial.printf("[DG] BC packet size mismatch: got %d, expected %u (status) or %u (etm)\n",
                  len,
                  (unsigned)sizeof(bodyControllerStatus_struct_message),
                  (unsigned)sizeof(espnow_struct_message));
  }

  // Body Controller sends a custom status struct with a different size —
  // handle it separately before the ETM path.
  if (senderIdx == ETM_BOARD_BC && len == sizeof(bodyControllerStatus_struct_message)) {
    memcpy(&commandsToReceiveFromBodyController, incomingData, sizeof(commandsToReceiveFromBodyController));
    incomingPassword = commandsToReceiveFromBodyController.structPassword;
    if (incomingPassword != ESPNOWPASSWORD) {
      Debug.ESPNOW("Wrong ESP-NOW Password from BC. Message Ignored\n");
    } else {
      incomingSenderID        = commandsToReceiveFromBodyController.structSenderID;
      incomingTargetID        = commandsToReceiveFromBodyController.structTargetID;
      BL_LDP_Bright           = commandsToReceiveFromBodyController.structBL_LDP_Bright;
      BL_MAINT_Bright         = commandsToReceiveFromBodyController.structBL_MAINT_Bright;
      BL_VU_Bright            = commandsToReceiveFromBodyController.structBL_VU_Bright;
      BL_CS_Bright            = commandsToReceiveFromBodyController.structBL_CS_Bright;
      BL_vuOffsetInt          = commandsToReceiveFromBodyController.structBL_vuOffsetInt;
      BL_vuBaselineInt        = commandsToReceiveFromBodyController.structBL_vuBaselineInt;
      BL_vuOffsetExt          = commandsToReceiveFromBodyController.structBL_vuOffsetExt;
      BL_vuBaselineExt        = commandsToReceiveFromBodyController.structBL_vuBaselineExt;
      BL_BatteryVoltage       = commandsToReceiveFromBodyController.structBL_BatteryVoltage;
      BL_BatteryPercentage    = commandsToReceiveFromBodyController.structBL_BatteryPercentage;
      bodyLEDControllerStatus = commandsToReceiveFromBodyController.structbodyLEDControllerStatus;
      FunctionSWState         = commandsToReceiveFromBodyController.structFunctionSWState;
      remoteConnected         = commandsToReceiveFromBodyController.struct_remoteConnected;
      incomingCommandIncluded = commandsToReceiveFromBodyController.structCommandIncluded;
      BCSuccessCounter        = commandsToReceiveFromBodyController.structSuccess;
      BCFailureCounter        = commandsToReceiveFromBodyController.structFailure;
      incomingCommand         = commandsToReceiveFromBodyController.structCommand;
      // Extract BC's ETM per-board delivery stats
      for (int i = 0; i < ETM_NUM_BOARDS; i++) {
        BC_etmBoardOnline[i]  = commandsToReceiveFromBodyController.etmBoardOnline[i];
        BC_etmBoardSent[i]    = commandsToReceiveFromBodyController.etmBoardSent[i];
        BC_etmBoardAckd[i]    = commandsToReceiveFromBodyController.etmBoardAckd[i];
        BC_etmBoardRetries[i] = commandsToReceiveFromBodyController.etmBoardRetries[i];
        BC_etmBoardFailed[i]  = commandsToReceiveFromBodyController.etmBoardFailed[i];
      }
      // 2026-05 (v4): ACK and cfg-chunk fields are NOT in this struct.
      // They arrive as ETM PACKET_TYPE_COMMAND text frames ("#A ..." /
      // "#C ...") and are dispatched in processESPNOWIncomingMessage().
      etmHandleHeartbeat(ETM_BOARD_BC);
      processESPNOWIncomingMessage();
    }
    colorWipeStatus("ES", blue, 10);
    return;
  }

  // 2026-05 (v4): the BcEvent receive branch is gone. BC's ACKs and
  // GET_CONFIG chunks now arrive as ETM PACKET_TYPE_COMMAND frames
  // (text "#A ..." / "#C ..." in structCommand) and are dispatched
  // inline at the top of processESPNOWIncomingMessage().

  // All other messages use the standard ETM struct
  if (len < (int)sizeof(espnow_struct_message)) {
    Debug.ESPNOW("Received undersized ESP-NOW packet (%d bytes), ignoring\n", len);
    colorWipeStatus("ES", blue, 10);
    return;
  }

  espnow_struct_message incoming;
  memcpy(&incoming, incomingData, sizeof(incoming));

  if (strncmp(incoming.structPassword, ESPNOWPASSWORD.c_str(), sizeof(incoming.structPassword)) != 0) {
    Debug.ESPNOW("Wrong ESP-NOW Password. Message Ignored\n");
    colorWipeStatus("ES", blue, 10);
    return;
  }

  // Any received packet counts as a heartbeat — marks the sender online
  if (senderIdx >= 0) etmHandleHeartbeat(senderIdx);

  switch (incoming.structPacketType) {
    case PACKET_TYPE_HEARTBEAT:
      // Already handled by etmHandleHeartbeat above
      break;

    case PACKET_TYPE_ACK:
      etmProcessAck(senderIdx, incoming.structSequenceNumber);
      break;

    case PACKET_TYPE_COMMAND:
      if (senderIdx < 0) {
        Debug.ESPNOW("Command from unknown MAC (sender: %s), ignoring\n", incoming.structSenderID);
        break;
      }
      incomingSenderID        = incoming.structSenderID;
      incomingTargetID        = incoming.structTargetID;
      incomingCommandIncluded = incoming.structCommandIncluded;
      incomingCommand         = incoming.structCommand;
      // Update per-board counters that processESPNOWIncomingMessage uses
      if (senderIdx == ETM_BOARD_BS) { BSSuccessCounter = incoming.structSuccess; BSFailureCounter = incoming.structFailure; }
      if (senderIdx == ETM_BOARD_DC) { DCSuccessCounter = incoming.structSuccess; DCFailureCounter = incoming.structFailure; }
      if (senderIdx == ETM_BOARD_DP) { DPSuccessCounter = incoming.structSuccess; DPFailureCounter = incoming.structFailure; }
      if (senderIdx == ETM_BOARD_HP) { HPSuccessCounter = incoming.structSuccess; HPFailureCounter = incoming.structFailure; }
      processESPNOWIncomingMessage();
      etmSendAck(senderIdx, incoming.structSequenceNumber);
      break;

    default:
      Debug.ESPNOW("Unknown ESP-NOW packet type %d, ignoring\n", incoming.structPacketType);
      break;
  }

  colorWipeStatus("ES", blue, 10);
}

void processESPNOWIncomingMessage(){
  Debug.ESPNOW("incoming target: %s\n", incomingTargetID.c_str());
  Debug.ESPNOW("incoming sender: %s\n", incomingSenderID.c_str());
  Debug.ESPNOW("incoming command included: %d\n", incomingCommandIncluded);
  Debug.ESPNOW("incoming command: %s\n", incomingCommand.c_str());

  // 2026-05 (v4): BC's reverse-channel ACKs and GET_CONFIG chunks now
  // arrive as ordinary ETM PACKET_TYPE_COMMAND frames addressed to "DG"
  // (or "BR" on a broadcast), with the payload text-encoded inside
  // incomingCommand:
  //     "#A <seq> <ok> <msg>"          — config ACK / NACK
  //     "#C <seq> <idx> <total>:<payload>" — GET_CONFIG chunk
  // Detect by prefix and copy fields into the same downstream globals
  // (BC_bcAck* / enqueueCfgChunk) the rest of the pipeline already uses.
  // ETM provides the reliability (sequence numbers, ACK, 3 retries with
  // 500ms timeout) so the dedicated BcEvent_struct_message frame is gone.
  if (incomingSenderID == "BC" && (incomingTargetID == "DG" || incomingTargetID == "BR")) {
    if (incomingCommand.startsWith("#A ")) {
      String rest = incomingCommand.substring(3);
      int sp1 = rest.indexOf(' ');
      int sp2 = (sp1 > 0) ? rest.indexOf(' ', sp1 + 1) : -1;
      if (sp1 > 0 && sp2 > 0) {
        uint16_t seq = (uint16_t)rest.substring(0, sp1).toInt();
        bool ok = rest.substring(sp1 + 1, sp2) == "1";
        String msg = rest.substring(sp2 + 1);
        BC_bcAckSeq = seq;
        BC_bcAckOk  = ok;
        strncpy(BC_bcAckMsg, msg.c_str(), sizeof(BC_bcAckMsg) - 1);
        BC_bcAckMsg[sizeof(BC_bcAckMsg) - 1] = '\0';
      }
      return;
    }
    if (incomingCommand.startsWith("#C ")) {
      int spaceA = incomingCommand.indexOf(' ', 3);
      int spaceB = (spaceA > 0) ? incomingCommand.indexOf(' ', spaceA + 1) : -1;
      int colon  = (spaceB > 0) ? incomingCommand.indexOf(':', spaceB + 1) : -1;
      if (spaceA > 0 && spaceB > 0 && colon > 0) {
        uint16_t seq   = (uint16_t)incomingCommand.substring(3, spaceA).toInt();
        uint16_t idx   = (uint16_t)incomingCommand.substring(spaceA + 1, spaceB).toInt();
        uint16_t total = (uint16_t)incomingCommand.substring(spaceB + 1, colon).toInt();
        String payload = incomingCommand.substring(colon + 1);
        uint8_t len = (uint8_t)payload.length();
        enqueueCfgChunk(seq, idx, total, len, payload.c_str());
      }
      return;
    }
  }

  // 2026-05 servo-limit read-back: a DC/BS/DP controller replies to the GUI's
  // requests with "#LV..." (page values, from #LG) or "#LI..." (one servo's
  // name + values, from #LD) addressed to DG. Capture it; the main loop ships
  // it to the Remote in its own ServoInfo LoRa packet (NOT the telemetry
  // struct, which is already at the 255-byte LoRa limit).
  if ((incomingSenderID == "DC" || incomingSenderID == "BS" || incomingSenderID == "DP") &&
      (incomingTargetID == "DG" || incomingTargetID == "BR") &&
      (incomingCommand.startsWith("#LV") || incomingCommand.startsWith("#LI"))) {
    strncpy(servoInfoPayload, incomingCommand.c_str(), sizeof(servoInfoPayload) - 1);
    servoInfoPayload[sizeof(servoInfoPayload) - 1] = '\0';
    servoInfoPending = true;          // main loop ships it out over LoRa
    Debug.ESPNOW("Servo reply captured: %s\n", servoInfoPayload);
    return;
  }

    if (incomingTargetID == "DG" || incomingTargetID == "BR"){
      if (incomingSenderID == "BC"){
        bodyControllerStatus = 1;
        bckeepAliveAge =millis();
        // sendStatusMessageNow();
        if (bodyLEDControllerStatus == 1){
          blkeepAliveAge = millis();
        }
        if (incomingCommandIncluded == 1){
          inputString = incomingCommand;
          stringComplete = true; 
        }
        //debug statements
        Debug.STATUS("\nBody Controller Status Update \n");
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
        Debug.STATUS("Function Sw State %i \n", FunctionSWState);
        Debug.STATUS("Remote Connected Status: %i\n", remoteConnected);
        Debug.STATUS("Body LED Controller Status: %d \n", bodyLEDControllerStatus);
        Debug.ESPNOW("ESP NOW Message Received from Body Controller \n");
        Debug.STATUS("Body Controller Statisics:  Success: %i  Failuure: %i\n", BCSuccessCounter, BCFailureCounter);

        
      }else if (incomingSenderID == "BS"){
        bodyServoControllerStatus = 1;
        bskeepAliveAge =millis();

        if (incomingCommandIncluded == 1){
          inputString = incomingCommand;
          stringComplete = true; 
        }

        Debug.STATUS("Body Servo Status Update \n");
        Debug.ESPNOW("ESP NOW Message Received from Body Servo Controller \n");
        Debug.STATUS("Body Servo Statisics:  Success: %i  Failuure: %i\n", BSSuccessCounter, BSFailureCounter);

      }
      else if (incomingSenderID == "DC"){
        domeControllerStatus = 1;
        dckeepAliveAge =millis();
        if (incomingCommandIncluded == 1){
          inputString = incomingCommand;
          stringComplete = true; 
        }

        Debug.STATUS("Dome Controller Status Update \n");
        Debug.ESPNOW("ESP NOW Message Received from Dome Controller \n");
        Debug.STATUS("Dome Controller Statisics:  Success: %i  Failuure: %i\n", DCSuccessCounter, DCFailureCounter);

      } else if (incomingSenderID == "DP"){
        domePlateControllerStatus = 1;
        dpkeepAliveAge =millis();
        if (incomingCommandIncluded == 1){
          inputString = incomingCommand;
          stringComplete = true; 
        }
        Debug.STATUS("Dome Plate Status Update \n");
        Debug.ESPNOW("ESP NOW Message Received from Dome Plate Controller \n");
        Debug.STATUS("Dome Plate Statisics:  Success: %i  Failuure: %i\n", DPSuccessCounter, DPFailureCounter);

      } else if (incomingSenderID == "HP"){
        hpControllerStatus = 1;
        hpkeepAliveAge =millis();
        if (incomingCommandIncluded == 1){
          inputString = incomingCommand;
          stringComplete = true; 
        }
        Debug.STATUS("HP Controller Status Update \n");
        Debug.ESPNOW("ESP NOW Message Received from HP Controller \n");
        Debug.STATUS("HP Controller Statisics:  Success: %i  Failuure: %i\n", HPSuccessCounter, HPFailureCounter);

      } else {Debug.ESPNOW("No Valid source identified \n");}

    } else {Debug.ESPNOW("No matching target ID \n");}
    // sendStatusMessage();
}

//////////////////////////////////////////////////////////////
///*****        LoRa Variables       *****///
//////////////////////////////////////////////////////////////////////
  String outgoing;
  String LoRaOutgoing;

  boolean oldState = HIGH;
  boolean newState ;

  int msgCount = 0;            // count of outgoing messages
  byte localAddress = 0x05;     // address of this device
  byte destination = 0x06;      // destination to send to
  long lastSendTime = 0;        // last send time
  int interval = 2000;          // interval between sends

  int LoRaRSSI;


//////////////////////////////////////////////////////////////
///*****        Variables for button      *****///
//////////////////////////////////////////////////////////////////////

  boolean RELAY_STATUS = HIGH;




/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////                                                                                       /////////     
/////////                             Start OF FUNCTIONS                                        /////////
/////////                                                                                       /////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
///*****   ColorWipe Function for Status LED                  *****///
//////////////////////////////////////////////////////////////////////

void colorWipeStatus(String statusled, uint32_t c, int brightness) {
  if(statusled == "ES"){
    ESP_LED.setBrightness(brightness);
    ESP_LED.setPixelColor(0, c);
    ESP_LED.show();
  } else if (statusled == "RS"){
    RELAY_LED.setBrightness(brightness);
    RELAY_LED.setPixelColor(0, c);
    RELAY_LED.show();
  }else if (statusled == "LS"){
    LORA_LED.setBrightness(brightness);
    LORA_LED.setPixelColor(0, c);
    LORA_LED.show();
  }
else{Debug.DBG("No LED was chosen \n");}
  };


////////////////////////////////////////////////////////////////////
/*****    Checks the age of the Status Variables            *****///
////////////////////////////////////////////////////////////////////



void checkAgeofkeepAlive(){    //checks for the variable's age
  if (domeControllerStatus== 1){
    if (millis() - dckeepAliveAge>=keepAliveTimeOut){
      domeControllerStatus = 0;
      Debug.STATUS("Dome Controller Offline\n");
    }
  }
  if (domePlateControllerStatus== 1){
    if (millis()-dpkeepAliveAge>=keepAliveTimeOut){
      domePlateControllerStatus= 0;
      Debug.STATUS("Dome Plate Controller Offline\n");
    }
  }
  if (bodyServoControllerStatus == 1){
    if (millis()-bskeepAliveAge>=keepAliveTimeOut){
      bodyServoControllerStatus = 0;
      Debug.STATUS("Body Servo Controller Offline\n");
    }
  }
  if (bodyControllerStatus == 1){
    if (millis()-bckeepAliveAge>=keepAliveTimeOut){
      bodyControllerStatus = 0;
      Debug.STATUS("Body Controller Offline\n");
    }
  }
  if (bodyLEDControllerStatus == 1){
    if (millis()-blkeepAliveAge>=keepAliveTimeOut){
      bodyLEDControllerStatus=0;
      BL_LDP_Bright =0;
      BL_MAINT_Bright = 0;
      BL_VU_Bright = 0;
      BL_CS_Bright =0;
      BL_vuOffsetInt = 0;
      BL_vuBaselineInt = 0;
      BL_vuOffsetExt = 0;
      BL_vuBaselineExt =0;
      BL_BatteryPercentage = 0;
      BL_BatteryVoltage = 0.0;
      Debug.STATUS("Body LED Controller Offline\n");
    }
  }
  if (droidRemoteStatus == 1){
    if (millis()-drkeepAliveAge>=keepAliveTimeOut){
      droidRemoteStatus = 0;
      colorWipeStatus("LS", red, 20);
      Debug.STATUS("Droid Remote Offline\n");
    }
  }
    if (hpControllerStatus == 1){
    if (millis()-hpkeepAliveAge>=keepAliveTimeOut){
      hpControllerStatus = 0;
      Debug.STATUS("HP Controller Offline\n");
    }
  }
}

void printKeepaliveStatus(){
  if (Debug.debugflag == 0)
  {
    Debug.debugflag = 1;
    Debug.DBG("\n\n------------------------------------\n");
    Debug.DBG("Board Name \t\t| Status:\n");
    Debug.DBG("------------------------------------\n");
    Debug.DBG("Dome Controller: \t| %d\n", domeControllerStatus);
    Debug.DBG("Dome Plate Controller: \t| %d\n", domePlateControllerStatus);
    Debug.DBG("Body Servo Controller: \t| %d\n", bodyServoControllerStatus);
    Debug.DBG("Body  Controller: \t| %d\n", bodyControllerStatus);
    Debug.DBG("Body LED Controller: \t| %d\n", bodyLEDControllerStatus);
    Debug.DBG("HP Controller: \t\t| %d\n", hpControllerStatus);
    Debug.DBG("\n------------------------------------\n");
    Debug.DBG("Body LED Setting \t| Value:\n");
    Debug.DBG("------------------------------------\n");
    Debug.DBG("LDP Bright: \t\t| %i\n", BL_LDP_Bright);
    Debug.DBG("Maintenence Bright: \t| %i\n", BL_MAINT_Bright);
    Debug.DBG("VU Bright: \t\t| %i\n", BL_VU_Bright);
    Debug.DBG("Coin SLots Bright: \t| %i\n", BL_CS_Bright);
    Debug.DBG("vuOffsetInt: \t\t| %i\n", BL_vuOffsetInt);
    Debug.DBG("vuBaselineInt: \t\t| %i\n", BL_vuBaselineInt);
    Debug.DBG("vuOffsetExt: \t\t| %i\n", BL_vuOffsetExt);
    Debug.DBG("vuBaselineExt: \t\t| %i\n", BL_vuBaselineExt);
    Debug.DBG("BL_BatteryVoltage: \t| %f\n", BL_BatteryVoltage);
    Debug.DBG("BL_BatteryPercentage: \t| %i\n", BL_BatteryPercentage);
    Debug.debugflag = 0;
    Local_Command[0]   = '\0';
  } else
  {  
    Debug.DBG("Dome Controller Status: %d\n", domeControllerStatus);
    Debug.DBG("Dome Plate Controller Status: %d\n", domePlateControllerStatus);
    Debug.DBG("Body Servo Controller Status: %d\n", bodyServoControllerStatus);
    Debug.DBG("Body  Controller Status: %d\n", bodyControllerStatus);
    Debug.DBG("Body LED Controller Status: %d\n", bodyLEDControllerStatus);
    Debug.DBG("HP Controller: \t\t| %d\n", hpControllerStatus);
    Local_Command[0]   = '\0';
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                              Communication Functions                                    /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/*//////////////////////////////////////////////////////////////////////
Turns on Wifi and enables OTA.  It will connect to the Kill Switch Remotes's
WiFi network to allow the uploading of sktches(.bin files) via the OTA process. 
It does not produce it's own WiFi network.  Once enables, a reboot is
required to regain ESP-NOW functionality.
*///////////////////////////////////////////////////////////////////////
void connectWiFi(){
  esp_now_deinit();
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(500);

  Serial.println(WiFi.config(local_IP, gateway, subnet) ? "Client IP Configured" : "Failed!");
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &oldLocalMACAddress[0]);
  
  delay(500);
  
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.print("SSID: \t");Serial.println(WiFi.SSID());
  Serial.print("IP Address: \t");Serial.println(WiFi.localIP());
  Serial.print("MAC Address: \t");Serial.println(WiFi.macAddress());
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Please go to http://192.168.4.112/update to upload file");
  });
  
  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

  Local_Command[0]   = '\0';
} ;

/////////////////////////////////////////////////////////
///*****          Serial Event Function          *****///
/////////////////////////////////////////////////////////

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      // stringComplete = true;            // set a flag so the main loop can do something about it.
      enqueueCommand(inputString);
    };
  };
  Debug.SERIAL_EVENT("USB Serial Input: %s \n",inputString);
};

void s1SerialEvent() {
  while (s1Serial.available()) {
    // get the new byte:
    char inChar = (char)s1Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      // stringComplete = true;            // set a flag so the main loop can do something about it.
      enqueueCommand(inputString);

};
  };
  Debug.SERIAL_EVENT("Serial 1 Input: %s \n",inputString);
};

  /////////////////////////////////////////////////////////
  ///*****          Serial Write Function          *****///
  /////////////////////////////////////////////////////////

void writeSerialString(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
      Serial.write(completeString[i]);
  };
};

void writes1Serial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
      s1Serial.write(completeString[i]);
  };
  Debug.SERIAL_EVENT("Writing to Serial 1\n");
};


//////////////////////////////////////////////////////////////////////
///*****             ESP-NOW Functions                        *****///
//////////////////////////////////////////////////////////////////////

void sendESPNOWCommand(String starget, String scomm) {
  bool isBroadcast = (starget == "BR");
  int  targetIdx   = etmBoardIndexFromID(starget.c_str());
  const uint8_t* targetMAC = isBroadcast ? ETM_BROADCAST_MAC
                            : (targetIdx >= 0) ? ETM_BOARD_MACS[targetIdx] : nullptr;

  if (!targetMAC) {
    Debug.ESPNOW("No valid destination for target: %s\n", starget.c_str());
    return;
  }

  uint16_t seqNum    = etmNextSequence();
  bool     hasCommand = scomm.length() > 0;

  espnow_struct_message msg;
  memset(&msg, 0, sizeof(msg));
  strncpy(msg.structPassword, ESPNOWPASSWORD.c_str(),                sizeof(msg.structPassword) - 1);
  strncpy(msg.structSenderID, ETM_BOARD_IDS[ETM_MY_BOARD_INDEX],    sizeof(msg.structSenderID) - 1);
  strncpy(msg.structTargetID, starget.c_str(),                       sizeof(msg.structTargetID) - 1);
  msg.structCommandIncluded = hasCommand;
  msg.structSuccess         = DGSuccessCounter;
  msg.structFailure         = DGFailureCounter;
  strncpy(msg.structCommand, scomm.c_str(), sizeof(msg.structCommand) - 1);
  msg.structPacketType     = PACKET_TYPE_COMMAND;
  msg.structSequenceNumber = seqNum;

  esp_err_t result = esp_now_send(targetMAC, (uint8_t*)&msg, sizeof(msg));
  if (result == ESP_OK) {
    etmAddToPending(scomm.c_str(), starget.c_str(), isBroadcast ? -1 : targetIdx, seqNum);
    Debug.ESPNOW("Sent command: %s to %s\n", scomm.c_str(), starget.c_str());
  } else {
    Debug.ESPNOW("Error sending to %s\n", starget.c_str());
  }
}

//////////////////////////////////////////////////////////////////////
///*****             LoRa Functions                           *****///
//////////////////////////////////////////////////////////////////////
void sendStatusToRemote(){
  if (sendUpdateStatus){
    // 2026-05 (v4): give cfg-chunk drain priority on the LoRa radio.
    // The 1Hz telemetry frame eats ~250ms of LoRa airtime; if it fires
    // while cfg chunks are queued, every chunk after it has to wait an
    // extra ~250ms and Gateway's chunk queue can overflow. Skip telemetry
    // entirely while a config transfer is in progress — status icons
    // will go stale for ~100 seconds during a GET_CONFIG, then refresh
    // when the queue empties.
    if (!cfgChunkQueueEmpty()) return;
    if (millis() - sendStatusMillis >= 1000) {
      sendStatusMillis = millis();
      // sendStatusMessage("Status Update");
      sendStatusMessage();
    }
  }
}


uint32_t LoraPasscode = 12345678;

// ── LoRa packet structs (binary wire format) ───────────────────────────────
// Disambiguated on the Remote by packetSize, so EVERY struct below MUST have a
// size unique vs the others AND byte-for-byte identical to Droid_Remote.ino.
// Sizes (verify at boot): Ack 8, LoRa_Struct(telemetry) 52, ServoInfo 104,
// CfgChunk 116, Diag 152. Fields are right-sized + ordered big→small so the
// compiler emits ~no padding; the human-readable values are reconstructed on
// each endpoint (Remote → JSON, Gateway → debug vars).
//
// 2026-06: slimmed from the old 252-byte frame. Diagnostics (per-board
// success/fail counters + ETM delivery stats) moved OUT of the 1 Hz telemetry
// into the on-demand Diag_LoRa_Struct (pulled only on web "Refresh stats").
// The command ACK moved out of telemetry into the tiny Ack_LoRa_Struct.

// struct_statusFlags bit positions (board-online bitmask). Must match Remote.
#define DGS_GATEWAY     0
#define DGS_BODY        1
#define DGS_BODYLED     2
#define DGS_BODYSERVO   3
#define DGS_DOMEPLATE   4
#define DGS_DOME        5
#define DGS_HP          6
#define DGS_DOMELOGICS  7
#define DGS_REMOTECONN  8

// NOTE: must match Droid_Remote.ino exactly — both sides of the LoRa link
typedef struct LoRa_Struct{
  uint32_t struct_LoraPasscode;          // validation + implicit telemetry tag
  uint16_t struct_statusFlags;           // board-online bitmask (DGS_* bits)
  uint16_t struct_BatteryCentivolts;     // battery volts * 100 (Remote ÷100 → float)
  int16_t  struct_BL_vuOffsetInt;
  int16_t  struct_BL_vuBaselineInt;
  int16_t  struct_BL_vuOffsetExt;
  int16_t  struct_BL_vuBaselineExt;
  uint16_t struct_bcAckSeq;
  uint8_t  struct_BL_LDP_Bright;
  uint8_t  struct_BL_MAINT_Bright;
  uint8_t  struct_BL_VU_Bright;
  uint8_t  struct_BL_CS_Bright;
  uint8_t  struct_BL_BatteryPercentage;
  uint8_t  struct_FunctionSWState;
  bool     struct_bcAckOk;
  char     struct_bcAckMsg[24];
  } LoRa_Struct;

// Tiny dedicated command-ACK packet — replaces piggybacking the ACK on the fat
// telemetry frame, so a command round-trip is ~9 ms not ~395 ms. magic = 0xA5.
typedef struct Ack_LoRa_Struct{
  uint32_t struct_msgAckID;
  uint8_t  struct_magic;
} Ack_LoRa_Struct;

// On-demand diagnostics packet (pulled only when the web GUI clicks Refresh).
// counters[] idx: 0 DGsucc,1 DGfail,2 BCsucc,3 BCfail,4 BSsucc,5 BSfail,
//   6 DPsucc,7 DPfail,8 DCsucc,9 DCfail,10 HPsucc,11 HPfail.
// etm*[] idx: 0=DG,1=BC,2=BS,3=DC,4=DP,5=HP.  magic = 0xD1.
typedef struct Diag_LoRa_Struct{
  uint32_t struct_counters[12];
  uint32_t struct_etmSent[6];
  uint32_t struct_etmAckd[6];
  uint32_t struct_etmRetries[6];
  uint32_t struct_etmFailed[6];
  bool     struct_etmOnline[6];
  uint8_t  struct_magic;
} Diag_LoRa_Struct;

LoRa_Struct      commandstoSendtoRemote;
Ack_LoRa_Struct  ackToSendtoRemote;
Diag_LoRa_Struct diagToSendtoRemote;

// Compile-time wire-format guards. These sizes MUST match Droid_Remote.ino and
// stay mutually unique vs ServoInfo (104) / CfgChunk (116) / CfgChunkAck (12)
// so the Remote's size-based packet routing is unambiguous. If a struct edit
// trips one of these, fix BOTH ends together before flashing.
static_assert(sizeof(Ack_LoRa_Struct)  == 8,   "Ack_LoRa_Struct size drift — sync Gateway+Remote");
static_assert(sizeof(LoRa_Struct)      == 52,  "LoRa_Struct (telemetry) size drift — sync Gateway+Remote");
static_assert(sizeof(Diag_LoRa_Struct) == 152, "Diag_LoRa_Struct size drift — sync Gateway+Remote");

// 2026-05 GET_CONFIG return path. Separate LoRa packet carrying one
// 100-byte chunk of the BC's rcConfigToJSON() reply. The Remote
// dispatches by packetSize: this struct MUST be smaller than
// sizeof(LoRa_Struct) so size-based disambiguation is reliable.
//   struct_LoraPasscode = 12345678 (same as telemetry)
//   struct_magic        = 0xC6     (extra paranoid guard)
//   struct_cfgSeq       = >0 always (BC never emits seq=0 chunks)
// MUST match Droid_Remote.ino's CfgChunk_LoRa_Struct exactly.
typedef struct CfgChunk_LoRa_Struct {
  uint32_t struct_LoraPasscode;
  uint8_t  struct_magic;
  uint16_t struct_cfgSeq;
  uint16_t struct_cfgIdx;
  uint16_t struct_cfgTotal;
  uint8_t  struct_cfgLen;
  char     struct_cfgPayload[100];
} CfgChunk_LoRa_Struct;

// 2026-05 (v7): LoRa-level ACK for cfg chunks. ETM provides reliability
// on the BC<->Gateway ESP-NOW hop, but the Gateway->Remote LoRa hop had
// no equivalent — chunks dropped at the air interface were gone forever,
// causing the GUI's reassembler to see gaps and either abort or produce
// malformed JSON. This struct ports the ETM pattern to LoRa: Remote
// sends one of these back for every chunk it successfully receives;
// Gateway tracks the in-flight chunk and retries on timeout.
//
// MUST match Droid_Remote.ino's CfgChunkAck_LoRa_Struct exactly.
// Size is deliberately small (and distinct from telemetry/chunk sizes)
// so Remote's onReceive can dispatch by packetSize.
typedef struct CfgChunkAck_LoRa_Struct {
  uint32_t struct_LoraPasscode;   // 4  (same passcode as telemetry: 12345678)
  uint8_t  struct_magic;          // 1  0xCA = "Chunk Ack"
  uint8_t  struct_pad;            // 1  padding to align next uint16
  uint16_t struct_cfgSeq;         // 2  identifies the GET_CONFIG transaction
  uint16_t struct_cfgIdx;         // 2  identifies the specific chunk
} CfgChunkAck_LoRa_Struct;
// sizeof ≈ 12 bytes (distinct from 116 chunk and 252 telemetry).

// 2026-05 servo read-back: dedicated LoRa packet carrying one "#LI..."/"#LV..."
// reply from a controller to the web GUI. Kept OFF the telemetry struct (which
// is already 252 of the 255-byte LoRa limit) and disambiguated by packetSize.
// sizeof = 104 bytes (distinct from 12 ack / 116 chunk / 252 telemetry).
// MUST match Droid_Remote.ino's ServoInfo_LoRa_Struct exactly.
typedef struct ServoInfo_LoRa_Struct {
  uint32_t struct_LoraPasscode;   // 12345678 (same passcode as telemetry)
  uint8_t  struct_magic;          // 0xC7 = "serVo info"
  char     struct_payload[96];    // the "#LI..." reply text, NUL-terminated
} ServoInfo_LoRa_Struct;
ServoInfo_LoRa_Struct servoInfoToSendtoRemote;

// Ship the buffered servo reply to the Remote as its own LoRa packet. No ACK:
// if it drops, the web GUI's per-request retry simply asks again. Call only
// from the main loop (LoRa SPI write blocks ~50ms) and not mid cfg-chunk.
void sendServoInfoLoRa() {
  memset(&servoInfoToSendtoRemote, 0, sizeof(servoInfoToSendtoRemote));
  servoInfoToSendtoRemote.struct_LoraPasscode = 12345678;
  servoInfoToSendtoRemote.struct_magic        = 0xC7;
  strncpy(servoInfoToSendtoRemote.struct_payload, servoInfoPayload,
          sizeof(servoInfoToSendtoRemote.struct_payload) - 1);
  LoRa.beginPacket();
  for (unsigned int i = 0; i < sizeof(servoInfoToSendtoRemote); i++) {
    LoRa.write(((byte *) &servoInfoToSendtoRemote)[i]);
  }
  LoRa.endPacket();
}

// 2026-05: queue of pending cfg chunks awaiting LoRa transmission.
//   Previously we called sendCfgChunkLoRa() synchronously from OnDataRecv
//   (the ESP-NOW receive callback context). That callback runs on the
//   WiFi/lwIP system task; doing 50-200 ms blocking LoRa SPI writes there
//   starves the WiFi driver and the Gateway hangs after a dozen chunks
//   (user reported the DG needing a reboot mid-test).
//   The new flow: OnDataRecv enqueues, the main loop calls
//   tickCfgChunkQueue() to drain one chunk per iteration (no more often
//   than CFG_CHUNK_LORA_MIN_GAP_MS).
//   Queue capacity sized to comfortably absorb a full config burst
//   (~78 chunks at 200 ms BC pacing vs. ~100-200 ms LoRa drain).
#define CFG_CHUNK_QUEUE_SIZE      32
// 2026-05 (v7): with LoRa-level ACK in place, the gap is mostly defined
// by ACK round-trip time (which is naturally enforced). 100ms gives
// the Remote a brief idle window after its ACK transmit to re-arm its
// receiver before the next chunk hits the air. Smaller than the
// previous 600ms because we no longer have to PRAY the Remote isn't
// busy — we KNOW it's ready because we got its ACK.
// 2026-06: bandwidth went 125→500 kHz, so a 116-byte chunk's airtime dropped
// ~190→~48 ms and its ACK round-trip is now ~85 ms. The old 100 ms floor is
// now larger than the ACK gate itself, so lower it and let the ACK round-trip
// govern the drain rate. (Must stay below the BC's chunk feed interval so the
// 32-slot queue never overflows — see BC_CFG_CHUNK_INTERVAL_MS.)
#define CFG_CHUNK_LORA_MIN_GAP_MS 45
// 2026-05 (v7): LoRa-level reliability for cfg chunks. ETM-style
// single-chunk-in-flight with timeout-based retry.
// 2026-06: 800→400 ms — the ACK now returns in ~85 ms at 500 kHz, so 400 ms is
// still a comfortable retry window without stalling 800 ms on a genuine drop.
#define CFG_CHUNK_ACK_TIMEOUT_MS  400   // wait this long for Remote ACK
#define CFG_CHUNK_LORA_MAX_RETRY  3     // before giving up on a chunk
struct PendingCfgChunk {
  uint16_t seq;
  uint16_t idx;
  uint16_t total;
  uint8_t  len;
  char     payload[100];
};
volatile PendingCfgChunk cfgChunkQueue[CFG_CHUNK_QUEUE_SIZE];
volatile uint8_t cfgChunkQueueHead = 0;   // next write slot
volatile uint8_t cfgChunkQueueTail = 0;   // next read slot
unsigned long    cfgChunkLastSendMs = 0;
uint32_t         cfgChunkDroppedCount = 0;
// 2026-05 (v7): in-flight chunk tracking for LoRa-level ACK/retry.
// Only one chunk is in flight at a time — simpler than a pending table
// and the ACK latency (~250-400ms total round trip) is small relative to
// the transfer duration. tickCfgChunkQueue() owns this state machine.
volatile bool     cfgChunkInFlight        = false;
volatile bool     cfgChunkAcked           = false;
uint16_t          cfgChunkInFlightSeq     = 0;
uint16_t          cfgChunkInFlightIdx     = 0;
unsigned long     cfgChunkInFlightStartMs = 0;
uint8_t           cfgChunkInFlightRetries = 0;

bool cfgChunkQueueEmpty()  { return cfgChunkQueueHead == cfgChunkQueueTail; }
bool cfgChunkQueueFull()   {
  return (uint8_t)((cfgChunkQueueHead + 1) % CFG_CHUNK_QUEUE_SIZE) == cfgChunkQueueTail;
}

void enqueueCfgChunk(uint16_t seq, uint16_t idx, uint16_t total,
                     uint8_t len, const char *payload) {
  if (cfgChunkQueueFull()) {
    cfgChunkDroppedCount++;
    // 2026-05 (v4): make overflow visible. Was silent before — operator
    // had no way to see when the LoRa drain was falling behind BC's send
    // rate. Each [CFG-DROP] line means BC must re-send this chunk (it
    // won't, currently — see N5 in the E2E review).
    Serial.printf("[CFG-DROP] queue full — dropped seq=%u idx=%u/%u total_dropped=%u\n",
                  seq, idx, total, (unsigned)cfgChunkDroppedCount);
    return;
  }
  volatile PendingCfgChunk *slot = &cfgChunkQueue[cfgChunkQueueHead];
  slot->seq   = seq;
  slot->idx   = idx;
  slot->total = total;
  slot->len   = len > sizeof(slot->payload) ? sizeof(slot->payload) : len;
  for (uint8_t i = 0; i < slot->len; i++) slot->payload[i] = payload[i];
  cfgChunkQueueHead = (cfgChunkQueueHead + 1) % CFG_CHUNK_QUEUE_SIZE;
}

CfgChunk_LoRa_Struct cfgChunkToSendtoRemote;

// Sends a single cfg chunk to the Droid_Remote over LoRa. Called from
// OnDataRecv() immediately on every chunk arrival from the BC so the
// dribble cadence (~50ms / chunk) is preserved end-to-end without
// piggybacking on the slow (1 Hz) periodic telemetry frame.
// 2026-05: NOW CALLED FROM THE MAIN LOOP via tickCfgChunkQueue() — never
// from the ESP-NOW receive callback. The LoRa SPI write loop below blocks
// for 50-200 ms and would starve the WiFi driver if called from the
// ESP-NOW system task.
void sendCfgChunkLoRa() {
  cfgChunkToSendtoRemote.struct_LoraPasscode = LoraPasscode;
  cfgChunkToSendtoRemote.struct_magic        = 0xC6;
  cfgChunkToSendtoRemote.struct_cfgSeq       = BC_cfgSeq;
  cfgChunkToSendtoRemote.struct_cfgIdx       = BC_cfgIdx;
  cfgChunkToSendtoRemote.struct_cfgTotal     = BC_cfgTotal;
  uint8_t len = BC_cfgLen;
  if (len > sizeof(cfgChunkToSendtoRemote.struct_cfgPayload)) {
    len = sizeof(cfgChunkToSendtoRemote.struct_cfgPayload);
  }
  cfgChunkToSendtoRemote.struct_cfgLen = len;
  memset(cfgChunkToSendtoRemote.struct_cfgPayload, 0,
         sizeof(cfgChunkToSendtoRemote.struct_cfgPayload));
  memcpy(cfgChunkToSendtoRemote.struct_cfgPayload, BC_cfgPayload, len);
  // 2026-05 unconditional logs so we can see if LoRa transmits are
  // actually happening even when Debug.LORA is off.
  unsigned long tStart = millis();
  Serial.printf("[CFG-LoRa] BEGIN tx seq=%u idx=%u/%u len=%u size=%u\n",
                BC_cfgSeq, BC_cfgIdx, BC_cfgTotal, BC_cfgLen,
                (unsigned)sizeof(cfgChunkToSendtoRemote));
  LoRa.beginPacket();
  for (unsigned int i = 0; i < sizeof(cfgChunkToSendtoRemote); i++) {
    LoRa.write(((byte *) &cfgChunkToSendtoRemote)[i]);
  }
  int ok = LoRa.endPacket();
  Serial.printf("[CFG-LoRa] END   tx seq=%u idx=%u/%u endPacket=%d duration=%lums\n",
                BC_cfgSeq, BC_cfgIdx, BC_cfgTotal, ok, millis() - tStart);
}

// 2026-05 (v7): LoRa-level reliable delivery for cfg chunks.
//
// Single-chunk-in-flight pattern (ETM-style, applied to LoRa):
//   1. Peek at head of queue, populate BC_cfg* globals, fire LoRa send.
//   2. Set cfgChunkInFlight=true, record seq/idx, start timer.
//   3. Wait for Remote's CfgChunkAck_LoRa_Struct response with matching
//      seq+idx — set by `onReceive` in the size-based dispatch.
//   4. On ACK: dequeue the slot, clear in-flight, ready for next chunk.
//   5. On timeout: re-send same chunk (up to CFG_CHUNK_LORA_MAX_RETRY).
//   6. On retry exhaustion: log, dequeue, give up on that chunk (the
//      GUI's reassembler will see the gap; with the v6 soft-tolerate
//      fix it limps along and attempts JSON parse anyway).
//
// Why single-chunk: ACK round trip is ~250-400ms (chunk airtime + ACK
// airtime + main-loop latency). Pipelining multiple in-flight chunks
// would require a pending table and complicate the retry logic. Single
// in-flight at ~1 chunk/sec wall-clock easily delivers all 98 chunks
// reliably in ~100 seconds.
void tickCfgChunkQueue() {
  unsigned long now = millis();

  // ── Step 1: handle an in-flight chunk first (ACK or timeout) ─────────
  if (cfgChunkInFlight) {
    if (cfgChunkAcked) {
      // Remote ACK'd — dequeue and move on.
      Serial.printf("[CFG-LoRa] ACK seq=%u idx=%u (retries=%u)\n",
                    cfgChunkInFlightSeq, cfgChunkInFlightIdx,
                    cfgChunkInFlightRetries);
      cfgChunkQueueTail = (cfgChunkQueueTail + 1) % CFG_CHUNK_QUEUE_SIZE;
      cfgChunkInFlight        = false;
      cfgChunkAcked           = false;
      cfgChunkInFlightRetries = 0;
      cfgChunkLastSendMs      = now;
      return;
    }
    if (now - cfgChunkInFlightStartMs > CFG_CHUNK_ACK_TIMEOUT_MS) {
      cfgChunkInFlightRetries++;
      if (cfgChunkInFlightRetries > CFG_CHUNK_LORA_MAX_RETRY) {
        // Give up on this chunk. Move on to the next.
        Serial.printf("[CFG-LoRa] GIVE UP seq=%u idx=%u after %u retries\n",
                      cfgChunkInFlightSeq, cfgChunkInFlightIdx,
                      (unsigned)CFG_CHUNK_LORA_MAX_RETRY);
        cfgChunkQueueTail = (cfgChunkQueueTail + 1) % CFG_CHUNK_QUEUE_SIZE;
        cfgChunkInFlight        = false;
        cfgChunkAcked           = false;
        cfgChunkInFlightRetries = 0;
        cfgChunkLastSendMs      = now;
        return;
      }
      // Retry: re-send the same chunk (still at head of queue).
      Serial.printf("[CFG-LoRa] TIMEOUT seq=%u idx=%u — retry %u/%u\n",
                    cfgChunkInFlightSeq, cfgChunkInFlightIdx,
                    cfgChunkInFlightRetries, (unsigned)CFG_CHUNK_LORA_MAX_RETRY);
      volatile PendingCfgChunk *slot = &cfgChunkQueue[cfgChunkQueueTail];
      BC_cfgSeq   = slot->seq;
      BC_cfgIdx   = slot->idx;
      BC_cfgTotal = slot->total;
      BC_cfgLen   = slot->len;
      memset(BC_cfgPayload, 0, sizeof(BC_cfgPayload));
      for (uint8_t i = 0; i < slot->len; i++) BC_cfgPayload[i] = slot->payload[i];
      sendCfgChunkLoRa();
      cfgChunkInFlightStartMs = millis();
      return;
    }
    return;  // Still waiting for ACK or timeout — keep waiting
  }

  // ── Step 2: no in-flight chunk — try to send the next one ────────────
  if (cfgChunkQueueEmpty()) return;
  if (now - cfgChunkLastSendMs < CFG_CHUNK_LORA_MIN_GAP_MS) return;

  // Peek at the head of the queue (do NOT dequeue until ACK arrives).
  volatile PendingCfgChunk *slot = &cfgChunkQueue[cfgChunkQueueTail];
  BC_cfgSeq   = slot->seq;
  BC_cfgIdx   = slot->idx;
  BC_cfgTotal = slot->total;
  BC_cfgLen   = slot->len;
  memset(BC_cfgPayload, 0, sizeof(BC_cfgPayload));
  for (uint8_t i = 0; i < slot->len; i++) BC_cfgPayload[i] = slot->payload[i];

  cfgChunkInFlight        = true;
  cfgChunkAcked           = false;
  cfgChunkInFlightSeq     = slot->seq;
  cfgChunkInFlightIdx     = slot->idx;
  cfgChunkInFlightStartMs = millis();
  cfgChunkInFlightRetries = 0;

  sendCfgChunkLoRa();
}

// 2026-06: populate the slim binary telemetry frame. Diagnostics (counters +
// ETM stats) and the command ACK no longer ride here — they have their own
// packets (Diag_LoRa_Struct / Ack_LoRa_Struct). Board-online flags are packed
// into one bitmask; brightness/%/funcSW are uint8; battery is centivolts.
void fillTelemetryStruct(){
    commandstoSendtoRemote.struct_LoraPasscode = LoraPasscode;
    uint16_t f = 0;
    if (droidGatewayStatus)         f |= (1 << DGS_GATEWAY);
    if (bodyControllerStatus)       f |= (1 << DGS_BODY);
    if (bodyLEDControllerStatus)    f |= (1 << DGS_BODYLED);
    if (bodyServoControllerStatus)  f |= (1 << DGS_BODYSERVO);
    if (domePlateControllerStatus)  f |= (1 << DGS_DOMEPLATE);
    if (domeControllerStatus)       f |= (1 << DGS_DOME);
    if (hpControllerStatus)         f |= (1 << DGS_HP);
    if (domeLogicsControllerStatus) f |= (1 << DGS_DOMELOGICS);
    if (remoteConnected)            f |= (1 << DGS_REMOTECONN);
    commandstoSendtoRemote.struct_statusFlags = f;
    // battery volts → centivolts (Remote reconstructs the float as /100.0)
    long cv = (long)(BL_BatteryVoltage * 100.0 + 0.5);
    if (cv < 0)     cv = 0;
    if (cv > 65535) cv = 65535;
    commandstoSendtoRemote.struct_BatteryCentivolts = (uint16_t)cv;
    // NOTE: struct_BL_vuBaselineInt intentionally fed from BL_vuOffsetInt to
    // preserve the original firmware's (pre-existing) assignment behaviour.
    commandstoSendtoRemote.struct_BL_vuOffsetInt   = (int16_t)BL_vuOffsetInt;
    commandstoSendtoRemote.struct_BL_vuBaselineInt = (int16_t)BL_vuOffsetInt;
    commandstoSendtoRemote.struct_BL_vuOffsetExt   = (int16_t)BL_vuOffsetExt;
    commandstoSendtoRemote.struct_BL_vuBaselineExt = (int16_t)BL_vuBaselineExt;
    commandstoSendtoRemote.struct_bcAckSeq = BC_bcAckSeq;
    commandstoSendtoRemote.struct_BL_LDP_Bright       = (uint8_t)BL_LDP_Bright;
    commandstoSendtoRemote.struct_BL_MAINT_Bright     = (uint8_t)BL_MAINT_Bright;
    commandstoSendtoRemote.struct_BL_VU_Bright        = (uint8_t)BL_VU_Bright;
    commandstoSendtoRemote.struct_BL_CS_Bright        = (uint8_t)BL_CS_Bright;
    commandstoSendtoRemote.struct_BL_BatteryPercentage= (uint8_t)BL_BatteryPercentage;
    commandstoSendtoRemote.struct_FunctionSWState     = (uint8_t)FunctionSWState;
    commandstoSendtoRemote.struct_bcAckOk  = BC_bcAckOk;
    strncpy(commandstoSendtoRemote.struct_bcAckMsg, BC_bcAckMsg, sizeof(commandstoSendtoRemote.struct_bcAckMsg) - 1);
    commandstoSendtoRemote.struct_bcAckMsg[sizeof(commandstoSendtoRemote.struct_bcAckMsg) - 1] = '\0';
}

void setupLoRaSendStruct(){ fillTelemetryStruct(); }
void setupLoRaSendStructNow(){ fillTelemetryStruct(); }

// Fill the on-demand diagnostics packet from the live counter vars + ETM table.
// Mirrors the DG-index special-case the old telemetry used (DG never sends to
// itself, so its stats stay 0 and its dot reflects whether BC has seen DG).
void fillDiagStruct(){
  diagToSendtoRemote.struct_magic = 0xD1;
  diagToSendtoRemote.struct_counters[0]  = DGSuccessCounter;
  diagToSendtoRemote.struct_counters[1]  = DGFailureCounter;
  diagToSendtoRemote.struct_counters[2]  = BCSuccessCounter;
  diagToSendtoRemote.struct_counters[3]  = BCFailureCounter;
  diagToSendtoRemote.struct_counters[4]  = BSSuccessCounter;
  diagToSendtoRemote.struct_counters[5]  = BSFailureCounter;
  diagToSendtoRemote.struct_counters[6]  = DPSuccessCounter;
  diagToSendtoRemote.struct_counters[7]  = DPFailureCounter;
  diagToSendtoRemote.struct_counters[8]  = DCSuccessCounter;
  diagToSendtoRemote.struct_counters[9]  = DCFailureCounter;
  diagToSendtoRemote.struct_counters[10] = HPSuccessCounter;
  diagToSendtoRemote.struct_counters[11] = HPFailureCounter;
  for (int i = 0; i < ETM_NUM_BOARDS && i < 6; i++) {
    if (i == ETM_BOARD_DG) {
      diagToSendtoRemote.struct_etmOnline[i]  = BC_etmBoardOnline[ETM_BOARD_DG];
      diagToSendtoRemote.struct_etmSent[i]    = 0;
      diagToSendtoRemote.struct_etmAckd[i]    = 0;
      diagToSendtoRemote.struct_etmRetries[i] = 0;
      diagToSendtoRemote.struct_etmFailed[i]  = 0;
    } else {
      diagToSendtoRemote.struct_etmOnline[i]  = etmBoardTable[i].online;
      diagToSendtoRemote.struct_etmSent[i]    = etmBoardTable[i].messagesSent;
      diagToSendtoRemote.struct_etmAckd[i]    = etmBoardTable[i].messagesAckd;
      diagToSendtoRemote.struct_etmRetries[i] = etmBoardTable[i].totalRetries;
      diagToSendtoRemote.struct_etmFailed[i]  = etmBoardTable[i].totalFailed;
    }
  }
}


void sendStatusMessageNow(){
  setupLoRaSendStructNow();
  LoRa.beginPacket();
  for (unsigned int i = 0; i < sizeof(commandstoSendtoRemote);i++) {
    LoRa.write(((byte *) &commandstoSendtoRemote)[i]);
  }
  LoRa.endPacket();
}

void sendStatusMessage(){
  setupLoRaSendStruct();
  LoRa.beginPacket();
  for (unsigned int i = 0; i < sizeof(commandstoSendtoRemote);i++) {
    LoRa.write(((byte *) &commandstoSendtoRemote)[i]);
  }
  LoRa.endPacket();
}

// 2026-06: on-demand diagnostics frame (counters + ETM stats). Sent only when
// the web GUI requests it via #L09 — never on the 1 Hz cadence.
void sendDiagMessage(){
  fillDiagStruct();
  LoRa.beginPacket();
  for (unsigned int i = 0; i < sizeof(diagToSendtoRemote); i++) {
    LoRa.write(((byte *) &diagToSendtoRemote)[i]);
  }
  LoRa.endPacket();
}

// 2026-06: command ACK is now its own tiny 8-byte packet instead of the whole
// 252-byte telemetry frame — a command round-trip drops from ~395 ms to ~9 ms,
// and ACK delivery no longer collides with / waits on the 1 Hz telemetry.
void sendACK(int msgAckID){
  ackToSendtoRemote.struct_msgAckID = (uint32_t)msgAckID;
  ackToSendtoRemote.struct_magic    = 0xA5;
  LoRa.beginPacket();
  for (unsigned int i = 0; i < sizeof(ackToSendtoRemote); i++) {
    LoRa.write(((byte *) &ackToSendtoRemote)[i]);
  }
  LoRa.endPacket();
}



void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // 2026-05 (v7): LoRa-level cfg-chunk ACK from Remote. Dispatched by
  // exact packetSize so it doesn't collide with the text-based Remote→
  // Gateway command protocol (which starts with recipient/sender/msgId/
  // length header bytes — significantly larger packet). Returns early
  // without falling through to the text parser.
  if (packetSize == (int)sizeof(CfgChunkAck_LoRa_Struct)) {
    CfgChunkAck_LoRa_Struct ack;
    for (int i = 0; i < packetSize; i++) {
      ((byte *) &ack)[i] = LoRa.read();
    }
    if (ack.struct_LoraPasscode != 12345678 || ack.struct_magic != 0xCA) {
      Debug.LORA("[CFG-ACK] bad passcode/magic — dropping\n");
      return;
    }
    // Only set cfgChunkAcked if it matches the chunk we're waiting for.
    if (cfgChunkInFlight &&
        ack.struct_cfgSeq == cfgChunkInFlightSeq &&
        ack.struct_cfgIdx == cfgChunkInFlightIdx) {
      cfgChunkAcked = true;
    } else {
      Debug.LORA("[CFG-ACK] mismatch — got seq=%u idx=%u, in-flight seq=%u idx=%u\n",
                 (unsigned)ack.struct_cfgSeq, (unsigned)ack.struct_cfgIdx,
                 (unsigned)cfgChunkInFlightSeq, (unsigned)cfgChunkInFlightIdx);
    }
    drkeepAliveAge = millis();
    return;
  }

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  
  String incoming = "";
  droidRemoteStatus = 1;
  drkeepAliveAge =millis();
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    if (Debug.debugflag_lora == 1){Serial.println("error: message length does not match length");}
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
      if (Debug.debugflag_lora == 1){Serial.println("This message is not for me.");}
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  if (Debug.debugflag_lora == 1){
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  }
  // Deduplication — if this is a retry of an already-processed command, ACK it but don't re-run it
  if (incomingMsgId == lastProcessedMsgId) {
    Debug.LORA("Duplicate LoRa msg ID %d — deferring ACK without re-processing\n", incomingMsgId);
    // 2026-05 (v4): defer LoRa ACK to main loop so it doesn't compete
    // with tickCfgChunkQueue for the LoRa radio during config receive.
    pendingLoRaAck   = true;
    pendingLoRaAckId = incomingMsgId;
    return;
  }
  lastProcessedMsgId = incomingMsgId;
  parseStrings(incoming);
  // 2026-05 live-move fast lane: don't ACK ephemeral servo "#LM" jog commands
  // (":E<board>#LM..."). The ACK is a full telemetry packet (~380ms airtime),
  // so ACKing a calibration drag would saturate the half-duplex channel. The
  // Remote sends these fire-and-forget and doesn't expect an ACK. All other
  // commands still get the normal deferred ACK.
  bool isLiveMove = (incoming.length() >= 7 && incoming[0] == ':' &&
                     (incoming[1] == 'E' || incoming[1] == 'e') && incoming[4] == '#' &&
                     (incoming[5] == 'L' || incoming[5] == 'l') && (incoming[6] == 'M' || incoming[6] == 'm'));
  if (!isLiveMove) {
    // 2026-05 (v4): defer LoRa ACK to main loop (see comment above).
    pendingLoRaAck   = true;
    pendingLoRaAckId = incomingMsgId;
  }
  if(LoRa.packetRssi() > -50 && LoRa.packetRssi() < 10){
    colorWipeStatus("LS", green, 10);
  }else if (LoRa.packetRssi() > -100 && LoRa.packetRssi()  <= -50){
    colorWipeStatus("LS", yellow, 10);
  } else{ colorWipeStatus("LS", red, 10);}

  // inputString = incoming;
  // stringComplete = true; 
      // sendMessage("Message Revieved");
};
// String LoRaStringReceived = "";
String queuecommand ="";
void parseStrings(String data){
    // 2026-05 fix: previously tokenized LoRa payload on '.' which broke
    // any payload containing a period — including the chunked-JSON BC
    // config push when a note string had a literal "." in it, and any
    // delay-style float values. The Remote sends exactly ONE command per
    // LoRa packet, so no in-payload splitting is needed.
    //
    // Also dropped the toCharArray(buf, 100) bottleneck: the buffer was
    // capped at 100 bytes which silently truncated anything close to the
    // ESP-NOW limit. Forwarding the Arduino String straight through has
    // no such cap.
    Serial.println(data);
    enqueueCommand(data);
  }




///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                             Miscellaneous Functions                                           /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


void MainRelayOn(){
  RELAY_STATUS = HIGH;
  digitalWrite(RELAY_CONTROL, RELAY_STATUS);
          colorWipeStatus("RS", green, 10);

  Serial.print("Mode is: ");Serial.println(RELAY_STATUS);
  Local_Command[0]   = '\0';
}

void MainRelayOff(){
  RELAY_STATUS = LOW;
  digitalWrite(RELAY_CONTROL, RELAY_STATUS);
          colorWipeStatus("RS", red, 10);

  Serial.print("Mode is: ");Serial.println(RELAY_STATUS);
  Local_Command[0]   = '\0';
}

void checkButton(){
  newState = digitalRead(RELAY_BUTTON);

  if((newState == LOW) && (oldState == HIGH)) {
    newState = digitalRead(RELAY_BUTTON);
    if(newState == LOW) {      // Yes, still low
      if (RELAY_STATUS == LOW) {
        colorWipeStatus("RS", red, 10);
        MainRelayOff();
      }
      if (RELAY_STATUS == HIGH){
        colorWipeStatus("RS", green, 10);
        MainRelayOn();
      }
      RELAY_STATUS = !RELAY_STATUS;
    }
  } 
}

// <-- code from Mimir for queueing incoming commands from GET params-->
////////////////////////////////////////////////////

#define MAX_QUEUE_DEPTH 5

////////////////////////////////////////////////////

template<class T, int maxitems>
class Queue {
  private:
    int _front = 0, _back = 0, _count = 0;
    T _data[maxitems + 1];
    int _maxitems = maxitems;
  public:
    inline int count() { return _count; }
    inline int front() { return _front; }
    inline int back()  { return _back;  }

    void push(const T &item) {
      if(_count < _maxitems) { // Drops out when full
        _data[_back++]=item;
        ++_count;
        // Check wrap around
        if (_back > _maxitems)
          _back -= (_maxitems + 1);
      }
    }

    T peek() {
      return (_count <= 0) ? T() : _data[_front];
    }

    T pop() {
      if (_count <= 0)
        return T(); // Returns empty

      T result = _data[_front];
      _front++;
      --_count;
      // Check wrap around
      if (_front > _maxitems) 
        _front -= (_maxitems + 1);
      return result; 
    }

    void clear() {
      _front = _back;
      _count = 0;
    }
};

template <int maxitems = MAX_QUEUE_DEPTH>
using CommandQueue = Queue<String, maxitems>;

////////////////////////////////////////////////////


CommandQueue<> commandQueue;

bool havePendingCommands()
{
  return (commandQueue.count() > 0);
}

String getNextCommand()
{
  return commandQueue.pop();
}

void enqueueCommand(String command)
{
  commandQueue.push(command);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////                                                                                       /////////     
/////////                             END OF FUNCTIONS                                          /////////
/////////                                                                                       /////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  while (!Serial);
  s1Serial.begin(SERIAL1_BAUD_RATE,SERIAL_8N1,SERIAL1_RX_PIN,SERIAL1_TX_PIN);

  Serial.println("\n\n----------------------------------------");
  Serial.print("Booting up the ");Serial.println(HOSTNAME);
  Serial.println("----------------------------------------");
  // 2026-05 diagnostic: print struct sizes at boot. The Gateway dispatches
  // incoming ESP-NOW packets from the BC by len — MUST match the BC's
  // sizes exactly (compare to the matching print in Body_Controller_ESP32_GUI.ino).
  Serial.printf("[DG] sizeof(bodyControllerStatus_struct_message) = %u bytes\n",
                (unsigned)sizeof(bodyControllerStatus_struct_message));
  Serial.printf("[DG] sizeof(LoRa_Struct)                         = %u bytes\n",
                (unsigned)sizeof(LoRa_Struct));
  Serial.printf("[DG] sizeof(Ack_LoRa_Struct)                     = %u bytes\n",
                (unsigned)sizeof(Ack_LoRa_Struct));
  Serial.printf("[DG] sizeof(Diag_LoRa_Struct)                    = %u bytes\n",
                (unsigned)sizeof(Diag_LoRa_Struct));
  Serial.printf("[DG] sizeof(ServoInfo_LoRa_Struct)               = %u bytes\n",
                (unsigned)sizeof(ServoInfo_LoRa_Struct));
  Serial.printf("[DG] sizeof(CfgChunk_LoRa_Struct)                = %u bytes\n",
                (unsigned)sizeof(CfgChunk_LoRa_Struct));
  
  //Button for relay setup
  pinMode(RELAY_BUTTON, INPUT);
  pinMode(RELAY_CONTROL, OUTPUT);
  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

// initialize WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, ETM_BOARD_MACS[ETM_MY_BOARD_INDEX]);
  Serial.print("Local STA MAC address = ");
  Serial.println(WiFi.macAddress());

  //Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add all peers from ETM MAC table
  memcpy(peerInfo.peer_addr, ETM_BROADCAST_MAC, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  }
  for (int i = 0; i < ETM_NUM_BOARDS; i++) {
    if (i == ETM_MY_BOARD_INDEX) continue;
    memcpy(peerInfo.peer_addr, ETM_BOARD_MACS[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.printf("Failed to add ESP-NOW peer %d\n", i);
      return;
    }
  }


  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize ETM
  etmInit(ESPNOWPASSWORD.c_str(), ETM_MY_BOARD_INDEX);

ESP_LED.begin();
ESP_LED.show();
colorWipeStatus("ES", blue, 10);

RELAY_LED.begin();
RELAY_LED.show();
colorWipeStatus("RS", green, 10);

LORA_LED.begin();
LORA_LED.show();
colorWipeStatus("LS", red, 10); 

  //LoRa Setup
SPI.begin(SCK_LORA, MISO_LORA, MOSI_LORA, NSS_LORA);
LoRa.setPins(NSS_LORA, RESET_LORA, DIO_LORA);

  // if (!LoRa.begin(915E6,true)) {
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (true);
  }
  // 2026-06: 125 kHz → 500 kHz signal bandwidth = ~4× faster airtime per frame.
  // MUST match Droid_Remote.ino. Safe at the droid's ≤50 yd operating range
  // (huge link margin). If the link is ever flaky here, drop BOTH ends to 250E3.
  LoRa.setSignalBandwidth(500E3);
}

void loop() {
  etmProcess();
  checkAgeofkeepAlive();
  sendStatusToRemote();
  checkButton();
  onReceive(LoRa.parsePacket());
  // 2026-05: drain pending cfg chunks (from BC's GET_CONFIG response).
  // OnDataRecv enqueues each arriving chunk; this dequeues + LoRa-sends
  // one per call, throttled to CFG_CHUNK_LORA_MIN_GAP_MS. Must run in
  // main-loop context (not ESP-NOW callback) because the LoRa SPI write
  // blocks 50-200ms which starves the WiFi driver if done in callback.
  tickCfgChunkQueue();
  // 2026-05 (v4): drain pending deferred LoRa ACKs. The onReceive path
  // used to fire sendACK() inline — a ~250ms LoRa transmit — which
  // starved tickCfgChunkQueue. Now we defer ACKs and only send them
  // when the cfg chunk queue is empty AND the LoRa-gap throttle has
  // elapsed, so cfg chunks get LoRa-radio priority during a config
  // transfer.
  if (pendingLoRaAck && cfgChunkQueueEmpty() &&
      (millis() - cfgChunkLastSendMs >= CFG_CHUNK_LORA_MIN_GAP_MS)) {
    int ackId = pendingLoRaAckId;
    pendingLoRaAck = false;
    sendACK(ackId);
  }
  // 2026-05 servo read-back: ship a buffered controller reply to the Remote as
  // its own LoRa packet, but only when no cfg-chunk transfer is using the radio
  // (cfg config-pull gets priority) and the LoRa gap has elapsed.
  if (servoInfoPending && !cfgChunkInFlight && cfgChunkQueueEmpty() &&
      (millis() - cfgChunkLastSendMs >= CFG_CHUNK_LORA_MIN_GAP_MS)) {
    servoInfoPending = false;
    sendServoInfoLoRa();
    cfgChunkLastSendMs = millis();   // share the LoRa-gap throttle with cfg chunks
  }
  yield();
  oldState = newState;

  if (millis() - MLMillis >= mainLoopDelayVar){
    MLMillis = millis();
    if(startUp) {
      startUp = false;
      Serial.println("Startup completed, now running loop");
    }

  if(Serial.available()){serialEvent();}

  // if (stringComplete) {autoComplete=false;}
  // if (stringComplete || autoComplete) {
  //   if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
  //   else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
  if (havePendingCommands()) {autoComplete=false;}
  if (havePendingCommands() || autoComplete) {
    if(havePendingCommands()) {inputString = getNextCommand(); inputString.toCharArray(inputBuffer, 100);inputString="";}
    else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}

      if (inputBuffer[0] == '#'){
        if (
            inputBuffer[1]=='D' ||          // Command for debugging
            inputBuffer[1]=='d' ||          // Command for debugging
            inputBuffer[1]=='L' ||          // Command designator for internal functions
            inputBuffer[1]=='l' ||          // Command designator for internal functions
            inputBuffer[1]=='E' ||          // Command designator for storing EEPROM data
            inputBuffer[1]=='e'           // Command designator for storing EEPROM data
          ){commandLength = strlen(inputBuffer); 
            if (inputBuffer[1]=='D' || inputBuffer[1]=='d'){
              debugInputIdentifier = "";                            // flush the string
              for (int i=2; i<=commandLength-2; i++){
                char inCharRead = inputBuffer[i];
                debugInputIdentifier += inCharRead;                   // add it to the inputString:
              }
              debugInputIdentifier.toUpperCase();
              if (debugInputIdentifier == "ETM") etmToggleDebug();
              else Debug.toggle(debugInputIdentifier);
              debugInputIdentifier = "";                             // flush the string
              } else if (inputBuffer[1]=='L' || inputBuffer[1]=='l') {
                localCommandFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');
                Local_Command[0]   = '\0';                                                            // Flushes Array
                Local_Command[0] = localCommandFunction;
              Debug.LOOP("Entered the Local Command Structure /n");
              } else if (inputBuffer[1] == 'E' || inputBuffer[1] == 'e'){
                Debug.LOOP("EEPROM configuration selected /n");
                // need to actually add the code to implement this.

              } else {Debug.LOOP("No valid command entered /n");}
              
          }
              if(Local_Command[0]){
                switch (Local_Command[0]){
                  case 1: Serial.println(HOSTNAME);
                      Local_Command[0]   = '\0';                                                            break;
                  case 2: Serial.println("Resetting the ESP in 3 Seconds");
                        //  DelayCall::schedule([] {ESP.restart();}, 3000);
                        ESP.restart();
                        Local_Command[0]   = '\0';                                                          break;
                  case 3: connectWiFi();                                                                    break;
                  case 4: break;  //reserved for future use
                  case 5: MainRelayOn();                                                                    break;  //reserved for future use
                  case 6: MainRelayOff();                                                                   break;  //reserved for future use
                  case 7: sendStatusMessage();                                                              break;  //reserved for future use
                  case 8: printKeepaliveStatus();                                                           break;  //reserved for future use
                  case 9: sendDiagMessage();                                                                break;  // #L09 — web GUI "Refresh stats": send on-demand Diag packet

                }
              }

        }else if (inputBuffer[0] == ':'){
     
          if(   inputBuffer[1]=='E'     ||        // Command for Sending ESP-NOW Messages
                inputBuffer[1]=='e'     ||        // Command for Sending ESP-NOW Messages
                inputBuffer[1]=='S'     ||        // Command for sending Serial Strings out Serial ports
                inputBuffer[1]=='s'               // Command for sending Serial Strings out Serial ports

            ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.
              Debug.DBG("Command: %s with a length of %d \n", inputBuffer, commandLength);

              if(commandLength >= 3) {
                if(inputBuffer[1]=='E' || inputBuffer[1]=='e') {

                  // 2026-05 fix: was `i<=commandLength` which read one
                  // past strlen() — that index holds the NUL terminator.
                  // Appending '\0' into an Arduino String corrupts the
                  // internal length tracking and silently truncates
                  // downstream substring() calls. Now stops at the last
                  // actual character.
                  for (int i=2; i<commandLength; i++){
                    char inCharRead = inputBuffer[i];
                    ESPNOWStringCommand += inCharRead;                   // add it to the inputString:
                  }
                  Debug.LOOP("\nFull Command Recieved: %s \n",ESPNOWStringCommand.c_str());
                  ESPNOWTarget = ESPNOWStringCommand.substring(0,2);
                  Debug.LOOP("ESP NOW Target: %s\n", ESPNOWTarget.c_str());
                  // substring(2) → from index 2 to end; no need to
                  // compute commandLength+1 (which used to overflow by
                  // one matching the loop bug above).
                  ESPNOWSubStringCommand = ESPNOWStringCommand.substring(2);
                  Debug.LOOP("Command to Forward: %s\n", ESPNOWSubStringCommand.c_str());
                  sendESPNOWCommand(ESPNOWTarget, ESPNOWSubStringCommand);
                  // reset ESP-NOW Variables
                  ESPNOWStringCommand = "";
                  ESPNOWSubStringCommand = "";
                  ESPNOWTarget = "";
                  }
                  if(inputBuffer[1]=='S' || inputBuffer[1]=='s') {
                    for (int i=2; i<commandLength-1;i++ ){
                      char inCharRead = inputBuffer[i];
                      serialStringCommand += inCharRead;  // add it to the inputString:
                    }
                    serialPort = serialStringCommand.substring(0,2);
                    serialSubStringCommand = serialStringCommand.substring(2,commandLength);
                    Debug.LOOP("Serial Command: %s to Serial Port: %s\n", serialSubStringCommand.c_str(), serialPort);                
                    if (serialPort == "SH"){
                      writes1Serial(serialSubStringCommand);
                    } else{
                      Debug.LOOP("Wrong Serial Port Identified /n");
                    } 

                  } 
              }
            }
        }

     ///***  Clear States and Reset for next command.  ***///
        stringComplete =false;
        autoComplete = false;
        inputBuffer[0] = '\0';
        inputBuffer[1] = '\0';



    // reset Local ESP Command Variables
        int localCommandFunction;

        String  ESPNOWStringCommand = "";
        String ESPNOWSubStringCommand = "";
        String ESPNOWTarget = "";
   }
  
   if(isStartUp) {
     isStartUp = false;
     delay(500);
   }
 }
// LoRa.receive();
}




