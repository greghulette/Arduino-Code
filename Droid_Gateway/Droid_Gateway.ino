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

// NOTE: must match Body_Controller_ESP32.ino exactly — both sides of the ESP-NOW link
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

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  colorWipeStatus("ES", orange, 255);

  int senderIdx = etmBoardIndexFromMAC(esp_now_info->src_addr);
  Debug.ESPNOW("Received ESP-NOW packet from board index %d, len %d\n", senderIdx, len);

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
      etmHandleHeartbeat(ETM_BOARD_BC);
      processESPNOWIncomingMessage();
    }
    colorWipeStatus("ES", blue, 10);
    return;
  }

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
    if (millis() - sendStatusMillis >= 1000) {
      sendStatusMillis = millis();
      // sendStatusMessage("Status Update");
      sendStatusMessage();
    } 
  }
}


uint32_t LoraPasscode = 12345678;

// NOTE: must match Droid_Remote.ino exactly — both sides of the LoRa link
typedef struct LoRa_Struct{
  uint32_t struct_LoraPasscode;
  bool struct_incomingMsgAck;
  uint32_t struct_msgAckID;
  bool struct_droidGatewayStatus;
  bool struct_bodyControllerStatus ;
  bool struct_bodyLEDControllerStatus;
  bool struct_bodyServoControllerStatus;
  bool struct_domePlateControllerStatus;
  bool struct_domeControllerStatus ;
  bool struct_droidRemoteStatus;
  bool struct_hpControllerStatus;
  bool struct_domeLogicsControllerStatus;
  int struct_BL_LDP_Bright;
  int struct_BL_MAINT_Bright;
  int struct_BL_VU_Bright;
  int struct_BL_CS_Bright;
  int struct_BL_vuOffsetInt;
  int struct_BL_vuBaselineInt;
  int struct_BL_vuOffsetExt;
  int struct_BL_vuBaselineExt;
  float struct_BL_BatteryVoltage;
  int struct_BL_BatteryPercentage;
  int struct_FunctionSWState;
  bool struct_remoteConnected;
  uint32_t struct_DGSuccessCounter;
  uint32_t struct_DGFailureCounter;
  uint32_t struct_BSSuccessCounter;
  uint32_t struct_BSFailureCounter;
  uint32_t struct_BCSuccessCounter;
  uint32_t struct_BCFailureCounter;
  uint32_t struct_DPSuccessCounter;
  uint32_t struct_DPFailureCounter;
  uint32_t struct_DCSuccessCounter;
  uint32_t struct_DCFailureCounter;
  uint32_t struct_HPSuccessCounter;
  uint32_t struct_HPFailureCounter;
  // ETM delivery stats from DG's perspective — arrays of 6 (indexed by ETM_BOARD_*)
  bool     struct_etmBoardOnline[6];
  uint32_t struct_etmBoardSent[6];
  uint32_t struct_etmBoardAckd[6];
  uint32_t struct_etmBoardRetries[6];
  uint32_t struct_etmBoardFailed[6];
  } LoRa_Struct;

LoRa_Struct commandstoSendtoRemote;

void setupLoRaSendStruct(){
    commandstoSendtoRemote.struct_LoraPasscode = LoraPasscode;
    commandstoSendtoRemote.struct_incomingMsgAck = ACKBool;
    commandstoSendtoRemote.struct_msgAckID = incomingMsgId;
    commandstoSendtoRemote.struct_droidGatewayStatus = droidGatewayStatus;
    commandstoSendtoRemote.struct_bodyControllerStatus = bodyControllerStatus;
    commandstoSendtoRemote.struct_bodyLEDControllerStatus = bodyLEDControllerStatus;
    commandstoSendtoRemote.struct_bodyServoControllerStatus = bodyServoControllerStatus;
    commandstoSendtoRemote.struct_domePlateControllerStatus = domePlateControllerStatus;
    commandstoSendtoRemote.struct_domeControllerStatus = domeControllerStatus;
    commandstoSendtoRemote.struct_hpControllerStatus = hpControllerStatus;
    commandstoSendtoRemote.struct_domeLogicsControllerStatus = domeLogicsControllerStatus;
    commandstoSendtoRemote.struct_BL_LDP_Bright = BL_LDP_Bright;
    commandstoSendtoRemote.struct_BL_MAINT_Bright = BL_MAINT_Bright;
    commandstoSendtoRemote.struct_BL_VU_Bright = BL_VU_Bright;
    commandstoSendtoRemote.struct_BL_CS_Bright = BL_CS_Bright;
    commandstoSendtoRemote.struct_BL_vuOffsetInt = BL_vuOffsetInt;
    commandstoSendtoRemote.struct_BL_vuBaselineInt = BL_vuOffsetInt;
    commandstoSendtoRemote.struct_BL_vuOffsetExt = BL_vuOffsetExt;
    commandstoSendtoRemote.struct_BL_vuBaselineExt = BL_vuBaselineExt;
    commandstoSendtoRemote.struct_BL_BatteryVoltage = BL_BatteryVoltage;
    commandstoSendtoRemote.struct_BL_BatteryPercentage = BL_BatteryPercentage;
    commandstoSendtoRemote.struct_FunctionSWState = FunctionSWState;
    commandstoSendtoRemote.struct_remoteConnected = remoteConnected;
    commandstoSendtoRemote.struct_DGSuccessCounter = DGSuccessCounter;
    commandstoSendtoRemote.struct_DGFailureCounter = DGFailureCounter;
    commandstoSendtoRemote.struct_BCSuccessCounter = BCSuccessCounter;
    commandstoSendtoRemote.struct_BCFailureCounter = BCFailureCounter;
    commandstoSendtoRemote.struct_BSSuccessCounter = BSSuccessCounter;
    commandstoSendtoRemote.struct_BSFailureCounter = BSFailureCounter;
    commandstoSendtoRemote.struct_DPSuccessCounter = DPSuccessCounter;
    commandstoSendtoRemote.struct_DPFailureCounter = DPFailureCounter;
    commandstoSendtoRemote.struct_DCSuccessCounter = DCSuccessCounter;
    commandstoSendtoRemote.struct_DCFailureCounter = DCFailureCounter;
    commandstoSendtoRemote.struct_HPSuccessCounter = HPSuccessCounter;
    commandstoSendtoRemote.struct_HPFailureCounter = HPFailureCounter;
    // ETM delivery stats — DG's own view of each board (indexed by ETM_BOARD_*)
    for (int i = 0; i < ETM_NUM_BOARDS; i++) {
      commandstoSendtoRemote.struct_etmBoardOnline[i]  = etmBoardTable[i].online;
      commandstoSendtoRemote.struct_etmBoardSent[i]    = etmBoardTable[i].messagesSent;
      commandstoSendtoRemote.struct_etmBoardAckd[i]    = etmBoardTable[i].messagesAckd;
      commandstoSendtoRemote.struct_etmBoardRetries[i] = etmBoardTable[i].totalRetries;
      commandstoSendtoRemote.struct_etmBoardFailed[i]  = etmBoardTable[i].totalFailed;
    }
};

void setupLoRaSendStructNow(){
  ACKBool = false;
  // incomingMsgId = 0;
    commandstoSendtoRemote.struct_LoraPasscode = LoraPasscode;
    commandstoSendtoRemote.struct_incomingMsgAck = ACKBool;
    commandstoSendtoRemote.struct_msgAckID = incomingMsgId;
    commandstoSendtoRemote.struct_droidGatewayStatus = droidGatewayStatus;
    commandstoSendtoRemote.struct_bodyControllerStatus = bodyControllerStatus;
    commandstoSendtoRemote.struct_bodyLEDControllerStatus = bodyLEDControllerStatus;
    commandstoSendtoRemote.struct_bodyServoControllerStatus = bodyServoControllerStatus;
    commandstoSendtoRemote.struct_domePlateControllerStatus = domePlateControllerStatus;
    commandstoSendtoRemote.struct_domeControllerStatus = domeControllerStatus;
    commandstoSendtoRemote.struct_hpControllerStatus = hpControllerStatus;
    commandstoSendtoRemote.struct_domeLogicsControllerStatus = domeLogicsControllerStatus;
    commandstoSendtoRemote.struct_BL_LDP_Bright = BL_LDP_Bright;
    commandstoSendtoRemote.struct_BL_MAINT_Bright = BL_MAINT_Bright;
    commandstoSendtoRemote.struct_BL_VU_Bright = BL_VU_Bright;
    commandstoSendtoRemote.struct_BL_CS_Bright = BL_CS_Bright;
    commandstoSendtoRemote.struct_BL_vuOffsetInt = BL_vuOffsetInt;
    commandstoSendtoRemote.struct_BL_vuBaselineInt = BL_vuOffsetInt;
    commandstoSendtoRemote.struct_BL_vuOffsetExt = BL_vuOffsetExt;
    commandstoSendtoRemote.struct_BL_vuBaselineExt = BL_vuBaselineExt;
    commandstoSendtoRemote.struct_BL_BatteryVoltage = BL_BatteryVoltage;
    commandstoSendtoRemote.struct_BL_BatteryPercentage = BL_BatteryPercentage;
    commandstoSendtoRemote.struct_FunctionSWState = FunctionSWState;
    commandstoSendtoRemote.struct_DGSuccessCounter = DGSuccessCounter;
    commandstoSendtoRemote.struct_DGFailureCounter = DGFailureCounter;
    commandstoSendtoRemote.struct_BCSuccessCounter = BCSuccessCounter;
    commandstoSendtoRemote.struct_BCFailureCounter = BCFailureCounter;
    commandstoSendtoRemote.struct_BSSuccessCounter = BSSuccessCounter;
    commandstoSendtoRemote.struct_BSFailureCounter = BSFailureCounter;
    commandstoSendtoRemote.struct_DPSuccessCounter = DPSuccessCounter;
    commandstoSendtoRemote.struct_DPFailureCounter = DPFailureCounter;
    commandstoSendtoRemote.struct_DCSuccessCounter = DCSuccessCounter;
    commandstoSendtoRemote.struct_DCFailureCounter = DCFailureCounter;
    commandstoSendtoRemote.struct_HPSuccessCounter = HPSuccessCounter;
    commandstoSendtoRemote.struct_HPFailureCounter = HPFailureCounter;
    // ETM delivery stats — DG's own view of each board (indexed by ETM_BOARD_*)
    for (int i = 0; i < ETM_NUM_BOARDS; i++) {
      commandstoSendtoRemote.struct_etmBoardOnline[i]  = etmBoardTable[i].online;
      commandstoSendtoRemote.struct_etmBoardSent[i]    = etmBoardTable[i].messagesSent;
      commandstoSendtoRemote.struct_etmBoardAckd[i]    = etmBoardTable[i].messagesAckd;
      commandstoSendtoRemote.struct_etmBoardRetries[i] = etmBoardTable[i].totalRetries;
      commandstoSendtoRemote.struct_etmBoardFailed[i]  = etmBoardTable[i].totalFailed;
    }
};


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


void sendACK(int msgAckID){
  ACKBool = true;
 sendStatusMessage();
}



void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
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
  parseStrings(incoming);
  sendACK(incomingMsgId);
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
// Convert from String Object to String.
    char buf[100];
    data.toCharArray(buf, sizeof(buf));
    // char *p = buf;
    // char *str;
    // while ((str = strtok(p, ".", &p)) != NULL) // delimiter is the period
    //   // Serial.println(str);
    //    queuecommand = String(str);
    //   Serial.println(queuecommand);
      // enqueueCommand(queuecommand);

      char *token;
      const char *delimiter =".";


   token = strtok(buf, delimiter);

   while (token != NULL) {
      Serial.println(token);
      enqueueCommand(token);
      token=strtok(NULL, delimiter);
   }


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
}

void loop() {
  etmProcess();
  checkAgeofkeepAlive();
  sendStatusToRemote();
  checkButton();
  onReceive(LoRa.parsePacket());
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
              Debug.toggle(debugInputIdentifier);
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
                  case 9:  break;  //reserved for future use

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

                  for (int i=2; i<=commandLength; i++){
                    char inCharRead = inputBuffer[i];
                    ESPNOWStringCommand += inCharRead;                   // add it to the inputString:
                  }
                  Debug.LOOP("\nFull Command Recieved: %s \n",ESPNOWStringCommand.c_str());
                  ESPNOWTarget = ESPNOWStringCommand.substring(0,2);
                  Debug.LOOP("ESP NOW Target: %s\n", ESPNOWTarget.c_str());
                  ESPNOWSubStringCommand = ESPNOWStringCommand.substring(2,commandLength+1);
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




