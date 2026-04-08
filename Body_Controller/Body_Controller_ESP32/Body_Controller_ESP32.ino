///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                                                                                                       *****///
///*****                            Created by Greg Hulette.                                                   *****///
///*****                                                                                                       *****///
///*****   I started with the code from flthymcnsty from from which I used the basic command structure and     *****///
///*****  serial input method.                                                                                 *****///
///*****                                                                                                       *****///
///*****                                     So exactly what does this all do.....?                            *****///
///*****                       - Receives commands via Serial or ESP-NOW                                       *****///
///*****                       - Sends Serial commands to the LED Controller(ATMEGA2560 onboard)               *****///
///*****                       - Sends Serial commands to the Stealth Controller                               *****///
///*****                       - Sends Serial commands to the HCR                                              *****///
///*****                       - Connects the 25 wire sliip ring to the dome plate controller                  *****///
///*****                       - Provides USB connectivity to the dome plate  and dome controllers             *****///
///*****                                                                                                       *****///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                            Roam A Dome Home Stored Sequences                                           *****///
///*****      [4] H:A170,55:W1:D500,40:W1:D-360:W1:A90:A45:A135:W2:H                                            *****///
///*****      [[7] H:D90,100:D-90,100:D65,70:D-180,90:D-420,80:A0:D100,50:H                                     *****///
///*****      [14] H:D90,100:D-90,100:W1:D65,70:D-80,70:A0,30:W1:D-25,20:W5:A0    Short Circuit                 *****///
///*****                                                                                                        *****///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
///*****        Libraries used in this sketch                 *****///
//////////////////////////////////////////////////////////////////////

// Standard Arduino library
#include <Arduino.h>

// Used for OTA
#include "ESPAsyncWebServer.h"              //https://github.com/me-no-dev/ESPAsyncWebServer
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
#include <ElegantOTA.h>
#include <AsyncTCP.h>
#include <WiFi.h>

//Used for ESP-NOW
#include "esp_wifi.h"
#include <esp_now.h>
#define ETM_MY_BOARD_INDEX ETM_BOARD_BC
#include <ETM_Droid.h>

//Used for Status LEDs
#include <Adafruit_NeoPixel.h>

//Used for pin definition
#include "body_controller_esp32_pin_map.h"

// Debug Functions  - Using my own library for this
#include <DebugR2.h>  //  https://github.com/greghulette/Arduino-Code/tree/main/libraries/DebugR2  Put these files in a folder called "DebugR2" in your libraries folder and restart the IDE

//ReelTwo libaries - Using my forked version of this libarary
#include <ReelTwo.h>
#include "core/DelayCall.h"

// Used for Software Serial to allow more useful naming
#include <SoftwareSerial.h>

// Used to parse status from the ATMEGA2560 status messages
#include "ArduinoJson.h"

#include <hcr.h>
#include "sbus.h"
//////////////////////////////////////////////////////////////////////
///*****       Preferences/Items to change        *****///
//////////////////////////////////////////////////////////////////////
 //ESPNOW Password - This must be the same across all devices
  String ESPNOWPASSWORD = "GregsAstromech";  //Must be less than 20 characters

  ////R2 Control Network Details for OTA only
  const char* ssid = "R2D2_Control_Network";
  const char* password =  "astromech";

    //Enables status tracking on the Droid Gateway
  bool STATUS_TRACKING = 1;
  
  // Keepalive timer to send status messages to the Kill Switch (Droid)
  int keepAliveDuration= 4000;  // 4 seconds

// used to sync timing with the dome controller better, allowing time for the ESP-NOW messages to travel to the dome
// Change this to work with how your droid performs
  int defaultESPNOWSendDuration = 50;

  // Serial Baud Rates
  #define BL_BAUD_RATE 9600
  #define RD_BAUD_RATE 9600 
  #define ST_BAUD_RATE 9600  //Should be lower than 57600
  #define MP_BAUD_RATE 9600  //Should be lower than 57600
  #define SERIAL1_BAUD_RATE 9600 //Should be lower than 57600
  #define SERIAL2_BAUD_RATE 9600  //Should be lower than 57600




//////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
  String HOSTNAME = "Body Controller"; 
  
  char inputBuffer[200];
  String inputString;    
       // a string to hold incoming data
  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA
  
  int commandLength;
  
  String serialCommandString;
  String serialPort;
  String serialCommandSubString;

  String ESPNOWCommandString;
  String ESPNOWTarget;
  String ESPNOWTargetCommand;
    
  String mp3CommandString;
  String mp3CommandSubString;

  String eepromCommandString;
  String eepromCommandSubString;
    
  String ledCommandString;

  String radhCommandString;

  String controllerCommandString;

  uint32_t Internal_Command[6]  = {0,0,0,0,0,0};
  int internalCommandFunction = 0;

  uint32_t Animation_Command[6]  = {0,0,0,0,0,0};
  int AnimationCommandFunction = 0;

  debugClass Debug;
  String debugInputIdentifier ="";

  int mp3Track;
  String mp3Comm;
  String HCRTriggerResponseString;

  int* getEmotions;
  int getEmotion;
  float getDuration;
  int getOverride;
  bool isPlaying;
  int getMuse;
  int getWAVCount;
  int getPlayingWAV;
  int HCRVolume;
  int happyEmotionValue;
  int sadEmotionValue;
  int madEmotionValue;
  int scaredEmotionValue;
//////////////////////////////////////////////////////////////////////
  ///*****       Startup and Loop Variables                     *****///
  //////////////////////////////////////////////////////////////////////
  
  boolean startUp = true;
  boolean isStartUp = true;
  
    //Main Loop Timers
  unsigned long mainLoopTime; // We keep track of the "Main Loop time" in this variable.
  unsigned long MLMillis;
  uint8_t mainLoopDelayVar = 5;


///////////////////////////////////////////////////////////////////////
  ///*****                Status Variables                     *****///
  /////////////////////////////////////////////////////////////////////

  unsigned long keepAliveMillisDuration = 15000;
  unsigned long blkeepAliveAge;
  unsigned long blkeepaliveAgeMillis;
  unsigned long keepAliveMillis;

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
  int FunctionSwState;


  bool bodyLEDControllerStatus = 0;


bool remoteConnected = false;



 
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
  
  //////////////////////////////////////////////////////////////////
  ///******       Serial Ports Definitions                  *****///
  //////////////////////////////////////////////////////////////////

  #define rdSerial Serial1

  SoftwareSerial blSerial;
  SoftwareSerial stSerial;
  SoftwareSerial mpSerial;
  SoftwareSerial s1Serial;
  SoftwareSerial s2Serial;
 
HCRVocalizer HCR(&mpSerial,MP_BAUD_RATE); // Serial (Stream Port, baud rate)

#define SBUS 
#ifdef SBUS
/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, SERIAL_RX_SB_PIN, SERIAL_TX_SB_PIN, true, false);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2, SERIAL_RX_SB_PIN, SERIAL_TX_SB_PIN, true, false);
/* SBUS data */
bfs::SbusData sbusData;

#endif

  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////

//Droid Remote ESP          192.168.4.101   
//Droid Gateway ESP         192.168.4.108   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Controller ESP       192.168.4.109   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Servo ESP            192.168.4.110   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Controller ESP       192.168.4.111   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Plate Controller ESP 192.168.4.112   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//HP Controller ESP         192.168.4.113   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Droid Raspberry Pi        192.168.4.114
//Remote Raspberry Pi       192.168.4.115
//Developer Laptop          192.168.4.125

// IP Address config of local ESP
IPAddress local_IP(192,168,4,109);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,4,100);

const uint8_t oldLocalMACAddress[] = {0x24, 0x0A, 0xC4, 0xED, 0x30, 0x12};    //used when connecting to WiFi for OTA

AsyncWebServer server(80);

 

//////////////////////////////////////////////////////////////////////
///*****            Status LED Variables and settings       *****///
//////////////////////////////////////////////////////////////////////
  
// -------------------------------------------------
// Define some constants to help reference objects,
// pins, leds, colors etc by name instead of numbers
// -------------------------------------------------
//    Status LED VARIABLES
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

  Adafruit_NeoPixel ESP_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, STAUS_LED_PIN, NEO_GRB + NEO_KHZ800);




































































/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

// MAC addresses and board IDs are defined in ETM_Droid.h

// Define variables to store commands to be sent
  String  senderID;
  String  targetID;
  bool    commandIncluded;
  String  command;
  uint32_t SuccessCounter = 0;
  uint32_t FailureCounter = 0;

// Define variables to store incoming commands
  String  incomingTargetID;  
  String  incomingSenderID;
  String  incomingCommand;
  bool    incomingCommandIncluded;
  String  incomingPassword;
 












// Variable to store if sending data was successful
  String success;

// espnow_struct_message is defined in ETM_Droid.h

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

// DG gets a special status struct; all other targets use standard espnow_struct_message
  bodyControllerStatus_struct_message commandsToSendtoDroidLoRa;
  espnow_struct_message outgoingMsg;
  espnow_struct_message incomingMsg;

  esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == 0) { SuccessCounter++; } else { FailureCounter++; }
  Debug.ESPNOW("Last Packet Send Status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  colorWipeStatus("ES", orange, 255);
  if (len < (int)sizeof(espnow_struct_message)) {
    Debug.ESPNOW("ESP-NOW packet too short (%d bytes), ignored\n", len);
    colorWipeStatus("ES", blue, 10);
    return;
  }
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
  if (strncmp(incomingMsg.structPassword, ESPNOWPASSWORD.c_str(), sizeof(incomingMsg.structPassword)) != 0) {
    Debug.ESPNOW("Wrong ESP-NOW Password was sent. Message Ignored\n");
    colorWipeStatus("ES", blue, 10);
    return;
  }
  int senderIdx = etmBoardIndexFromMAC(esp_now_info->src_addr);
  if (senderIdx >= 0) etmHandleHeartbeat(senderIdx);
  switch (incomingMsg.structPacketType) {
    case PACKET_TYPE_HEARTBEAT:
      break;
    case PACKET_TYPE_ACK:
      etmProcessAck(senderIdx, incomingMsg.structSequenceNumber);
      break;
    case PACKET_TYPE_COMMAND:
      incomingSenderID = incomingMsg.structSenderID;
      incomingTargetID = incomingMsg.structTargetID;
      incomingCommandIncluded = incomingMsg.structCommandIncluded;
      incomingCommand = incomingMsg.structCommand;
      processESPNOWIncomingMessage();
      etmSendAck(senderIdx, incomingMsg.structSequenceNumber);
      break;
    default:
      Debug.ESPNOW("ESP-NOW unknown packet type, ignored\n");
      break;
  }
  colorWipeStatus("ES", blue, 10);
}

void processESPNOWIncomingMessage(){
  Debug.ESPNOW("incoming target: %s\n", incomingTargetID.c_str());
  Debug.ESPNOW("incoming sender: %s\n", incomingSenderID.c_str());
  Debug.ESPNOW("incoming command included: %d\n", incomingCommandIncluded);
  Debug.ESPNOW("incoming command: %s\n", incomingCommand.c_str());
  if (incomingTargetID == "BC" || incomingTargetID == "BR"){
    inputString = incomingCommand;
    stringComplete = true; 
    Debug.ESPNOW("Recieved command from $sn", incomingSenderID);

  }
}


 //////////////////////////////////////////////////////////////////////
  ///******      Arduino Mega Reset Pin Specific Setup          *****///
  //////////////////////////////////////////////////////////////////////

  #define RST RESET_PIN_2560


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
  } 
  else{Debug.DBG("No LED was chosen \n");}
};

//////////////////////////////////////////////////////////////////////
///*****    Send Keepalive Messages for Status                *****///
//////////////////////////////////////////////////////////////////////

void keepAlive(){
  if (STATUS_TRACKING == 1){
    if (millis() - keepAliveMillis >= (keepAliveDuration + random(1, 1000))){
    keepAliveMillis = millis();
    sendESPNOWCommand("DG","");  
    } 
  }
};

//////////////////////////////////////////////////////////////////////
///*****    Checks the age of the Status Variables            *****///
//////////////////////////////////////////////////////////////////////

void checkAgeofkeepAlive(){    //checks for the variable's age

    if (bodyLEDControllerStatus==1){
    if (millis()-blkeepAliveAge>=keepAliveMillisDuration){
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
      Debug.ESPNOW("Body LED Controller Offline\n");
    }
  }
}


void printKeepaliveStatus(){
  if (Debug.debugflag_status == 0)
  {
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
  } else
  {  
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
  Internal_Command[0]   = '\0';
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

  Internal_Command[0]   = '\0';
} ;

/////////////////////////////////////////////////////////
///*****          Serial Event Function          *****///
/////////////////////////////////////////////////////////

void serialEvent() {
  while (Serial.available()>0) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  Debug.SERIAL_EVENT("USB Serial Input: %s \n",inputString.c_str());
};

void serialRdEvent() {
  while (rdSerial.available()) {
    // get the new byte:
    char inChar = (char)rdSerial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  Debug.SERIAL_EVENT("Roam-A-Dome Input: %s \n",inputString);
};

void serialBlEvent() {
  while (blSerial.available()) {
    // get the new byte:
    StaticJsonDocument<2048> doc;
colorWipeStatus("ES", green, 10);
    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(doc, blSerial);

    if (err == DeserializationError::Ok) 
    {
      bodyLEDControllerStatus = doc["bodyLEDControllerStatus"].as<bool>();
      if (bodyLEDControllerStatus == 1){
        bodyLEDControllerStatus=1;
        blkeepAliveAge = millis();
        Debug.JSON("Body LED Controler Keepalive Received \n");
        }
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
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
      Debug.JSON("Body LED Controller Status = %d\n", bodyLEDControllerStatus); //Serial.println(doc["BL_Status"].as<String>());
      Debug.JSON("LDP Brightness = %i\n", BL_LDP_Bright); //Serial.println(doc["LDPBright"].as<int>());
      Debug.JSON("Maint Brightness = %i\n", BL_MAINT_Bright); //Serial.println(doc["MaintBright"].as<int>());
      Debug.JSON("VU Brightness = %i\n", BL_VU_Bright); //Serial.println(doc["VUBright"].as<int>());
      Debug.JSON("Coin Slots Brightness = %i\n",BL_CS_Bright); //Serial.println(doc["CoinBright"].as<int>());
      Debug.JSON("Spectrum Int Offset = %i\n",BL_vuOffsetInt); //Serial.println(doc["VUIntOffset"].as<int>());
      Debug.JSON("Spectrum Int Baseline = %i\n",BL_vuBaselineInt); //Serial.println(doc["VUIntBaseline"].as<int>());
      Debug.JSON("Spectrum Ext Offset = %i\n",BL_vuOffsetExt); //Serial.println(doc["VUExtOffset"].as<int>());
      Debug.JSON("Spectrum Ext Baseline = %i\n",BL_vuBaselineExt); //Serial.println(doc["VUExtOffset"].as<int>());
      Debug.JSON("Battery Voltage = %.2f\n",BL_BatteryVoltage); //Serial.println(doc["BatteryVoltage"].as<float>());
      Debug.JSON("Battery Percentage = %i\n",BL_BatteryPercentage);//Serial.println(doc["BatteryPercent"].as<int>());
    } 
    else 
    {
      // Print error to the "debug" serial port
      if (Debug.debugflag_json == 1){
        Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());
      }
    }
  };
  colorWipeStatus("ES", blue, 10);
  Debug.SERIAL_EVENT("BL Status Recieved: \n");

};

void serialStEvent() {
  while (stSerial.available()) {
    // get the new byte:
    char inChar = (char)stSerial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        inputString = ":" + inputString;
        stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  // inputString = ":" + inputString;
  Debug.SERIAL_EVENT("Stealth Serial Input: %s \n",inputString.c_str());
};

void serialMpEvent() {
  while (mpSerial.available()) {
    char inChar = (char)mpSerial.read();
    HCRTriggerResponseString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
    };
  };
  Debug.SERIAL_EVENT("HCR Response: %s \n",HCRTriggerResponseString);
  HCRTriggerResponseString = "";
};

void serial1Event() {
  while (s1Serial.available()) {
    // get the new byte:
    char inChar = (char)s1Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  Debug.SERIAL_EVENT("Serial 1 Input: %s \n",inputString);
};
void serial2Event() {
  while (s2Serial.available()) {
    // get the new byte:
    char inChar = (char)s2Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
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

void writeRdSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    rdSerial.write(completeString[i]);
  };
  Debug.SERIAL_EVENT("Writing to Roam-A-Dome\n");
};
void writeBlSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    blSerial.write(completeString[i]);
  };
  Debug.SERIAL_EVENT("Writing to ATMEGA\n");
};

void writeStSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    stSerial.write(completeString[i]);
  };
  Debug.SERIAL_EVENT("Writing to the Stealth Controller\n");
};

void writeMpSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    mpSerial.write(completeString[i]);
  };
  Debug.SERIAL_EVENT("Writing to the HCR Controller\n");
};

void writeS1Serial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    s1Serial.write(completeString[i]);
  };
  Debug.SERIAL_EVENT("Writing to Serial 1: %s\n", stringData.c_str());
};

void writeS2Serial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    s2Serial.write(completeString[i]);
  };
  Debug.SERIAL_EVENT("Writing to Serial 2: %s\n", stringData.c_str());
};


/////////////////////////////////////////////////////////////////////
///*****             ESP-NOW Functions                        *****///
//////////////////////////////////////////////////////////////////////

void setupSendStatusStruct(bodyControllerStatus_struct_message* msg, String pass, String sender, String targetID, bool hascommand, String cmd)
{
    snprintf(msg->structPassword, sizeof(msg->structPassword), "%s", pass.c_str());
    snprintf(msg->structSenderID, sizeof(msg->structSenderID), "%s", sender.c_str());
    snprintf(msg->structTargetID, sizeof(msg->structTargetID), "%s", targetID.c_str());
    msg->structBL_LDP_Bright = BL_LDP_Bright;
    msg->structBL_MAINT_Bright = BL_MAINT_Bright;
    msg->structBL_VU_Bright = BL_VU_Bright;
    msg->structBL_CS_Bright = BL_CS_Bright;
    msg->structBL_vuOffsetInt = BL_vuOffsetInt;
    msg->structBL_vuBaselineInt = BL_vuBaselineInt;
    msg->structBL_vuOffsetExt = BL_vuOffsetExt;
    msg->structBL_vuBaselineExt = BL_vuBaselineExt;
    msg->structBL_BatteryVoltage = BL_BatteryVoltage;
    msg->structBL_BatteryPercentage = BL_BatteryPercentage;
    msg->structbodyLEDControllerStatus = bodyLEDControllerStatus;
    msg->structFunctionSWState = FunctionSwState;
    msg->struct_remoteConnected = remoteConnected;
    msg->structCommandIncluded = hascommand;
    msg->structSuccess = SuccessCounter;
    msg->structFailure = FailureCounter;
    snprintf(msg->structCommand, sizeof(msg->structCommand), "%s", cmd.c_str());
    // Populate per-board ETM delivery stats
    for (int i = 0; i < ETM_NUM_BOARDS; i++) {
      msg->etmBoardOnline[i]  = etmBoardTable[i].online;
      msg->etmBoardSent[i]    = etmBoardTable[i].messagesSent;
      msg->etmBoardAckd[i]    = etmBoardTable[i].messagesAckd;
      msg->etmBoardRetries[i] = etmBoardTable[i].totalRetries;
      msg->etmBoardFailed[i]  = etmBoardTable[i].totalFailed;
    }
};

void sendESPNOWCommand(String starget, String scomm) {
  bool hasCommand = (scomm.length() > 0);

  // DG gets the full bodyControllerStatus struct (special case)
  if (starget == "DG") {
    setupSendStatusStruct(&commandsToSendtoDroidLoRa, ESPNOWPASSWORD, "BC", starget, hasCommand, scomm);
    esp_err_t result = esp_now_send(ETM_BOARD_MACS[ETM_BOARD_DG], (uint8_t*)&commandsToSendtoDroidLoRa, sizeof(commandsToSendtoDroidLoRa));
    if (result == ESP_OK) { Debug.ESPNOW("Sent status to DG: %s\n", scomm.c_str()); }
    else { Debug.ESPNOW("Error sending to DG\n"); }
    return;
  }

  bool isBroadcast = (starget == "BR");
  int targetIdx = etmBoardIndexFromID(starget.c_str());
  const uint8_t* targetMAC = isBroadcast ? ETM_BROADCAST_MAC
                           : (targetIdx >= 0) ? ETM_BOARD_MACS[targetIdx]
                           : nullptr;
  if (targetMAC == nullptr) {
    Debug.ESPNOW("No valid destination for target: %s\n", starget.c_str());
    return;
  }
  uint16_t seqNum = etmNextSequence();
  memset(&outgoingMsg, 0, sizeof(outgoingMsg));
  snprintf(outgoingMsg.structPassword, sizeof(outgoingMsg.structPassword), "%s", ESPNOWPASSWORD.c_str());
  snprintf(outgoingMsg.structSenderID, sizeof(outgoingMsg.structSenderID), "BC");
  snprintf(outgoingMsg.structTargetID, sizeof(outgoingMsg.structTargetID), "%s", starget.c_str());
  outgoingMsg.structCommandIncluded = hasCommand;
  outgoingMsg.structSuccess = SuccessCounter;
  outgoingMsg.structFailure = FailureCounter;
  snprintf(outgoingMsg.structCommand, sizeof(outgoingMsg.structCommand), "%s", scomm.c_str());
  outgoingMsg.structPacketType = PACKET_TYPE_COMMAND;
  outgoingMsg.structSequenceNumber = seqNum;
  esp_err_t result = esp_now_send(targetMAC, (uint8_t*)&outgoingMsg, sizeof(outgoingMsg));
  if (result == ESP_OK) {
    Debug.ESPNOW("Sent command: %s to %s\n", scomm.c_str(), starget.c_str());
    if (!isBroadcast) etmAddToPending(scomm.c_str(), starget.c_str(), targetIdx, seqNum);
  } else {
    Debug.ESPNOW("Error sending ESP-NOW data\n");
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                             Miscellaneous Functions                                           /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
///*****    Resets Arduino Mega due to bug in my PCB          *****///
//////////////////////////////////////////////////////////////////////

void resetArduino(int delayperiod){
  Debug.DBG("Body LED Controller Reset Function\n");
  digitalWrite(RST,LOW);
  delay(delayperiod);
  digitalWrite(RST,HIGH);
}

//////////////////////////////////////////////////////////////////////
///*****         Function for MP3 Trigger                     *****///
//////////////////////////////////////////////////////////////////////
// enum hcrCommand {update = 1,
//                   setEmote, 
//                   trigger, 
//                   stimulate, 
//                   overload, 
//                   muse, 
//                   museMinMax, 
//                   stop, 
//                   stopEmote,
//                   overrideEmotions,
//                   resetEmotions,
//                   setEmotions,
//                   setMuse,
//                   playWAVInt,
//                   playWAVString,
//                   stopWAV,
//                   setVolume,
//                   getEmotions,
//                   getEmotion,
//                   getDuration,
//                   getOverride,
//                   isPlaying,
//                   isPlayingChannel,
//                   getWAVCount,
//                   getPlayingWAV,
//                   getVolume,
//                   getUpdate
//                   };
/*
NEED TO WRITE NEW FUNCTION TO WORK WITH THE HCR
*/

void mp3Trigger(String comm, int track){
  mpSerial.print(comm);
  mpSerial.write(track);
}

void HCRFunction(int command = 0, int chan = 0, int track = 0, String filename= ""){

    switch (command) {
    case 1: HCR.update();                               break;
    case 2: HCR.SetEmotion(chan, track);   HCR.update();              break;
    case 3: HCR.Trigger(chan, track);                   break;
    case 4: HCR.Stimulate(chan, track);                 break;
    case 5: HCR.Overload();                             break;
    case 6: HCR.Muse();                                 break;
    case 7: HCR.Muse(chan, track);                      break;
    case 8: HCR.Stop();                                 break;
    case 9: HCR.StopEmote();                            break;
    case 10: HCR.OverrideEmotions(chan);                break;
    case 11: HCR.ResetEmotions();                       break;
    case 12: HCR.SetEmotion(chan, track);               break;
    case 13: HCR.SetMuse(track);                        break;
    case 14: HCR.PlayWAV(chan, track); HCR.update();                 break;
    case 15: HCR.PlayWAV(chan, filename);               break;
    case 16: HCR.StopWAV(chan);                         break;
    case 17: HCR.SetVolume(chan, track);                break;
    case 18: getEmotions = HCR.GetEmotions();           break;
    case 19: getEmotion = HCR.GetEmotion(chan);         break;
    case 20: getDuration = HCR.GetDuration();           break;
    case 21: getOverride = HCR.GetOverride();           break;
    case 22: isPlaying = HCR.IsPlaying();               break;
    case 24: isPlaying = HCR.IsPlaying(chan);           break;
    case 25: getWAVCount = HCR.GetWAVCount();           break;
    case 26: getPlayingWAV = HCR.GetPlayingWAV(chan);   break;
    case 27: HCR.getUpdate();                           break;
    case 28: HCRVolume = HCR.getVolume(chan);           break;
    case 29: HCR.getUpdate(); getHCRStatus();                        break;
    // case 29: HCR.dfPlayer();                            break;
    };
  
}


String getValue(String data, char separator, int index){
  int found=0;
  int strIndex[] = {0,-1};
  int maxIndex = data.length()-1;
  for (int i=0; i <= maxIndex && found <= index; i++){
    if(data.charAt(i) == separator || i == maxIndex){
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i == maxIndex)? i+1 : i;

    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void getHCRStatus(){
  happyEmotionValue = HCR.GetEmotion(0);
  sadEmotionValue = HCR.GetEmotion(1);
  madEmotionValue = HCR.GetEmotion(2);
  scaredEmotionValue = HCR.GetEmotion(3);
  float vocalizerVolume = HCR.getVolume(0);
  float chan_AVolume = HCR.getVolume(1);
  float chan_BVolume = HCR.getVolume(2);
  Debug.STATUS("Happy: %i\nSad: %i\nMad: %i\nScared: %i\n", happyEmotionValue, sadEmotionValue, madEmotionValue, scaredEmotionValue);
  Debug.STATUS("Vocalizer Volume: %f\nChannel A Volume: %f\nChanel B Volume: %f\n\n", vocalizerVolume, chan_AVolume, chan_BVolume);
};

/*///////////////////////////////////////////////////////////////////////////////////////////////////////
                    Animations
              case 1: normalOperations();   break;
              case 2: panelWave();                                          break;
              case 3: panelWaveFast();                                      break;
              case 4: domePeriscope();                                      break;
              case 5: allOpenClose();                                       break;
              case 6: HarlemShake();                                        break;
              case 7: allClose();                                           break;
*////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void normalOperations(){
writeBlSerial("A99");
sendESPNOWCommand("BS", ":D304");
HCR.Stop();
 Animation_Command[0]   = '\0'; 
};

void panelWave(){
  sendESPNOWCommand("BS", ":D310D03125");
 Animation_Command[0]   = '\0'; 
};

void panelWaveFast(){
  sendESPNOWCommand("BS", ":D311D01000");
   Animation_Command[0]   = '\0'; 
};

void domePeriscope(){
    sendESPNOWCommand("DP", ":SUS:PS4");
    writeRdSerial(":DPS4");
 Animation_Command[0]   = '\0'; 
};

void allOpenClose(){
    sendESPNOWCommand("BS", ":D306");
 Animation_Command[0]   = '\0'; 
};

void HarlemShake(){
  // float currentVolume = HCR.getVolume(1);
  sendESPNOWCommand("BS", ":D313");
  // HCR.SetVolume(1,100);
  HCR.PlayWAV(1,202);  //Figure out which is the correct track to play
  HCR.update();
  // HCR.SetVolume(1,currentVolume);
 Animation_Command[0]   = '\0'; 
};

void allClose(){
    sendESPNOWCommand("BS", ":D304");
    writeBlSerial("J98");   // VU + data panel + CBI off (clear + disable auto)
 Animation_Command[0]   = '\0';
};

void allOpen(){
    sendESPNOWCommand("BS", ":D303");
    writeBlSerial("J99");   // VU + data panel + CBI on (auto mode)
 Animation_Command[0]   = '\0';
};
bool doorsOpen = false;
void toggleDoors(){
  if (doorsOpen){
    sendESPNOWCommand("BS", ":D304");
    writeBlSerial("J98");   // VU + data panel + CBI off (clear + disable auto)
    doorsOpen = false;
  } else{
    doorsOpen = true;
    sendESPNOWCommand("BS", ":D303");
    writeBlSerial("J99");   // VU + data panel + CBI on (auto mode)
  }
   Animation_Command[0]   = '\0';
}

void allFlutter(){
  sendESPNOWCommand("DC", ":R0112");
  sendESPNOWCommand("BS", ":D308");  
  DelayCall::schedule([]{sendESPNOWCommand("DC", ":R0155");}, 1000);

     Animation_Command[0]   = '\0'; 

}

void CompleteshortCircuit(){
  writeBlSerial("E14");
  HCR.Overload();
  DelayCall::schedule([]{HCR.Stimulate(3, 1);},1500);
  sendESPNOWCommand("HP", ":HA014");
  // sendESPNOWCommand("DC", ":SDL@APLE20530");
  writeRdSerial(":DPS14");  // #DPS14:H:D90,100:D-90,100:D65,70:D-180,90:D-420,80:A0:D100,50:H,30
  

  DelayCall::schedule([]{sendESPNOWCommand("DC", ":SPS0T4");}, 50);
  DelayCall::schedule([]{sendESPNOWCommand("BS", ":D305");}, 100);
  DelayCall::schedule([]{sendESPNOWCommand("DP", ":A54");}, 150);
  DelayCall::schedule([]{sendESPNOWCommand("DC", ":SDL@APLE20530");}, 200);
  DelayCall::schedule([]{sendESPNOWCommand("DP", ":SUS:PS14");}, 250);

  DelayCall::schedule([]{resetLightsafterShortCircuit();}, 18000);
  Animation_Command[0]   = '\0'; 

}

void resetLightsafterShortCircuit(){
  writeBlSerial("K99");
  sendESPNOWCommand("HP", ":HA0025");
}

bool lightsOn = true;

void allLightsToggle(){
  if (lightsOn){
    writeBlSerial("E98");
    // writeBlSerial("B98");
    lightsOn = false;
  } else {
    lightsOn = true;
    writeBlSerial("E99");
    // writeBlSerial("B99");

  }
  Animation_Command[0]   = '\0'; 

}


void drawerWave(){
  sendESPNOWCommand("BS", ":D118");
  Animation_Command[0]   = '\0'; 
};

void OpenClosewithEasing(){
  sendESPNOWCommand("BS", ":D306B312000400000050");
  writeBlSerial("J99");   // VU + data panel + CBI on (auto mode)
  Animation_Command[0]   = '\0';
};

void StarWarsThemeSong(){
  HCR.PlayWAV(1, 0001);
  Animation_Command[0]   = '\0'; 
};

void VaderThemeSong(){
  HCR.PlayWAV(1, 0002);
  Animation_Command[0]   = '\0'; 
};

void PeriscopeSeq2(){
  sendESPNOWCommand("DP", ":SUS:PS2");
  Animation_Command[0]   = '\0'; 
};

void PerisopeSeq10(){
  sendESPNOWCommand("DP", ":SUS:PS10");
  Animation_Command[0]   = '\0'; 
};

void LightsOff(){
  writeBlSerial("K98");
  Animation_Command[0]   = '\0'; 
};

void LightsOn(){
  writeBlSerial("K99");
  Animation_Command[0]   = '\0'; 
};

void FartNoise(){
  HCR.PlayWAV(1, 0004);
  Animation_Command[0]   = '\0'; 
};

void WaveUtilityArm(){
  sendESPNOWCommand("BS", ":D119");
  Animation_Command[0]   = '\0'; 
};

void LaunchSaber(){
  sendESPNOWCommand("DP", ":A60");
    Animation_Command[0]   = '\0'; 
}

void ArmSaber(){
  sendESPNOWCommand("DP", ":A61");
    Animation_Command[0]   = '\0'; 
}

void StowSaber(){
  sendESPNOWCommand("BS", ":D10104");
  DelayCall::schedule([]{ sendESPNOWCommand("BS", ":D10204");}, 7050);
  Animation_Command[0]   = '\0'; 
}

void SmokeSequence(){
  sendESPNOWCommand("DP", ":A54");
    Animation_Command[0]   = '\0'; 
}

void display(){
  sendESPNOWCommand("DP", ":SUS:PP100:L0");
  sendESPNOWCommand("BS", ":D320");
    Animation_Command[0]   = '\0'; 
}

void openCloseWave(){
  sendESPNOWCommand("BS", ":D312");
    Animation_Command[0]   = '\0'; 
}

void longDance(){
  sendESPNOWCommand("BS", ":D315");
      Animation_Command[0]   = '\0'; 
}

void cpuSequence(){
  sendESPNOWCommand("BS", ":D126");
      Animation_Command[0]   = '\0'; 
}

void cpuRaise(){
  sendESPNOWCommand("BS", ":D10103");
  DelayCall::schedule([]{sendESPNOWCommand("BS", ":D121");}, 500);
      Animation_Command[0]   = '\0'; 
}

void cpuLower(){
  sendESPNOWCommand("BS", ":D122");
  DelayCall::schedule([]{sendESPNOWCommand("BS", ":D10203");}, 1000);

      Animation_Command[0]   = '\0'; 
}

void cpuExtend(){
  sendESPNOWCommand("BS", ":D123");
      Animation_Command[0]   = '\0'; 
}

void cpuRetract(){
  sendESPNOWCommand("BS", ":D124");
      Animation_Command[0]   = '\0'; 
}

void cpuRotate(){
  sendESPNOWCommand("BS", ":D125");
      Animation_Command[0]   = '\0'; 
}

void fireExinguisher(){
  sendESPNOWCommand("DP", ":A70");
      Animation_Command[0]   = '\0'; 
}

#ifdef SBUS


#define CHANNEL_4_ARRAY_INDEX 3
#define CHANNEL_5_ARRAY_INDEX 4
#define CHANNEL_6_ARRAY_INDEX 5
#define CHANNEL_7_ARRAY_INDEX 6
#define CHANNEL_8_ARRAY_INDEX 7
#define CHANNEL_9_ARRAY_INDEX 8
#define CHANNEL_10_ARRAY_INDEX 9

#define CHANNEL_16_ARRAY_INDEX 15
#define CHANNEL_18_ARRAY_INDEX 16


int newValue = 0;
int oldValueChannel4 = 100;
int oldValueChannel5 = 100;
int oldValueChannel6 = 100;
int oldValueChannel7 = 100;
int oldValueChannel8 = 100;
int oldValueChannel9 = 100;
int oldValueChannel10 = 100;
int oldValueChannel16 = 100;
int oldvalueChannel18;

bool Channel6Bool;
int Channel6State = 0;
int lastChannel6State = 0;

bool Channel7Bool;
int Channel7State = 0;
int lastChannel7State = 0;

bool Channel8Bool;
int Channel8State = 0;
int lastChannel8State = 0;
bool lostFrameOld = false;
bool Channel9Bool;
int Channel9State = 0;
int lastChannel9State = 0;

bool Channel18bool;
bool Channel10Bool;
int Channel10State = 0;
int lastChannel10State = 0;
int sbusValues[] = {};
void processSbus(){
    if (sbus_rx.Read()) {
    /* Grab the received data */
    sbusData = sbus_rx.data();
    /* Display the received data */
    for (int8_t i = 0; i < sbusData.NUM_CH; i++) {
      if (Debug.debugflag2 == 1){  
        Serial.print(sbusData.ch[i]);
      Serial.print("\t");
      }
      // newValue = sbusData.ch[i];
      sbusValues[i] = sbusData.ch[i];

    }
  
    /* Display lost frames and failsafe data */      
    if (Debug.debugflag2 == 1){  

    Serial.print(sbusData.lost_frame);
    Serial.print("\t");
    Serial.println(sbusData.failsafe);
}
    // Serial.println(sbusValues[]);
              // Serial.println(oldValue);
          // unsigned long  oldValueCurrentMillisChannel4 = millis();
if (lostFrameOld != sbusData.lost_frame){
  if (sbusData.lost_frame == true ){
      lostFrameOld = sbusData.lost_frame;

    // Debug.STATUS("Remote Disconnected \n");
      remoteConnected = false;
    } else if (sbusData.lost_frame == false){
        lostFrameOld = sbusData.lost_frame;

      // Debug.STATUS("Remote Connected \n");

      remoteConnected = true;
    }

}

    
  
  if (abs(sbusValues[CHANNEL_16_ARRAY_INDEX] - oldValueChannel16) >5){
    oldValueChannel16 = sbusValues[CHANNEL_16_ARRAY_INDEX];
    if (oldValueChannel16 < 200){
    FunctionSwState = 1;
  } else if (oldValueChannel16 > 700 && oldValueChannel16 <= 1200){
    FunctionSwState = 2;
  } else if (oldValueChannel16 > 1700){
    FunctionSwState = 3;
  }
  sendESPNOWCommand("DG","");
  }

          if(abs(sbusValues[CHANNEL_4_ARRAY_INDEX] - oldValueChannel4) >=5 ){
           oldValueChannel4 = sbusValues[CHANNEL_4_ARRAY_INDEX];
            RCRadio_HCRVolChange(0, CHANNEL_4_ARRAY_INDEX);
      }

          if(abs(sbusValues[CHANNEL_5_ARRAY_INDEX] - oldValueChannel5) >=5 ){
           oldValueChannel5 = sbusValues[CHANNEL_5_ARRAY_INDEX];
            RCRadio_HCRVolChange(1, CHANNEL_5_ARRAY_INDEX);
      }
       if(abs(sbusValues[CHANNEL_6_ARRAY_INDEX] - oldValueChannel6) >=5 ){
           oldValueChannel6 = sbusValues[CHANNEL_6_ARRAY_INDEX];
          Serial.println("Channel 6 changed");
          RCRadio_Matrix_Buttons(oldValueChannel6);
      }       
      if(abs(sbusValues[CHANNEL_7_ARRAY_INDEX] - oldValueChannel7) >=5 ){
           oldValueChannel7 = sbusValues[CHANNEL_7_ARRAY_INDEX];
          Serial.println("Channel 7 changed");
          RC_RADH_Auto(oldValueChannel7);
          // RCRadio_ToggleDoors(oldValueChannel7);
      }
      if(abs(sbusValues[CHANNEL_8_ARRAY_INDEX] - oldValueChannel8) >=5 ){
           oldValueChannel8 = sbusValues[CHANNEL_8_ARRAY_INDEX];
          Serial.println("Channel 8 changed");
          maintLights(oldValueChannel8);
          // RC_RADH_Auto(oldValueChannel8);
          // RCRadio_ToggleDoors(oldValueChannel7);
      }     
      if(abs(sbusValues[CHANNEL_9_ARRAY_INDEX] - oldValueChannel9) >=5 ){
           oldValueChannel9 = sbusValues[CHANNEL_9_ARRAY_INDEX];
          Serial.println("Channel 9 changed");
          lightsMode(oldValueChannel9);
          // RC_RADH_Auto(oldValueChannel8);
          // RCRadio_ToggleDoors(oldValueChannel7);
      }
      if(abs(sbusValues[CHANNEL_10_ARRAY_INDEX] - oldValueChannel10) >=5 ){
           oldValueChannel10 = sbusValues[CHANNEL_10_ARRAY_INDEX];
          Serial.println("Channel 10 changed");
          updateMuse(oldValueChannel10);
          // RC_RADH_Auto(oldValueChannel8);
          // RCRadio_ToggleDoors(oldValueChannel7);
      }

  }
}

void updateMuse(int PWMvalue){
  if (PWMvalue >= 100 && PWMvalue <= 799){
    HCR.SetMuse(1);
  } else if (PWMvalue >= 800){
      HCR.SetMuse(0);
  }
}

void lightsMode(int PWMvalue){
if (PWMvalue >= 1500){
    writeBlSerial("E98");
} else if (PWMvalue >= 800 && PWMvalue <= 1200){
  writeBlSerial("E98");

  DelayCall::schedule([]{writeBlSerial("K99");}, 50);

} else if (PWMvalue <= 799){
    writeBlSerial("E98");
      DelayCall::schedule([]{writeBlSerial("X99");}, 50);

    // writeBlSerial("X99");
}
}



void maintLights(int PWMvalue){
if (PWMvalue >= 1500){
writeBlSerial("M1518");
} else {
writeBlSerial("M98");
  }

}
void RC_RADH_Auto(int PWMvalue){
  if(PWMvalue <= 300){
    Channel7Bool = true;
    // Serial.println("True, less than 300");
  } else { Channel7Bool = false;}
  Channel7State = Channel7Bool;
  if(Channel7State != lastChannel7State){
    if(Channel7State == true){
    Serial.println("RADH DISABLED");
    writeRdSerial("#DPAUTO0");
    }
    if(Channel7State == false){
    Serial.println("RADH Enabled");
    writeRdSerial("#DPAUTO1");

  }
  } 

      lastChannel7State = Channel7State;


}


void RCRadio_Matrix_Buttons(int PWMvalue){
  if (sbusValues[CHANNEL_16_ARRAY_INDEX] <= 200){
    if (PWMvalue >= 100){
      Channel6Bool = true;
    } else {Channel6Bool = false;}
    if (PWMvalue >= 1800){
    Channel6State = Channel6Bool;
    if (Channel6State != lastChannel6State){
      if (Channel6State = true){
          Serial.println("Matrix Sw1: Mute Sounds");
          HCR.StopWAV(1);
          HCR.StopEmote();  
        }
      }
      }
    lastChannel6State = Channel6State;
    // if (PWMvalue <= 1900 && PWMvalue >= 1800){
    //   Serial.println("Matrix Sw2: Happy Sound");
    //   HCR.StopWAV(1);
    //   HCR.StopEmote();
    // }
    if (PWMvalue <= 1799 && PWMvalue >= 1750){
      Serial.println("Matrix Sw2: Happy Sound");
      HCR.Stimulate(0,1);    
    }
    if (PWMvalue <= 1749 && PWMvalue >= 1700){
      Serial.println("Matrix Sw3: Sad Sound");
      HCR.Stimulate(1,1);    
    }
  if (PWMvalue <= 1699 && PWMvalue >= 1650){
      Serial.println("Matrix Sw4: Mad Sound");
      HCR.Stimulate(2,1);
    }
    if (PWMvalue <= 1649 && PWMvalue >= 1600){
      Serial.println("Matrix Sw5: Scared Sound");
    HCR.Stimulate(3, 1);    
    }
    if (PWMvalue <= 1599 && PWMvalue >= 1550){
    Serial.println("Matrix Sw6: Fart Sound");
    HCR.PlayWAV(1, 4);    
    }
    if (PWMvalue <= 1549 && PWMvalue >= 1500){
      Serial.println("T4 Left: All Open");
      allOpen();  
    }
    if (PWMvalue <= 1499 && PWMvalue >= 1450){
      Serial.println("T5 Left: Wave Utility Arm");
      WaveUtilityArm();

    }
    if (PWMvalue <= 1449 && PWMvalue >= 1400){
      Serial.println("T5 Right: Drawer Wave");
      drawerWave();
    }    
    if (PWMvalue <= 1399 && PWMvalue >= 1350){
      Serial.println("T3 Up: Function 10:  Harlem Shake");
      HarlemShake();
    }
    if (PWMvalue <= 1349 && PWMvalue >= 1300){
      Serial.println("T4 Right: Function 11:  Close all Doors(stop animation)");
      allClose();
      sendESPNOWCommand("DP", ":SUS:PH");
    }
    if (PWMvalue <= 1299 && PWMvalue >= 1250){
      Serial.println("T3 Down: Function 12:  Panel Wave ");
      panelWave();
    }
    if (PWMvalue <= 1249 && PWMvalue >= 1200){
      Serial.println("T2 Up: Function 13:  Periscope Seq 4  ");
      sendESPNOWCommand("DP", ":SUS:PS4");
    }
    if (PWMvalue <= 1199 && PWMvalue >= 1150){
      Serial.println("T2 Down: Function 14: Short Circuit ");
      CompleteshortCircuit();
    }
    if (PWMvalue <= 1149 && PWMvalue >= 1100){
      Serial.println("T6 Left: Function 15: Door open with easing ");
      OpenClosewithEasing();
    }
    if (PWMvalue <= 1099 && PWMvalue >= 1050){
      Serial.println("T6 Right: Function 16: Fast Wave ");
      panelWaveFast();
    }
    if (PWMvalue <= 1049 && PWMvalue >= 1000){
      Serial.println("T1 Left: Function 17:  Display Mode ");
      display();
    }
    if (PWMvalue <= 899 && PWMvalue >= 850){
      Serial.println("T1 Right: Function 18: Open Close Wave ");
      openCloseWave();
    }
    if (PWMvalue <= 849 && PWMvalue >= 800){
      Serial.println("Function 19:  ");
    }
   } else if (sbusValues[CHANNEL_16_ARRAY_INDEX] <= 999 && sbusValues[CHANNEL_16_ARRAY_INDEX] >=201){ // switch in middle
    if (PWMvalue >= 100){
      Channel6Bool = true;
    } else {Channel6Bool = false;}
        
    Channel6State = Channel6Bool;
    if (Channel6State != lastChannel6State){
      if (Channel6State = true){
        if (PWMvalue >= 1800){
          Serial.println("Matrix SW 1: Function 1: Stop Sounds ");
          HCR.StopWAV(1);
          HCR.StopEmote();  
        }
      if (PWMvalue <= 1799 && PWMvalue >= 1750){
      Serial.println("Matrix SW 2: Function 2: Dance Star Wars Song");
           HCR.PlayWAV(1, 204);

    }
    if (PWMvalue <= 1749 && PWMvalue >= 1700){
      Serial.println("Matrix SW 3: Function 3: Leia Long");
        HCR.PlayWAV(1, 3);

    }
  if (PWMvalue <= 1699 && PWMvalue >= 1650){
      Serial.println("Matrix SW 4: Function 4: Star Wars Theme ");
      HCR.PlayWAV(1, 1);
    }
    if (PWMvalue <= 1649 && PWMvalue >= 1600){
      Serial.println("Matrix SW 5: Function 5:  Vader Theme");
      HCR.PlayWAV(1, 2);
    }
    if (PWMvalue <= 1599 && PWMvalue >= 1550){
      Serial.println("Matrix SW 6: Function 6: Cantina Song ");
      HCR.PlayWAV(1, 203);
    }
    if (PWMvalue <= 1549 && PWMvalue >= 1500){
      Serial.println("T4 Left: Flutter");
      allFlutter();
      }
    if (PWMvalue <= 1499 && PWMvalue >= 1450){
      Serial.println("T5 Left: ");
      cpuSequence();
    }
    if (PWMvalue <= 1449 && PWMvalue >= 1400){
      Serial.println("T5 Right: ");
      }    
    if (PWMvalue <= 1399 && PWMvalue >= 1350){
      Serial.println("T3 Up: Function 10:  Periscope Up and Lights On");
      sendESPNOWCommand("DP", ":SUS:PP100:L0");
    }
    if (PWMvalue <= 1349 && PWMvalue >= 1300){
      Serial.println("T4 Right: Function 11:  smoke sequence");
      SmokeSequence();
    }
    if (PWMvalue <= 1299 && PWMvalue >= 1250){
      Serial.println("T3 Down: Function 12:  Periscope Home");
      sendESPNOWCommand("DP", ":SUS:PH");
    }
    if (PWMvalue <= 1249 && PWMvalue >= 1200){
      Serial.println("T2 Up: Function 13: Rotate Random Periscope ");
      sendESPNOWCommand("DP", ":SUS:PAR,50");
    }
    if (PWMvalue <= 1199 && PWMvalue >= 1150){
      Serial.println("T2 Down: Function 14: Periscope Sequence 10 ");
      sendESPNOWCommand("DP", ":SUS:PS10");
    }
    if (PWMvalue <= 1149 && PWMvalue >= 1100){
      Serial.println("T6 Left: Function 15:  Long Dance ");
      longDance();
    }
    if (PWMvalue <= 1099 && PWMvalue >= 1050){
      Serial.println("T6 Right: Function 16: ");
    }
    if (PWMvalue <= 1049 && PWMvalue >= 1000){
      Serial.println("T1 Left: Function 17: ");
      fireExinguisher();
    }
    if (PWMvalue <= 899 && PWMvalue >= 850){
      Serial.println("T1 Right: Function 18: Launch Saber ");
      ArmSaber();

    }
      
        Serial.println("Switch selection in the middle");

      
      
      }
          lastChannel6State = Channel6State;

      }

    
  }
  else if (sbusValues[CHANNEL_16_ARRAY_INDEX] <= 1900 && sbusValues[CHANNEL_16_ARRAY_INDEX] >=1000){
    Serial.println("Switch selection in the top");
 if (PWMvalue >= 1800){
      Channel6Bool = true;
    } else {Channel6Bool = false;}
        if (PWMvalue >= 1800){
    Channel6State = Channel6Bool;
    if (Channel6State != lastChannel6State){
      if (Channel6State = true){
          Serial.println("Matrix SW 1: Function 1: Stop Sounds ");
          HCR.StopWAV(1);
          HCR.StopEmote();  
        }
        }
      }
      
    lastChannel6State = Channel6State;

    if (PWMvalue <= 1799 && PWMvalue >= 1750){
      Serial.println("Matrix SW 2: Whistle ");
      HCR.PlayWAV(1, 8);    
      }
    if (PWMvalue <= 1749 && PWMvalue >= 1700){
      Serial.println("Matrix SW 3: Staying Alive Song ");
      HCR.PlayWAV(1, 201);    
    }
  if (PWMvalue <= 1699 && PWMvalue >= 1650){
      Serial.println("Matrix SW 4: Function 4:  ");
      HCR.PlayWAV(1, 100);    

    }
    if (PWMvalue <= 1649 && PWMvalue >= 1600){
      Serial.println("Matrix SW 5: Function 5:  ");
      HCR.PlayWAV(1, 101);    

    }
    if (PWMvalue <= 1599 && PWMvalue >= 1550){
      Serial.println("Matrix SW 6: Function 6:  ");
      HCR.PlayWAV(1, 202);    


      int trackCount = HCR.GetWAVCount();
      Serial.println(trackCount);
      Serial.println(HCR.GetMuse());
    }
    if (PWMvalue <= 1549 && PWMvalue >= 1500){
      Serial.println("T4 Left: ");
      cpuRaise();
    }
    if (PWMvalue <= 1499 && PWMvalue >= 1450){
      Serial.println("T5 Left: ");
      cpuExtend();
    }
    if (PWMvalue <= 1449 && PWMvalue >= 1400){
      Serial.println("T5 Right: ");
      cpuRetract();
    }    
    if (PWMvalue <= 1399 && PWMvalue >= 1350){
      Serial.println("T3 Up: Function 10:  ");
      cpuSequence();
    }
    if (PWMvalue <= 1349 && PWMvalue >= 1300){
      Serial.println("T4 Right: Function 11:  ");
      cpuLower();
    }
    if (PWMvalue <= 1299 && PWMvalue >= 1250){
      Serial.println("T3 Down: Function 12:  ");
      cpuRotate();
    }
    if (PWMvalue <= 1249 && PWMvalue >= 1200){
      Serial.println("T2 Up: Function 13:  ");
    }
    if (PWMvalue <= 1199 && PWMvalue >= 1150){
      Serial.println("T2 Down: Function 14:  ");
    }
    if (PWMvalue <= 1149 && PWMvalue >= 1100){
      Serial.println("T6 Left: Function 15:  ");
    }
    if (PWMvalue <= 1099 && PWMvalue >= 1050){
      Serial.println("T6 Right: Function 16: ");
    }
    if (PWMvalue <= 1049 && PWMvalue >= 1000){
      Serial.println("T1 Left: Function 17: ");
      StowSaber();
    }
    if (PWMvalue <= 899 && PWMvalue >= 850){
      Serial.println("T1 Right: Function 18: ");
      LaunchSaber();
    }
  Serial.println("Switch selection in the middle");
  }

  }



void RCRadio_HCRVolChange(int chan, int sbusPos){
  int RCVocalizerVolume = map( sbusValues[sbusPos], 170, 1820, 0, 100);
  Serial.print("HVR Vocalizer Volume Mapped to : "); 
  Serial.println(RCVocalizerVolume);
  HCR.SetVolume(chan, RCVocalizerVolume);
}

#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////                                                                                       /////////     
/////////                             END OF FUNCTIONS                                          /////////
/////////                                                                                       /////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


 
void setup(){
  //Initialize the Serial Ports
  Serial.begin(115200);
  rdSerial.begin(RD_BAUD_RATE,SERIAL_8N1,SERIAL_RX_RD_PIN,SERIAL_TX_RD_PIN);
  blSerial.begin(BL_BAUD_RATE,SWSERIAL_8N1,SERIAL_RX_BL_PIN,SERIAL_TX_BL_PIN,false,95);
  stSerial.begin(ST_BAUD_RATE,SWSERIAL_8N1,SERIAL_RX_ST_PIN,SERIAL_TX_ST_PIN,false,95);
  mpSerial.begin(MP_BAUD_RATE,SWSERIAL_8N1,SERIAL_RX_MP_PIN,SERIAL_TX_MP_PIN,false,95);
  s1Serial.begin(SERIAL1_BAUD_RATE,SWSERIAL_8N1,SERIAL1_RX_PIN,SERIAL1_TX_PIN,false,95);
  s2Serial.begin(SERIAL2_BAUD_RATE,SWSERIAL_8N1,SERIAL2_RX_PIN,SERIAL2_TX_PIN,false,95);
  
  #ifdef SBUS
  sbus_rx.Begin();
  sbus_tx.Begin();
  #endif
  Serial.println("\n\n\n----------------------------------------");
  Serial.println("Booting up the Body ESP Controller");
  
  //Configure the Reset Pins for the arduinoReset() function
  pinMode(RST, OUTPUT);
  digitalWrite(RST,HIGH);

  //Initialize I2C for the Servo Expander Board
//  Wire.begin();
  
  //Initialize the ReelTwo Library
//  SetupEvent::ready();

  ESP_LED.begin();
  ESP_LED.show();
  colorWipeStatus("ES",red,255);
  Serial.println("LED Setup Complete");


  //Reserve the inputStrings
  inputString.reserve(200);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(200);

  //Initialize the Soft Access Point
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
  etmInit(ESPNOWPASSWORD.c_str(), ETM_MY_BOARD_INDEX);

 
  //Initialize the AsycWebServer
  //  server.begin();

  //Reset Arudino Mega
  resetArduino(500);


}  //end of Setup


void loop(){
  etmProcess();
  keepAlive();
  checkAgeofkeepAlive();
#ifdef SBUS
 processSbus();
#endif
  if (millis() - MLMillis >= mainLoopDelayVar){
    MLMillis = millis();
    AnimatedEvent::process();
    if(startUp) {
      colorWipeStatus("ES",blue,10);

      startUp = false;
      Serial.println("Startup completed, now running loop");
      // Play Startup Sound
      // mp3Trigger("v",16);
      // mp3Trigger("t",1);
      // mp3Trigger("v",0);

    }
    if(Serial.available()){serialEvent();}
    // if(rdSerial.available()){serialRdEvent();}
    if(blSerial.available()){serialBlEvent();}
    if(stSerial.available()){serialStEvent();}
    if(mpSerial.available()){serialMpEvent();}
    if(s1Serial.available()){serial1Event();}
    if(s1Serial.available()){serial2Event();}

    
    if (stringComplete) {autoComplete=false;}
    if (stringComplete || autoComplete) {
      if(stringComplete) {inputString.toCharArray(inputBuffer, 200);inputString="";}
      else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 200);autoInputString="";}
      if (inputBuffer[0] == '#'){
        if (
            inputBuffer[1]=='D' ||          // Command for debugging
            inputBuffer[1]=='d' ||          // Command for debugging
            inputBuffer[1]=='L' ||          // Command designator for internal functions
            inputBuffer[1]=='l' ||          // Command designator for internal functions
            inputBuffer[1]=='E' ||          // Command designator for storing EEPROM data
            inputBuffer[1]=='e'             // Command designator for storing EEPROM data

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
                internalCommandFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');
                Internal_Command[0]   = '\0';                                                            // Flushes Array
                Internal_Command[0] = internalCommandFunction;
              Debug.LOOP("Entered the Local Command Structure /n");
              } else if (inputBuffer[1] == 'E' || inputBuffer[1] == 'e'){
                Debug.LOOP("EEPROM configuration selected /n");
                // need to actually add the code to implement this.
                if (inputBuffer[2] == 'E'){
                  for (int i=2; i<=commandLength-2; i++){
                  char inCharRead = inputBuffer[i];
                  eepromCommandString += inCharRead;                   // add it to the inputString:
                  }
                }
                if (inputBuffer[2] == 'A'){
                  for (int i=3; i<=commandLength-1; i++){
                    char inCharRead = inputBuffer[i];
                    eepromCommandString += inCharRead;                   // add it to the inputString:
                  }
                  writeBlSerial(eepromCommandString);
                  eepromCommandString = "";
                }
              } else {Debug.LOOP("No valid command entered /n");}
              
          }
              if(Internal_Command[0]){
                switch (Internal_Command[0]){
                  case 1: Serial.println(HOSTNAME);
                        Internal_Command[0]   = '\0';                                                           break;
                  case 2: Serial.println("Resetting the ESP in 3 Seconds");
                        //  DelayCall::schedule([] {ESP.restart();}, 3000);
                        ESP.restart();
                        Internal_Command[0]   = '\0';                                                           break;
                  case 3: break;  //reserved for commonality. Used for connecting to WiFi and enabling OTA on ESP-NOW Boards 
                  case 4: break;  //reserved for future use
                  case 5:   ;                                                                    break;  //reserved for future use
                  case 6:   ;                                                                   break;  //reserved for future use
                  case 7: ;                  break;  //reserved for future use
                  case 8: printKeepaliveStatus();                                                           break;  //reserved for future use
                  case 9:  break;  //reserved for future use

                }
              }

        } else if (inputBuffer[0] == ':'){
     
          if( inputBuffer[1]=='E'     ||        // Command designator for sending ESP-NOW messages
              inputBuffer[1]=='e'     ||        // Command designator for sending ESP-NOW messages
              inputBuffer[1]=='S'     ||        // Command designator for sending Serial Strings out Serial ports
              inputBuffer[1]=='s'     ||        // Command designator for sending Serial Strings out Serial ports
              inputBuffer[1]=='L'     ||        // Command designator for LED Control
              inputBuffer[1]=='l'     ||        // Command designator for LED Control
              inputBuffer[1]=='A'     ||        // Command designator for Animations
              inputBuffer[1]=='a'     ||        // Command designator for Animations
              inputBuffer[1]=='M'     ||        // Command designator for MP3 Commands(HCR Vocalizer)
              inputBuffer[1]=='m'     ||        // Command designator for MP3 Commands(HCR Vocalizer)
              inputBuffer[1]=='R'     ||        // Command designator for Roam-A-Dome Home
              inputBuffer[1]=='r'     ||        // Command designator for Roam-A-Dome Home
              inputBuffer[1]=='C'     ||        // Command designator for Main Controller(Stealth/Shadow)
              inputBuffer[1]=='c'               // Command designator for Main Controller(Stealth/Shadow)
            ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.
              Debug.DBG("Command: %s with a length of %d \n", inputBuffer, commandLength);

              if(commandLength >= 3) {
     
                if(inputBuffer[1]=='E' || inputBuffer[1]=='e') {
                  for (int i=2; i<=commandLength; i++){
                    char inCharRead = inputBuffer[i];
                    ESPNOWCommandString += inCharRead;                   // add it to the inputString:
                    }
                  Debug.LOOP("\nFull Command Recieved: %s \n",ESPNOWCommandString.c_str());
                  ESPNOWTarget = ESPNOWCommandString.substring(0,2);
                  Debug.LOOP("ESP NOW Target: %s\n", ESPNOWTarget.c_str());
                  ESPNOWTargetCommand = ESPNOWCommandString.substring(2,commandLength+1);
                  Debug.LOOP("Command to Forward: %s\n", ESPNOWTargetCommand.c_str());
                  sendESPNOWCommand(ESPNOWTarget, ESPNOWTargetCommand);
                  // reset ESP-NOW Variables
                  ESPNOWCommandString = "";
                  ESPNOWTargetCommand = "";
                  ESPNOWTarget = "";  
                } 
                if(inputBuffer[1]=='S' || inputBuffer[1]=='s') {
                  for (int i=2; i<commandLength-1;i++ ){
                    char inCharRead = inputBuffer[i];
                    serialCommandString += inCharRead;  // writeS1Serialadd it to the inputString:
                  }
                  Debug.DBG("Full Serial Command Captured: %s\n", serialCommandString.c_str());
                  serialPort = serialCommandString.substring(0,2);
                  serialCommandSubString = serialCommandString.substring(2,commandLength);
                  Debug.DBG("Serial Command: %s to Serial Port: %s\n", serialCommandSubString.c_str(), serialPort);
                  if (serialPort == "S1"){
                    writeS1Serial(serialCommandSubString);
                    Debug.DBG("Sending out AUX1 Serial\n");
                  } else if (serialPort == "S2"){
                    writeS2Serial(serialCommandSubString);
                    Debug.DBG("Sending out Aux 2 Serial\n");
                  } else if (serialPort == "MP"){
                    writeMpSerial(serialCommandSubString);
                    Debug.DBG("Sending out HCR\n");
                  } else { Debug.DBG("No valid Serial port identified\n");}
                  serialCommandString = "";
                  serialPort = "";
                  serialCommandSubString = "";
                }
                if(inputBuffer[1]=='L' || inputBuffer[1]=='l') {
                  for (int i=2; i<commandLength;i++ ){
                    char inCharRead = inputBuffer[i];
                    ledCommandString += inCharRead;
                      // add it to the inputString:
                  }              
                  writeBlSerial(ledCommandString);
                  Debug.LOOP("Sent ATMEGA2560 command of %s \n", ledCommandString);
                  ledCommandString = "";
                }
                if(inputBuffer[1] == 'A' || inputBuffer[1] == 'a'){
                    AnimationCommandFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');
                    Animation_Command[0]   = '\0';                                                            // Flushes Array
                    Animation_Command[0] = AnimationCommandFunction;
                }
                if(inputBuffer[1]=='R' || inputBuffer[1]=='r') {
                    for (int i=2; i<commandLength;i++ ){
                      char inCharRead = inputBuffer[i];
                      radhCommandString += inCharRead;  // add it to the inputString:
                    }              
                    writeRdSerial(radhCommandString);
                    Debug.LOOP("Sent RADH command of %s \n", radhCommandString);
                    radhCommandString = "";
                }
                if(inputBuffer[1]=='M' || inputBuffer[1]=='m') {
                  for (int i=2; i<commandLength;i++ ){
                    char inCharRead = inputBuffer[i];
                    mp3CommandString += inCharRead;  // add it to the inputString:
                  }              
                  String hcrCommandFunction = getValue(mp3CommandString, ',', 0);
                  String hcrCommandChannel = getValue(mp3CommandString, ',', 1);
                  String hcrCommandTrack = getValue(mp3CommandString, ',', 2);
                  String hcrCommandString = getValue(mp3CommandString, ',', 3);

                  Debug.LOOP("HCR Function %i \n", hcrCommandFunction.toInt());
                  Debug.LOOP("HCR Channel: %i \n", hcrCommandChannel.toInt());
                  Debug.LOOP("HCR Track: %i \n", hcrCommandTrack.toInt());
                  Debug.LOOP("HCR String: %s \n", hcrCommandString.c_str());
                  HCRFunction(hcrCommandFunction.toInt(), hcrCommandChannel.toInt(), hcrCommandTrack.toInt(),hcrCommandTrack);
                  // writeMpSerial(mp3CommandString);
                  // Debug.LOOP("Sent HCR command of %s \n", mp3CommandString);
                  mp3CommandString = "";
                }
                if(inputBuffer[1]=='C' || inputBuffer[1]=='c') {
                  for (int i=2; i<commandLength;i++ ){
                    char inCharRead = inputBuffer[i];
                    controllerCommandString += inCharRead;  // add it to the inputString:
                  }              
                  writeStSerial(controllerCommandString);
                  Debug.LOOP("Sent Stealth command of %s \n", controllerCommandString);
                  controllerCommandString = "";
                }
              }
            }
        
          if(Animation_Command[0]){
            switch (AnimationCommandFunction) {
              case 1: normalOperations();                                   break;
              case 2: panelWave();                                          break;
              case 3: panelWaveFast();                                      break;
              case 4: domePeriscope();                                      break;
              case 5: allOpenClose();                                       break;
              case 6: HarlemShake();                                        break;
              case 7: allClose();                                           break;
              case 8: allFlutter();                                         break;
              case 9: toggleDoors();                                        break;
              case 10: allLightsToggle();                                   break;
              case 11: allOpen();                                           break;
              case 12: drawerWave();                                        break;
              case 13: OpenClosewithEasing();                               break;
              case 14: CompleteshortCircuit();                              break;
              case 15: StarWarsThemeSong();                                 break;
              case 16: VaderThemeSong();                                    break;
              case 17: PeriscopeSeq2();                                     break;
              case 18: PerisopeSeq10();                                     break;
              case 19: LightsOff();                                         break;
              case 20: LightsOn();                                          break;
              case 21: FartNoise();                                         break;
              case 22: WaveUtilityArm();                                    break;
              case 23: ArmSaber();                                          break;
              case 24: LaunchSaber();                                       break;
              case 25: SmokeSequence();                                     break;
              case 26: break;
              case 27: break;
              case 28: break;
              case 29: break;
              case 30: break;
              case 31: break;
              case 32: break;
              case 33: break;
              case 34: break;
              case 35: break;
              case 36: break;
              case 37: break;
              case 38: break;
              case 39: break;
              case 40: break;

            
            }
          }       
        }



      ///***  Clear States and Reset for next command.  ***///
        stringComplete =false;
        autoComplete = false;
        inputBuffer[0] = '\0';
        inputBuffer[1] = '\0';



    }


    
    if(isStartUp) {
      isStartUp = false;
      delay(500);
    }
  }
}  // end of main loop
