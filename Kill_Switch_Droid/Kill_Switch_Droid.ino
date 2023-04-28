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
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <AsyncTCP.h>
#include <WiFi.h>

//Used for ESP-NOW
#include "esp_wifi.h"
#include <esp_now.h>

//Used for status LEDs
#include <Adafruit_NeoPixel.h>

//Used for pin definition
#include "kill_switch_droid_pin_map.h"

// Debug Functions  - Using my own library for this
#include <DebugR2.h>  //  https://github.com/greghulette/Arduino-Code/tree/main/libraries/DebugR2  Put these files in a folder called "DebugR2" in your libraries folder and restart the IDE

//ReelTwo libaries - Using my forked version of this libarary
#include <ReelTwo.h>
#include "core/DelayCall.h"

//Used for LoRa
#include <SPI.h>
#include "LoRa.h"  // Using the ReelTwo LoRa library.  Files included in this sketch's folder.






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

// used to sync timing with the dome controller better, allowing time for the ESP-NOW messages to travel to the dome
// Change this to work with how your droid performs
  int defaultESPNOWSendDuration = 50;  

  // Serial Baud Rates
  #define SERIAL1_BAUD_RATE 115200







//////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
  String HOSTNAME = " Kill Switch (Droid) LoRa to ESP-NOW Gateway";

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
  bool bodyControllerStatus = 0;
  bool bodyLEDControllerStatus = 0;  
  bool bodyServoControllerStatus = 0;
  bool domePlateControllerStatus = 0;
  bool domeControllerStatus = 0;
  bool droidRemoteStatus = 0;
  bool hpControllerStatus = 0;
  

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


  //////////////////////////////////////////////////////////////////
  ///******       Serial Ports Definitions                  *****///
  //////////////////////////////////////////////////////////////////

  #define s1Serial Serial1







  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////

//LoRa Remote ESP           192.168.4.101   
//LoRa Droid ESP            192.168.4.108    ************ (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Controller ESP       192.168.4.109    (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Servo ESP            192.168.4.110   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Controller ESP       192.168.4.111   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Plate Controller ESP 192.168.4.112   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Droid Raspberry Pi        192.168.4.113
//Remote Raspberry Pi       192.168.4.114
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

//  ESP-NOW MAC Addresses used in the Droid. 
  const uint8_t droidLoRaMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
  const uint8_t bodyControllerMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
  const uint8_t bodyServosControllerMACAddress[] =  {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};
  const uint8_t domeControllerMACAddress[]=  {0x02, 0x00, 0x00, 0x00, 0x00, 0x04};
  const uint8_t domePlateControllerMACAddress[] =   {0x02, 0x00, 0x00, 0x00, 0x00, 0x05};
  const uint8_t hpControllerMACAddress[] =   {0x02, 0x00, 0x00, 0x00, 0x00, 0x06};
  const uint8_t broadcastMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Uses these Strings for comparators
  String droidLoRaMACAddressString =            "02:00:00:00:00:01";
  String bodyControllerMACAddressString =       "02:00:00:00:00:02";
  String bodyServosControllerMACAddressString = "02:00:00:00:00:03";
  String domeControllerMACAddressString =       "02:00:00:00:00:04";
  String domePlateControllerMACAddressString =  "02:00:00:00:00:05";
  String hpControllerMACAddressString =         "02:00:00:00:00:06";
  String broadcastMACAddressString =            "FF:FF:FF:FF:FF:FF";

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
  bool incomingstructbodyLEDControllerStatus;
  
// Variable to store if sending data was successful
  String success;

//Structure example to send data
//Must match the receiver structure
typedef struct espnow_struct_message {
      char structPassword[20];
      char structSenderID[4];
      char structTargetID[4];
      bool structCommandIncluded;
      char structCommand[100];
  } espnow_struct_message;

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
      bool structCommandIncluded;
      char structCommand[100];
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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (Debug.debugflag_espnow == 1){
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status ==0){
      success = "Delivery Success :)";
    }
    else{
      success = "Delivery Fail :(";
    }
  }
}
//   Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  colorWipeStatus("ES",orange,255);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  String IncomingMacAddress(macStr);
  Debug.ESPNOW("Recieved ESP-NOW packet from %s \n", IncomingMacAddress.c_str());
  if (IncomingMacAddress == droidLoRaMACAddressString) {
      memcpy(&commandsToReceiveFromDroidLoRa, incomingData, sizeof(commandsToReceiveFromDroidLoRa));
      incomingPassword == commandsToReceiveFromDroidLoRa.structPassword;
      if (incomingPassword != ESPNOWPASSWORD){
        Debug.ESPNOW("Wrong ESP-NOW Password was sent from DL.  Message Ignored\n");  
      } else {
        incomingSenderID = commandsToReceiveFromDroidLoRa.structSenderID;
        incomingTargetID = commandsToReceiveFromDroidLoRa.structTargetID;
        incomingCommandIncluded = commandsToReceiveFromDroidLoRa.structCommandIncluded;
        incomingCommand = commandsToReceiveFromDroidLoRa.structCommand;
        processESPNOWIncomingMessage();
        }
    } else if (IncomingMacAddress == bodyControllerMACAddressString){
        Debug.ESPNOW("ESP-NOW Packet Size from BC: %i \n", len);
      memcpy(&commandsToReceiveFromBodyController, incomingData, sizeof(commandsToReceiveFromBodyController));
      incomingPassword = commandsToReceiveFromBodyController.structPassword;
      if (incomingPassword != ESPNOWPASSWORD){
        Debug.ESPNOW("Incoming Password Received:  %s\n", incomingPassword);
        Debug.ESPNOW("Wrong ESP-NOW Password was sent from BC.  Message Ignored\n");  
      } else {
        incomingSenderID = commandsToReceiveFromBodyController.structSenderID;
        incomingTargetID = commandsToReceiveFromBodyController.structTargetID;
        BL_LDP_Bright = commandsToReceiveFromBodyController.structBL_LDP_Bright;
        BL_MAINT_Bright = commandsToReceiveFromBodyController.structBL_MAINT_Bright;
        BL_VU_Bright = commandsToReceiveFromBodyController.structBL_VU_Bright;
        BL_CS_Bright = commandsToReceiveFromBodyController.structBL_CS_Bright;
        BL_vuOffsetInt = commandsToReceiveFromBodyController.structBL_vuOffsetInt;
        BL_vuBaselineInt = commandsToReceiveFromBodyController.structBL_vuBaselineInt;
        BL_vuOffsetExt = commandsToReceiveFromBodyController.structBL_vuOffsetExt;
        BL_vuBaselineExt = commandsToReceiveFromBodyController.structBL_vuBaselineExt;
        BL_BatteryVoltage = commandsToReceiveFromBodyController.structBL_BatteryVoltage;
        BL_BatteryPercentage = commandsToReceiveFromBodyController.structBL_BatteryPercentage;
        bodyLEDControllerStatus = commandsToReceiveFromBodyController.structbodyLEDControllerStatus;
        incomingCommandIncluded = commandsToReceiveFromBodyController.structCommandIncluded;
        incomingCommand = commandsToReceiveFromBodyController.structCommand;
        processESPNOWIncomingMessage();
        }
    }else if (IncomingMacAddress == bodyServosControllerMACAddressString) {
      memcpy(&commandsToReceiveFromBodyServoController, incomingData, sizeof(commandsToReceiveFromBodyServoController));
      incomingPassword = commandsToReceiveFromBodyServoController.structPassword;
      if (incomingPassword != ESPNOWPASSWORD){
        Debug.ESPNOW("Wrong ESP-NOW Password was sent from BS.  Message Ignored\n");  
      } else {
        incomingSenderID = commandsToReceiveFromBodyServoController.structSenderID;
        incomingTargetID = commandsToReceiveFromBodyServoController.structTargetID;
        incomingCommandIncluded = commandsToReceiveFromBodyServoController.structCommandIncluded;
        incomingCommand = commandsToReceiveFromBodyServoController.structCommand;
        processESPNOWIncomingMessage();
        }
    } else if (IncomingMacAddress == domeControllerMACAddressString) {
      memcpy(&commandsToReceiveFromDomeController, incomingData, sizeof(commandsToReceiveFromDomeController));
      incomingPassword = commandsToReceiveFromDomeController.structPassword;
      if (incomingPassword != ESPNOWPASSWORD){
        Debug.ESPNOW("Wrong ESP-NOW Password was sent from DC.  Message Ignored\n");  
      } else {
        incomingSenderID = commandsToReceiveFromDomeController.structSenderID;
        incomingTargetID = commandsToReceiveFromDomeController.structTargetID;
        incomingCommandIncluded = commandsToReceiveFromDomeController.structCommandIncluded;
        incomingCommand = commandsToReceiveFromDomeController.structCommand;
        processESPNOWIncomingMessage();
        }
    } else if (IncomingMacAddress == domePlateControllerMACAddressString) {
      memcpy(&commandsToReceiveFromDomePlateController, incomingData, sizeof(commandsToReceiveFromDomePlateController));
      incomingPassword = commandsToReceiveFromDomePlateController.structPassword;
      if (incomingPassword != ESPNOWPASSWORD){
        Debug.ESPNOW("Wrong ESP-NOW Password was sent from DP.  Message Ignored\n");  
      } else {
        incomingSenderID = commandsToReceiveFromDomePlateController.structSenderID;
        incomingTargetID = commandsToReceiveFromDomePlateController.structTargetID;
        incomingCommandIncluded = commandsToReceiveFromDomePlateController.structCommandIncluded;
        incomingCommand = commandsToReceiveFromDomePlateController.structCommand;
        processESPNOWIncomingMessage();
        }
    } else if (IncomingMacAddress == broadcastMACAddressString) {
      memcpy(&commandsToReceiveFromBroadcast, incomingData, sizeof(commandsToReceiveFromBroadcast));
      incomingPassword = commandsToReceiveFromBroadcast.structPassword;
      if (incomingPassword != ESPNOWPASSWORD){
        Debug.ESPNOW("Wrong ESP-NOW Password was sent.  Message Ignored\n");  
      } else {
        incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
        incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
        incomingCommandIncluded = commandsToReceiveFromBroadcast.structCommandIncluded;
        incomingCommand = commandsToReceiveFromBroadcast.structCommand;
        processESPNOWIncomingMessage();
        }
    }  else if (IncomingMacAddress == hpControllerMACAddressString) {
      memcpy(&commandsToReceiveFromHPController, incomingData, sizeof(commandsToReceiveFromHPController));
      incomingPassword = commandsToReceiveFromHPController.structPassword;
      if (incomingPassword != ESPNOWPASSWORD){
        Debug.ESPNOW("Wrong ESP-NOW Password was sent.  Message Ignored\n");  
      } else {
        incomingSenderID = commandsToReceiveFromHPController.structSenderID;
        incomingTargetID = commandsToReceiveFromHPController.structTargetID;
        incomingCommandIncluded = commandsToReceiveFromHPController.structCommandIncluded;
        incomingCommand = commandsToReceiveFromHPController.structCommand;
        processESPNOWIncomingMessage();
        }
    }  else {Debug.ESPNOW("ESP-NOW Mesage ignored \n");}  
  colorWipeStatus("ES",blue,10);
  IncomingMacAddress ="";

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
        Debug.STATUS("Body LED Controller Status: %d \n", bodyLEDControllerStatus);
        Debug.ESPNOW("ESP NOW Message Received from Body Controller \n");

      }else if (incomingSenderID == "BS"){
        bodyServoControllerStatus = 1;
        bskeepAliveAge =millis();

        if (incomingCommandIncluded == 1){
          inputString = incomingCommand;
          stringComplete = true; 
        }

        Debug.STATUS("Body Servo Status Update \n");
        Debug.ESPNOW("ESP NOW Message Received from Body Servo Controller \n");
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
      } else if (incomingSenderID == "DP"){
        domePlateControllerStatus = 1;
        dpkeepAliveAge =millis();
        if (incomingCommandIncluded == 1){
          inputString = incomingCommand;
          stringComplete = true; 
        }
        Debug.STATUS("Dome Plate Status Update \n");
        Debug.ESPNOW("ESP NOW Message Received from Dome Plate Controller \n");
      } else if (incomingSenderID == "HP"){
        hpControllerStatus = 1;
        hpkeepAliveAge =millis();
        if (incomingCommandIncluded == 1){
          inputString = incomingCommand;
          stringComplete = true; 
        }
        Debug.STATUS("HP Controller Status Update \n");
        Debug.ESPNOW("ESP NOW Message Received from HP Controller \n");
      } else {Debug.ESPNOW("No Valid source identified \n");}

    } else {Debug.ESPNOW("No matching target ID \n");}
}

//////////////////////////////////////////////////////////////
///*****        LoRa Variables       *****///
//////////////////////////////////////////////////////////////////////
  String outgoing;
  String LoRaOutgoing;

  boolean oldState = HIGH;
  boolean newState ;

  byte msgCount = 0;            // count of outgoing messages
  byte localAddress = 0xBC;     // address of this device
  byte destination = 0xFF;      // destination to send to
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
  
  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
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
      stringComplete = true;            // set a flag so the main loop can do something about it.
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

void setupSendStruct(espnow_struct_message* msg, String pass, String sender, String targetID, bool hascommand, String cmd)
{
    snprintf(msg->structPassword, sizeof(msg->structPassword), "%s", pass.c_str());
    snprintf(msg->structSenderID, sizeof(msg->structSenderID), "%s", sender.c_str());
    snprintf(msg->structTargetID, sizeof(msg->structTargetID), "%s", targetID.c_str());
    msg->structCommandIncluded = hascommand;
    snprintf(msg->structCommand, sizeof(msg->structCommand), "%s", cmd.c_str());
};

void sendESPNOWCommand(String starget, String scomm){
  String senderID = "DG";   // change to match location (BC/BS/DC/DP/LD)
  String scommEval = "";
  bool hasCommand;
  if (scommEval = scomm){
    hasCommand = 0;
  } else {hasCommand = 1;};

  if (starget == "DG"){
    setupSendStruct(&commandsToSendtoDroidLoRa, ESPNOWPASSWORD, senderID, starget, hasCommand, scomm);
    esp_err_t result = esp_now_send(droidLoRaMACAddress, (uint8_t *) &commandsToSendtoDroidLoRa, sizeof(commandsToSendtoDroidLoRa));
    if (result == ESP_OK) {Debug.ESPNOW("Sent the command: %s to ESP-NOW LoRa Droid Neighbor\n", scomm.c_str());
    }else {Debug.ESPNOW("Error sending the data\n");}
  } else if (starget == "BC"){
    setupSendStruct(&commandsToSendtoBodyController, ESPNOWPASSWORD, senderID, starget, hasCommand, scomm);
    esp_err_t result = esp_now_send(bodyControllerMACAddress, (uint8_t *) &commandsToSendtoBodyController, sizeof(commandsToSendtoBodyController));
    if (result == ESP_OK) {Debug.ESPNOW("Sent the command: %s to ESP-NOW Neighbor \n", scomm.c_str());
    }else {Debug.ESPNOW("Error sending the data\n");}
  } else if (starget == "BS"){
    setupSendStruct(&commandsToSendtoBodyServoController, ESPNOWPASSWORD, senderID, starget, hasCommand, scomm);
       esp_err_t result = esp_now_send(bodyServosControllerMACAddress, (uint8_t *) &commandsToSendtoBodyServoController, sizeof(commandsToSendtoBodyServoController));
    if (result == ESP_OK) {Debug.ESPNOW("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
    }else {Debug.ESPNOW("Error sending the data\n");}
  }  else if (starget == "DC"){
    setupSendStruct(&commandsToSendtoDomeController, ESPNOWPASSWORD, senderID, starget, hasCommand, scomm);
       esp_err_t result = esp_now_send(domeControllerMACAddress, (uint8_t *) &commandsToSendtoDomeController, sizeof(commandsToSendtoDomeController));
    if (result == ESP_OK) {Debug.ESPNOW("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
    }else {Debug.ESPNOW("Error sending the data\n");}
  } else if (starget == "DP"){
    setupSendStruct(&commandsToSendtoDomePlateController, ESPNOWPASSWORD, senderID, starget, hasCommand, scomm);
       esp_err_t result = esp_now_send(domePlateControllerMACAddress, (uint8_t *) &commandsToSendtoDomePlateController, sizeof(commandsToSendtoDomePlateController));
    if (result == ESP_OK) {Debug.ESPNOW("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
    }else {Debug.ESPNOW("Error sending the data\n");}
  } else if (starget == "BR"){
    setupSendStruct(&commandsToSendtoBroadcast, ESPNOWPASSWORD, senderID, starget, hasCommand, scomm);
       esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
    if (result == ESP_OK) {Debug.ESPNOW("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
    }else {Debug.ESPNOW("Error sending the data\n");}
  } else if (starget == "HP"){
    setupSendStruct(&commandsToSendtoHPController, ESPNOWPASSWORD, senderID, starget, hasCommand, scomm);
       esp_err_t result = esp_now_send(hpControllerMACAddress, (uint8_t *) &commandsToSendtoHPController, sizeof(commandsToSendtoHPController));
    if (result == ESP_OK) {Debug.ESPNOW("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
    }else {Debug.ESPNOW("Error sending the data\n");}
  }else {Debug.ESPNOW("No valid destination \n");}
};

//////////////////////////////////////////////////////////////////////
///*****             LoRa Functions                           *****///
//////////////////////////////////////////////////////////////////////

void sendStatusMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  
  LoRa.write(droidGatewayStatus);
  LoRa.write(RELAY_STATUS);
  LoRa.write(bodyControllerStatus);
  LoRa.write(bodyLEDControllerStatus);
  LoRa.write(bodyServoControllerStatus);
  LoRa.write(domePlateControllerStatus);
  LoRa.write(domeControllerStatus);
  LoRa.write(BL_LDP_Bright);
  LoRa.write(BL_MAINT_Bright);
  LoRa.write(BL_VU_Bright);
  LoRa.write(BL_CS_Bright);
  LoRa.write(BL_vuOffsetInt);
  LoRa.write(BL_vuBaselineInt);
  LoRa.write(BL_vuOffsetExt);
  LoRa.write(BL_vuBaselineExt);
  LoRa.write(BL_BatteryVoltage);
  LoRa.write(BL_BatteryPercentage);

  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  Debug.LORA("Status Sent\n");
  Local_Command[0]   = '\0';
};

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
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
  
  if(LoRa.packetRssi() > -50 && LoRa.packetRssi() < 10){
    colorWipeStatus("LS", green, 10);
  }else if (LoRa.packetRssi() > -100 && LoRa.packetRssi()  <= -50){
    colorWipeStatus("LS", yellow, 10);
  } else{ colorWipeStatus("LS", red, 10);}

  inputString = incoming;
  stringComplete = true; 
      // sendMessage("Message Revieved");
};


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
  Serial.print("Mode is: ");Serial.println(RELAY_STATUS);
  Local_Command[0]   = '\0';
}

void MainRelayOff(){
  RELAY_STATUS = LOW;
  digitalWrite(RELAY_CONTROL, RELAY_STATUS);
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

  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

// initialize WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &droidLoRaMACAddress[0]);
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

  // Add peers  
 // Broadcast
  memcpy(peerInfo.peer_addr, broadcastMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  }
  //   // Droid LoRa Controller
  // memcpy(peerInfo.peer_addr, droidLoRaMACAddress, 6);
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add Broadcast ESP-NOW peer");
  //   return;
  // }  
 // Body Controller
  memcpy(peerInfo.peer_addr, bodyControllerMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  }
   // Body Servo Controller
  memcpy(peerInfo.peer_addr, bodyServosControllerMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  }
// Dome Controller
  memcpy(peerInfo.peer_addr, domeControllerMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  } 
// Dome Plate Controller
  memcpy(peerInfo.peer_addr, domePlateControllerMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  }
    // HP Controller
  memcpy(peerInfo.peer_addr, hpControllerMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  }


  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);


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

  if (!LoRa.begin(915E6,true)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  checkAgeofkeepAlive();
  checkButton();
  onReceive(LoRa.parsePacket());
  oldState = newState;

  if (millis() - MLMillis >= mainLoopDelayVar){
    MLMillis = millis();
    if(startUp) {
      startUp = false;
      Serial.println("Startup completed, now running loop");
    }

  if(Serial.available()){serialEvent();}

  if (stringComplete) {autoComplete=false;}
  if (stringComplete || autoComplete) {
    if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
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
                        Local_Command[0]   = '\0';                                                           break;
                  case 2: Serial.println("Resetting the ESP in 3 Seconds");
                        //  DelayCall::schedule([] {ESP.restart();}, 3000);
                        ESP.restart();
                        Local_Command[0]   = '\0';                                                           break;
                  case 3: connectWiFi();                                                                          break;
                  case 4: break;  //reserved for future use
                  case 5: MainRelayOn();                                                                    break;  //reserved for future use
                  case 6: MainRelayOff();                                                                   break;  //reserved for future use
                  case 7: sendStatusMessage("Status Update");                  break;  //reserved for future use
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

}




