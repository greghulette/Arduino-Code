///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                                                                                                       *****///
///*****                            Created by Greg Hulette.  I started with the code from flthymcnsty         *****///
///*****                                                                                                       *****///
///*****                                     So exactly what does this all do.....?                            *****///
///*****                       - Receives commands via Serial or ESP-NOW                                       *****///
///*****                       - Controls the body servos                                                      *****///
///*****                                                                                                       *****///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>

//Used for ESP-NOW
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>

//Used for PC9685 - Servo Expansion Board
#include <Wire.h>

//reeltwo libaries
#include <ReelTwo.h>
#include "core/DelayCall.h"
#include "ServoDispatchPCA9685.h"
#include "ServoSequencer.h"

//Used for pin definition
#include "body_servo_pin_map.h"

//Used for the Status LED
#include <Adafruit_NeoPixel.h>


//////////////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
  String HOSTNAME = "Body Servo Controller";
  char inputBuffer[100];
  String inputString;         // a string to hold incoming data
  String inputStringCommand;
  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA
  int commandLength;
  String serialPort;
  String serialStringCommand;
  String serialSubStringCommand;

  uint32_t ESP_command[6]  = {0,0,0,0,0,0};
  int espCommandFunction = 0;

  uint32_t ESPNOW_command[6]  = {0,0,0,0,0,0};
  int espNowCommandFunction = 0;
  String espNowCommandFunctionString;
  String tempESPNOWTargetID;
  String ESPNOWPASSWORD = "GregsAstromech";
  int defaultESPNOWSendDuration = 50;

// Flags to enable/disable debugging in runtime
  boolean debugflag = 1;    // Normally set to 0, but leaving at 1 during coding and testing.
  boolean debugflag1 = 0;  // Used for optional level of debuging
  boolean debugflag_espnow = 0;
  boolean debugflag_servo = 0;
  boolean debugflag_serial_event = 0;
  boolean debugflag_loop = 0;
  boolean debugflag_http = 0;
  boolean debugflag_lora = 0;

 /////////////////////////////////////////////////////////////////////////
///*****              ReelTwo Servo Set Up                       *****///
/////////////////////////////////////////////////////////////////////////

#define TOP_UTILITY_ARM       0x0001 //b000000000001
#define BOTTOM_UTILITY_ARM    0x0002 //b000000000010
#define LARGE_LEFT_DOOR       0x0004 //b000000000100
#define LARGE_RIGHT_DOOR      0x0008 //b000000001000
#define CHARGE_BAY_DOOR       0x0010 //b000000010000
#define DATA_PANEL_DOOR       0x0020 //b000000100000
#define DRAWER_S1             0X0040 //b000001000000
#define DRAWER_S2             0x0080 //b000010000000
#define DRAWER_S3             0x0100 //b000100000000
#define DRAWER_S4             0x0200 //b001000000000
#define REAR_LEFT_DOOR        0x0400 //b010000000000
#define REAR_RIGHT_DOOR       0x0800 //b100000000000

#define UTILITY_ARMS_MASK     (TOP_UTILITY_ARM|BOTTOM_UTILITY_ARM)
#define LARGE_DOORS_MASK      (LARGE_LEFT_DOOR|LARGE_RIGHT_DOOR)
#define SMALL_DOORS_MASK      (CHARGE_BAY_DOOR|DATA_PANEL_DOOR)
#define DRAWERS_MASK          (DRAWER_S1|DRAWER_S2|DRAWER_S3|DRAWER_S4)
#define ALL_DOORS_MASK        (LARGE_DOORS_MASK|SMALL_DOORS_MASK|DRAWERS_MASK)
#define ALL_SERVOS_MASK       (ALL_DOORS_MASK|UTILITY_ARMS_MASK)

// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.

//     Pin  Close Pos, Open Pos,  Group ID  (Change the Close and Open to your Droids actual limits)
const ServoSettings servoSettings[] PROGMEM = {
    { 1,  1688, 700, TOP_UTILITY_ARM },       /* 0: Top Utility Arm */
    { 2,  2300, 950, BOTTOM_UTILITY_ARM },    /* 1: Bottom Utility Arm */
    { 3,  1600, 2240, LARGE_LEFT_DOOR },      /* 2: Large Left Door as viewing from looking at R2 */
    { 4,  1650, 937, LARGE_RIGHT_DOOR },      /* 3: Large Right door as viewing from looking at R2 */
    { 5,  1526, 726, CHARGE_BAY_DOOR },       /* 4: Charge Bay Inidicator Door*/
    { 6,  2030, 1549, DATA_PANEL_DOOR },      /* 5: Data Panel Door */
    { 7,  2050, 700, DRAWER_S1 },             /* 5: Data Panel Door */
    { 8,  2345, 700, DRAWER_S2 },             /* 5: Data Panel Door */
    { 9,  550, 2300, DRAWER_S3 },             /* 5: Data Panel Door */
    { 10,  2070, 650, DRAWER_S4 },            /* 5: Data Panel Door */
    { 11,  2030, 1549, REAR_LEFT_DOOR },      /* 5: Data Panel Door */
    { 12,  2030, 1549, REAR_RIGHT_DOOR }      /* 5: Data Panel Door */
  };

ServoDispatchPCA9685<SizeOfArray(servoSettings)> servoDispatch(servoSettings);
ServoSequencer servoSequencer(servoDispatch);

//////////////////////////////////////////////////////////////////////
  ///*****   Door Values, Containers, Flags & Timers   *****///
  //////////////////////////////////////////////////////////////////////

  int door = -1;
  // Door Command Container
  uint32_t D_command[7]  = {0,0,0,0,0,0,0};
  int doorFunction = 0;
  int doorBoard = 0; 
  int doorEasingMethod;
  uint32_t cVarSpeedMin;
  uint32_t cVarSpeedMax;
  uint32_t doorEasingDuration;
  uint32_t delayCallTime;

  // variables for individual functions
  uint32_t varSpeedMin;
  uint32_t varSpeedMax;
  char stringToSend[25];
  uint32_t fVarSpeedMin;
  uint32_t fVarSpeedMax;
  
  //////////////////////////////////////////////////////////////////////
  ///*****       Startup and Loop Variables                     *****///
  //////////////////////////////////////////////////////////////////////
  
  boolean startUp = true;
  boolean isStartUp = true;
  
  //Main Loop Timers
  unsigned long mainLoopTime;
  unsigned long MLMillis;
  byte mainLoopDelayVar = 5;

  //Timers for Status updates
  int keepAliveDuration = 5000;  // 5 seconds
  int keepAliveMillis;

  //////////////////////////////////////////////////////////////////////
  ///******       Serial Ports Specific Setup                   *****///
  //////////////////////////////////////////////////////////////////////

  #define headerSerial Serial2
  #define HEADER_BAUD_RATE 115200

/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

//    MAC Address to broadcast to all senders at once
  uint8_t broadcastMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//    MAC Address for the Local ESP to use - This prevents having to capture the MAC address of reciever boards.
  uint8_t espNowLocalMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};

// Define variables to store commands to be sent
  String senderID;
  String targetID;
  String command;
  String commandSubString;
  String espnowpassword;


// Define variables to store incoming commands
  String incomingTargetID;  
  String incomingSenderID;
  String incomingCommand;
  String incomingPassword;
  
// Variable to store if sending data was successful
  String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
      char structPassword[25];
      char structSenderID[15];
      char structTargetID[5];
      char structCommand[100];
  } struct_message;


//  // Create a struct_message calledcommandsTosend to hold variables that will be sent
  struct_message commandsToSendtoBroadcast;

//  // Create a struct_message to hold incoming commands from the Dome
  struct_message commandsToReceiveFromBroadcast;
//
  esp_now_peer_info_t peerInfo;

//  // Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (debugflag_espnow == 1){
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

  // Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&commandsToReceiveFromBroadcast, incomingData, sizeof(commandsToReceiveFromBroadcast));
  incomingPassword = commandsToReceiveFromBroadcast.structPassword;
  if (incomingPassword != ESPNOWPASSWORD){
  DBG_ESPNOW("Wrong ESP-NOW Password was sent.  Message Ignored\n");  
  } else {
  incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
    if (incomingTargetID == "BS"){
      incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
      incomingCommand = commandsToReceiveFromBroadcast.structCommand;
      DBG_ESPNOW("Bytes received from ESP-NOW Message: %i\n", len);
      DBG_ESPNOW("Target ID= %s\n", incomingTargetID);
      DBG_ESPNOW("Sender ID = %s\n",incomingSenderID);
      DBG_ESPNOW("Command = %s\n" , incomingCommand); 
      inputString = incomingCommand;
      stringComplete = true; 
    } else {
        DBG_ESPNOW("ESP-NOW Message Ignored\n");
    }
  }
} 

//////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////
  
  //Raspberry Pi              192.168.4.100
  //Body Controller ESP       192.168.4.101
  //Body Servo Controller     192.168.4.110   (Only used for OTA)  ************
  //Dome Controller ESP       192.168.4.111   (Only used for OTA)  
  //Periscope Controller ESP  192.168.4.112   (Only used for OTA)
  //Remote                    192.168.4.107
  //Developer Laptop          192.168.4.125
  
  // IP Address config of local ESP
  IPAddress local_IP(192,168,4,110);
  IPAddress subnet(255,255,255,0);
  IPAddress gateway(192,168,4,101);
  
  ////R2 Control Network Details
  // const char* ssid = "R2D2_Control_Network";  // only commented out for testing at work.
  const char* ssid = "R_Remote";      // comment when testing is complete

  const char* password =  "astromech";
  
  uint8_t oldLocalMACAddress[] = {0x24, 0x0A, 0xC4, 0xED, 0x30, 0x13};

  AsyncWebServer server(80);
  

//////////////////////////////////////////////////////////////////////
///*****             Status LED Varibles and settings         *****///
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

#define NUM_STATUS_LEDS 1

Adafruit_NeoPixel StatusLED = Adafruit_NeoPixel(NUM_STATUS_LEDS, ST_LED_DATA, NEO_GRB + NEO_KHZ800);

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                                       Door Functions                                          /////
///////                                                                                               /////
///////              Door Command Stucture: Dxyyzzabbccccddddeeeee                                    /////
///////                 D = Door Command                                                              /////
///////                 x = Servo Board                                                               /////
///////                   1 = Body Only                                                               /////
///////                   2 = Dome Only                                                               /////
///////                   3 = Both, starting with the body                                            /////
///////                   4 = Both, starting with the dome                                            /////
///////                 yy = Door Function                                                            /////
///////                 zz = Door Specified (Only used for Dx01 & Dx02)                               /////
///////                 a = Specify easing/delayCall options (B,E,D)                                  /////
///////                   B = Use Easing and Delay Call                                               /////
///////                   E = Easing Only (if VarSpeed is not needed, don't input dddd)               /////
///////                   D = Delay Call Only (don't input bbccccdddd)                                /////
///////                 bb = Easing Method (only needed if B or E was selected)
///////                 cccc = Easing Method Duration (only needed if B or E was selected)
///////                 dddd = Easing Method VarSpeedMax(only needed if B or E was selected and VARSPEED wanted)
///////                 eeeee = Delay Call Duration (only needed if B or D was selected)
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void openDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  //Command: Dx01zz
  DBG_SERVO("Open Specific Door\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch (doorpos){
      case 1: DBG_SERVO("Open Top Utility Arm\n");            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, TOP_UTILITY_ARM, varSpeedMin, varSpeedMax);      break;
      case 2: DBG_SERVO("Open Bottom Utility Arm\n");         SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, BOTTOM_UTILITY_ARM, varSpeedMin, varSpeedMax);   break;
      case 3: DBG_SERVO("Open Large Left Door\n");            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, LARGE_LEFT_DOOR, varSpeedMin, varSpeedMax);      break;
      case 4: DBG_SERVO("Open Large Right Door\n");           SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, LARGE_RIGHT_DOOR, varSpeedMin, varSpeedMax);     break;
      case 5: DBG_SERVO("Open Charge Bay Indicator Door\n");  SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, CHARGE_BAY_DOOR, varSpeedMin, varSpeedMax);      break;
      case 6: DBG_SERVO("Open Data Panel Door\n");            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, DATA_PANEL_DOOR, varSpeedMin, varSpeedMax);      break;
      case 7: DBG_SERVO("Open Drawer S-1\n");                 SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, DRAWER_S1, varSpeedMin, varSpeedMax);            break;
      case 8: DBG_SERVO("Open Drawer S-1\n");                 SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, DRAWER_S2, varSpeedMin, varSpeedMax);            break;
      case 9: DBG_SERVO("Open Drawer S-1\n");                 SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, DRAWER_S3, varSpeedMin, varSpeedMax);            break;
      case 10: DBG_SERVO("Open Drawer S-1\n");                SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, DRAWER_S4, varSpeedMin, varSpeedMax);            break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG_SERVO("Open SMALL_PANEL_ONE\n");      
              sprintf(stringToSend, "D20101E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 2: DBG_SERVO("Open SMALL_PANEL_TWO\n");      
              sprintf(stringToSend, "D20102E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 3: DBG_SERVO("Open SMALL_PANEL_THREE\n");    
              sprintf(stringToSend, "D20103E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 4: DBG_SERVO("Open MEDIUM_PANEL_PAINTED\n"); 
              sprintf(stringToSend, "D20104E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 5: DBG_SERVO("Open MEDIUM_PANEL_SILVER\n");  
              sprintf(stringToSend, "D20105E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 6: DBG_SERVO("Open BIG_PANEL\n");            
              sprintf(stringToSend, "D20106E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 7: DBG_SERVO("Open PIE_PANEL_ONE\n");         
              sprintf(stringToSend, "D20107E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 8: DBG_SERVO("Open PIE_PANEL_TWO\n");        
              sprintf(stringToSend, "D20108E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 9: DBG_SERVO("Open PIE_PANEL_THREE\n");      
              sprintf(stringToSend, "D20109E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 10:  DBG_SERVO("Open PIE_PANEL_FOUR\n");      
                sprintf(stringToSend, "D20110E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
                sendESPNOWCommand("DS", stringToSend);  break;
    }
  };
  D_command[0]   = '\0';
};


void closeDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx02zz
  DBG_SERVO("Close Specific Door");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch(doorpos){    
      case 1: DBG_SERVO("Close Top Utility Arm\n");           SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, TOP_UTILITY_ARM, varSpeedMin, varSpeedMax);       break;
      case 2: DBG_SERVO("Close Bottom Utility Arm\n");        SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, BOTTOM_UTILITY_ARM, varSpeedMin, varSpeedMax);    break;
      case 3: DBG_SERVO("Close Large Left Door\n");           SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, LARGE_LEFT_DOOR, varSpeedMin, varSpeedMax);       break;
      case 4: DBG_SERVO("Close Large Right Door\n");          SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, LARGE_RIGHT_DOOR, varSpeedMin, varSpeedMax);      break;
      case 5: DBG_SERVO("Close Charge Bay Indicator Door\n"); SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, CHARGE_BAY_DOOR, varSpeedMin, varSpeedMax);       break;
      case 6: DBG_SERVO("Close Data Panel Door\n");           SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, DATA_PANEL_DOOR, varSpeedMin, varSpeedMax);       break;
      case 7: DBG_SERVO("Close Drawer S-1\n");                 SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, DRAWER_S1, varSpeedMin, varSpeedMax);            break;
      case 8: DBG_SERVO("Close Drawer S-1\n");                 SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, DRAWER_S2, varSpeedMin, varSpeedMax);            break;
      case 9: DBG_SERVO("Close Drawer S-1\n");                 SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, DRAWER_S3, varSpeedMin, varSpeedMax);            break;
      case 10: DBG_SERVO("Close Drawer S-1\n");                SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, DRAWER_S4, varSpeedMin, varSpeedMax);            break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG_SERVO("Close SMALL_PANEL_ONE\n");     
              sprintf(stringToSend, "D20201E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 2: DBG_SERVO("Close SMALL_PANEL_TWO\n");      
              sprintf(stringToSend, "D20202E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 3: DBG_SERVO("Close SMALL_PANEL_THREE\n");   
              sprintf(stringToSend, "D20203E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 4: DBG_SERVO("Close MEDIUM_PANEL_PAINTED\n"); 
              sprintf(stringToSend, "D20204E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 5: DBG_SERVO("Close MEDIUM_PANEL_SILVER\n");  
              sprintf(stringToSend, "D20205E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 6: DBG_SERVO("Close BIG_PANEL\n");            
              sprintf(stringToSend, "D20206E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 7: DBG_SERVO("Close PIE_PANEL_ON\nE");        
              sprintf(stringToSend, "D20207E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 8: DBG_SERVO("Close PIE_PANEL_TWO\n");        
              sprintf(stringToSend, "D20208E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 9: DBG_SERVO("Close PIE_PANEL_THREE\n");      
              sprintf(stringToSend, "D20209E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 10:  DBG_SERVO("Close PIE_PANEL_FOUR\n");      
                sprintf(stringToSend, "D20210E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
                sendESPNOWCommand("DS", stringToSend);  break;
    }
  };
  D_command[0]   = '\0';
}


void openAllDoors(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx03
  DBG_SERVO("Open all Doors\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D203E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); 
            // writeBcSerial("SMPt001");  break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0] = '\0';
}

  
void closeAllDoors(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx04
  DBG_SERVO("Close all Doors\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D204E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0] = '\0';
}


void shortCircuit(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx05
  // add sequence for this routine.  
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it

  SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllShortCircuit, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);
  
  D_command[0] = '\0';

}

void servoTest(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx05
  // add sequence for this routine.  
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it

  SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelTest, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);
  
  D_command[0] = '\0';

}


void allOpenClose(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration){
  // Command: Dx06
  DBG_SERVO("Open and Close All Doors\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D206E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0]   = '\0';                                           
}


void allOpenCloseLong(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration){
  // Command: Dx07
  DBG_SERVO("Open and Close Doors Long\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D207E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0]   = '\0';                                                 
}


void allFlutter(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration){
  // Command: Dx08
  DBG_SERVO("Flutter All Doors\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D208E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }  
  D_command[0]   = '\0';   
}


void allOpenCloseRepeat(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration){
  // Command: Dx09
  DBG_SERVO("Open and Close All Doors Repeat\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D209E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0]   = '\0';             
}


void panelWave(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration){
  // Command: Dx10
  DBG_SERVO("Wave\n");
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;
  snprintf(stringToSend, sizeof(stringToSend), "D210E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);
            DelayCall::schedule([] {sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend);
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);}, delayCallDuration); break;
  }
  D_command[0]   = '\0';                                             
}


void panelWaveFast(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration){
  // Command: Dx11  
  DBG_SERVO("Wave Fast\n");
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;
  snprintf(stringToSend, sizeof(stringToSend),"D211E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);
            DelayCall::schedule([] {sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend);
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);}, delayCallDuration); break;
  }
  D_command[0]   = '\0';                                             
}


void openCloseWave(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx12
  DBG_SERVO("Open Close Wave \n");
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;  
  snprintf(stringToSend, sizeof(stringToSend), "D212E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);
            DelayCall::schedule([] {sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend);
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);}, delayCallDuration); break;
  }
  D_command[0]   = '\0';                                             
}


void marchingAnts(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx13
  DBG_SERVO("Marching Ants\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D213E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0]   = '\0';                                             
}


void panelAlternate(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx14
  DBG_SERVO("Panel Alternate\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D214E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0]   = '\0';                                             
}                                                            


void panelDance(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
 // Command: Dx15
  DBG_SERVO("Panel Dance\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D215E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0]   = '\0';                                             
}


void longDisco(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx16
  DBG_SERVO("Panel Dance Long\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D216E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0]   = '\0';                                             
}


void longHarlemShake(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx17
  DBG_SERVO("Harlem Shake\n");
  fVarSpeedMin = varSpeedMin;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  fVarSpeedMax = varSpeedMax;                                                               // sets Global Variable from the local variable to allow the lambda function to utilize it
  if (delayCallDuration == 0){delayCallDuration = defaultESPNOWSendDuration;}               //sets default delayCall to allow time for ESP-NOW message to get to reciever ESP.
  snprintf(stringToSend, sizeof(stringToSend),"D217E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("DS", stringToSend);}, delayCallDuration); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);},delayCallDuration);  break;
  }
  D_command[0]   = '\0';                                             
}                                                       


///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                             Serial & ESP-NOW Communication Functions                          /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      /////////////////////////////////////////////////////////
//      ///*****          Serial Event Function          *****///
//      /////////////////////////////////////////////////////////
//      /// This routine is run between loop() runs, so using ///
//      /// delay inside loop can delay response.  Multiple   ///
//      /// bytes of data may be available.                   ///
//      /////////////////////////////////////////////////////////
                                                  
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  DBG_SERIAL_EVENT("%s\n", inputString);
}

void headerSerialEvent() {
  while (headerSerial.available()) {
    char inChar = (char)headerSerial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  DBG_SERIAL_EVENT("%s\n", inputString);
}

//      /////////////////////////////////////////////////////////
//      ///*****          Serial Write Function          *****///
//      /////////////////////////////////////////////////////////
//      /// These functions recieve a string and transmits    ///
//      /// one character at a time and adds a '/r' to the    ///
//      /// end of the string.                                ///
//      /////////////////////////////////////////////////////////


void writeSerialString(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++)
  {
    Serial.write(completeString[i]);
  }
}

void writeheaderSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++)
  {
    headerSerial.write(completeString[i]);
  }
}


//////////////////////////////////////////////////////////////////////
///*****             ESP-NOW Functions                        *****///
//////////////////////////////////////////////////////////////////////

void setupSendStruct(struct_message* msg, String pass, String sender, String targetID, String cmd)
{
    snprintf(msg->structPassword, sizeof(msg->structPassword), "%s", pass.c_str());
    snprintf(msg->structSenderID, sizeof(msg->structSenderID), "%s", sender.c_str());
    snprintf(msg->structTargetID, sizeof(msg->structTargetID), "%s", targetID.c_str());
    snprintf(msg->structCommand, sizeof(msg->structCommand), "%s", cmd.c_str());
}

void sendESPNOWCommand(String starget, String scomm){
  String senderID = "BS";     // change to match location (BC, BS, DP, DC, LD)
  setupSendStruct(&commandsToSendtoBroadcast ,ESPNOWPASSWORD, senderID, starget, scomm);
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
  if (result == ESP_OK) {
    DBG_ESPNOW("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
  }
  else {
    DBG_ESPNOW("Error sending the data\n");
  }
  ESPNOW_command[0] = '\0';
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                             Miscellaneous Functions                                           /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
///*****             Debug Functions                          *****///
//////////////////////////////////////////////////////////////////////


void DBG(const char *format, ...) {
        if (!debugflag)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void DBG_1(const char *format, ...) {
        if (!debugflag1)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void DBG_ESPNOW(const char *format, ...) {
        if (!debugflag_espnow)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void DBG_SERVO(const char *format, ...) {
        if (!debugflag_servo)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void DBG_SERIAL_EVENT(const char *format, ...) {
        if (!debugflag_serial_event)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void DBG_LOOP(const char *format, ...) {
        if (!debugflag_loop)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void DBG_HTTP(const char *format, ...) {
        if (!debugflag_loop)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void DBG_LORA(const char *format, ...) {
        if (!debugflag_loop)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void toggleDebug(){
  debugflag = !debugflag;
  if (debugflag == 1){
    Serial.println("Debugging Enabled");
    }
  else{
    Serial.println("Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}

void toggleDebug1(){
  debugflag1 = !debugflag1;
  if (debugflag1 == 1){
    Serial.println("Parameter Debugging Enabled");
    }
  else{
    Serial.println("Parameter Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}

void toggleDebug_ESPNOW(){
  debugflag_espnow = !debugflag_espnow;
  if (debugflag_espnow == 1){
    Serial.println("ESP-NOW Debugging Enabled");
    }
  else{
    Serial.println("ESP-NOW Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}

void toggleDebug_Servo(){
  debugflag_servo = !debugflag_servo;
  if (debugflag_servo == 1){
    Serial.println("Servo Debugging Enabled");
    }
  else{
    Serial.println("Servo Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}

void toggleDebug_SerialEvent(){
  debugflag_serial_event = !debugflag_serial_event;
  if (debugflag_serial_event == 1){
    Serial.println("Serial Events Debugging Enabled");
    }
  else{
    Serial.println("Serial Events Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}

void toggleDebug_Loop(){
  debugflag_loop = !debugflag_loop;
  if (debugflag_loop == 1){
    Serial.println("Main Loop Debugging Enabled");
    }
  else{
    Serial.println("Main Loop Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}
void toggleDebug_HTTP(){
  debugflag_http = !debugflag_http;
  if (debugflag_http == 1){
    Serial.println("HTTP Debugging Enabled");
    }
  else{
    Serial.println("HTTP Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}

void toggleDebug_LORA(){
  debugflag_lora = !debugflag_lora;
  if (debugflag_lora == 1){
    Serial.println("LoRa Debugging Enabled");
    }
  else{
    Serial.println("LoRa Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}

//////////////////////////////////////////////////////////////////////
///*****    Connects to WiFi and turns on OTA functionality   *****///
//////////////////////////////////////////////////////////////////////
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
    request->send(200, "text/plain", "Please go to http://192.168.4.110/update to upload file");
  });
  
  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  server.begin();

  ESP_command[0]   = '\0';
}   

//////////////////////////////////////////////////////////////////////
///*****        Sets Servo Easing Method                      *****///
//////////////////////////////////////////////////////////////////////

void setServoEasingMethod(int easingMethod){
  switch(easingMethod){
    case 1: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::LinearInterpolation);    break;
    case 2: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseIn);        break;
    case 3: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseOut);       break;
    case 4: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseInOut);     break;
    case 5: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseIn);            break;
    case 6: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseOut);           break;
    case 7: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseInOut);         break;
    case 8: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseIn);          break;
    case 9: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseOut);         break;
    case 10:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseInOut);      break;
    case 11:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseIn);         break;
    case 12:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseOut);        break;
    case 13:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseInOut);      break;
    case 14:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseIn);            break;
    case 15:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseOut);           break;
    case 16:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseInOut);         break;
    case 17:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseIn);        break;
    case 18:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseOut);       break;
    case 19:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseInOut);     break;
    case 20:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseIn);     break;
    case 21:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseOut);    break;
    case 22:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseInOut);  break;
    case 23:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseIn);         break;
    case 24:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseOut);        break;
    case 25:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseInOut);      break;
    case 26:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseIn);            break;
    case 27:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseOut);           break;
    case 28:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseInOut);         break;
    case 29:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseIn);          break;
    case 30:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseOut);         break;
    case 31:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseInOut);       break;
    default:  servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::LinearInterpolation); 
              DBG("No Easing Method Selected\n");                                                  break;
  }
}


//////////////////////////////////////////////////////////////////////
///*****          Scan I2C for devices                        *****///
//////////////////////////////////////////////////////////////////////

void scan_i2c()
{
    unsigned nDevices = 0;
    for (byte address = 1; address < 127; address++)
    {
        String name = "<unknown>";
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        if (address == 0x70)
        {
            // All call address for PCA9685
            name = "PCA9685:all";
        }
        if (address == 0x40)
        {
            // Adafruit PCA9685
            name = "PCA9685";
        }
        if (address == 0x14)
        {
            // IA-Parts magic panel
            name = "IA-Parts Magic Panel";
        }
        if (address == 0x20)
        {
            // IA-Parts periscope
            name = "IA-Parts Periscope";
        }
        if (address == 0x16)
        {
            // PSIPro
            name = "PSIPro";
        }

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.print(" ");
            Serial.println(name);
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

//////////////////////////////////////////////////////////////////////
///*****    Send Keepalive Messages for Status                *****///
//////////////////////////////////////////////////////////////////////


void keepAlive(){
  if (millis() - keepAliveMillis >= (keepAliveDuration + random(1, 500))){
    keepAliveMillis = millis();
   sendESPNOWCommand("HB","HEARTBEAT");  
} 
}

//////////////////////////////////////////////////////////////////////
///*****   ColorWipe Function for Status LED                  *****///
//////////////////////////////////////////////////////////////////////
void colorWipeStatus(uint32_t c, int brightness) {
  for(uint16_t i=0; i<2; i++) {
    StatusLED.setBrightness(brightness);
    StatusLED.setPixelColor(i, c);
    StatusLED.show();
  }
}
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
  headerSerial.begin(HEADER_BAUD_RATE, SERIAL_8N1, SERIAL_RX_HEADER, SERIAL_TX_HEADER);
  
  delay(500);
  
  Serial.println("\n\n----------------------------------------");
  Serial.print("Booting up the ");Serial.println(HOSTNAME);
  Serial.println("----------------------------------------");

  StatusLED.begin();
  StatusLED.show();
  colorWipeStatus(red,255);

  //Initialize I2C for the Servo Expander Board
  Wire.begin();
  
  //Initialize the ReelTwo Library
  SetupEvent::ready();
  
  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

  //initialize WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &espNowLocalMACAddress[0]);
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

  // Register peer configuration
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peers  
  memcpy(peerInfo.peer_addr, broadcastMACAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add Broadcast ESP-NOW peer");
      return;
    }    
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  
}   // end of setup

void loop(){

  if (millis() - MLMillis >= mainLoopDelayVar){
      MLMillis = millis();
      AnimatedEvent::process();

    if(startUp) {
      // writeBcSerial("SMPt003");
      closeAllDoors(1,0,0,0,0);
      startUp = false;
      Serial.print("Startup complete\nStarting main loop\n\n\n");
      colorWipeStatus(blue,25);

    }  
    keepAlive();
    if(Serial.available()){serialEvent();}
    // if(bcSerial.available()){bcSerialEvent();}
    if(headerSerial.available()){headerSerialEvent();}

    if (stringComplete) {autoComplete=false;}
    if (stringComplete || autoComplete) {
      if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
        else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
        if( inputBuffer[0]=='D' ||        // Door Designator
            inputBuffer[0]=='d' ||        // Door Designator
            inputBuffer[0]=='E' ||        // Command designatore for internal ESP functions
            inputBuffer[0]=='e' ||        // Command designatore for internal ESP functions
            inputBuffer[0]=='N' ||        // Command for Sending ESP-NOW Messages
            inputBuffer[0]=='n' ||        // Command for Sending ESP-NOW Messages
            inputBuffer[0]=='S' ||        // Command for sending Serial Strings out Serial ports
            inputBuffer[0]=='s'           // Command for sending Serial Strings out Serial ports


         ){commandLength = strlen(inputBuffer);                     //  Determines length of command character array.
            DBG_LOOP("Command: %s with a length of %d \n", inputBuffer, commandLength);
            if(commandLength >= 3) {
               if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {                                                            // specifies the overall door command
              doorBoard = inputBuffer[1]-'0';                                                                           // Sets the board to call the command on.
              doorFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');                                              // Sets the door command to a specific function
              if (doorFunction == 1 || doorFunction == 2){                                                              // Checks the door command to see if it's calling to open a single door
                door = (inputBuffer[4]-'0')*10+(inputBuffer[5]-'0');                                                    // Sets the specific door to move
                if (inputBuffer[6] == 'D' || inputBuffer[6] == 'd'){
                  DBG_LOOP("with DelayCall \n");
                  delayCallTime =  (inputBuffer[7]-'0')*10000+(inputBuffer[8]-'0')*1000+(inputBuffer[9]-'0')*100+(inputBuffer[10]-'0')*10+(inputBuffer[11]-'0');  // converts 5 digit character to uint32_t
                  doorEasingMethod = 0;                                                                                                                           // Sets Easing Method to 0-Off
                  cVarSpeedMin = 0;
                  cVarSpeedMax = 0;                                                                                                                        // Sets Easing duration to 0-Off
                } else if (inputBuffer[6] == 'E' ||inputBuffer[6] == 'e'){
                  DBG_LOOP("with Easing \n");
                  doorEasingMethod = (inputBuffer[7]-'0')*10+(inputBuffer[8]-'0');
                  doorEasingDuration = (inputBuffer[9]-'0')*1000+(inputBuffer[10]-'0')*100+(inputBuffer[11]-'0')*10+(inputBuffer[12]-'0');                
                  delayCallTime = 0;
                } else if (inputBuffer[6] == 'B' || inputBuffer[6] == 'b'){
                  DBG_LOOP("with Both Easing and Delay Call \n");
                  doorEasingMethod = (inputBuffer[7]-'0')*10+(inputBuffer[8]-'0');
                  cVarSpeedMin = (inputBuffer[9]-'0')*1000+(inputBuffer[10]-'0')*100+(inputBuffer[11]-'0')*10+(inputBuffer[12]-'0');                
                  cVarSpeedMax = (inputBuffer[13]-'0')*1000+(inputBuffer[14]-'0')*100+(inputBuffer[15]-'0')*10+(inputBuffer[16]-'0');
                  delayCallTime =  (inputBuffer[17]-'0')*10000+(inputBuffer[18]-'0')*1000+(inputBuffer[19]-'0')*100+(inputBuffer[20]-'0')*10+(inputBuffer[21]-'0');
                }else{
                  DBG_LOOP("No easing or Delay time specified \n");
                  delayCallTime = 0;
                  doorEasingMethod = 0;
                  cVarSpeedMin = 0;
                  cVarSpeedMax = 0;                
                }
              }
              else if (doorFunction != 1 || doorFunction != 2) {
                DBG_LOOP("Other Door Function Called \n");
                if (inputBuffer[4] == 'D' || inputBuffer[4] == 'd'){
                  DBG_LOOP("with DelayCall \n");
                  delayCallTime =  (inputBuffer[5]-'0')*10000+(inputBuffer[6]-'0')*1000+(inputBuffer[7]-'0')*100+(inputBuffer[8]-'0')*10+(inputBuffer[9]-'0');
                  doorEasingMethod = 0;
                  cVarSpeedMin = 0;
                  cVarSpeedMax = 0;
                } else if (inputBuffer[4] == 'E' ||inputBuffer[4] == 'e'){
                  DBG_LOOP("with Easing \n");
                  doorEasingMethod = (inputBuffer[5]-'0')*10+(inputBuffer[6]-'0');
                  if (commandLength >= 13){
                    DBG_LOOP("Variable Speed Selected\n");
                    cVarSpeedMin = (inputBuffer[7]-'0')*1000+(inputBuffer[8]-'0')*100+(inputBuffer[9]-'0')*10+(inputBuffer[10]-'0');                
                    cVarSpeedMax = (inputBuffer[11]-'0')*1000+(inputBuffer[12]-'0')*100+(inputBuffer[13]-'0')*10+(inputBuffer[14]-'0');  
                  } else {
                    DBG_LOOP("No Variable Speed selected\n");
                    cVarSpeedMin = (inputBuffer[7]-'0')*1000+(inputBuffer[8]-'0')*100+(inputBuffer[9]-'0')*10+(inputBuffer[10]-'0');                
                    cVarSpeedMax = cVarSpeedMin; 
                  }              
                  delayCallTime = 0;
                } else if (inputBuffer[4] == 'B' || inputBuffer[4] == 'b'){
                  DBG_LOOP("Both Easing and Delay Call \n");
                  doorEasingMethod = (inputBuffer[5]-'0')*10+(inputBuffer[6]-'0');
                  if (commandLength >= 17){
                    cVarSpeedMin = (inputBuffer[7]-'0')*1000+(inputBuffer[8]-'0')*100+(inputBuffer[9]-'0')*10+(inputBuffer[10]-'0');                
                    cVarSpeedMax = (inputBuffer[11]-'0')*1000+(inputBuffer[12]-'0')*100+(inputBuffer[13]-'0')*10+(inputBuffer[14]-'0');
                    delayCallTime =  (inputBuffer[15]-'0')*10000+(inputBuffer[16]-'0')*1000+(inputBuffer[17]-'0')*100+(inputBuffer[18]-'0')*10+(inputBuffer[19]-'0');
                  } else {
                    cVarSpeedMin = (inputBuffer[7]-'0')*1000+(inputBuffer[8]-'0')*100+(inputBuffer[9]-'0')*10+(inputBuffer[10]-'0');                
                    cVarSpeedMax = cVarSpeedMin;
                    delayCallTime =  (inputBuffer[11]-'0')*10000+(inputBuffer[12]-'0')*1000+(inputBuffer[13]-'0')*100+(inputBuffer[14]-'0')*10+(inputBuffer[15]-'0');
                  }
                }else{
                  DBG_LOOP("No easing or DelayCall time specified \n");
                  delayCallTime = 0;
                  doorEasingMethod = 0;
                  cVarSpeedMin = 0;
                  cVarSpeedMax = 0;
                }
              }
            } 
              if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {
                espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              };
              if(inputBuffer[0]=='N' || inputBuffer[0]=='n') {
                for (int i=1; i<=commandLength; i++){
                  char inCharRead = inputBuffer[i];
                  inputStringCommand += inCharRead;                   // add it to the inputString:
                }
                DBG_ESPNOW("\nFull Command Recieved: %s ",inputStringCommand.c_str());
                espNowCommandFunctionString = inputStringCommand.substring(0,2);
                espNowCommandFunction = espNowCommandFunctionString.toInt();
                DBG_LOOP("ESP NOW Command State: %i\n", espNowCommandFunction);
                targetID = inputStringCommand.substring(0,2);
                DBG_LOOP("Target ID: %s\n", targetID);
                commandSubString = inputStringCommand.substring(2,commandLength);
                DBG_LOOP("Command to Forward: %s\n", commandSubString.c_str());
                sendESPNOWCommand(targetID, commandSubString);
                inputStringCommand="";
              }
              if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
//                serialPort =  (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
                for (int i=1; i<commandLength-1;i++ ){
                  char inCharRead = inputBuffer[i];
                  serialStringCommand += inCharRead;  // add it to the inputString:
                }
                serialPort = serialStringCommand.substring(0,2);
                serialSubStringCommand = serialStringCommand.substring(2,commandLength);
                DBG_LOOP("Serial Command: %s to Serial Port: %s\n", serialSubStringCommand.c_str(), serialPort);                
                if (serialPort == "BC"){
                  // writeBcSerial(serialSubStringCommand);
                } else if (serialPort == "FU"){
                  writeheaderSerial(serialSubStringCommand);
                } 
                
                serialStringCommand = "";
                serialSubStringCommand = "";
                serialPort = "";
            } 
              
              

            if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {
              D_command[0]   = '\0';                                                            // Flushes Array
              D_command[0] = doorFunction;
              D_command[1] = doorBoard;
                if(door>=0) {D_command[2] = door;}
              D_command[3] = doorEasingMethod;
              D_command[4] = cVarSpeedMin;
              D_command[5] = cVarSpeedMax;
              D_command[6] = delayCallTime;

              DBG_LOOP("Door Function Called: %d\n",doorFunction);
              DBG_LOOP("Easing Method: %d \n",doorEasingMethod);
              DBG_LOOP("VarSpeed - Min:%d, Max:%d \n",cVarSpeedMin, cVarSpeedMax);
              DBG_LOOP("DelayCall Duration: %d\n",delayCallTime);
            }

              if(inputBuffer[0]=='E' || inputBuffer[0] == 'e') {
                ESP_command[0]   = '\0';                                                            // Flushes Array
                ESP_command[0] = espCommandFunction;
              }
              if(inputBuffer[0]=='N' || inputBuffer[0] == 'n') {
                ESPNOW_command[0]   = '\0';                                                            // Flushes Array
                ESPNOW_command[0] = espNowCommandFunction;
                tempESPNOWTargetID = targetID;
              }
            }
          }

      ///***  Clear States and Reset for next command.  ***///
        stringComplete =false;
        autoComplete = false;
        inputBuffer[0] = '\0';
        
        // reset Local ESP Command Variables
        int espCommandFunction;
      
      // reset ESP-NOW variables
        targetID = "";
        espNowCommandFunction =0;

      // reset Door Variables
        int door = -1;
        int doorFunction;
        int doorBoard;
        int doorEasingMethod;
        uint32_t cVarSpeedMin;
        uint32_t cVarSpeedMax;
        uint32_t delayCallTime; 

      DBG_LOOP("command Proccessed\n");

    }


    if(ESP_command[0]){
      switch (ESP_command[0]){
        case 1: Serial.println(HOSTNAME);
                ESP_command[0]   = '\0';                                                                break;
        case 2: Serial.println("Resetting the ESP in 3 Seconds");
                DelayCall::schedule([] {ESP.restart();}, 3000) ;                                        
                ESP_command[0]   = '\0';                                                                break;
        case 3: connectWiFi();                                                                          break;
        case 4: ESP.restart();                                                                          break;
        case 5: break;
        case 6: break;
        case 7: break;
        case 8: break;
        case 9: scan_i2c();ESP_command[0]='\0';                                                         break;
        case 20: toggleDebug();                                                                         break;
        case 21: toggleDebug1();                                                                        break;
        // case 22: toggleDebug_HTTP();                                                                    break;
        // case 23: toggleDebug_LORA();                                                                    break;
        case 24: toggleDebug_ESPNOW();                                                                  break;
        case 25: toggleDebug_Servo();                                                                   break;
        case 26: toggleDebug_SerialEvent();                                                             break;
        case 27: toggleDebug_Loop();                                                                    break;
      }
    }
    if(ESPNOW_command[0]){
      switch(ESPNOW_command[0]){
        case 1: sendESPNOWCommand(tempESPNOWTargetID,commandSubString);                                 break; 
        case 2: sendESPNOWCommand("ALL","ALL");                                                       break;  //reserved for future use
        case 3: break;  //reserved for future use      
      }
    }

    if(D_command[0]) {
      if((D_command[0] == 1 || D_command[0] == 2) && D_command[1] >= 11) {
        DBG_LOOP("Incorrect Door Value Specified, Command Aborted!");
        D_command[0] = '\0';
      }
      else {
        switch (D_command[0]) {
case 1: openDoor(D_command[1],D_command[2],D_command[3],D_command[4],D_command[5]);                     break;
          case 2: closeDoor(D_command[1],D_command[2],D_command[3],D_command[4],D_command[5]);          break;
          case 3: openAllDoors(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                    break;
          case 4: closeAllDoors(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                   break;
          case 5: shortCircuit(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                    break;
          case 6: allOpenClose(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                    break;
          case 7: allOpenCloseLong(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                break;
          case 8: allFlutter(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                      break;
          case 9: allOpenCloseRepeat(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);              break;
          case 10: panelWave(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);         break;
          case 11: panelWaveFast(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);     break;
          case 12: openCloseWave(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);     break;
          case 13: marchingAnts(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                   break;
          case 14: panelAlternate(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                 break;
          case 15: panelDance(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                     break;
          case 16: longDisco(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                      break;
          case 17: longHarlemShake(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                break;
          case 95: servoTest(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);                break;
          case 98: closeAllDoors(2,0,0,0,0);                                                              break;
          case 99: closeAllDoors(2,0,0,0,0);                                                              break;
          default: break;
        }
      }
    }
    if(isStartUp) {
      isStartUp = false;
      delay(500);

    }
  }
}
