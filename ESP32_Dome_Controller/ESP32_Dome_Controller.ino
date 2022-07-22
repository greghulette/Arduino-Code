// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
//#include <elegantWebpage.h>
#include <AsyncTCP.h>
#include <WiFi.h>

//#include <Hash.h>

//Used for ESP-NOW
#include "esp_wifi.h"
#include <esp_now.h>

//Used for Camera LEDs
#include <Adafruit_NeoPixel.h>

//Used for PC9685 - Servo Expansion Board
#include <Wire.h>

//ReelTwo libaries
//#define USE_DEBUG
//#define USE_SERVO_DEBUG
#include "ReelTwo.h"
#include "core/DelayCall.h"
#include "ServoDispatchPCA9685.h"
#include "ServoSequencer.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                                                                                                       *****///
///*****                            Created by Greg Hulette.                                                   *****///
///*****                                                                                                       *****///
///*****   I started with the code from flthymcnsty from from which I used the basic command structure and     *****///
///*****  serial input method.  This code also relies on the ReelTwo library for all it's servo movements.     *****///
///*****                                                                                                       *****///
///*****                      So exactly what does this all do.....?                                           *****///
///*****                       - Controls the Dome servos                                                      *****///
///*****                       - Controls the Camera Light                                                     *****///
///*****                       - Sends Serial commands to the HPs and RSeries Logic Displays                   *****///
///*****                                                                                                       *****///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
///*****              ReelTwo Servo Set Up                       *****///
/////////////////////////////////////////////////////////////////////////

#define SMALL_PANEL_ONE       0x0001 //b0000 0000 0000 0001
#define SMALL_PANEL_TWO       0x0002 //b0000 0000 0000 0010
#define SMALL_PANEL_THREE     0x0004 //b0000 0000 0000 0100
#define MEDIUM_PANEL_PAINTED  0x0008 //b0000 0000 0000 1000
#define MEDIUM_PANEL_SILVER   0x0010 //b0000 0000 0001 0000
#define BIG_PANEL             0x0020 //b0000 0000 0010 0000
#define PIE_PANEL_ONE         0x0040 //b0000 0000 0100 0000
#define PIE_PANEL_TWO         0x0080 //b0000 0000 1000 0000
#define PIE_PANEL_THREE       0x0100 //b0000 0001 0000 0000
#define PIE_PANEL_FOUR        0x0200 //b0000 0010 0000 0000

#define SMALL_PANELS_MASK     (SMALL_PANEL_ONE|SMALL_PANEL_TWO|SMALL_PANEL_THREE)
#define MEDIUM_PANELS_MASK    (MEDIUM_PANEL_PAINTED|MEDIUM_PANEL_SILVER)
#define DOME_PANELS_MASK      (SMALL_PANELS_MASK|MEDIUM_PANELS_MASK|BIG_PANEL)
#define PIE_PANELS_MASK       (PIE_PANEL_ONE|PIE_PANEL_TWO|PIE_PANEL_THREE|PIE_PANEL_FOUR)
#define ALL_SERVOS_MASK       (DOME_PANELS_MASK|PIE_PANELS_MASK)

// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.

//     Pin  Min, ,Max,  Group ID  (Change the Min and Max to your Droids actual limits)
const ServoSettings servoSettings[] PROGMEM = {
    { 1,   600, 2400, SMALL_PANEL_ONE },       /* 0: door 1 small left door by radar eye */
    { 2,   600, 2400, SMALL_PANEL_TWO },       /* 1: door 2 small middle door by radar eye */
    { 3,   600, 2400, SMALL_PANEL_THREE },     /* 2: door 3 small right door by radar eye */
    { 4,   600, 2400, MEDIUM_PANEL_PAINTED },  /* 3: door 4 medium painted door */
    { 5,   2400, 600, MEDIUM_PANEL_SILVER },   /* 4: door 5 Medium Unpainted door*/
    { 6,   2400, 600, BIG_PANEL },             /* 5: door 6 Big Lower door */
    { 7,   2400, 600, PIE_PANEL_ONE },         /* 6: door 7 Pie Panel near Periscope */
    { 8,   600, 2400, PIE_PANEL_TWO },         /* 7: door 8 Pie Panel clockwise from Periscope*/
    { 9,   2400, 600, PIE_PANEL_THREE },       /* 8: door 9 Pie Panel clockwise-2 from Periscope */
    { 10,  600, 2400, PIE_PANEL_FOUR }        /* 9: door 10 Pie Panel clockwise-3 from Periscope */
};

ServoDispatchPCA9685<SizeOfArray(servoSettings)> servoDispatch(servoSettings);
ServoSequencer servoSequencer(servoDispatch);


//////////////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////

  char inputBuffer[100];
  String inputString;         // a string to hold incoming data
  String inputStringCommand;
  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA
  int ledFunction;
  int commandLength;
  int serialCommandFunction;
  String serialCommandFunctionString;

  uint32_t ESP_command[6]  = {0,0,0,0,0,0};
  int espCommandFunction     = 0;
  
  String serialPort;
  String serialStringCommand;

  uint32_t ESPNOW_command[6]  = {0,0,0,0,0,0};
  int espNowCommandFunction = 0;
  String espNowCommandFunctionString;
  String tempESPNOWTargetID;
    
  int debugflag = 0;
  int debugflag1 = 0;  // Used for optional level of debuging



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
  char stringToSend[20];
  uint32_t fVarSpeedMin;
  uint32_t fVarSpeedMax;

  uint32_t servoMovementDurationInDelayCall;

//////////////////////////////////////////////////////////////////////
///*****       Startup and Loop Variables                     *****///
//////////////////////////////////////////////////////////////////////
  
  boolean startUp = true;
  boolean isStartUp = true;
  
  unsigned long mainLoopTime; // We keep track of the "Main Loop time" in this variable.
  unsigned long MLMillis;
  byte mainLoopDelayVar = 5;

//////////////////////////////////////////////////////////////////////
///*****             Camera Lens Variables and settings       *****///
//////////////////////////////////////////////////////////////////////
  
  unsigned long loopTime; // We keep track of the "time" in this variable.

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

  #define NUM_CAMERA_PIXELS 12
  #define CAMERA_LENS_DATA_PIN 12
  //#define CAMERA_LENS_CLOCK_PIN 13
  int dim = 75;
  unsigned long CLMillis;
  byte CLSpeed = 50;
  unsigned long startMillis;
  unsigned long currentMillis;
  byte CL_command[6] = {0,0,0,0,0,0};
  
  int colorState1;
  int speedState;
  
  Adafruit_NeoPixel stripCL = Adafruit_NeoPixel(NUM_CAMERA_PIXELS, CAMERA_LENS_DATA_PIN, NEO_GRB + NEO_KHZ800);

  boolean countUp=false;
///////////////////////////////////////////////////////////////////////
///*****                 Auto Sequence Settings                *****///
///*****                                                       *****///
///*****  When enabled, each display will automatically        *****///
///*****  trigger random display sequences. This setting can   *****///
///*****  be overridden by sending the following display       *****///
///*****  Display Commands to the appropriate display device.  *****///
///*****                                                       *****///
///*****     98 - Auto Off Trigger Command                     *****///
///*****     99 - Auto On Trigger  Command                     *****///
///*****                                                       *****///
///*****     1 = enabled   0 = disable                         *****///
///*****                                                       *****///
///////////////////////////////////////////////////////////////////////
   byte enableCLAuto   = 1;   //1
  
//////////////////////////////////////////////////////////////////////
///*****             Auto Mode Interval Settings               *****///
///*****                                                       *****///
///*****  Each display enabled by auto mode neeed 4 values.    *****///
///*****  The first two values of each category below are      *****///
///*****  the min and max range that determine the run time    *****///
///*****  of each display sequence.  Run times will be chosen  *****///
///*****  from with in this range.                             *****///
///*****                                                       *****///
///*****  Likewise, the second two values of each category     *****///
///*****  determine the range from which the interval between  *****///
///*****  display sequences will be randomly chosen.           *****///
///*****                                                       *****///
///////////////////////////////////////////////////////////////////////

  // Coin Slots
   unsigned const int CLAutoIntMin    = 300000;
   unsigned const int CLAutoIntMax    = 300000;
   unsigned const int CLAutoPauseMin  = 1;
   unsigned const int CLAutoPauseMax  = 1;
   unsigned int CLAutoPause;
   unsigned int CLAutoInt;

///////////////////////////////////////////////////////////////////////
///*****               Auto Sequence Assignments               *****///
///*****                                                       *****///
///*****   You can choose which sequences will be available    *****///
///*****  to each auto function. Simply add the command value  *****///
///*****  for each desired sequence to the string array below. *****///
///*****                                                       *****///
///////////////////////////////////////////////////////////////////////
  String CLautoCommands[] = {
                            "R0155",        // Pulse Blue at medium speed
                            "R0155"        // Pulse Blue at medium speed
                            };

  int CLautoCommandsCount = sizeof(CLautoCommands) / sizeof(CLautoCommands[ 0 ]);     // Determins the # of Commands in List

  unsigned long CLAutoTimer;

  //////////////////////////////////////////////////////////////////////
  ///******       Serial Ports Specific Setup                   *****///
  //////////////////////////////////////////////////////////////////////
  
  #define RXHP 19
  #define TXHP 18 
  #define RXRS 25
  #define TXRS 27 
  
  #define hpSerial Serial1
  #define rsSerial Serial2

  #define HP_BAUD_RATE 115200
  #define RS_BAUD_RATE 115200


/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

//  MAC Addresses used in the Droid
//  ESP-NOW Master =        {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
//  Dome Controller =       {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
//  Periscope Controller =  {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};

//  MAC Address of the receivers (Not Currenlty needed but have this in the code for future use)
uint8_t bodyPeerMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8_t periscopePeerMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};

//    MAC Address to broadcast to all senders at once
uint8_t broadcastMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//    MAC Address for the Local ESP to use - This prevents having to capture the MAC address of reciever boards.
uint8_t newLocalMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};

// Define variables to store commands to be sent
  String senderID;
  String destinationID;
  String targetID;
  String command;
  String commandSubString;


// Define variables to store incoming commands
  String incomingDestinationID;
  String incomingTargetID;  
  String incomingSenderID;
  String incomingCommand;
  
// Variable to store if sending data was successful
  String success;

//Structure example to send data
//Must match the receiver structure
  typedef struct struct_message {
      String structSenderID;
      String structDestinationID;
      String structTargetID;  
      String structCommand;
  } struct_message;

// Create a struct_message calledcommandsTosend to hold variables that will be sent
  struct_message commandsToSendtoBody;
  struct_message commandsToSendtoPeriscope;
  struct_message commandsToSendtoBroadcast;

// Create a struct_message to hold incoming commands from the Body
  struct_message commandsToReceiveFromBody;
  struct_message commandsToReceiveFromPeriscope;
  struct_message commandsToReceiveFromBroadcast;

  esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (debugflag == 1){
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
  memcpy(&commandsToReceiveFromBroadcast, incomingData, sizeof(commandsToReceiveFromBroadcast));
  incomingDestinationID = commandsToReceiveFromBroadcast.structDestinationID;
  incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
  incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
  incomingCommand = commandsToReceiveFromBroadcast.structCommand;
//  Serial.print("Bytes recieved from ESP-NOW Message: ");Serial.println(len);
//  Serial.print("Sender ID = "); Serial.println(incomingSenderID);
//  Serial.print("Destination ID= ");Serial.println(incomingDestinationID);
//  Serial.print("Command = "); Serial.println(incomingCommand); 
  DBG("Bytes received from ESP-NOW Message: %i\n", len);
  DBG("Sender ID = %s\n",incomingSenderID);
  DBG("Destination ID= %s\n" ,incomingDestinationID);
  DBG("Target ID= %s\n", incomingTargetID);
  DBG("Command = %s\n" , incomingCommand); 
  if (incomingDestinationID =="Dome"){
    DBG("ESP-NOW Command Accepted\n");
    DBG("Target ID= %s\n", incomingTargetID);
//    Serial.print("Target ID= "); Serial.println(incomingTargetID);
    if (incomingTargetID == "RS"){
        DBG("Sending %s out rsSerial\n", incomingCommand);
//        Serial.println("Sending out rsSerial");
        writeRsSerial(incomingCommand);
    } else if (incomingTargetID == "HP"){
        DBG("Sending %s out hpSerial\n", incomingCommand);
//        Serial.println("Sending out hpSerial");
        writeHpSerial(incomingCommand);
    } else if (incomingTargetID == "DS"){
        DBG("Execute Local Command = %s\n", incomingCommand);
//        Serial.print("Execute Local Command = "); Serial.println(incomingCommand);
        inputString = incomingCommand;
        stringComplete = true; 
    } else {
        DBG("Wrong Target ID Sent\n");
//        Serial.println("Wrong Target ID Sent");
      }
  }
    else {DBG("ESP-NOW Message Ignored\n");}
//  else {Serial.println("ESP-NOW Message Ignored");}
}

  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////
  
//Raspberry Pi              192.168.4.100
//Body Controller ESP       192.168.4.101   
//ESP-NOW Master ESP        192.168.4.110   (Only used for OTA) 
//Dome Controller ESP       192.168.4.111   (Only used for OTA) ************
//Periscope Controller ESP  192.168.4.112   (Only used for OTA)
//Remote                    192.168.4.107
//Developer Laptop          192.168.4.125
  
  // IP Address config of local ESP
  IPAddress local_IP(192,168,4,111);
  IPAddress subnet(255,255,255,0);
  IPAddress gateway(192,168,4,101);
  
  ////R2 Control Network Details
  const char* ssid = "R2D2_Control_Network";
  const char* password =  "astromech";
  
  AsyncWebServer server(80);
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                                      Radar eye Camera LED Functions                           /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void cameraLED(uint32_t color, int CLSpeed1){
  int CLRLow = 1;
  int CLRHigh = 50;

  currentMillis = millis();
      CLSpeed = map(CLSpeed1, 1, 9, 1, 250);
      if (currentMillis - startMillis >= CLSpeed){
      if(countUp == false){                   // check to see if the boolean flag is false.  If false, starting dimming the LEDs
      
          dim=dim - random(CLRLow, CLRHigh);  // set the brightness to current minus a random number between 5 and 40. I think that
                                              //adding a random causes a less smooth transition which makes it look a little better
          colorWipe(color, dim);              // Set the LEDs to the color and brightness using the colorWipe function
          }
      
        if(dim <= 10){                        //Check to see if the brightness is at or below 20.  Modifying the "20" will
                                              //allow the dim variable to go below zero causing the flicker.  The closer you
                                              //set the "20" to zero, the more flickering will happen. I use half the larger
                                              //dim random number to allow a small flicker without being too annoying.
      
           countUp = true;                    // if the dim variable is at or below "20", change the countUp flag to true      
          }
        if(countUp == true){                 // check to see if the boolean flag is true.  If true, starting brightening the LEDs
            dim=dim + random(CLRLow, CLRHigh); // set the brightness to current plus a random number between 5 and 40.  I think that
                                              //adding a random causes a less smooth transition which makes it look a little better
            colorWipe(color, dim);           // Set the LEDs to the color and brightness using the colorWheel function
        }
          if(dim>=250){                       //Check to see if the brightness is at or above 235.  Modifying the "235" will
                                               //allow the dim variable to go above 255 causing the flicker.  The closer you
                                              //set the "235" to 255, the more flickering will happen. I use half the larger
                                              //dim random number to allow a small flicker without being too annoying.
            countUp = false;                  // if the dim variable is at or above "235", change the countUp flag to false
          }
          startMillis = currentMillis; 
      }
      
  }

  //  Color Changing Function for the Camera Lens LEDs
void colorWipe(uint32_t c, int brightness) {
  for(uint16_t i=0; i<NUM_CAMERA_PIXELS; i++) {
    stripCL.setBrightness(brightness);
    stripCL.setPixelColor(i, c);
    stripCL.show();
  }
};

void clearCL() {
  for(uint16_t i=0; i<NUM_CAMERA_PIXELS; i++) {
    stripCL.setPixelColor(i, off);
    stripCL.show();
  }
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                                       Door Functions                                          /////
///////                                                                                               /////
///////                           Door Command Stucture: Dxyyzz                                       /////
///////                             D = Door Command                                                  /////
///////                             x = Servo Board                                                   /////
///////                               1 = Body Only                                                   /////
///////                               2 = Dome Only                                                   /////
///////                               3 = Both, starting with the body                                /////
///////                               4 = Both, starting with the dome                                /////
///////                             yy = Door Function                                                /////
///////                             zz = Door Specified (Only used for Dx01 & Dx02)                   /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void openDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  //Command: Dx01zz
  DBG("Open Specific Door\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG("Open Top Utility Arm\n");
              sprintf(stringToSend, "D10101%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 2: DBG("Open Bottom Utility Arm\n");
              sprintf(stringToSend, "D10102%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 3: DBG("Open Large Left Door\n");
              sprintf(stringToSend, "D10103%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 4: DBG("Open Large Right Door\n");
              sprintf(stringToSend, "D10104%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 5: DBG("Open Charge Bay Indicator Door\n");
              sprintf(stringToSend, "D10105%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 6: DBG("Open Data Panel Door\n");
              sprintf(stringToSend, "D10106%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch (doorpos){
      case 1: Serial.println("Open SMALL_PANEL_ONE");       SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, SMALL_PANEL_ONE, varSpeedMin, varSpeedMax);     break;
      case 2: Serial.println("Open SMALL_PANEL_TWO");       SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, SMALL_PANEL_TWO, varSpeedMin, varSpeedMax);     break;
      case 3: Serial.println("Open SMALL_PANEL_THREE");     SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, SMALL_PANEL_THREE, varSpeedMin, varSpeedMax);   break;
      case 4: Serial.println("Open MEDIUM_PANEL_PAINTED");  SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, MEDIUM_PANEL_PAINTED, varSpeedMin, varSpeedMax);break;
      case 5: Serial.println("Open MEDIUM_PANEL_SILVER");   SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, MEDIUM_PANEL_SILVER, varSpeedMin, varSpeedMax); break;
      case 6: Serial.println("Open BIG_PANEL");             SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, BIG_PANEL, varSpeedMin, varSpeedMax);           break;
      case 7: Serial.println("Open PIE_PANEL_ONE");         SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, PIE_PANEL_ONE, varSpeedMin, varSpeedMax);       break;
      case 8: Serial.println("Open PIE_PANEL_TWO");         SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, PIE_PANEL_TWO, varSpeedMin, varSpeedMax);       break;
      case 9: Serial.println("Open PIE_PANEL_THREE");       SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, PIE_PANEL_THREE, varSpeedMin, varSpeedMax);     break;
      case 10: Serial.println("Open PIE_PANEL_FOUR");       SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, PIE_PANEL_FOUR, varSpeedMin, varSpeedMax);      break;

    }
  };
  D_command[0]   = '\0';
};

void closeDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx02zz
  DBG("Close Specific Door");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    switch(doorpos){
      case 1: DBG("Close Top Utility Arm\n");             
              sprintf(stringToSend, "D10201%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 2: DBG("Close Bottom Utility Arm\n");              
              sprintf(stringToSend, "D10202%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 3: DBG("Close Large Left Door\n");
              sprintf(stringToSend, "D10203%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 4: DBG("Close Large Right Door\n");
              sprintf(stringToSend, "D10204%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 5: DBG("Close Charge Bay Indicator Door\n");
              sprintf(stringToSend, "D10205%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
      case 6: DBG("Close Data Panel Door\n");
              sprintf(stringToSend, "D10206%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("BS", stringToSend);                                        
              break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch (doorpos){
      case 1: Serial.println("Close SMALL_PANEL_ONE");       SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, SMALL_PANEL_ONE, varSpeedMin, varSpeedMax);     break;
      case 2: Serial.println("Close SMALL_PANEL_TWO");       SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, SMALL_PANEL_TWO, varSpeedMin, varSpeedMax);     break;
      case 3: Serial.println("Close SMALL_PANEL_THREE");     SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, SMALL_PANEL_THREE, varSpeedMin, varSpeedMax);   break;
      case 4: Serial.println("Close MEDIUM_PANEL_PAINTED");  SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, MEDIUM_PANEL_PAINTED, varSpeedMin, varSpeedMax);break;
      case 5: Serial.println("Close MEDIUM_PANEL_SILVER");   SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, MEDIUM_PANEL_SILVER, varSpeedMin, varSpeedMax); break;
      case 6: Serial.println("Close BIG_PANEL");             SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, BIG_PANEL, varSpeedMin, varSpeedMax);           break;
      case 7: Serial.println("Close PIE_PANEL_ONE");         SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, PIE_PANEL_ONE, varSpeedMin, varSpeedMax);       break;
      case 8: Serial.println("Close PIE_PANEL_TWO");         SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, PIE_PANEL_TWO, varSpeedMin, varSpeedMax);       break;
      case 9: Serial.println("Close PIE_PANEL_THREE");       SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, PIE_PANEL_THREE, varSpeedMin, varSpeedMax);     break;
      case 10: Serial.println("Close PIE_PANEL_FOUR");       SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, PIE_PANEL_FOUR, varSpeedMin, varSpeedMax);      break;
    }
  };
  D_command[0]   = '\0';
}


void openAllDoors(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx03
  DBG("Open all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D103%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("BS",stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
      setServoEasingMethod(servoEasingMethod);
      SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  };
  D_command[0] = '\0';
}


void closeAllDoors(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx04
  DBG("Close all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D104%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("BS", stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  };
  D_command[0] = '\0';
}


void shortCircuit(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx05
  // add sequence for this routine.  
}


void allOpenClose(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax){
  // Command: Dx06
  DBG("Open and Close All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D106%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("BS", stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  };
  D_command[0]   = '\0';                                           
}


void allOpenCloseLong(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax){
  // Command: Dx07
  DBG("Open and Close Doors Long\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D107%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("BS", stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  };
  D_command[0]   = '\0';                                                 
}


void allFlutter(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax){
  // Command: Dx08
  DBG("Flutter All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D108%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("BS", stringToSend);  
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  };
  D_command[0]   = '\0';   
}


void allOpenCloseRepeat(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax){
  // Command: Dx09
  DBG("Open and Close All Doors Repeat\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D109%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("BS", stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  };
  D_command[0]   = '\0';             
}


void panelWave(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration=2125){
  // Command: Dx10
  DBG("Wave\n");
  // servoMovementDurationInDelayCall = servoMovementDuration;
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;
  sprintf(stringToSend, "D110%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BS", stringToSend); break;
    case 2: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 3: sendESPNOWCommand("BS", stringToSend);
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);}, delayCallDuration); break;
    case 4: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([]{sendESPNOWCommand("BS", stringToSend);}, delayCallDuration); break;
  }
  D_command[0]   = '\0';                                             
}


void panelWaveFast(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration){
  // Command: Dx11  
  DBG("Wave Fast\n");
  // servoMovementDurationInDelayCall = varSpeedMin, varSpeedMax;
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;
  sprintf(stringToSend, "D111%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BS", stringToSend);  break;
    case 2: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);  break;
    case 3: sendESPNOWCommand("BS", stringToSend);     
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);}, delayCallDuration); break;
    case 4: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
            DelayCall::schedule([] {sendESPNOWCommand("BS", stringToSend);}, delayCallDuration); break;
  }
  D_command[0]   = '\0';                                             
}


void openCloseWave(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration) {
  // Command: Dx12
  DBG("Open Close Wave \n");
  // servoMovementDurationInDelayCall = varSpeedMin, varSpeedMax;
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;
  sprintf(stringToSend, "D112%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BS", stringToSend); break;
    case 2: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);  break;
    case 3: sendESPNOWCommand("BS", stringToSend);     
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, fVarSpeedMin, fVarSpeedMax);}, delayCallDuration); break;
    case 4: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            DelayCall::schedule([] {sendESPNOWCommand("BS", stringToSend);}, delayCallDuration); break;
  }
  D_command[0]   = '\0';                                             
}


void marchingAnts(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx13
  DBG("Marching Ants\n");
  sprintf(stringToSend, "D113%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BS", stringToSend);                                      break;
    case 2: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 3: sendESPNOWCommand("BS", stringToSend);
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 4: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); 
            sendESPNOWCommand("BS", stringToSend); break;
  }
  D_command[0]   = '\0';                                             
}


void panelAlternate(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx14
  DBG("Panel Alternate\n");
  sprintf(stringToSend, "D114%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BS", stringToSend);                                    break;
    case 2: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 3: sendESPNOWCommand("BS", stringToSend); 
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 4: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
            sendESPNOWCommand("BS", stringToSend); break;
  }
  D_command[0]   = '\0';                                             
}                                                            


void panelDance(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
 // Command: Dx15
  DBG("Panel Dance\n");
  sprintf(stringToSend, "D115%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BS", stringToSend);                                    break;
    case 2: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 3: sendESPNOWCommand("BS", stringToSend); 
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 4: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
            sendESPNOWCommand("BS", stringToSend); break;
  }
  D_command[0]   = '\0';                                             
}


void longDisco(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx16
  DBG("Panel Dance Long\n");
  sprintf(stringToSend, "D116%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BS", stringToSend);                                    break;
    case 2: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 3: sendESPNOWCommand("BS", stringToSend); 
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 4: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
            sendESPNOWCommand("BS", stringToSend);break;
  }
  D_command[0]   = '\0';                                             
}


void longHarlemShake(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx17
  DBG("Harlem Shake\n");
  sprintf(stringToSend, "D117%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BS", stringToSend);                                    break;
    case 2: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 3: sendESPNOWCommand("BS", stringToSend); 
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 4: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
            sendESPNOWCommand("BS", stringToSend);break;
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
//
void serialEvent() {
  //int count = 0;
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    }
  }
  DBG("Received %s\n", inputString);
}


void hpSerialEvent() {
  while (hpSerial.available()) {
    char inChar = (char)hpSerial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  DBG("Recieved %s\n", inputString);
}


void rsSerialEvent() {
  while (rsSerial.available()) {
    char inChar = (char)rsSerial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  DBG("Received %s\n", inputString);
}

 /////////////////////////////////////////////////////////
  ///*****          Serial Write Function          *****///
  /////////////////////////////////////////////////////////
  /// These functions recieve a string and transmits    ///
  /// one character at a time and adds a '/r' to the    ///
  /// end of the string.                                ///
  /////////////////////////////////////////////////////////

void writeSerialString(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++)
  {
    Serial.write(completeString[i]);
  }
}
void writeRsSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++)
  {
    rsSerial.write(completeString[i]);
  }
  DBG("Printing to rsSerial\n");
}

void writeHpSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    hpSerial.write(completeString[i]);
  }
  DBG("Printing to hpSerial\n");
}

//////////////////////////////////////////////////////////////////////
///*****             ESP-NOW Functions                        *****///
//////////////////////////////////////////////////////////////////////


void sendESPNOWCommand(String starget,String scomm){
  String sdest;
  if (starget == "DS" || starget == "RS" || starget == "HP"){
    sdest = "Dome";
  } else if (starget == "PC" || starget == "PL"){
    sdest = "Periscope";
  }else if (starget == "EN" || starget == "BS" || starget == "BL" || starget == "ST"|| starget == "BS"){
    sdest = "Body";
  }
  
  commandsToSendtoBroadcast.structDestinationID = sdest;
  DBG("Setting sdest to: %s \n", sdest);
  commandsToSendtoBroadcast.structTargetID = starget;
  commandsToSendtoBroadcast.structSenderID = "Dome";
  commandsToSendtoBroadcast.structCommand = scomm;
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
  if (result == ESP_OK) {
    DBG("Sent with success \n");
  }
  else {
    DBG("Error sending the data\n");
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
///*****             Debugging Functions                      *****///
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



//////////////////////////////////////////////////////////////////////
///*****    Connects to WiFi and turns on OTA functionality   *****///
//////////////////////////////////////////////////////////////////////

void connectWiFi(){
  Serial.println(WiFi.config(local_IP, gateway, subnet) ? "Client IP Configured" : "Failed!");
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.SSID());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Please go to  update to upload file");
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
    case 10: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseInOut);      break;
    case 11: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseIn);         break;
    case 12: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseOut);        break;
    case 13: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseInOut);      break;
    case 14: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseIn);            break;
    case 15: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseOut);           break;
    case 16: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseInOut);         break;
    case 17: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseIn);        break;
    case 18: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseOut);       break;
    case 19: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseInOut);     break;
    case 20: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseIn);     break;
    case 21: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseOut);    break;
    case 22: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseInOut);  break;
    case 23: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseIn);         break;
    case 24: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseOut);        break;
    case 25: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseInOut);      break;
    case 26: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseIn);            break;
    case 27: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseOut);           break;
    case 28: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseInOut);         break;
    case 29: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseIn);          break;
    case 30: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseOut);         break;
    case 31: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseInOut);       break;
  }
}
 

void setup(){
  //Initialize the Serial Ports
  Serial.begin(115200);                                                                   // Initialize Serial Connection at 115200:
  hpSerial.begin(HP_BAUD_RATE,SERIAL_8N1,RXHP,TXHP);
  rsSerial.begin(RS_BAUD_RATE,SERIAL_8N1,RXRS,TXRS);
    
  Serial.println("\n\n\n----------------------------------------");
  Serial.println("Booting up the Periscope Controller");

  //Initialize I2C for the Servo Expander Board
  Wire.begin();
  
  //Initialize the ReelTwo Library
  SetupEvent::ready();

  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

  //Initialize the NeoPixel ring for the camera lens/radar eye
  stripCL.begin();
  stripCL.show(); // Initialize all pixels to 'off'
  colorWipe(red, 255); // red during bootup
  Serial.println("LED Setup Complete");


  //initialize WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &newLocalMACAddress[0]);
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
  //  peerInfo.ifidx=WIFI_IF_AP;

  // Add peers  
    memcpy(peerInfo.peer_addr, bodyPeerMACAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add Body ESP-NOW peer");
      return;
    }

    memcpy(peerInfo.peer_addr, periscopePeerMACAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add Periscope ESP-NOW peer");
      return;
    }
    memcpy(peerInfo.peer_addr, broadcastMACAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add Broadcast ESP-NOW peer");
      return;
    }  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

 
}  // end of Setup


void loop() {
if (millis() - MLMillis >= mainLoopDelayVar){
  MLMillis = millis();
  loopTime = millis();
  AnimatedEvent::process();
  if(startUp) {
      closeAllDoors(2,0,0,0);
      startUp = false;
      Serial.println("Startup");
  }
  if(Serial.available()){serialEvent();}
  if(hpSerial.available()){hpSerialEvent();}
  if(rsSerial.available()){rsSerialEvent();}

//  cameraLED(blue, 5); // blue

  if (stringComplete) {autoComplete=false;}
  if (stringComplete || autoComplete) {
    if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
      else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
      if( inputBuffer[0]=='D' ||        // Door Designator
          inputBuffer[0]=='d' ||        // Door Designator
          inputBuffer[0]=='R' ||        // Radar Eye LED
          inputBuffer[0]=='r' ||        // Radar Eye LED
          inputBuffer[0]=='E' ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='e' ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='N' ||        // Command for Sending ESP-NOW Messages
          inputBuffer[0]=='n' ||        // Command for Sending ESP-NOW Messages
          inputBuffer[0]=='S' ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='s'           // Command for sending Serial Strings out Serial ports

        ){commandLength = strlen(inputBuffer);                     //  Determines length of command character array.
          DBG("Command: %s with a length of %d \n", inputBuffer, commandLength);
          if(commandLength >= 3) {
            if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {                                                            // specifies the overall door command
              doorBoard = inputBuffer[1]-'0';                                                                           // Sets the board to call the command on.
              doorFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');                                              // Sets the door command to a specific function
              if (doorFunction == 1 || doorFunction == 2){                                                              // Checks the door command to see if it's calling to open a single door
                door = (inputBuffer[4]-'0')*10+(inputBuffer[5]-'0');                                                    // Sets the specific door to move
                if (inputBuffer[6] == 'D' || inputBuffer[6] == 'd'){
                  DBG("with DelayCall \n");
                  delayCallTime =  (inputBuffer[7]-'0')*10000+(inputBuffer[8]-'0')*1000+(inputBuffer[9]-'0')*100+(inputBuffer[10]-'0')*10+(inputBuffer[11]-'0');  // converts 5 digit character to uint32_t
                  doorEasingMethod = 0;                                                                                                                           // Sets Easing Method to 0-Off
                  cVarSpeedMin = 0;
                  cVarSpeedMax = 0;                                                                                                                        // Sets Easing duration to 0-Off
                } else if (inputBuffer[6] == 'E' ||inputBuffer[6] == 'e'){
                  DBG("with Easing \n");
                  doorEasingMethod = (inputBuffer[7]-'0')*10+(inputBuffer[8]-'0');
                  doorEasingDuration = (inputBuffer[9]-'0')*1000+(inputBuffer[10]-'0')*100+(inputBuffer[11]-'0')*10+(inputBuffer[12]-'0');                
                  delayCallTime = 0;
                } else if (inputBuffer[6] == 'B' || inputBuffer[6] == 'b'){
                  DBG("with Both Easing and Delay Call \n");
                  doorEasingMethod = (inputBuffer[7]-'0')*10+(inputBuffer[8]-'0');
                  cVarSpeedMin = (inputBuffer[9]-'0')*1000+(inputBuffer[10]-'0')*100+(inputBuffer[11]-'0')*10+(inputBuffer[12]-'0');                
                  cVarSpeedMax = (inputBuffer[13]-'0')*1000+(inputBuffer[14]-'0')*100+(inputBuffer[15]-'0')*10+(inputBuffer[16]-'0');
                  delayCallTime =  (inputBuffer[17]-'0')*10000+(inputBuffer[18]-'0')*1000+(inputBuffer[19]-'0')*100+(inputBuffer[20]-'0')*10+(inputBuffer[21]-'0');
                }else{
                  DBG("No easing or Delay time specified \n");
                  delayCallTime = 0;
                  doorEasingMethod = 0;
                  cVarSpeedMin = 0;
                  cVarSpeedMax = 0;                
                }
              }
              else if (doorFunction != 1 || doorFunction != 2) {
                DBG("Other Door Function Called \n");
                if (inputBuffer[4] == 'D' || inputBuffer[4] == 'd'){
                  DBG("with DelayCall \n");
                  delayCallTime =  (inputBuffer[5]-'0')*10000+(inputBuffer[6]-'0')*1000+(inputBuffer[7]-'0')*100+(inputBuffer[8]-'0')*10+(inputBuffer[9]-'0');
                  doorEasingMethod = 0;
                  cVarSpeedMin = 0;
                  cVarSpeedMax = 0;
                } else if (inputBuffer[4] == 'E' ||inputBuffer[4] == 'e'){
                  DBG("with Easing \n");
                  doorEasingMethod = (inputBuffer[5]-'0')*10+(inputBuffer[6]-'0');
                  cVarSpeedMin = (inputBuffer[7]-'0')*1000+(inputBuffer[8]-'0')*100+(inputBuffer[9]-'0')*10+(inputBuffer[10]-'0');                
                  cVarSpeedMax = (inputBuffer[11]-'0')*1000+(inputBuffer[12]-'0')*100+(inputBuffer[13]-'0')*10+(inputBuffer[14]-'0');                
                  delayCallTime = 0;
                } else if (inputBuffer[4] == 'B' || inputBuffer[4] == 'b'){
                  DBG("with Both Easing and Delay Call \n");
                  doorEasingMethod = (inputBuffer[5]-'0')*10+(inputBuffer[6]-'0');
                  cVarSpeedMin = (inputBuffer[7]-'0')*1000+(inputBuffer[8]-'0')*100+(inputBuffer[9]-'0')*10+(inputBuffer[10]-'0');                
                  cVarSpeedMax = (inputBuffer[11]-'0')*1000+(inputBuffer[12]-'0')*100+(inputBuffer[13]-'0')*10+(inputBuffer[14]-'0');
                  delayCallTime =  (inputBuffer[15]-'0')*10000+(inputBuffer[16]-'0')*1000+(inputBuffer[17]-'0')*100+(inputBuffer[18]-'0')*10+(inputBuffer[19]-'0');
                }else{
                  DBG("No easing or Delay time specified \n");
                  delayCallTime = 0;
                  doorEasingMethod = 0;
                  cVarSpeedMin = 0;
                  cVarSpeedMax = 0;
                }
              }
            }                                    
            if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {
              espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
            }
            if(inputBuffer[0]=='N' || inputBuffer[0]=='n') {
              for (int i=1; i<=commandLength; i++){
                char inCharRead = inputBuffer[i];
                inputStringCommand += inCharRead;                   // add it to the inputString:
              }
              DBG("\nFull Command Recieved: %s ",inputStringCommand);
              espNowCommandFunctionString = inputStringCommand.substring(0,2);
              espNowCommandFunction = espNowCommandFunctionString.toInt();
              DBG("ESP NOW Command State: %s\n", espNowCommandFunction);
              targetID = inputStringCommand.substring(2,4);
              DBG("Target ID: %s\n", targetID);
              commandSubString = inputStringCommand.substring(4,commandLength);
              DBG("Command to Forward: %s\n", commandSubString);
            }
            if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
              serialPort =  (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              for (int i=3; i<commandLength-2;i++ ){
              char inCharRead = inputBuffer[i];
              serialStringCommand += inCharRead;  // add it to the inputString:
              }
              DBG("Serial Command: %s to Serial Port: %s\n", serialStringCommand, serialPort);
              if (serialPort == "HP"){
                writeHpSerial(serialStringCommand);
              } else if (serialPort == "RS"){
                writeRsSerial(serialStringCommand);
              } else if (serialPort == "DS"){
                inputString = serialStringCommand;
                stringComplete = true; 
              }
              serialStringCommand = "";
              serialPort = "";
            }   
            if(inputBuffer[0]=='R' || inputBuffer[0]=='r') {
              ledFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              colorState1 = (inputBuffer[3]-'0');
              speedState = (inputBuffer[4]-'0');
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

              DBG("Door Function Called: %d\n",doorFunction);
              DBG("Easing Method: %d \n",doorEasingMethod);
              DBG("VarSpeed - Min:%d, Max:%d \n",cVarSpeedMin, cVarSpeedMax);
              DBG("DelayCall Duration: %d\n",delayCallTime);
            }

            if(inputBuffer[0]=='R' || inputBuffer[0]=='r'){
              CL_command[0]   = '\0';                                                            // Flushes Array
              CL_command[0] = ledFunction;
              CL_command[1] = colorState1;
              CL_command[2] = speedState;
              if(!autoComplete) {enableCLAuto = 0; }                                            //  Disables Automode to keep it from overriding User selected commands
              DBG(" LED Function:%d, ColorState:%d, Color(Dec):%d, Speed:%d\n",ledFunction, colorState1, basicColors[colorState1], speedState);
              DBG(" LED Function:%d, ColorState:%d, Color(Dec):%d, Speed:%d\n",CL_command[0], CL_command[1], basicColors[CL_command[1]], CL_command[2]);
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

        // reset Camera Variables
        int ledFunction;
        int speedState;
        int colorState1;
        
        // reset Door Variables
        int door = -1;
        int doorFunction;
        int doorBoard;
        int doorEasingMethod;
        uint32_t cVarSpeedMin;
        uint32_t cVarSpeedMax;
        uint32_t delayCallTime;

        // reset ESP-NOW Variables
        inputStringCommand = "";
        targetID = "";
    
        // DBG("command taken\n");

    }

    if(ESP_command[0]){
      switch (ESP_command[0]){
        case 1: Serial.println("Controller: Dome ESP Controller");   
                ESP_command[0]   = '\0';                                                  break;
        case 2: Serial.println("Resetting the ESP in 3 Seconds");
                DelayCall::schedule([] {ESP.restart();}, 3000);
                ESP_command[0]   = '\0';                                                  break;
        case 3: connectWiFi();                                                            break;
        case 4: break;  //reserved for future use
        case 5: break;  //reserved for future use
        case 6: break;  //reserved for future use
        case 7: break;  //reserved for future use
        case 8: break;  //reserved for future use
        case 9: break;  //reserved for future use
        case 10: toggleDebug();                                                           break;
        case 11: toggleDebug1();                                                          break; 
      }
    }

    if(D_command[0]) {
      if((D_command[0] == 1 || D_command[0] == 2) && D_command[1] >= 11) {
//        DBG("Incorrect Door Value Specified, Command Aborted!");
        D_command[0] = '\0';
      }
      else {
        switch (D_command[0]) {
          case 1: openDoor(D_command[1],D_command[2],D_command[3],D_command[4],D_command[5]);          break;
          case 2: closeDoor(D_command[1],D_command[2],D_command[3],D_command[4],D_command[5]);         break;
          case 3: openAllDoors(D_command[1],D_command[3],D_command[4],D_command[5]);                   break;
          case 4: closeAllDoors(D_command[1],D_command[3],D_command[4],D_command[5]);                  break;
          case 5: shortCircuit(D_command[1],D_command[3],D_command[4],D_command[5]);                   break;
          case 6: allOpenClose(D_command[1],D_command[3],D_command[4],D_command[5]);                   break;
          case 7: allOpenCloseLong(D_command[1],D_command[3],D_command[4],D_command[5]);               break;
          case 8: allFlutter(D_command[1],D_command[3],D_command[4],D_command[5]);                     break;
          case 9: allOpenCloseRepeat(D_command[1],D_command[3],D_command[4],D_command[5]);             break;
          case 10: panelWave(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);        break;
          case 11: panelWaveFast(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);    break;
          case 12: openCloseWave(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);    break;
          case 13: marchingAnts(D_command[1],D_command[3],D_command[4],D_command[5]);                  break;
          case 14: panelAlternate(D_command[1],D_command[3],D_command[4],D_command[5]);                break;
          case 15: panelDance(D_command[1],D_command[3],D_command[4],D_command[5]);                    break;
          case 16: longDisco(D_command[1],D_command[3],D_command[4],D_command[5]);                     break;
          case 17: longHarlemShake(D_command[1],D_command[3],D_command[4],D_command[5]);               break;
          case 98: closeAllDoors(2,0,0,0);                                                  break;
          case 99: closeAllDoors(2,0,0,0);                                                  break;
          default: break;
        }
      }
    }

    if(CL_command[0]){
      switch(CL_command[0]){
        case 1: cameraLED(basicColors[CL_command[1]], CL_command[2]);                     break;
        case 2: break;  //reserved for future use
        case 3: break;  //reserved for future use
        case 96: enableCLAuto = 0;                                                                               break;     // Disable Auto Mode
        case 97: enableCLAuto = 1;                                                                               break;     // Enables Auto Mode
        case 98:  CL_command[0] = '\0'; 
                  clearCL();
                  enableCLAuto = 0; break;
        case 99:  CL_command[0] = '\0'; 
                  clearCL();
                  enableCLAuto = 1; break;
      }
    }

    if(ESPNOW_command[0]){
      switch(ESPNOW_command[0]){
        case 1: sendESPNOWCommand(tempESPNOWTargetID,commandSubString);                   break; 
        case 2: break;  //reserved for future use
        case 3: break;  //reserved for future use
      }
    }
      if(!stringComplete && inputString) {
          if(enableCLAuto == 1) {CLAuto();}
      
        }
    if(isStartUp) {
      isStartUp = false;
      delay(500);
    }
  }
}  //end of main loop

     void CLAuto () {
       if(millis() - CLAutoTimer >= CLAutoInt*1000) {       // and the timer has reached the set interval
        if(millis() - CLAutoTimer >= (CLAutoInt+CLAutoPause)*1000) {     // Assign a random command string from the Auto Command Array to the input string
          if(!autoComplete) {
           CLAutoTimer = millis();
           CLAutoPause = random(CLAutoPauseMin,CLAutoPauseMax);
           CLAutoInt = random(CLAutoIntMin,CLAutoIntMax);
           autoInputString = CLautoCommands[random((CLautoCommandsCount-1))];
           autoComplete = true;
          }
         }
        else {
           CL_command[0] = 99;
        }                                                             // and set flag so new command is processes at beginning of loop
      }
     }
