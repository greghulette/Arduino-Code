#define DEBUG
// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>

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
    { 5,   600, 2400, MEDIUM_PANEL_SILVER },   /* 4: door 5 Medium Unpainted door*/
    { 6,   600, 2400, BIG_PANEL },             /* 5: door 6 Big Lower door */
    { 7,   600, 2400, PIE_PANEL_ONE },         /* 6: door 7 Pie Panel near Periscope */
    { 8,   600, 2400, PIE_PANEL_TWO },         /* 7: door 8 Pie Panel clockwise from Periscope*/
    { 9,   600, 2400, PIE_PANEL_THREE },       /* 8: door 9 Pie Panel clockwise-2 from Periscope */
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
    
  int debugflag = 1;
  int debugflag1 = 1;  // Used for optional level of debuging

   char stringToSend[20];
  uint32_t servoMovementDuration;

//////////////////////////////////////////////////////////////////////
///*****   Door Values, Containers, Flags & Timers   *****///
//////////////////////////////////////////////////////////////////////
  int door = -1;
  // Door Command Container
  uint32_t D_command[6]  = {0,0,0,0,0,0};
  int doorFunction = 0;
  int doorBoard = 0; 
  int doorEasingMethod;
  uint32_t doorEasingDuration;

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
  
  byte CL_command[4] = {0,0,0,0};
  
  int colorState1;
  int speedState;
  // Set some primary and secondary default color values as a fall back in case no colors
  // are provided in input commands. This makes the ssytem much more user friendly.

    byte defaultPrimaryColorInt     = 5;          //1 Integer color value from list above
    byte defaultSecondaryColorInt   = 1;          //5 Integer color value from list above

  Adafruit_NeoPixel stripCL = Adafruit_NeoPixel(NUM_CAMERA_PIXELS, CAMERA_LENS_DATA_PIN, NEO_GRB + NEO_KHZ800);

  boolean countUp=false;

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
  char macStr[18];
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  DBG("Packet to: %s\n", macStr);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//  DBG("Send Status:\t %s\n", status);
}

//   Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&commandsToReceiveFromBroadcast, incomingData, sizeof(commandsToReceiveFromBroadcast));
  incomingDestinationID = commandsToReceiveFromBroadcast.structDestinationID;
  incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
  incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
  incomingCommand = commandsToReceiveFromBroadcast.structCommand;
  Serial.print("Bytes recieved from ESP-NOW Message: ");Serial.println(len);
  Serial.print("Sender ID = "); Serial.println(incomingSenderID);
  Serial.print("Destination ID= ");Serial.println(incomingDestinationID);
  Serial.print("Command = "); Serial.println(incomingCommand); 
  if (incomingDestinationID =="Dome"){
    DBG("ESP-NOW Command Accepted\n");
    Serial.print("Target ID= "); Serial.println(incomingTargetID);
    if (incomingTargetID == "RS"){
        Serial.println("Sending out rsSerial");
        writeRsSerial(incomingCommand);
    } else if (incomingTargetID == "HP"){
        Serial.println("Sending out hpSerial");
        writeHpSerial(incomingCommand);
    } else if (incomingTargetID == "DS"){
        Serial.print("Execute Local Command = "); Serial.println(incomingCommand);
        inputString = incomingCommand;
        stringComplete = true; 
    } else {
        Serial.println("Wrong Target ID Sent");
      }
  }
  else {Serial.println("ESP-NOW Message Ignored");}
}

  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////
  
//Raspberry Pi              192.168.4.100
//Body Controller ESP       192.168.4.101   
//ESP-NOW Master ESP        192.168.4.110   (Only used for OTA) ************
//Dome Controller ESP       192.168.4.111   (Only used for OTA) 
//Periscope Controller ESP  192.168.4.112   (Only used for OTA)
//Remote                    192.168.4.107
//Developer Laptop          192.168.4.125
  
  // IP Address config of local ESP
  IPAddress local_IP(192,168,4,111);
  IPAddress subnet(255,255,255,0);
  IPAddress gateway(192,168,4,100);
  
  ////R2 Control Network Details
  const char* ssid = "R2D2_Control_Network";
  const char* password =  "astromech";
  
  AsyncWebServer server(80);
  


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
  WiFi.mode(WIFI_AP_STA);
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

//
void loop() {
if (millis() - MLMillis >= mainLoopDelayVar){
  MLMillis = millis();
  loopTime = millis();
  AnimatedEvent::process();
  if(startUp) {
      closeAllDoors(2,0,0);
      startUp = false;
      Serial.println("Startup");
  }
  if(Serial.available()){serialEvent();}
  if(hpSerial.available()){hpSerialEvent();}
  if(rsSerial.available()){rsSerialEvent();}

  cameraLED(blue, 5); // blue

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
          Serial.print("Command: ");Serial.println(inputBuffer);
          Serial.print("Command Length: ");Serial.println(commandLength);
          if(commandLength >= 3) {
            if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {
              doorBoard = inputBuffer[1]-'0';
              doorFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');
              if (doorFunction == 1 || doorFunction == 2){
                door = (inputBuffer[4]-'0')*10+(inputBuffer[5]-'0');
                if(commandLength >= 8){
                  Serial.println("Door Function Called");
                  doorEasingMethod = (inputBuffer[6]-'0')*10+(inputBuffer[7]-'0');
                  doorEasingDuration = (inputBuffer[8]-'0')*10+(inputBuffer[9]-'0')*100+(inputBuffer[10]-'0')*1000+(inputBuffer[11]-'0');
                } else{
                  doorEasingMethod = 0;
                  doorEasingDuration = 0;
                }
              }
              if (doorFunction != 1 || doorFunction != 2) {
                 Serial.print("Other Door Function Called ");
                if (commandLength >=6){
                Serial.println("with Easing");
                doorEasingMethod = (inputBuffer[4]-'0')*10+(inputBuffer[5]-'0');
                doorEasingDuration = (inputBuffer[8]-'0')*10+(inputBuffer[7]-'0')*100+(inputBuffer[6]-'0')*1000+(inputBuffer[9]-'0');
                }else{
                  Serial.println("without Easing");
                  doorEasingMethod = 0;
                  doorEasingDuration = 0;
                }
              }
              }                                    
            if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');}
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
            if(inputBuffer[0]=='R' || inputBuffer[0]=='r') {ledFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');}              //  Converts Sequence character values into an integer.
            
            if(commandLength >= 4) {
              if(inputBuffer[0]=='D' || inputBuffer[0]=='d' ) {}
              if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');};
              if(inputBuffer[0]=='R' || inputBuffer[0]=='r') {colorState1 = inputBuffer[3]-'0';};
            }

            if(commandLength >= 5) {
              if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {}
              else {speedState = inputBuffer[4]-'0';} 
            }

            if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {
              D_command[0]   = '\0';                                                            // Flushes Array
              D_command[0] = doorFunction;
              D_command[1] = doorBoard;
                if(door>=0) {D_command[2] = door;}
              D_command[3] = doorEasingMethod;
              D_command[4] = doorEasingDuration;
              Serial.println(doorFunction);
              Serial.println(doorEasingMethod);
              Serial.println(doorEasingDuration);
            }

            if(inputBuffer[0]=='R' || inputBuffer[0]=='r'){
              CL_command[0]   = '\0';                                                            // Flushes Array
              CL_command[0] = ledFunction;
              CL_command[1] = colorState1;
              CL_command[2] = speedState;
              CLMillis = millis();
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
        uint32_t doorEasingDuration;

        // reset ESP-NOW Variables
        inputStringCommand = "";
        targetID = "";
    
      DBG("command taken\n");

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
        DBG("Incorrect Door Value Specified, Command Aborted!");
        D_command[0] = '\0';
      }
      else {
        switch (D_command[0]) {
          case 1: openDoor(D_command[1],D_command[2],D_command[3],D_command[4]);          break;
          case 2: closeDoor(D_command[1],D_command[2],D_command[3],D_command[4]);         break;
          case 3: openAllDoors(D_command[1],D_command[3],D_command[4]);                   break;
          case 4: closeAllDoors(D_command[1],D_command[3],D_command[4]);                  break;
          case 5: shortCircuit(D_command[1],D_command[3],D_command[4]);                   break;
          case 6: allOpenClose(D_command[1],D_command[3],D_command[4]);                   break;
          case 7: allOpenCloseLong(D_command[1],D_command[3],D_command[4]);               break;
          case 8: allFlutter(D_command[1],D_command[3],D_command[4]);                     break;
          case 9: allOpenCloseRepeat(D_command[1],D_command[3],D_command[4]);             break;
          case 10: panelWave(D_command[1],D_command[3],D_command[4]);                     break;
          case 11: panelWaveFast(D_command[1],D_command[3],D_command[4]);                 break;
          case 12: openCloseWave(D_command[1],D_command[3],D_command[4]);                 break;
          case 13: marchingAnts(D_command[1],D_command[3],D_command[4]);                  break;
          case 14: panelAlternate(D_command[1],D_command[3],D_command[4]);                break;
          case 15: panelDance(D_command[1],D_command[3],D_command[4]);                    break;
          case 16: longDisco(D_command[1],D_command[3],D_command[4]);                     break;
          case 17: longHarlemShake(D_command[1],D_command[3],D_command[4]);               break;
          case 98: closeAllDoors(2,0,0);                                                  break;
          case 99: closeAllDoors(2,0,0);                                                  break;
          default: break;
        }
      }
    }

    if(CL_command[0]){
      switch(CL_command[0]){
        case 1: cameraLED(basicColors[CL_command[2]], CL_command[3]);                     break;
        case 2: break;  //reserved for future use
        case 3: break;  //reserved for future use
      }
    }

    if(ESPNOW_command[0]){
      switch(ESPNOW_command[0]){
        case 1: sendESPNOWCommand(tempESPNOWTargetID,commandSubString);                   break; 
        case 2: break;  //reserved for future use
        case 3: break;  //reserved for future use
      }
    }

    if(isStartUp) {
      isStartUp = false;
      delay(500);
    }
  }
}  //end of main loop

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                                      Radar eye Camera LED Functions                           /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void cameraLED(uint32_t color, int CLSpeed){
  int CLRLow = 1;
  int CLRHigh = 50;
      CLSpeed = map(CLSpeed, 0, 9, 1, 250);
      if (millis() - CLMillis >= CLSpeed){
        CLMillis = millis();
      if(countUp == false){                   // check to see if the boolean flag is false.  If false, starting dimming the LEDs
      
          dim=dim - random(CLRLow, CLRHigh);  // set the brightness to current minus a random number between 5 and 40. I think that
                                              //adding a random causes a less smooth transition which makes it look a little better
          colorWipe(color, dim);              // Set the LEDs to the color and brightness using the colorWheel function
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

void openDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t servoMovementDuration) {
  //Command: Dx01zz
  DBG("Open Specific Door\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    char stringToSend[20];
    switch (doorpos){
      case 1: DBG("Open Top Utility Arm\n");
              sprintf(stringToSend, "D10101%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 2: DBG("Open Bottom Utility Arm\n");
              sprintf(stringToSend, "D10102%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 3: DBG("Open Large Left Door\n");
              sprintf(stringToSend, "D10103%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 4: DBG("Open Large Right Door\n");
              sprintf(stringToSend, "D10104%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 5: DBG("Open Charge Bay Indicator Door\n");
              sprintf(stringToSend, "D10105%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 6: DBG("Open Data Panel Door\n");
              sprintf(stringToSend, "D10106%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch (doorpos){
      case 1: Serial.println("Open SMALL_PANEL_ONE");       SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, SMALL_PANEL_ONE, servoMovementDuration);     break;
      case 2: Serial.println("Open SMALL_PANEL_TWO");       SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, SMALL_PANEL_TWO, servoMovementDuration);     break;
      case 3: Serial.println("Open SMALL_PANEL_THREE");     SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, SMALL_PANEL_THREE, servoMovementDuration);   break;
      case 4: Serial.println("Open MEDIUM_PANEL_PAINTED");  SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, MEDIUM_PANEL_PAINTED, servoMovementDuration);break;
      case 5: Serial.println("Open MEDIUM_PANEL_SILVER");   SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, MEDIUM_PANEL_SILVER, servoMovementDuration); break;
      case 6: Serial.println("Open BIG_PANEL");             SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, BIG_PANEL, servoMovementDuration);           break;
      case 7: Serial.println("Open PIE_PANEL_ONE");         SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, PIE_PANEL_ONE, servoMovementDuration);       break;
      case 8: Serial.println("Open PIE_PANEL_TWO");         SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, PIE_PANEL_TWO, servoMovementDuration);       break;
      case 9: Serial.println("Open PIE_PANEL_THREE");       SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, PIE_PANEL_THREE, servoMovementDuration);     break;
      case 10: Serial.println("Open PIE_PANEL_FOUR");       SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, PIE_PANEL_FOUR, servoMovementDuration);      break;

    }
  };
  D_command[0]   = '\0';
};

void closeDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx02zz
  DBG("Close Specific Door");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    char stringToSend[20];
    switch(doorpos){
      case 1: DBG("Close Top Utility Arm\n");             
              sprintf(stringToSend, "D10201%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 2: DBG("Close Bottom Utility Arm\n");              
              sprintf(stringToSend, "D10202%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 3: DBG("Close Large Left Door\n");
              sprintf(stringToSend, "D10203%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 4: DBG("Close Large Right Door\n");
              sprintf(stringToSend, "D10204%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 5: DBG("Close Charge Bay Indicator Door\n");
              sprintf(stringToSend, "D10205%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
      case 6: DBG("Close Data Panel Door\n");
              sprintf(stringToSend, "D10206%02d%04d", servoEasingMethod, servoMovementDuration);
              sendESPNOWCommand("BC", stringToSend);                                        
              break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch (doorpos){
      case 1: Serial.println("Close SMALL_PANEL_ONE");       SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, SMALL_PANEL_ONE, servoMovementDuration);     break;
      case 2: Serial.println("Close SMALL_PANEL_TWO");       SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, SMALL_PANEL_TWO, servoMovementDuration);     break;
      case 3: Serial.println("Close SMALL_PANEL_THREE");     SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, SMALL_PANEL_THREE, servoMovementDuration);   break;
      case 4: Serial.println("Close MEDIUM_PANEL_PAINTED");  SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, MEDIUM_PANEL_PAINTED, servoMovementDuration);break;
      case 5: Serial.println("Close MEDIUM_PANEL_SILVER");   SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, MEDIUM_PANEL_SILVER, servoMovementDuration); break;
      case 6: Serial.println("Close BIG_PANEL");             SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, BIG_PANEL, servoMovementDuration);           break;
      case 7: Serial.println("Close PIE_PANEL_ONE");         SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, PIE_PANEL_ONE, servoMovementDuration);       break;
      case 8: Serial.println("Close PIE_PANEL_TWO");         SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, PIE_PANEL_TWO, servoMovementDuration);       break;
      case 9: Serial.println("Close PIE_PANEL_THREE");       SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, PIE_PANEL_THREE, servoMovementDuration);     break;
      case 10: Serial.println("Close PIE_PANEL_FOUR");       SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, PIE_PANEL_FOUR, servoMovementDuration);      break;
    }
  };
  D_command[0]   = '\0';
}


void openAllDoors(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx03
  DBG("Open all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    char stringToSend[20];
    sprintf(stringToSend, "D103%02d%04d", servoEasingMethod, servoMovementDuration);
    sendESPNOWCommand("BC",stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
      setServoEasingMethod(servoEasingMethod);
      SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK, servoMovementDuration);
  };
  D_command[0] = '\0';
}


void closeAllDoors(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx04
  DBG("Close all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    char stringToSend[20];
    sprintf(stringToSend, "D104%02d%04d", servoEasingMethod, servoMovementDuration);
    sendESPNOWCommand("BC", stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK, servoMovementDuration);
  };
  D_command[0] = '\0';
}


void shortCircuit(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx05
  // add sequence for this routine.  
}


void allOpenClose(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx06
  DBG("Open and Close All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    char stringToSend[20];
    sprintf(stringToSend, "D106%02d%04d", servoEasingMethod, servoMovementDuration);
    sendESPNOWCommand("BC", stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK, servoMovementDuration);
  };
  D_command[0]   = '\0';                                           
}


void allOpenCloseLong(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx07
  DBG("Open and Close Doors Long\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    char stringToSend[20];
    sprintf(stringToSend, "D107%02d%04d", servoEasingMethod, servoMovementDuration);
    sendESPNOWCommand("BC", stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK, servoMovementDuration);
  };
  D_command[0]   = '\0';                                                 
}


void allFlutter(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx08
  DBG("Flutter All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    char stringToSend[20];
    sprintf(stringToSend, "D108%02d%04d", servoEasingMethod, servoMovementDuration);
    sendESPNOWCommand("BC", stringToSend);  
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK, servoMovementDuration);
  };
  D_command[0]   = '\0';   
}


void allOpenCloseRepeat(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx09
  DBG("Open and Close All Doors Repeat\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    char stringToSend[20];
    sprintf(stringToSend, "D109%02d%04d", servoEasingMethod, servoMovementDuration);
    sendESPNOWCommand("BC", stringToSend);
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK, servoMovementDuration);
  };
  D_command[0]   = '\0';             
}


void panelWave(int servoBoard, int servoEasingMethod, uint32_t servoMovementDurationParam){
  // Command: Dx10
  DBG("Wave\n");
  servoMovementDuration = servoMovementDurationParam;
  sprintf(stringToSend, "D110%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BC", stringToSend); break;
    case 2: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 3: sendESPNOWCommand("BC", stringToSend);
            DelayCall::schedule([servoMovementDuration] {SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, servoMovementDuration);}, 3000); break;
    case 4: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, servoMovementDuration); break;
            DelayCall::schedule([stringToSend]{sendESPNOWCommand("BC", stringToSend);}, 2000); break;
  }
  D_command[0]   = '\0';                                             
}


void panelWaveFast(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx11  
  DBG("Wave Fast\n");
  char stringToSend[20];
  sprintf(stringToSend, "D111%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BC", stringToSend);  break;
    case 2: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, servoMovementDuration);  break;
    case 3: sendESPNOWCommand("BC", stringToSend);     
//            DelayCall::schedule([servoMovementDuration] {SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, servoMovementDuration);}, 3000); break;
    case 4: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, servoMovementDuration); break;
//            DelayCall::schedule([stringToSend] {sendESPNOWCommand("BC", stringToSend);}, 2000); break;
  }
  D_command[0]   = '\0';                                             
}


void openCloseWave(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx12
  DBG("Open Close Wave \n");
  char stringToSend[20];
  sprintf(stringToSend, "D112%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BC", stringToSend); break;
    case 2: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, servoMovementDuration);  break;
    case 3: sendESPNOWCommand("BC", stringToSend);     
//            DelayCall::schedule([servoMovementDuration] {SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, servoMovementDuration);}, 3000); break;
    case 4: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, servoMovementDuration); 
//            DelayCall::schedule([stringToSend] {sendESPNOWCommand("BC", stringToSend);}, 2000); break;
  }
  D_command[0]   = '\0';                                             
}


void marchingAnts(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx13
  DBG("Marching Ants\n");
  char stringToSend[20];
  sprintf(stringToSend, "D113%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BC", stringToSend);                                      break;
    case 2: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 3: sendESPNOWCommand("BC", stringToSend);
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 4: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, servoMovementDuration); 
            sendESPNOWCommand("BC", stringToSend); break;
  }
  D_command[0]   = '\0';                                             
}


void panelAlternate(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx14
  DBG("Panel Alternate\n");
  char stringToSend[20];
  sprintf(stringToSend, "D114%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BC", stringToSend);                                    break;
    case 2: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 3: sendESPNOWCommand("BC", stringToSend); 
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 4: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, servoMovementDuration); break;
            sendESPNOWCommand("BC", stringToSend); break;
  }
  D_command[0]   = '\0';                                             
}                                                            


void panelDance(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
 // Command: Dx15
  DBG("Panel Dance\n");
  char stringToSend[20];
  sprintf(stringToSend, "D115%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BC", stringToSend);                                    break;
    case 2: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 3: sendESPNOWCommand("BC", stringToSend); 
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 4: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, servoMovementDuration); break;
            sendESPNOWCommand("BC", stringToSend); break;
  }
  D_command[0]   = '\0';                                             
}


void longDisco(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx16
  DBG("Panel Dance Long\n");
  char stringToSend[20];
  sprintf(stringToSend, "D116%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BC", stringToSend);                                    break;
    case 2: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 3: sendESPNOWCommand("BC", stringToSend); 
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 4: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, servoMovementDuration); break;
            sendESPNOWCommand("BC", stringToSend);break;
  }
  D_command[0]   = '\0';                                             
}


void longHarlemShake(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx17
  DBG("Harlem Shake\n");
  char stringToSend[20];
  sprintf(stringToSend, "D117%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: sendESPNOWCommand("BC", stringToSend);                                    break;
    case 2: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 3: sendESPNOWCommand("BC", stringToSend); 
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 4: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, servoMovementDuration); break;
            sendESPNOWCommand("BC", stringToSend);break;
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
  DBG("%s\n", inputString);
}


void hpSerialEvent() {
  while (hpSerial.available()) {
    char inChar = (char)hpSerial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  DBG("%s\n", inputString);
}


void rsSerialEvent() {
  while (rsSerial.available()) {
    char inChar = (char)rsSerial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  DBG("%s\n", inputString);
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
  Serial.println("Printing to rsSerial");
}

void writeHpSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    hpSerial.write(completeString[i]);
  }
  Serial.println("Printing to hpSerial");
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
  }else if (starget == "EN" || starget == "BC" || starget == "BL" || starget == "ST"){
    sdest = "Body";
  }
  
  commandsToSendtoBroadcast.structDestinationID = sdest;
  DBG("sdest: %s\n", sdest);
  commandsToSendtoBroadcast.structTargetID = starget;
  commandsToSendtoBroadcast.structSenderID = "Dome";
  commandsToSendtoBroadcast.structCommand = scomm;
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
  if (result == ESP_OK) {
    DBG("Sent with success\n");
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

void DBG(char *fmt, ...){
  va_list ap; /* points to each unnamed arg in turn */
  char *p, *sval;
  int ival;
  double dval;
  va_start(ap, fmt); /* make ap point to 1st unnamed arg */
  for (p = fmt; *p; p++) 
    {
      if (*p != '%'){
        putchar(*p);
        continue;
      }
      switch (*++p){
        case 'd': ival = va_arg(ap, int); printf("%d", ival); break;
        case 'f': dval = va_arg(ap, double);printf("%f", dval); break;
        case 's': for (sval = va_arg(ap, char *); *sval; sval++)
                    putchar(*sval);  break;
        default: putchar(*p); break;
      }
    }
   va_end(ap); /* clean up when done */
}


//void DBG(char *format, ...) {
//        if (!debugflag)
//                return;
//        va_list ap;
//        va_start(ap, format);
//        vfprintf(stderr, format, ap);
//        va_end(ap);
//}
//

void DBG_1(char *format, ...) {
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
  WiFi.begin();
  while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.println("Connecting to WiFi..");
  Serial.println(WiFi.localIP());
  }
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
