// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>

//Used for ESP-NOW
#include "esp_wifi.h"
#include <esp_now.h>

//Used for PC9685 - Servo Expansion Board
#include <Wire.h>

//reeltwo libaries
#include "ReelTwo.h"
#include "core/DelayCall.h"
#include "ServoDispatchPCA9685.h"
#include "ServoSequencer.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                                                                                                       *****///
///*****                            Created by Greg Hulette.  I started with the code from flthymcnsty         *****///
///*****                                                                                                       *****///
///*****                                     So exactly what does this all do.....?                            *****///
///*****                       - Receives commands via Serial and transmits out via ESP-NOW                    *****///
///*****                       - Receives commands via ESP-NOW and tranmits out via Serial                     *****///
///*****                       - Sends Serial commands to the LED Controller and the Stealth Board             *****///
///*****                                                                                                       *****///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
///*****              ReelTwo Servo Set Up                       *****///
/////////////////////////////////////////////////////////////////////////

#define TOP_UTILITY_ARM       0x0001 //b0000000001
#define BOTTOM_UTILITY_ARM    0x0002 //b0000000010
#define LARGE_LEFT_DOOR       0x0004 //b0000000100
#define LARGE_RIGHT_DOOR      0x0008 //b0000001000
#define CHARGE_BAY_DOOR       0x0010 //b0000010000
#define DATA_PANEL_DOOR       0x0020 //b0000100000

#define UTILITY_ARMS_MASK     (TOP_UTILITY_ARM|BOTTOM_UTILITY_ARM)
#define LARGE_DOORS_MASK      (LARGE_LEFT_DOOR|LARGE_RIGHT_DOOR)
#define SMALL_DOORS_MASK      (CHARGE_BAY_DOOR|DATA_PANEL_DOOR)
#define ALL_DOORS_MASK        (LARGE_DOORS_MASK|SMALL_DOORS_MASK)
#define ALL_SERVOS_MASK       (ALL_DOORS_MASK|UTILITY_ARMS_MASK)

// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.

//     Pin  Min, ,Max,  Group ID  (Change the Min and Max to your Droids actual limits)
const ServoSettings servoSettings[] PROGMEM = {
    { 1,  700, 2400, TOP_UTILITY_ARM },       /* 0: Top Utility Arm */
    { 2,  700, 2400, BOTTOM_UTILITY_ARM },    /* 1: Bottom Utility Arm */
    { 3,  700, 2400, LARGE_LEFT_DOOR },       /* 2: Large Left Door as viewing from looking at R2 */
    { 4,  700, 2400, LARGE_RIGHT_DOOR },      /* 3: Large Right door as viewing from looking at R2 */
    { 5,  700, 2400, CHARGE_BAY_DOOR },       /* 4: Charge Bay Inidicator Door*/
    { 6,  700, 2400, DATA_PANEL_DOOR }        /* 5: Data Panel Door */
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
  int commandLength;
  String serialPort;
  String serialStringCommand;

  uint32_t ESP_command[6]  = {0,0,0,0,0,0};
  int espCommandFunction     = 0;

  uint32_t ESPNOW_command[6]  = {0,0,0,0,0,0};
  int espNowCommandFunction = 0;
  String espNowCommandFunctionString;
  String tempESPNOWTargetID;

// Flags to enable/disable debugging in runtime
  int debugflag = 1;
  int debugflag1 = 0;  // debugging for params recieved from clients

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
  
  unsigned long mainLoopTime; // We keep track of the "Main Loop time" in this variable.
  unsigned long MLMillis;
  byte mainLoopDelayVar = 5;

  //////////////////////////////////////////////////////////////////////
  ///******       Serial Ports Specific Setup                   *****///
  //////////////////////////////////////////////////////////////////////

  #define RXBC 15
  #define TXBC 16 
  #define RXFU 25
  #define TXFU 26 
  #define bcSerial Serial1
  #define fuSerial Serial2

  #define BC_BAUD_RATE 115200
  #define FU_BAUD_RATE 115200

/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

//  MAC Addresses used in the Droid
//  ESP-NOW Master =        {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
//  Dome Controller =       {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
//  Periscope Controller =  {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};

//    MAC Address of your receivers 
  uint8_t domePeerMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
  uint8_t periscopePeerMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};

//    MAC Address to broadcast to all senders at once
  uint8_t broadcastMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//    MAC Address for the Local ESP to use - This prevents having to capture the MAC address of reciever boards.
  uint8_t newLocalMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

//  // Define variables to store commands to be sent
  String senderID;
  String destinationID;
  String targetID;
  String command;
  String commandSubString;
    
//

//  // Define variables to store incoming commands
  String incomingDestinationID;
  String incomingTargetID;  
  String incomingSenderID;
  String incomingCommand;
//    
//  // Variable to store if sending data was successful
  String success;
//
//  //Structure example to send data
//  //Must match the receiver structure
  typedef struct struct_message {
      String structSenderID;
      String structDestinationID;
      String structTargetID;  
      String structCommand;
  } struct_message;
//
//  // Create a struct_message calledcommandsTosend to hold variables that will be sent
  struct_message commandsToSendtoDome;
  struct_message commandsToSendtoPeriscope;
  struct_message commandsToSendtoBroadcast;

//
//  // Create a struct_message to hold incoming commands from the Dome
  struct_message commandsToReceiveFromDome;
  struct_message commandsToReceiveFromPeriscope;
  struct_message commandsToReceiveFromBroadcast;
//
  esp_now_peer_info_t peerInfo;

//  // Callback when data is sent
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

  // Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&commandsToReceiveFromBroadcast, incomingData, sizeof(commandsToReceiveFromBroadcast));
  incomingDestinationID = commandsToReceiveFromBroadcast.structDestinationID;
  incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
  incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
  incomingCommand = commandsToReceiveFromBroadcast.structCommand;
  DBG("Bytes received from ESP-NOW Message: %i\n", len);
  DBG("Sender ID = %s\n",incomingSenderID);
  DBG("Destination ID= %s\n" ,incomingDestinationID);
  DBG("Target ID= %s\n", incomingTargetID);
  DBG("Command = %s\n" , incomingCommand); 
  if (incomingDestinationID =="Body"){
    DBG("ESP-NOW Command Accepted\n");
    DBG("Target ID= %s\n", incomingTargetID);
    if (incomingTargetID == "BC" || incomingTargetID == "BL" || incomingTargetID =="ST"){
        DBG("Sending %s out bcSerial\n", incomingCommand);
        writeBcSerial(incomingCommand);
    } else if (incomingTargetID == "FU"){
        DBG("Sending %s out fuSerial\n", incomingCommand);
        writeFuSerial(incomingCommand);
    } else if (incomingTargetID == "BS"){
        DBG("Execute Local Command = %s\n", incomingCommand);
        inputString = incomingCommand;
        stringComplete = true; 
    } else {DBG("Wrong Target ID Sent\n");}
  }
  else {DBG("ESP-NOW Message Ignored\n");}
}

//////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////
  
  //Raspberry Pi              192.168.4.100
  //Body Controller ESP       192.168.4.101
  //ESP-NOW Master ESP        192.168.4.110   (Only used for OTA)  ************
  //Dome Controller ESP       192.168.4.111   (Only used for OTA)  
  //Periscope Controller ESP  192.168.4.112   (Only used for OTA)
  //Remote                    192.168.4.107
  //Developer Laptop          192.168.4.125
  
  // IP Address config of local ESP
  IPAddress local_IP(192,168,4,110);
  IPAddress subnet(255,255,255,0);
  IPAddress gateway(192,168,4,101);
  
   ////R2 Control Network Details
  const char* ssid = "R2D2_Control_Network";
  const char* password =  "astromech";
  
  AsyncWebServer server(80);
  
void setup(){
  //Initialize the Serial Ports
  Serial.begin(115200);
  bcSerial.begin(BC_BAUD_RATE, SERIAL_8N1, RXBC, TXBC);
  fuSerial.begin(FU_BAUD_RATE, SERIAL_8N1, RXFU, TXFU);

  Serial.print("\n\n\n----------------------------------------\n");
  Serial.print("Booting up the Master ESP-NOW Controller\n");

  
  //Initialize I2C for the Servo Expander Board
  Wire.begin();
  
  //Initialize the ReelTwo Library
  SetupEvent::ready();
  
  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

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

  // Register peer configuration
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peers  
    memcpy(peerInfo.peer_addr, domePeerMACAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add Dome ESP-NOW peer");
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
  
}   // end of setup

void loop(){
  if (millis() - MLMillis >= mainLoopDelayVar){
      MLMillis = millis();
      AnimatedEvent::process();

    if(startUp) {
      closeAllDoors(1,0,0,0);
      startUp = false;
      Serial.print("Startup complete\nStarting main loop\n\n\n");
    }  
    if(Serial.available()){serialEvent();}
    if(bcSerial.available()){bcSerialEvent();}
    if(fuSerial.available()){fuSerialEvent();}

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
                  if (commandLength >= 13){
                    DBG("Variable Speed Selected\n");
                    cVarSpeedMin = (inputBuffer[7]-'0')*1000+(inputBuffer[8]-'0')*100+(inputBuffer[9]-'0')*10+(inputBuffer[10]-'0');                
                    cVarSpeedMax = (inputBuffer[11]-'0')*1000+(inputBuffer[12]-'0')*100+(inputBuffer[13]-'0')*10+(inputBuffer[14]-'0');  
                  } else {
                    DBG("No Variable Speed selected\n");
                    cVarSpeedMin = (inputBuffer[7]-'0')*1000+(inputBuffer[8]-'0')*100+(inputBuffer[9]-'0')*10+(inputBuffer[10]-'0');                
                    cVarSpeedMax = cVarSpeedMin; 
                  }              
                  delayCallTime = 0;
                } else if (inputBuffer[4] == 'B' || inputBuffer[4] == 'b'){
                  DBG("Both Easing and Delay Call \n");
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
                  DBG("No easing or DelayCall time specified \n");
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
//                DBG("\nFull Command Recieved: %s ",inputStringCommand);
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
                if (serialPort == "BC"){
                  writeBcSerial(serialStringCommand);
                } else if (serialPort == "FU"){
                  writeFuSerial(serialStringCommand);
                } 
                serialStringCommand = "";
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

              DBG("Door Function Called: %d\n",doorFunction);
              DBG("Easing Method: %d \n",doorEasingMethod);
              DBG("VarSpeed - Min:%d, Max:%d \n",cVarSpeedMin, cVarSpeedMax);
              DBG("DelayCall Duration: %d\n",delayCallTime);
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

      DBG("command Proccessed\n");

    }


    if(ESP_command[0]){
      switch (ESP_command[0]){
        case 1: Serial.println("Controller: Master ESP-NOW Controller");   
                ESP_command[0]   = '\0';                                                                break;
        case 2: Serial.println("Resetting the ESP in 3 Seconds");
                DelayCall::schedule([] {ESP.restart();}, 3000) ;                                        break;
        case 3: connectWiFi();                                                                          break;
        case 4: ESP.restart();                                                                          break;
        case 5: break;
        case 6: break;
        case 7: break;
        case 8: break;
        case 9: scan_i2c();ESP_command[0]='\0';                                                         break;
        case 10: toggleDebug();                                                                         break;
        case 11: toggleDebug1();                                                                        break;
      }
    }
    if(ESPNOW_command[0]){
      switch(ESPNOW_command[0]){
        case 1: sendESPNOWCommand(tempESPNOWTargetID,commandSubString);                                 break; 
        case 2: break;  //reserved for future use
        case 3: break;  //reserved for future use      
      }
    }

    if(D_command[0]) {
      if((D_command[0] == 1 || D_command[0] == 2) && D_command[1] >= 11) {
        DBG("Incorrect Door Value Specified, Command Aborted!");
        D_command[0] = '\0';
      }
      else {
        switch (D_command[0]) {
case 1: openDoor(D_command[1],D_command[2],D_command[3],D_command[4],D_command[5]);                     break;
          case 2: closeDoor(D_command[1],D_command[2],D_command[3],D_command[4],D_command[5]);          break;
          case 3: openAllDoors(D_command[1],D_command[3],D_command[4],D_command[5]);                    break;
          case 4: closeAllDoors(D_command[1],D_command[3],D_command[4],D_command[5]);                   break;
          case 5: shortCircuit(D_command[1],D_command[3],D_command[4],D_command[5]);                    break;
          case 6: allOpenClose(D_command[1],D_command[3],D_command[4],D_command[5]);                    break;
          case 7: allOpenCloseLong(D_command[1],D_command[3],D_command[4],D_command[5]);                break;
          case 8: allFlutter(D_command[1],D_command[3],D_command[4],D_command[5]);                      break;
          case 9: allOpenCloseRepeat(D_command[1],D_command[3],D_command[4],D_command[5]);              break;
          case 10: panelWave(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);         break;
          case 11: panelWaveFast(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);     break;
          case 12: openCloseWave(D_command[1],D_command[3],D_command[4],D_command[5],D_command[6]);     break;
          case 13: marchingAnts(D_command[1],D_command[3],D_command[4],D_command[5]);                   break;
          case 14: panelAlternate(D_command[1],D_command[3],D_command[4],D_command[5]);                 break;
          case 15: panelDance(D_command[1],D_command[3],D_command[4],D_command[5]);                     break;
          case 16: longDisco(D_command[1],D_command[3],D_command[4],D_command[5]);                      break;
          case 17: longHarlemShake(D_command[1],D_command[3],D_command[4],D_command[5]);                break;
          case 98: closeAllDoors(2,0,0,0);                                                              break;
          case 99: closeAllDoors(2,0,0,0);                                                              break;
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
    setServoEasingMethod(servoEasingMethod);
    switch (doorpos){
      case 1: DBG("Open Top Utility Arm\n");            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, TOP_UTILITY_ARM, varSpeedMin, varSpeedMax);     break;
      case 2: DBG("Open Bottom Utility Arm\n");         SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, BOTTOM_UTILITY_ARM, varSpeedMin, varSpeedMax);  break;
      case 3: DBG("Open Large Left Door\n");            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, LARGE_LEFT_DOOR, varSpeedMin, varSpeedMax);     break;
      case 4: DBG("Open Large Right Door\n");           SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, LARGE_RIGHT_DOOR, varSpeedMin, varSpeedMax);    break;
      case 5: DBG("Open Charge Bay Indicator Door\n");  SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, CHARGE_BAY_DOOR, varSpeedMin, varSpeedMax);     break;
      case 6: DBG("Open Data Panel Door\n");            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, DATA_PANEL_DOOR, varSpeedMin, varSpeedMax);     break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG("Open SMALL_PANEL_ONE\n");      
              sprintf(stringToSend, "D20101E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
//             sendESPNOWCommand("DS", stringToSend);  break;
              sendESPNOWCommand("DS", "D201001E0100010001");  break;
      case 2: DBG("Open SMALL_PANEL_TWO\n");      
              sprintf(stringToSend, "D20102E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 3: DBG("Open SMALL_PANEL_THREE\n");    
              sprintf(stringToSend, "D20103E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 4: DBG("Open MEDIUM_PANEL_PAINTED\n"); 
              sprintf(stringToSend, "D20104E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 5: DBG("Open MEDIUM_PANEL_SILVER\n");  
              sprintf(stringToSend, "D20105E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 6: DBG("Open BIG_PANEL\n");            
              sprintf(stringToSend, "D20106E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 7: DBG("Open PIE_PANEL_ONE\n");         
              sprintf(stringToSend, "D20107E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 8: DBG("Open PIE_PANEL_TWO\n");        
              sprintf(stringToSend, "D20108E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
              sendESPNOWCommand("DS", stringToSend);  break;
      case 9: DBG("Open PIE_PANEL_THREE\n");      
              sprintf(stringToSend, "D20109E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 10: DBG("Open PIE_PANEL_FOUR\n");      
              sprintf(stringToSend, "D20110E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
    }
  };
  D_command[0]   = '\0';
};


void closeDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx02zz
  DBG("Close Specific Door");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch(doorpos){    
      case 1: DBG("Close Top Utility Arm\n");SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, TOP_UTILITY_ARM, varSpeedMin, varSpeedMax);  break;
      case 2: DBG("Close Bottom Utility Arm\n");SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, BOTTOM_UTILITY_ARM, varSpeedMin, varSpeedMax);  break;
      case 3: DBG("Close Large Left Door\n");SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, LARGE_LEFT_DOOR, varSpeedMin, varSpeedMax);break;
      case 4: DBG("Close Large Right Door\n");SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, LARGE_RIGHT_DOOR, varSpeedMin, varSpeedMax);  break;
      case 5: DBG("Close Charge Bay Indicator Door\n");SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, CHARGE_BAY_DOOR, varSpeedMin, varSpeedMax);break;
      case 6: DBG("Close Data Panel Door\n");SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, DATA_PANEL_DOOR, varSpeedMin, varSpeedMax);  break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG("Close SMALL_PANEL_ONE\n");     
              sprintf(stringToSend, "D20201E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 2: DBG("Close SMALL_PANEL_TWO\n");      
              sprintf(stringToSend, "D20202E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 3: DBG("Close SMALL_PANEL_THREE\n");   
              sprintf(stringToSend, "D20203E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 4: DBG("Close MEDIUM_PANEL_PAINTED\n"); 
              sprintf(stringToSend, "D20204E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 5: DBG("Close MEDIUM_PANEL_SILVER\n");  
              sprintf(stringToSend, "D20205E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 6: DBG("Close BIG_PANEL\n");            
              sprintf(stringToSend, "D20206E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 7: DBG("Close PIE_PANEL_ON\nE");        
              sprintf(stringToSend, "D20207E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 8: DBG("Close PIE_PANEL_TWO\n");        
              sprintf(stringToSend, "D20208E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 9: DBG("Close PIE_PANEL_THREE\n");      
              sprintf(stringToSend, "D20209E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
             sendESPNOWCommand("DS", stringToSend);  break;
      case 10:  DBG("Close PIE_PANEL_FOUR\n");      
                sprintf(stringToSend, "D20210E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
               sendESPNOWCommand("DS", stringToSend);  break;
    }
  };
  D_command[0]   = '\0';
}


void openAllDoors(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx03
  String easing = "E";
  DBG("Open all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D203E%02i%04i%04i ", servoEasingMethod, varSpeedMin, varSpeedMax);
    DBG("Sent the Command: %s to the sendESPNOWCommand Function\n", stringToSend);
    sendESPNOWCommand("DS", "D2033015001500");  
  };
  D_command[0] = '\0';
}

  
void closeAllDoors(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx04
  DBG("Close all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D204E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    DBG("Sent the Command: %s to the sendESPNOWCommand Function\n", stringToSend);
    sendESPNOWCommand("DS", stringToSend); 
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
      setServoEasingMethod(servoEasingMethod);
      SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D206E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("DS", stringToSend); 
  };
  D_command[0]   = '\0';                                           
}


void allOpenCloseLong(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax){
  // Command: Dx07
  DBG("Open and Close Doors Long\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "D207E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("DS", stringToSend); 
  };
  D_command[0]   = '\0';                                                 
}


void allFlutter(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax){
  // Command: Dx08
  DBG("Flutter All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    sprintf(stringToSend, "D208E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
   sendESPNOWCommand("DS", stringToSend);  
  };
  D_command[0]   = '\0';   
}


void allOpenCloseRepeat(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax){
  // Command: Dx09
  DBG("Open and Close All Doors Repeat\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    sprintf(stringToSend, "D209E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
    sendESPNOWCommand("DS", stringToSend);  
  };
  D_command[0]   = '\0';             
}


void panelWave(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax, uint32_t delayCallDuration){
  // Command: Dx10
  DBG("Wave\n");
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;
  sprintf(stringToSend, "D210E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
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
  DBG("Wave Fast\n");
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;
  sprintf(stringToSend, "D211E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
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
  DBG("Open Close Wave \n");
  fVarSpeedMin = varSpeedMin;
  fVarSpeedMax = varSpeedMax;  
  sprintf(stringToSend, "D212E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
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


void marchingAnts(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx13
  DBG("Marching Ants\n");
  sprintf(stringToSend, "D213E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
            sendESPNOWCommand("DS", stringToSend); break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
  }
  D_command[0]   = '\0';                                             
}


void panelAlternate(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx14
  DBG("Panel Alternate\n");
  sprintf(stringToSend, "D214E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend);  break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
            sendESPNOWCommand("DS", stringToSend);  break;
    case 4: sendESPNOWCommand("DS", stringToSend);
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
  }
  D_command[0]   = '\0';                                             
}                                                            


void panelDance(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
 // Command: Dx15
  DBG("Panel Dance\n");
  sprintf(stringToSend, "D215E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend);  break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
            sendESPNOWCommand("DS", stringToSend);   break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);  break;
  }
  D_command[0]   = '\0';                                             
}


void longDisco(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx16
  DBG("Panel Dance Long\n");
  sprintf(stringToSend, "D216E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend);  break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
            sendESPNOWCommand("DS", stringToSend);    break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);  break;
  }
  D_command[0]   = '\0';                                             
}


void longHarlemShake(int servoBoard, int servoEasingMethod, uint32_t varSpeedMin, uint32_t varSpeedMax) {
  // Command: Dx17
  DBG("Harlem Shake\n");
  sprintf(stringToSend, "D217E%02d%04d%04d", servoEasingMethod, varSpeedMin, varSpeedMax);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax); break;
    case 2: sendESPNOWCommand("DS", stringToSend);  break;
    case 3: SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);
            sendESPNOWCommand("DS", stringToSend);    break;
    case 4: sendESPNOWCommand("DS", stringToSend); 
            SEQUENCE_PLAY_ONCE_VARSPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, varSpeedMin, varSpeedMax);  break;
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
//  DBG("%s\ n", inputString);
}
void bcSerialEvent() {
  while (bcSerial.available()) {
    char inChar = (char)bcSerial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  DBG("%s\n", inputString);
}

void fuSerialEvent() {
  while (fuSerial.available()) {
    char inChar = (char)fuSerial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  DBG("%s\n", inputString);
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
void writeBcSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++)
  {
    bcSerial.write(completeString[i]);
  }
}

void writeFuSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++)
  {
    fuSerial.write(completeString[i]);
  }
}


//////////////////////////////////////////////////////////////////////
///*****             ESP-NOW Functions                        *****///
//////////////////////////////////////////////////////////////////////

void sendESPNOWCommand(String starget, String scomm){
  String sdest;
  DBG("Desitation is %s\n", starget);
  DBG("Recieved the command: %s from the door function\n", scomm);
  
  if (starget == "DS" || starget == "RS" || starget == "HP"){
    sdest = "Dome";
  } else if (starget == "PC" || starget == "PL"){
    sdest = "Periscope";
  }
  commandsToSendtoBroadcast.structDestinationID = sdest;
  DBG("sdest: %s\n", sdest);
  commandsToSendtoBroadcast.structTargetID = starget;
  commandsToSendtoBroadcast.structSenderID = "Body";
  commandsToSendtoBroadcast.structCommand = scomm;
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
  if (result == ESP_OK) {
    DBG("Sent with success\n");
//    DBG("Sent the command: %s to ESP-NOW Neighbors\n", scomm);
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
