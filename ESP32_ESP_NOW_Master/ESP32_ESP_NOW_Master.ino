// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>

//Used for ESP-NOW
#include "esp_wifi.h"
#include <esp_now.h>

// Used for Software Serial to allow more useful naming
#include <SoftwareSerial.h>


//reeltwo libaries
#include "ReelTwo.h"
#include "core/DelayCall.h"


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
  int commandState = 0;

  uint32_t ESPNOW_command[6]  = {0,0,0,0,0,0};
  int espNowCommandFunction = 0;
  String espNowCommandFunctionString;
  String tempESPNOWTargetID;

// Flags to enable/disable debugging in runtime
  int debugflag = 0;
  int debugflagparam = 0;  // debugging for params recieved from clients

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
  #define BC_BAUD_RATE 115200

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
  char macStr[18];
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  DBG("Packet to: %s\n", macStr);
  DBG(" send status:\t");
  DBG(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\n" : "Delivery Fail\n");
}

  // Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&commandsToReceiveFromBroadcast, incomingData, sizeof(commandsToReceiveFromBroadcast));
  incomingDestinationID = commandsToReceiveFromBroadcast.structDestinationID;
  incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
  incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
  incomingCommand = commandsToReceiveFromBroadcast.structCommand;
  DBG("Bytes received from ESP-NOW Message: %s\n", len);
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
        writeFuSerial(incomingCommand)
    } else if (incomingTargetID == "NM"){
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
  //ESP-NOW Master ESP        192.168.4.110  
  //Dome Controller ESP       192.168.4.111  ************
  //Periscope Controller ESP  192.168.4.112
  //Remote                    192.168.4.107
  //Developer Laptop          192.168.4.125
  
  // IP Address config of local ESP
  IPAddress local_IP(192,168,4,110);
  IPAddress subnet(255,255,255,0);
  IPAddress gateway(192,168,4,100);
  
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

  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

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
  
    if(startUp) {
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
        if( inputBuffer[0]=='E' ||        // Command designatore for internal ESP functions
            inputBuffer[0]=='e' ||        // Command designatore for internal ESP functions
            inputBuffer[0]=='N' ||        // Command for Sending ESP-NOW Messages
            inputBuffer[0]=='n' ||        // Command for Sending ESP-NOW Messages
            inputBuffer[0]=='S' ||        // Command for sending Serial Strings out Serial ports
            inputBuffer[0]=='s'           // Command for sending Serial Strings out Serial ports


         ){commandLength = strlen(inputBuffer);                     //  Determines length of command character array.
            DBG("Command Length is: %i\n", commandLength);
            if(commandLength >= 3) {
              if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');};
              if(inputBuffer[0]=='N' || inputBuffer[0]=='n') {
                for (int i=1; i<=commandLength; i++){
                  char inCharRead = inputBuffer[i];
                  inputStringCommand += inCharRead;                   // add it to the inputString:
                }
                DBG("\nFull Command Recieved: %s ",inputStringCommand);
                espNowCommandFunctionString = inputStringCommand.substring(0,2);
                espNowCommandFunction = espNowespCommandFunctionString.toInt();
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
        
      DBG("command Proccessed\n");

    }


      if(ESP_command[0]){
        switch (ESP_command[0]){
          case 1: Serial.println("Controller: Master ESP-NOW Controller");   
                  ESP_command[0]   = '\0';  break;
          case 2: Serial.println("Resetting the ESP in 3 Seconds");
                  DelayCall::schedule([] {ESP.restart();}, 3000) ; break;
          case 3: connectWiFi();            break;
          case 4: ESP.restart();            break;
          case 5: break;
          case 6: break;
          case 7: break;
          case 8: break;
          case 9: break;
          case 10: toggleDebug();           break;
          case 11: toggleDebugParam();      break;
        }
      }
      if(ESPNOW_command[0]){
        switch(ESPNOW_command[0]){
          case 1: sendESPNOWCommand(tempESPNOWTargetID,commandSubString); break; 
          case 2: break;  //reserved for future use
          case 3: break;  //reserved for future use      
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
  DBG("%s\n", inputString);
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

void sendESPNOWCommand(String starget,String scomm){
  String sdest;
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

void DBG(char *format, ...) {
        if (!debugflag)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}


void DBG_P(char *format, ...) {
        if (!debugflagparam)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}


void toggleDebug(){
  debugflag = !debugflag;
  if (debugflag == 1){
    Serial.println(("Debugging Enabled \n");
    }
  else{
    Serial.println("Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}


void toggleDebugParam(){
  debugflagparam = !debugflagparam;
  if (debugflagparam == 1){
    Serial.println("Parameter Debugging Enabled \n");
    }
  else{
    erial.println(("Parameter Debugging Disabled\n");
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
