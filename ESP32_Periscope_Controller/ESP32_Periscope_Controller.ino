#define DEBUG

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                                                                                                                                                           *****///
///*****                       Created by Greg Hulette.  I started with the code from flthymcnsty
///*****                                                                                                                                                           *****///
///*****                                 So exactly what does this all do.....?                                                            *****///
///*****                       - Sends Serial commands to the Uppity Spinner                                                                                       *****///
///*****                                                                                                                                                           *****///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
    
    char inputBuffer[25];
    String inputString;         // a string to hold incoming data
    String inputStringCommand;
    volatile boolean stringComplete  = false;      // whether the serial string is complete
    String autoInputString;         // a string to hold incoming data
    volatile boolean autoComplete    = false;    // whether an Auto command is setA
    int displayState;
    int typeState;
    int commandLength;

    uint32_t ESP_command[6]  = {0,0,0,0,0,0};
    int commandState = 0;

    uint32_t ESPNOW_command[6]  = {0,0,0,0,0,0};
    int espNowCommandState = 0;
    String espNowCommandStateString;
    String tempESPNOWTargetID;
 
#ifdef DEBUG
  #define DPRINT(x)     Serial.print (x)
  #define DPRINTLN(x)  Serial.println (x)
#else
  #define DPRINT(x)
  #define DPRINTLN(x) 
#endif
 
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

  #define RXFU 19
  #define TXFU 18 
  #define RXPL 25
  #define TXPL 27
#define plSerial Serial1
#define fuSerial Serial2

#define PL_BAUD_RATE 9600
#define BAUD_RATE 115200

///////////////////////////////////////////////////////////////////////////
/////*****              ESP NOW Set Up                       *****///
///////////////////////////////////////////////////////////////////////////


//    MAC Address of your receivers - Not really used but have it in case it's needed.
    uint8_t domePeerMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
    uint8_t bodyPeerMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

//    MAC Address to broadcast to all senders at once - Mainly Used in my code
    uint8_t broadcastMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//    MAC Address for the Local ESP to use - This prevents having to capture the MAC address of reciever boards.
    uint8_t newLocalMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};

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

//
//  // Callback when data is sent
    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
      char macStr[18];
      Serial.print("Packet to: ");
      // Copies the sender mac address to a string
      snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      Serial.print(macStr);
      Serial.print(" send status:\t");
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }
//
//  // Callback when data is received
      void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
      memcpy(&commandsToReceiveFromBroadcast, incomingData, sizeof(commandsToReceiveFromBroadcast));
      DPRINT("Bytes received from ESP-NOW Message: ");DPRINTLN(len);
      incomingDestinationID = commandsToReceiveFromBroadcast.structDestinationID;
      incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
      incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
      incomingCommand = commandsToReceiveFromBroadcast.structCommand;
      DPRINT("Sender ID = ");DPRINTLN(incomingSenderID);
      DPRINT("Destination ID= ");DPRINTLN(incomingDestinationID);
      DPRINT("Target ID= ");DPRINTLN(incomingTargetID);
      DPRINT("Command = ");DPRINTLN(incomingCommand); 
      if (incomingDestinationID =="Periscope"){
        DPRINTLN("Accepted");

        if (incomingTargetID == "PL"){
          DPRINT("Sending out plSerial");DPRINTLN(incomingCommand);
          writePlSerial(incomingCommand);
        }
        else{
        inputString = incomingCommand;
        stringComplete = true;   
        }
      } else {DPRINTLN("Ignored");}
  
        
    }
//////////////////////////////////////////////////////////////////////
///***   WiFi Specific Setup  (Only used when OTA is enabled)   ***///
//////////////////////////////////////////////////////////////////////
  
  //Raspberry Pi              192.168.4.100
  //Body Controller ESP       192.168.4.101
  //ESP-NOW Master ESP        192.168.4.110  
  //Dome Controller ESP       192.168.4.111  
  //Periscope Controller ESP  192.168.4.112   ************
  //Remote                    192.168.4.107
  //Developer Laptop          192.168.4.125
  
  // IP Address config of local ESP 
  IPAddress local_IP(192,168,4,112);
  IPAddress subnet(255,255,255,0);
  IPAddress gateway(192,168,4,100);
  
 ////R2 Control Network Details     
  const char* ssid = "R2D2_Control_Network";
  const char* password =  "astromech";
  
  AsyncWebServer server(80);
  
void setup(){


  Serial.begin(115200);
  plSerial.begin(PL_BAUD_RATE,SERIAL_8N1,RXPL,TXPL);
  fuSerial.begin(BAUD_RATE,SERIAL_8N1,RXFU,TXFU);

  Serial.println("\n\n\n----------------------------------------");
  Serial.println("Booting up the Periscope Controller");


   inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
   autoInputString.reserve(100);

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

    memcpy(peerInfo.peer_addr, bodyPeerMACAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add Body ESP-NOW peer");
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
     if(Serial.available()){
     serialEvent();
     }
     if(plSerial.available()){
     plSerialEvent();
     }
     if(fuSerial.available()){
     fuSerialEvent();
     }
if (stringComplete) {autoComplete=false;}
  if (stringComplete || autoComplete) {
    if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
     else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
//     if(inputBuffer[0]=='S'  || inputBuffer[0]=='s') {inputBuffer[0]='E' || inputBuffer[0]=='e';}
     if( inputBuffer[0]=='E' ||       // Command designatore for internal ESP functions
         inputBuffer[0]=='e' ||       // Command designatore for internal ESP functions
         inputBuffer[0]=='S' ||       // Command for Sending ESP-NOW Messages
         inputBuffer[0]=='s'          // Command for Sending ESP-NOW Messages



         ) {
            commandLength = strlen(inputBuffer);                     //  Determines length of command character array.
            DEBUG_PRINT("Command Length is: " );DEBUG_PRINTLN(commandLength);
//            Serial.println(commandLength);
            if(commandLength >= 3) {
                if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {commandState = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
                };
                if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
                  for (int i=1; i<=commandLength; i++)
                   {char inCharRead = inputBuffer[i];
                // add it to the inputString:
                inputStringCommand += inCharRead;
                   }
                   DEBUG_PRINT("\nFull Command Recieved: ");DEBUG_PRINTLN(inputStringCommand);
//                   Serial.println(inputStringCommand);
                   espNowCommandStateString = inputStringCommand.substring(0,2);
                   espNowCommandState = espNowCommandStateString.toInt();
                   
                   Serial.print("ESP NOW Command State: ");
                   Serial.println(espNowCommandState);
                   targetID = inputStringCommand.substring(2,4);
                   Serial.print ("Target ID: ");
                   Serial.println(targetID);
                   commandSubString = inputStringCommand.substring(4,commandLength);
                   Serial.print("Command to Forward: ");
                   Serial.println(commandSubString);
                  
//                  espNowCommandState  = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0')*100+(inputBuffer[3]-'0')*1000+(inputBuffer[4]-'0');
//                  espNowCommandStateString = String(espNowCommandState);
//                  Serial.print("Recieved Command: ");
//                  Serial.println(espNowCommandStateString);
                  };
                
//                if(commandLength >= 4) {
//                  if(inputBuffer[0]=='D' || inputBuffer[0]=='d' ) {typeState = inputBuffer[3]-'0';}
//                }
//                else {
//                     typeState = -1;
//                }
//                if(commandLength >= 5) {
//                  if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {door = (inputBuffer[3]-'0')*10+(inputBuffer[4]-'0');}
//                  }
//                 if(commandLength >= 6) {colorState2 = inputBuffer[5]-'0';}


                

               
//                if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {
//                  D_command[0]   = '\0';                                                            // Flushes Array
//                  DaltToggle = true;
//                  D_command[0] = doorState;
//                  if(door>=0) {
//                               D_command[1] = door;
//                               Dcounts[door] = 0;
//                  }
//                  else {Dcount = 0;}
//                }
                if(inputBuffer[0]=='E' || inputBuffer[0] == 'e') {
                  ESP_command[0]   = '\0';                                                            // Flushes Array
                  ESP_command[0] = commandState;
                }

                 if(inputBuffer[0]=='S' || inputBuffer[0] == 's') {
                  ESPNOW_command[0]   = '\0';                                                            // Flushes Array
                  ESPNOW_command[0] = espNowCommandState;
                  tempESPNOWTargetID = targetID;
//                  ESPNOW_command[2] = commandSubString;
                }


              }
            }

      ///***  Clear States and Reset for next command.  ***///
       stringComplete =false;
       autoComplete = false;
       inputBuffer[0] = '\0';
       int commandState;
       inputStringCommand = "";
       targetID = "";
       espNowCommandState =0;
//       targetID="";
//       Serial.println("command Proccessed");

     }


          if(ESP_command[0]){
          switch (ESP_command[0]){
            case 1: Serial.println("Controller: Periscope Controller");   
                    ESP_command[0]   = '\0'; break;
            case 2: Serial.println("Resetting the ESP in 3 Seconds");
                    delay(3000);
                    ESP.restart();
                    ESP_command[0]   = '\0'; break;
            case 3: connectWiFi();
                    break;
            case 4: ESP.restart();
          }
        }
        if(ESPNOW_command[0]){
          switch(ESPNOW_command[0]){
            case 1: Serial.println("Case 1 Selected");
                  ESPNOW_command[0] = '\0'; break;
            case 2: sendESPNOWCommand(tempESPNOWTargetID,commandSubString); break; 
                  
            
          }
        }
  if(isStartUp) {
        isStartUp = false;

        delay(500);

    }
  }
 }

                                                  
 void serialEvent() {
        //int count = 0;
       while (Serial.available()) {
          // get the new byte:
          char inChar = (char)Serial.read();
          // add it to the inputString:
         inputString += inChar;
          if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
            stringComplete = true;            // set a flag so the main loop can do something about it.
          }

        }
               Serial.println(inputString);
      }
       void plSerialEvent() {
        //int count = 0;
       while (plSerial.available()) {
          // get the new byte:
          char inChar = (char)plSerial.read();
          delay(5);
          // add it to the inputString:
         inputString += inChar;
          if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
            stringComplete = true;            // set a flag so the main loop can do something about it.
           Serial.println(inputString);
           }

        }
      }

      void fuSerialEvent() {
        //int count = 0;
       while (fuSerial.available()) {
          // get the new byte:
          char inChar = (char)fuSerial.read();
          // add it to the inputString:
         inputString += inChar;
          if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
            stringComplete = true;            // set a flag so the main loop can do something about it.
          }

        }
               Serial.println(inputString);
      }


      void writeString(String stringData){
        String completeString = stringData + '\r';
        for (int i=0; i<completeString.length(); i++)
        {
          Serial.write(completeString[i]);
        }
      }
      void writePlSerial(String stringData){
        String completeString = stringData + '\r';
        for (int i=0; i<completeString.length(); i++)
        {
          plSerial.write(completeString[i]);
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
///*****             Test Functions                        *****///
//////////////////////////////////////////////////////////////////////
// 
  void sendESPNOWCommand(String starget,String scomm){
    String sdest;
    if (starget == "DS" || starget == "RS" || starget == "HP"){
        sdest = "Dome";
      } else if (starget == "BC"){
        sdest = "Body";
        }
    commandsToSendtoBroadcast.structDestinationID = sdest;
    DPRINT("sdest: ");DPRINTLN(sdest);
    commandsToSendtoBroadcast.structTargetID = starget;
    commandsToSendtoBroadcast.structSenderID = "Periscope";
    commandsToSendtoBroadcast.structCommand = scomm;
//    DPRINT("Command to Send in Function: ");DPRINTLN(commandsToSendtoBroadcast.structCommand);
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
   if (result == ESP_OK) {
    Serial.println("Sent with success");
    }
    else {
      Serial.println(result);
      Serial.println("Error sending the data");
    }
        ESPNOW_command[0] = '\0';
   }


    void connectWiFi(){
    Serial.println(WiFi.config(local_IP, gateway, subnet) ? "Client IP Configured" : "Failed!");
      WiFi.begin();
      while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi..");
      }
      Serial.println(WiFi.localIP());
      AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
      server.begin();
       ESP_command[0]   = '\0';
      }
