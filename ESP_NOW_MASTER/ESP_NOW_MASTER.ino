//#define USE_DEBUG
//#define USE_SERVO_DEBUG

#define DEBUG




#include <WiFi.h>
//#include "ESPAsyncWebServer.h"
//#include <WiFiClient.h>

//#include <WiF/iAP.h>
#include "esp_wifi.h"
//#include <Wire.h>
#include <esp_now.h>

#include <SoftwareSerial.h>

//reeltwo libaries
//#include "ReelTwo.h"
//#include "core/DelayCall.h"
//#include "ServoDispatchPCA9685.h"
//#include "ServoSequencer.h"
//#include "core/Animation.h"

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
    
    char inputBuffer[25];
    String inputString;         // a string to hold incoming data
    String inputStringCommand;
    volatile boolean stringComplete  = false;      // whether the serial string is complete
    String autoInputString;         // a string to hold incoming data
    volatile boolean autoComplete    = false;    // whether an Auto command is setA
    int displayState;
    int typeState;
    int commandLength;
    int paramVar = 9;

    uint32_t ESP_command[6]  = {0,0,0,0,0,0};
    int commandState = 0;

    uint32_t ESPNOW_command[6]  = {0,0,0,0,0,0};
    int espNowCommandState = 0;
    String espNowCommandStateString;
    String tempESPNOWTargetID;
 
#ifdef DEBUG
  #define DEBUG_PRINT(x)     Serial.print (x)
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) 
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

  #define RXBC 15
  #define TXBC 16 
  #define RXD2 25
  #define TXD2 26 
SoftwareSerial bcSerial;
#define BAUD_RATE 9600

  //////////////////////////////////////////////////////////////////////
  ///******      Arduino Mega Reset Pin Specific Setup          *****///
  //////////////////////////////////////////////////////////////////////

  #define RST 4

  //////////////////////////////////////////////////////////////////////
  ///******            ESP- Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
/////*****              ESP NOW Set Up                       *****///
///////////////////////////////////////////////////////////////////////////


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
      Serial.print("Bytes received from Dome: ");
      Serial.println(len);
      incomingDestinationID = commandsToReceiveFromBroadcast.structDestinationID;
      incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
      incomingCommand = commandsToReceiveFromBroadcast.structCommand;
      Serial.print("Sender ID = ");
      Serial.println(incomingSenderID);
      Serial.print("Destination ID= ");
      Serial.println(incomingDestinationID);
      Serial.print("Target ID= ");
      Serial.println(incomingTargetID);
      Serial.print("Command = ");
      Serial.println(incomingCommand); 
      if (incomingDestinationID =="Body"){
        Serial.println("Accepted");
        inputString = incomingCommand;
        stringComplete = true;   
      } else {Serial.println("Ignored");}
  
        
    }


void setup(){
  //Initialize the Serial Ports
  Serial.begin(9600);
//  .begin(9600, SERIAL_8N1, RXD1, TXD1);
  bcSerial.begin(BAUD_RATE, SWSERIAL_8N1, RXBC, TXBC, false, 95);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(50);
//  Serial.println(" ");
//  Serial.println(" ");
//  Serial.println(" ");
  Serial.print("\n\n\n----------------------------------------\n");
  Serial.print("Booting up the Master ESP-NOW Controller\n");
//  Serial.print("\n\n----------------------------------------\n");

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
  
//      AnimatedEvent::process();

  if(startUp) {
//      closeAllDoors();
      startUp = false;
      Serial.print("Startup complete\nStarting main loop\n\n\n");
   }
   if(Serial.available()){
   serialEvent();
   }
   if(bcSerial.available()){
   bcSerialEvent();
   }
   if(Serial2.available()){
   serialTwoEvent();
   }
  if (stringComplete) {autoComplete=false;}
  if (stringComplete || autoComplete) {
    if(stringComplete) {inputString.toCharArray(inputBuffer, 25);inputString="";}
     else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 25);autoInputString="";}
//     if(inputBuffer[0]=='S'  || inputBuffer[0]=='s') {inputBuffer[0]='E' || inputBuffer[0]=='e';}
     if( inputBuffer[0]=='E' ||        // Door Designator
         inputBuffer[0]=='e' ||        // Door Designator
         inputBuffer[0]=='S' ||
        inputBuffer[0]=='s'



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
            case 1: Serial.println("Controller: Master ESP-NOW Controller");   
                    ESP_command[0]   = '\0'; break;
            case 2: Serial.println("Resetting the ESP in 3 Seconds");
                    delay(3000);
                    ESP.restart();
                    ESP_command[0]   = '\0'; break;
//            case 3: #define DEBUG; break;
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
       void bcSerialEvent() {
        //int count = 0;
       while (bcSerial.available()) {
          // get the new byte:
          char inChar = (char)bcSerial.read();
          delay(5);
          // add it to the inputString:
         inputString += inChar;
          if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
            stringComplete = true;            // set a flag so the main loop can do something about it.
           Serial.println(inputString);
           }

        }
      }

      void serialTwoEvent() {
        //int count = 0;
       while (Serial2.available()) {
          // get the new byte:
          char inChar = (char)Serial2.read();
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
      void writebcSerial(String stringData){
        String completeString = stringData + '\r';
        for (int i=0; i<completeString.length(); i++)
        {
          bcSerial.write(completeString[i]);
        }
      }
      
      void writeString2(String stringData){
        String completeString = stringData + '\r';
        for (int i=0; i<completeString.length(); i++)
        {
          Serial2.write(completeString[i]);
        }
      }


//////////////////////////////////////////////////////////////////////
///*****             Test Functions                        *****///
//////////////////////////////////////////////////////////////////////
// 
  void sendESPNOWCommand(String starget,String scomm){
//    Serial.println("sendESPNOWCommand Function called");
//    destination = sdest;
//    command = scomm;
String sdest;
if (starget == "DS" || starget == "RS" || starget == "HP"){
    sdest = "Dome";
  }
    commandsToSendtoBroadcast.structDestinationID = sdest;
    Serial.print("sdest: ");
    Serial.println(sdest);
    commandsToSendtoBroadcast.structTargetID = starget;
    commandsToSendtoBroadcast.structSenderID = "Body";
    commandsToSendtoBroadcast.structCommand = scomm;
//    Serial.print("Command to Send in Function: ");
//    Serial.println(commandsToSendtoBroadcast.structCommand);
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
