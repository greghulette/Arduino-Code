
// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>

//Used for WiFi
#include "esp_wifi.h"
#include <esp_now.h>


// Used for Software Serial to allow more useful naming
#include <SoftwareSerial.h>

#include "ArduinoJson.h"

//ReelTwo libaries
//#define USE_DEBUG
//#define USE_SERVO_DEBUG

#include "ReelTwo.h"
#include "core/DelayCall.h"

//Used for Status LEDs
#include <Adafruit_NeoPixel.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                                                                                                       *****///
///*****                            Created by Greg Hulette.                                                   *****///
///*****                                                                                                       *****///
///*****   I started with the code from flthymcnsty from from which I used the basic command structure and     *****///
///*****  serial input method.  This code also relies on the ReelTwo library for all it's servo movements.     *****///
///*****                                                                                                       *****///
///*****                                     So exactly what does this all do.....?                            *****///
///*****                       - Controls the Body servos                                                      *****///
///*****                       - Creates the WiFI network                                                      *****///
///*****                       - Sends Serial commands to the LED Controller and the Stealth Board             *****///
///*****                       - Sends Serial commands to the ESP-NOW Master to relay to other Controllers     *****///
///*****                                                                                                       *****///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
    
  char inputBuffer[100];
  String inputString;         // a string to hold incoming data
  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA

  int commandLength;
  int paramVar = 9;
  
  String serialPort;
  String serialStringCommand;
  String serialSubStringCommand;
  int mp3Track;
  String mp3Comm;
  
  String infofunction;
  String infoCommandString;
  String infoCommandSubString;

  uint32_t ESPNOW_command[6]  = {0,0,0,0,0,0};
  int espNowCommandFunction = 0;
  String espNowCommandFunctionString;
  String tempESPNOWTargetID;


  uint32_t ESP_command[6]  = {0,0,0,0,0,0};
  int espCommandFunction     = 0;

  int debugflag = 1;
  int debugflag1 = 0;  // Used for debugging params recieved from clients
  int debugflag2 = 0;

  String periscopeControllerStatus = "Offline";
  String domeControllerStatus = "Offline";
  String bodyServoControllerStatus  = "Offline";
  String bodyLEDControllerStatus  = "Offline";
  
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
  String BL_Status = "Offline";
  
  String LB;
  String MB;
  String VB;
//  String VB;
  String OI;
  String BI;
  String OE;
  String BE;
  String BV;
  String BP;

  int keepAliveTimeOut = 15000;
  unsigned long keepAliveMillis;
  unsigned long dckeepAliveAge;
  unsigned long dckeepaliveAgeMillis;
  unsigned long pckeepAliveAge;
  unsigned long pckeepaliveAgeMillis;
  unsigned long bskeepAliveAge;
  unsigned long bskeepaliveAgeMillis;
  unsigned long blkeepAliveAge;
  unsigned long blkeepaliveAgeMillis;

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

  #define RXBS 25
  #define TXBS 26 
  #define RXBL 26
  #define TXBL 25
  #define RXST 12
  #define TXST 14
  #define RXMP 17
  #define TXMP 18
  #define RXRD 2    //need to implement and test this
  #define TXRD 34
  
  #define bsSerial Serial1
  #define blSerial Serial2
  SoftwareSerial stSerial;
  SoftwareSerial mpSerial;
  SoftwareSerial rdSerial;

  #define BS_BAUD_RATE 115200
  #define BL_BAUD_RATE 9600
  #define ST_BAUD_RATE 9600
  #define MP_BAUD_RATE 9600
  #define RD_BAUD_RATE 115200



  //////////////////////////////////////////////////////////////////////
  ///******      Arduino Mega Reset Pin Specific Setup          *****///
  //////////////////////////////////////////////////////////////////////

  #define RST 4


/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

//  MAC Addresses used in the Droid
//  Droid LoRa =              {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
//  Body Controller =         {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
//  Body Servos Controller =  {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};
//  Dome Controller =         {0x02, 0x00, 0x00, 0x00, 0x00, 0x04};
//  Periscope Controller =    {0x02, 0x00, 0x00, 0x00, 0x00, 0x05};



//    MAC Address to broadcast to all senders at once
uint8_t broadcastMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//    MAC Address for the Local ESP to use - This prevents having to capture the MAC address of reciever boards.
uint8_t newLocalMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t oldLocalMACAddress[] = {0x24, 0x0A, 0xC4, 0xED, 0x30, 0x12};

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
      char structSenderID[15];
      char structDestinationID[15];
      char structTargetID[5];
      char structCommand[25];
  } struct_message;


  // typedef struct struct_message {
  //     String structSenderID;
  //     String structDestinationID;
  //     String structTargetID;  
  //     String structCommand;
  // } struct_message;

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
  DBG("Bytes received from ESP-NOW Message: %i\n", len);
  DBG("Sender ID = %s\n",incomingSenderID);
  DBG("Destination ID= %s\n" ,incomingDestinationID);
  DBG("Target ID= %s\n", incomingTargetID);
  DBG("Command = %s\n" , incomingCommand); 
  if(incomingTargetID == "BC" || incomingTargetID == "BL" || incomingTargetID == "ST" || incomingTargetID == "MT" || incomingTargetID == "ALL"){
    if (incomingTargetID=="BC"){
      inputString = incomingCommand;
      stringComplete = true; 
      }
    else if (incomingTargetID == "BL"){
      DBG("Sending command to Body LED Controller");
        writeBlSerial(incomingCommand);
    }
    else  if (incomingTargetID == "ST"){
      DBG("Sending command to Body LED Controller");
        writeStSerial(incomingCommand);
    }
    else if (incomingTargetID == "MT"){
      DBG("Sending command to Body LED Controller");
//        writeMpSerial(incomingCommand);
    } else {
      DBG("Command Ignored");
    }

  }    else {
      DBG("Command Ignored");
    } 
}
//   if (incomingDestinationID =="Body" || incomingTargetID == "ALL"){
//     DBG("ESP-NOW Command Accepted\n");
//     DBG("Target ID= %s\n", incomingTargetID);
//     if (incomingTargetID == "RS"){
//         DBG("Sending %s out rsSerial\n", incomingCommand);
//         writeRsSerial(incomingCommand);
//     } else if (incomingTargetID == "HP"){
//         DBG("Sending %s out hpSerial\n", incomingCommand);
//         writeHpSerial(incomingCommand);
//     } else if (incomingTargetID == "DS" || incomingTargetID == "ALL"){
//         DBG("Execute Local Command = %s\n", incomingCommand);
//  if (incomingCommand == "Status"){
//           DBG("Status is good\n");                                                                                                                                       
//           sendESPNOWCommand("BS","DCONLINE");
//         }else if(incomingCommand != "Status"){
//         inputString = incomingCommand;
//         stringComplete = true; 
//         }
//     } else {
//         DBG("Wrong Target ID Sent\n");
//       }
//   }
//     else {DBG("ESP-NOW Message Ignored\n");}
// }

  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////
//LoRa Remote ESP           192.168.4.101   
//LoRa Droid ESP            192.168.4.108    (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Controller ESP       192.168.4.109   ************ (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//ESP-NOW Master ESP        192.168.4.110   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Controller ESP       192.168.4.111   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Periscope Controller ESP  192.168.4.112   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Droid Raspberry Pi        192.168.4.113
//Remote Raspberry Pi       192.168.4.114
//Developer Laptop          192.168.4.125

// IP Address config of local ESP
IPAddress local_IP(192,168,4,109);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,4,100);
uint8_t newMACAddress[] = {0x02, 0x00, 0xC0, 0xA8, 0x04, 0x65};

 ////R2 Control Network Details
const char* ssid = "R2D2_Remote";
const char* password =  "astromech";
int channel =  8;

AsyncWebServer server(80);

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

  #define NUM_CAMERA_PIXELS 7
  #define CAMERA_LENS_DATA_PIN 27
  //#define CAMERA_LENS_CLOCK_PIN 13
  int dim = 75;
  unsigned long CLMillis;
  byte CLSpeed = 50;
  unsigned long startMillis;
  unsigned long currentMillis;
  byte CL_command[6] = {0,0,0,0,0,0};
  
  int colorState1;
  int speedState;
  
  Adafruit_NeoPixel StatusLED = Adafruit_NeoPixel(1, 5, NEO_GRB + NEO_KHZ800);

  boolean countUp=false;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////                                                                                       /////////     
/////////                             Start OF FUNCTIONS                                        /////////
/////////                                                                                       /////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


void colorWipeStatus(uint32_t c, int brightness) {
  for(uint16_t i=0; i<2; i++) {
    StatusLED.setBrightness(brightness);
    StatusLED.setPixelColor(i, c);
    StatusLED.show();
  }
};

void clearCLStatus() {
  for(uint16_t i=0; i<2; i++) {
    StatusLED.setPixelColor(i, off);
    StatusLED.show();
  }
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                             Serial Communication Functions                                    /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

      /////////////////////////////////////////////////////////
      ///*****          Serial Event Function          *****///
      /////////////////////////////////////////////////////////
      /// This routine is called each loop() runs, so using ///
      /// delay inside loop can delay response.  Multiple   ///
      /// bytes of data may be available.                   ///
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
  DBG("InputString: %s \n",inputString);
};


void serialBsEvent() {
  while (bsSerial.available()) {
    // get the new byte:
    char inChar = (char)bsSerial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  DBG_2("InputString: %s \n",inputString);
};


void serialBlEvent() {
  while (blSerial.available()) {
    // get the new byte:
    StaticJsonDocument<2048> doc;

    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(doc, blSerial);

    if (err == DeserializationError::Ok) 
    {
      BL_Status = doc["BL_Status"].as<String>();
      if (BL_Status == "Online"){
        BL_Status="Online";
        blkeepAliveAge = millis();
        DBG_2("Body LED Controler Keepalive Received \n");
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
      DBG_2("Body LED Controller Status = %s\n", BL_Status); //Serial.println(doc["BL_Status"].as<String>());
      DBG_2("LDP Brightness = %i\n", BL_LDP_Bright); //Serial.println(doc["LDPBright"].as<int>());
      DBG_2("Maint Brightness = %i\n", BL_MAINT_Bright); //Serial.println(doc["MaintBright"].as<int>());
      DBG_2("VU Brightness = %i\n", BL_VU_Bright); //Serial.println(doc["VUBright"].as<int>());
      DBG_2("Coin Slots Brightness = %i\n",BL_CS_Bright); //Serial.println(doc["CoinBright"].as<int>());
      DBG_2("Spectrum Int Offset = %i\n",BL_vuOffsetInt); //Serial.println(doc["VUIntOffset"].as<int>());
      DBG_2("Spectrum Int Baseline = %i\n",BL_vuBaselineInt); //Serial.println(doc["VUIntBaseline"].as<int>());
      DBG_2("Spectrum Ext Offset = %i\n",BL_vuOffsetExt); //Serial.println(doc["VUExtOffset"].as<int>());
      DBG_2("Spectrum Ext Baseline = %i\n",BL_vuBaselineExt); //Serial.println(doc["VUExtOffset"].as<int>());
      DBG_2("Battery Voltage = %.2f\n",BL_BatteryVoltage); //Serial.println(doc["BatteryVoltage"].as<float>());
      DBG_2("Battery Percentage = %i\n",BL_BatteryPercentage);//Serial.println(doc["BatteryPercent"].as<int>());
    } 
    else 
    {
      // Print error to the "debug" serial port
      Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());
    
      // Flush all bytes in the "link" serial port buffer
//      while (blSerial.available() > 0)
//        blSerial.read();
    }
  };
};


void serialStEvent() {
  while (stSerial.available()) {
    // get the new byte:
    char inChar = (char)stSerial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  DBG("InputString: %s \n",inputString);
};

String mp3TriggerResponseString;

void serialMpEvent() {
  while (mpSerial.available()) {
    char inChar = (char)mpSerial.read();
    mp3TriggerResponseString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
    };
  };
  DBG("MP3 Trigger Response: %s \n",mp3TriggerResponseString);
  mp3TriggerResponseString = "";
};

  /////////////////////////////////////////////////////////
  ///*****          Serial Write Function          *****///
  /////////////////////////////////////////////////////////
  /// These functions recieve a string and transmits    ///
  /// one character at a time and adds a '/r' to the    ///
  /// end of the string.                                ///
  /////////////////////////////////////////////////////////

void writeSerialString(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
      Serial.write(completeString[i]);
  };
};


void writeBlSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    blSerial.write(completeString[i]);
  };
};


void writeBsSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    bsSerial.write(completeString[i]);
  };
  DBG("String to Send over ESPNOW Serial: %s \n" , completeString.c_str());
};


void writeStSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    stSerial.write(completeString[i]);
  };
};

/////////////////////////////////////////////////////////////////////
///*****             ESP-NOW Functions                        *****///
//////////////////////////////////////////////////////////////////////

void setupSendStruct(struct_message* msg, String sender, String destID, String targetID, String cmd)
{
    snprintf(msg->structSenderID, sizeof(msg->structSenderID), "%s", sender.c_str());
    snprintf(msg->structDestinationID, sizeof(msg->structDestinationID), "%s", destID.c_str());
    snprintf(msg->structTargetID, sizeof(msg->structTargetID), "%s", targetID.c_str());
    snprintf(msg->structCommand, sizeof(msg->structCommand), "%s", cmd.c_str());
}

void sendESPNOWCommand(String starget, String scomm){
  String sdest;
  String senderID = "Body";     // change to match location (Dome, Body, Periscope)
  if (starget == "DS" || starget == "RS" || starget == "HP"){
    sdest = "Dome";
  } else if (starget == "PC" || starget == "PL"){
    sdest = "Periscope";
  }else if (starget == "EN" || starget == "DS" || starget == "BL" || starget == "ST"|| starget == "BS"){
    sdest = "Body";
  }

  setupSendStruct(&commandsToSendtoBroadcast ,senderID, sdest, starget, scomm);
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
  if (result == ESP_OK) {
    DBG("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
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

void DBG_2(const char *format, ...) {
        if (!debugflag2)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}


void toggleDebug(){
  debugflag = !debugflag;
  if (debugflag == 1){
    Serial.println("Debugging Enabled \n"); 
    }
  else{
    Serial.println("Debugging Disabled");
  }
    ESP_command[0]   = '\0';
}


void toggleDebug1(){
  debugflag1 = !debugflag1;
  if (debugflag1 == 1){
    Serial.println("Parameter Debugging Enabled \n");
    }
  else{
    Serial.println("Parameter Debugging Disabled\n");
  }
    ESP_command[0]   = '\0';
}
void toggleDebug2(){
  debugflag2 = !debugflag2;
  if (debugflag2 == 1){
    Serial.println("Debug 2 Debugging Enabled \n");
    }
  else{
    Serial.println("Debug 2 Debugging Disabled\n");
  }
    ESP_command[0]   = '\0';
}


//////////////////////////////////////////////////////////////////////
///*****    Resets Arduino Mega due to bug in my PCB          *****///
//////////////////////////////////////////////////////////////////////

void resetArduino(int delayperiod){
  DBG("Body LED Controller Reset Function\n");
  digitalWrite(RST,LOW);
  delay(delayperiod);
  digitalWrite(RST,HIGH);
}

//////////////////////////////////////////////////////////////////////
///*****         Function for MP3 Trigger                     *****///
//////////////////////////////////////////////////////////////////////
void mp3Trigger(String comm, int track){
  mpSerial.print(comm);
  mpSerial.write(track);
}


//////////////////////////////////////////////////////////////////////
///*****    Checks the age of the Status Variables            *****///
//////////////////////////////////////////////////////////////////////

void checkAgeofkeepAlive(){    //checks for the variable's age
  if (domeControllerStatus=="Online"){
    if (millis()-dckeepAliveAge>=keepAliveTimeOut){
      domeControllerStatus="Offline";
      DBG_2("Dome Controller Offline\n");
    }
  }
  if (periscopeControllerStatus=="Online"){
    if (millis()-pckeepAliveAge>=keepAliveTimeOut){
      periscopeControllerStatus="Offline";
      DBG_2("Periscope Controller Offline\n");
    }
  }
  if (bodyServoControllerStatus=="Online"){
    if (millis()-bskeepAliveAge>=keepAliveTimeOut){
      bodyServoControllerStatus="Offline";
      DBG_2("Body Servo Controller Offline\n");
    }
  }
    if (BL_Status=="Online"){
    if (millis()-blkeepAliveAge>=keepAliveTimeOut){
      BL_Status="Offline";
      BL_BatteryPercentage = 0;
      BL_BatteryVoltage = 0.0;
      DBG_2("Body LED Controller Offline\n");
    }
  }
}

void checkAgeofBLKeepAlive(){
  if (BL_LDP_Bright > 0){
    }
}
//void keepAliveBC(){
//  if (millis() - keepAliveMillis >= keepAliveDuration){
//    keepAliveMillis = millis();
//    sendESPNOWCommand("RL","BCKA");
//  } 
//}
//String LBF;
//void keepAliveBL(){
//  if (millis() - blkeepAliveMillis >= blkeepAliveDuration){
//    BLkeepAliveMillis = millis();
//    LB = String(BL_LDP_Bright);
//    LBF = "BL" + LB;
//    sendESPNOWCommand("RL", LBF);
//  } 
//}
void printKeepaliveStatus(){

  DBG("Dome Controller Status: %s\n", domeControllerStatus);
  DBG("Body Servo Controller Status: %s\n", bodyServoControllerStatus);
  DBG("Periscope Controller Status: %s\n", periscopeControllerStatus);
  DBG("Body LED Controller Status: %s\n", BL_Status);

  ESP_command[0]   = '\0';

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
  bsSerial.begin(BS_BAUD_RATE,SERIAL_8N1,RXBS,TXBS);
  blSerial.begin(BL_BAUD_RATE,SERIAL_8N1,RXBL,TXBL);
  stSerial.begin(ST_BAUD_RATE,SWSERIAL_8N1,RXST,TXST,false,95);
  mpSerial.begin(MP_BAUD_RATE,SWSERIAL_8N1,RXMP,TXMP,false,95);
  rdSerial.begin(RD_BAUD_RATE,SWSERIAL_8N1,RXRD,TXRD,false,95);
  Serial.println("\n\n\n----------------------------------------");
  Serial.println("Booting up the Body ESP Controller");
  
  //Configure the Reset Pins for the arduinoReset() function
  pinMode(4, OUTPUT);
  digitalWrite(4,HIGH);

  //Initialize I2C for the Servo Expander Board
//  Wire.begin();
  
  //Initialize the ReelTwo Library
//  SetupEvent::ready();

  StatusLED.begin();
  StatusLED.show();
  colorWipeStatus(red,255);
  Serial.println("LED Setup Complete");


  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

  //Initialize the Soft Access Point
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
  memcpy(peerInfo.peer_addr, broadcastMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  }  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

 //Setup the webpage and accept the GET requests, and parses the variables 
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      
     int paramsNr = request->params();               // Gets the number of parameters sent
// //    DBG("Parameter %i \n",paramsNr);                       // Variable for selecting which Serial port to send out
     for(int i=0;i<paramsNr;i++){                     //Loops through all the paramaters
       AsyncWebParameter* p = request->getParam(i);

// ////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////                                                                //////////////////////////        
// //////////  These If statements choose where to send the commands         //////////////////////////
// //////////  This way we can control multiple serial ports from one ESP32. //////////////////////////
// //////////                                                                //////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////////
        
     if ((p->name())== "param0" & (p->value()) == "Serial0"){
         DBG_1("Serial0 Chosen with If Statement\n");
         paramVar = 0;
         };
     if ((p->name())== "param0" & (p->value()) == "bsSerial"){
         DBG_1("Serial 1 Chosen with If Statement\n");
         paramVar = 1;
         };
     if ((p->name())== "param0" & (p->value()) == "blSerial"){
       DBG_1("Serial 2 Chosen with If Statement\n");
           paramVar = 2;
     };
         if ((p->name())== "param0" & (p->value()) == "stSerial"){
           DBG_1("Serial 2 Chosen with If Statement\n");
           paramVar = 3;
     };
     if ((p->name())== "param0" & (p->value()) == "ESP"){
           DBG_1("ESP(Self) Chosen with If Statement\n");
           paramVar = 4;
     };
     if ((p->name())== "param0" & (p->value()) == "ArduinoReset"){
         DBG_1("Reset Only Arduino Chosen with If Statement\n");
           resetArduino(500);
         };
     if ((p->name())== "param0" & (p->value()) == "ESPReset"){
         DBG_1("Reset ESP and Arduino Chosen with If Statement\n");
         ESP.restart();
         };
  
         DBG_1("Param name: %s\n", (p->name()));
         DBG_1("Param value: %s\n", (p->value()).c_str());

         if (paramVar == 0){
           DBG_1("Writing to Serial 0\n");      
           writeSerialString(p->value());
         };
         if (paramVar == 1){
           DBG_1("Writing to bsSerial\n"); 
// //          delay(100);     
         if ((p->name())== "param0" & (p->value()) == "bsSerial"){
             DBG_1("Skipping param 0 in the EspNowSerial Write\n");
           } 
           else {
             writeBsSerial(p->value());
           };
         } ;      
           if (paramVar == 2){
           DBG_1("Writing to blSerial\n");   

          if ((p->name())== "param0" & (p->value()) == "blSerial"){
             DBG_1("Skipping param 0 in the EspNowSerial Write\n");
           } 
           else {
             writeBlSerial(p->value());
           };
 //          writeBlSerial(p->value());
        
        
        
         };
         if (paramVar == 3){
           DBG_1("Writing to stSerial\n");      
           writeStSerial(p->value());
         };
         if (paramVar == 4){
           DBG_1("Executing on self\n");      
           inputString = (p->value());
           stringComplete = true;  
         };

         DBG_1("------\n");
         delay(50);
     }

     request->send(200, "text/plain", "Message Received on Body Controller");
   });

//   server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
//     request->send_P(200, "text/html", html, processor);
//   });
//   

 server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
       AsyncResponseStream *response = request->beginResponseStream("application/json");
       DynamicJsonDocument json(2048);
       json["BL_Status"] = "Online";
       json["BodyController"] = "Online";
       json["BodyServo"] = bodyServoControllerStatus;
       json["Dome"] = domeControllerStatus;
       json["Periscope"] = periscopeControllerStatus;
       json["LDPBright"] = BL_LDP_Bright;
       json["MaintBright"] = BL_MAINT_Bright;
       json["VUBright"] = BL_VU_Bright;
       json["CoinBright"] = BL_CS_Bright;
       json["VUIntOffset"] = BL_vuOffsetInt;
       json["VUIntBaseline"] = BL_vuBaselineInt;
       json["VUExtOffset"] = BL_vuOffsetExt;
       json["VUExtBaseline"] = BL_vuBaselineExt;
       json["BatteryVoltage"] = BL_BatteryVoltage;
       json["BatteryPercent"] = BL_BatteryPercentage;

       serializeJson(json, *response);
       request->send(response);
   });

  
//   //Enable Access-Control-Allow-Origin to mitigate errors from website polling
   DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
   AsyncElegantOTA.begin(&server);    // Start ElegantOTA

  //Initialize the AsycWebServer
   server.begin();

  //Reset Arudino Mega
  resetArduino(500);


}  //end of Setup


void loop(){
  if (millis() - MLMillis >= mainLoopDelayVar){
    MLMillis = millis();
    AnimatedEvent::process();
    if(startUp) {
      colorWipeStatus(blue,25);

      startUp = false;
      Serial.println("Startup");
      // Play Startup Sound
      mp3Trigger("v",16);
      mp3Trigger("t",1);
      mp3Trigger("v",0);

    }
    checkAgeofkeepAlive();
    
    if(Serial.available()){serialEvent();}
    if(blSerial.available()){serialBlEvent();}
    if(bsSerial.available()){serialBsEvent();}
    if(stSerial.available()){serialStEvent();}
    if(mpSerial.available()){serialMpEvent();}

    
    if (stringComplete) {autoComplete=false;}
    if (stringComplete || autoComplete) {
      if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
      else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
      if( inputBuffer[0]=='E'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='e'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='S'     ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='s'     ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='I'     ||        // Command for receiving status/info from other boards
          inputBuffer[0]=='i'               // Command for receiving status/info from other boards
        ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.
          DBG("Command: %s with a length of %d \n", inputBuffer, commandLength);

          if(commandLength >= 3) {
            if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {
              espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              }; 

            if(inputBuffer[0]=='I' || inputBuffer[0]=='i'){
              for (int i=1; i<commandLength-1;i++ ){
                char inCharRead = inputBuffer[i];
                infoCommandString += inCharRead;  // add it to the inputString:
              }
              DBG_2("I Command Proccessing: %s \n", infoCommandString.c_str());
              if(infoCommandString == "PC"){
                periscopeControllerStatus="Online";
                pckeepAliveAge = millis();
                DBG_2("Periscope Controller Keepalive Received\n");
              }
              if(infoCommandString == "BS"){
                bodyServoControllerStatus="Online";
                bskeepAliveAge = millis();
                DBG_2("Body Servo Controller Keepalive Received\n");
              }
              if(infoCommandString == "DC"){
                domeControllerStatus="Online";
                dckeepAliveAge = millis();
                DBG_2("Dome Controller Keepalive Received\n");
              }
              infoCommandString="";
            }     
            if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
              // serialPort =  (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              for (int i=1; i<commandLength;i++ ){
                char inCharRead = inputBuffer[i];
                serialStringCommand += inCharRead;  // add it to the inputString:
              }
              DBG("Full Serial Command Captured: %s\n", serialStringCommand.c_str());
              serialPort = serialStringCommand.substring(0,2);
              serialSubStringCommand = serialStringCommand.substring(2,commandLength);
              DBG("Serial Command: %s to Serial Port: %s\n", serialSubStringCommand.c_str(), serialPort);
              if (serialPort == "BL"){
                writeBlSerial(serialSubStringCommand);
                DBG("Sending out BL Serial\n");
              } else if (serialPort == "EN"){
                writeBsSerial(serialSubStringCommand);
                DBG("Sending out EN Serial\n");
              } else if (serialPort == "ST"){
                writeStSerial(serialSubStringCommand);
                DBG("Sending out ST Serial\n");
              }else if (serialPort == "MP"){
                mp3Comm = serialStringCommand.substring(2,3);
                mp3Track = (inputBuffer[4]-'0')*100+(inputBuffer[5]-'0')*10+(inputBuffer[6]-'0');
                DBG("Command: %s, Track: %i\n",mp3Comm, mp3Track);
                mp3Trigger(mp3Comm,mp3Track);
                DBG("Sending out MP Serial\n ");
              } else { DBG("No valid Serial identified\n");}
              serialStringCommand = "";
              serialPort = "";
              serialSubStringCommand = "";
              int mp3Track;
            } 
            


            if(inputBuffer[0]=='E' || inputBuffer[0] == 'e') {
              ESP_command[0]   = '\0';                                                            // Flushes Array
              ESP_command[0] = espCommandFunction;
            }
          }
        }

      ///***  Clear States and Reset for next command.  ***///
        stringComplete =false;
        autoComplete = false;
        inputBuffer[0] = '\0';

        // reset Local ESP Command Variables
        int espCommandFunction;


    }

    if(ESP_command[0]){
      switch (ESP_command[0]){
        case 1: Serial.println("Body ESP Controller");   
                ESP_command[0]   = '\0';                                                        break;
        case 2: Serial.println("Resetting the ESP in 3 Seconds");
                DelayCall::schedule([] {ESP.restart();}, 3000);
                ESP_command[0]   = '\0';                                                        break;
        case 3: break;  //reserved for commonality. Used for connecting to WiFi and enabling OTA on ESP-NOW Boards 
        case 4: printKeepaliveStatus();break;  //reserved for future use
        case 5: break;  //reserved for future use
        case 6: break;  //reserved for future use
        case 7: break;  //reserved for future use
        case 8: break;  //reserved for future use
        case 9:  break;  //reserved for future use
        case 10: toggleDebug();                                                                 break;
        case 11: toggleDebug1();                                                                break;
        case 12: toggleDebug2();                                                                break;

      }
    }


    
    if(isStartUp) {
      isStartUp = false;
      delay(500);
    }
  }
}  // end of main loop
