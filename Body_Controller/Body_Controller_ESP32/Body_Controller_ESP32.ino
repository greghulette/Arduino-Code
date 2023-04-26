
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

//Used for Status LEDs
#include <Adafruit_NeoPixel.h>

//Used for pin definition
#include "body_controller_esp32_pin_map.h"

// Debug Functions  - Using my own library for this
#include <DebugR2.h>

//ReelTwo libaries - Using my forked version of this libarary
#include <ReelTwo.h>
#include "core/DelayCall.h"

// Used for Software Serial to allow more useful naming
#include <SoftwareSerial.h>

// Used to parse status from the ATMEGA2560 status messages
#include "ArduinoJson.h"




//////////////////////////////////////////////////////////////////////
///*****       Preferences/Items to change        *****///
//////////////////////////////////////////////////////////////////////
 //ESPNOW Password - This must be the same across all devices
  String ESPNOWPASSWORD = "GregsAstromech";  //Must be less than 20 characters

  ////R2 Control Network Details for OTA only
  const char* ssid = "R2D2_Control_Network";
  const char* password =  "astromech";

  // Keepalive timer to send status messages to the Kill Switch (Droid)
 int keepAliveDuration= 5000;  // 5 seconds

// used to sync timing with the dome controller better, allowing time for the ESP-NOW messages to travel to the dome
// Change this to work with how your droid performs
  int defaultESPNOWSendDuration = 50;

  // Serial Baud Rates
  #define BL_BAUD_RATE 9600
  #define RD_BAUD_RATE 115200 
  #define ST_BAUD_RATE 9600  //Should be lower than 57600
  #define MP_BAUD_RATE 9600  //Should be lower than 57600
  #define AUX1_BAUD_RATE 57600 //Should be lower than 57600
  #define AUX2_BAUD_RATE 57600  //Should be lower than 57600


//////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
  String HOSTNAME = "Body Controller"; 
  
  char inputBuffer[100];
  String inputString;    
       // a string to hold incoming data
  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA
  
  int commandLength;
  
  String serialStringCommand;
  String serialPort;
  String serialSubStringCommand;

 uint32_t Local_Command[6]  = {0,0,0,0,0,0};
  int localCommandFunction     = 0;

  uint32_t ESPNOW_command[6]  = {0,0,0,0,0,0};
  int espNowCommandFunction = 0;
  String espNowCommandFunctionString;
  String tempESPNOWTargetID;
  String espNowInputStringCommand;

  int mp3Track;
  String mp3Comm;

  debugClass Debug;




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

  bool bodyLEDControllerStatus = 0;
  


  //////////////////////////////////////////////////////////////////////
  ///*****       Startup and Loop Variables                     *****///
  //////////////////////////////////////////////////////////////////////
  
  boolean startUp = true;
  boolean isStartUp = true;
  
  unsigned long mainLoopTime; // We keep track of the "Main Loop time" in this variable.
  unsigned long MLMillis;
  byte mainLoopDelayVar = 5;

 //Timers for Status updates
  unsigned long keepAliveMillisDuration = 15000;
  unsigned long blkeepAliveAge;
  unsigned long blkeepaliveAgeMillis;
  unsigned long keepAliveMillis;








  //////////////////////////////////////////////////////////////////////
  ///******       Serial Ports Specific Setup                   *****///
  //////////////////////////////////////////////////////////////////////

  
  #define rdSerial Serial1
  #define blSerial Serial2
  SoftwareSerial stSerial;
  SoftwareSerial mpSerial;
  SoftwareSerial aux1Serial;
  SoftwareSerial aux2Serial;
 


  //////////////////////////////////////////////////////////////////////
  ///******      Arduino Mega Reset Pin Specific Setup          *****///
  //////////////////////////////////////////////////////////////////////

  #define RST RESET_PIN_2560


/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

//  ESP-NOW MAC Addresses used in the Droid.  
const uint8_t droidLoRaMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
const uint8_t bodyControllerMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
const uint8_t bodyServosControllerMACAddress[] =  {0x02, 0x00, 0x00, 0x00, 0x00, 0x03};
const int8_t domeControllerMACAddress[]=  {0x02, 0x00, 0x00, 0x00, 0x00, 0x04};
const uint8_t domePlateControllerMACAddress[] =   {0x02, 0x00, 0x00, 0x00, 0x00, 0x05};

//    MAC Address to broadcast to all senders at once
uint8_t broadcastMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//    MAC Address for the Local ESP to use - This prevents having to capture the MAC address of reciever boards.
uint8_t newLocalMACAddress[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t oldLocalMACAddress[] = {0x24, 0x0A, 0xC4, 0xED, 0x30, 0x12};  //used when connecting to WiFi for OTA

// Define variables to store commands to be sent
  String senderID;
  String targetID;
  String command;
  String commandSubString;
  String espnowpassword;
    // variables for storing status and settings from ATMEGA2560. Declared earlier in code but added here for clarity
  // int BL_LDP_Bright;
  // int BL_MAINT_Bright;
  // int BL_VU_Bright;
  // int BL_CS_Bright;
  // int BL_vuOffsetInt;
  // int BL_vuBaselineInt;
  // int BL_vuOffsetExt;
  // int BL_vuBaselineExt;
  // float BL_BatteryVoltage;
  // int BL_BatteryPercentage;


// Define variables to store incoming commands
  String incomingTargetID;  
  String incomingSenderID;
  String incomingCommand;
  String incomingPassword;
  // int incomingstructBL_LDP_Bright;
  // int incomingstructBL_MAINT_Bright;
  // int incomingstructBL_VU_Bright;
  // int incomingstructBL_CS_Bright;
  // int incomingstructBL_vuOffsetInt;
  // int incomingstructBL_vuBaselineInt;
  // int incomingstructBL_vuOffsetExt;
  // int incomingstructBL_vuBaselineExt;
  // float incomingstructBL_BatteryVoltage;
  // int incomingstructBL_BatteryPercentage;
  // bool incomingstructbodyLEDControllerStatus;
  
  
// Variable to store if sending data was successful
  String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
      char structPassword[20];
      char structSenderID[4];
      char structTargetID[4];
      char structCommand[100];
  } struct_message;


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
      bool structbodyLEDControllerStatus[2];
  } bodyControllerStatus_struct_message;

// Create a struct_message calledcommandsTosend to hold variables that will be sent
  struct_message commandsToSendtoBroadcast;
  bodyControllerStatus_struct_message commandstoSendForStatus;

// Create a struct_message to hold incoming commands from the Body
  struct_message commandsToReceiveFromBroadcast;

  esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (Debug.debugflag == 1){
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
  incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
  incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
  incomingCommand = commandsToReceiveFromBroadcast.structCommand;
  Debug.DBG("Bytes received from ESP-NOW Message: %i\n", len);
  Debug.DBG("Sender ID = %s\n",incomingSenderID);
  Debug.DBG("Target ID= %s\n", incomingTargetID);
  Debug.DBG("Command = %s\n" , incomingCommand); 
  if(incomingTargetID == "BC" || incomingTargetID == "BL" || incomingTargetID == "ST" || incomingTargetID == "MT" || incomingTargetID == "ALL"){
    if (incomingTargetID=="BC"){
      inputString = incomingCommand;
      stringComplete = true; 
      }
    else if (incomingTargetID == "BL"){
      Debug.DBG("Sending command to Body LED Controller");
        writeBlSerial(incomingCommand);
    }
    else  if (incomingTargetID == "ST"){
      Debug.DBG("Sending command to Body LED Controller");
        writeStSerial(incomingCommand);
    }
    else if (incomingTargetID == "MT"){
      Debug.DBG("Sending command to Body LED Controller");
//        writeMpSerial(incomingCommand);
    } else {
      Debug.DBG("Command Ignored");
    }

  }    else {
      Debug.DBG("Command Ignored");
    } 
    
}


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
IPAddress local_IP(192,168,4,109);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,4,100);
uint8_t newMACAddress[] = {0x02, 0x00, 0xC0, 0xA8, 0x04, 0x65};

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

  #define STATUS_LED_COUNT 7
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
  
  Adafruit_NeoPixel StatusLED = Adafruit_NeoPixel(STATUS_LED_COUNT, STAUS_LED_PIN, NEO_GRB + NEO_KHZ800);

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
  Debug.DBG("InputString: %s \n",inputString);
};


void serialAux1Event() {
  while (aux1Serial.available()) {
    // get the new byte:
    char inChar = (char)aux1Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  Debug.ESPNOW("InputString: %s \n",inputString);
};

void serialBlEvent() {
  while (blSerial.available()) {
    // get the new byte:
    StaticJsonDocument<2048> doc;

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
  Debug.DBG("InputString: %s \n",inputString);
};

String mp3TriggerResponseString;

void serialMpEvent() {
  while (mpSerial.available()) {
    char inChar = (char)mpSerial.read();
    mp3TriggerResponseString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
    };
  };
  Debug.DBG("MP3 Trigger Response: %s \n",mp3TriggerResponseString);
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


void writeaux1Serial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    aux1Serial.write(completeString[i]);
  };
  Debug.DBG("String to Send over ESPNOW Serial: %s \n" , completeString.c_str());
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

void setupSendCommandStruct(struct_message* msg, String pass, String sender, String targetID, String cmd)
{
    snprintf(msg->structPassword, sizeof(msg->structPassword), "%s", pass.c_str());
    snprintf(msg->structSenderID, sizeof(msg->structSenderID), "%s", sender.c_str());
    snprintf(msg->structTargetID, sizeof(msg->structTargetID), "%s", targetID.c_str());
    snprintf(msg->structCommand, sizeof(msg->structCommand), "%s", cmd.c_str());
}

void sendESPNOWCommand(String starget, String scomm)
{
  String senderID = "BC";     // change to match location (BC, BS, DP, DC, LD)
  setupSendCommandStruct(&commandsToSendtoBroadcast ,ESPNOWPASSWORD, senderID, starget, scomm);
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
  if (result == ESP_OK) {
    Debug.ESPNOW("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
  }
  else {
    Debug.ESPNOW("Error sending the data\n");
  }
  ESPNOW_command[0] = '\0';
}

void setupSendStatusStruct(bodyControllerStatus_struct_message* msg, String pass, String sender, String targetID)
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
}
int ESPNowSuccessCounter = 1;
int ESPNowFailureCounter = 1;
void sendESPNOWStatus()
{
  String senderID = "BC";     // change to match location (BC, BS, DP, DC, LD)
  String starget = "LD";
  setupSendStatusStruct(&commandstoSendForStatus ,ESPNOWPASSWORD, senderID, starget);
  esp_err_t result = esp_now_send(droidLoRaMACAddress, (uint8_t *) &commandstoSendForStatus, sizeof(commandstoSendForStatus));
  if (result == ESP_OK) {
    // printf("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
    printf("Sent a command/status update to : %x, packet number: %i \n", droidLoRaMACAddress, ESPNowSuccessCounter);
    ESPNowSuccessCounter++;
  }
  else {
   printf("Error sending the data to: %X, packet number: %i\n", droidLoRaMACAddress, ESPNowFailureCounter);
   ESPNowFailureCounter++;
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
void mp3Trigger(String comm, int track){
  mpSerial.print(comm);
  mpSerial.write(track);
}


//////////////////////////////////////////////////////////////////////
///*****    Checks the age of the Status Variables            *****///
//////////////////////////////////////////////////////////////////////

void checkAgeofkeepAlive(){    //checks for the variable's age

    if (bodyLEDControllerStatus==1){
    if (millis()-blkeepAliveAge>=keepAliveMillisDuration){
      bodyLEDControllerStatus=0;
      BL_BatteryPercentage = 0;
      BL_BatteryVoltage = 0.0;
      Debug.ESPNOW("Body LED Controller Offline\n");
    }
  }
}



void printKeepaliveStatus(){

  Debug.DBG("Body LED Controller Status: %d\n", bodyLEDControllerStatus);

  ESP_command[0]   = '\0';



}

void keepAlive(){
  if (millis() - keepAliveMillis >= keepAliveDuration){
    keepAliveMillis = millis();
    sendESPNOWStatus();
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
  rdSerial.begin(RD_BAUD_RATE,SERIAL_8N1,SERIAL_RX_RD_PIN,SERIAL_TX_RD_PIN);
  blSerial.begin(BL_BAUD_RATE,SERIAL_8N1,SERIAL_RX_BL_PIN,SERIAL_TX_BL_PIN);
  stSerial.begin(ST_BAUD_RATE,SWSERIAL_8N1,SERIAL_RX_ST_PIN,SERIAL_TX_ST_PIN,false,95);
  mpSerial.begin(MP_BAUD_RATE,SWSERIAL_8N1,SERIAL_RX_MP_PIN,SERIAL_TX_MP_PIN,false,95);
  aux1Serial.begin(AUX1_BAUD_RATE,SWSERIAL_8N1,SERIAL_RX_AUX1_PIN,SERIAL_TX_AUX1_PIN,false,95);
  aux2Serial.begin(AUX2_BAUD_RATE,SWSERIAL_8N1,SERIAL_RX_AUX2_PIN,SERIAL_TX_AUX2_PIN,false,95);
  
  Serial.println("\n\n\n----------------------------------------");
  Serial.println("Booting up the Body ESP Controller");
  
  //Configure the Reset Pins for the arduinoReset() function
  pinMode(RST, OUTPUT);
  digitalWrite(RST,HIGH);

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
  esp_wifi_set_mac(WIFI_IF_STA, &bodyControllerMACAddress[0]);
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
  peerInfo.encrypt = true;
  //  peerInfo.ifidx=WIFI_IF_AP;

  // Add peers  
  memcpy(peerInfo.peer_addr, broadcastMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Broadcast ESP-NOW peer");
    return;
  }  
    memcpy(peerInfo.peer_addr, droidLoRaMACAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Kill Switch Droid ESP-NOW peer");
    return;
  }  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

 
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
      keepAlive();
    if(Serial.available()){serialEvent();}
    if(blSerial.available()){serialBlEvent();}
    if(aux1Serial.available()){serialAux1Event();}
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
          inputBuffer[0]=='N'     ||        // Command for receiving status/info from other boards
          inputBuffer[0]=='n'               // Command for receiving status/info from other boards
        ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.
          Debug.DBG("Command: %s with a length of %d \n", inputBuffer, commandLength);

          if(commandLength >= 3) {
            if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {
              espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              }; 
  
            if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
              // serialPort =  (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              for (int i=1; i<commandLength;i++ ){
                char inCharRead = inputBuffer[i];
                serialStringCommand += inCharRead;  // add it to the inputString:
              }
              Debug.DBG("Full Serial Command Captured: %s\n", serialStringCommand.c_str());
              serialPort = serialStringCommand.substring(0,2);
              serialSubStringCommand = serialStringCommand.substring(2,commandLength);
              Debug.DBG("Serial Command: %s to Serial Port: %s\n", serialSubStringCommand.c_str(), serialPort);
              if (serialPort == "BL"){
                writeBlSerial(serialSubStringCommand);
                Debug.DBG("Sending out BL Serial\n");
              } else if (serialPort == "EN"){
                writeaux1Serial(serialSubStringCommand);
                Debug.DBG("Sending out EN Serial\n");
              } else if (serialPort == "ST"){
                writeStSerial(serialSubStringCommand);
                Debug.DBG("Sending out ST Serial\n");
              }else if (serialPort == "MP"){
                mp3Comm = serialStringCommand.substring(2,3);
                mp3Track = (inputBuffer[4]-'0')*100+(inputBuffer[5]-'0')*10+(inputBuffer[6]-'0');
                Debug.DBG("Command: %s, Track: %i\n",mp3Comm, mp3Track);
                mp3Trigger(mp3Comm,mp3Track);
                Debug.DBG("Sending out MP Serial\n ");
              } else { Debug.DBG("No valid Serial identified\n");}
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
        case 3: connectWiFi();                                                                  break;        
        case 4: printKeepaliveStatus();                                                         break;  //reserved for future use
        case 5: break;  //reserved for future use
        case 6: break;  //reserved for future use
        case 7: break;  //reserved for future use
        case 8: break;  //reserved for future use
        case 9:  break;  //reserved for future use
        case 10: Debug.toggle_Debug();                                                                 break;
        case 11: Debug.toggle_Debug1();                                                                break;
        // case 12: toggleDebug2();                                                                break;

      }
    }


    
    if(isStartUp) {
      isStartUp = false;
      delay(500);
    }
  }
}  // end of main loop
