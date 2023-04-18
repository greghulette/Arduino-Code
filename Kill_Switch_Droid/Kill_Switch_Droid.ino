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
#include <Arduino.h>

//Used for LoRa
#include <SPI.h>
// #include <LoRa.h>
#include "LoRa.h"
//Used for status LEDs
#include <Adafruit_NeoPixel.h>

//Used for ESP-NOW
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>

//////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
  String HOSTNAME = "LoRa to ESP-NOW Gateway";

  char inputBuffer[100];
  String inputString;         // a string to hold incoming data
  String inputStringCommand;

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
  String ESPNOWPASSWORD = "GregsAstromech";

  uint32_t ESP_command[6]  = {0,0,0,0,0,0};
  int espCommandFunction     = 0;

  int debugflag = 0;
  int debugflag1 = 0; 
  int debugflag2 = 0;
  boolean debugflag_espnow = 0;
  boolean debugflag_lora = 0;




//////////////////////////////////////////////////////////////
///*****        LoRa Variables       *****///
//////////////////////////////////////////////////////////////////////
  String outgoing;
  String LoRaOutgoing;

  boolean oldState = HIGH;

  byte msgCount = 0;            // count of outgoing messages
  byte localAddress = 0xBC;     // address of this device
  byte destination = 0xFF;      // destination to send to
  long lastSendTime = 0;        // last send time
  int interval = 2000;          // interval between sends

  const int csPin = 15;          // LoRa radio chip select
  const int resetPin = 27;       // LoRa radio reset
  const int irqPin = 26;         // change for your board; must be a hardware interrupt pin

  int LoRaRSSI;


//////////////////////////////////////////////////////////////
///*****        Variables for button      *****///
//////////////////////////////////////////////////////////////////////
  unsigned long buttonCurrentMillis;
  unsigned long buttonPreviousMillis;
  unsigned long buttonDelayInterval;
  boolean RELAY_STATUS = HIGH;
  #define RELAY_STATE_OUTPUT 25  
  #define ON_BUTTON_PIN   8

//////////////////////////////////////////////////////////////////////
  ///*****       Startup and Loop Variables                     *****///
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
///*****            Status and Camera Lens Variables and settings       *****///
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

  #define STATUS_LED_COUNT 1
  #define ESP_STATUS_DATA_PIN 19
  #define RELAY_STATUS_DATA_PIN 5
  #define LORA_STATUS_DATA_PIN 20
  //#define CAMERA_LENS_CLOCK_PIN 13
  int dim = 75;
  unsigned long CLMillis;
  byte CLSpeed = 50;
  unsigned long startMillis;
  unsigned long currentMillis;
  byte LED_command[6] = {0,0,0,0,0,0};
  
  int colorState1;
  int speedState;
  
  Adafruit_NeoPixel ESP_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, ESP_STATUS_DATA_PIN, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel RELAY_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, RELAY_STATUS_DATA_PIN, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel LORA_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, LORA_STATUS_DATA_PIN, NEO_GRB + NEO_KHZ800);
  boolean startUp = true;
  boolean isStartUp = true;
  
  unsigned long mainLoopTime; // We keep track of the "Main Loop time" in this variable.
  unsigned long MLMillis;
  unsigned long LMillis;
  int LoraDelay = 5000;
  byte mainLoopDelayVar = 5;

  unsigned long statusCurrentMillis;
  unsigned long statusPreviousMillis;
  unsigned long statusDelayInterval = 5000;

  boolean droidGatewayStatus = 1;
  boolean bodyControllerStatus = 0;
  boolean bodyLEDControllerStatus = 0;  
  boolean bodyServoStatus = 0;
  boolean domePlateControllerStatus = 0;
  boolean domeControllerStatus = 0;
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
  unsigned long bckeepAliveAge;
  unsigned long bckeepaliveAgeMillis;
  unsigned long blkeepAliveAge;
  unsigned long blkeepaliveAgeMillis;

  //////////////////////////////////////////////////////////////////////
  ///******       Serial Ports Specific Setup                   *****///
  //////////////////////////////////////////////////////////////////////

  #define TXLD 2
  #define RXLD 4 

  #define ldSerial Serial1

  #define LD_BAUD_RATE 115200

/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

//    MAC Address to broadcast to all senders at once
uint8_t broadcastMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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

// Create a struct_message calledcommandsTosend to hold variables that will be sent
  struct_message commandsToSendtoBroadcast;

// Create a struct_message calledcommandsTosend to hold variables that will be sent
  struct_message commandsToReceiveFromBroadcast;

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
  incomingPassword = commandsToReceiveFromBroadcast.structPassword;
  if (incomingPassword != ESPNOWPASSWORD){
  DBG("Wrong ESP-NOW Password was sent.  Message Ignored\n");  
  } else {
  incomingTargetID = commandsToReceiveFromBroadcast.structTargetID;
  incomingSenderID = commandsToReceiveFromBroadcast.structSenderID;
  incomingCommand = commandsToReceiveFromBroadcast.structCommand;
    DBG("Bytes received from ESP-NOW Message: %i\n", len);
    DBG("Sender ID = %s\n",incomingSenderID);
    DBG("Target ID= %s\n", incomingTargetID);
    DBG("Command = %s\n" , incomingCommand); 
  
    if (incomingTargetID == "LD" ||incomingTargetID == "LR" || incomingTargetID == "HB"){
    if(incomingTargetID == "LD"){
      if (incomingCommand == "DC-ONLINE"){
        domeControllerStatus = 1;
        dckeepAliveAge =millis();
      }
      if (incomingCommand == "BS-ONLINE"){
        bodyServoStatus = 1;
      }   
      if (incomingCommand == "DP-ONLINE"){
        domePlateControllerStatus = 1;
      }                             
      if (incomingCommand == "BC-ONLINE"){
        bodyControllerStatus = 1;
      }
      if (incomingCommand == "BL-ONLINE"){
        bodyLEDControllerStatus = 1;
      }      
      inputString = incomingCommand;
      stringComplete = true; 
    } else if (incomingTargetID == "LR"){
      // add code to implement LoRa Send in main loop so I can implement this
    } else if (incomingTargetID == "HB") {
        DBG("Received heartbeat from: \n");

      if(incomingSenderID == "BS"){
        DBG("Body Servo Controller\n");
        bodyServoStatus = 1;
        bskeepAliveAge = millis();
      }
    }
  } else {
    DBG("ESP-NOW Message Ignored\n");
    }
  }
}
  esp_now_peer_info_t peerInfo;


////////////////////////////////////////////////////////////////////
/*****    Checks the age of the Status Variables            *****///
////////////////////////////////////////////////////////////////////

void checkAgeofkeepAlive(){    //checks for the variable's age
  if (domeControllerStatus== 1){
    if (millis() - dckeepAliveAge>=keepAliveTimeOut){
      domeControllerStatus = 0;
      DBG_2("Dome Controller Offline\n");
    }
  }
  if (domePlateControllerStatus== 1){
    if (millis()-pckeepAliveAge>=keepAliveTimeOut){
      domePlateControllerStatus= 0;
      DBG_2("Dome Plate Controller Offline\n");
    }
  }
  if (bodyServoStatus == 1){
    if (millis()-bskeepAliveAge>=keepAliveTimeOut){
      bodyServoStatus = 0;
      DBG_2("Body Servo Controller Offline\n");
    }
  }
  if (bodyControllerStatus == 1){
    if (millis()-bckeepAliveAge>=keepAliveTimeOut){
      bodyControllerStatus = 0;
      DBG_2("Body Controller Offline\n");
    }
  }
  if (bodyLEDControllerStatus == 1){
    if (millis()-blkeepAliveAge>=keepAliveTimeOut){
      bodyLEDControllerStatus = 0;
      DBG_2("Body LED Controller Offline\n");
    }
  }
}

void printKeepaliveStatus(){

  // DBG("Dome Controller Status: %s\n", domeControllerStatus);
  DBG("Dome Controller Status: %d\n", domeControllerStatus);
  // DBG("Periscope Controller Status: %s\n", periscopeControllerStatus);
  // DBG("Body LED Controller Status: %s\n", bodyLEDControllerStatus);

  ESP_command[0]   = '\0';

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////                                                                                       /////////     
/////////                             Start OF FUNCTIONS                                        /////////
/////////                                                                                       /////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


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
else{
  // DBG("No LED was chosen \n");
  }
  };



/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                              Communication Functions                                    /////
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
  String sdest;
  String senderID = "LD";   // change to match location (Dome, Body, Periscope)
  if (starget == "DS" || starget == "RS" || starget == "HP"){
    sdest = "Dome";
  } else if (starget == "PC" || starget == "PL"){
    sdest = "Periscope";
  }else if (starget == "EN" || starget == "BC" || starget == "BL" || starget == "ST"|| starget == "BS"){
    sdest = "Body";
  } else if (starget =="SR"){
    sdest = "Broadcast";
  }


  setupSendStruct(&commandsToSendtoBroadcast , ESPNOWPASSWORD, senderID, starget, scomm);
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &commandsToSendtoBroadcast, sizeof(commandsToSendtoBroadcast));
  if (result == ESP_OK) {
    DBG("Sent the command: %s to ESP-NOW Neighbors\n", scomm.c_str());
  }
  else {
    DBG("Error sending the data\n");
  }
  ESPNOW_command[0] = '\0';
}


//   //////////////////////////////////////////////////////////////////////
//   ///*****             LoRa Functions                           *****///
//   //////////////////////////////////////////////////////////////////////

// void sendMessage(String outgoing) {
//   LoRa.beginPacket();                   // start packet
//   LoRa.write(destination);              // add destination address
//   LoRa.write(localAddress);             // add sender address
//   LoRa.write(msgCount);                 // add message ID
//   LoRa.write(outgoing.length());        // add payload length
//   LoRa.print(outgoing);                 // add payload
//   LoRa.endPacket();                     // finish packet and send it
//   msgCount++;                           // increment message ID
//   Serial.println("Message Sent");
// }

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
  LoRa.write(bodyServoStatus);
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
  Serial.println("Status Sent");
  ESP_command[0]   = '\0';


}


void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  
  if(LoRa.packetRssi() > -50 && LoRa.packetRssi() < 10){
    colorWipeStatus("LS", green, 10);
  }else if (LoRa.packetRssi() > -100 && LoRa.packetRssi()  <= -50){
     colorWipeStatus("LS", yellow, 10);
  } else{ colorWipeStatus("LS", red, 10);}

  inputString = incoming;
  stringComplete = true; 
      // sendMessage("Message Revieved");
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

//
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

  void DBG_ESPNOW(const char *format, ...) {
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

void MainRelayOn(){
         RELAY_STATUS = HIGH;
       digitalWrite(RELAY_STATE_OUTPUT, RELAY_STATUS);
        Serial.print("Mode is: ");Serial.println(RELAY_STATUS);
        ESP_command[0]   = '\0';


}

void MainRelayOff(){
         RELAY_STATUS = LOW;
       digitalWrite(RELAY_STATE_OUTPUT, RELAY_STATUS);
        Serial.print("Mode is: ");Serial.println(RELAY_STATUS);
        ESP_command[0]   = '\0';

}





void setup() {
  Serial.begin(115200);
  while (!Serial);
  ldSerial.begin(LD_BAUD_RATE,SERIAL_8N1,RXLD,TXLD);

  Serial.println("\n\n----------------------------------------");
  Serial.print("Booting up the ");Serial.println(HOSTNAME);
  Serial.println("----------------------------------------");
  
  //Button for relay setup
  pinMode(ON_BUTTON_PIN, INPUT);
  // // pinMode(RELAY_STATE_OUTPUT, OUTPUT);
  // RELAY_STATUS = LOW;
  // // digitalWrite(RELAY_STATE_OUTPUT, RELAY_STATUS);
  // delay(200);
  // RELAY_STATUS = HIGH;
  // digitalWrite(RELAY_STATE_OUTPUT, RELAY_STATUS);

  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

  
    

  // Serial.println("LoRa init succeeded.");

//initialize WiFi for ESP-NOW
  // WiFi.mode(WIFI_STA);
  // //Initialize ESP-NOW
  // if (esp_now_init() != ESP_OK) {
  //   Serial.println("Error initializing ESP-NOW");
  // return;
  // }

  // // Once ESPNow is successfully Init, we will register for Send CB to
  // // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);
  
  // // Register peer
  // peerInfo.channel = 0;  
  // peerInfo.encrypt = false;
  // //  peerInfo.ifidx=WIFI_IF_AP;

  // // Add peers  
  // memcpy(peerInfo.peer_addr, broadcastMACAddress, 6);
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add Broadcast ESP-NOW peer");
  //   return;
  // }  
  // // Register for a callback function that will be called when data is received
  // esp_now_register_recv_cb(OnDataRecv);


ESP_LED.begin();
ESP_LED.show();
colorWipeStatus("ES", blue, 10);

RELAY_LED.begin();
RELAY_LED.show();
colorWipeStatus("RS", green, 10);

LORA_LED.begin();
LORA_LED.show();
colorWipeStatus("LS", blue, 10);  
delay(3000);
  //LoRa Setup
SPI.begin(14, 12, 13, 15);
LoRa.setPins(15, 27, 26);

  // Serial.println("LoRa Receiver");
  // LoRa.setPins(5, 14, 2);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6,true)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

 

void loop() {
  checkAgeofkeepAlive();

  boolean newState = digitalRead(ON_BUTTON_PIN);

   if((newState == LOW) && (oldState == HIGH)) {
  //   // Short delay to debounce button.
    delay(20);
  //   // Check if button is still low after debounce.
    buttonCurrentMillis = millis();
    if(buttonCurrentMillis - buttonPreviousMillis >= buttonDelayInterval){
    newState = digitalRead(ON_BUTTON_PIN);
    if(newState == LOW) {      // Yes, still low
       if (RELAY_STATUS == LOW) {
                colorWipeStatus("RS", red, 10);
       }
             if (RELAY_STATUS == HIGH){
                colorWipeStatus("RS", green, 10);
       }
       
       RELAY_STATUS = !RELAY_STATUS;
      //  colorWipeStatus("RS", red, 50);
       digitalWrite(RELAY_STATE_OUTPUT, RELAY_STATUS);
        Serial.print("Mode is: ");Serial.println(RELAY_STATUS);
      }
      // else{colorWipeStatus("RS", red, 50);}
    }
    }
    onReceive(LoRa.parsePacket());

// statusCurrentMillis = millis();
// if(statusCurrentMillis - statusPreviousMillis >= statusDelayInterval){
//   statusPreviousMillis = millis() ;
//   sendESPNOWCommand("RS", "StatusRequest");
// }

 oldState = newState;

  if (millis() - MLMillis >= mainLoopDelayVar){
   MLMillis = millis();
   if(startUp) {
     startUp = false;
     Serial.println("Startup and running loop");
   }

   if(Serial.available()){serialEvent();}

   if (stringComplete) {autoComplete=false;}
   if (stringComplete || autoComplete) {
     if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
     else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
     if(  inputBuffer[0]=='E'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='e'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='S'     ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='s'     ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='I'     ||        // Command for receiving status/info from other boards
          inputBuffer[0]=='i'     ||          // Command for receiving status/info from other boards
          inputBuffer[0]=='N'     ||        // Command for Sending ESP-NOW Messages
          inputBuffer[0]=='n'         // Command for Sending ESP-NOW Messages
       ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.
         DBG("Command: %s with a length of %d \n", inputBuffer, commandLength);

        if(commandLength >= 3) {
          if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {
             espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
             }; 

          if(inputBuffer[0]=='N' || inputBuffer[0]=='n') {
            for (int i=1; i<=commandLength; i++){
              char inCharRead = inputBuffer[i];
              inputStringCommand += inCharRead;                   // add it to the inputString:
              }
              DBG("\nFull Command Recieved: %s \n",inputStringCommand.c_str());
              targetID = inputStringCommand.substring(0,2);
              // espNowCommandFunction = espNowCommandFunctionString.toInt();
              DBG("ESP NOW Target: %s\n", targetID.c_str());
              // targetID = inputStringCommand.substring(2,4);
              // DBG("Target ID: %s\n", targetID)n
              commandSubString = inputStringCommand.substring(2,commandLength+1);
              DBG("Command to Forward: %s\n", commandSubString.c_str());
              sendESPNOWCommand(targetID, commandSubString);
              String  inputStringCommand = "";
              String commandSubString = "";
              String targetID = "";
            }

          //  if(inputBuffer[0]=='I' || inputBuffer[0]=='i'){
          //    for (int i=1; i<commandLength-1;i++ ){
          //      char inCharRead = inputBuffer[i];
          //      infoCommandString += inCharRead;  // add it to the inputString:
          //    }
          //    DBG_2("I Command Proccessing: %s \n", infoCommandString.c_str());
          //    if(infoCommandString == "PC"){
          //      periscopeControllerStatus="Online";
          //      pckeepAliveAge = millis();
          //      DBG_2("Periscope Controller Keepalive Received\n");
          //    }
          //    if(infoCommandString == "BS"){
          //      bodyServoControllerStatus="Online";
          //      bskeepAliveAge = millis();
          //      DBG_2("Body Servo Controller Keepalive Received\n");
          //    }
          //    if(infoCommandString == "DC"){
          //      domeControllerStatus="Online";
          //      dckeepAliveAge = millis();
          //      DBG_2("Dome Controller Keepalive Received\n");
          //    }
          //    infoCommandString="";
          //  }     
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
               // writeBlSerial(serialSubStringCommand);
               DBG("Sending out BL Serial\n");
             } else if (serialPort == "EN"){
               // writeBsSerial(serialSubStringCommand);
               DBG("Sending out EN Serial\n");
             } else if (serialPort == "ST"){
               // writeStSerial(serialSubStringCommand);
               DBG("Sending out ST Serial\n");
             }else if (serialPort == "MP"){
               mp3Comm = serialStringCommand.substring(2,3);
               // mp3Track = (inputBuffer[4]-'0')*100+(inputBuffer[5]-'0')*10+(inputBuffer[6]-'0');
               DBG("Command: %s, Track: %i\n",mp3Comm, mp3Track);
               // mp3Trigger(mp3Comm,mp3Track);
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

        // reset ESP-NOW Variables
        inputStringCommand = "";
        targetID = "";


   }

    if(ESP_command[0]){
      switch (ESP_command[0]){
        case 1: Serial.println(HOSTNAME);
               ESP_command[0]   = '\0';                                                           break;
        case 2: Serial.println("Resetting the ESP in 3 Seconds");
              //  DelayCall::schedule([] {ESP.restart();}, 3000);
              ESP.restart();
               ESP_command[0]   = '\0';                                                           break;
        case 3: break;  //reserved for commonality. Used for connecting to WiFi and enabling OTA on ESP-NOW Boards 
        case 4: break;  //reserved for future use
        case 5: MainRelayOn();                                                                    break;  //reserved for future use
        case 6: MainRelayOff();                                                                   break;  //reserved for future use
        case 7: sendStatusMessage("Status Update");                  break;  //reserved for future use
        case 8: printKeepaliveStatus();                                                           break;  //reserved for future use
        case 9:  break;  //reserved for future use
        case 10: toggleDebug();                                                                   break;
        case 11: toggleDebug1();                                                                  break;
        case 12: toggleDebug2();                                                                  break;

     }
   }


   
   if(isStartUp) {
     isStartUp = false;
     delay(500);
   }
 }

}




