// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>

//Used for WiFi
#include "esp_wifi.h" 

#include "ArduinoJson.h"

#include <SPI.h>
#include <LoRa.h>
#include "ds3231.h"
#include <SD.h>
//ReelTwo libaries
//#define USE_DEBUG
//#define USE_SERVO_DEBUG
#include "ReelTwo.h"
#include "core/DelayCall.h"

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
  String paramVarString;
  
  String serialPort;
  String serialStringCommand;
  String serialSubStringCommand;
  int mp3Track;
  String mp3Comm;
  
  String infofunction;
  String infoCommandString;
  String infoCommandSubString;

  uint32_t ESP_command[6]  = {0,0,0,0,0,0};
  int espCommandFunction     = 0;

  int debugflag = 1;
  int debugflag1 = 0;  // Used for debugging params recieved from clients
  int debugflag2 = 0;


  //////////////////////////////////////////////////////////////////////
  ///*****            Status Variables                          *****///
  //////////////////////////////////////////////////////////////////////
  String remoteLoRaControllerStatus = "Online";
  String droidLoRaControllerStatus = "Offline";
  String bodyControllerStatus  = "Offline";
  String bodyLEDControllerStatus = "Offline";
  String bodyServoControllerStatus  = "Offline";
  String domeControllerStatus = "Offline";
  String periscopeControllerStatus = "Offline";

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
  
  
  int LoRaRSSI;


  int keepAliveTimeOut = 15000;
  unsigned long dckeepAliveAge;
  unsigned long dckeepaliveAgeMillis;
  unsigned long pckeepAliveAge;
  unsigned long pckeepaliveAgeMillis;
  unsigned long bskeepAliveAge;
  unsigned long bskeepaliveAgeMillis;
  unsigned long bckeepAliveAge;
  unsigned long bckeepaliveAgeMillis;
  unsigned long dlkeepAliveAge;
  unsigned long dlkeepaliveAgeMillis;
  //declare OLED 
  OLED_CLASS_OBJ display(OLED_ADDRESS, OLED_SDA, OLED_SCL);

  //////////////////////////////////////////////////////////////////////
  ///*****       Startup and Loop Variables                     *****///
  //////////////////////////////////////////////////////////////////////
  
  boolean startUp = true;
  boolean isStartUp = true;
  
  unsigned long mainLoopTime; // We keep track of the "Main Loop time" in this variable.
  unsigned long MLMillis;
  unsigned long LMillis;
  int LoraDelay = 1000;
  byte mainLoopDelayVar = 5;

  //////////////////////////////////////////////////////////////////////
  ///******       Serial Ports Specific Setup                   *****///
  //////////////////////////////////////////////////////////////////////

  #define TX 16
  #define RX 17 

  #define bsSerial Serial1

  #define BAUD_RATE 115200

  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////

//LoRa Remote ESP           192.168.4.101   ************
//LoRa Droid ESP            192.168.4.108   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Controller ESP       192.168.4.109   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//ESP-NOW Master ESP        192.168.4.110   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Controller ESP       192.168.4.111   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Periscope Controller ESP  192.168.4.112   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Droid Raspberry Pi        192.168.4.113
//Remote Raspberry Pi       192.168.4.114
//Developer Laptop          192.168.4.125

// IP Address config of local ESP
IPAddress local_IP(192,168,4,101);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,4,101);
uint8_t newMACAddress[] = {0x02, 0x00, 0xC0, 0xA8, 0x04, 0x65};

 ////R2 Control Network Details
const char* ssid = "R2D2_Remote";
const char* password =  "astromech";
int channel =  6;
int broadcastSSID = 0;  //0 for yes, 1 for no
int maxConnections = 8;


AsyncWebServer server(80);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////                                                                                       /////////     
/////////                             Start OF FUNCTIONS                                        /////////
/////////                                                                                       /////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

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




// void serialBlEvent() {
//   while (blSerial.available()) {
//     // get the new byte:
//     StaticJsonDocument<2048> doc;

//     // Read the JSON document from the "link" serial port
//     DeserializationError err = deserializeJson(doc, blSerial);

//     if (err == DeserializationError::Ok) 
//     {
//       BL_Status = doc["BL_Status"].as<String>();
//       if (BL_Status == "Online"){
//         BL_Status="Online";
//         blkeepAliveAge = millis();
//         DBG_2("Body LED Controler Keepalive Received \n");
//         }
//       BL_LDP_Bright = doc["LDPBright"].as<int>();
//       BL_MAINT_Bright = doc["MaintBright"].as<int>();
//       BL_VU_Bright = doc["VUBright"].as<int>();
//       BL_CS_Bright = doc["CoinBright"].as<int>();
//       BL_vuOffsetInt = doc["VUIntOffset"].as<int>();
//       BL_vuBaselineInt = doc["VUIntBaseline"].as<int>();
//       BL_vuOffsetExt = doc["VUExtOffset"].as<int>();
//       BL_vuBaselineExt = doc["VUExtBaseline"].as<int>();
//       BL_BatteryVoltage = doc["BatteryVoltage"].as<float>();
//       BL_BatteryPercentage = doc["BatteryPercent"].as<int>();
//       // Print the values
//       // (we must use as<T>() to resolve the ambiguity)
//       DBG_2("Body LED Controller Status = %s\n", BL_Status); //Serial.println(doc["BL_Status"].as<String>());
//       DBG_2("LDP Brightness = %i\n", BL_LDP_Bright); //Serial.println(doc["LDPBright"].as<int>());
//       DBG_2("Maint Brightness = %i\n", BL_MAINT_Bright); //Serial.println(doc["MaintBright"].as<int>());
//       DBG_2("VU Brightness = %i\n", BL_VU_Bright); //Serial.println(doc["VUBright"].as<int>());
//       DBG_2("Coin Slots Brightness = %i\n",BL_CS_Bright); //Serial.println(doc["CoinBright"].as<int>());
//       DBG_2("Spectrum Int Offset = %i\n",BL_vuOffsetInt); //Serial.println(doc["VUIntOffset"].as<int>());
//       DBG_2("Spectrum Int Baseline = %i\n",BL_vuBaselineInt); //Serial.println(doc["VUIntBaseline"].as<int>());
//       DBG_2("Spectrum Ext Offset = %i\n",BL_vuOffsetExt); //Serial.println(doc["VUExtOffset"].as<int>());
//       DBG_2("Spectrum Ext Baseline = %i\n",BL_vuBaselineExt); //Serial.println(doc["VUExtOffset"].as<int>());
//       DBG_2("Battery Voltage = %.2f\n",BL_BatteryVoltage); //Serial.println(doc["BatteryVoltage"].as<float>());
//       DBG_2("Battery Percentage = %i\n",BL_BatteryPercentage);//Serial.println(doc["BatteryPercent"].as<int>());
//     } 
//     else 
//     {
//       // Print error to the "debug" serial port
//       Serial.print("deserializeJson() returned ");
//       Serial.println(err.c_str());
    
//       // Flush all bytes in the "link" serial port buffer
// //      while (blSerial.available() > 0)
// //        blSerial.read();
//     }
//   };
// };


// void serialStEvent() {
//   while (stSerial.available()) {
//     // get the new byte:
//     char inChar = (char)stSerial.read();
//     // add it to the inputString:
//     inputString += inChar;
//     if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
//       stringComplete = true;            // set a flag so the main loop can do something about it.
//     };
//   };
//   DBG("InputString: %s \n",inputString);
// };

// String mp3TriggerResponseString;

// void serialMpEvent() {
//   while (mpSerial.available()) {
//     char inChar = (char)mpSerial.read();
//     mp3TriggerResponseString += inChar;
//     if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
//     };
//   };
//   DBG("MP3 Trigger Response: %s \n",mp3TriggerResponseString);
//   mp3TriggerResponseString = "";
// };

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

void LoRaSend(String loRaData){
  LoRa.beginPacket();
  LoRa.print(loRaData);
  LoRa.endPacket();
  
}
int LoRaDataCommandLength;
  String LoRaStatusTarget;
  String LoRaStatusCommand ;
  String LoRaStatusVariable;
void readLoRa(){
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet ");
    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
      LoRaDataCommandLength = LoRaData.length();
      LoRaStatusTarget= LoRaData.substring(0,2);
      LoRaStatusCommand = LoRaData.substring(2,4);
      LoRaStatusVariable= LoRaData.substring(4,LoRaDataCommandLength);
      if (LoRaStatusTarget == "BC" & LoRaStatusCommand == "KA"){
        bodyControllerStatus = "Online";
        bckeepAliveAge = millis();
      } else if (LoRaStatusTarget == "BS" & LoRaStatusCommand == "KA"){
        bodyServoControllerStatus = "Online";
        bskeepAliveAge = millis();
      } else if (LoRaStatusTarget == "DC" & LoRaStatusCommand == "KA"){
        domeControllerStatus = "Online";
        bskeepAliveAge = millis();
      }else if (LoRaStatusTarget == "PC" & LoRaStatusCommand == "KA"){
        periscopeControllerStatus = "Online";
        pckeepAliveAge = millis();
      }


      displayOLEDString(LoRaDataCommand);
    }
}
}
void displayOLEDString(String StringtoDisplay){
   display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64,0,"Remote");
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0,25,"Network: " + WiFi.SSID());
    display.drawString(0,35, "IP:" + WiFi.localIP().toString());
    display.drawString(0,45, StringtoDisplay);
    display.display();
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
    if (bodyLEDControllerStatus=="Online"){
    if (millis()-bckeepAliveAge>=keepAliveTimeOut){
      bodyLEDControllerStatus="Offline";
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

void printKeepaliveStatus(){

  DBG("Dome Controller Status: %s\n", domeControllerStatus);
  DBG("Body Servo Controller Status: %s\n", bodyServoControllerStatus);
  DBG("Periscope Controller Status: %s\n", periscopeControllerStatus);
  DBG("Body LED Controller Status: %s\n", bodyLEDControllerStatus);

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
//   bsSerial.begin(BS_BAUD_RATE,SERIAL_8N1,RXBS,TXBS);
//   blSerial.begin(BL_BAUD_RATE,SERIAL_8N1,RXBL,TXBL);
//   stSerial.begin(ST_BAUD_RATE,SWSERIAL_8N1,RXST,TXST,false,95);
//   mpSerial.begin(MP_BAUD_RATE,SWSERIAL_8N1,RXMP,TXMP,false,95);

  Serial.println("\n\n----------------------------------------");
  Serial.println("Booting up the Remote's LoRa to WiFi Bridge");
  
  //Configure the Reset Pins for the arduinoReset() function
//   pinMode(4, OUTPUT);
//   digitalWrite(4,HIGH);


  
  //Initialize the ReelTwo Library
//  SetupEvent::ready();

  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);
  // Initialize the OLED
    Wire.begin();
  if (OLED_RST > 0) {
        pinMode(OLED_RST, OUTPUT);
        digitalWrite(OLED_RST, HIGH);
        delay(100);
        digitalWrite(OLED_RST, LOW);
        delay(100);
        digitalWrite(OLED_RST, HIGH);
    }

    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(display.getWidth() / 2, display.getHeight() / 2, LORA_SENDER ? "LoRa Sender" : "LoRa Receiver");
    display.display();
    delay(1000);

    if (SDCARD_CS >  0) {
        display.clear();
        SPIClass sdSPI(VSPI);
        sdSPI.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);
        if (!SD.begin(SDCARD_CS, sdSPI)) {
            display.drawString(display.getWidth() / 2, display.getHeight() / 2, "SDCard  FAIL");
        } else {
            display.drawString(display.getWidth() / 2, display.getHeight() / 2 - 16, "SDCard  PASS");
            uint32_t cardSize = SD.cardSize() / (1024 * 1024);
            display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Size: " + String(cardSize) + "MB");
        }
        display.display();
        delay(1000);
    }


    // String info = ds3231_test();
    // if (info != "") {
    //     display.clear();
    //     display.setFont(ArialMT_Plain_16);
    //     display.setTextAlignment(TEXT_ALIGN_LEFT);
    //     display.drawString(display.getWidth() / 2, display.getHeight() / 2, info);
    //     display.display();
    //     delay(2000);
    // }
  //Initialize the Soft Access Point
  WiFi.mode(WIFI_AP);
  Serial.println(WiFi.softAP(ssid,password,channel,broadcastSSID,maxConnections) ? "AP Ready" : "Failed!");
  delay(200);
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "AP IP Configured" : "Failed!");
  delay(200);
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  display.clear();
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, "WiFi Ready");
  display.display();


esp_wifi_set_mac(WIFI_IF_AP, &newMACAddress[0]);
delay(1000);
  Serial.print("Local AP MAC address = ");
Serial.println(WiFi.softAPmacAddress());


SPI.begin(CONFIG_CLK, CONFIG_MISO, CONFIG_MOSI, CONFIG_NSS);
LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0);
    LoRa.setSyncWord(0xF3);

    if (!LoRa.begin(BAND)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

        display.clear();
        display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Lora Ready");
        display.display();
  delay(1000);

  displayOLEDString("Bootup Complete");
    // display.clear();
    // display.setFont(ArialMT_Plain_16);
    // display.setTextAlignment(TEXT_ALIGN_CENTER);
    // display.drawString(64,0,"R2 Remote");
    // display.setFont(ArialMT_Plain_10);
    // display.setTextAlignment(TEXT_ALIGN_LEFT);
    // display.drawString(0,25,"Network: R2_Remote");
    // display.drawString(0,35, "IP:" + WiFi.softAPIP().toString());
    // display.display();
 
 //Setup the webpage and accept the GET requests, and parses the variables 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      
    int paramsNr = request->params();               // Gets the number of parameters sent
//    DBG("Parameter %i \n",paramsNr);                       // Variable for selecting which Serial port to send out
    for(int i=0;i<paramsNr;i++){                     //Loops through all the paramaters
      AsyncWebParameter* p = request->getParam(i);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////                                                                //////////////////////////        
//////////  These If statements choose where to send the commands         //////////////////////////
//////////  This way we can control multiple serial ports from one ESP32. //////////////////////////
//////////                                                                //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
        
    if ((p->name())== "param0" & (p->value()) == "LR"){
        DBG_1("LoRa Remote Chosen with If Statement\n");
        paramVar = 0;
        };
    if ((p->name())== "param0" & (p->value()) == "LD"){
        DBG_1("LoRa Droid Chosen with If Statement\n");
        paramVar = 1;
        };
        
        DBG_1("Param name: %s\n", (p->name()));
        DBG_1("Param value: %s\n", (p->value()).c_str());
  
        if (paramVar == 0){
          DBG_1("Executing on local ESP device\n");
          if ((p->name())== "param0" & (p->value()) == "LR"){
            DBG_1("Skipping param 0 in the EspNowSerial Write\n");
          } 
          else {
          inputString = (p->value());
          stringComplete = true;            
          };
        };
        if (paramVar == 1){
          DBG_1("Sending out LoRa Link\n"); 
//          delay(100);     
        if ((p->name())== "param0" & (p->value()) == "LD"){
            DBG_1("Skipping param 0 in the Lora Message\n");
          } 
          else {
            LoRaSend(p->value());
          };
        } ;      

        DBG_1("------\n");
        delay(50);
    }

    request->send(200, "text/plain", "Message Received on Remote LoRa Controller");
  });

server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
      AsyncResponseStream *response = request->beginResponseStream("application/json");
      DynamicJsonDocument json(2048);
      json["remoteLoRaControllerStatus"] = remoteLoRaControllerStatus;
      json["droidLoRaControllerStatus"] = droidLoRaControllerStatus;
      json["LoRaRSSI"] = LoRaRSSI;
      json["bodyControllerStatus"] = bodyControllerStatus;
      json["bodyLEDControllerStatus"] = bodyLEDControllerStatus;
      json["bodyServoControllerStatus"] = bodyServoControllerStatus;
      json["domeControllerStatus"] = domeControllerStatus;
      json["periscopeControllerStatus"] = periscopeControllerStatus;
      json["BL_LDP_Bright"] = BL_LDP_Bright;
      json["BL_MAINT_Bright"] = BL_MAINT_Bright;
      json["BL_VU_Bright"] = BL_VU_Bright;
      json["BL_CS_Bright"] = BL_CS_Bright;
      json["BL_vuOffsetInt"] = BL_vuOffsetInt;
      json["BL_vuBaselineInt"] = BL_vuBaselineInt;
      json["BL_vuOffsetExt"] = BL_vuOffsetExt;
      json["BL_vuBaselineExt"] = BL_vuBaselineExt;
      json["BL_BatteryVoltage"] = BL_BatteryVoltage;
      json["BL_BatteryPercentage"] = BL_BatteryPercentage;

      serializeJson(json, *response);
      request->send(response);
  });

  
  //Enable Access-Control-Allow-Origin to mitigate errors from website polling
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA

  //Initialize the AsycWebServer
  server.begin();


}  //end of Setup
      int one;
      String one1;
      int counter123 = 1;

void loop(){
  // readLoRa();
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet ");
    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
      LoRaDataCommandLength = LoRaData.length();
      LoRaStatusTarget= LoRaData.substring(0,2);
      LoRaStatusCommand = LoRaData.substring(2,4);
      LoRaStatusVariable= LoRaData.substring(4,LoRaDataCommandLength);
      if (LoRaStatusTarget == "BC" & LoRaStatusCommand == "KA"){
        bodyControllerStatus = "Online";
        bckeepAliveAge = millis();
      } else if (LoRaStatusTarget == "BS" & LoRaStatusCommand == "KA"){
        bodyServoControllerStatus = "Online";
        bskeepAliveAge = millis();
      } else if (LoRaStatusTarget == "DC" & LoRaStatusCommand == "KA"){
        domeControllerStatus = "Online";
        bskeepAliveAge = millis();
      }else if (LoRaStatusTarget == "PC" & LoRaStatusCommand == "KA"){
        periscopeControllerStatus = "Online";
        pckeepAliveAge = millis();
      }


      displayOLEDString(LoRaDataCommand);
    }
  }
  if (millis() - MLMillis >= mainLoopDelayVar){
    MLMillis = millis();
    AnimatedEvent::process();
    if(startUp) {
      startUp = false;
      Serial.println("Startup");
      // Play Startup Sound
      LoRa.beginPacket();
       one = random(0,100);
       one1 =String(one);
      LoRa.print("Remote " + one1);
      LoRa.endPacket();

    }
  // if(millis() - LMillis >= LoraDelay){
  //   LMillis = millis();
  //   LoRa.beginPacket();
  //   LoRa.print("");
  //   LoRa.endPacket();
  // }
    
    if(Serial.available()){serialEvent();}

    
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

            // if(inputBuffer[0]=='I' || inputBuffer[0]=='i'){
            //   for (int i=1; i<commandLength-1;i++ ){
            //     char inCharRead = inputBuffer[i];
            //     infoCommandString += inCharRead;  // add it to the inputString:
            //   }
            //   DBG_2("I Command Proccessing: %s \n", infoCommandString.c_str());
            //   if(infoCommandString == "PC"){
            //     periscopeControllerStatus="Online";
            //     pckeepAliveAge = millis();
            //     DBG_2("Periscope Controller Keepalive Received\n");
            //   }
            //   if(infoCommandString == "BS"){
            //     bodyServoControllerStatus="Online";
            //     bskeepAliveAge = millis();
            //     DBG_2("Body Servo Controller Keepalive Received\n");
            //   }
            //   if(infoCommandString == "DC"){
            //     domeControllerStatus="Online";
            //     dckeepAliveAge = millis();
            //     DBG_2("Dome Controller Keepalive Received\n");
            //   }
            //   infoCommandString="";
            // }     
            // if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
            //   // serialPort =  (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
            //   for (int i=1; i<commandLength;i++ ){
            //     char inCharRead = inputBuffer[i];
            //     serialStringCommand += inCharRead;  // add it to the inputString:
            //   }
            //   DBG("Full Serial Command Captured: %s\n", serialStringCommand.c_str());
            //   serialPort = serialStringCommand.substring(0,2);
            //   serialSubStringCommand = serialStringCommand.substring(2,commandLength);
            //   DBG("Serial Command: %s to Serial Port: %s\n", serialSubStringCommand.c_str(), serialPort);
            //   if (serialPort == "BL"){
            //     // writeBlSerial(serialSubStringCommand);
            //     DBG("Sending out BL Serial\n");
            //   } else if (serialPort == "EN"){
            //     // writeBsSerial(serialSubStringCommand);
            //     DBG("Sending out EN Serial\n");
            //   } else if (serialPort == "ST"){
            //     // writeStSerial(serialSubStringCommand);
            //     DBG("Sending out ST Serial\n");
            //   }else if (serialPort == "MP"){
            //     mp3Comm = serialStringCommand.substring(2,3);
            //     // mp3Track = (inputBuffer[4]-'0')*100+(inputBuffer[5]-'0')*10+(inputBuffer[6]-'0');
            //     DBG("Command: %s, Track: %i\n",mp3Comm, mp3Track);
            //     // mp3Trigger(mp3Comm,mp3Track);
            //     DBG("Sending out MP Serial\n ");
            //   } else { DBG("No valid Serial identified\n");}
            //   serialStringCommand = "";
            //   serialPort = "";
            //   serialSubStringCommand = "";
            //   int mp3Track;
            // } 
            


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
        // case 4: printKeepaliveStatus();break;  //reserved for future use
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

