
// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>

//Used for WiFi
#include "esp_wifi.h" 

#include "ArduinoJson.h"

#include <SPI.h>
#include "LoRa.h"
#include "kill-switch-remote-pin-map.h"
// #include "ds3231.h"
// #include <SD.h>
#include <Adafruit_NeoPixel.h>

//ReelTwo libaries
//#define USE_DEBUG
//#define USE_SERVO_DEBUG
// #include "ReelTwo.h"
// #include "core/DelayCall.h"

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
  String HOSTNAME = "Remote's LoRa/WiFi";

  #define ADCPIN A7
  int adcValue;
float voltValue;

  char inputBuffer[100];
  String inputString;         // a string to hold incoming data
  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA

  int commandLength;
  int paramVar = 9;
  String paramVarString;
  String combinedString;
  
  String serialPort;
  String serialStringCommand;
  String serialSubStringCommand;
  int mp3Track;
  String mp3Comm;
  
  String infofunction;
  String infoCommandString;
  String infoCommandSubString;

  String loRaFunction;
  String loRaCommandString;
  String loRaCommandSubString;

  uint32_t ESP_command[6]  = {0,0,0,0,0,0};
  int espCommandFunction     = 0;

  int debugflag = 1;
  int debugflag1 = 1;  // Used for debugging params recieved from clients
  int debugflag2 = 0;


  //////////////////////////////////////////////////////////////////////
  ///*****            Status Variables                          *****///
  //////////////////////////////////////////////////////////////////////
  boolean droidGatewayStatus = 1;
  boolean relayStatus = 0;
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
  
  String outgoing;
  String LoRaOutgoing;

  byte msgCount = 0;            // count of outgoing messages
  byte receiveMsgCount =0;      // count of incoming messages
  byte localAddress = 0xBC;     // address of this device
  byte destination = 0xFF;      // destination to send to
  long lastSendTime = 0;        // last send time
  int interval = 500;          // interval between sends
  const int csPin = 18;          // LoRa radio chip select
  const int resetPin = 14;       // LoRa radio reset
  const int irqPin = 26;         // change for your board; must be a hardware interrupt pin

  String LoRaRSSI;

  String BATVoltage;
  float batteryFraction;
  int roundedpercent;

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
  // OLED_CLASS_OBJ display(OLED_ADDRESS, OLED_SDA, OLED_SCL);

  //////////////////////////////////////////////////////////////////////
  ///*****       Startup and Loop Variables                     *****///
  //////////////////////////////////////////////////////////////////////
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
const char* ssid = "R_Remote";
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

// void checkAgeofkeepAlive(){    //checks for the variable's age
//   if (domeControllerStatus=="Online"){
//     if (millis()-dckeepAliveAge>=keepAliveTimeOut){
//       domeControllerStatus="Offline";
//       DBG_2("Dome Controller Offline\n");
//     }
//   }
//   if (periscopeControllerStatus=="Online"){
//     if (millis()-pckeepAliveAge>=keepAliveTimeOut){
//       periscopeControllerStatus="Offline";
//       DBG_2("Periscope Controller Offline\n");
//     }
//   }
//   if (bodyServoControllerStatus=="Online"){
//     if (millis()-bskeepAliveAge>=keepAliveTimeOut){
//       bodyServoControllerStatus="Offline";
//       DBG_2("Body Servo Controller Offline\n");
//     }
//   }
//     if (bodyLEDControllerStatus=="Online"){
//     if (millis()-bckeepAliveAge>=keepAliveTimeOut){
//       bodyLEDControllerStatus="Offline";
//       BL_BatteryPercentage = 0;
//       BL_BatteryVoltage = 0.0;
//       DBG_2("Body LED Controller Offline\n");
//     }
//   }
// }

// void checkAgeofBLKeepAlive(){
//   if (BL_LDP_Bright > 0){
//     }
// }

// void printKeepaliveStatus(){

//   DBG("Dome Controller Status: %s\n", domeControllerStatus);
//   DBG("Body Servo Controller Status: %s\n", bodyServoControllerStatus);
//   DBG("Periscope Controller Status: %s\n", periscopeControllerStatus);
//   DBG("Body LED Controller Status: %s\n", bodyLEDControllerStatus);

//   ESP_command[0]   = '\0';

// }

// void displayOLEDString(String StringtoDisplay){
//    display.clear();
//     display.setFont(ArialMT_Plain_16);
//     display.setTextAlignment(TEXT_ALIGN_CENTER);
//     display.drawString(72,0,"Remote");
//     display.setFont(ArialMT_Plain_10);
//     display.setTextAlignment(TEXT_ALIGN_RIGHT);
//     display.drawString(128, 0, LoRaRSSI);
//     display.setTextAlignment(TEXT_ALIGN_LEFT);
//     display.drawString(0, 0, "V: " + BATVoltage);
//     display.drawString(0, 12, String(roundedpercent) + "%");
//     display.setTextAlignment(TEXT_ALIGN_LEFT);
//     display.drawString(0,25,"Network: " + WiFi.SSID());
//     display.drawString(0,35, "IP:" + WiFi.softAPIP().toString());
//     display.drawString(0,45, StringtoDisplay);
//     display.display();
// }



void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  droidGatewayStatus = LoRa.read();
  relayStatus = LoRa.read();
bodyControllerStatus = LoRa.read();
bodyLEDControllerStatus = LoRa.read();
bodyServoStatus = LoRa.read();
domePlateControllerStatus = LoRa.read();
domeControllerStatus = LoRa.read();
BL_LDP_Bright = LoRa.read();
BL_MAINT_Bright = LoRa.read();
BL_VU_Bright = LoRa.read();
BL_CS_Bright = LoRa.read();
BL_vuOffsetInt = LoRa.read();
BL_vuBaselineInt = LoRa.read();
BL_vuOffsetExt = LoRa.read();
BL_vuBaselineExt = LoRa.read();
BL_BatteryVoltage = LoRa.read();
BL_BatteryPercentage = LoRa.read();
  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  // if (incomingLength != incoming.length()) {   // check length for error
  //   Serial.println("error: message length does not match length");
  //   return;                             // skip rest of function
  // }

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
  DBG_2("Droid Gateway Status: %d\n", droidGatewayStatus);
  DBG_2("Main Relay Status: %d\n", relayStatus);
  DBG_2("Body Controller Status: %d\n", bodyControllerStatus);
  DBG_2("Body LED Controller Status: %d\n", bodyLEDControllerStatus);
  DBG_2("Body Servo Status: %d\n", bodyServoStatus);
  DBG_2("Dome Plate Controller Status: %d\n", domePlateControllerStatus);  
  DBG_2("Dome Controller Status: %d\n", domeControllerStatus);
  DBG_2("LDP Brightness: %d\n", BL_LDP_Bright);
  DBG_2("Maintenence Brightness: %d\n", BL_MAINT_Bright);
  DBG_2("VU Brightness: %d\n", BL_VU_Bright);
  DBG_2("Coin Slots Brightness: %d\n", BL_CS_Bright);
  DBG_2("vu Offset Internal: %d\n", BL_vuOffsetInt);
  DBG_2("vu  Baseline Internal: %d\n", BL_vuBaselineInt);
  DBG_2("vu Offset External: %d\n", BL_vuOffsetExt);
  DBG_2("vu Baseline External: %d\n", BL_vuBaselineExt);
  DBG_2("Droid Battery Voltage: %d\n", BL_BatteryVoltage);
  DBG_2("Droid Battery Percentage: %d\n", BL_BatteryPercentage);

  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  LoRaRSSI = String(LoRa.packetRssi());
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  receiveMsgCount++;
  // displayOLEDString(" ");
  // LoRaRSSI = "";
}
int updateStatusDuration = 5000;
unsigned long updateStatusPreviousMillis;
int statusCounter =1;
const float MAX_BATTERY_VOLTAGE = 4.3;

#define R1BAT 100
#define R2BAT 100
#define VOLTAGE_OUT(Vin) (((Vin) * R2BAT) / (R1BAT + R2BAT))
#define VOLTAGE_MAX 4200
#define VOLTAGE_MIN 3300
#define ADC_REFERENCE 2300


#define VOLTAGE_TO_ADC(in) ((ADC_REFERENCE * (in)) / (4096/2))
#define BATTERY_MAX_ADC VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MAX))
#define BATTERY_MIN_ADC VOLTAGE_TO_ADC(VOLTAGE_OUT(VOLTAGE_MIN))

int calc_battery_percentage(int adc)
{
    int battery_percentage = 100 * (adc - BATTERY_MIN_ADC) / (BATTERY_MAX_ADC - BATTERY_MIN_ADC);

    if (battery_percentage < 0)
        battery_percentage = 0;
    if (battery_percentage > 100)
        battery_percentage = 100;

    return battery_percentage;
}

int getBatteryPercentage(){
  

}
void updateStatus(){

  if (millis() - updateStatusPreviousMillis >= updateStatusDuration){
  updateStatusPreviousMillis = millis();
  sendLoRaMessage("E07");
  statusCounter++;
  adcValue = analogRead(ADCPIN);
  voltValue = (adcValue /4095.0) * 2 * 1.1 * 3.3;
  batteryFraction = (voltValue / MAX_BATTERY_VOLTAGE)*100;
  // roundedpercent = round_to_dp(batteryFraction, 2);
  roundedpercent = calc_battery_percentage(adcValue);
     DBG("ADC: %i, Voltage: %f, Percentage %i \n", adcValue, voltValue, roundedpercent);

   BATVoltage = String(voltValue);
   String displayTest = "Sc:" + String(statusCounter) + " Lc:" + String(receiveMsgCount);
  //  displayOLEDString(displayTest);

  }

}

float round_to_dp(float in_value, int decimal_place){
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier)/multiplier;
  return in_value;
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

  Serial.println("\n\n----------------------------------------");
  Serial.print("Booting up the ");Serial.println(HOSTNAME);
  Serial.println("----------------------------------------");

  // digitalWrite(25, HIGH);
    //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);
  // Initialize the OLED
  // Wire.begin();
   
  // if (OLED_RST > 0) {
  //       pinMode(OLED_RST, OUTPUT);
  //       digitalWrite(OLED_RST, HIGH);
  //       delay(100);
  //       digitalWrite(OLED_RST, LOW);
  //       delay(100);
  //       digitalWrite(OLED_RST, HIGH);
  //   }

  //   display.init();
    // display.flipScreenVertically();
    // display.clear();
    // display.setFont(ArialMT_Plain_10);
    // display.setTextAlignment(TEXT_ALIGN_CENTER);
    // display.drawString(display.getWidth() / 2, display.getHeight() / 2, LORA_SENDER ? "LoRa Sender" : "LoRa Receiver");
    // display.display();
    // delay(1000);




    //  String info = ds3231_test();
    //  if (info != "") {
    //      display.clear();
    //      display.setFont(ArialMT_Plain_16);
    //      display.setTextAlignment(TEXT_ALIGN_LEFT);
    //      display.drawString(display.getWidth() / 2, display.getHeight() / 2, info);
    //      display.display();
    //      delay(2000);
    //  }
  // //Initialize the Soft Access Point
  // WiFi.mode(WIFI_AP);
  // Serial.println(WiFi.softAP(ssid,password,channel,broadcastSSID,maxConnections) ? "AP Ready" : "Failed!");
  // delay(200);
  // Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "AP IP Configured" : "Failed!");
  // delay(200);
  // Serial.print("Soft-AP IP address = ");
  // Serial.println(WiFi.softAPIP());
  // disp/lay.clear();
  // display.drawString(display.getWidth() / 2, display.getHeight() / 2, "WiFi Ready");
  // display.display();

  delay(1000);

  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

// SPI.begin(14, 12, 13, 15);
// LoRa.setPins(15, 27, 26);
  if (!LoRa.begin(915E6, true)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  // displayOLEDString("LoRa Ready");

//        display.clear();
//        display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Lora Ready");
//        display.display();
  delay(1000);

  // displayOLEDString("Bootup Complete");

 //Setup the webpage and accept the GET requests, and parses the variables 
//   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//     combinedString = "";

//     int paramsNr = request->params();               // Gets the number of parameters sent
// //    DBG("Parameter %i \n",paramsNr);                       // Variable for selecting which Serial port to send out
//     for(int i=0;i<paramsNr;i++){                     //Loops through all the paramaters
//       AsyncWebParameter* p = request->getParam(i);

// ////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////                                                                //////////////////////////        
// //////////  These If statements choose where to send the commands         //////////////////////////
// //////////  This way we can control multiple serial ports from one ESP32. //////////////////////////
// //////////                                                                //////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////////
        
//     if ((p->name())== "param0" & (p->value()) == "LR"){
//         DBG_1("LoRa Remote Chosen with If Statement\n");
//         paramVar = 0;
//         };
//     if ((p->name())== "param0" & (p->value()) == "LD"){
//         DBG_1("LoRa Droid Chosen with If Statement\n");
//         paramVar = 1;
//         };
        
//         DBG_1("Param name: %s\n", (p->name()));
//         DBG_1("Param value: %s\n", (p->value()).c_str());
  
//         if (paramVar == 0){
//           DBG_1("Executing on local ESP device\n");
//           if ((p->name())== "param0" & (p->value()) == "LR"){
//             DBG_1("Skipping param 0 in the EspNowSerial Write\n");
//           } 
//           else {
//           inputString = (p->value());
//           stringComplete = true;            
//           };
//         };
//         if (paramVar == 1){
//           DBG_1("Sending out LoRa Link\n"); 
// //          delay(100);     
//         if ((p->name())== "param0"){
//             DBG_1("Skipping param 0 in the Lora Message\n");
//           } 
//           else {      
//             combinedString = combinedString + p->value();
//             //  LoRaOutgoing = (p->value());
//             // sendLoRaMessage(LoRaOutgoing);
//             // delay(10);
//             // displayOLEDString(p->value());
//           };
//           DBG_1("Combined Value: %s\n",combinedString.c_str());
//           // sendLoRaMessage(combinedString);
//           delay(1500);
//         } ;      
          
//         DBG_1("------\n");
//         delay(75);
//     }

//     request->send(200, "text/plain", "Message Received on Remote LoRa Controller");
//   });

// server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
//       AsyncResponseStream *response = request->beginResponseStream("application/json");
//       DynamicJsonDocument json(2048);
//       json["remoteLoRaControllerStatus"] = 1;
//       json["droidLoRaControllerStatus"] = droidGatewayStatus;
//       json["LoRaRSSI"] = LoRaRSSI;
//       json["bodyControllerStatus"] = bodyControllerStatus;
//       json["bodyLEDControllerStatus"] = bodyLEDControllerStatus;
//       json["bodyServoControllerStatus"] = bodyServoStatus;
//       json["domeControllerStatus"] = domeControllerStatus;
//       json["periscopeControllerStatus"] = domePlateControllerStatus;
//       json["BL_LDP_Bright"] = BL_LDP_Bright;
//       json["BL_MAINT_Bright"] = BL_MAINT_Bright;
//       json["BL_VU_Bright"] = BL_VU_Bright;
//       json["BL_CS_Bright"] = BL_CS_Bright;
//       json["BL_vuOffsetInt"] = BL_vuOffsetInt;
//       json["BL_vuBaselineInt"] = BL_vuBaselineInt;
//       json["BL_vuOffsetExt"] = BL_vuOffsetExt;
//       json["BL_vuBaselineExt"] = BL_vuBaselineExt;
//       json["BL_BatteryVoltage"] = BL_BatteryVoltage;
//       json["BL_BatteryPercentage"] = BL_BatteryPercentage;

//       serializeJson(json, *response);
//       request->send(response);
//   });

  
//   //Enable Access-Control-Allow-Origin to mitigate errors from website polling
//   DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
//   AsyncElegantOTA.begin(&server);    // Start ElegantOTA

//   //Initialize the AsycWebServer
//   server.begin();

// ESP_LED.begin();
// ESP_LED.show();
// colorWipeStatus("ES", blue, 10);

// RELAY_LED.begin();
// RELAY_LED.show();
// colorWipeStatus("RS", green, 10);

// LORA_LED.begin();
// LORA_LED.show();
// colorWipeStatus("LS", blue, 10);  
}  //end of Setup
      int one;
      String one1;
      int counter123 = 1;

void loop(){
  updateStatus();

  // if (millis() - lastSendTime > interval) {
  //   String message = "Hello from R2's Remote";   // send a message
  //   sendLoRaMessage(message);
  //   Serial.println("Sending " + message);
  //   lastSendTime = millis();            // timestamp the message
  //   interval = random(2000) + 1000;    // 2-3 seconds
  // }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  

  if (millis() - MLMillis >= mainLoopDelayVar){
    MLMillis = millis();
    // AnimatedEvent::process();
    if(startUp) {
      startUp = false;
      Serial.println("Startup");
      sendLoRaMessage("Bootup of Remote Complete");
      // delay(2000);
    }
    
    if(Serial.available()){serialEvent();}
    onReceive(LoRa.parsePacket());

      //  readLoRa();

    if (stringComplete) {autoComplete=false;}
    if (stringComplete || autoComplete) {
      if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
      else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
      if( inputBuffer[0]=='E'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='e'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='S'     ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='s'     ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='L'     ||        // Command for receiving status/info from other boards
          inputBuffer[0]=='l'               // Command for receiving status/info from other boards
        ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.
          DBG("Command: %s with a length of %d \n", inputBuffer, commandLength);

          if(commandLength >= 3) {
            if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {
              espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              }; 

            if(inputBuffer[0]=='L' || inputBuffer[0]=='l'){
              for (int i=1; i<commandLength;i++ ){
                char inCharRead = inputBuffer[i];
                loRaCommandString += inCharRead;  // add it to the inputString:
              }
              DBG_2("I Command Proccessing: %s \n", infoCommandString.c_str());
              loRaCommandSubString = loRaCommandString.substring(0,commandLength+1);
              // DBG("LoRa in Loop : %s\n", loRaCommandSubString);
              sendLoRaMessage(loRaCommandSubString);
              loRaCommandString="";
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
        case 1: Serial.println(HOSTNAME);   
                ESP_command[0]   = '\0';                                                        break;
        case 2: Serial.println("Resetting the ESP in 3 Seconds");
                ESP.restart();
                ESP_command[0]   = '\0';                                                        break;
        case 3: break;  //reserved for commonality. Used for connecting to WiFi and enabling OTA on ESP-NOW Boards 
        // case 4: printKeepaliveStatus();break;  //reserved for future use
        case 5: break;  //reserved for future use
        case 6: break;  //reserved for future use
        case 7: break;  //reserved for future use
        case 8: break;  //reserved for future use
        case 9: break;  //reserved for future use
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


void sendLoRaMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  // Serial.println("Sent LoRa Message");
}
