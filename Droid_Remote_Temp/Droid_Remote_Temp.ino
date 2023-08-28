
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
#include "pin-map.h"
#include "ds3231.h"
// #include <SD.h>
#include <Adafruit_NeoPixel.h>
#include "ArduinoJson.h"


//ReelTwo libaries
//#define USE_DEBUG
//#define USE_SERVO_DEBUG
// #include "ReelTwo.h"
// #include "core/DelayCall.h"

#include <Toggle.h>
#include <DebugR2.h>  //  https://github.com/greghulette/Arduino-Code/tree/main/libraries/DebugR2  Put these files in a folder called "DebugR2" in your libraries folder and restart the IDE


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
  #define MAX_LINE_LENGTH 100
  char buffer[MAX_LINE_LENGTH];
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
  // int mp3Track;
  // String mp3Comm;
  
  String infofunction;
  String infoCommandString;
  String infoCommandSubString;

  String loRaFunction;
  String loRaCommandString;
  String loRaCommandSubString;



    uint32_t Local_Command[6]  = {0,0,0,0,0,0};
  int localCommandFunction     = 0;



debugClass Debug;
String debugInputIdentifier ="";

  //////////////////////////////////////////////////////////////////////
  ///*****            Status Variables                          *****///
  //////////////////////////////////////////////////////////////////////
  boolean droidGatewayStatus = 1;
  boolean relayStatus;
  boolean bodyControllerStatus = 0;
  boolean bodyLEDControllerStatus = 0;  
  boolean bodyServoControllerStatus = 0;
  boolean domePlateControllerStatus = 0;
  boolean domeControllerStatus = 0;
  boolean hpControllerStatus = 0;
  boolean domeLogicsControllerStatus = 0;

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
  byte localAddress = 0xFF;     // address of this device
  byte destination = 0xBC;      // destination to send to
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
  unsigned long keepAliveMillis;
  unsigned long dckeepAliveAge;
  unsigned long dckeepAliveAgeMillis;
  unsigned long dpkeepAliveAge;
  unsigned long dpkeepAliveAgeMillis;
  unsigned long bskeepAliveAge;
  unsigned long bskeepAliveAgeMillis;
  unsigned long bckeepAliveAge;
  unsigned long bckeepAliveAgeMillis;
  unsigned long blkeepAliveAge;
  unsigned long blkeepAliveAgeMillis;
  unsigned long drkeepAliveAge;
  unsigned long hpkeepAliveAgeMillis;
  unsigned long hpkeepAliveAge;
  unsigned long dgkeepAliveAgeMillis;
  unsigned long dgkeepAliveAge;
  //declare OLED 
  OLED_CLASS_OBJ display(OLED_ADDRESS, OLED_SDA, OLED_SCL);

  int buttonState = 0;            // the current reading from the input pin
// int lastButtonState = 0;  // the previous reading from the input pin

// unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
// unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

Toggle button(BUTTON);

  int mp3Track;
  String mp3Comm;
  String mp3TriggerResponseString;
    String mp3CommandString;
  String mp3CommandSubString;

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
  int dim = 75;
  unsigned long CLMillis;
  byte CLSpeed = 50;
  unsigned long startMillis;
  unsigned long currentMillis;
  byte LED_command[6] = {0,0,0,0,0,0};
  
  int colorState1;
  int speedState;
  
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
const char* ssid = "R_Remote";                  // Test Network
// const char* ssid = "R2D2_Control_Network";   
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

// /////////////////////////////////////////////////////////////////////////////////////////////////////////
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

// void serialEvent() {
//   while (Serial.available()) {
//     // get the new byte:
//     char inChar = (char)Serial.read();
//     // add it to the inputString:
//     inputString += inChar;
//     if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
//       // stringComplete = true;            // set a flag so the main loop can do something about it.
//       enqueueCommand(inputString);
//     };
//   };
//   Debug.DBG("InputString: %s \n",inputString);
// };

void serialEvent() {
  if (readLine() > 0) //Did we receive something?
  {
    char *token = strtok(buffer, " "); //Start tokenization
    while (token) //While a token is available
    {
      Serial.println(token); //Print current token
      enqueueCommand(token);
      token = strtok(NULL, ","); //Fetch next token
    }
  }
};

int readLine()
{
  int cc = 0; //Number of chars read
  bool done = false;
  while (Serial.available() && (!done) && (cc < MAX_LINE_LENGTH - 1))
  {
    char cur = Serial.read(); //Read a char
    if (cur == '\r') done = true; //If the received char is \n then we are done
    else buffer[cc++] = cur; //Append to buffer and increment the index
  }
  buffer[cc] = 0; //Terminate the line with a \0
  // delay(50);
  return cc; //Return line length
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




//////////////////////////////////////////////////////////////////////
///*****    Checks the age of the Status Variables            *****///
//////////////////////////////////////////////////////////////////////


void checkAgeofkeepAlive(){    //checks for the variable's age
  if (droidGatewayStatus== true){
    if (millis() - dgkeepAliveAge>=keepAliveTimeOut){
      droidGatewayStatus = false;
      Debug.STATUS("Dome Controller Offline\n");
    }
  }  if (domeControllerStatus== true){
    if (millis() - dckeepAliveAge>=keepAliveTimeOut){
      domeControllerStatus = false;
      Debug.STATUS("Dome Controller Offline\n");
    }
  }
  if (domePlateControllerStatus== true){
    if (millis()-dpkeepAliveAge>=keepAliveTimeOut){
      domePlateControllerStatus= false;
      Debug.STATUS("Dome Plate Controller Offline\n");
    }
  }
  if (bodyServoControllerStatus == true){
    if (millis()-bskeepAliveAge>=keepAliveTimeOut){
      bodyServoControllerStatus = false;
      Debug.STATUS("Body Servo Controller Offline\n");
    }
  }
  if (bodyControllerStatus == true){
    if (millis()-bckeepAliveAge>=keepAliveTimeOut){
      bodyControllerStatus = false;
      BL_LDP_Bright =0;
      BL_MAINT_Bright = 0;
      BL_VU_Bright = 0;
      BL_CS_Bright =0;
      BL_vuOffsetInt = 0;
      BL_vuBaselineInt = 0;
      BL_vuOffsetExt = 0;
      BL_vuBaselineExt =0;
      BL_BatteryPercentage = 0;
      BL_BatteryVoltage = 0.0;
      Debug.STATUS("Body Controller Offline\n");
    }
  }
  // if (droidRemoteStatus == true){
  //   if (millis()-drkeepAliveAge>=keepAliveTimeOut){
  //     droidRemoteStatus = false;
  //     colorWipeStatus("LS", red, 20);
  //     Debug.STATUS("Droid Remote Offline\n");
  //   }
  // }
    if (hpControllerStatus == true){
    if (millis()-hpkeepAliveAge>=keepAliveTimeOut){
      hpControllerStatus = false;
      Debug.STATUS("HP Controller Offline\n");
    }
  }
}

void checkAgeofBLKeepAlive(){
  if (BL_LDP_Bright > 0){
    }
}

void printKeepaliveStatus(){

  Debug.DBG("Dome Controller Status: %i\n", domeControllerStatus);
  Debug.DBG("Body Servo Controller Status: %i\n", bodyServoControllerStatus);
    Debug.DBG("Periscope Controller Status: %i\n", domePlateControllerStatus);
  Debug.DBG("Body LED Controller Status: %i\n", bodyLEDControllerStatus);

  Local_Command[0]   = '\0';

}

void displayOLEDString(String StringtoDisplay){
   display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(72,0,"Remote");
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 0, LoRaRSSI);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "V: " + BATVoltage);
    display.drawString(0, 12, String(roundedpercent) + "%");
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    if (relayStatus == 1){display.drawString(0, 25, "RELAY STATUS: ON"); }
    if (relayStatus == 0){display.drawString(0, 25, "RELAY STATUS: OFF"); }

    // display.drawString(0,25,"Network: " + WiFi.SSID());
    // display.drawString(0,35, "IP:" + WiFi.softAPIP().toString());
    display.drawString(0,45, StringtoDisplay);
    display.display();
}


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
bodyServoControllerStatus = LoRa.read();
domePlateControllerStatus = LoRa.read();
domeControllerStatus = LoRa.read();
hpControllerStatus = LoRa.read();
domeLogicsControllerStatus = LoRa.read();
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
String senderString = String(sender, HEX);
String recipientString = String(recipient, HEX);
  // if message is for this device, or broadcast, print details:
  if (Debug.debugflag_lora == 1){
    Serial.println("Received from: 0x" + senderString);
    Serial.println("Sent to: 0x" + recipientString);
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
  }
  Debug.STATUS("Droid Gateway Status: %d\n", droidGatewayStatus);
  Debug.STATUS("Main Relay Status: %d\n", relayStatus);
  Debug.STATUS("Body Controller Status: %d\n", bodyControllerStatus);
  Debug.STATUS("Body LED Controller Status: %d\n", bodyLEDControllerStatus);
  Debug.STATUS("Body Servo Status: %d\n", bodyServoControllerStatus);
  Debug.STATUS("Dome Plate Controller Status: %d\n", domePlateControllerStatus);  
  Debug.STATUS("Dome Controller Status: %d\n", domeControllerStatus);
  Debug.STATUS("LDP Brightness: %d\n", BL_LDP_Bright);
  Debug.STATUS("Maintenence Brightness: %d\n", BL_MAINT_Bright);
  Debug.STATUS("VU Brightness: %d\n", BL_VU_Bright);
  Debug.STATUS("Coin Slots Brightness: %d\n", BL_CS_Bright);
  Debug.STATUS("vu Offset Internal: %d\n", BL_vuOffsetInt);
  Debug.STATUS("vu  Baseline Internal: %d\n", BL_vuBaselineInt);
  Debug.STATUS("vu Offset External: %d\n", BL_vuOffsetExt);
  Debug.STATUS("vu Baseline External: %d\n", BL_vuBaselineExt);
  Debug.STATUS("Droid Battery Voltage: %d\n", BL_BatteryVoltage);
  Debug.STATUS("Droid Battery Percentage: %d\n", BL_BatteryPercentage);

  LoRaRSSI = String(LoRa.packetRssi());

  // Serial.println();
  receiveMsgCount++;
  // displayOLEDString(" ");
  // LoRaRSSI = "";
  if (droidGatewayStatus == true){  dgkeepAliveAge = millis();};
  if (bodyControllerStatus == true) {bckeepAliveAge = millis();};
  if (bodyServoControllerStatus == true){bskeepAliveAge = millis();};
  if (domePlateControllerStatus == true) {dpkeepAliveAge = millis();};
  if (domeControllerStatus == true){dckeepAliveAge = millis();};
  if (hpControllerStatus == true){hpkeepAliveAge = millis();};

  


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

void sendUpdates(){
//  Serial.println("Executing JSON");
  DynamicJsonDocument doc(2048);
  doc["droidremoteControllerStatus"] = true;
  doc["droidgatewayControllerStatus"] = droidGatewayStatus;
  doc["relayStatus"] = relayStatus;
  doc["LoRaRSSI"] = LoRaRSSI;
  doc["bodyControllerStatus"] = bodyControllerStatus;
  doc["bodyLEDControllerStatus"] = bodyLEDControllerStatus;
  doc["bodyServoControllerStatus"] = bodyServoControllerStatus;
  doc["domeControllerStatus"] = domeControllerStatus;
  doc["domePlateControllerStatus"] = domePlateControllerStatus;
  doc["hpControllerStatus"] = hpControllerStatus;
  doc["LDPBright"] = BL_LDP_Bright;
  doc["MaintBright"] = BL_MAINT_Bright;
  doc["VUBright"] = BL_VU_Bright;
  doc["CoinBright"] = BL_CS_Bright;
  doc["VUIntBaseline"] = BL_vuBaselineInt;
  doc["VUIntOffset"] = BL_vuOffsetInt;
  doc["VUExtBaseline"] = BL_vuBaselineExt;
  doc["VUExtOffset"] = BL_vuOffsetExt;
  doc["BatteryVoltage"] = BL_BatteryVoltage;
  doc["BatteryPercent"] = BL_BatteryPercentage;
  doc["JSONDone"] = true;
  
  serializeJson(doc, Serial);
  Serial.println("");

}






void updateStatus(){

  if (millis() - updateStatusPreviousMillis >= updateStatusDuration){
  updateStatusPreviousMillis = millis();
  // sendLoRaMessage("#L07");
  statusCounter++;
  sendUpdates();
  adcValue = analogRead(ADCPIN);
  voltValue = (adcValue /4095.0) * 2 * 1.1 * 3.3;
  batteryFraction = (voltValue / MAX_BATTERY_VOLTAGE)*100;
  // roundedpercent = round_to_dp(batteryFraction, 2);
  roundedpercent = calc_battery_percentage(adcValue);
     Debug.STATUS("ADC: %i, Voltage: %f, Percentage %i \n", adcValue, voltValue, roundedpercent);

   BATVoltage = String(voltValue);
   String displayTest = "Sc:" + String(statusCounter) + " Lc:" + String(receiveMsgCount);
   displayOLEDString(displayTest);
  }

}

float round_to_dp(float in_value, int decimal_place){
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier)/multiplier;
  return in_value;
}


String getValue(String data, char separator, int index){
  int found=0;
  int strIndex[] = {0,-1};
  int maxIndex = data.length()-1;
  for (int i=0; i <= maxIndex && found <= index; i++){
    if(data.charAt(i) == separator || i == maxIndex){
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i == maxIndex)? i+1 : i;

    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


// <-- code from Mimir for queueing incoming commands from GET params-->
////////////////////////////////////////////////////

#define MAX_QUEUE_DEPTH 5

////////////////////////////////////////////////////

template<class T, int maxitems>
class Queue {
  private:
    int _front = 0, _back = 0, _count = 0;
    T _data[maxitems + 1];
    int _maxitems = maxitems;
  public:
    inline int count() { return _count; }
    inline int front() { return _front; }
    inline int back()  { return _back;  }

    void push(const T &item) {
      if(_count < _maxitems) { // Drops out when full
        _data[_back++]=item;
        ++_count;
        // Check wrap around
        if (_back > _maxitems)
          _back -= (_maxitems + 1);
      }
    }

    T peek() {
      return (_count <= 0) ? T() : _data[_front];
    }

    T pop() {
      if (_count <= 0)
        return T(); // Returns empty

      T result = _data[_front];
      _front++;
      --_count;
      // Check wrap around
      if (_front > _maxitems) 
        _front -= (_maxitems + 1);
      return result; 
    }

    void clear() {
      _front = _back;
      _count = 0;
    }
};

template <int maxitems = MAX_QUEUE_DEPTH>
using CommandQueue = Queue<String, maxitems>;

////////////////////////////////////////////////////


CommandQueue<> commandQueue;

bool havePendingCommands()
{
  return (commandQueue.count() > 0);
}

String getNextCommand()
{
  return commandQueue.pop();
}

void enqueueCommand(String command)
{
  commandQueue.push(command);
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
  // pinMode(BUTTON, INPUT);
  button.begin(BUTTON);
  // button.setToggleState(0);
  button.setToggleTrigger(0);
  // button.setInputInvert(1);
  Serial.println("\n\n----------------------------------------");
  Serial.print("Booting up the ");Serial.println(HOSTNAME);
  Serial.println("----------------------------------------\n");

  // HCR.begin();
  // digitalWrite(25, HIGH);
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




     String info = ds3231_test();
     if (info != "") {
         display.clear();
         display.setFont(ArialMT_Plain_16);
         display.setTextAlignment(TEXT_ALIGN_LEFT);
         display.drawString(display.getWidth() / 2, display.getHeight() / 2, info);
         display.display();
         delay(2000);
     }
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

  delay(1000);
 SPI.begin(SCK_PIN,MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

// SPI.begin(14, 12, 13, 15);
// LoRa.setPins(15, 27, 26);
  if (!LoRa.begin(915E6, true)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  displayOLEDString("LoRa Ready");

      //  display.clear();
      //  display.drawString(display.getWidth() / 2, display.getHeight() / 2, "Lora Ready");
      //  display.display();
  delay(1000);

  displayOLEDString("Bootup Complete");

 //Setup the webpage and accept the GET requests, and parses the variables 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    combinedString = "";

    int paramsNr = request->params();               // Gets the number of parameters sent
//    Debug.DBG("Parameter %i \n",paramsNr);                       // Variable for selecting which Serial port to send out
    for(int i=0;i<paramsNr;i++){                     //Loops through all the paramaters
      AsyncWebParameter* p = request->getParam(i);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////                                                                //////////////////////////        
//////////  These If statements choose where to send the commands         //////////////////////////
//////////  This way we can control multiple serial ports from one ESP32. //////////////////////////
//////////                                                                //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
  // String pvalueSubString = (p->value().substring(0,1));
    // if ((p->name())== "param0" & (p->value()) == "#"){
    //     Debug.PARAM("Executing command for local device \n");
    //     paramVar = 0;
    //     };
    // if ((p->name())== "param0" & (p->value()) == ":"){
    //     Debug.PARAM("LoRa Droid Chosen with If Statement\n");
    //     paramVar = 1;
    //     };
        
        Debug.PARAM("Param name: %s\n", (p->name()));
        Debug.PARAM("Param value: %s\n", (p->value()).c_str());
  
//         if (paramVar == 0){
//           Debug.DBG_1("Executing on local ESP device\n");
//           if ((p->name())== "param0" & (p->value()) == "LR"){
//             Debug.DBG_1("Skipping param 0 in the EspNowSerial Write\n");
//           } 
//           else {
//           inputString = (p->value());
//           stringComplete = true;            
//           };
//         };
        // if (paramVar == 1){
          Debug.PARAM("Executing Command\n"); 
//          delay(100);     
        if ((p->name())== "param0"){
            Debug.PARAM("Skipping param 0 in the Lora Message\n");
          } 
          else {      
            enqueueCommand(p->value());

          // inputString = (p->value());
          // stringComplete = true;  

          Debug.PARAM("sent to loop: %s \n", inputString.c_str());
          delay(75);
          };
    }

    request->send(200, "text/plain", "Message Received on Remote LoRa Controller");
  });

server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
      AsyncResponseStream *response = request->beginResponseStream("application/json");
      DynamicJsonDocument json(2048);
      json["droidremoteControllerStatus"] = true;
      json["droidgatewayControllerStatus"] = droidGatewayStatus;
      json["relayStatus"] = relayStatus;
      json["LoRaRSSI"] = LoRaRSSI;
      json["bodyControllerStatus"] = bodyControllerStatus;
      json["bodyLEDControllerStatus"] = bodyLEDControllerStatus;
      json["bodyServoControllerStatus"] = bodyServoControllerStatus;
      json["domeControllerStatus"] = domeControllerStatus;
      json["domePlateControllerStatus"] = domePlateControllerStatus;
      json["hpControllerStatus"] = hpControllerStatus;
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
  checkAgeofkeepAlive();
  onReceive(LoRa.parsePacket());
  yield();
button.poll();
if (button.onPress()) {
  Serial.println("Changed");
  relayStatus = !relayStatus;
  if (relayStatus == 0){
    Serial.println("Relay OFF");
    displayOLEDString("RELAY OFF");
    sendLoRaMessage("#L06");
  }  if (relayStatus == 1){
    Serial.println("Relay ON");
    displayOLEDString("RELAY ON");
    sendLoRaMessage("#L05");
  }
}
  if (millis() - MLMillis >= mainLoopDelayVar){
    MLMillis = millis();
    // AnimatedEvent::process();
    if(startUp) {
      startUp = false;
      Serial.println("Startup");
      // sendLoRaMessage("Bootup of Remote Complete");
      // delay(2000);
      displayOLEDString("Running Loop");
    }
    
    if(Serial.available()){serialEvent();}
// 
      //  readLoRa();

  if (havePendingCommands()) {Serial.println("HasCommands");autoComplete=false;}
  if (havePendingCommands() || autoComplete) {
      
    if(havePendingCommands()) {inputString = getNextCommand();  displayOLEDString(inputString);inputString.toCharArray(inputBuffer, 100);inputString="";}
    else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}

    
// if (havePendingCommands()){ inputString = getNextCommand(); Serial.print("Param Recieved and passed havePending: ");Serial.println(inputString);}
//     if (havePendingCommands()) {autoComplete=false;}
//     if (havePendingCommands() || autoComplete) {
// //     //   inputString = "";
//      inputString = getNextCommand();
//       if(havePendingCommands()) {inputString.toCharArray(inputBuffer, 100);inputString="";}
// //       // if(stringComplete || havePendingCommands()) {inputString.toCharArray(inputBuffer, 100);}
//   // if (stringComplete) {autoComplete=false;}
//   // if (stringComplete || autoComplete) {
//     if(havePendingCommands()) {inputString.toCharArray(inputBuffer, 100);inputString="";} else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}

      if (inputBuffer[0] == '#'){
        if (
            inputBuffer[1]=='D' ||          // Command for debugging
            inputBuffer[1]=='d' ||          // Command for debugging
            inputBuffer[1]=='L' ||          // Command designator for internal functions
            inputBuffer[1]=='l' ||          // Command designator for internal functions
            inputBuffer[1]=='E' ||          // Command designator for storing EEPROM data
            inputBuffer[1]=='e'           // Command designator for storing EEPROM data
          ){commandLength = strlen(inputBuffer); 
            if (inputBuffer[1]=='D' || inputBuffer[1]=='d'){
              debugInputIdentifier = "";                            // flush the string
              for (int i=2; i<=commandLength-2; i++){
                char inCharRead = inputBuffer[i];
                debugInputIdentifier += inCharRead;                   // add it to the inputString:
              }
              debugInputIdentifier.toUpperCase();
              Serial.println(debugInputIdentifier);
              Debug.toggle(debugInputIdentifier);
              debugInputIdentifier = "";                             // flush the string
              } else if (inputBuffer[1]=='L' || inputBuffer[1]=='l') {
                localCommandFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');
                Local_Command[0]   = '\0';                                                            // Flushes Array
                Local_Command[0] = localCommandFunction;
              Debug.LOOP("Entered the Local Command Structure /n");
              } else if (inputBuffer[1] == 'E' || inputBuffer[1] == 'e'){
                Debug.LOOP("EEPROM configuration selected /n");
                // need to actually add the code to implement this.

              } else {Debug.LOOP("No valid command entered /n");}
              
          }
              if(Local_Command[0]){
                switch (Local_Command[0]){
                  case 1: Serial.println(HOSTNAME);
                        Local_Command[0]   = '\0';                                                           break;
                  case 2: Serial.println("Resetting the ESP in 3 Seconds");
                        //  DelayCall::schedule([] {ESP.restart();}, 3000);
                        ESP.restart();
                        Local_Command[0]   = '\0';                                                           break;
                  // case 3: connectWiFi();                                                                          break;
                  case 4: break;  //reserved for future use
                  // case 5: MainRelayOn();                                                                    break;  //reserved for future use
                  // case 6: MainRelayOff();                                                                   break;  //reserved for future use
                  // case 7: sendStatusMessage("Status Update");                  break;  //reserved for future use
                  case 8: printKeepaliveStatus();                                                           break;  //reserved for future use
                  case 9:  break;  //reserved for future use

                }
              }

        }else if (inputBuffer[0] == ':'){
          Serial.println("Enetered Execution of Commands");
     
      if( 
          inputBuffer[1]=='L'     ||        // Command designator for sending LoRa messages
          inputBuffer[1]=='l'               // Command fdesignator for sending LoRa messages
        ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.
          Debug.LOOP("Command: %s with a length of %d \n", inputBuffer, commandLength);

          if(commandLength >= 3) {

            if(inputBuffer[1]=='L' || inputBuffer[1]=='l'){
              for (int i=2; i<commandLength;i++ ){
                char inCharRead = inputBuffer[i];
                loRaCommandString += inCharRead;  // add it to the inputString:
              }
              Debug.LOOP("L Command Proccessing: %s \n", loRaCommandString.c_str());
              loRaCommandSubString = loRaCommandString.substring(0,commandLength+1);
              sendLoRaMessage(loRaCommandString);
              loRaCommandString="";
            }     
          }
        }
      

      ///***  Clear States and Reset for next command.  ***///
        stringComplete =false;
        autoComplete = false;
        inputBuffer[0] = '\0';
        inputBuffer[1] = '\0';

        // reset Local ESP Command Variables
        int espCommandFunction;
        loRaCommandString = "";

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
  // LoRa.receive();
  msgCount++;                           // increment message ID
  Debug.DBG_1("\n\n Sent Message of: %s \n\n", outgoing.c_str());

}
