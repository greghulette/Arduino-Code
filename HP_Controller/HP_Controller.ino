///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*****                                                                                                        *****///
///*****                                Created by Greg Hulette.  I                                             *****///
///*****   I started with the code from flthymcnsty from from which I used the basic command structure and      *****///
///*****  serial input method.  This code also relies on the ReelTwo library for all it's servo movements.     *****///
///*****                                                                                                       *****///
///*****                                     So exactly what does this all do.....?                            *****///
///*****                       - Receives commands via Serial or ESP-NOW                                       *****///
///*****                       - Controls the body servos                                                      *****///
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
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
#include <ElegantOTA.h>
#include <AsyncTCP.h>
#include <WiFi.h>

//Used for ESP-NOW / the WCB mesh
#include "esp_wifi.h"
#include <esp_now.h>
#include <WCB_Client.h>   // joins the WCB ESP-NOW mesh (replaces ETM_Droid)

//Used for the Status LED
#include <Adafruit_NeoPixel.h>

//Used for pin definition
#include "hp_controller_pin_map.h"

// Debug Functions  - Using my own library for this
#include <DebugR2.h>  //  https://github.com/greghulette/Arduino-Code/tree/main/libraries/DebugR2  Put these files in a folder called "DebugR2" in your libraries folder and restart the IDE

//ReelTwo libaries - Using my forked version of this libarary
#include <ReelTwo.h>
#include "core/DelayCall.h"
#include "dome/HoloLights.h"
#include "ServoDispatchDirect.h"
#include "ServoDispatchPCA9685.h"


// //Used for PC9685 - Servo Expansion Board
#include <Wire.h>





//////////////////////////////////////////////////////////////////////
///*****          Preferences/Items to change                 *****///
//////////////////////////////////////////////////////////////////////
 // ── WCB mesh credentials (WCB_Client) ───────────────────────────────────────
 // These MUST match your live WCB system.  Query any WCB over USB: ?WCBM ?WCBP ?WCBQ
  String        ESPNOWPASSWORD = "GregsAstromech";  // ?WCBP — WCB network password (set to match!)
  const uint8_t WCB_MAC_OCT2   = 0x00;              // ?WCBM — 2nd MAC octet   TODO: set for your system
  const uint8_t WCB_MAC_OCT3   = 0x00;              // ?WCBM — 3rd MAC octet   TODO: set for your system
  const uint8_t WCB_QUANTITY   = 8;                 // ?WCBQ — total WCBs      TODO: set for your system
  const uint8_t WCB_DEVICE_ID  = 8;                 // this board = HP = W8 on the mesh

  // R2 Control Network Details for OTA
  const char* ssid = "R2D2_Control_Network";
  const char* password =  "astromech";

  //Enables status tracking on the LoRa Droid
  bool STATUS_TRACKING = 1;
  
    // Keepalive timer to send status messages to the Kill Switch (Droid)
  int keepAliveDuration= 4000;  // 4 seconds

// used to sync timing with the dome controller better, allowing time for the ESP-NOW messages to travel to the peers
// Change this to work with how your droid performs
  int defaultESPNOWSendDuration = 50;  

  // Serial Baud Rates
  #define SERIAL1_BAUD_RATE 115200







//////////////////////////////////////////////////////////////////////
///*****        Command Varaiables, Containers & Flags        *****///
//////////////////////////////////////////////////////////////////////
  String HOSTNAME = "HP Controller";
  
  char inputBuffer[100];
  String inputString;         // a string to hold incoming data

  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA
  
  int commandLength;
  
  String serialStringCommand;
  String serialPort;
  String serialSubStringCommand;

  uint32_t Local_Command[6]  = {0,0,0,0,0,0};
  int localCommandFunction = 0;

  String ESPNOWStringCommand;
  String ESPNOWTarget;
  String ESPNOWSubStringCommand;

  String HPStringCommand;
  String HPFullStringCommand;

  debugClass Debug;
  String debugInputIdentifier ="";

  
  
  
  
  
  
  
  //////////////////////////////////////////////////////////////////////
  ///*****       Startup and Loop Variables                     *****///
  //////////////////////////////////////////////////////////////////////
  
  boolean startUp = true;
  boolean isStartUp = true;
  
  //Main Loop Timers
  unsigned long mainLoopTime;
  unsigned long MLMillis;
  byte mainLoopDelayVar = 5;


///////////////////////////////////////////////////////////////////////
  ///*****                Status Variables                     *****///
  /////////////////////////////////////////////////////////////////////

  unsigned long keepAliveMillis;

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  //////////////////////////////////////////////////////////////////
  ///******       Serial Ports Definitions                  *****///
  //////////////////////////////////////////////////////////////////

  #define s1Serial Serial2







//////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////

//Droid Remote ESP          192.168.4.101   
//Droid Gateway ESP         192.168.4.108   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Controller ESP       192.168.4.109   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Body Servo ESP            192.168.4.110   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Controller ESP       192.168.4.111   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Dome Plate Controller ESP 192.168.4.112   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//HP Controller ESP         192.168.4.113   (Only used for OTA, Remote LoRa ESP must be on and close to Droid)
//Droid Raspberry Pi        192.168.4.114
//Remote Raspberry Pi       192.168.4.115
//Developer Laptop          192.168.4.125
  
  // IP Address config of local ESP
  IPAddress local_IP(192,168,4,113);
  IPAddress subnet(255,255,255,0);
  IPAddress gateway(192,168,4,101);
  
  uint8_t oldLocalMACAddress[] = {0x24, 0x0A, 0xC4, 0xED, 0x30, 0x16};    //used when connecting to WiFi for OTA

  AsyncWebServer server(80);
  

//////////////////////////////////////////////////////////////////////
///*****            Status LED Variables and settings       *****///
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

  Adafruit_NeoPixel ESP_LED = Adafruit_NeoPixel(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

HoloLights frontHolo(25, HoloLights::kRGB);		// PIN 25
HoloLights rearHolo(18, HoloLights::kRGB);			// PIN 18
HoloLights topHolo(16, HoloLights::kRGB);			// PIN 16



































































/////////////////////////////////////////////////////////////////////////
///*****                  ESP NOW Set Up                         *****///
/////////////////////////////////////////////////////////////////////////

// ESP-NOW addressing + packet format are handled by WCB_Client (see below).

// Define variables to store commands to be sent
  String  senderID;
  String  targetID;
  bool    commandIncluded;
  String  command;
  uint32_t SuccessCounter = 0;
  uint32_t FailureCounter = 0;

// Define variables to store incoming commands
  String  incomingPassword;
  String  incomingTargetID;  
  String  incomingSenderID;
  bool    incomingCommandIncluded;
  String  incomingCommand;
  












// Variable to store if sending data was successful
  String success;

// (ESP-NOW packet format is internal to WCB_Client — these legacy send/recv
//  String vars below are now unused and can be pruned later.)




















// ── WCB mesh client — replaces the ETM_Droid send/recv layer below ──────────
// This board is device W8 (HP) on the WCB ESP-NOW mesh. WCB_Client handles all
// ESP-NOW init, heartbeats, online/offline tracking, and reliable delivery.
bool wcbReady = false;   // set from wcb.begin(); guards update()/send()

// Forward declarations (defined just below) — the client is built with them.
void onWcbCommand(uint8_t senderID, const char* command);
void onWcbStatus (uint8_t wcbID,    bool online);

WCB_Client wcb(WCB_MAC_OCT2, WCB_MAC_OCT3, ESPNOWPASSWORD.c_str(),
               WCB_QUANTITY, WCB_DEVICE_ID, onWcbCommand, onWcbStatus);

// Legacy 2-char board aliases → WCB mesh IDs (the shared alias matrix). Lets the
// existing command syntax (e.g. ":EDC<cmd>") keep addressing boards by name.
uint8_t wcbIdFromAlias(const String& a) {
  if (a == "BC") return 2;                     // Body Controller
  if (a == "DG") return 3;                     // Droid Gateway
  if (a == "DP") return 5;                     // Dome Plate Controller
  if (a == "DC") return 7;                     // Dome Controller
  if (a == "HP") return 8;                     // HP Controller (self)
  if (a == "NC" || a == "NAVI") return 20;     // NaviCore (special peer)
  return 0;                                    // unknown / not on the mesh
}

// Fired by WCB_Client when a WCB (or NaviCore) sends us a command or broadcasts.
// Runs on the ESP-NOW RX task (Core 0) — keep it light. We only copy the string
// (operator= copies before the library reuses its buffer) and set the flag; the
// main loop picks it up. The mesh already routed it here, so no target check.
void onWcbCommand(uint8_t senderID, const char* command) {
  inputString    = command;
  stringComplete = true;
  Debug.ESPNOW("[WCB] RX from W%d: %s\n", senderID, command);
}

// Fired when a WCB comes online / goes offline (library tracks it internally).
void onWcbStatus(uint8_t wcbID, bool online) {
  Debug.ESPNOW("[WCB] W%d is now %s\n", wcbID, online ? "ONLINE" : "OFFLINE");
}

// (ETM_Droid OnDataSent / OnDataRecv / processESPNOWIncomingMessage removed —
//  WCB_Client now handles TX/RX and delivers commands via onWcbCommand() above.)
//////////////////////////////////////////////////////////////////////
///       *****   Servo Values, Containers, Flags & Timers   *****////
//////////////////////////////////////////////////////////////////////

  char stringToSend[25];


  /////////////////////////////////////////////////////////////////////////
///*****              ReelTwo Servo Set Up                       *****///
/////////////////////////////////////////////////////////////////////////
#define HOLO_HSERVO        0x1000
#define HOLO_VSERVO        0x2000

// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.

//     Pin  Close Pos, Open Pos,  Group ID  (Change the Close and Open to your Droids actual limits)
const ServoSettings servoSettings[] PROGMEM = {
    { 1,   1100, 1800, HOLO_HSERVO },  /* 0: Vertical front holo */
    { 2,   1000, 1700, HOLO_VSERVO },  /* 1: Horizontal front holo */
    { 3,   1500, 2100, HOLO_HSERVO },  /* 2: horizontal top holo */
    { 4,   1200, 1900, HOLO_VSERVO },  /* 3: vertical top holo */
    { 5,   1850, 1200, HOLO_VSERVO },  /* 4: vertical rear holo */
    { 6,   1300, 2100, HOLO_HSERVO },  /* 5: horizontal rear holo */
  };

ServoDispatchPCA9685<SizeOfArray(servoSettings)> servoDispatch(servoSettings);


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////                                                                                       /////////     
/////////                             Start OF FUNCTIONS                                        /////////
/////////                                                                                       /////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
///*****   ColorWipe Function for Status LED                  *****///
//////////////////////////////////////////////////////////////////////
void colorWipeStatus(String statusled, uint32_t c, int brightness) {
  if(statusled == "ES"){
    ESP_LED.setBrightness(brightness);
    ESP_LED.setPixelColor(0, c);
    ESP_LED.show();
  } 
  else{Debug.DBG("No LED was chosen \n");}
};

//////////////////////////////////////////////////////////////////////
///*****    Send Keepalive Messages for Status                *****///
//////////////////////////////////////////////////////////////////////

void keepAlive(){
  // Superseded by WCB_Client heartbeats — the library announces this board to the
  // whole mesh, and the Droid Gateway (W3) learns our online/offline state from
  // its own WCB_Client status callback. No manual keepalive send is needed.
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                              Communication Functions                                    /////
/////                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/*//////////////////////////////////////////////////////////////////////
Turns on Wifi and enables OTA.  It will connect to the Kill Switch Remotes's
WiFi network to allow the uploading of sktches(.bin files) via the OTA process. 
It does not produce it's own WiFi network.  Once enables, a reboot is
required to regain ESP-NOW functionality.
*///////////////////////////////////////////////////////////////////////
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
    request->send(200, "text/plain", "Please go to http://192.168.4.112/update to upload file");
  });
  
  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

  Local_Command[0]   = '\0';
};

/////////////////////////////////////////////////////////
///*****          Serial Event Function          *****///
/////////////////////////////////////////////////////////                                  

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  Debug.SERIAL_EVENT("USB Serial Input: %s \n",inputString);
};

void s1SerialEvent() {
  while (s1Serial.available()) {
    char inChar = (char)s1Serial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
  Debug.SERIAL_EVENT("Serial 1 Input: %s \n",inputString);
};


//      /////////////////////////////////////////////////////////
//      ///*****          Serial Write Function          *****///
//      /////////////////////////////////////////////////////////

void writeSerialString(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++)
  {
    Serial.write(completeString[i]);
  }
};

void writes1Serial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++)
  {
    s1Serial.write(completeString[i]);
  }
    Debug.SERIAL_EVENT("Writing to Serial 1\n");

};


//////////////////////////////////////////////////////////////////////
///*****             ESP-NOW Functions                        *****///
//////////////////////////////////////////////////////////////////////

void sendESPNOWCommand(String starget, String scomm) {
  if (!wcbReady) return;                          // mesh not up — nothing to send on
  if (starget == "BR") {                          // broadcast to every WCB
    wcb.broadcast(scomm.c_str());
    Debug.ESPNOW("[WCB] Broadcast: %s\n", scomm.c_str());
    return;
  }
  uint8_t id = wcbIdFromAlias(starget);
  if (id == 0) {
    Debug.ESPNOW("[WCB] Unknown target alias: %s\n", starget.c_str());
    return;
  }
  wcb.send(id, scomm.c_str());                    // ensured (ACK-tracked) unicast
  Debug.ESPNOW("[WCB] Sent to W%d: %s\n", id, scomm.c_str());
}


 /*   * //////////////////////////////////////////////////////////////////////
      *             Command structure for HPs                      
      * //////////////////////////////////////////////////////////////////////
      * 
      * Command Prefix: :H
      *
      * DT##C or DT##CS or DT##R or DT##P
      *
      * D - the HP designator
      *     F - Front HP
      *     R - Rear HP
      *     T - Top HP
      *     D - Radar Eye
      *     O - Other HP
      *     A - All 3 HPs
      *     X - Front & Rear HPs
      *     Y - Front & Top HPs
      *     Z - Rear & Top HPs
      *     S - Sequences (See Below)
      * 
      * T - the Sequence Type is either 0-Led Fuctions and 1-Servo Functions
      * 
      *     ## - the Sequence Value including leading zero if necessary, ie sequence 3 is 03
      * 
      *     C - (Optional), the Color integer value from list below:
      *         Basic Color Integer Values
      *             1 = Red
      *             2 = Yellow
      *             3 = Green
      *             4 = Cyan (Aqua)
      *             5 = Blue
      *             6 = Magenta
      *             7 = Orange
      *             8 = Purple
      *             9 = White
      *             0 = Random
      * 
      *     S - (Optional), Speed setting integer for the Dim Pulse LED function below (0-9)
      * 
      *     R - (Optional), Random State for clearing LED displays
      *         Random State Integer Values
      *             1 = Use Default Sequences
      *             2 = Use Random Sequences
      * 
      *     P - (Optional), the Position integer value from list below:
      *         Preset Position Integer Values
      *             0 = Down
      *             1 = Center
      *             2 = Up
      *             3 = Left
      *             4 = Upper Left
      *             5 = Lower Left
      *             6 = Right
      *             7 = Upper Right
      *             8 = Lower Right
      * 
      *     D001    - Leia Sequence, Random shades of blue to mimic Leia Hologram
      *     D002C   - Color Projector Sequence, Like Leia above but using color command value
      *     D003CS  - Dim Pulse Sequence, Color slowly pulses on and off
      *     D004C   - Cycle Sequence, using color command value
      *     D005C   - Toggles Color, Simply sets LEDs tp solid color value
      *     D006    - Rainbow Sequence
      *     D007C   - Short Circuit, Led flashes on and off with interval slowing over time
      *     D096    - Clears LED, Disables Auto LED Sequence & "Off Color"
      *     D0971   - Clears LED, Enables Auto LED Sequence,Enables Default Sequences, Disables "Off Color"
      *     D0972   - Clears LED, Enables Auto LED Sequence,Enables Random Sequences, Disables "Off Color"
      *     D098    - Clears LED, Disables Auto LED Sequence, enables "Off Color"
      *     D0991   - Clears LED, Enables Auto LED Sequence,Enables Default Sequences, Enables "Off Color"
      *     D0992   - Clears LED, Enables Auto LED Sequence,Enables Random Sequences, Enables "Off Color"
      * 
      *     D101P   - Sends HP to a Preset Position
      *     D102    - Enables RC Control on HP (Left/Right)
      *     D103    - Enables RC Control on HP (Up/Down)
      *     D104    - Sends HP to a Random Position
      *     D105    - Wags HP Left/Right 5 times
      *     D106    - Wags HP Up/Down 5 times
      *     D198    - Disables Auto HP Twitch
      *     D199    - Enables Auto HP Twitch
      * 
      *       S1    - Leia Mode (Front HP in Down Position, Leia LED Sequence, all other HPs disabled)*
      *       S2    - Play R2 Cartoon [OLED]
      *       S3    - Play Deathstar Plans movie [OLED]
      *       S4    - Clear all LEDs, Disable Auto HP Twitch, Disable Auto LED Sequence, Disables Off Color
      *       S5    - Clear all LEDs, Enable Auto HP Twitch, Enable Auto LED Sequence (w/ default seqs.),
      *               Disables Off
      *       S9    - Clear all LEDs, Enable Auto HP Twitch, Enable Auto LED Sequence (w/ random seqs.),
      *               Disables Off Color
      *       S7    - Clear all LEDs, Disable Auto HP Twitch, Disable Auto LED Sequence, Enables Off Color
      *       S8    - Clear all LEDs, Enable Auto HP Twitch, Enable Auto LED Sequence (w/ default seqs.),
      *               Enables Off Color
      *       S9    - Clear all LEDs, Enable Auto HP Twitch, Enable Auto LED Sequence (w/ random seqs.),
      *               Enables Off Color
      * 
      * Runtime values can be added to any command string by appending a pipe (|) followed by a
      * numeric value indicating the desired time in seconds you wish the sequence to run.
      * 
      * e.g.  A007|25 would run the Rainbow Sequence on all 3 HPs for 25 seconds then clear each
      *       one, returning to the system's last known auto twitch mode.
      */

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                             Miscellaneous Functions                                           /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
///*****          Scan I2C for devices                        *****///
//////////////////////////////////////////////////////////////////////

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
  s1Serial.begin(SERIAL1_BAUD_RATE, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  
  delay(500);
  
  Serial.println("\n\n----------------------------------------");
  Serial.print("Booting up the ");Serial.println(HOSTNAME);
  Serial.println("----------------------------------------");



  //Initialize I2C for the Servo Expander Board
  Wire.begin();
  
  //Initialize the ReelTwo Library
  SetupEvent::ready();
  frontHolo.assignServos(&servoDispatch, 1, 2);
  topHolo.assignServos(&servoDispatch, 3, 4);
  rearHolo.assignServos(&servoDispatch, 5, 6);
  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

  // ── Join the WCB ESP-NOW mesh (replaces the old ETM_Droid bring-up) ──
  // WCB_Client handles WiFi/ESP-NOW init, this board's MAC, peer setup,
  // heartbeats, and online/offline tracking internally.
  wcbReady = wcb.begin();
  if (wcbReady) {
    Serial.printf("[WCB] On the mesh as W%d.\n", WCB_DEVICE_ID);
  } else {
    Serial.println("[WCB] begin() FAILED — running serial-only, not on the mesh.");
  }

  ESP_LED.begin();
  ESP_LED.show();
  colorWipeStatus("ES",red,10);
  

   	CommandEvent::process("HPA0026|20");


}   // end of setup

void loop(){
  if (wcbReady) wcb.update();   // mesh heartbeats + offline detection (was etmProcess)
  AnimatedEvent::process();

  if (millis() - MLMillis >= mainLoopDelayVar){
      MLMillis = millis();

    if(startUp) {
      startUp = false;
      Serial.print("Startup complete\nStarting main loop\n\n\n");
  colorWipeStatus("ES",blue,10);

    }  
    keepAlive();
    if(Serial.available()){serialEvent();}
    if(s1Serial.available()){s1SerialEvent();}

    if (stringComplete) {autoComplete=false;}
    if (stringComplete || autoComplete) {
      if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
        else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
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
                  case 3: connectWiFi();                                                                          break;
                  case 4: CommandEvent::process("HPA0012|20"); Local_Command[0]   = '\0'; break;  //reserved for future use
                  case 5: CommandEvent::process("HPA006|20"); Local_Command[0]   = '\0'; break;  //reserved for future use
                  case 6: CommandEvent::process("HPA007|20"); Local_Command[0]   = '\0'; break;  //reserved for future use
                  case 7: ; break;  //reserved for future use
                  case 8: ; break;  //reserved for future use                                                         break;  //reserved for future use
                  case 9: scan_i2c(); ;break;  //reserved for future use

                }
              }

        }else if (inputBuffer[0] == ':'){
       if(  inputBuffer[1]=='H' ||        // Command designator for Holo Projector functions
            inputBuffer[1]=='h' ||        // Command designator for Holo Projector functions
            inputBuffer[1]=='E' ||        // Command designator for ESP-NOW functions
            inputBuffer[1]=='e' ||        // Command designator for ESP-NOW functions
            inputBuffer[1]=='N' ||        // Command for Sending ESP-NOW Messag=es
            inputBuffer[1]=='n' ||        // Command for Sending ESP-NOW Messages
            inputBuffer[1]=='S' ||        // Command for sending Serial Strings out Serial ports
            inputBuffer[1]=='s'           // Command for sending Serial Strings out Serial ports


         ){commandLength = strlen(inputBuffer);                     //  Determines length of command character array.
            Debug.LOOP("Command: %s with a length of %d \n", inputBuffer, commandLength);
            if(commandLength >= 3) {
                if(inputBuffer[1]=='H' || inputBuffer[1]=='h') {
                  for (int i=2; i<=commandLength; i++){
                    char inCharRead = inputBuffer[i];
                    HPStringCommand += inCharRead;                   // add it to the inputString:
                  }
                  Debug.LOOP("\nFull Command Recieved: %s \n",HPStringCommand.c_str());
                  sprintf(stringToSend, "HP%s", HPStringCommand);
                  CommandEvent::process(stringToSend);
                  
                  // reset HP Variables
                  HPStringCommand = "";
                  HPFullStringCommand = "";                  
                  } 

                if(inputBuffer[1]=='E' || inputBuffer[1]=='e') {
                  for (int i=2; i<=commandLength; i++){
                    char inCharRead = inputBuffer[i];
                    ESPNOWStringCommand += inCharRead;                   // add it to the inputString:
                  }
                  Debug.LOOP("\nFull Command Recieved: %s \n",ESPNOWStringCommand.c_str());
                  ESPNOWTarget = ESPNOWStringCommand.substring(0,2);
                  Debug.LOOP("ESP NOW Target: %s\n", ESPNOWTarget.c_str());
                  ESPNOWSubStringCommand = ESPNOWStringCommand.substring(2,commandLength+1);
                  Debug.LOOP("Command to Forward: %s\n", ESPNOWSubStringCommand.c_str());
                  sendESPNOWCommand(ESPNOWTarget, ESPNOWSubStringCommand);
                  // reset ESP-NOW Variables
                  ESPNOWStringCommand = "";
                  ESPNOWSubStringCommand = "";
                  ESPNOWTarget = "";                  
                  } 
              if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
                for (int i=1; i<commandLength-1;i++ ){
                  char inCharRead = inputBuffer[i];
                  serialStringCommand += inCharRead;  // add it to the inputString:
                }
                serialPort = serialStringCommand.substring(0,2);
                serialSubStringCommand = serialStringCommand.substring(2,commandLength);
                Debug.LOOP("Serial Command: %s to Serial Port: %s\n", serialSubStringCommand.c_str(), serialPort);                
                if (serialPort == "BC"){
                  // writeBcSerial(serialSubStringCommand);
                } else if (serialPort == "FU"){
                  writes1Serial(serialSubStringCommand);
                } 
                
                serialStringCommand = "";
                serialSubStringCommand = "";
                serialPort = "";
              } 
            }
          }
        }
      ///***  Clear States and Reset for next command.  ***///
        stringComplete =false;
        autoComplete = false;
        inputBuffer[0] = '\0';
        
        // reset Local ESP Command Variables
        int localCommandFunction;


      // reset Door Variables
        int door = -1;
        int doorFunction;
        int doorBoard;
        int doorEasingMethod;
        uint32_t cVarSpeedMin;
        uint32_t cVarSpeedMax;
        uint32_t delayCallTime; 

      Debug.LOOP("command Proccessed\n");

    }


    if(Local_Command[0]){
      switch (Local_Command[0]){
        case 1: Serial.println(HOSTNAME);
                Local_Command[0]   = '\0';                                                                break;
        case 2: Serial.println("Resetting the ESP in 3 Seconds");
                DelayCall::schedule([] {ESP.restart();}, 3000) ;                                        
                Local_Command[0]   = '\0';                                                                break;
        case 3: connectWiFi();                                                                          break;
        case 4: ESP.restart();                                                                          break;
        case 5: break;
        case 6: break;
        case 7: break;
        case 8: break;
        case 9: scan_i2c();Local_Command[0]='\0';                                                         break;
        }
    }

    if(isStartUp) {
      isStartUp = false;
      delay(500);

    }
  }
}
