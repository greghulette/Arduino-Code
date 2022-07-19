// Used for OTA
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>

//Used for WiFi
#include "esp_wifi.h"

//Used for PC9685 - Servo Expansion Board
#include <Wire.h>

// Used for Software Serial to allow more useful naming
#include <SoftwareSerial.h>

//ReelTwo libaries
//#define USE_DEBUG
//#define USE_SERVO_DEBUG
#include "ReelTwo.h"
#include "core/DelayCall.h"
#include "ServoDispatchPCA9685.h"
#include "ServoSequencer.h"
#include "core/Animation.h"

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

/////////////////////////////////////////////////////////////////////////
///*****              ReelTwo Servo Set Up                       *****///
/////////////////////////////////////////////////////////////////////////

#define TOP_UTILITY_ARM       0x0001 //b0000000001
#define BOTTOM_UTILITY_ARM    0x0002 //b0000000010
#define LARGE_LEFT_DOOR       0x0004 //b0000000100
#define LARGE_RIGHT_DOOR      0x0008 //b0000001000
#define CHARGE_BAY_DOOR       0x0010 //b0000010000
#define DATA_PANEL_DOOR       0x0020 //b0000100000

#define UTILITY_ARMS_MASK     (TOP_UTILITY_ARM|BOTTOM_UTILITY_ARM)
#define LARGE_DOORS_MASK      (LARGE_LEFT_DOOR|LARGE_RIGHT_DOOR)
#define SMALL_DOORS_MASK      (CHARGE_BAY_DOOR|DATA_PANEL_DOOR)
#define ALL_DOORS_MASK        (LARGE_DOORS_MASK|SMALL_DOORS_MASK)
#define ALL_SERVOS_MASK       (ALL_DOORS_MASK|UTILITY_ARMS_MASK)

// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.

//     Pin  Min, ,Max,  Group ID  (Change the Min and Max to your Droids actual limits)
const ServoSettings servoSettings[] PROGMEM = {
    { 1,  700, 2400, TOP_UTILITY_ARM },       /* 0: Top Utility Arm */
    { 2,  700, 2400, BOTTOM_UTILITY_ARM },    /* 1: Bottom Utility Arm */
    { 3,  700, 2400, LARGE_LEFT_DOOR },       /* 2: Large Left Door as viewing from looking at R2 */
    { 4,  700, 2400, LARGE_RIGHT_DOOR },      /* 3: Large Right door as viewing from looking at R2 */
    { 5,  700, 2400, CHARGE_BAY_DOOR },       /* 4: Charge Bay Inidicator Door*/
    { 6,  700, 2400, DATA_PANEL_DOOR }        /* 5: Data Panel Door */
    };

ServoDispatchPCA9685<SizeOfArray(servoSettings)> servoDispatch(servoSettings);
ServoSequencer servoSequencer(servoDispatch);

//////////////////////////////////////////////////////////////////////
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
  
  uint32_t ESP_command[6]  = {0,0,0,0,0,0};
  int espCommandFunction     = 0;

  int debugflag = 0;
  int debugflag1 = 0;  // Used for debugging params recieved from clients

  //////////////////////////////////////////////////////////////////////
  ///*****   Door Values, Containers, Flags & Timers   *****///
  //////////////////////////////////////////////////////////////////////

  int door = -1;
  // Door Command Container
  uint32_t D_command[6]  = {0,0,0,0,0,0};
  int doorFunction     = 0;
  int doorBoard = 0;
  int doorEasingMethod;
  uint32_t doorEasingDuration;

  char stringToSend[20];
  uint32_t servoMovementDurationInDelayCall;
  
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

  #define RXEN 15
  #define TXEN 16 
  #define RXBL 25
  #define TXBL 26
  #define RXST 12
  #define TXST 14
  
  #define enSerial Serial1
  #define blSerial Serial2
  SoftwareSerial stSerial;

  #define EN_BAUD_RATE 115200
  #define BL_BAUD_RATE 115200
  #define ST_BAUD_RATE 9600



  //////////////////////////////////////////////////////////////////////
  ///******      Arduino Mega Reset Pin Specific Setup          *****///
  //////////////////////////////////////////////////////////////////////

  #define RST 4

  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////

//Raspberry Pi              192.168.4.100
//Body Controller ESP       192.168.4.101   ************
//ESP-NOW Master ESP        192.168.4.110   (Only used for OTA)
//Dome Controller ESP       192.168.4.111   (Only used for OTA) 
//Periscope Controller ESP  192.168.4.112   (Only used for OTA)
//Remote                    192.168.4.107
//Developer Laptop          192.168.4.125

// IP Address config of local ESP
IPAddress local_IP(192,168,4,101);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,4,100);
uint8_t newMACAddress[] = {0x02, 0x00, 0xC0, 0xA8, 0x04, 0x65};


 ////R2 Control Network Details
const char* ssid = "R2D2_Control_Network";
const char* password =  "astromech";
int channel =  6;
int broadcastSSID = 0;  //0 for yes, 1 for no
int maxConnections = 8;

AsyncWebServer server(80);

void setup(){
  //Initialize the Serial Ports
  Serial.begin(115200);
  enSerial.begin(EN_BAUD_RATE,SERIAL_8N1,RXEN,TXEN);
  blSerial.begin(BL_BAUD_RATE,SERIAL_8N1,RXBL,TXBL);
  stSerial.begin(ST_BAUD_RATE,SWSERIAL_8N1,RXST,TXST,false,95);

  Serial.println("\n\n\n----------------------------------------");
  Serial.println("Booting up the Body ESP Controller");
  
  //Configure the Reset Pins for the arduinoReset() function
  pinMode(4, OUTPUT);
  digitalWrite(4,HIGH);

  //Initialize I2C for the Servo Expander Board
  Wire.begin();
  
  //Initialize the ReelTwo Library
  SetupEvent::ready();

  //Reserve the inputStrings
  inputString.reserve(100);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(100);

  //Initialize the Soft Access Point
  WiFi.mode(WIFI_AP);
  Serial.println(WiFi.softAP(ssid,password,channel,broadcastSSID,maxConnections) ? "AP Ready" : "Failed!");
  delay(200);
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "AP IP Configured" : "Failed!");
  delay(200);
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

  esp_wifi_set_mac(WIFI_IF_AP, &newMACAddress[0]);
  delay(2000);
  Serial.print("Local AP MAC address = ");
  Serial.println(WiFi.softAPmacAddress());
 
 //Setup the webpage and accept the GET requests, and parses the variables 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      
    int paramsNr = request->params();               // Gets the number of parameters sent
//    DBG_1("Parameter %i \n",paramsNr);                       // Variable for selecting which Serial port to send out
    for(int i=0;i<paramsNr;i++){                     //Loops through all the paramaters
      AsyncWebParameter* p = request->getParam(i);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////                                                                //////////////////////////        
//////////  These If statements choose where to send the commands         //////////////////////////
//////////  This way we can control multiple serial ports from one ESP32. //////////////////////////
//////////                                                                //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
        
    if ((p->name())== "param0" & (p->value()) == "Serial0"){
        DBG_1("Serial0 Chosen with If Statement\n");
        paramVar = 0;
        };
    if ((p->name())== "param0" & (p->value()) == "enSerial"){
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
        DBG_1("Param value: %s\n", (p->value()));
  
        if (paramVar == 0){
          DBG_1("Writing to Serial 0\n");      
          writeSerialString(p->value());
        };
        if (paramVar == 1){
          DBG_1("Writing to enSerial\n"); 
//          delay(100);     
        if ((p->name())== "param0" & (p->value()) == "EnSerial"){
            DBG_1("Skipping param 0 in the EspNowSerial Write\n");
          } 
          else {
            writeEnSerial(p->value());
          };
        } ;      
          if (paramVar == 2){
          DBG_1("Writing to blSerial\n");      
          writeBlSerial(p->value());
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
//        delay(50);
    }

    request->send(200, "text/plain", "message received");
  });
  
  //Enable Access-Control-Allow-Origin to mitigate errors from website polling
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA

  //Initialize the AsycWebServer
  server.begin();

  //Reset Arudino Mega
  resetArduino(500);

}

void loop(){
  if (millis() - MLMillis >= mainLoopDelayVar){
    MLMillis = millis();
    AnimatedEvent::process();
    if(startUp) {
      closeAllDoors(1,0,0);
      startUp = false;
      Serial.println("Startup");
    }
    if(Serial.available()){serialEvent();}
    if(blSerial.available()){serialBlEvent();}
    if(enSerial.available()){serialEnEvent();}
    if(stSerial.available()){serialStEvent();}
    
    if (stringComplete) {autoComplete=false;}
    if (stringComplete || autoComplete) {
      if(stringComplete) {inputString.toCharArray(inputBuffer, 100);inputString="";}
      else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 100);autoInputString="";}
      if( inputBuffer[0]=='D'     ||        // Door Designator
          inputBuffer[0]=='d'     ||        // Door Designator
          inputBuffer[0]=='E'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='e'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='S'     ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='s'               // Command for sending Serial Strings out Serial ports
        
        ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.

          if(commandLength >= 3) {
            if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {
              doorBoard = inputBuffer[1]-'0';
              doorFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');
              if (doorFunction == 1 || doorFunction == 2){
                door = (inputBuffer[4]-'0')*10+(inputBuffer[5]-'0');
                if(commandLength >= 8){
                  DBG("Door Function Called \n");
                  doorEasingMethod = (inputBuffer[6]-'0')*10+(inputBuffer[7]-'0');
                  doorEasingDuration = (inputBuffer[8]-'0')*1000+(inputBuffer[9]-'0')*100+(inputBuffer[10]-'0')*10+(inputBuffer[11]-'0');
                } else{
                  doorEasingMethod = 0;
                  doorEasingDuration = 0;
                }
              }
              else if (doorFunction != 1 || doorFunction != 2) {
                DBG("Other Door Function Called \n");
                if (commandLength >=6){
                  DBG("with Easing \n");
                  doorEasingMethod = (inputBuffer[4]-'0')*10+(inputBuffer[5]-'0');
                  doorEasingDuration = (inputBuffer[6]-'0')*1000+(inputBuffer[7]-'0')*100+(inputBuffer[8]-'0')*10+(inputBuffer[9]-'0');
                } else {
                  DBG("without Easing \n");
                  doorEasingMethod = 0;
                  doorEasingDuration = 0;
                }
              }
            }
            if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {
              espCommandFunction = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              };      
            if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
              serialPort =  (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              for (int i=3; i<commandLength-2;i++ ){
                char inCharRead = inputBuffer[i];
                serialStringCommand += inCharRead;  // add it to the inputString:
              }
              DBG("Serial Command: %s to Serial Port: %s\n", serialStringCommand, serialPort);
              if (serialPort == "BL"){
                writeBlSerial(serialStringCommand);
              } else if (serialPort == "EN"){
                writeEnSerial(serialStringCommand);
              } else if (serialPort == "ST"){
                writeStSerial(serialStringCommand);
              }
              serialStringCommand = "";
              serialPort = "";
            } 


            if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {
              D_command[0]   = '\0';                                                            // Flushes Array
              D_command[0] = doorFunction;
              D_command[1] = doorBoard;
                if(door>=0) {D_command[2] = door;}
              D_command[3] = doorEasingMethod;
              D_command[4] = doorEasingDuration;
              Serial.println(doorFunction);
              Serial.println(doorEasingMethod);
              Serial.println(doorEasingDuration);
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

        // reset Door Variables
        int door = -1;
        int doorFunction;
        int doorBoard;
        int doorEasingMethod;
        uint32_t doorEasingDuration;

      DBG("command Proccessed\n");
    }

    if(ESP_command[0]){
      switch (ESP_command[0]){
        case 1: Serial.println("Body ESP Controller");   
                ESP_command[0]   = '\0';                                                        break;
        case 2: Serial.println("Resetting the ESP in 3 Seconds");
                DelayCall::schedule([] {ESP.restart();}, 3000);
                ESP_command[0]   = '\0';                                                        break;
        case 3: break;  //reserved for commonality. Used for connecting to WiFi and enabling OTA on ESP-NOW Boards 
        case 4: break;  //reserved for future use
        case 5: break;  //reserved for future use
        case 6: break;  //reserved for future use
        case 7: break;  //reserved for future use
        case 8: break;  //reserved for future use
        case 9: break;  //reserved for future use
        case 10: toggleDebug();                                                                 break;
        case 11: toggleDebug1();                                                                break;

      }
    }

    if(D_command[0]) {
      if((D_command[0] == 1 || D_command[0] == 2) && D_command[1] >= 11) {
        DBG("Incorrect Door Value Specified, Command Aborted!");
        D_command[0] = '\0';
      }
      else {
        switch (D_command[0]) {
          case 1: openDoor(D_command[1],D_command[2],D_command[3],D_command[4]);          break;
          case 2: closeDoor(D_command[1],D_command[2],D_command[3],D_command[4]);         break;
          case 3: openAllDoors(D_command[1],D_command[3],D_command[4]);                   break;
          case 4: closeAllDoors(D_command[1],D_command[3],D_command[4]);                  break;
          case 5: shortCircuit(D_command[1],D_command[3],D_command[4]);                   break;
          case 6: allOpenClose(D_command[1],D_command[3],D_command[4]);                   break;
          case 7: allOpenCloseLong(D_command[1],D_command[3],D_command[4]);               break;
          case 8: allFlutter(D_command[1],D_command[3],D_command[4]);                     break;
          case 9: allOpenCloseRepeat(D_command[1],D_command[3],D_command[4]);             break;
          case 10: panelWave(D_command[1],D_command[3],D_command[4]);                     break;
          case 11: panelWaveFast(D_command[1],D_command[3],D_command[4]);                 break;
          case 12: openCloseWave(D_command[1],D_command[3],D_command[4]);                 break;
          case 13: marchingAnts(D_command[1],D_command[3],D_command[4]);                  break;
          case 14: panelAlternate(D_command[1],D_command[3],D_command[4]);                break;
          case 15: panelDance(D_command[1],D_command[3],D_command[4]);                    break;
          case 16: longDisco(D_command[1],D_command[3],D_command[4]);                     break;
          case 17: longHarlemShake(D_command[1],D_command[3],D_command[4]);               break;
          case 98: closeAllDoors(1,0,0);                                                  break;
          case 99: closeAllDoors(1,0,0);                                                  break;
          default: break;
        }
      }
    }
    
    if(isStartUp) {
      isStartUp = false;
      delay(500);
    }
  }
}  // end of main loop

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                                       Door Functions                                          /////
///////                                                                                               /////
///////                           Door Command Stucture: Dxyyzz                                       /////
///////                             D = Door Command                                                  /////
///////                             x = Servo Board                                                   /////
///////                               1 = Body Only                                                   /////
///////                               2 = Dome Only                                                   /////
///////                               3 = Both, starting with the body                                /////
///////                               4 = Both, starting with the dome                                /////
///////                             yy = Door Function                                                /////
///////                             zz = Door Specified (Only used for Dx01 & Dx02)                   /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void openDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t servoMovementDuration) {
  //Command: Dx01zz
  DBG("Open Specific Door\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch (doorpos){
      case 1: DBG("Open Top Utility Arm\n");            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, TOP_UTILITY_ARM, servoMovementDuration);     break;
      case 2: DBG("Open Bottom Utility Arm\n");         SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, BOTTOM_UTILITY_ARM, servoMovementDuration);  break;
      case 3: DBG("Open Large Left Door\n");            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, LARGE_LEFT_DOOR, servoMovementDuration);     break;
      case 4: DBG("Open Large Right Door\n");           SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, LARGE_RIGHT_DOOR, servoMovementDuration);    break;
      case 5: DBG("Open Charge Bay Indicator Door\n");  SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, CHARGE_BAY_DOOR, servoMovementDuration);     break;
      case 6: DBG("Open Data Panel Door\n");            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, DATA_PANEL_DOOR, servoMovementDuration);     break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG("Open SMALL_PANEL_ONE\n");      
              sprintf(stringToSend, "N01DSD20101%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 2: DBG("Open SMALL_PANEL_TWO\n");      
              sprintf(stringToSend, "N01DSD20102%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 3: DBG("Open SMALL_PANEL_THREE\n");    
              sprintf(stringToSend, "N01DSD20103%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 4: DBG("Open MEDIUM_PANEL_PAINTED\n"); 
              sprintf(stringToSend, "N01DSD20104%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 5: DBG("Open MEDIUM_PANEL_SILVER\n");  
              sprintf(stringToSend, "N01DSD20105%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 6: DBG("Open BIG_PANEL\n");            
              sprintf(stringToSend, "N01DSD20106%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 7: DBG("Open PIE_PANEL_ONE\n");         
              sprintf(stringToSend, "N01DSD20107%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 8: DBG("Open PIE_PANEL_TWO\n");        
              sprintf(stringToSend, "N01DSD20108%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 9: DBG("Open PIE_PANEL_THREE\n");      
              sprintf(stringToSend, "N01DSD20109%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 10: DBG("Open PIE_PANEL_FOUR\n");      
              sprintf(stringToSend, "N01DSD20110%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
    }
  };
  D_command[0]   = '\0';
};


void closeDoor(int servoBoard, int doorpos, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx02zz
  DBG("Close Specific Door");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    switch(doorpos){    
      case 1: DBG("Close Top Utility Arm\n");SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, TOP_UTILITY_ARM, servoMovementDuration);  break;
      case 2: DBG("Close Bottom Utility Arm\n");SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, BOTTOM_UTILITY_ARM, servoMovementDuration);  break;
      case 3: DBG("Close Large Left Door\n");SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, LARGE_LEFT_DOOR, servoMovementDuration);break;
      case 4: DBG("Close Large Right Door\n");SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, LARGE_RIGHT_DOOR, servoMovementDuration);  break;
      case 5: DBG("Close Charge Bay Indicator Door\n");SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, CHARGE_BAY_DOOR, servoMovementDuration);break;
      case 6: DBG("Close Data Panel Door\n");SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, DATA_PANEL_DOOR, servoMovementDuration);  break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG("Close SMALL_PANEL_ONE\n");     
              sprintf(stringToSend, "N01DSD20201%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 2: DBG("Close SMALL_PANEL_TWO\n");      
              sprintf(stringToSend, "N01DSD20202%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 3: DBG("Close SMALL_PANEL_THREE\n");   
              sprintf(stringToSend, "N01DSD20203%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 4: DBG("Close MEDIUM_PANEL_PAINTED\n"); 
              sprintf(stringToSend, "N01DSD20204%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 5: DBG("Close MEDIUM_PANEL_SILVER\n");  
              sprintf(stringToSend, "N01DSD20205%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 6: DBG("Close BIG_PANEL\n");            
              sprintf(stringToSend, "N01DSD20206%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 7: DBG("Close PIE_PANEL_ON\nE");        
              sprintf(stringToSend, "N01DSD20207%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 8: DBG("Close PIE_PANEL_TWO\n");        
              sprintf(stringToSend, "N01DSD20208%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 9: DBG("Close PIE_PANEL_THREE\n");      
              sprintf(stringToSend, "N01DSD20209%02d%04d", servoEasingMethod, servoMovementDuration);
              writeEnSerial(stringToSend);  break;
      case 10:  DBG("Close PIE_PANEL_FOUR\n");      
                sprintf(stringToSend, "N01DSD20210%02d%04d", servoEasingMethod, servoMovementDuration);
                writeEnSerial(stringToSend);  break;
    }
  };
  D_command[0]   = '\0';
}


void openAllDoors(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx03
  DBG("Open all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK, servoMovementDuration);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "N01DSD203%02d%04d", servoEasingMethod, servoMovementDuration);
    writeEnSerial(stringToSend);  
  };
  D_command[0] = '\0';
}

  
void closeAllDoors(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx04
  DBG("Close all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK, servoMovementDuration);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "N01DSD204%02d%04d", servoEasingMethod, servoMovementDuration);
    writeEnSerial(stringToSend);  
  };
  D_command[0] = '\0';
}


void shortCircuit(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx05
  // add sequence for this routine.  
}


void allOpenClose(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx06
  DBG("Open and Close All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
      setServoEasingMethod(servoEasingMethod);
      SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK, servoMovementDuration);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "N01DSD206%02d%04d", servoEasingMethod, servoMovementDuration);
    writeEnSerial(stringToSend);  
  };
  D_command[0]   = '\0';                                           
}


void allOpenCloseLong(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx07
  DBG("Open and Close Doors Long\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK, servoMovementDuration);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    sprintf(stringToSend, "N01DSD207%02d%04d", servoEasingMethod, servoMovementDuration);
    writeEnSerial(stringToSend); 
  };
  D_command[0]   = '\0';                                                 
}


void allFlutter(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx08
  DBG("Flutter All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK, servoMovementDuration);
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    sprintf(stringToSend, "N01DSD208%02d%04d", servoEasingMethod, servoMovementDuration);
    writeEnSerial(stringToSend); 
  };
  D_command[0]   = '\0';   
}


void allOpenCloseRepeat(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx09
  DBG("Open and Close All Doors Repeat\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    setServoEasingMethod(servoEasingMethod);
    SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK, servoMovementDuration);
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    sprintf(stringToSend, "N01DSD209%02d%04d", servoEasingMethod, servoMovementDuration);
    writeEnSerial(stringToSend); 
  };
  D_command[0]   = '\0';             
}


void panelWave(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx10
  DBG("Wave\n");
  servoMovementDurationInDelayCall = servoMovementDuration;
  sprintf(stringToSend, "N01DSD210%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 2: writeEnSerial(stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, servoMovementDuration);
            DelayCall::schedule([stringToSend] {writeEnSerial(stringToSend);}, 2000); break;
    case 4: writeEnSerial(stringToSend);
            DelayCall::schedule([servoMovementDurationInDelayCall] {SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, servoMovementDurationInDelayCall);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void panelWaveFast(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration){
  // Command: Dx11  
  DBG("Wave Fast\n");
  servoMovementDurationInDelayCall = servoMovementDuration;
  sprintf(stringToSend, "N01DSD211%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 2: writeEnSerial(stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK, servoMovementDuration);
            DelayCall::schedule([stringToSend] {writeEnSerial(stringToSend);}, 2000); break;
    case 4: writeEnSerial(stringToSend);
            DelayCall::schedule([servoMovementDurationInDelayCall] {SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK, servoMovementDurationInDelayCall);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void openCloseWave(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx12
  DBG("Open Close Wave \n");
  servoMovementDurationInDelayCall = servoMovementDuration;
  sprintf(stringToSend, "N01DSD212%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 2: writeEnSerial(stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, servoMovementDuration);
            DelayCall::schedule([stringToSend] {writeEnSerial(stringToSend);}, 2000); break;
    case 4: writeEnSerial(stringToSend);
            DelayCall::schedule([servoMovementDurationInDelayCall] {SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK, servoMovementDurationInDelayCall);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void marchingAnts(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx13
  DBG("Marching Ants\n");
  sprintf(stringToSend, "N01DSD213%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 2: writeEnSerial(stringToSend); break;
    case 3: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, servoMovementDuration);
            writeEnSerial(stringToSend); break;
    case 4: writeEnSerial(stringToSend); 
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK, servoMovementDuration); break;
  }
  D_command[0]   = '\0';                                             
}


void panelAlternate(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx14
  DBG("Panel Alternate\n");
  sprintf(stringToSend, "N01DSD214%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 2: writeEnSerial(stringToSend);  break;
    case 3: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, servoMovementDuration);
            writeEnSerial(stringToSend);  break;
    case 4: writeEnSerial(stringToSend);
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK, servoMovementDuration); break;
  }
  D_command[0]   = '\0';                                             
}                                                            


void panelDance(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
 // Command: Dx15
  DBG("Panel Dance\n");
  sprintf(stringToSend, "N01DSD215%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 2: writeEnSerial(stringToSend);  break;
    case 3: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, servoMovementDuration);
            writeEnSerial(stringToSend);   break;
    case 4: writeEnSerial(stringToSend); 
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK, servoMovementDuration);  break;
  }
  D_command[0]   = '\0';                                             
}


void longDisco(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx16
  DBG("Panel Dance Long\n");
  sprintf(stringToSend, "N01DSD216%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 2: writeEnSerial(stringToSend);  break;
    case 3: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, servoMovementDuration);
            writeEnSerial(stringToSend);    break;
    case 4: writeEnSerial(stringToSend); 
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK, servoMovementDuration);  break;
  }
  D_command[0]   = '\0';                                             
}


void longHarlemShake(int servoBoard, int servoEasingMethod, uint32_t servoMovementDuration) {
  // Command: Dx17
  DBG("Harlem Shake\n");
  sprintf(stringToSend, "N01DSD217%02d%04d", servoEasingMethod, servoMovementDuration);
  setServoEasingMethod(servoEasingMethod);
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, servoMovementDuration); break;
    case 2: writeEnSerial(stringToSend);  break;
    case 3: SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, servoMovementDuration);
            writeEnSerial(stringToSend);    break;
    case 4: writeEnSerial(stringToSend); 
            SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK, servoMovementDuration);  break;
  }
  D_command[0]   = '\0';                                             
}                                                       

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


void serialEnEvent() {
  while (enSerial.available()) {
    // get the new byte:
    char inChar = (char)enSerial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  DBG("InputString: %s \n",inputString);
};


void serialBlEvent() {
  while (blSerial.available()) {
    // get the new byte:
    char inChar = (char)blSerial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
      stringComplete = true;            // set a flag so the main loop can do something about it.
    };
  };
  DBG("InputString: %s \n",inputString);
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


void writeEnSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    enSerial.write(completeString[i]);
  };
  DBG("String to Send over ESPNOW Serial: %s \n" , completeString);
};


void writeStSerial(String stringData){
  String completeString = stringData + '\r';
  for (int i=0; i<completeString.length(); i++){
    stSerial.write(completeString[i]);
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

void DBG(char *format, ...) {
        if (!debugflag)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}


void DBG_1(char *format, ...) {
        if (!debugflag1)
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



//////////////////////////////////////////////////////////////////////
///*****    Resets Arduino Mega due to bug in my PCB          *****///
//////////////////////////////////////////////////////////////////////

void resetArduino(int delayperiod){
  DBG("Opening of reset function");
  digitalWrite(4,LOW);
  delay(delayperiod);
  digitalWrite(4,HIGH);
  DBG("reset witin function");
}

//////////////////////////////////////////////////////////////////////
///*****        Sets Servo Easing Method                      *****///
//////////////////////////////////////////////////////////////////////

void setServoEasingMethod(int easingMethod){
  switch(easingMethod){
    case 1: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::LinearInterpolation);    break;
    case 2: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseIn);        break;
    case 3: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseOut);       break;
    case 4: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseInOut);     break;
    case 5: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseIn);            break;
    case 6: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseOut);           break;
    case 7: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseInOut);         break;
    case 8: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseIn);          break;
    case 9: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseOut);         break;
    case 10: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseInOut);      break;
    case 11: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseIn);         break;
    case 12: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseOut);        break;
    case 13: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseInOut);      break;
    case 14: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseIn);            break;
    case 15: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseOut);           break;
    case 16: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseInOut);         break;
    case 17: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseIn);        break;
    case 18: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseOut);       break;
    case 19: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseInOut);     break;
    case 20: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseIn);     break;
    case 21: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseOut);    break;
    case 22: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseInOut);  break;
    case 23: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseIn);         break;
    case 24: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseOut);        break;
    case 25: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseInOut);      break;
    case 26: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseIn);            break;
    case 27: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseOut);           break;
    case 28: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseInOut);         break;
    case 29: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseIn);          break;
    case 30: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseOut);         break;
    case 31: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseInOut);       break;
  }
}

//////////////////////////////////////////////////////////////////////
///*****        Sets Servo Easing Method                      *****///
//////////////////////////////////////////////////////////////////////

void setServoEasingMethod(int easingMethod){
  switch(easingMethod){
    case 1: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::LinearInterpolation);    break;
    case 2: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseIn);        break;
    case 3: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseOut);       break;
    case 4: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuadraticEaseInOut);     break;
    case 5: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseIn);            break;
    case 6: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseOut);           break;
    case 7: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CubicEaseInOut);         break;
    case 8: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseIn);          break;
    case 9: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseOut);         break;
    case 10: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuarticEaseInOut);      break;
    case 11: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseIn);         break;
    case 12: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseOut);        break;
    case 13: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::QuinticEaseInOut);      break;
    case 14: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseIn);            break;
    case 15: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseOut);           break;
    case 16: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::SineEaseInOut);         break;
    case 17: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseIn);        break;
    case 18: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseOut);       break;
    case 19: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::CircularEaseInOut);     break;
    case 20: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseIn);     break;
    case 21: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseOut);    break;
    case 22: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ExponentialEaseInOut);  break;
    case 23: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseIn);         break;
    case 24: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseOut);        break;
    case 25: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::ElasticEaseInOut);      break;
    case 26: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseIn);            break;
    case 27: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseOut);           break;
    case 28: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BackEaseInOut);         break;
    case 29: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseIn);          break;
    case 30: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseOut);         break;
    case 31: servoDispatch.setServosEasingMethod(ALL_SERVOS_MASK, Easing::BounceEaseInOut);       break;
  }
}  
