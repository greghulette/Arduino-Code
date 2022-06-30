

#define DEBUG
#include "ESPAsyncWebServer.h"
#include <AsyncElegantOTA.h>
#include <elegantWebpage.h>
#include <Hash.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <SoftwareSerial.h>

//reeltwo libaries
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
///*****                            Created by Greg Hulette.  I started with the code from flthymcnsty         *****///
///*****                                                                                                       *****///
///*****                                     So exactly what does this all do.....?                            *****///
///*****                       - Controls the Body servos                                                      *****///
///*****                       - Creates the WiFI network                                                      *****///
///*****                       - Sends Serial commands to the LED Controller and the Stealth Board             *****///
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

//     Pin  Min, ,Max,  Group ID
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
    
    char inputBuffer[25];
    String inputString;         // a string to hold incoming data
   volatile boolean stringComplete  = false;      // whether the serial string is complete

    String autoInputString;         // a string to hold incoming data
    volatile boolean autoComplete    = false;    // whether an Auto command is setA
    int displayState;
    int typeState;
    int commandLength;
    int paramVar = 9;

    uint32_t ESP_command[6]  = {0,0,0,0,0,0};
    int commandState     = 0;

    #ifdef DEBUG
      #define DPRINT(x)    Serial.print (x)
      #define DPRINTLN(x)  Serial.println (x)
    #else
      #define DPRINT(x)
      #define DPRINTLN(x) 
    #endif

  //////////////////////////////////////////////////////////////////////
  ///*****   Door Values, Containers, Flags & Timers   *****///
  //////////////////////////////////////////////////////////////////////
   int door = -1;
  // Door Command Container
   uint32_t D_command[6]  = {0,0,0,0,0,0};
   int doorState     = 0;
   int doorBoard = 0;
  // Door Counters
   long int Dcounts[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
   long int Dcount  = 0;
   long int D1count  = 0;
   long int Dpcount = 0;
   long int qwDuration = 800;
  // Door Timer
   unsigned long Dmillis;
   unsigned long Doorsmillis[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
   unsigned long D1millis;
   unsigned long Doors1millis[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
   unsigned long D2millis;
   unsigned long Doors2millis[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  // Door Flags
   boolean DaltToggle = true;
   boolean DWToggle   = false;
   boolean GaltToggle = true;

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

  #define RXen 15
  #define TXen 16 
  #define RXbc 25
  #define TXbc 26
  #define RXst 12
  #define TXst 14
  
  #define BAUD_RATE 9600
  SoftwareSerial enSerial;
  SoftwareSerial blSerial;
  SoftwareSerial stSerial;
  //////////////////////////////////////////////////////////////////////
  ///******      Arduino Mega Reset Pin Specific Setup          *****///
  //////////////////////////////////////////////////////////////////////

  #define RST 4

  //////////////////////////////////////////////////////////////////////
  ///******             WiFi Specific Setup                     *****///
  //////////////////////////////////////////////////////////////////////

//Raspberry Pi              192.168.4.100
//Body Controller ESP       192.168.4.101  ************
//Dome Controller ESP       192.168.4.102
//Periscope Controller ESP  192.168.4.103
//Stealth Controller ESP    192.168.4.104  (Probably not going to be different then Body Controller ESP IP)
//Dome Servo Controller     192.168.4.105  (Probably not going to be different then Dome Controller ESP IP)
//Body Servo Controller     192.168.4.106  (Probably not going to be different then Body Controller ESP IP)
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
int channel =  0;
int broadcastNetwork = 0;
int maxConnections = 8;

AsyncWebServer server(80);

void setup(){
  //Initialize the Serial Ports
  Serial.begin(9600);
   enSerial.begin(BAUD_RATE,SWSERIAL_8N1,RXen,TXen,false,95);
   blSerial.begin(BAUD_RATE,SWSERIAL_8N1,RXbc,TXbc,false,95);
   stSerial.begin(BAUD_RATE,SWSERIAL_8N1,RXst,TXst,false,95);

//  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
//  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("\n\n\n----------------------------------------");
  Serial.println("Booting up the Body LED/Servo Controller");

  
  
  //Configure the Reset Pins for the arduinoReset() function
  pinMode(4, OUTPUT);
  digitalWrite(4,HIGH);

  //Initialize I2C for the Servo Expander Board
  Wire.begin();
  
  //Initialize the ReelTwo Library
  SetupEvent::ready();

  //Initialize the Soft Access Point
  WiFi.mode(WIFI_AP);
    Serial.println(WiFi.softAP(ssid,password,6,0,8) ? "AP Ready" : "Failed!");
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
    Serial.println(paramsNr);                       // Variable for selecting which Serial port to send out
    for(int i=0;i<paramsNr;i++){                     //Loops through all the paramaters
         AsyncWebParameter* p = request->getParam(i);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////                                                                //////////////////////////        
//////////  These If statements choose where to send the commands         //////////////////////////
//////////  This way we can control multiple serial ports from one ESP32. //////////////////////////
//////////                                                                //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
        
    if ((p->name())== "param0" & (p->value()) == "Serial0"){
//        Serial.println("Serial0 Chosen with If Statement");
        paramVar = 0;
        };
    if ((p->name())== "param0" & (p->value()) == "enSerial"){
//        Serial.println("Serial 1 Chosen with If Statement");
        paramVar = 1;
        };
    if ((p->name())== "param0" & (p->value()) == "blSerial"){
//      Serial.println("Serial 2 Chosen with If Statement");
          paramVar = 2;
    };
        if ((p->name())== "param0" & (p->value()) == "stSerial"){
//      Serial.println("Serial 2 Chosen with If Statement");
          paramVar = 3;
    };
     if ((p->name())== "param0" & (p->value()) == "ESP"){
//      Serial.println("ESP(Self) Chosen with If Statement");
          paramVar = 4;
    };
    if ((p->name())== "param0" & (p->value()) == "ArduinoReset"){
        Serial.println("Reset Only Arduino Chosen with If Statement");
          resetArduino(500);
        };
    if ((p->name())== "param0" & (p->value()) == "ESPReset"){
        Serial.println("Reset ESP and Arduino Chosen with If Statement");
        ESP.restart();
        };
        
        Serial.print("Param name: ");
        Serial.println(p->name());
        Serial.print("Param value: ");
        Serial.println(p->value());
        Serial.println(paramVar);
  
        if (paramVar == 0){
          Serial.println("Writing to Serial 0");      
          writeString(p->value());
        };
         if (paramVar == 1){
          Serial.println("Writing to enSerial"); 
          delay(100);     
         if ((p->name())== "param0" & (p->value()) == "EnSerial"){
            Serial.println("Skipping param 0 in the EspNowSerial Write");
          } 
          else {
            writeEnSerial(p->value());
          };
        } ;      
          if (paramVar == 2){
          Serial.println("Writing to blSerial");      
          writeBlSerial(p->value());
        };
        if (paramVar == 3){
          Serial.println("Writing to stSerial");      
          writeStSerial(p->value());
        };
         if (paramVar == 4){
          Serial.println("Executing on self");      
          inputString = (p->value());
          stringComplete = true;  
        };

        Serial.println("------");
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
      closeAllDoors(1);
      startUp = false;
      Serial.println("Startup");
   }
   if(Serial.available()){
   serialEvent();
   }
  if (stringComplete) {autoComplete=false;}
  if (stringComplete || autoComplete) {
    if(stringComplete) {inputString.toCharArray(inputBuffer, 25);inputString="";}
     else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 25);autoInputString="";}
     if( inputBuffer[0]=='D'  ||       // Door Designator
         inputBuffer[0]=='d'  ||       // Door Designator
         inputBuffer[0]=='E'  ||       // Command designatore for internal ESP functions
         inputBuffer[0]=='e'           // Command designatore for internal ESP functions


         ) {
            commandLength = strlen(inputBuffer);                     //  Determines length of command character array.

            if(commandLength >= 3) {
               if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {doorBoard = inputBuffer[1]-'0';}  
               if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {commandState = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');};       //  Converts 2 Door Sequence Indentifier Characters to Integer
              if(commandLength >= 4) {
                if(inputBuffer[0]=='D' || inputBuffer[0]=='d' ) {doorState = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');}
                }
                else {
                     typeState = -1;
                }
                if(commandLength >= 5) {
                  if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {door = (inputBuffer[4]-'0')*10+(inputBuffer[5]-'0');}
                  }
//                 if(commandLength >= 6) {colorState2 = inputBuffer[5]-'0';}


                

               
                if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {
                  D_command[0]   = '\0';                                                            // Flushes Array
                  DaltToggle = true;
                  D_command[0] = doorState;
                  D_command[1] = doorBoard;
                  if(door>=0) {
                               D_command[2] = door;
                               Dcounts[door] = 0;
                  }
                  else {Dcount = 0;}
                }
                if(inputBuffer[0]=='E' || inputBuffer[0] == 'e') {
                  ESP_command[0]   = '\0';                                                            // Flushes Array
//                  DaltToggle = true;
                  ESP_command[0] = commandState;
//                  if(door>=0) {
//                   D_command[1] = commandState;
//                               Dcounts[door] = 0;
//                  }
//                  else {Dcount = 0;}
                }

              }
            }

      ///***  Clear States and Reset for next command.  ***///
       stringComplete =false;
       autoComplete = false;
       inputBuffer[0] = '\0';
       int displayState;
       int typeState;
       int speedState;
       int door = -1;
       int doorState;
       Serial.println("command Proccessed");

     }

  if(ESP_command[0]){
    switch (ESP_command[0]){
      case 1: Serial.println("Body ESP Controller");   
              ESP_command[0]   = '\0'; break;
      case 2: Serial.println("Resetting the ESP in 5 Seconds");
              DelayCall::schedule([] {ESP.restart();}, 5000);
              ESP_command[0]   = '\0'; break;
      case 3: writeEnSerial("DCE01"); 
              ESP_command[0]   = '\0'; break;
    }
  }

  if(D_command[0]) {
       if((D_command[2] == 1 || D_command[2] == 2) && D_command[2] >= 11) {
         //Serial.println("Incorrect Door Value Specified, Command Aborted!");
         D_command[0] = '\0';
//         Serial.println("wrong if");
       }
       else {
         switch (D_command[0]) {
           case 1:  openDoor(D_command[1],D_command[2]);                                             break;
            case 2:  closeDoor(D_command[1],D_command[2]);                                            break;
           case 3:  openAllDoors(D_command[1]);                                                     break;
           case 4:  closeAllDoors(D_command[1]);                                                    break;
           case 5:  alternateDoors();                                                   break;
           case 6:  cycleDoors();                                                       break;
           case 7:  waveAllDoors();                                                     break;
           case 8:  quickWaveAllDoors();                                                break;
           case 10: allOpenClose();                                                     break;
           case 11: allOpenCloseLong();                                                 break;
           case 12: allFlutter();                                                       break;
           case 13: allOpenCloseRepeat();                                               break;
           case 14: panelWave();                                                        break;
           case 15: panelWaveFast();                                                    break;
           case 16: openCloseWave();                                                    break;
           case 17: marchingAnts();                                                     break;
           case 18: panelAlternate();                                                   break;
           case 19: panelDance();                                                       break;
           case 20: longDisco();                                                        break;
           case 21: longHarlemShake();                                                  break;
//           case 22: ();                                                                 break;
//           case 23: ();                                                                 break;
           case 50: testESPNOW();                                                                 break;
//           case 98: closeAllDoors();                                                    break;
//           case 99: closeAllDoors();                                                    break;
           default: break;
          }
       }
     }
   
  if(isStartUp) {
        isStartUp = false;

        delay(500);

    }
  }
 }


 ///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                                       Door Functions                                          /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//


  void openDoor(int servoBoard, int doorpos) {
    Serial.println("Open Specific Door");
    if (servoBoard == 1 || servoBoard == 3){
      switch (doorpos){
       case 1: Serial.println("Open Top Utility Arm");            SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, TOP_UTILITY_ARM);     break;
       case 2: Serial.println("Open Bottom Utility Arm");         SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, BOTTOM_UTILITY_ARM);  break;
       case 3: Serial.println("Open Large Left Door");            SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, LARGE_LEFT_DOOR);     break;
       case 4: Serial.println("Open Large Right Door");           SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, LARGE_RIGHT_DOOR);    break;
       case 5: Serial.println("Open Charge Bay Indicator Door");  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, CHARGE_BAY_DOOR);     break;
       case 6: Serial.println("Open Data Panel Door");            SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, DATA_PANEL_DOOR);     break;
        }};
     if (servoBoard == 2 || servoBoard == 3){
      switch (doorpos){
       case 1: Serial.println("Open SMALL_PANEL_ONE");      writeEnSerial("S02DSD0101");  break;
       case 2: Serial.println("Open SMALL_PANEL_TWO");      writeEnSerial("S02DSD0102");  break;
       case 3: Serial.println("Open SMALL_PANEL_THREE");    writeEnSerial("S02DSD0103");  break;
       case 4: Serial.println("Open MEDIUM_PANEL_PAINTED"); writeEnSerial("S02DSD0104");  break;
       case 5: Serial.println("Open MEDIUM_PANEL_SILVER");  writeEnSerial("S02DSD0105");  break;
       case 6: Serial.println("Open BIG_PANEL");            writeEnSerial("S02DSD0106");  break;
       case 7: Serial.println("Open PIE_PANEL_ONE");        writeEnSerial("S02DSD0107");  break;
       case 8: Serial.println("Open PIE_PANEL_TWO");        writeEnSerial("S02DSD0108");  break;
       case 9: Serial.println("Open PIE_PANEL_THREE");      writeEnSerial("S02DSD0109");  break;
       case 10: Serial.println("Open PIE_PANEL_FOUR");      writeEnSerial("S02DSD0110");  break;
        }};
     D_command[0]   = '\0';
  };


  void closeDoor(int servoBoard, int doorpos) {
    Serial.println("Close Specific Door");
if (servoBoard == 1 || servoBoard == 3){
    switch(doorpos){
       case 1: Serial.println("Close Top Utility Arm");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, TOP_UTILITY_ARM);  break;
       case 2: Serial.println("Close Bottom Utility Arm");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, BOTTOM_UTILITY_ARM);  break;
       case 3: Serial.println("Close Large Left Door");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, LARGE_LEFT_DOOR);break;
       case 4: Serial.println("Close Large Right Door");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, LARGE_RIGHT_DOOR);  break;
       case 5: Serial.println("Close Charge Bay Indicator Door");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, CHARGE_BAY_DOOR);break;
       case 6: Serial.println("Close Data Panel Door");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, DATA_PANEL_DOOR);  break;
    }};
     if (servoBoard == 2 || servoBoard == 3){
      switch (doorpos){
       case 1: Serial.println("Close SMALL_PANEL_ONE");      writeEnSerial("S02DSD0201");  break;
       case 2: Serial.println("Close SMALL_PANEL_TWO");      writeEnSerial("S02DSD0202");  break;
       case 3: Serial.println("Close SMALL_PANEL_THREE");    writeEnSerial("S02DSD0203");  break;
       case 4: Serial.println("Close MEDIUM_PANEL_PAINTED"); writeEnSerial("S02DSD0204");  break;
       case 5: Serial.println("Close MEDIUM_PANEL_SILVER");  writeEnSerial("S02DSD0205");  break;
       case 6: Serial.println("Close BIG_PANEL");            writeEnSerial("S02DSD0206");  break;
       case 7: Serial.println("Close PIE_PANEL_ONE");        writeEnSerial("S02DSD0207");  break;
       case 8: Serial.println("Close PIE_PANEL_TWO");        writeEnSerial("S02DSD0208");  break;
       case 9: Serial.println("Close PIE_PANEL_THREE");      writeEnSerial("S02DSD0209");  break;
       case 10: Serial.println("Close PIE_PANEL_FOUR");      writeEnSerial("S02DSD0210");  break;
        }};
    D_command[0]   = '\0';
  }


  void openAllDoors(int servoBoard) {
        DPRINTLN("Open all Doors");

    if (servoBoard == 1 || servoBoard == 3){
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK);
    }
    if (servoBoard == 2 || servoBoard == 3){
      writeEnSerial("S02DSD03");
        };
//    DelayCall::schedule([] {sendESPNOWCommand("ESP","d03");}, 0);
    D_command[0] = '\0';
   }

  
  void closeAllDoors(int servoBoard) {
    DPRINTLN("Close all Doors");

    if (servoBoard == 1 || servoBoard == 3){
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK);
    }
    if (servoBoard == 2 || servoBoard == 3){
      writeEnSerial("S02DSD04");
        };
    D_command[0] = '\0';
  }

  void alternateDoors() {
    Serial.println("Alternate All Doors");
        SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK);
//    sendESPNOWCommand("ESP","d03");

    D_command[0]   = '\0';
  }

  void cycleDoors() {
    Serial.println("Cycle All Doors");
    D_command[0]   = '\0';
  }

  void waveAllDoors() {
    Serial.println("Open Doors 1 at a time");
    D_command[0]   = '\0';
  }

  void waveAllDoorsClose() {
    Serial.println("Close Doors 1 at a time");
    D_command[0]   = '\0';
  }

  void quickWaveAllDoors() {
    Serial.println("Open Doors 1 at a time");
    D_command[0]   = '\0';
  }
 

//
void shortCircuit(int count) {

}



  //////////////  Functions to call ReelTwo animations

  void allOpenClose(){
      Serial.println("Open and Close All Doors");
      SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK);
       D_command[0]   = '\0';                                           
      }
      
  void allOpenCloseLong(){
      Serial.println("Open and Close Doors Long");
      SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK);
      D_command[0]   = '\0';                                                 
      }
          
  void allFlutter(){
      Serial.println("Flutter All Doors");
      SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK);
//      sendESPNOWCommand("ESP","d12");
      D_command[0]   = '\0';   
      }
  void allOpenCloseRepeat(){
      Serial.println("Open and Close All Doors Repeat");
      SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK);
      D_command[0]   = '\0';             
             }
  void panelWave(){
       Serial.println("Wave");
       SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK);
       DelayCall::schedule([] {writeEnSerial("S02DSD14");}, 2000);
//       writeString1("S02DSD14");

       D_command[0]   = '\0';                                             
       }
  void panelWaveFast(){
       Serial.println("Wave Fast");
       SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK);
       D_command[0]   = '\0';                                             
       }
  void openCloseWave() {
       Serial.println("Open Close Wave ");
       SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK);
       D_command[0]   = '\0';                                             
       }                                          
 
  void marchingAnts() {
       Serial.println("Marching Ants");
       SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK);
       D_command[0]   = '\0';                                             
       }                                             
  void panelAlternate() {
       Serial.println("Panel Alternate");
       SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK);
       D_command[0]   = '\0';                                             
       }                                                            

  void panelDance() {
       Serial.println("Panel Dance");
       SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK);
       D_command[0]   = '\0';                                             
       }

  void longDisco() {
         Serial.println("Panel Dance");
         SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK);
         D_command[0]   = '\0';                                             
         }

  void longHarlemShake() {
         Serial.println("Panel Dance");
         SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK);
         D_command[0]   = '\0';                                             
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
          };

        };
               Serial.println(inputString);
      };

      void writeString(String stringData){
        String completeString = stringData + '\r';
        for (int i=0; i<completeString.length(); i++)
        {
          Serial.write(completeString[i]);
        };
      };

      void writeBlSerial(String stringData){
              String completeString = stringData + '\r';
              for (int i=0; i<completeString.length(); i++)
              {
               blSerial.write(completeString[i]);
              };
            };

      void writeEnSerial(String stringData){
        String completeString = stringData + '\r';
        for (int i=0; i<completeString.length(); i++)
        {
          enSerial.write(completeString[i]);
          delay(5);
        };
        Serial.println(String("String to Send over ESPNOW Serial: " + completeString));
      };

      void writeStSerial(String stringData){
        String completeString = stringData + '\r';
        for (int i=0; i<completeString.length(); i++)
        {
          stSerial.write(completeString[i]);
        };
      };

    void resetArduino(int delayperiod){
      Serial.println("Opening of reset function");
      digitalWrite(4,LOW);
      delay(delayperiod);
      digitalWrite(4,HIGH);
      Serial.println("reset witin function");
    //  paramVar = 0;
    
    }

//////////////////////////////////////////////////////////////////////
///*****             Test Functions                        *****///
//////////////////////////////////////////////////////////////////////
// 
//  void sendESPNOWCommand(String sdest,String scomm){
////    Serial.println("sendESPNOWCommand Function called");
//    destination = sdest;
//    command = scomm;
//    commandsToSend.dest = destination;
//    commandsToSend.comm = command;
//    // Send message via ESP-NOW
//    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &commandsToSend, sizeof(commandsToSend));
//   if (result == ESP_OK) {
//    Serial.println("Sent with success");
//    }
//    else {
//      Serial.println(result);
//      Serial.println("Error sending the data");
//    }
//   }
//
//  void parseESPNOWCommand(String idest, String icomm){
//    
//  }
AnimationPlayer player(servoSequencer);
  void testESPNOW(){
//    sendESPNOWCommand("ESP","d03");
//    Serial.println("testESPNOW Function called");
    D_command[0] = '\0';
    ANIMATION_PLAY_ONCE(player, WaveFullBody);

    };



  ANIMATION(WaveFullBody)
  {
    DO_START()
//    DO_SEQUENCE(SeqPanelWave,ALL_SERVOS_MASK)
//      {
DO_ONCE_AND_WAIT({SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK);
 ; }, 2100)

//    DO_ONCE_LABEL(looping, {
//        printf("Looping all over again\n");
//    })
//    DO_WHILE_SEQUENCE(looping)
    DO_ONCE_AND_WAIT({
//       sendESPNOWCommand("ESP","d14");
//        StealthCommand("tmprnd=60");
    }, 10)
    DO_END()
}
  
