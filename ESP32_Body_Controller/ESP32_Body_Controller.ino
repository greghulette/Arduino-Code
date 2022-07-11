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
    
    char inputBuffer[25];
    String inputString;         // a string to hold incoming data
    volatile boolean stringComplete  = false;      // whether the serial string is complete

    String autoInputString;         // a string to hold incoming data
    volatile boolean autoComplete    = false;    // whether an Auto command is setA
    int displayState;
    int typeState;
    int commandLength;
    int commandState;
    int paramVar = 9;

    int serialBoard;
    String serialStringCommand;

    uint32_t ESP_command[6]  = {0,0,0,0,0,0};
    int commandState     = 0;

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
  
  #define EN_BAUD_RATE 115200
  #define BL_BAUD_RATE 115200
  #define ST_BAUD_RATE 9600

  #define enSerial Serial1
  #define blSerial Serial2
  SoftwareSerial stSerial;

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
   blSerial.begin(BL_BAUD_RATE,SERIAL_8N1,RXBL,TXBL;
   stSerial.begin(ST_BAUD_RATE,SWSERIAL_8N1,RXST,TXST,false,95);

   Serial.println("\n\n\n-----------------------------------\nBooting up the Dome Controller");

  
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
          writeString(p->value());
        };
         if (paramVar == 1){
          DBG_1("Writing to enSerial\n"); 
          delay(100);     
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
      closeAllDoors(1);
      startUp = false;
      Serial.println("Startup");
    }
    if(Serial.available()){serialEvent();}

    if (stringComplete) {autoComplete=false;}
    if (stringComplete || autoComplete) {
      if(stringComplete) {inputString.toCharArray(inputBuffer, 25);inputString="";}
      else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 25);autoInputString="";}
      if( inputBuffer[0]=='D'     ||        // Door Designator
          inputBuffer[0]=='d'     ||        // Door Designator
          inputBuffer[0]=='E'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='e'     ||        // Command designatore for internal ESP functions
          inputBuffer[0]=='S'     ||        // Command for sending Serial Strings out Serial ports
          inputBuffer[0]=='s'               // Command for sending Serial Strings out Serial ports
        ){commandLength = strlen(inputBuffer);                                                                                  //  Determines length of command character array.

          if(commandLength >= 3) {
            if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {doorBoard = inputBuffer[1]-'0';}                                    
            if(inputBuffer[0]=='E' || inputBuffer[0]=='e') {commandState = (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');};       //  Converts 2 digit sequence sndentifier Characters to Integer
            if(inputBuffer[0]=='S' || inputBuffer[0]=='s') {
              serialBoard =  (inputBuffer[1]-'0')*10+(inputBuffer[2]-'0');
              for (int i=3; i<commandLength-2;i++ ){
                char inCharRead = inputBuffer[i];
                // add it to the inputString:
                serialStringCommand += inCharRead;
              }
              if (serialBoard == 'BL'){
              writeBlSerial(serialStringCommand)
              } else if (serialBoard == 'BC'){
                inputString = serialStringCommand;
                stringComplete = true; 
              } else if (serialBoard == 'ST'){
                writeStSerial(serialStringCommand);
              }
            };
            if(commandLength >= 4) {
              if(inputBuffer[0]=='D' || inputBuffer[0]=='d' ) {doorFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');}
            }
            else {
              typeState = -1;
            }
            if(commandLength >= 5) {
              if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {door = (inputBuffer[4]-'0')*10+(inputBuffer[5]-'0');}
            }

            if(inputBuffer[0]=='D' || inputBuffer[0]=='d') {
              D_command[0]   = '\0';                                                            // Flushes Array
              D_command[0] = doorFunction;
              D_command[1] = doorBoard;
              if(door>=0) {
                D_command[2] = door;
              }
            }

            if(inputBuffer[0]=='E' || inputBuffer[0] == 'e') {
              ESP_command[0]   = '\0';                                                            // Flushes Array
              ESP_command[0] = commandState;
            }
          }
        }

      ///***  Clear States and Reset for next command.  ***///
      stringComplete =false;
      autoComplete = false;
      inputBuffer[0] = '\0';
      int commandState;
      int door = -1;
      int doorFunction;
      int doorBoard;
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
      if((D_command[2] == 1 || D_command[2] == 2) && D_command[2] >= 11) {
        DBG("Incorrect Door Value Specified, Command Aborted!");
        D_command[0] = '\0';
      }
      else {
        switch (D_command[0]) {
          case 1: openDoor(D_command[1],D_command[2]);                                            break;
          case 2: closeDoor(D_command[1],D_command[2]);                                           break;
          case 3: openAllDoors(D_command[1]);                                                     break;
          case 4: closeAllDoors(D_command[1]);                                                    break;
          case 5: shortCircuit(D_Command[1]);                                                     break;
          case 6: allOpenClose(D_command[1]);                                                     break;
          case 7: allOpenCloseLong(D_command[1]);                                                 break;
          case 8: allFlutter(D_command[1]);                                                       break;
          case 9: allOpenCloseRepeat(D_command[1]);                                               break;
          case 10: panelWave(D_command[1]);                                                       break;
          case 11: panelWaveFast(D_command[1]);                                                   break;
          case 12: openCloseWave(D_command[1]);                                                   break;
          case 13: marchingAnts(D_command[1]);                                                    break;
          case 14: panelAlternate(D_command[1]);                                                  break;
          case 15: panelDance(D_command[1]);                                                      break;
          case 16: longDisco(D_command[1]);                                                       break;
          case 17: longHarlemShake(D_command[1]);                                                 break;
          case 98: closeAllDoors(3);                                                               break;
          case 99: closeAllDoors(3);                                                               break;
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

void openDoor(int servoBoard, int doorpos) {
  //Command: Dx01zz
  DBG("Open Specific Door\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG("Open Top Utility Arm\n");            SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, TOP_UTILITY_ARM);     break;
      case 2: DBG("Open Bottom Utility Arm\n");         SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, BOTTOM_UTILITY_ARM);  break;
      case 3: DBG("Open Large Left Door\n");            SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, LARGE_LEFT_DOOR);     break;
      case 4: DBG("Open Large Right Door\n");           SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, LARGE_RIGHT_DOOR);    break;
      case 5: DBG("Open Charge Bay Indicator Door\n");  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, CHARGE_BAY_DOOR);     break;
      case 6: DBG("Open Data Panel Door\n");            SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, DATA_PANEL_DOOR);     break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG("Open SMALL_PANEL_ONE\n");      writeEnSerial("N01DSD20101");  break;
      case 2: DBG("Open SMALL_PANEL_TWO\n");      writeEnSerial("N01DSD20102");  break;
      case 3: DBG("Open SMALL_PANEL_THREE\n");    writeEnSerial("N01DSD20103");  break;
      case 4: DBG("Open MEDIUM_PANEL_PAINTED\n"); writeEnSerial("N01DSD20104");  break;
      case 5: DBG("Open MEDIUM_PANEL_SILVER\n");  writeEnSerial("N01DSD20105");  break;
      case 6: DBG("Open BIG_PANEL\n");            writeEnSerial("N01DSD20106");  break;
      case 7: DBG("Open PIE_PANEL_ONE\n");        writeEnSerial("N01DSD20107");  break;
      case 8: DBG("Open PIE_PANEL_TWO\n");        writeEnSerial("N01DSD20108");  break;
      case 9: DBG("Open PIE_PANEL_THREE\n");      writeEnSerial("N01DSD20109");  break;
      case 10: DBG("Open PIE_PANEL_FOUR\n");      writeEnSerial("N01DSD20110");  break;
    }
  };
  D_command[0]   = '\0';
};


void closeDoor(int servoBoard, int doorpos) {
  // Command: Dx02zz
  DBG("Close Specific Door");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    switch(doorpos){
      case 1: DBG("Close Top Utility Arm\n");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, TOP_UTILITY_ARM);  break;
      case 2: DBG("Close Bottom Utility Arm\n");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, BOTTOM_UTILITY_ARM);  break;
      case 3: DBG("Close Large Left Door\n");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, LARGE_LEFT_DOOR);break;
      case 4: DBG("Close Large Right Door\n");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, LARGE_RIGHT_DOOR);  break;
      case 5: DBG("Close Charge Bay Indicator Door\n");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, CHARGE_BAY_DOOR);break;
      case 6: DBG("Close Data Panel Door\n");SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, DATA_PANEL_DOOR);  break;
    }
  };
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    switch (doorpos){
      case 1: DBG("Close SMALL_PANEL_ONE\n");      writeEnSerial("N01DSD20201");  break;
      case 2: DBG("Close SMALL_PANEL_TWO\n");      writeEnSerial("N01DSD20202");  break;
      case 3: DBG("Close SMALL_PANEL_THREE\n");    writeEnSerial("N01DSD20203");  break;
      case 4: DBG("Close MEDIUM_PANEL_PAINTED\n"); writeEnSerial("N01DSD20204");  break;
      case 5: DBG("Close MEDIUM_PANEL_SILVER\n");  writeEnSerial("N01DSD20205");  break;
      case 6: DBG("Close BIG_PANEL\n");            writeEnSerial("N01DSD20206");  break;
      case 7: DBG("Close PIE_PANEL_ON\nE");        writeEnSerial("N01DSD20207");  break;
      case 8: DBG("Close PIE_PANEL_TWO\n");        writeEnSerial("N01DSD20208");  break;
      case 9: DBG("Close PIE_PANEL_THREE\n");      writeEnSerial("N01DSD20209");  break;
      case 10: DBG("Close PIE_PANEL_FOUR\n");      writeEnSerial("N01DSD20210");  break;
    }
  };
  D_command[0]   = '\0';
}


void openAllDoors(int servoBoard) {
  // Command: Dx03
  DBG("Open all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, ALL_SERVOS_MASK);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    writeEnSerial("N01DSD203");
  };
  D_command[0] = '\0';
}

  
void closeAllDoors(int servoBoard) {
  // Command: Dx04
  DBG("Close all Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, ALL_SERVOS_MASK);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    writeEnSerial("N01DSD204");
  };
  D_command[0] = '\0';
}


void shortCircuit(int servoBoard) {
  // Command: Dx05
  // add sequence for this routine.  
}


void allOpenClose(int servoBoard){
  // Command: Dx06
  DBG("Open and Close All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
      SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpenClose, ALL_SERVOS_MASK);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    writeEnSerial("N01DSD206");
  };
  D_command[0]   = '\0';                                           
}


void allOpenCloseLong(int servoBoard){
  // Command: Dx07
  DBG("Open and Close Doors Long\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpenCloseLong, ALL_SERVOS_MASK);
  }
  if (servoBoard == 2 || servoBoard == 3 || servoBoard == 4){
    writeEnSerial("N01DSD207");
  };
  D_command[0]   = '\0';                                                 
}


void allFlutter(int servoBoard){
  // Command: Dx08
  DBG("Flutter All Doors\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllFlutter, ALL_SERVOS_MASK);
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    writeEnSerial("N01DSD208");
  };
  D_command[0]   = '\0';   
}


void allOpenCloseRepeat(int servoBoard){
  // Command: Dx09
  DBG("Open and Close All Doors Repeat\n");
  if (servoBoard == 1 || servoBoard == 3 || servoBoard == 4){
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllFOpenCloseRepeat, ALL_SERVOS_MASK);
  }
  if (servoBoard == 2 || servoBoard == 3  || servoBoard == 4){
    writeEnSerial("N01DSD209");
  };
  D_command[0]   = '\0';             
}


void panelWave(int servoBoard){
  // Command: Dx10
  DBG("Wave\n");
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK); break;
    case 2: writeEnSerial("N01DSD210"); break;
    case 3: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK);
            DelayCall::schedule([] {writeEnSerial("N01DSD210");}, 2000); break;
    case 4: writeEnSerial("N01DSD210");
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void panelWaveFast(int servoBoard){
  // Command: Dx11  
  DBG("Wave Fast\n");
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK); break;
    case 2: writeEnSerial("N01DSD211"); break;
    case 3: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWaveFast, ALL_SERVOS_MASK);
            DelayCall::schedule([] {writeEnSerial("N01DSD211");}, 2000); break;
    case 4: writeEnSerial("N01DSD212");
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void openCloseWave(int servoBoard) {
  // Command: Dx12
  DBG("Open Close Wave \n");
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK); break;
    case 2: writeEnSerial("N01DSD212"); break;
    case 3: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelOpenCloseWave, ALL_SERVOS_MASK);
            DelayCall::schedule([] {writeEnSerial("N01DSD5212");}, 2000); break;
    case 4: writeEnSerial("N01DSD212");
            DelayCall::schedule([] {SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelWave, ALL_SERVOS_MASK);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void marchingAnts(int servoBoard) {
  // Command: Dx13
  DBG("Marching Ants\n");
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK); break;
    case 2: writeEnSerial("N01DSD213"); break;
    case 3: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK);
          DelayCall::schedule([] {writeEnSerial("N01DSD5213");}, 2000); break;
    case 4: writeEnSerial("N01DSD213");
          DelayCall::schedule([] {SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelMarchingAnts, ALL_SERVOS_MASK);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void panelAlternate(int servoBoard) {
  // Command: Dx14
  DBG("Panel Alternate\n");
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK); break;
    case 2: writeEnSerial("N01DSD214"); break;
    case 3: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK);
          DelayCall::schedule([] {writeEnSerial("N01DSD5214");}, 2000); break;
    case 4: writeEnSerial("N01DSD214");
          DelayCall::schedule([] {SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAlternate, ALL_SERVOS_MASK);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}                                                            


void panelDance(int servoBoard) {
 // Command: Dx15
  DBG("Panel Dance\n");
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK); break;
    case 2: writeEnSerial("N01DSD215"); break;
    case 3: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK);
          DelayCall::schedule([] {writeEnSerial("N01DSD5215");}, 2000); break;
    case 4: writeEnSerial("N01DSD215");
          DelayCall::schedule([] {SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelDance, ALL_SERVOS_MASK);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void longDisco(int servoBoard) {
  // Command: Dx16
  DBG("Panel Dance Long\n");
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK); break;
    case 2: writeEnSerial("N01DSD216"); break;
    case 3: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK);
          DelayCall::schedule([] {writeEnSerial("N01DSD5216");}, 2000); break;
    case 4: writeEnSerial("N01DSD216");
          DelayCall::schedule([] {SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelLongDisco, ALL_SERVOS_MASK);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}


void longHarlemShake(int servoBoard) {
  // Command: Dx17
  DBG("Harlem Shake\n");
  switch(servoBoard){
    case 1: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK); break;
    case 2: writeEnSerial("N01DSD217"); break;
    case 3: SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK);
          DelayCall::schedule([] {writeEnSerial("N01DSD5217");}, 2000); break;
    case 4: writeEnSerial("N01DSD217");
          DelayCall::schedule([] {SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelLongHarlemShake, ALL_SERVOS_MASK);}, 3000); break;
  }
  D_command[0]   = '\0';                                             
}                                                       

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                               /////
/////                             Serial Communication Functions                              /////
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

void writeString(String stringData){
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
    DBG("Debugging Enabled \n");
  }
  else{
    Serial.println("Debugging Disabled");
  }
  ESP_command[0]   = '\0';
}


void toggleDebug1(){
  debugflag1 = !debugflag1;
  if (debugflag1 == 1){
    DBG("Parameter Debugging Enabled \n");
  }
  else{
    DBG("Parameter Debugging Disabled\n");
  }
  ESP_command[0]   = '\0';
}


//////////////////////////////////////////////////////////////////////
///*****             Misc. Functions                          *****///
//////////////////////////////////////////////////////////////////////

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


  
