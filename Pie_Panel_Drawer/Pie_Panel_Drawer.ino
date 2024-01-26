// #include <SoftwareSerial.h>

#include "pie_panel_drawer_pin_map.h"

//ReelTwo libaries - Using my forked version of this libarary
#include <ReelTwo.h>
#include "core/DelayCall.h"
#include "ServoDispatchDirect.h"
#include "ServoSequencer.h"
#include "ServoDispatchPCA9685.h"


// Used for Software Serial to allow more useful naming
#include <SoftwareSerial.h>


  
  char inputBuffer[10];
  String inputString;         // a string to hold incoming data

  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA
  
  int commandLength;
  

  uint32_t Local_Command[6]  = {0,0,0,0,0,0};
  int localCommandFunction = 0;



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





    //////////////////////////////////////////////////////////////////
  ///******       Serial Ports Definitions                  *****///
  //////////////////////////////////////////////////////////////////

  SoftwareSerial s1Serial(12,13);


/////////////////////////////////////////////////////////////////////
  ///*****   Door Values, Containers, Flags & Timers   *****///
  //////////////////////////////////////////////////////////////////////

  // Door Command Container
  uint32_t D_command[7]  = {0,0,0,0,0,0,0};
  int doorFunction = 0;


  /////////////////////////////////////////////////////////////////////////
///*****              ReelTwo Servo Set Up                       *****///
/////////////////////////////////////////////////////////////////////////

#define RAISE_LOWER           0x0001 //b000000000001
#define RETRACT               0x0002 //b000000000010


#define ALL_SERVOS_MASK       (RAISE_LOWER|RETRACT)
// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.

//     Pin,  Close Pos, Open Pos,  Group ID  (Change the Close and Open to your Droids actual limits)
const ServoSettings servoSettings[] PROGMEM = {
        { 25,  500, 2500, RETRACT },    //Slide the drawer
        { 2,  1000, 2000, RAISE_LOWER },       // Raise and Lower
  };

ServoDispatchDirect<SizeOfArray(servoSettings)> servoDispatch(servoSettings);
ServoSequencer servoSequencer(servoDispatch);


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
};

void s1SerialEvent() {
  while (s1Serial.available()) {
    char inChar = (char)s1Serial.read();
    inputString += inChar;
      if (inChar == '\r') {               // if the incoming character is a carriage return (\r)
        stringComplete = true;            // set a flag so the main loop can do something about it.
      }
  }
};


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

};



///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                                       Door Functions                                          /////
///////                                                                                               /////
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void openSlider() {
  // Command: :D01
  Serial.println("Open");
    // servoDispatch.moveServosTo(RETRACT, 3000, 1.0);
  // D_command[0] = '\0';

  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, RETRACT); 
};


void closeSlider() {
  // Command: :D02
  Serial.println("Close");
  // servoDispatch.moveServosTo(RETRACT, 3000, 0.0);
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, RETRACT); 
  D_command[0] = '\0';
};

void lowerPiePanel() {
  // Command: :D03
  Serial.println("Lower");
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, RAISE_LOWER); 
  // D_command[0] = '\0';
};

void raisePiePanel() {
  // Command: :D04
  Serial.println("Raise");
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, RAISE_LOWER); 
  D_command[0] = '\0';
};

void completeSequence8sec(){
  // Command: :D05
   SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawer8sec, ALL_SERVOS_MASK); 
  D_command[0] = '\0';
}

void completeSequence5sec(){
  // Command: :D06
     SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawer5sec, ALL_SERVOS_MASK); 
  D_command[0] = '\0';
}

void completeSequence3sec(){
  // Command: :D07
  SEQUENCE_PLAY_ONCE_SPEED(servoSequencer, PieDrawer3sec, ALL_SERVOS_MASK, 100); 
  D_command[0] = '\0';
}

void completeSequence11sec(){
  // Command: :D05
  SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawer11sec, ALL_SERVOS_MASK); 
  D_command[0] = '\0';
}

 void PieDrawerBounceSequence(){
  // Command: :D09
  SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawerBounce, ALL_SERVOS_MASK); 
  D_command[0] = '\0';


 }

 void stopSlider(){
    // servoDispatch.disable(25);
    servoSequencer.stop();
    D_command[0] = '\0';
 }


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("\n\n----------------------------------------");
  Serial.println("Booting up the Pie Drawer ");
  Serial.println("----------------------------------------");
  s1Serial.begin(9600);  

    

 pinMode(SLIDE_CLOSE_LS, INPUT);


  //Reserve the inputStrings
  inputString.reserve(10);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(10);


  //Initialize the ReelTwo Library
  SetupEvent::ready();

}
int pls = 1;
void loop() {
//   // put your main code here, to run repeatedly:
bool SLIDE_CLOSE_LS_STATUS = digitalRead(SLIDE_CLOSE_LS);
if (SLIDE_CLOSE_LS_STATUS == true){
  stopSlider();

  Serial.println("Switch Activated");
}


    AnimatedEvent::process();

  if (millis() - MLMillis >= mainLoopDelayVar){
      MLMillis = millis();
        int ls = digitalRead(PIE_LOWERED_LS);

    if(startUp) {
      startUp = false;
      Serial.print("Startup complete\nStarting main loop\n\n\n");
    }  
    if(Serial.available()){serialEvent();}
    if(s1Serial.available()){s1SerialEvent();}


    if (stringComplete) {autoComplete=false;}
    if (stringComplete || autoComplete) {
      if(stringComplete) {inputString.toCharArray(inputBuffer, 10);inputString="";}
        else if (autoComplete) {autoInputString.toCharArray(inputBuffer, 10);autoInputString="";}
      if (inputBuffer[0] == ':'){
       if(  inputBuffer[1]=='D' ||        // Door Designator
            inputBuffer[1]=='d'         // Door Designator

         ){commandLength = strlen(inputBuffer);                     //  Determines length of command character array.
            if(commandLength >= 3) {
              if(inputBuffer[1]=='D' || inputBuffer[1]=='d') {                                                            // specifies the overall door command
              doorFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');                                              // Sets the door command to a specific function
            }   
 

              
              

            if(inputBuffer[1]=='D' || inputBuffer[1]=='d') {
              D_command[0]   = '\0';                                                            // Flushes Array
              D_command[0] = doorFunction;
            }


            }
          }
        }
      ///***  Clear States and Reset for next command.  ***///
        stringComplete =false;
        autoComplete = false;
        inputBuffer[0] = '\0';
        
        // reset Local  Command Variables
        int localCommandFunction;


      // reset Door Variables
        int doorFunction;


    }




    

    if(D_command[0]) {
        switch (D_command[0]) {
        case 1: openSlider();                         break;
        case 2: closeSlider();                         break;
        case 3: lowerPiePanel();                         break;
        case 4: raisePiePanel();                         break;
        case 5: completeSequence8sec();                         break;
        case 6: completeSequence5sec();                         break;
        case 7: completeSequence3sec();                         break;
        case 8: completeSequence11sec();                         break;
        case 9: PieDrawerBounceSequence();                         break;
        default: break;
        }
      }
    // }
    if(isStartUp) {
      isStartUp = false;
      delay(500);
    }
  }
}






