// #include <SoftwareSerial.h>

#include "pie_panel_drawer_pin_map.h"

// Debug Functions  - Using my own library for this
#include <DebugR2.h>  //  https://github.com/greghulette/Arduino-Code/tree/main/libraries/DebugR2  Put these files in a folder called "DebugR2" in your libraries folder and restart the IDE

//ReelTwo libaries - Using my forked version of this libarary
#include <ReelTwo.h>
#include "core/DelayCall.h"
#include "ServoDispatchDirect.h"
#include "ServoSequencer.h"


// Used for Software Serial to allow more useful naming
#include <SoftwareSerial.h>


//  String HOSTNAME = "Pie Drawer Controller";
  
  char inputBuffer[10];
  String inputString;         // a string to hold incoming data

  volatile boolean stringComplete  = false;      // whether the serial string is complete
  String autoInputString;         // a string to hold incoming data
  volatile boolean autoComplete    = false;    // whether an Auto command is setA
  
  int commandLength;
  
  // String serialStringCommand;
  // String serialPort;
  // String serialSubStringCommand;

  uint32_t Local_Command[6]  = {0,0,0,0,0,0};
  int localCommandFunction = 0;



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





    //////////////////////////////////////////////////////////////////
  ///******       Serial Ports Definitions                  *****///
  //////////////////////////////////////////////////////////////////

  SoftwareSerial s1Serial(12,13);


/////////////////////////////////////////////////////////////////////
  ///*****   Door Values, Containers, Flags & Timers   *****///
  //////////////////////////////////////////////////////////////////////

  // int door = -1;
  // Door Command Container
  uint32_t D_command[7]  = {0,0,0,0,0,0,0};
  int doorFunction = 0;
  // int doorBoard = 0; 
  // int doorEasingMethod;
  // uint32_t cVarSpeedMin;
  // uint32_t cVarSpeedMax;
  // uint32_t doorEasingDuration;
  // uint32_t delayCallTime;

  // variables for individual functions
  // uint32_t varSpeedMin;
  // uint32_t varSpeedMax;
  // char stringToSend[25];
  // uint32_t fVarSpeedMin;
  // uint32_t fVarSpeedMax;

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
    { 10,  1400, 1600, RAISE_LOWER },       /* 0: Top Utility Arm 2350,675*/
    { 11,  1100, 2000, RETRACT }    /* 1: Bottom Utility Arm 1950,960*/

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



///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////                                                                                               /////
///////                                       Door Functions                                          /////
///////                                                                                               /////
///////              Door Command Stucture: Dxyyzzabbccccddddeeeee                                    /////
///////                 D = Door Command                                                              /////
///////                 x = Servo Board                                                               /////
///////                   1 = Body Only                                                               /////
///////                   2 = Dome Only                                                               /////
///////                   3 = Both, starting with the body                                            /////
///////                   4 = Both, starting with the dome                                            /////
///////                 yy = Door Function                                                            /////
///////                 zz = Door Specified (Only used for Dx01 & Dx02)                               /////
///////                 a = Specify easing/delayCall options (B,E,D)                                  /////
///////                   B = Use Easing and Delay Call                                               /////
///////                   E = Easing Only (if VarSpeed is not needed, don't input dddd)               /////
///////                   D = Delay Call Only (don't input bbccccdddd)                                /////
///////                 bb = Easing Method (only needed if B or E was selected)
///////                 cccc = Easing Method Duration (only needed if B or E was selected)
///////                 dddd = Easing Method VarSpeedMax(only needed if B or E was selected and VARSPEED wanted)
///////                 eeeee = Delay Call Duration (only needed if B or D was selected)
///////                                                                                               /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void openSlider() {
  // Command: Dx03
  Debug.SERVO("Open all Doors\n");
  Serial.println("Open");
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, RAISE_LOWER); 
  // D_command[0] = '\0';
};

void openSliderSingle() {
  // Command: Dx03
  Debug.SERVO("Open all Doors\n");
  Serial.println("Open");
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, RAISE_LOWER); 
  // D_command[0] = '\0';
};

void closeSlider() {
  // Command: Dx03
  Debug.SERVO("Open all Doors\n");
  Serial.println("Close");
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, RAISE_LOWER); 
  D_command[0] = '\0';
};

void closeSliderSingle() {
  // Command: Dx03
  Debug.SERVO("Open all Doors\n");
  Serial.println("Close");
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, RAISE_LOWER); 
  // D_command[0] = '\0';
};
unsigned long timeoutMillis;

void lowerPiePanelTimeout(int duration) {

  // inputString = ":D03";
  // stringComplete = true;

    Debug.SERVO("Open all Doors\n");
  Serial.println("Lower");
  // SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpenHold, RETRACT); 
  // Command: Dx03
// timeoutMillis = millis();
// if (millis() - timeoutMillis <= duration){
// timeoutMillis = millis();
// // SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, RETRACT); 
//               D_command[0] = 1;


// }
  Serial.println("Lower");
// SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, RETRACT); 
  // D_command[0] = '\0';
};

void lowerPiePanel() {
  // Command: Dx03
  Debug.SERVO("Open all Doors\n");
  Serial.println("Lower");
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, RETRACT); 
  // D_command[0] = '\0';
};

void raisePiePanel() {
  // Command: Dx03
  Debug.SERVO("Open all Doors\n");
  Serial.println("Raise");
  SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, RETRACT); 
  D_command[0] = '\0';
};

void completeSequence8sec(){
   SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawer8sec, ALL_SERVOS_MASK); 
  D_command[0] = '\0';
}
void completeSequence5sec(){
   SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawer5sec, ALL_SERVOS_MASK); 
  D_command[0] = '\0';
}

void completeSequence3sec(){
   SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawer3sec, ALL_SERVOS_MASK); 
  D_command[0] = '\0';
}

void completeSequence11sec(){
   SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawer11sec, ALL_SERVOS_MASK); 
  D_command[0] = '\0';
}

 void PieDrawerBounceSequence(){
  SEQUENCE_PLAY_ONCE(servoSequencer, PieDrawerBounce, ALL_SERVOS_MASK); 
  D_command[0] = '\0';


 }

void setup() {
  // put your setup code here, to run once:
  // openSlider(); 
  Serial.begin(9600);
  Serial.println("\n\n----------------------------------------");
  Serial.println("Booting up the Pie Drawer ");
  Serial.println("----------------------------------------");
    // s1Serial.begin(9600, SERIAL_8N1, TRIGGER_RX, TRIGGER_TX);
  s1Serial.begin(9600);  

    




  //Reserve the inputStrings
  inputString.reserve(10);                                                              // Reserve 100 bytes for the inputString:
  autoInputString.reserve(10);


  //Initialize the ReelTwo Library
  SetupEvent::ready();
  pinMode(PIE_LOWERED_LS, INPUT_PULLUP);
// digitalWrite(PIE_LOWERED_LS, HIGH);

}
int pls = 1;
void loop() {
//   // put your main code here, to run repeatedly:



    AnimatedEvent::process();

  if (millis() - MLMillis >= mainLoopDelayVar){
      MLMillis = millis();
        int ls = digitalRead(PIE_LOWERED_LS);
if (ls != pls){
    pls = ls;
      Serial.println(ls);
}
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
            Debug.LOOP("Command: %s with a length of %d \n", inputBuffer, commandLength);
            if(commandLength >= 3) {
              if(inputBuffer[1]=='D' || inputBuffer[1]=='d') {                                                            // specifies the overall door command
              // doorBoard = inputBuffer[2]-'0';                                                                           // Sets the board to call the command on.
              doorFunction = (inputBuffer[2]-'0')*10+(inputBuffer[3]-'0');                                              // Sets the door command to a specific function
            }   
 

              
              

            if(inputBuffer[1]=='D' || inputBuffer[1]=='d') {
              D_command[0]   = '\0';                                                            // Flushes Array
              D_command[0] = doorFunction;
              // D_command[1] = doorBoard;
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
        // int doorEasingMethod;
        // uint32_t cVarSpeedMin;
        // uint32_t cVarSpeedMax;
        // uint32_t delayCallTime; 

      Debug.LOOP("command Proccessed\n");

    }




    

    if(D_command[0]) {
      // if((D_command[0] == 1 || D_command[0] == 2) && D_command[1] >= 11) {
      //   Debug.LOOP("Incorrect Door Value Specified, Command Aborted!");
      //   D_command[0] = '\0';
      // }
      // else {
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






