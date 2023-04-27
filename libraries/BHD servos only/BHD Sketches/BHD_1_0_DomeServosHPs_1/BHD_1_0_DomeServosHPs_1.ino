// =======================================================================================
// ====================== BHD Dome Servos and HPs by BigHappyDude ========================
// =======================================================================================
//
// This is the dome servo and PWM HPs unit for the droid, it runs all dome door servos
// and HP servos and PWM HP lights.
// Special thanks to Bill Porter for the PS2X_Lib and Easy Transfer Libraries and to all 
// the guys on .net that gave me the idea especially danf
//
// IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT
// This has been written for an arduino Uno or Mega, please make sure all servo positions
// are found and edited before upload to your dome.
//
// The following pins are used on the uno:
//    I2C connection: A4+A5
//    HP LEDs: FrontHPPin = 9, RearHPPin = 10, TopHPPin = 11
//
// The Adafruit for the body is on 0x40 (default address)
//
// The list of commands is available in the BHDLists.xlsx spreadsheet, or on the cheatsheet
//
// NOTE: All I2C devices must share the same GND level.

#include <Wire.h>
#include <EasyTransferI2C.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servos.h>   

Servos Dome(0x40);
//  O-2 are 0 = front HP, 1 = Rear HP, 2 = Top HP
//  3-6 anticlockwise from Periscope
//  7-11 anticlockwise from front PSI

EasyTransferI2C ArduinoTransfer; 

struct RECEIVE_DATA_STRUCTURE{
  int Script;
};

RECEIVE_DATA_STRUCTURE Commands;

#define SlaveArduino 11  //define slave i2c address

int LoopScript = 0;
int PieDoorOpen[4] = {280, 260, 300, 267}; // open spots
int PieDoorShut[4] = {471, 416, 488, 450}; // close spots
//int PieDoorOpen[4] = {418, 360, 293, 267}; // james open spots
//int PieDoorShut[4] = {623, 581, 488, 453}; // james close spots
int LDoorOpen[5] ={270, 276, 241, 250, 260}; // open spots
int LDoorShut[5] ={450, 472, 404, 438, 452}; // close spots 
int timer = 0;
int HPTime = 0;
int DelayFactor = 30;
int DanceDelay = 600;
boolean LDoorOpenCheck[5];
boolean PieDoorOpenCheck[4];
boolean HPsOn = true;

int HPTimes[15] = {8, 11, 15, 18, 22, 26, 30, 32, 33, 36, 41, 44, 47, 49};
int HPSpot[3][5] = {600, 520, 445, 370, 290,   // 5 spots equally spaced in range of HP travel
                    580, 505, 430, 355, 280,
                    240, 325, 410, 495, 580};  // servo 3 is the other way up :)
                    
int FrontHPPin = 9;
int RearHPPin = 10;
int TopHPPin = 11;

int DelayTime=20;
unsigned char HPCounter=0;
int Brightness[12]={1, 2, 5, 9, 13, 20, 30, 45, 90, 120, 170, 255};
boolean FrontHPOnState=false;
boolean RearHPOnState=false;
boolean TopHPOnState=false;
int FrontHPCounter=0;
int RearHPCounter=0;
int TopHPCounter=0;
int FrontHPPosition=0;
int RearHPPosition=0;
int TopHPPosition=0;
int FrontHPWait=2;
int RearHPWait=1;
int TopHPWait=2;
boolean FrontHPDir=true;
boolean RearHPDir=true;
boolean TopHPDir=true;

unsigned char FrontHPStateDefault=0;
unsigned char RearHPStateDefault=0;
unsigned char TopHPStateDefault=0;

int adc_key_val[5] ={50, 200, 400, 600, 800 };  //  for keypad
int NUM_KEYS = 5;
int adc_key_in;
int key=-1;
int oldkey=-1;

void setup(){
  Wire.begin(SlaveArduino);
  ArduinoTransfer.begin(details(Commands), &Wire);
  Wire.onReceive(receive);
  randomSeed(analogRead(A0));
  LShut(0);
  LShut(1);
  LShut(2);
  LShut(3);
  LShut(4);
  PieShut(0);
  PieShut(1);
  PieShut(2);
  PieShut(3);
  FrontHPStateDefault=7;
  RearHPStateDefault=7;
  TopHPStateDefault=7;
  HPDanceTogether();
  AllOpenClose();
  HPDelay(100);
  AllOpenClose();
  HPDelay(300);
  FrontHPStateDefault=0;
  RearHPStateDefault=0;
  TopHPStateDefault=0;
}

void loop(){
  if(ArduinoTransfer.receiveData()){
    if(Commands.Script != 0){
      if(Commands.Script == 4){  // R1 + Triangle - HP Dance
        HPDanceTogether();
      } else if(Commands.Script == 8){ //  L1+R1+Triangle - Top HP on/off
        if(TopHPStateDefault==0){
          TopHPStateDefault=1;
        } else if(TopHPStateDefault==1){
          TopHPStateDefault=2;
        } else if(TopHPStateDefault==2){
          TopHPStateDefault=4;
        } else if(TopHPStateDefault==4){
          TopHPStateDefault=3;
        } else if(TopHPStateDefault==3){
          TopHPStateDefault=5;
        } else {
          TopHPStateDefault=0;
        }  
      } else if(Commands.Script == 9){  // L2+R2 + Triangle - Pie 1
        PieOpenShut(0);  
      } else if(Commands.Script == 12){  // All 4 + Triangle - Servo demo
        DemoScript();  
      } else if(Commands.Script == 19){  // L2 + Circle - j5 script 
        HPsOn = false;
        FrontHPStateDefault=6;
        RearHPStateDefault=6;
        TopHPStateDefault=6;
        HPDelay(7000);
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
        HPsOn = true;
      } else if(Commands.Script == 20){  // R1 + Circle - Ripple Clockwise
        RippleClock(100);  
      } else if(Commands.Script == 25){  // L2+R2 + Circle - Pie 2
        PieOpenShut(1);
      } else if(Commands.Script == 33){  // Cross - HPs Random Flicker or Pulse or Off
        if((FrontHPStateDefault==0) && (RearHPStateDefault==0) && (TopHPStateDefault==0)){
          FrontHPStateDefault=6;
          RearHPStateDefault=6;
          TopHPStateDefault=6;
        } else if((FrontHPStateDefault==6) && (RearHPStateDefault==6) && (TopHPStateDefault==7)){
          FrontHPStateDefault=7;
          RearHPStateDefault=7;
          TopHPStateDefault=7;
        } else {
          FrontHPStateDefault=0;
          RearHPStateDefault=0;
          TopHPStateDefault=0;
        }          
      } else if(Commands.Script == 35){ //  L2 + Cross - failure 
        HPsOn = false;
        FrontHPStateDefault=6;
        RearHPStateDefault=6;
        TopHPStateDefault=6;
        HPChosenMove(0,2);
        HPChosenMove(1,2);
        HPChosenMove(2,2); 
        AllOpenCloseSlow(800);
        HPDelay(5500);
        AllOpenCloseSlow(1000);
        HPDelay(500); 
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
        HPsOn = true;
      } else if(Commands.Script == 36){  // R1 + Cross - Open delay then close
        AllUpThenDown(1500);
      } else if(Commands.Script == 41){  //  L2+R2 + Cross - Pie 3
        PieOpenShut(2);
      } else if(Commands.Script == 51){  //  L2 + Square - Love msg
        HPsOn = false;
        FrontHPStateDefault=7;
        RearHPStateDefault=7;
        TopHPStateDefault=7;
        PieOpenShut(2);
        HPChosenMove(0,2);
        HPChosenMove(1,2);
        HPChosenMove(2,2); 
        HPDelay(800);
        HPChosenMove(0,4);
        HPChosenMove(1,4);
        HPChosenMove(2,4);  
        HPDelay(2200);
        HPChosenMove(0,0);
        HPChosenMove(1,0);
        HPChosenMove(2,0);  
        HPDelay(1000);
        HPChosenMove(0,2);
        HPChosenMove(1,2);
        HPChosenMove(2,2);
        HPDelay(1000); 
        PieOpenShut(2);
        HPDelay(500); 
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
        HPsOn = true;
      } else if(Commands.Script == 52){  // R1 + Square - Ripple Anticlockwise
        RippleAnti(100);
      } else if(Commands.Script == 56){  // L1+R1+Square - All HP Random On/Off
        if((FrontHPStateDefault==0) && (RearHPStateDefault==0) && (TopHPStateDefault==0)){
          FrontHPStateDefault=3;
          RearHPStateDefault=3;
          TopHPStateDefault=3;
        } else {
          FrontHPStateDefault=0;
          RearHPStateDefault=0;
          TopHPStateDefault=0;
        }
      } else if(Commands.Script == 57){  // L2+R2 + Left - Pie 4
        PieOpenShut(3);  
      } else if(Commands.Script == 67){  // L2 + Up - Short circuit 
        HPsOn = false;
        FrontHPStateDefault=7;
        RearHPStateDefault=7;
        TopHPStateDefault=7;
        HPDelay(1000);
        HPChosenMove(0,4);
        HPChosenMove(1,4);
        HPChosenMove(2,4); 
        AllOpenCloseSlow(600);
        AllOpenClose();
        HPDelay(150);
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(150);
        AllOpenClose();
        HPDelay(50);
        AllOpenClose();
        HPDelay(50);
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(150);
        AllOpenClose();
        HPDelay(50);
        HPChosenMove(0,0);
        HPChosenMove(1,0);
        HPChosenMove(2,0); 
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(50);
        AllOpenClose();
        HPDelay(150);
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(150);
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(50);
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(50);
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(100);
        AllOpenClose();
        HPDelay(150);
        AllOpenClose();
        HPChosenMove(0,2);
        HPChosenMove(1,2);
        HPChosenMove(2,2);
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0; 
        HPDelay(2000);
        FrontHPStateDefault=7;
        RearHPStateDefault=7;
        TopHPStateDefault=7;
        AllOpenCloseSlow(900);
        HPDelay(700);  
        AllOpenClose();
        HPDelay(700); 
        AllOpenClose();
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
        HPsOn = true;
      } else if((Commands.Script == 68)||(Commands.Script == 132)){  // R1 + Up - Open/Close All Pies
        PieOpenClose();  
      } else if(Commands.Script == 73){  // L2+R2 + Up - Lower Door 1
        LOpenShut(0);  
      } else if(Commands.Script == 81){  // Right - lights demo 1
        HPsOn = false;
        FrontHPStateDefault=2;
        RearHPStateDefault=2;
        TopHPStateDefault=2;
        HPDelay(7000);
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
        HPsOn = true;
      } else if(Commands.Script == 83){  // L2 + Right - Cantina
        FrontHPStateDefault=7;
        RearHPStateDefault=7;
        TopHPStateDefault=7;
        AllOpenClose();
        HPDelay(600);
        AllOpenClose();
        HPDelay(DanceDelay);
        AllOpenClose();
        Dance();
        Dance();
        Dance();
        AllOpenClose();
        HPDelay(DanceDelay);
        AllOpenClose();
        HPDelay(DanceDelay);
        AllOpenClose();
        HPDelay(DanceDelay);
        AllOpenClose();
        HPDelay(DanceDelay);
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
      } else if(Commands.Script == 84){  // R1 + Right - Ripple Slow
        MexWaveSlow();
      } else if(Commands.Script == 89){  // L2+R2 + Right - Lower Door 2
        LOpenShut(1);
      } else if(Commands.Script == 99){  // L2 + Down - Luke
        HPsOn = false;
        HPChosenMove(0,2);
        HPChosenMove(1,2);
        HPChosenMove(2,2); 
        HPDelay(500);
        FrontHPStateDefault=6;
        RearHPStateDefault=6;
        TopHPStateDefault=6;
        HPDelay(21400);
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
        PieOpenClose(); 
        HPDelay(1400);
        PieOpenClose(); 
        FrontHPStateDefault=6;
        RearHPStateDefault=6;
        TopHPStateDefault=6;
        HPDelay(3000);
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
        HPsOn = true;
      } else if((Commands.Script == 100)||(Commands.Script == 148)){  // R1 + Down - Open/Close all Lower Doors
        LowerOpenClose();
      } else if(Commands.Script == 105){  // L2+R2 + Cross - Lower Door 3
        LOpenShut(2);
      } else if(Commands.Script == 113){  // Left - lights demo 2
        // lights demo 2
      } else if(Commands.Script == 115){  // L2 + Left - Liea
        HPsOn = false;
        HPChosenMove(0,2);
        HPChosenMove(1,2);
        HPChosenMove(2,2); 
        HPDelay(500);
        FrontHPStateDefault=6;
        RearHPStateDefault=6;
        TopHPStateDefault=6;
        HPDelay(32000);
        HPDelay(1500);
        FrontHPStateDefault=0;
        RearHPStateDefault=0;
        TopHPStateDefault=0;
        HPsOn = true;
      } else if(Commands.Script == 116){  // R1 + Left - Open Close ALL Doors
        AllOpenClose();
      } else if(Commands.Script == 121){  // L2+R2 + Left - Lower Door 4
        LOpenShut(3);
      } else if(Commands.Script == 136){  //  L1+R1 + L3 - Rear HP On/Off
        if(RearHPStateDefault==0){
          RearHPStateDefault=1;
        } else if(RearHPStateDefault==1){
          RearHPStateDefault=2;
        } else if(RearHPStateDefault==2){
          RearHPStateDefault=4;
        } else if(RearHPStateDefault==4){
          RearHPStateDefault=3;
        } else if(RearHPStateDefault==3){
          RearHPStateDefault=5;
        } else {
          RearHPStateDefault=0;
        } 
      } else if(Commands.Script == 137){  // R1 + L3 - Ripple Anticlockwise     
        RippleAnti(600);
      } else if(Commands.Script == 152){  //  L1+R1 + R3 - Front HP On/Off
        if(FrontHPStateDefault==0){
          FrontHPStateDefault=1;
        } else if(FrontHPStateDefault==1){
          FrontHPStateDefault=2;
        } else if(FrontHPStateDefault==2){
          FrontHPStateDefault=4;
        } else if(FrontHPStateDefault==4){
          FrontHPStateDefault=3;
        } else if(FrontHPStateDefault==3){
          FrontHPStateDefault=5;
        } else {
          FrontHPStateDefault=0;
        }
      } else if(Commands.Script == 153){  // R1 + R3 - Open then Close ALL Doors Slowly
        AllOpenCloseSlow(800);
      } else if(Commands.Script == 164){  // R1 + Start - Stagger Up Stagger Down
        StaggerUp();
      } else if(Commands.Script == 169){  // L2+R2 + Start - Random HP On/Off
        if(HPsOn == true){
          HPsOn = false;
          HPChosenMove(0,2);  
          HPChosenMove(1,2);
          HPChosenMove(2,2);
        } else {
          HPsOn = true;
        }
      } else if(Commands.Script == 180){  // L1+R1 + Select - Alternate 5 then 4 open then close
        Dance();
      } else if(Commands.Script == 184){  // L1+R1 + Select - All HP On/Off
        if((FrontHPStateDefault==0) && (RearHPStateDefault==0) && (TopHPStateDefault==0)){
          FrontHPStateDefault=1;
          RearHPStateDefault=1;
          TopHPStateDefault=1;
        } else {
          FrontHPStateDefault=0;
          RearHPStateDefault=0;
          TopHPStateDefault=0;
        }  
      } else if(Commands.Script == 185){  // R2 + Select - Lower Door 5
        LOpenShut(4);
      }  
    }
  }
  HPDelay(DelayTime);
}

void HPDelay(int Time){   
  div_t TimeResult;
  TimeResult = div(Time, DelayTime);
  Time = TimeResult.quot;
  for(int Looper = 0; Looper <Time; Looper++){
    if(HPsOn == true){  
      if(timer == DelayFactor*HPTimes[HPTime]){
        HPMove();
        HPTime = HPTime +1;
        if(HPTime == 15){
          HPTime = 0;
        }
      }
      timer = timer +1; 
      if(timer == DelayFactor*50){
        timer = 0;
      }
    }
    SetHPs(FrontHPStateDefault, RearHPStateDefault, TopHPStateDefault);
    Servos::delay(DelayTime);
  }
}

void LOpenShut(int No){
  if(LDoorOpenCheck[No] == false){
    LOpen(No);
  } else {
    LShut(No);
  }
}

void LOpen(int No){  //  No is 7, 8, 9, 10 or 11  
  Dome.moveTo(No+7, 0, LDoorOpen[No], LDoorOpen[No]);
  LDoorOpenCheck[No] = true;
}

void LOpenSlow(int No, int Dur){
  Dome.moveTo(No+7, Dur, LDoorShut[No], LDoorOpen[No]);
  LDoorOpenCheck[No] = true;
}

void LShut(int No){    
  Dome.moveTo(No+7, 0, LDoorShut[No], LDoorShut[No]);
  LDoorOpenCheck[No] = false;
}

void LShutSlow(int No, int Dur){    
  Dome.moveTo(No+7, Dur, LDoorOpen[No], LDoorShut[No]);
  LDoorOpenCheck[No] = false;
}

void PieOpenShut(int No){
  if(PieDoorOpenCheck[No] == false){
    PieOpen(No);
  } else {
    PieShut(No);
  }
}

void PieOpen(int No){  //  No is 3, 4, 5 or 6 (so add 3 to No for servo)
  Dome.moveTo(No+3, 0, PieDoorOpen[No], PieDoorOpen[No]);
  PieDoorOpenCheck[No] = true;
}

void PieOpenSlow(int No, int Dur){
  Dome.moveTo(No+3, Dur, PieDoorShut[No], PieDoorOpen[No]);
  PieDoorOpenCheck[No] = true;
}

void PieShut(int No){
  Dome.moveTo(No+3, 0, PieDoorShut[No], PieDoorShut[No]);
  PieDoorOpenCheck[No] = false;
}

void PieShutSlow(int No, int Dur){
  Dome.moveTo(No+3, Dur, PieDoorOpen[No], PieDoorShut[No]);
  PieDoorOpenCheck[No] = false;
}

void AllOpenClose(){
  if((PieDoorOpenCheck[0] == false) && (PieDoorOpenCheck[1] == false) && (PieDoorOpenCheck[2] == false) && (PieDoorOpenCheck[3] == false) && 
    (LDoorOpenCheck[0] == false) && (LDoorOpenCheck[1] == false) && (LDoorOpenCheck[2] == false) && (LDoorOpenCheck[3] == false) && (LDoorOpenCheck[4] == false)){
    LOpen(0);
    LOpen(1);
    LOpen(2);
    LOpen(3);
    LOpen(4);
    PieOpen(0);
    PieOpen(1);
    PieOpen(2);
    PieOpen(3);
  } else {
    LShut(0);
    LShut(1);
    LShut(2);
    LShut(3);
    LShut(4);
    PieShut(0);
    PieShut(1);
    PieShut(2);
    PieShut(3);
  }
}

void AllOpenCloseSlow(int Time){
  if((PieDoorOpenCheck[0] == false) && (PieDoorOpenCheck[1] == false) && (PieDoorOpenCheck[2] == false) && (PieDoorOpenCheck[3] == false) && 
    (LDoorOpenCheck[0] == false) && (LDoorOpenCheck[1] == false) && (LDoorOpenCheck[2] == false) && (LDoorOpenCheck[3] == false) && (LDoorOpenCheck[4] == false)){
    LOpenSlow(0, Time);
    LOpenSlow(1, Time);
    LOpenSlow(2, Time);
    LOpenSlow(3, Time);
    LOpenSlow(4, Time);
    PieOpenSlow(0, Time);
    PieOpenSlow(1, Time);
    PieOpenSlow(2, Time);
    PieOpenSlow(3, Time);
  } else {
    LShutSlow(0, Time);
    LShutSlow(1, Time);
    LShutSlow(2, Time);
    LShutSlow(3, Time);
    LShutSlow(4, Time);
    PieShutSlow(0, Time);
    PieShutSlow(1, Time);
    PieShutSlow(2, Time);
    PieShutSlow(3, Time);
  }
  HPDelay(Time);
}

void LowerOpenClose(){
  if((LDoorOpenCheck[0] == false) && (LDoorOpenCheck[1] == false) && (LDoorOpenCheck[2] == false) && (LDoorOpenCheck[3] == false) && (LDoorOpenCheck[4] == false)){
    LOpen(0);
    LOpen(1);
    LOpen(2);
    LOpen(3);
    LOpen(4);
  } else {
    LShut(0);
    LShut(1);
    LShut(2);
    LShut(3);
    LShut(4);
  }
}

void PieOpenClose(){
  if((PieDoorOpenCheck[0] == false) && (PieDoorOpenCheck[1] == false) && (PieDoorOpenCheck[2] == false) && (PieDoorOpenCheck[3] == false)){
    PieOpen(0);
    PieOpen(1);
    PieOpen(2);
    PieOpen(3);
  } else {
    PieShut(0);
    PieShut(1);
    PieShut(2);
    PieShut(3);
  }
}

void AllUpThenDown(int Time){
  AllOpenClose();
  HPDelay(Time);
  AllOpenClose();
}

void Dance(){
  HPsOn = false;
  LOpen(0);
  LOpen(2);
  LOpen(4);
  PieOpen(0);
  PieOpen(2);
  LShut(1);
  LShut(3);
  PieShut(1);
  PieShut(3);
  HPDelay(DanceDelay);
  LOpen(1);
  LOpen(3);
  PieOpen(1);
  PieOpen(3);  
  HPDelay(DanceDelay);
  LShut(0);
  LShut(2);
  LShut(4);
  PieShut(0);
  PieShut(2);
  HPDelay(DanceDelay);
  LShut(1);
  LShut(3);
  PieShut(1);
  PieShut(3);
  HPDelay(DanceDelay);
  LOpen(0);
  LOpen(2);
  LOpen(4);
  PieOpen(0);
  PieOpen(2);
  HPDelay(DanceDelay);
  LShut(0);
  LShut(2);
  LShut(4);
  PieShut(0);
  PieShut(2);
  HPDelay(DanceDelay);
  LOpen(1);
  LOpen(3);
  PieOpen(1);
  PieOpen(3);  
  HPDelay(DanceDelay);
  LShut(1);
  LShut(3);
  PieShut(1);
  PieShut(3);
  HPsOn = true;
}

void StaggerUp(){
  HPsOn = false;
  LOpen(0);
  HPDelay(260);
  LOpen(1);
  HPDelay(260);
  LOpen(2);
  HPDelay(260);
  LOpen(3);
  HPDelay(260);
  LOpen(4);
  HPDelay(260);
  PieOpen(0);
  HPDelay(260);
  PieOpen(1);
  HPDelay(260);
  PieOpen(2);
  HPDelay(260);
  PieOpen(3);
  HPDelay(800);
  PieShut(3);
  HPDelay(260);
  PieShut(2);
  HPDelay(260);
  PieShut(1);
  HPDelay(260);
  PieShut(0);
  HPDelay(260);
  LShut(4);
  HPDelay(260);
  LShut(3);
  HPDelay(260);
  LShut(2);
  HPDelay(260);
  LShut(1);
  HPDelay(260);
  LShut(0);
  HPsOn = true;
}

void MexWaveSlow(){
  HPsOn = false;
  LOpenSlow(0, 400);
  HPDelay(200);
  LOpenSlow(1, 400);
  HPDelay(200);
  LShutSlow(0, 400);
  LOpenSlow(2, 400);
  HPDelay(200);
  LShutSlow(1, 400);
  LOpenSlow(3, 400);
  HPDelay(200);
  LShutSlow(2, 400);
  LOpenSlow(4, 400);
  HPDelay(200);
  LShutSlow(3, 400);
  PieOpenSlow(0, 400);
  HPDelay(200);
  LShutSlow(4, 400);
  PieOpenSlow(1, 400);
  HPDelay(200);
  PieShutSlow(0, 400);
  PieOpenSlow(2, 400);
  HPDelay(200);
  PieShutSlow(1, 400);
  PieOpenSlow(3, 400);
  HPDelay(200);
  PieShutSlow(2, 400);
  HPDelay(200);
  PieShutSlow(3, 400);
  HPDelay(500);
  HPsOn = true;
}

void RippleAnti(int Time){
  HPsOn = false;
  LOpen(0);
  HPDelay(Time);
  LShut(0);
  LOpen(1);
  HPDelay(Time);
  LShut(1);
  LOpen(2);
  HPDelay(Time);
  LShut(2);
  LOpen(3);
  HPDelay(Time);
  LShut(3);
  LOpen(4);
  HPDelay(Time);
  LShut(4);
  Servos::delay(200);
  PieOpen(0);
  HPDelay(Time);
  PieShut(0);
  PieOpen(1);
  HPDelay(Time);
  PieShut(1);
  PieOpen(2);
  HPDelay(Time);
  PieShut(2);
  PieOpen(3);
  HPDelay(Time);
  PieShut(3);
  HPsOn = true;
}

void RippleClock(int Time){
  HPsOn = false;
  LOpen(4);
  HPDelay(Time);
  LShut(4);
  LOpen(3);
  HPDelay(Time);
  LShut(3);
  LOpen(2);
  HPDelay(Time);
  LShut(2);
  LOpen(1);
  HPDelay(Time);
  LShut(1);
  LOpen(0);
  HPDelay(Time);
  LShut(0);
  Servos::delay(300);
  PieOpen(3);
  HPDelay(Time);
  PieShut(3);
  PieOpen(2);
  HPDelay(Time);
  PieShut(2);
  PieOpen(1);
  HPDelay(Time);
  PieShut(1);
  PieOpen(0);
  HPDelay(Time);
  PieShut(0);
  HPsOn = true;
}

void HPDanceTogether(){
  HPsOn = false;
  HPChosenMove(0,2);  // Postions 0 to 5 lower being lower on lower on all 3
  HPChosenMove(1,2);
  HPChosenMove(2,2);
  HPDelay(400);
  HPChosenMove(0,1);
  HPChosenMove(1,1);
  HPChosenMove(2,1);
  HPDelay(500);
  HPChosenMove(0,0);
  HPChosenMove(1,0);
  HPChosenMove(2,0);
  HPDelay(300);
  HPChosenMove(0,1);
  HPChosenMove(1,1);
  HPChosenMove(2,1);
  HPDelay(300);
  HPChosenMove(0,3);
  HPChosenMove(1,3);
  HPChosenMove(2,3);
  HPDelay(700);
  HPChosenMove(0,4);
  HPChosenMove(1,4);
  HPChosenMove(2,4);
  HPDelay(300);
  HPChosenMove(0,3);
  HPChosenMove(1,3);
  HPChosenMove(2,3);
  HPDelay(300);
  HPChosenMove(0,2);
  HPChosenMove(1,2);
  HPChosenMove(2,2); 
  HPDelay(300);
  HPsOn = true;
}

void HPMove(){
  int HPNo = rand() % 3;
  int SpotNo = rand() % 5;
  HPChosenMove(HPNo, SpotNo);
  if(HPNo==0){
    analogWrite(FrontHPPin, 255); 
    Servos::delay(150);
    analogWrite(FrontHPPin, 0); 
  }else if(HPNo==1){
    analogWrite(RearHPPin, 255); 
    Servos::delay(150);
    analogWrite(RearHPPin, 0); 
  }else{
    analogWrite(TopHPPin, 255); 
    Servos::delay(150);
    analogWrite(TopHPPin, 0); 
  }
}

void HPChosenMove(int N, int S){  // N is 0, 1 or 2 servo
  Dome.moveTo(N, 0, HPSpot[N][S], HPSpot[N][S]);
}

void DemoScript(){
  HPsOn = false;
  AllOpenCloseSlow(500);
  HPDelay(900);
  LowerOpenClose();
  HPDelay(500);
  PieOpenClose();
  HPDelay(400);
  RippleAnti(100);
  HPDelay(200);
  RippleAnti(300);
  HPDelay(200);
  StaggerUp();
  HPDelay(400);
  Dance();
  HPDelay(400);  
  LowerOpenClose();
  HPDelay(500);
  PieOpenClose();
  HPDelay(400);
  AllOpenCloseSlow(500);
  HPsOn = true;
}

void FrontHPOn(){
  FrontHPOnState=true;
  analogWrite(FrontHPPin, 255); 
}

void FrontHPOff(){
  FrontHPOnState=false;
  analogWrite(FrontHPPin, 0); 
}

void FrontHPRandom(){
  if(random(2)==1){
    FrontHPOn();
  } else {
    FrontHPOff();
  }
}

void FrontHPRandomOff(){
  if(random(2)&random(2)&random(2)==1){
    FrontHPOn();
  } else {
    FrontHPOff();
  }
}

void FrontHPRandomOn(){
  if(random(2)|random(2)|random(2)==1){
    FrontHPOn();
  } else {
    FrontHPOff();
  }
}

void FrontHPFlash(){
  if(FrontHPCounter==0){
    if(FrontHPOnState==false){
      FrontHPOn();
    } else {
      FrontHPOff();
    }
  }
  if(FrontHPCounter>=FrontHPWait){
    FrontHPCounter=0;
  }else{
    FrontHPCounter++;
  }
}

void FrontHPRandomBrightness(){
  if(random(FrontHPWait)==0){
    analogWrite(FrontHPPin, Brightness[random(12)]);
  } 
}

void FrontHPScan(){
  if(FrontHPCounter==0){
    analogWrite(FrontHPPin, Brightness[FrontHPPosition]);
  }
  if(FrontHPCounter>=FrontHPWait){
    if(FrontHPDir==true){
      if(FrontHPPosition==11){
        FrontHPDir=false;
        FrontHPPosition--;
      }else{
        FrontHPPosition++;
      }
    }else{
      if(FrontHPPosition==0){
        FrontHPDir=true;
        FrontHPPosition++;
      }else{
        FrontHPPosition--;
      }        
    }
    FrontHPCounter=0;
  }else{
    FrontHPCounter++;
  }
}

void RearHPOn(){
  RearHPOnState=true;
  analogWrite(RearHPPin, 255); 
}

void RearHPOff(){
  RearHPOnState=false;
  analogWrite(RearHPPin, 0); 
}

void RearHPRandom(){
  if(random(2)==1){
    RearHPOn();
  } else {
    RearHPOff();
  }
}

void RearHPRandomOff(){
  if(random(2)&random(2)&random(2)==1){
    RearHPOn();
  } else {
    RearHPOff();
  }
}

void RearHPRandomOn(){
  if(random(2)|random(2)|random(2)==1){
    RearHPOn();
  } else {
    RearHPOff();
  }
}

void RearHPFlash(){
  if(RearHPCounter==0){
    if(RearHPOnState==false){
      RearHPOn();
    } else {
      RearHPOff();
    }
  }
  if(RearHPCounter>=RearHPWait){
    RearHPCounter=0;
  }else{
    RearHPCounter++;
  }
}

void RearHPRandomBrightness(){
  if(random(RearHPWait)==0){
    analogWrite(RearHPPin, Brightness[random(12)]);
  } 
}

void RearHPScan(){
  if(RearHPCounter==0){
    analogWrite(RearHPPin, Brightness[RearHPPosition]);
  }
  if(RearHPCounter>=RearHPWait){
    if(RearHPDir==true){
      if(RearHPPosition==11){
        RearHPDir=false;
        RearHPPosition--;
      }else{
        RearHPPosition++;
      }
    }else{
      if(RearHPPosition==0){
        RearHPDir=true;
        RearHPPosition++;
      }else{
        RearHPPosition--;
      }        
    }
    RearHPCounter=0;
  }else{
    RearHPCounter++;
  }
}

void TopHPOn(){
  TopHPOnState=true;
  analogWrite(TopHPPin, 255); 
}

void TopHPOff(){
  TopHPOnState=false;
  analogWrite(TopHPPin, 0); 
}

void TopHPRandom(){
  if(random(2)==1){
    TopHPOn();
  } else {
    TopHPOff();
  }
}

void TopHPRandomOff(){
  if(random(2)&random(2)&random(2)==1){
    TopHPOn();
  } else {
    TopHPOff();
  }
}

void TopHPRandomOn(){
  if(random(2)|random(2)|random(2)==1){
    TopHPOn();
  } else {
    TopHPOff();
  }
}

void TopHPFlash(){
  if(TopHPCounter==0){
    if(TopHPOnState==false){
      TopHPOn();
    } else {
      TopHPOff();
    }
  }
  if(TopHPCounter>=TopHPWait){
    TopHPCounter=0;
  }else{
    TopHPCounter++;
  }
}

void TopHPRandomBrightness(){
  if(random(TopHPWait)==0){
    analogWrite(TopHPPin, Brightness[random(12)]);
  } 
}

void TopHPScan(){
  if(TopHPCounter==0){
    analogWrite(TopHPPin, Brightness[TopHPPosition]);
  }
  if(TopHPCounter>=TopHPWait){
    if(TopHPDir==true){
      if(TopHPPosition==11){
        TopHPDir=false;
        TopHPPosition--;
      }else{
        TopHPPosition++;
      }
    }else{
      if(TopHPPosition==0){
        TopHPDir=true;
        TopHPPosition++;
      }else{
        TopHPPosition--;
      }        
    }
    TopHPCounter=0;
  }else{
    TopHPCounter++;
  }
}

void SetHPs(unsigned char FrontHP, unsigned char RearHP, unsigned char TopHP){
  if(FrontHP==0){
    FrontHPOff();
  } else if(FrontHP==1){
    FrontHPOn();
  } else if(FrontHP==2){
    FrontHPFlash();
  } else if(FrontHP==3){
    FrontHPRandom();
  } else if(FrontHP==4){
    FrontHPRandomOn();
  } else if(FrontHP==5){
    FrontHPRandomOff();
  } else if(FrontHP==6){
    FrontHPRandomBrightness();
  } else if(FrontHP==7){
    FrontHPScan();
  }
  if(RearHP==0){
    RearHPOff();
  } else if(RearHP==1){
    RearHPOn();
  } else if(RearHP==2){
    RearHPFlash();
  } else if(RearHP==3){
    RearHPRandom();
  } else if(RearHP==4){
    RearHPRandomOn();
  } else if(RearHP==5){
    RearHPRandomOff();
  } else if(RearHP==6){
    RearHPRandomBrightness();
  } else if(RearHP==7){
    RearHPScan();
  }
  if(TopHP==0){
    TopHPOff();
  } else if(TopHP==1){
    TopHPOn();
  } else if(TopHP==2){
    TopHPFlash();
  } else if(TopHP==3){
    TopHPRandom();
  } else if(TopHP==4){
    TopHPRandomOn();
  } else if(TopHP==5){
    TopHPRandomOff();
  } else if(TopHP==6){
    TopHPRandomBrightness();
  } else if(TopHP==7){
    TopHPScan();
  }
}

void receive(int numBytes) {}

