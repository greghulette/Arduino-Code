// =======================================================================================
// =========================== BHD Body by BigHappyDude ==================================
// =======================================================================================
//
// This is the body unit for the droid, it runs the CBI, Vegas Kit and all body door servos
// including the Utility arms.
// Special thanks to Bill Porter for the PS2X_Lib and Easy Transfer Libraries and to all 
// the guys on .net that gave me the idea especially danf
//
// IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT
// This has been written for an arduino Uno.
//
// The following pins are used on the uno:
//    I2C connection: A4+A5
//    CBI: 11, 12, 13
//    Vegas: LDP: 2, 3  CS: 4, 5
//
// The Adafruit for the body is on 0x41 (solder the first jumper)
//
// The list of commands is available in the BHDLists.xlsx spreadsheet, or on the cheatsheet
//
// NOTE: All I2C devices must share the same GND level.

#include <Wire.h>
#include <EasyTransferI2C.h>
#include <SPI.h>
#include <LedControl.h>    
#include <Adafruit_WS2801.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servos.h>
EasyTransferI2C ArduinoTransfer; 

uint8_t LDPdataPin  = 2;    // Yellow wire on Adafruit LDP Pixels
uint8_t LDPclockPin = 3;    // Green wire on Adafruit LDP Pixels
uint8_t CoinSdataPin  = 4;    // Yellow wire on Adafruit Coin Slots Pixels
uint8_t CoinSclockPin = 5;    // Green wire on Adafruit Coin Slots Pixels
Adafruit_WS2801 LDP13 = Adafruit_WS2801(13, LDPdataPin, LDPclockPin);
Adafruit_WS2801 CoinS12 = Adafruit_WS2801(12, CoinSdataPin, CoinSclockPin);

struct RECEIVE_DATA_STRUCTURE{  //put your variable definitions here for the data you want to receive
  int Script;  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
};
RECEIVE_DATA_STRUCTURE Commands;
#define SlaveArduino 11  //define slave i2c address

long LDPColor1=Color(150, 0, 0);
long LDPColor2=Color(0, 0, 0);
long LDPColor1D=Color(38, 0, 0);  // 1/2 brightness
long LDPColor1d=Color(15, 0, 0);  // 1/10 brightness
long CSColor1=Color(150, 150, 150);
long CSColor2=Color(0, 0, 0);
int LDPMode=0;  // Off
int CSMode=0;  // Off
int LDPColor1M=1;   //  0=off, 1=red, 2=blue, 3=white, 4=green, 5=purple
int LDPColor2M=0;   //  0=off, 1=red, 2=blue, 3=white, 4=green, 5=purple
int CSColor1M=3;    //  0=off, 1=red, 2=blue, 3=white, 4=green, 5=purple
int CSColor2M=0;    //  0=off, 1=red, 2=blue, 3=white, 4=green, 5=purple
int LDPWait;    
int CSWait;  
int CSRandomNo=2;  
int CSCounter=0;
int LDPCounter=0;
int CSPosition=0;
int LDPPosition=0;
boolean CSAltState=true;
boolean LDPAltState=true;
boolean CSDir=true;
boolean LDPDir=true;

LedControl CBI = LedControl(13,12,11,1);  //  Setup CBI Chip     

Servos BodyServos(0x41);

int UArmTop=0;                            //  Servo number on controller
int UArmTopSpot[3] = {290, 645, 471};     //  Open/Close/Mid points
boolean UArmTopCheck = false;                     //  Check value, open = true, closed = false
int UArmBottom=1;
int UArmBottomSpot[3] = {260, 639, 449};
boolean UArmBottomCheck = false;
int LeftLDoor=2;
int LeftLDoorSpot[2] = {250, 500};         //  Open/Close points
boolean LeftLDoorCheck = false;
int CBIDoor=3;
int CBIDoorSpot[2] = {500, 250};
boolean CBIDoorCheck = false;
int SmallDoor=4;
int SmallDoorSpot[2] = {250, 500};
boolean SmallDoorCheck = false;
int BigDoor=5;
int BigDoorSpot[2] = {500, 250};
boolean BigDoorCheck = false;
int RightLDoor=6;
int RightLDoorSpot[2] = {500, 250};
boolean RightLDoorCheck = false;
int UArmSpeed = 500;
int LoopWait = 20;                        //  loop delay
int DelayLoopCount = 0;                   //  Used as store of current loop number
int DelayLoops = 16;                      //  CBI randoom mode Delay loops * LoopWait = delay between each random flash change

int DividerIn = A2;                       //  this must be between 0.0 and 5.0 - otherwise you'll let the blue smoke out of your arduino 
float Max = 13.8;                         //  Full battery charge level, all above this will show as 20/20
float Min = 12;                         //  Empty battery charge level, all below will show as 0/20
float R1 = 47000.0;                       //  Value of R1 in ohms
float R2 = 23700.0;                       //  Value of R2


void setup(){
  BodyServos.moveTo(UArmTop, UArmSpeed, UArmTopSpot[1], UArmTopSpot[1]); //  Close all servos/doors
  BodyServos.moveTo(UArmBottom, UArmSpeed, UArmBottomSpot[1], UArmBottomSpot[1]);
  CBIDoorClose(); 
  LeftLDoorClose();
  SmallDoorClose();
  BigDoorClose();
  RightLDoorClose();
  Wire.begin(SlaveArduino);
  ArduinoTransfer.begin(details(Commands), &Wire);
  Wire.onReceive(receive);
  LDP13.begin();
  LDP13.show();
  CoinS12.begin();
  CoinS12.show();
  setLDPOff();
  setCSOff();
  Max = (Max*(R2/(R1+R2)))*1024/5;    //  Use divider equation so ((0-15v value)*(R2/(R1+R2))) to get 0-5v value and ((0-5v value)*1024/5) to get 0-1024 value
  Min = (Min*(R2/(R1+R2)))*1024/5; 
  randomSeed(analogRead(A3));         //  Random seed for true random
  pinMode(DividerIn, INPUT);          // this must be between 0.0 and 5.0 - otherwise you'll let the blue smoke out of your arduino
  ChipOn();                           //  Setup CBI Chip 
  CBI.setIntensity(0,0);              //  Setup CBI Chip 
  CBI.clearDisplay(0);                //  Setup CBI Chip 
  ChipOff(); 
  
  delay(1000);
  for (int i=0; i <25; i++) {
    if(i<13){
      LDP13.setPixelColor(i, Color(80, 80, 80));
      LDP13.show();
      delay(20);
    } else {
      CoinS12.setPixelColor(i-13, Color(80, 80, 80));
      CoinS12.show();
      delay(20);
    }
  }
  for (int i=0; i <25; i++) {
    if(i<13){
      LDP13.setPixelColor(i, Color(0, 0, 0));
      LDP13.show();
      delay(20);
    } else {
      CoinS12.setPixelColor(i-13, Color(0, 0, 0));
      CoinS12.show();
      delay(20);
    }
  }
  setLDPOff();
  setCSOff();
//  CBIDoorOpen(); //  Open CBI Door to turn on CBI display by default
}

void loop(){
  if(ArduinoTransfer.receiveData()){  //check and see if a data packet has come in. 
    if(Commands.Script != 0){
      if(Commands.Script == 1){  //  Triangle - Cylon Red and random white cs for 6 secs
          SetLDPColour1(150, 0, 0);
          LDPColor1M=1;
          SetLDPColour2(0, 0, 0);
          LDPColor2M=0;
          setLDPCylon(6);
          SetCSColour1(150, 150, 150);
          CSColor1M=3;
          SetCSColour2(0, 0, 0);
          CSColor2M=0;
          setCSAltFlash(6);
          LEDDelay(5100);
          setLDPOff();
          setCSOff();
      } else if(Commands.Script == 2){  //  L1 + Triangle - Utility arm dance 0
        //  Utility arm dance 0
      } else if((Commands.Script == 7)||(Commands.Script == 135)){  //  R1+R2 + Triangle - All body doors close if open, but all open if all closed
        if((UArmTopCheck==true)||(UArmBottomCheck==true)||(LeftLDoorCheck==true)||(CBIDoorCheck==true)
        ||(SmallDoorCheck==true)||(BigDoorCheck==true)||(RightLDoorCheck==true)){
          UArmTopClose();
          UArmBottomClose();
          LeftLDoorClose();
          RightLDoorClose();
          SmallDoorClose();
          BigDoorClose();
          CBIDoorClose();
        } else {
          UArmTopOpen();
          UArmBottomOpen();
          LeftLDoorOpen();
          RightLDoorOpen();
          SmallDoorOpen();
          BigDoorOpen();
          CBIDoorOpen();
        }
      } else if(Commands.Script == 11){  //  L2+R1 + Triangle - CS Mode Change
        switch (CSMode) {
          case 0:   
            setCSOn();
          break;
          case 1:    
            setCSScan(3);
          break;
          case 2:   
            setCSKitt(6);
          break;
          case 3:   
            setCSRandom(4, 3);
          break;
          case 4:    
            setCSAltFlash(20);
          break;
          case 5:   
            setCSAllRandom(6, 4);
          break;
          case 6:   
            setCSOff();
          break;
        }
      } else if(Commands.Script == 18){  //  L1 + Circle - Utility arm dance 1
      //  Utility arm dance 1  
      } else if(Commands.Script == 23){  //  R1+R2 + Circle - Big Door (right of vents)
        BigDoorOpenClose(); 
      } else if(Commands.Script == 27){  // L2+R1 + Circle - CS Colour 2
        switch (CSColor2M) {
          case 0:    
            SetCSColour2(150, 0, 0);
            CSColor2M=1;
          break;
          case 1:    
            SetCSColour2(0, 0, 150);
            CSColor2M=2;
          break;
          case 2:   
            SetCSColour2(150, 150, 150);
            CSColor2M=3;
          break;
          case 3:   
            SetCSColour2(0, 150, 0);
            CSColor2M=4;
          break;
          case 4:   
            SetCSColour2(150, 0, 150);
            CSColor2M=5;
          break;
          case 5:  
            SetCSColour2(0, 0, 0);
            CSColor2M=0;
          break;
        }  
      } else if(Commands.Script == 34){  //  L1 + Cross - Utility arm dance 2
      //  Utility arm dance 2  
      } else if(Commands.Script == 39){  //  R1+R2 + Cross - Little Door (left of vents)
        SmallDoorOpenClose(); 
      } else if(Commands.Script == 43){  //  L2+R1 + Cross - CS Off
        setCSOff();  
      } else if(Commands.Script == 50){  //  L1 + Square - Utility arm dance 3
      //  Utility arm dance 3    
      } else if(Commands.Script == 51){  //  L2 + Square - Love msg
      //  Love    
      } else if(Commands.Script == 55){  //  R1+R2 + Square - CBI Door
        CBIDoorOpenClose();
      } else if(Commands.Script == 59){
        switch (CSColor1M) {
          case 0:   
            SetCSColour1(150, 0, 0);
            CSColor1M=1;
          break;
          case 1:   
            SetCSColour1(0, 0, 150);
            CSColor1M=2;
          break;
          case 2:  
            SetCSColour1(150, 150, 150);
            CSColor1M=3;
          break;
          case 3:    
            SetCSColour1(0, 150, 0);
            CSColor1M=4;
          break;
          case 4:  
            SetCSColour1(150, 0, 150);
            CSColor1M=5;
          break;
          case 5:  
            SetCSColour1(0, 0, 0);
            CSColor1M=0;
          break;
        }
      } else if(Commands.Script == 66){  //  L1 + Up - CBI charge level
        if(CBIDoorCheck == false){
          int TEMP = ChargeLevelValue();
          CBIDoorOpen();
          ChargeLevel(TEMP);
          CBIDoorClose();
        }else{
          ChargeLevel(12);
        }
      } else if(Commands.Script == 67){  //  L2 + Up - Short Circuit
      //  Short Circuit   
      } else if(Commands.Script == 71){  //  R1+R2 + Up - Top Utility Arm
        UArmTopOpenClose(); 
      } else if(Commands.Script == 75){  //  L2+R1 + Up - LDP Mode Change
        switch (LDPMode) {
          case 0: 
            setLDPOn();
          break;
          case 1: 
            setLDPScan(3);
          break;
          case 2: 
            setLDPKitt(6);
          break;
          case 3: 
            setLDPCylon(4);
          break;
          case 4: 
            setLDPAltFlash(20);
          break;
          case 5: 
            setLDPOff();
          break;
        }
      } else if(Commands.Script == 81){  //  Right - lights demo 1
      //  lights demo 1
      } else if(Commands.Script == 82){  
        if(CBIDoorCheck == false){
          CBIDoorOpen();
          BoxFlash();
          BoxFlash();
          CBIDoorClose();
        }else{
          BoxFlash();
          BoxFlash();
        }
      } else if(Commands.Script == 83){  //  L2 + Right - Cantina
      //  Cantina      
      } else if(Commands.Script == 87){  //  R1+R2 + Right - Long Right Door
        RightLDoorOpenClose();
      } else if(Commands.Script == 91){  //  L2+R1 + Right - LDP Colour 2
        switch (LDPColor2M) {
          case 0:   
            SetLDPColour2(150, 0, 0);
            LDPColor2M=1;
          break;
          case 1:   
            SetLDPColour2(0, 0, 150);
            LDPColor2M=2;
          break;
          case 2:   
            SetLDPColour2(150, 150, 150);
            LDPColor2M=3;
          break;
          case 3:    
            SetLDPColour2(0, 150, 0);
            LDPColor2M=4;
          break;
          case 4:   
            SetLDPColour2(150, 0, 150);
            LDPColor2M=5;
          break;
          case 5:   
            SetLDPColour2(0, 0, 0);
            LDPColor2M=0;
          break;
        } 
      } else if(Commands.Script == 98){  //  L1 + Down - CBI Arrows Down
        if(CBIDoorCheck == false){
          CBIDoorOpen();
          ArrowsDown();
          CBIDoorClose();
        }else{
          ArrowsDown();
        }  
      } else if(Commands.Script == 103){  //  R1+R2 + Down - Bottom Utility Arm
        UArmBottomOpenClose();  
      } else if(Commands.Script == 107){  //  L2+R1 + Down - LDP Off
        setLDPOff();  
      } else if(Commands.Script == 113){  //  Left - lights demo 2
      //  lights demo 2  
      } else if(Commands.Script == 114){  //  L1 + Left - CBI Arrows Down
        if(CBIDoorCheck == false){
          CBIDoorOpen();
          AltLED();
          CBIDoorClose();
        }else{
          AltLED();
        }  
      } else if(Commands.Script == 119){  //  R1+R2 + Left - Long Left Door
        LeftLDoorOpenClose();
      } else if(Commands.Script == 123){  //  L2+R1 + Left - LDP Colour 1
        switch (LDPColor1M) {
          case 0:    
            SetLDPColour1(150, 0, 0);
            LDPColor1M=1;
          break;
          case 1:  
            SetLDPColour1(0, 0, 150);
            LDPColor1M=2;
          break;
          case 2:  
            SetLDPColour1(150, 150, 150);
            LDPColor1M=3;
          break;
          case 3:  
            SetLDPColour1(0, 150, 0);
            LDPColor1M=4;
          break;
          case 4:   
            SetLDPColour1(150, 0, 150);
            LDPColor1M=5;
          break;
          case 5:   
            SetLDPColour1(0, 0, 0);
            LDPColor1M=0;
          break;
        } 
      } else if(Commands.Script == 130){  //  R1+R2 + L3 - Top Utility Arm
        UArmTopOpenClose();
      } else if(Commands.Script == 139){  //  L2+R1 + L3 - Scanners in red then blue
        setLDPScan(5);
        setCSScan(4);
        SetLDPColour2(0, 0, 0);
        LDPColor2M=0;
        SetCSColour2(0, 0, 0);
        CSColor2M=0;
        if((LDPColor1M==1)||(CSColor1M==1)){
          SetLDPColour1(0, 0, 150);
          LDPColor1M=2;
          SetCSColour1(0, 0, 150);
          CSColor1M=2;
        } else {
          SetLDPColour1(150, 0, 0);
          LDPColor1M=1;
          SetCSColour1(150, 0, 0);
          CSColor1M=1;
        }
      } else if(Commands.Script == 146){  //  R1+R2 + R3 - Bottom Utility Arm
        UArmBottomOpenClose();
      } else if(Commands.Script == 151){  //  R1+R2 + R3 - Bottom Utility Arm
        UArmTopOpenClose();
        UArmBottomOpenClose();
      } else if(Commands.Script == 155){  //  L2+R1 + R3 - Kit in blue then red
        setLDPKitt(5);
        setCSKitt(4);
        SetLDPColour2(0, 0, 0);
        LDPColor2M=0;
        SetCSColour2(0, 0, 0);
        CSColor2M=0;
        if((LDPColor1M==1)||(CSColor1M==1)){
          SetLDPColour1(0, 0, 150);
          LDPColor1M=2;
          SetCSColour1(0, 0, 150);
          CSColor1M=2;
        } else {
          SetLDPColour1(150, 0, 0);
          LDPColor1M=1;
          SetCSColour1(150, 0, 0);
          CSColor1M=1;
        }
      } else if(Commands.Script == 167){  //  R1+R2 + Start - Ripple body doors
      //  Ripple body doors
      } else if(Commands.Script == 171){  //  L2+R1 + Start - LDP and CS Off
        setCSOff(); 
        setLDPOff(); 
      } else if(Commands.Script == 183){  //  R1+R2 + Select - Alternate open close outers inners
      //  Ripple body doors       
      }
    }
  } else {  // if no date recieved
    LEDDelay(LoopWait);  // run all LED elemnts for loop time
  }
}

void LEDDelay(int Wait){
  if(CBIDoorCheck == false){
    LEDNoCBIDelay(Wait);
  } else {
    LEDCBIDelay(Wait);
  }
}

void LEDCBIDelay(int Wait){
  div_t TimeResult;
  TimeResult = div(Wait, LoopWait);
  Wait = TimeResult.quot;
  for(int Looper = 0; Looper <Wait; Looper++){
    if(CBIDoorCheck == true){
      if(DelayLoopCount==0){
        DelayLoopCount++; 
        int TEMP = ChargeLevelValue();
        RandomPrint(0, TEMP); 
      } else {
        DelayLoopCount++;
        if(DelayLoopCount == DelayLoops){
          DelayLoopCount = 0;
        }
      }
    }
    LDP();
    CS();
    LDP13.show();
    CoinS12.show();
    Servos::delay(LoopWait);
  }
}

void LEDNoCBIDelay(int Wait){
  div_t TimeResult;
  TimeResult = div(Wait, LoopWait);
  Wait = TimeResult.quot;
  for(int Looper = 0; Looper <Wait; Looper++){
    LDP();
    CS();
    LDP13.show();
    CoinS12.show();
    Servos::delay(LoopWait);
  }
}

void ChangeDir(){
  if(LDPDir==true){
    LDPDir=false;
  } else {
    LDPDir=true;
  }
}

void LDP(){
  switch (LDPMode) {
  case 0:    // your hand is on the sensor
    LDPOff();
    break;
  case 1:    // your hand is close to the sensor
    LDPOn();
    break;
  case 2:    // your hand is a few inches from the sensor
    LDPScan();
    break;
  case 3:    // your hand is nowhere near the sensor
    LDPKitt();
    break;
  case 4:    // your hand is nowhere near the sensor
    LDPCylon();
    break;
  case 5:    // your hand is nowhere near the sensor
    LDPAltFlash();
    break;
  } 
}

void setLDPOff(){
  LDPCounter=0;
  LDPMode=0;
  LDPOff();
}

void setLDPOn(){
  LDPCounter=0;
  LDPMode=1;
  LDPOn();
}

void setLDPScan(int Wait){
  LDPWait=Wait;
  LDPCounter=0;
  LDPPosition=0;
  LDPMode=2;
}

void setLDPKitt(int Wait){
  LDPWait=Wait;
  LDPCounter=0;
  LDPPosition=0;
  LDPMode=3;
}

void setLDPCylon(int Wait){
  LDPWait=Wait;
  LDPCounter=0;
  LDPPosition=0;
  LDPMode=4;
}

void setLDPAltFlash(int Wait){
  LDPWait=Wait;
  LDPCounter=0;
  LDPMode=5;
}

void SetLDPColour1(int r, int g, int b){
  LDPColor1=Color(r, g, b);
  LDPColor1D=Color(r/4, g/4, b/4);
  LDPColor1d=Color(r/10, g/10, b/10);
}

void SetLDPColour2(int r, int g, int b){
  LDPColor2=Color(r, g, b);
}
  
void LDPOn(){
  for (int i=0; i<13; i++) {
    LDP13.setPixelColor(i, LDPColor1);
  }
}

void LDPOn2(){
  for (int i=0; i<13; i++) {
    LDP13.setPixelColor(i, LDPColor2);
  }
}

void LDPOff(){
  for (int i=0; i<13; i++) {
    LDP13.setPixelColor(i, Color(0, 0, 0));
  }
}

void LDPScan(){
  if(LDPCounter==0){
    LDPOn2();
    LDP13.setPixelColor(5-LDPPosition, LDPColor1d);
    LDP13.setPixelColor(6-LDPPosition, LDPColor1);
    LDP13.setPixelColor(6+LDPPosition, LDPColor1);
    LDP13.setPixelColor(7+LDPPosition, LDPColor1d);
    if(LDPPosition>=1){
      LDP13.setPixelColor(6, LDPColor1);
    }
    if(LDPPosition>=2){
      LDP13.setPixelColor(5, LDPColor1);
      LDP13.setPixelColor(7, LDPColor1);
    }
    if(LDPPosition>=3){
      LDP13.setPixelColor(4, LDPColor1);
      LDP13.setPixelColor(8, LDPColor1);
    }
    if(LDPPosition>=4){
      LDP13.setPixelColor(3, LDPColor1);
      LDP13.setPixelColor(9, LDPColor1);
    }
    if(LDPPosition>=5){
      LDP13.setPixelColor(2, LDPColor1);
      LDP13.setPixelColor(10, LDPColor1);
    }
  }
  if(LDPCounter>=LDPWait){
    if(LDPDir==true){
      if(LDPPosition==5){
        LDPDir=false;
        LDPPosition--;
      }else{
        LDPPosition++;
      }
    }else{
      if(LDPPosition==0){
        LDPDir=true;
        LDPPosition++;
      }else{
        LDPPosition--;
      }        
    }
    LDPCounter=0;
  }else{
    LDPCounter++;
  }
}

void LDPKitt(){
  if(LDPCounter==0){
    LDPOn2();
    if(LDPDir==true){
      if(LDPPosition<11){
        LDP13.setPixelColor(LDPPosition, LDPColor1d);
        LDP13.setPixelColor(1+LDPPosition, LDPColor1D);
        LDP13.setPixelColor(2+LDPPosition, LDPColor1);
        LDPPosition++;
      }else{
        LDP13.setPixelColor(10, LDPColor1D);
        LDP13.setPixelColor(11, LDPColor1);
        LDP13.setPixelColor(12, LDPColor1D);
        LDPDir=false;
      }
    }else{
      if(LDPPosition>0){
        LDP13.setPixelColor(LDPPosition-1, LDPColor1);
        LDP13.setPixelColor(1+LDPPosition-1, LDPColor1D);
        LDP13.setPixelColor(2+LDPPosition-1, LDPColor1d);
        LDPPosition--;
      }else{
        LDP13.setPixelColor(0, LDPColor1D);
        LDP13.setPixelColor(1, LDPColor1);
        LDP13.setPixelColor(2, LDPColor1D);
        LDPDir=true;
      }            
    }
  }
  if(LDPCounter>=LDPWait){
    LDPCounter=0;
  }else{
    LDPCounter++;
  }
}

void LDPCylon(){
  if(LDPCounter==0){
    LDPOn2();
    LDP13.setPixelColor(LDPPosition, LDPColor1d);
    LDP13.setPixelColor(1+LDPPosition, LDPColor1);
    LDP13.setPixelColor(2+LDPPosition, LDPColor1);
    LDP13.setPixelColor(3+LDPPosition, LDPColor1d);
  }
  if(LDPCounter>=LDPWait){
    if(LDPDir==true){
      if(LDPPosition==9){
        LDPDir=false;
        LDPPosition--;
      }else{
        LDPPosition++;
      }
    }else{
      if(LDPPosition==0){
        LDPDir=true;
        LDPPosition++;
      }else{
        LDPPosition--;
      }        
    }
    LDPCounter=0;
  }else{
    LDPCounter++;
  }
}

void LDPAltFlash(){
  if(LDPCounter==0){
    if(LDPAltState==true){
      LDPOn2();
      LDPAltState=false;
    }else{
      LDPOn();
      LDPAltState=true;
    }  
  }  
  if(LDPCounter>=LDPWait){
    LDPCounter=0;
  }else{
    LDPCounter++;
  }
}

void CS(){
  switch (CSMode) {
  case 0:    // your hand is on the sensor
    CSOff();
    break;
  case 1:    // your hand is close to the sensor
    CSOn();
    break;
  case 2:    // your hand is a few inches from the sensor
    CSScan();
    break;
  case 3:    // your hand is nowhere near the sensor
    CSKitt();
    break;
  case 4:    // your hand is nowhere near the sensor
    CSRandom();
    break;
  case 5:    // your hand is nowhere near the sensor
    CSAltFlash();
    break;
  case 6:    // your hand is nowhere near the sensor
    CSAllRandom();
    break;
  } 
}

void setCSOff(){
  CSCounter=0;
  CSMode=0;
  CSOff();
}

void setCSOn(){
  CSCounter=0;
  CSMode=1;
  CSOn();
}

void setCSScan(int Wait){
  CSWait=Wait;
  CSCounter=0;
  CSPosition=0;
  CSMode=2;
}

void setCSKitt(int Wait){
  CSWait=Wait;
  CSCounter=0;
  CSPosition=0;
  CSMode=3;
}

void setCSRandom(int Wait, int No){
  CSRandomNo=No;
  CSWait=Wait;
  CSCounter=0;
  CSMode=4;
}

void setCSAltFlash(int Wait){
  CSWait=Wait;
  CSCounter=0;
  CSMode=5;
}

void setCSAllRandom(int Wait, int No){
  CSRandomNo=No;
  CSWait=Wait;
  CSCounter=0;
  CSMode=6;
}

void SetCSColour1(int r, int g, int b){
  CSColor1=Color(r, g, b);
}

void SetCSColour2(int r, int g, int b){
  CSColor2=Color(r, g, b);
}

void CSOn(){
  for (int i=0; i<12; i++){
    CoinS12.setPixelColor(i, CSColor1);
  }
}

void CSOn2(){
  for (int i=0; i<12; i++){
    CoinS12.setPixelColor(i, CSColor2);
  }
}

void CSOff(){
  for (int i=0; i<12; i++){
    CoinS12.setPixelColor(i, Color(0, 0, 0));
  }
}

void CSScan(){
  if(CSCounter>=CSWait){
    CSOn2();
    for(int i=0; i<CSPosition; i++){
      CoinS12.setPixelColor((2*i), CSColor1);
      CoinS12.setPixelColor((2*i)+1, CSColor1);
    }
    if(CSDir==true){
      if(CSPosition==6){
        CSDir=false;
        CSPosition--;
      }else{
        CSPosition++;
      }
    }else{
      if(CSPosition==0){
        CSDir=true;
        CSPosition++;
      }else{
        CSPosition--;
      }        
    }
    CSCounter=0;
  }else{
    CSCounter++;
  }
}

void CSKitt(){
  if(CSCounter>=CSWait){
    CSOn2();
    CoinS12.setPixelColor(2*CSPosition, CSColor1);
    CoinS12.setPixelColor((2*CSPosition)+1, CSColor1);
    if(CSDir==true){
      if(CSPosition==5){
        CSDir=false;
        CSPosition--;
      }else{
        CSPosition++;
      }
    }else{
      if(CSPosition==0){
        CSDir=true;
        CSPosition++;
      }else{
        CSPosition--;
      }        
    }
    CSCounter=0;
  }else{
    CSCounter++;
  }
}

void CSRandom(){
  if(CSCounter>=CSWait){
    CSOn2();
    for(int i=0; i<CSRandomNo; i++){
      int Temp=random(6);
      CoinS12.setPixelColor(2*Temp, CSColor1);
      CoinS12.setPixelColor((2*Temp)+1, CSColor1);
    }
    CSCounter=0;
  }else{
    CSCounter++;
  }
}

void CSAllRandom(){
  if(CSCounter>=CSWait){
    CSOn2();
    for(int i=0; i<CSRandomNo; i++){
      CoinS12.setPixelColor(random(12), random(16777216));
    }
    CSCounter=0;
  }else{
    CSCounter++;
  }
}

void CSAltFlash(){
  if(CSCounter==0){
    if(CSAltState==true){
      for (int i=0; i <3; i++) {
        CoinS12.setPixelColor((4*i), CSColor1);
        CoinS12.setPixelColor((4*i)+1, CSColor1);
        CoinS12.setPixelColor((4*i)+2, CSColor2);
        CoinS12.setPixelColor((4*i)+3, CSColor2);
      }
      CSAltState=false;
    }else{
      for (int i=0; i <3; i++) {
        CoinS12.setPixelColor((4*i), CSColor2);
        CoinS12.setPixelColor((4*i)+1, CSColor2);
        CoinS12.setPixelColor((4*i)+2, CSColor1);
        CoinS12.setPixelColor((4*i)+3, CSColor1);
      }
      CSAltState=true;
    }
  }
  if(CSCounter>=CSWait){
    CSCounter=0;
  }else{
    CSCounter++;
  }
}

void colorWipe(uint32_t c) {
  for (int i=0; i < 25; i++) {
    if(i<13){
      LDP13.setPixelColor(i, c);
    } else {
      CoinS12.setPixelColor(i-13, c);
    }
  }
  CoinS12.show();
  LDP13.show();
}

uint32_t Color(byte r, byte g, byte b){  // Create a 24 bit color value from R,G,B
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

int ChargeLevelValue(){  //  Function to return 0-20 value of voltage between max and min.
  int Value = 0;                                  // Set the container to Zero
  for(int i=0; i<4; i++){
    Value = Value + analogRead(DividerIn);        // Read 4 times and sum total
  }
  Value = Value/4;                                // Divide by 4 to get average
  int NumberValue = map(Value, Min, Max, 0, 20);  //  Map the read average Value in the range between max and min (now in 0-1024 format)
  return(NumberValue);                            //  Send back NumberValue
}

void Flash(){
  AllOff();
  AllOn();
  AllOff();
}

void TwoAcross(){
  for(int repeat=0; repeat<7; repeat++) {
    for(int row=0; row<4; row++) {
      CBI.setLed(0,row,repeat-1,false);
      CBI.setLed(0,row,repeat-3,false);
      CBI.setLed(0,row,repeat,true);
      CBI.setLed(0,row,repeat-2,true);
    }
    LEDNoCBIDelay(200);
  }
}

void TwoDown(){
  CBI.setRow(0,0,B11111000);
  CBI.setRow(0,1,B00000000); 
  CBI.setRow(0,2,B00000000);
  CBI.setRow(0,3,B00000000);  
  LEDNoCBIDelay(200);
  CBI.setRow(0,0,B00000000);
  CBI.setRow(0,1,B11111000);  
  LEDNoCBIDelay(200);
  CBI.setRow(0,0,B11111000);
  CBI.setRow(0,1,B00000000); 
  CBI.setRow(0,2,B11111000); 
  LEDNoCBIDelay(200);
  CBI.setRow(0,0,B00000000);
  CBI.setRow(0,1,B11111000); 
  CBI.setRow(0,2,B00000000);
  CBI.setRow(0,3,B11111000);  
  LEDNoCBIDelay(200);
  CBI.setRow(0,1,B00000000); 
  CBI.setRow(0,2,B11111000);
  CBI.setRow(0,3,B00000000);  
  LEDNoCBIDelay(200);
  CBI.setRow(0,2,B00000000);
  CBI.setRow(0,3,B11111000);  
  LEDNoCBIDelay(200);  
}

void BoxFlash(){
  for(int repeat=0;repeat<8;repeat++) {
    CBI.setRow(0,0,B11111000);
    CBI.setRow(0,1,B10001000); 
    CBI.setRow(0,2,B10001000);
    CBI.setRow(0,3,B11111000);
    LEDNoCBIDelay(300);
    CBI.setRow(0,0,B00000000);
    CBI.setRow(0,1,B01110000); 
    CBI.setRow(0,2,B01110000);
    CBI.setRow(0,3,B00000000);
    LEDNoCBIDelay(300);
  } 
}

void AltRow(){
  for(int repeat=0;repeat<8;repeat++) {
    CBI.setRow(0,0,B11111000);
    CBI.setRow(0,1,B00000000); 
    CBI.setRow(0,2,B11111000);
    CBI.setRow(0,3,B00000000);
    LEDNoCBIDelay(300);
    CBI.setRow(0,0,B00000000);
    CBI.setRow(0,1,B11111000); 
    CBI.setRow(0,2,B00000000);
    CBI.setRow(0,3,B11111000);
    LEDNoCBIDelay(300);
  } 
}

void AltCol(){
  for(int repeat=0;repeat<8;repeat++) {
    CBI.setRow(0,0,B10101000);
    CBI.setRow(0,1,B10101000); 
    CBI.setRow(0,2,B10101000);
    CBI.setRow(0,3,B10101000);
    LEDNoCBIDelay(300);
    CBI.setRow(0,0,B01010000);
    CBI.setRow(0,1,B01010000); 
    CBI.setRow(0,2,B01010000);
    CBI.setRow(0,3,B01010000);
    LEDNoCBIDelay(300);
  } 
}

void ArrowsDown(){
  CBI.setRow(0,0,B00100000);
  CBI.setRow(0,1,B00000000); 
  CBI.setRow(0,2,B00000000);
  CBI.setRow(0,3,B00000000);
  LEDNoCBIDelay(200);
  CBI.setRow(0,0,B01010000);
  CBI.setRow(0,1,B00100000); 
  CBI.setRow(0,2,B00000000);
  CBI.setRow(0,3,B00000000);
  LEDNoCBIDelay(200);
  CBI.setRow(0,0,B10001000);
  CBI.setRow(0,1,B01010000); 
  CBI.setRow(0,2,B00100000);
  CBI.setRow(0,3,B00000000);
  LEDNoCBIDelay(200);
  for(int repeat=0;repeat<8 ;repeat++){
    CBI.setRow(0,0,B00100000);
    CBI.setRow(0,1,B10001000); 
    CBI.setRow(0,2,B01010000);
    CBI.setRow(0,3,B00100000);
    LEDNoCBIDelay(200);
    CBI.setRow(0,0,B01010000);
    CBI.setRow(0,1,B00100000); 
    CBI.setRow(0,2,B10001000);
    CBI.setRow(0,3,B01010000);
    LEDNoCBIDelay(200);
    CBI.setRow(0,0,B10001000);
    CBI.setRow(0,1,B01010000); 
    CBI.setRow(0,2,B00100000);
    CBI.setRow(0,3,B10001000);
    LEDNoCBIDelay(200);
  }
  CBI.setRow(0,0,B00000000);
  CBI.setRow(0,1,B10001000); 
  CBI.setRow(0,2,B01010000);
  CBI.setRow(0,3,B00100000);
  LEDNoCBIDelay(200);
  CBI.setRow(0,1,B00000000); 
  CBI.setRow(0,2,B10001000);
  CBI.setRow(0,3,B01010000);
  LEDNoCBIDelay(200);
  CBI.setRow(0,2,B00000000);
  CBI.setRow(0,3,B10001000);
  LEDNoCBIDelay(200); 
  CBI.setRow(0,3,B00000000);
  LEDNoCBIDelay(200); 
}

void AltLED(){
  for(int repeat=0;repeat<8;repeat++){
    CBI.setRow(0,0,B10101000);
    CBI.setRow(0,1,B01010000); 
    CBI.setRow(0,2,B10101000);
    CBI.setRow(0,3,B01010000);
    LEDNoCBIDelay(300);
    CBI.setRow(0,0,B01010000);
    CBI.setRow(0,1,B10101000); 
    CBI.setRow(0,2,B01010000);
    CBI.setRow(0,3,B10101000);
    LEDNoCBIDelay(300);
  }
}

void ChargeLevel(int Charge){
  AllOff();
  if(Charge < 6){
    AllOff3();
  }else if(Charge < 11){
    OneOn3();
  }else if(Charge < 16){
    TwoOn3();
  }else{
    AllOn3();
  }
  LEDNoCBIDelay(2*LoopWait);
  int Check = 0;
  if(Charge > 0) {
    for(int row=0; row<4; row++){
      for(int col=0; col<5; col++){
        CBI.setLed(0,row,col,true);
        Check = Check+1;
        if(Check == Charge){
          col=5;
          row=4;
        }
        LEDNoCBIDelay(8*LoopWait);
      }
    }
  }else{
    LEDNoCBIDelay(30*LoopWait);  
  }
  LEDNoCBIDelay(12*LoopWait);
  for(int row=0; row<4; row++){
    for(int col=0; col<5; col++){
      LEDNoCBIDelay(LoopWait);
      CBI.setLed(0,(3-row),(4-col),false);
      LEDNoCBIDelay(2*LoopWait);
    }
  }
  AllOff3();
  LEDNoCBIDelay(8*LoopWait);
}

void RandomPrint(int RandomFactor, int Charge){
  if(RandomFactor == 0){
    CBI.setRow(0, 0, random(256));
    CBI.setRow(0, 1, random(256));
    CBI.setRow(0, 2, random(256));
    CBI.setRow(0, 3, random(256));
  } else if(RandomFactor == 1){
    CBI.setRow(0, 0, (random(256)|random(256)));
    CBI.setRow(0, 1, (random(256)|random(256)));
    CBI.setRow(0, 2, (random(256)|random(256)));
    CBI.setRow(0, 3, (random(256)|random(256)));
  }
  if(Charge < 6){
    AllOff3();
  }else if(Charge < 11){
    OneOn3();
  }else if(Charge < 16){
    TwoOn3();
  }else{
    AllOn3();
  }
}
  
void AllOn(){
  CBI.setRow(0,0,255);
  CBI.setRow(0,1,255); 
  CBI.setRow(0,2,255);
  CBI.setRow(0,3,255);
  AllOn3();
  LEDNoCBIDelay(LoopWait);
}

void AllOff(){
  CBI.clearDisplay(0);
  LEDNoCBIDelay(LoopWait);
}

void AllOn3(){
  CBI.setLed(0, 4, 5, true);
  CBI.setLed(0, 5, 5, true);
  CBI.setLed(0, 6, 5, true);
}

void TwoOn3(){
  CBI.setLed(0, 4, 5, false);
  CBI.setLed(0, 5, 5, true);
  CBI.setLed(0, 6, 5, true);
}

void OneOn3(){
  CBI.setLed(0, 4, 5, false);
  CBI.setLed(0, 5, 5, false);
  CBI.setLed(0, 6, 5, true);
}

void AllOff3(){
  CBI.setLed(0, 4, 5, false);
  CBI.setLed(0, 5, 5, false);
  CBI.setLed(0, 6, 5, false);
}

void ChipOn(){
  CBI.shutdown(0,false); 
}
void ChipOff(){
  CBI.shutdown(0,true); 
}

void CBIDoorOpen(){
  BodyServos.moveTo(CBIDoor, 0, CBIDoorSpot[0], CBIDoorSpot[0]);
  CBIDoorCheck = true;
  ChipOn();
}

void CBIDoorClose(){
  BodyServos.moveTo(CBIDoor, 0, CBIDoorSpot[1], CBIDoorSpot[1]);
  CBIDoorCheck = false;    
  ChipOff();
}

void CBIDoorOpenClose(){
  if(CBIDoorCheck == false){
    CBIDoorOpen();
  } else {
    CBIDoorClose();
  }
}

void UArmTopOpen(){
  BodyServos.moveTo(UArmTop, UArmSpeed, UArmTopSpot[1], UArmTopSpot[0]);
  UArmTopCheck = true;
}

void UArmTopClose(){
  BodyServos.moveTo(UArmTop, UArmSpeed, UArmTopSpot[0], UArmTopSpot[1]);
  UArmTopCheck = false;
}

void UArmTopOpenClose(){
  if(UArmTopCheck == false){
    UArmTopOpen();
  } else {
    UArmTopClose();
  }
}

void UArmBottomOpen(){
  BodyServos.moveTo(UArmBottom, UArmSpeed, UArmBottomSpot[1], UArmBottomSpot[0]);
  UArmBottomCheck = true;
}

void UArmBottomClose(){
  BodyServos.moveTo(UArmBottom, UArmSpeed, UArmBottomSpot[0], UArmBottomSpot[1]);
  UArmBottomCheck = false;
}

void UArmBottomOpenClose(){
  if(UArmBottomCheck == false){
    UArmBottomOpen();
  } else {
    UArmBottomClose();
  }
}
void LeftLDoorOpen(){
  BodyServos.moveTo(LeftLDoor, 0, LeftLDoorSpot[0], LeftLDoorSpot[0]);
  LeftLDoorCheck = true;
}

void LeftLDoorClose(){
  BodyServos.moveTo(LeftLDoor, 0, LeftLDoorSpot[1], LeftLDoorSpot[1]);
  LeftLDoorCheck = false;
}

void LeftLDoorOpenClose(){
  if(LeftLDoorCheck == false){
    LeftLDoorOpen();
  } else {
    LeftLDoorClose();
  }
}

void RightLDoorOpen(){
  BodyServos.moveTo(RightLDoor, 0, RightLDoorSpot[0], RightLDoorSpot[0]);
  RightLDoorCheck = true;
}

void RightLDoorClose(){
  BodyServos.moveTo(RightLDoor, 0, RightLDoorSpot[1], RightLDoorSpot[1]);
  RightLDoorCheck = false;
}

void RightLDoorOpenClose(){
  if(RightLDoorCheck == false){
    RightLDoorOpen();
  } else {
    RightLDoorClose();
  }
}

void SmallDoorOpen(){
  BodyServos.moveTo(SmallDoor, 0, SmallDoorSpot[0], SmallDoorSpot[0]);
  SmallDoorCheck = true;
}

void SmallDoorClose(){
  BodyServos.moveTo(SmallDoor, 0, SmallDoorSpot[1], SmallDoorSpot[1]);
  SmallDoorCheck = false;
}

void SmallDoorOpenClose(){
  if(SmallDoorCheck == false){
    SmallDoorOpen();
  } else {
    SmallDoorClose();
  }
}

void BigDoorOpen(){
  BodyServos.moveTo(BigDoor, 0, BigDoorSpot[0], BigDoorSpot[0]);
  BigDoorCheck = true;
}

void BigDoorClose(){
  BodyServos.moveTo(BigDoor, 0, BigDoorSpot[1], BigDoorSpot[1]);
  BigDoorCheck = false;
}

void BigDoorOpenClose(){
  if(BigDoorCheck == false){
    BigDoorOpen();
  } else {
    BigDoorClose();
  }
}

void receive(int numBytes) {}
