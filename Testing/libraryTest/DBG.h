#include <Adafruit_NeoPixel.h>

#include <Adafruit_NeoPixel.h>

#ifndef DBG_h
#define DBG_h

#include "Arduino.h"

class DBG
{
public:
  boolean debugflag = 1;    // Normally set to 0, but leaving at 1 during coding and testing.
  boolean debugflag1 = 0;  // Used for optional level of debuging
  boolean debugflag_espnow = 0;
  boolean debugflag_servo = 0;
  boolean debugflag_serial_event = 0;
  boolean debugflag_loop = 0;
  boolean debugflag_http = 0;
  boolean debugflag_lora = 0;

  void DBG();
  // void DBG_1();
  // void DBG_HTTP();
  // void DBG_LORA();
  // void DBG_ESPNOW();
  // void DBG_SERVO();
  // void DBG_SERIAL_EVENT();
  // void DBG_LOOP();

  void toggleDebug();
  // void toggleDebug1();
  // void toggleDebug_HTTP();
  // void toggleDebug_LORA();
  // void toggleDebug_ESPNOW();
  // void toggleDebug_Servo();
  // void toggleDebug_SerialEvent();
  // void toggleDebug_Loop();



};
#endif