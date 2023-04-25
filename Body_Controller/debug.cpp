#include "debug.h"


debugClass::debugClass(bool displayMsg){

};

//////////////////////////////////////////////////////////////////////
///*****             Debug Functions                          *****///
//////////////////////////////////////////////////////////////////////


void debugClass::DBG(const char *format, ...) {
        if (!debugflag)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void debugClass::DBG_1(const char *format, ...) {
        if (!debugflag1)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}
void debugClass::DBG_2(const char *format, ...) {
        if (!debugflag2)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void debugClass::ESPNOW(const char *format, ...) {
        if (!debugflag_espnow)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void debugClass::SERVO(const char *format, ...) {
        if (!debugflag_servo)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void debugClass::SERIAL_EVENT(const char *format, ...) {
        if (!debugflag_serial_event)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void debugClass::LOOP(const char *format, ...) {
        if (!debugflag_loop)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void debugClass::HTTP(const char *format, ...) {
        if (!debugflag_loop)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void debugClass::LORA(const char *format, ...) {
        if (!debugflag_loop)
                return;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
}

void debugClass::toggleDebug(){
  debugflag = !debugflag;
  if (debugflag == 1){
    Serial.println("Debugging Enabled");
    }
  else{
    Serial.println("Debugging Disabled");
  }
}

void debugClass::toggle_Debug1(){
  debugflag1 = !debugflag1;
  if (debugflag1 == 1){
    Serial.println("Debugging 1 Enabled");
    }
  else{
    Serial.println("Debugging 1 Disabled");
  }
}
void debugClass::toggle_Debug2(){
  debugflag2 = !debugflag2;
  if (debugflag2 == 1){
    Serial.println("Debugging 2 Enabled");
    }
  else{
    Serial.println("Debugging 2 Disabled");
  }
}

void debugClass::toggle_ESPNOW(){
  debugflag_espnow = !debugflag_espnow;
  if (debugflag_espnow == 1){
    Serial.println("ESP-NOW Debugging Enabled");
    }
  else{
    Serial.println("ESP-NOW Debugging Disabled");
  }
}

void debugClass::toggle_Servo(){
  debugflag_servo = !debugflag_servo;
  if (debugflag_servo == 1){
    Serial.println("Servo Debugging Enabled");
    }
  else{
    Serial.println("Servo Debugging Disabled");
  }
}

void debugClass::toggle_SerialEvent(){
  debugflag_serial_event = !debugflag_serial_event;
  if (debugflag_serial_event == 1){
    Serial.println("Serial Events Debugging Enabled");
    }
  else{
    Serial.println("Serial Events Debugging Disabled");
  }
}

void debugClass::toggle_Loop(){
  debugflag_loop = !debugflag_loop;
  if (debugflag_loop == 1){
    Serial.println("Main Loop Debugging Enabled");
    }
  else{
    Serial.println("Main Loop Debugging Disabled");
  }
}
void debugClass::toggle_HTTP(){
  debugflag_http = !debugflag_http;
  if (debugflag_http == 1){
    Serial.println("HTTP Debugging Enabled");
    }
  else{
    Serial.println("HTTP Debugging Disabled");
  }
}

void debugClass::toggle_LORA(){
  debugflag_lora = !debugflag_lora;
  if (debugflag_lora == 1){
    Serial.println("LoRa Debugging Enabled");
    }
  else{
    Serial.println("LoRa Debugging Disabled");
  }
}