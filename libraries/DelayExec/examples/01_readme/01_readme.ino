// 01_readme.ino

#include "DelayExec.h"

extern DelayExec dx;

const char *str = "Hello world";
void my_scheduled_function(void *data) {
    Serial.println((char *)data);
}

void setup() {
    Serial.begin(9600);
    dx.set_task(1000, my_scheduled_function, (void*)str, true);
    dx.activate();
}

void loop() { }
