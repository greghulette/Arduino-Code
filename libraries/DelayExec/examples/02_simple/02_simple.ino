// 02_simple.ino

// Example ino file provided with DelayExec library.

/*
  Copyright 2021 Sébastien Millet

  `DelayExec' is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  `DelayExec' is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program. If not, see
  <https://www.gnu.org/licenses>.
*/

#include "DelayExec.h"
#include <Arduino.h>

extern DelayExec dx;

const char *str1 = "string1";
const char *str2 = "string2";

void delayed_func(void *data) {
    static unsigned long last_t = 0;
    unsigned long t = millis();

    Serial.print("delayed_func():  elapsed = ");
    Serial.print(t - last_t);
    Serial.print(", str: ");
    Serial.print((char *)data);
    Serial.print("\n");
    last_t = t;
}

void delayed_func2(void *data) {
    static unsigned long last_t = 0;
    unsigned long t = millis();

    static unsigned long d = 63;

    Serial.print("delayed_func2(): elapsed = ");
    Serial.print(t - last_t);
    Serial.print(", str: ");
    Serial.print((char *)data);
    Serial.print("\n");
    dx.set_task(d, delayed_func2, data, false);
    d <<= 1;
    if (d >= 2400)
        d = 63;
    last_t = t;
}

void setup() {
    Serial.begin(115200);

    dx.set_task(500, delayed_func, (void*)str1, true);
    dx.set_task(0, delayed_func2, (void*)str2, false);
    dx.activate();
}

void loop() {

}

// vim: ts=4:sw=4:tw=80:et
