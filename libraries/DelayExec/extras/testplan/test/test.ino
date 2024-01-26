// test.ino

// Sketch to run test plan of `DelayExec' library

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
#include "MemoryFree.h"
#include <Arduino.h>

static char serial_printf_buffer[120];

static void serial_printf(const char* fmt, ...)
     __attribute__((format(printf, 1, 2)));

static void serial_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(serial_printf_buffer, sizeof(serial_printf_buffer), fmt, args);
    va_end(args);

    serial_printf_buffer[sizeof(serial_printf_buffer) - 1] = '\0';
    Serial.print(serial_printf_buffer);
}

static void serial_begin(long speed) {
    Serial.begin(speed);
}

extern DelayExec dx;

const char *str1 = "string1";
const char *str2 = "string2";

void delayed_func(void *data) {
    serial_printf("delayed_func(): %s\n", (char *)data);
}

void delayed_func2(void *data) {
    serial_printf("delayed_func2(): enter\n");
    dx.set_task(800, delayed_func2, nullptr, false);
    delay(500);
    serial_printf("delayed_func2(): leave\n");
}

void setup() {
    serial_begin(115200);
    delay(500);
    serial_printf("\n----- BEGIN TEST -----\n");

    dx.set_task(500, delayed_func, (void*)str1, true);
    dx.set_task(1000, delayed_func, (void*)str2, true);
    dx.set_task(0, delayed_func2, (void*)str2, false);
    dx.activate();
}

void loop() {
    static long unsigned last_t = 0;
    static int counter = 0;

    unsigned long t = millis();
    if (t - last_t > 1000) {
        last_t = t;
        ++counter;
        serial_printf("Tick: %u, free memory: %d\n",
                dx.get_tick(), freeMemory());
        serial_printf("Count added: %d, count deleted: %d, count: %d\n",
                dx.get_count_added(), dx.get_count_deleted(),
                dx.get_task_count());

#ifdef DELAYEXEC_TEST_RESET
        if (counter == 7) {
            dx.inactivate();
        } else if (counter == 10) {
            dx.activate();
        } else if (counter == 14) {
            dx.delete_all_tasks();
        }
#endif

        if (counter == 20) {
            serial_printf("----- END TEST -----\n");
        }
    }
}

// vim: ts=4:sw=4:tw=80:et
