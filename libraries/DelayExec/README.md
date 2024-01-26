DelayExec
=========

Execute functions in a deferred manner.


Installation
------------

Download a zip of this repository, then include it from the Arduino IDE.


Usage Example
-------------

```c++
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
```


Re-entrance
-----------

set_task() can be called from inside a function called by the scheduler.

Thus, the called function can itself (and at the last moment) decide if/when to
run again.


Details
-------

* The list of scheduled tasks is a queue, without pre-defined maximum size.

* The scheduler is called 50 times per second, giving it a precision of .02
second to start a task. Nevertheless this frequency has no impact on the code
logic, you may change it if you wish.


Example
-------

See [examples/02_simple/02_simple.ino](examples/02_simple/02_simple.ino) for a
more elaborate example.
