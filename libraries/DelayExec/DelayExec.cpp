// DelayExec.cpp

// See README.md about the purpose of this library

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

#define DELAYEXEC_ASSERT_OUTPUT_TO_SERIAL

#define assert(cond) { \
    if (!(cond)) { \
        assert_failed(__LINE__); \
    } \
}

void assert_failed(int line) {
#ifdef DELAYEXEC_ASSERT_OUTPUT_TO_SERIAL
    Serial.print("\nDelayExec.cpp:");
    Serial.print(line);
    Serial.println(": assertion failed, aborted.");
#endif
    while (1)
        ;
}

DelayExec dx;

ISR(TIMER1_COMPA_vect) {
    unsigned long t0 = millis();

    dx.inc_tick();

    if (dx.get_commit_pending_changes_underway())
        return;

    uint16_t current_tick = dx.get_tick();

    Task *ptask = dx.get_head();
    while (ptask) {
        if (!ptask->run_underway && !ptask->to_delete &&
                t0 >= ptask->start_time) {

            bool is_periodic = ptask->is_periodic;
            if (!is_periodic)
                ptask->to_delete = 1;
            ptask->run_underway = 1;

            sei();
            ptask->func(ptask->data);
            cli();

            if (is_periodic) {
                ptask->start_time = t0 + ptask->period;
                ptask->run_underway = 0;
            }

                // If the tick has changed since we entered the ISR, this means
                // func() executions did last more than the timer period (20
                // milliseconds). In this case, the Task chain is not guaranteed
                // to be safe (the current Task could be already deleted, or the
                // following Task, etc.)
                // We therefore must quit immediately.
            if (current_tick != dx.get_tick()) {
                sei();
                return;
            }

        }
        ptask = ptask->next;
    }

    sei();

    dx.commit_pending_changes();
}

Task::Task(unsigned long arg_start_time, unsigned long arg_period,
        void (*arg_func)(void *data), void *arg_data, bool arg_is_periodic):
        start_time(arg_start_time),
        period(arg_period),
        func(arg_func),
        data(arg_data),
        run_underway(0),
        to_delete(0),
        is_periodic(arg_is_periodic ? 1 : 0),
        next(nullptr) {
    assert(func);
}

static Task* get_tail(const Task* head) {
    const Task* ptask = head;
    if (ptask) {
        while (ptask->next)
            ptask = ptask->next;
    }
    return (Task*)ptask;
}

DelayExec::DelayExec()
        :active(false),
        tick(0),
        commit_pending_changes_underway(false),
        head(nullptr),
        head_pending_add(nullptr)
#ifdef DELAYEXEC_COUNT
        ,count_added(0)
        ,count_deleted(0)
#endif
{

}

DelayExec::~DelayExec() {

}

void DelayExec::activate() {
    if (active)
        return;

    cli();

        // From https://www.instructables.com/id/Arduino-Timer-Interrupts/
        // Arduino timer maths:
        //   FREQ = ARDCLOCKFREQ / (prescaler * (comparison match register + 1))
        //   With ARDCLOCKFREQ = 16000000 (Arduino board built-in frequency)

        // Setup timer1 interrupts
        // freq below = 16000000 / (256 * (1249 + 1)) = 50Hz
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 =  0;
    OCR1A =  1249;         // Comparison match register
    TCCR1B |= 1 << WGM12;  // Turn on CTC mode
    TCCR1B |= 1 << CS12;   // Set CS10 for 256 prescaler
    TIMSK1 |= 1 << OCIE1A; // Enable timer compare interrupt

    active = true;
    sei();
}

void DelayExec::inactivate() {
    if (!active)
        return;

    cli();

    // From
    //   https://forum.arduino.cc/t/actually-disabling-timer-interrupts/217915
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 &= ~(1 << OCIE1A);

    active = false;
    sei();
}

void DelayExec::delete_all_tasks() {

    cli(); // vvvvvvvvvv

    Task* ptask = head;
    while (ptask) {
        ptask->to_delete = 1;
        ptask = ptask->next;
    }
    ptask = head_pending_add;
    while (ptask) {
        ptask->to_delete = 1;
        ptask = ptask->next;
    }

    sei(); // ^^^^^^^^^^

}

void DelayExec::commit_pending_changes() {

// We are using the variable commit_pending_changes_underway in order to avoid
// race conditions (this function can be interrupted... by an interrupt). We
// don't want to leave cli() in the whole function execution, cause it'd reduce
// the timer precision. Hence we prefer to manage this complicated logic, all
// this, to have interrupts activated inside all of commit_pending_changes
// execution.
    cli();
    if (get_commit_pending_changes_underway()) {
        sei();
        return;
    }
    commit_pending_changes_underway = true;
    sei();

// 1: commit deletions

    Task *prev_ptask = head;
    Task *ptask = head;
    while (ptask) {
        if (ptask->to_delete) {
#ifdef DELAYEXEC_COUNT
            ++count_deleted;
#endif
            Task *ptask_to_delete = ptask;
            if (ptask == head) {
                assert(ptask == prev_ptask);
                head = ptask->next;
                prev_ptask = head;
                ptask = head;
            } else {
                ptask = ptask->next;
                prev_ptask->next = ptask;
            }
            delete ptask_to_delete;
        } else {
            prev_ptask = ptask;
            ptask = ptask->next;
        }
    }

// 2: commit additions

#ifdef DELAYEXEC_COUNT
    const Task* ptx = head_pending_add;
    if (ptx) {
        ++count_added;
        while (ptx->next) {
            ++count_added;
            ptx = ptx->next;
        }
    }
#endif
    Task *ptail = get_tail(head_pending_add);
    if (ptail) {
        ptail->next = head;
        head = head_pending_add;
        head_pending_add = nullptr;
    }

    commit_pending_changes_underway = false;
}

int DelayExec::get_task_count() const {
    int count_task = 0;
    const Task* ptask = head;
    if (ptask) {
        ++count_task;
        while (ptask->next) {
            ++count_task;
            ptask = ptask->next;
        }
    }
    return count_task;
}

int DelayExec::set_task(unsigned long delay, void (*func)(void *data),
        void *data, bool respawn) {
    unsigned long actual_time = millis();

    Task *ptask = new Task(actual_time + delay, delay, func,
            data, respawn);
    if (!ptask)
        return DELAYEXEC_ENOMEMORY;

    cli(); // vvvvvvvvvv

    Task *ptail = get_tail(head_pending_add);

    if (ptail) {
        assert(!ptail->next);
        ptail->next = ptask;
    } else {
        assert(!head_pending_add);
        head_pending_add = ptask;
    }

    sei(); // ^^^^^^^^^^

    return DELAYEXEC_EOK;
}

// vim: ts=4:sw=4:tw=80:et
