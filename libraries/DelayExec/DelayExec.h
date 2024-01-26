// DelayExec.h

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

#ifndef _DELAYEXEC_H
#define _DELAYEXEC_H

#include <Arduino.h>

#define DELAYEXEC_COUNT // FIXME

// ****************************************************************************
// DELAYEXEC_TESTPLAN *********************************************************
#if DELAYEXEC_TESTPLAN == 1
#define DELAYEXEC_COUNT

#elif DELAYEXEC_TESTPLAN == 2
#define DELAYEXEC_COUNT
#define DELAYEXEC_TEST_RESET

#else // DELAYEXEC_TESTPLAN

#ifdef DELAYEXEC_TESTPLAN
#error "DELAYEXEC_TESTPLAN macro has an illegal value."
#endif
// DELAYEXEC_TESTPLAN *********************************************************
// ****************************************************************************

#endif // DELAYEXEC_TESTPLAN

#define DELAYEXEC_EOK       0
#define DELAYEXEC_ENOMEMORY 1

struct Task {
    unsigned long start_time;
    unsigned long period;
    void (*func)(void *data);
    void *data;
    byte run_underway :1;
    byte to_delete    :1;
    byte is_periodic  :1;
    Task* volatile next;

    Task(unsigned long arg_start_time, unsigned long arg_period,
            void (*arg_func)(void *data), void *arg_data, bool arg_is_periodic);
};

class DelayExec {
    private:
        bool active;
        volatile uint16_t tick;
        volatile bool commit_pending_changes_underway;
        Task* volatile head;
            // A task addition is done here (not in head), so as to be able to
            // schedule a new task while in an ISR.
        Task* volatile head_pending_add;
#ifdef DELAYEXEC_COUNT
        int count_added;
        int count_deleted;
#endif

    public:
        DelayExec();
        virtual ~DelayExec();

        void activate();
        void inactivate();
        bool is_active() const { return active; }

        Task *get_head() const { return head; }
        uint16_t get_tick() const { return tick; }
        void inc_tick() { ++tick; }

        int get_task_count() const;

#ifdef DELAYEXEC_COUNT
        int get_count_added() const { return count_added; }
        int get_count_deleted() const { return count_deleted; }
#endif

        bool get_commit_pending_changes_underway() const {
            return commit_pending_changes_underway;
        }
        void commit_pending_changes();

        virtual int set_task(unsigned long delay,
                void (*func)(void *data), void *data, bool respawn);

        void delete_all_tasks();
};

#endif // _DELAYEXEC_H

// vim: ts=4:sw=4:tw=80:et
