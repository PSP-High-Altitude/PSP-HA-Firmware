#include "cond_timer.h"

#include "timer.h"

void cond_timer_init(CondTimer* cond_timer, uint64_t duration_ms) {
    cond_timer->duration_us = duration_ms * 1000;
    cond_timer->last_status = false;
}

void cond_timer_reset(CondTimer* cond_timer) {
    cond_timer->last_status = false;
}

bool cond_timer_update(CondTimer* cond_timer, bool status) {
    if (status) {
        if (!cond_timer->last_status) {
            // False -> True transition, so reset start time
            cond_timer->start_us = MICROS();
        }

        uint64_t elapsed_us = MICROS() - cond_timer->start_us;
        if (elapsed_us >= cond_timer->duration_us) {
            return true;
        }
    }

    // If this becomes false, start_us is automatically invalid
    cond_timer->last_status = status;

    return false;
}
