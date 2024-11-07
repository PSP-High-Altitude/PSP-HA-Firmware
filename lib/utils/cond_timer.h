#ifndef COND_TIMER_H
#define COND_TIMER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint64_t duration_us;
    uint64_t start_us;
    bool last_status;
} CondTimer;

/**
 * @brief Initializes a CondTimer instance with a specified duration.
 *
 * @param cond_timer Pointer to the CondTimer instance to initialize.
 * @param duration_ms Duration in milliseconds for which the condition must be
 * true for the condition to be considered as met.
 */
void cond_timer_init(CondTimer* cond_timer, uint64_t duration_ms);

/**
 * @brief Updates the condition timer based on the current status of the
 * condition.
 *
 * Checks if the condition has been true long enough to meet the timer's
 * duration threshold.
 *
 * @param cond_timer Pointer to the CondTimer instance to update.
 * @param status The current status of the condition (true or false).
 * @return true if the condition has been continuously true for at least the
 * configured duration of the timer; false otherwise.
 */
bool cond_timer_update(CondTimer* cond_timer, bool status);

#endif  // COND_TIMER_H
