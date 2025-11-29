#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include <stdint.h>

#include "status.h"
#include "timer.h"

#define WAIT_COND_BLOCKING_TIMEOUT(cond, timeout, result) \
    do {                                                  \
        uint64_t start = MILLIS();                        \
        while (!(cond)) {                                 \
            if (MILLIS() - start > (timeout)) {           \
                result = STATUS_TIMEOUT_ERROR;            \
                break;                                    \
            }                                             \
        }                                                 \
    } while (0);

#endif  // UTILS_H