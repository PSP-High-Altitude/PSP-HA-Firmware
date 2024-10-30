#include "timer.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TIMER_NOISE_MAX_US (100)

volatile static uint64_t system_timestamp_us = 0;

void init_timers() { printf("Initializing timers\n"); };

uint64_t MICROS() { return system_timestamp_us++; }

uint64_t MILLIS() { return MICROS() / 1000; }

void DELAY(uint16_t mS) {
    uint64_t offset = 0;

    if (TIMER_NOISE_MAX_US) {
        offset = rand() % (2 * TIMER_NOISE_MAX_US);
        offset -= TIMER_NOISE_MAX_US;
    }

    system_timestamp_us += mS * 1000 + offset;
}

void DELAY_MICROS(uint32_t uS) { system_timestamp_us += uS; }
