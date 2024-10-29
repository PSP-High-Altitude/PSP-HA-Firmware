#include "timer.h"

#include <stdint.h>
#include <stdio.h>

volatile static uint64_t system_timestamp_us = 0;

void init_timers() { printf("Initializing timers\n"); };

uint64_t MICROS() { return system_timestamp_us; }

uint64_t MILLIS() { return MICROS() / 1000; }

void DELAY(uint16_t mS) { system_timestamp_us += mS * 1000; }

void DELAY_MICROS(uint32_t uS) { system_timestamp_us += uS; }
