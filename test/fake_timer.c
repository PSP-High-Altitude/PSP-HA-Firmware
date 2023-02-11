#include <stdint.h>

#include "timer.h"

// Simple counter to fake monotonically increasing time
static uint64_t s_ms = 0;

void initTimers() {}

void DELAY_NOP(uint16_t mS) {}

uint64_t MICROS() { return 1000 * s_ms++; }

uint64_t MILLIS() { return s_ms++; }

void DELAY(uint16_t mS) {}

void DELAY_MICROS(uint32_t uS) {}
