#ifndef HAB_TIMER_H
#define HAB_TIMER_H

#include <stdint.h>

void init_timers();

uint64_t MICROS();

uint64_t MILLIS();

void DELAY(volatile uint64_t mS);

void DELAY_MICROS(volatile uint64_t uS);

void while_equals_timeout(uint64_t timeout_ms, void* comp1, void* comp2,
                          void (*timeout_clbk)());
void while_not_equals_timeout(uint64_t timeout_ms, void* comp1, void* comp2,
                              void (*timeout_clbk)());

#endif
