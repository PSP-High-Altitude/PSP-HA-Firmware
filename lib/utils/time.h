#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

void initTimers();

void DELAY_NOP(uint16_t mS);

uint64_t MICROS();

uint64_t MILLIS();

void DELAY(uint16_t mS);

void DELAY_MICROS(uint32_t uS);

#endif