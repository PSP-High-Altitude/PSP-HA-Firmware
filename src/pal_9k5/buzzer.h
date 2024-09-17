#ifndef BUZZER_H
#define BUZZER_H

#include "status.h"

typedef enum {
    BUZZER_OFF = 0,
    BUZZER_ON = 1,
} BuzzerState;

typedef enum {
    BUZZER_FREQ_8KHZ = 125,
    BUZZER_FREQ_4KHZ = 250,
    BUZZER_FREQ_2KHZ = 500,
    BUZZER_FREQ_1KHZ = 1000,
    BUZZER_FREQ_500HZ = 2000,
    BUZZER_FREQ_250HZ = 4000,
} BuzzerFreq;

Status buzzer_set(BuzzerFreq freq);

Status buzzer_clear();

#endif  // BUZZER_H