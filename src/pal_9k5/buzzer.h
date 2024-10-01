#ifndef BUZZER_H
#define BUZZER_H

#include "status.h"
#include "stdint.h"

#define BUZZER_QUEUE_LEN 20

typedef enum {
    BUZZER_OFF = 0,
    BUZZER_ON = 1,
} BuzzerState;

enum {
    BUZZER_FREQ_8KHZ = 8000,
    BUZZER_FREQ_4KHZ = 4000,
    BUZZER_FREQ_2KHZ = 2000,
    BUZZER_FREQ_1KHZ = 1000,
    BUZZER_FREQ_500HZ = 500,
    BUZZER_FREQ_250HZ = 250,
};

typedef enum {
    BUZZER_SOUND_REST_1S,
    BUZZER_SOUND_REST_500MS,
    BUZZER_SOUND_REST_200MS,
    BUZZER_SOUND_BEEP,
    BUZZER_SOUND_DESCENDING_BEEP,
    BUZZER_SOUND_LONG_BEEP,
    BUZZER_SOUND_LONG_DESCENDING_BEEP,
    BUZZER_SOUND_DOUBLE_BEEP,
    BUZZER_SOUND_INIT,
    BUZZER_SOUND_SONG,
} BuzzerSound;

enum {
    NOTE_C6,
    NOTE_CS6,
    NOTE_D6,
    NOTE_DS6,
    NOTE_E6,
    NOTE_F6,
    NOTE_FS6,
    NOTE_G6,
    NOTE_GS6,
    NOTE_A6,
    NOTE_AS6,
    NOTE_B6,
    NOTE_C7,
    NOTE_CS7,
    NOTE_D7,
    NOTE_DS7,
    NOTE_E7,
    NOTE_F7,
    NOTE_FS7,
    NOTE_G7,
    NOTE_GS7,
    NOTE_A7,
    NOTE_AS7,
    NOTE_B7,
};

enum {
    NOTE_WHOLE = 16,
    NOTE_THREE_QUARTERS = 12,
    NOTE_HALF = 8,
    NOTE_QUARTER_AND_HALF = 6,
    NOTE_QUARTER = 4,
    NOTE_EIGHTH = 2,
};

typedef struct {
    uint32_t freq;
    uint32_t duration;
    uint8_t pair;  // Is note in a pair, discluding last note
} Note;

// C6 to B7
__attribute__((unused)) static uint32_t note_table[] = {
    1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
    2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,
};

Status buzzer_init();

void buzzer_task();

Status buzzer_set(uint32_t freq);

Status buzzer_clear();

void buzzer_play(BuzzerSound sound);

#endif  // BUZZER_H