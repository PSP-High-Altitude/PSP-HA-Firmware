#include "buzzer.h"

#include "FreeRTOS.h"
#include "board.h"
#include "queue.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

QueueHandle_t buzzer_queue;

TIM_HandleTypeDef tim15_handle = {0};

#define GET_DIVIDER(base, freq) ((base) / (freq) - 1)
#define BASE_FREQ 50000000
#define GET_NOTE_TIME(bpm, note) ((note / 4.0) * (1000.0 / (bpm / 60.0)))

Status buzzer_init() {
    __HAL_RCC_TIM15_CLK_ENABLE();
    tim15_handle.Instance = TIM15;

    // Prescaler to 50MHz, period to 4khz
    // 50Mhz seems high, but we don't care about hitting low notes (they
    // don't sound good) and in return we get a higher resolution between
    // notes in the range we care about
    TIM_Base_InitTypeDef tim15_conf = {
        .Prescaler = (200000000 / BASE_FREQ) - 1,
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = GET_DIVIDER(BASE_FREQ, BUZZER_FREQ_4KHZ),
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };

    tim15_handle.Init = tim15_conf;

    TIM_OC_InitTypeDef tim15_oc_conf = {
        .OCMode = TIM_OCMODE_TOGGLE,
        .Pulse = 0,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET,
    };

    GPIO_InitTypeDef gpio_conf = {
        .Pin = PAL_GPIO_PIN(PIN_BUZZER),
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = GPIO_AF4_TIM15,
    };
    HAL_GPIO_Init(PAL_GPIO_PORT(PIN_BUZZER), &gpio_conf);

    HAL_TIM_OC_Init(&tim15_handle);
    HAL_TIM_OC_ConfigChannel(&tim15_handle, &tim15_oc_conf, TIM_CHANNEL_1);

    // Configure queue
    buzzer_queue = xQueueCreate(BUZZER_QUEUE_LEN, sizeof(BuzzerSound));
    if (buzzer_queue == NULL) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

Status buzzer_set(uint32_t freq) {
    // Initialize if not initialized
    if (tim15_handle.State == 0) {
        buzzer_init();
    }

    HAL_TIM_OC_Stop(&tim15_handle, TIM_CHANNEL_1);
    tim15_handle.Init.Period = GET_DIVIDER(BASE_FREQ, freq);
    HAL_TIM_OC_Init(&tim15_handle);
    HAL_TIM_OC_Start(&tim15_handle, TIM_CHANNEL_1);

    return STATUS_OK;
}

Status buzzer_clear() {
    HAL_TIM_OC_Stop(&tim15_handle, TIM_CHANNEL_1);
    return STATUS_OK;
}

static void sound_beep(uint16_t period) {
    buzzer_set(BUZZER_FREQ_4KHZ);
    DELAY(period);
    buzzer_clear();
    DELAY(period);
}

static void sound_descending_beep(uint16_t period) {
    buzzer_set(BUZZER_FREQ_4KHZ);
    DELAY(period / 2);
    buzzer_set(BUZZER_FREQ_2KHZ);
    DELAY(period / 2);
    buzzer_clear();
    DELAY(period);
}

static void sound_init() {
    buzzer_set(BUZZER_FREQ_1KHZ);
    DELAY(200);
    buzzer_set(BUZZER_FREQ_2KHZ);
    DELAY(200);
    buzzer_set(BUZZER_FREQ_4KHZ);
    DELAY(200);
    buzzer_clear();
    DELAY(200);
}

static void sound_song() {
    int num_notes = 75;
    int bpm = 200;
    Note notes[] = {
        // https://cdn.ustatik.com/musescore/scoredata/g/c5faa2ffb201947d11981100a7f8643497e485ee/score_0.svg?no-cache=1715695311

        // First line
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_CS7, NOTE_QUARTER, 0},
        {NOTE_CS7, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_HALF, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_E6, NOTE_QUARTER_AND_HALF, 0},
        {NOTE_E6, NOTE_EIGHTH, 0},
        {NOTE_B6, NOTE_EIGHTH, 1},
        {NOTE_A6, NOTE_EIGHTH, 1},
        {NOTE_GS6, NOTE_EIGHTH, 1},
        {NOTE_FS6, NOTE_EIGHTH, 0},
        {NOTE_E6, NOTE_THREE_QUARTERS, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_CS7, NOTE_QUARTER, 0},
        {NOTE_CS7, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_HALF, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_QUARTER, 0},
        {NOTE_FS6, NOTE_EIGHTH, 0},
        {NOTE_GS6, NOTE_EIGHTH, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_DS6, NOTE_QUARTER, 0},

        // Second line
        {NOTE_E6, NOTE_THREE_QUARTERS, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_EIGHTH, 1},
        {NOTE_GS6, NOTE_EIGHTH, 1},
        {NOTE_FS6, NOTE_EIGHTH, 1},
        {NOTE_GS6, NOTE_EIGHTH, 0},
        {NOTE_A6, NOTE_HALF, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_EIGHTH, 1},
        {NOTE_D7, NOTE_EIGHTH, 1},
        {NOTE_B6, NOTE_EIGHTH, 1},
        {NOTE_GS6, NOTE_EIGHTH, 0},
        {NOTE_A6, NOTE_THREE_QUARTERS, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_FS6, NOTE_QUARTER, 0},
        {NOTE_FS6, NOTE_QUARTER, 0},
        {NOTE_FS6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_HALF, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_E6, NOTE_QUARTER_AND_HALF, 0},
        {NOTE_E6, NOTE_EIGHTH, 0},
        {NOTE_B6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_QUARTER, 0},

        // Third line
        {NOTE_A6, NOTE_THREE_QUARTERS, 0},
        {NOTE_A6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_EIGHTH, 1},
        {NOTE_FS6, NOTE_EIGHTH, 0},
        {NOTE_FS6, NOTE_QUARTER, 0},
        {NOTE_FS6, NOTE_EIGHTH, 1},
        {NOTE_A6, NOTE_EIGHTH, 1},
        {NOTE_GS6, NOTE_EIGHTH, 1},
        {NOTE_B6, NOTE_EIGHTH, 0},
        {NOTE_A6, NOTE_HALF, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_E6, NOTE_QUARTER, 0},
        {NOTE_E6, NOTE_QUARTER_AND_HALF, 0},
        {NOTE_E6, NOTE_EIGHTH, 0},
        {NOTE_B6, NOTE_QUARTER, 0},
        {NOTE_GS6, NOTE_QUARTER, 0},
        {NOTE_A6, NOTE_THREE_QUARTERS, 0},
    };

    for (int i = 0; i < num_notes; i++) {
        buzzer_set(note_table[notes[i].freq]);
        DELAY(GET_NOTE_TIME(bpm, notes[i].duration));
        if (!notes[i].pair) {
            buzzer_clear();
        }
        DELAY(10);
    }

    DELAY(200);
}

void buzzer_play(BuzzerSound sound) {
    if (buzzer_queue == NULL) {
        buzzer_init();
        return;
    }
    xQueueSend(buzzer_queue, &sound, 0);
}

void buzzer_task() {
    if (buzzer_queue == NULL) {
        return;
    }

    while (1) {
        while (uxQueueMessagesWaiting(buzzer_queue)) {
            BuzzerSound sound;
            xQueueReceive(buzzer_queue, &sound, 0);
            switch (sound) {
                case BUZZER_SOUND_BEEP:
                    sound_beep(200);
                    break;
                case BUZZER_SOUND_DOUBLE_BEEP:
                    sound_beep(200);
                    sound_beep(200);
                    break;
                case BUZZER_SOUND_LONG_BEEP:
                    sound_beep(1000);
                    break;
                case BUZZER_SOUND_DESCENDING_BEEP:
                    sound_descending_beep(200);
                    break;
                case BUZZER_SOUND_LONG_DESCENDING_BEEP:
                    sound_descending_beep(1000);
                    break;
                case BUZZER_SOUND_INIT:
                    sound_init();
                    break;
                case BUZZER_SOUND_SONG:
                    sound_song();
                    break;
                case BUZZER_SOUND_REST_1S:
                    DELAY(1000);
                    break;
                case BUZZER_SOUND_REST_500MS:
                    DELAY(500);
                    break;
                case BUZZER_SOUND_REST_200MS:
                    DELAY(200);
                    break;
                default:
                    break;
            }
        }
        portYIELD();
    }
}