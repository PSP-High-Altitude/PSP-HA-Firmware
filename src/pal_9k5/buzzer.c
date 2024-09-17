#include "buzzer.h"

#include "board.h"
#include "stm32h7xx_hal.h"

TIM_HandleTypeDef tim15_handle = {0};

static Status buzzer_init() {
    __HAL_RCC_TIM15_CLK_ENABLE();
    tim15_handle.Instance = TIM15;

    // Prescaler to 1MHz, period to 4khz
    TIM_Base_InitTypeDef tim15_conf = {
        .Prescaler = 100 - 1,
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = 250 - 1,
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

    return STATUS_OK;
}

Status buzzer_set(BuzzerFreq freq) {
    // Initialize if not initialized
    if (tim15_handle.State == 0) {
        buzzer_init();
    }

    HAL_TIM_OC_Stop(&tim15_handle, TIM_CHANNEL_1);
    tim15_handle.Init.Period = freq;
    HAL_TIM_OC_Init(&tim15_handle);
    HAL_TIM_OC_Start(&tim15_handle, TIM_CHANNEL_1);

    return STATUS_OK;
}

Status buzzer_clear() {
    HAL_TIM_OC_Stop(&tim15_handle, TIM_CHANNEL_1);
    return STATUS_OK;
}