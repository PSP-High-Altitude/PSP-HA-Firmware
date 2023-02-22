#include "timer.h"

#include "stm32g4xx_hal.h"

TIM_HandleTypeDef tim2_handle;
TIM_HandleTypeDef tim3_handle;

void init_timers() {
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM_Base_InitTypeDef tim2_conf = {
        .Prescaler = __HAL_TIM_CALC_PSC(HAL_RCC_GetPCLK1Freq(), 1000000),
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = 0xFFFFFFFF - 1,
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };
    tim2_handle.Init = tim2_conf;
    tim2_handle.Instance = TIM2;
    tim2_handle.Channel = TIM_CHANNEL_1;
    TIM_ClockConfigTypeDef tim2_source_conf = {
        .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
    };
    TIM_MasterConfigTypeDef tim2_master_conf = {
        .MasterOutputTrigger = TIM_TRGO_UPDATE,
    };
    HAL_TIM_Base_Init(&tim2_handle);
    HAL_TIM_ConfigClockSource(&tim2_handle, &tim2_source_conf);
    HAL_TIMEx_MasterConfigSynchronization(&tim2_handle, &tim2_master_conf);

    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM_Base_InitTypeDef tim3_conf = {
        .Prescaler = 1,
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = 0xFFFF,
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };
    tim3_handle.Init = tim3_conf;
    tim3_handle.Instance = TIM3;
    tim3_handle.Channel = TIM_CHANNEL_1;
    TIM_SlaveConfigTypeDef tim3_slave_conf = {
        .SlaveMode = TIM_SLAVEMODE_EXTERNAL1,
        .InputTrigger = TIM_TS_ITR1,
    };
    HAL_TIM_Base_Init(&tim3_handle);
    HAL_TIM_SlaveConfigSynchro(&tim3_handle, &tim3_slave_conf);

    HAL_TIM_Base_Start(&tim3_handle);
    HAL_TIM_Base_Start(&tim2_handle);
};

uint64_t MICROS() {
    uint64_t tim2_count = __HAL_TIM_GET_COUNTER(&tim2_handle);
    uint64_t tim3_count = __HAL_TIM_GET_COUNTER(&tim3_handle);
    return (tim3_count << 32) + tim2_count;
}

uint64_t MILLIS() { return HAL_GetTick(); };

void DELAY(uint16_t mS) {
    uint64_t start = MILLIS();
    while (MILLIS() < start + mS) {
    }
}

void DELAY_MICROS(uint32_t uS) {
    uint64_t start = MICROS();
    while (MICROS() < start + uS) {
    }
}