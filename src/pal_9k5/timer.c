#include "timer.h"

#include "FreeRTOS.h"
#include "backup/backup.h"
#include "stm32h7xx_hal.h"
#include "task.h"

TIM_HandleTypeDef tim2_handle;
TIM_HandleTypeDef tim3_handle;

void init_timers() {
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM_Base_InitTypeDef tim2_conf = {
        .Prescaler = 100 - 1,
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
        .Prescaler = 0,
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

#ifndef HWIL_TEST
    // If we have a valid backed up timestamp value, set the counters to that
    Backup* backup = backup_get_ptr();
    if (backup->timestamp) {
        uint64_t tim2_count = (backup->timestamp & 0xFFFFFFFF);
        uint64_t tim3_count = (backup->timestamp >> 32);
        __HAL_TIM_SET_COUNTER(&tim2_handle, tim2_count);
        __HAL_TIM_SET_COUNTER(&tim3_handle, tim3_count);
    }
#endif

    // Disable timers during debug
    DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM2;
    DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM3;

    HAL_TIM_Base_Start(&tim3_handle);
    HAL_TIM_Base_Start(&tim2_handle);
};

uint64_t get_systick_freq() {
    uint64_t systemClockFreq = HAL_RCC_GetHCLKFreq();
    uint64_t reloadValue = SysTick->LOAD + 1;
    return systemClockFreq / reloadValue;
}

uint64_t MICROS() {
    uint64_t tim2_count = __HAL_TIM_GET_COUNTER(&tim2_handle);
    uint64_t tim3_count = __HAL_TIM_GET_COUNTER(&tim3_handle);
    return (tim3_count << 32) + tim2_count;
}

uint64_t MILLIS() { return MICROS() / 1000; }

void DELAY(uint16_t mS) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(mS));
    } else {
        uint64_t start = MILLIS();
        while (MILLIS() < start + mS) {
        }
    }
}

void DELAY_MICROS(uint32_t uS) {
    uint64_t start = MICROS();
    while (MICROS() < start + uS) {
    }
}