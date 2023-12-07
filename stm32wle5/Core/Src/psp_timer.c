#include "psp_timer.h"

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void init_timers(uint32_t sensor_interval_ms) {
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim1);
};

uint64_t MICROS() {
    volatile uint64_t tim1_count = __HAL_TIM_GET_COUNTER(&htim1);
    volatile uint64_t tim2_count = __HAL_TIM_GET_COUNTER(&htim2);
    return (tim2_count << 16) + tim1_count;
}

uint64_t MILLIS() { return MICROS() / 1000; }

void DELAY(volatile uint64_t mS) {
    volatile uint64_t start = MILLIS();
    while (MILLIS() - start < mS) {
        asm("NOP");
    }
}

void DELAY_MICROS(volatile uint64_t uS) {
    volatile uint64_t start = MICROS();
    while (MICROS() - start < uS) {
        asm("NOP");
    }
}

void while_equals_timeout(uint64_t timeout_ms, void* comp1, void* comp2,
                          void (*timeout_clbk)()) {
    volatile uint64_t start = MILLIS();
    while (MILLIS() - start < timeout_ms) {
        if (comp1 != comp2) {
            return;
        }
    }
    timeout_clbk();
}

void while_not_equals_timeout(uint64_t timeout_ms, void* comp1, void* comp2,
                              void (*timeout_clbk)()) {
    volatile uint64_t start = MILLIS();
    while (MILLIS() - start < timeout_ms) {
        if (comp1 == comp2) {
            return;
        }
    }
    timeout_clbk();
}