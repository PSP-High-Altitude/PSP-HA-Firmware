#include "button_event.h"

#include "board.h"
#include "gpio/gpio.h"
#include "stdio.h"
#include "stm32h7xx.h"

static void (*callbacks[16])() = {NULL, NULL, NULL, NULL, NULL, NULL,
                                  NULL, NULL, NULL, NULL, NULL, NULL,
                                  NULL, NULL, NULL, NULL};

Status button_event_init() {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    return STATUS_OK;
}

Status button_event_create(ButtonEventConfig *config) {
    // Must have a valid event handler
    if (config->event_handler == NULL) {
        return STATUS_PARAMETER_ERROR;
    }

    // Configure the pin as input
    gpio_mode(config->pin, GPIO_INPUT);

    // Enable interrupt on requested edge transitions
    EXTI->RTSR1 |= (config->rising << (config->pin & 0xF));
    EXTI->FTSR1 |= (config->falling << (config->pin & 0xF));

    // Direct the right port to the EXTI peripheral
    SYSCFG->EXTICR[(config->pin & 0xF) >> 2] |= (config->pin >> 4)
                                                << ((config->pin & 0x3) << 2);

    // Unmask the interrupt
    EXTI->IMR1 |= (1U << (config->pin & 0xF));

    // Register the callback
    callbacks[config->pin & 0xF] = config->event_handler;

    return STATUS_OK;
}

Status button_event_destroy(ButtonEventConfig *config) {
    // Mask interrupt and remove callback
    EXTI->IMR1 &= ~(1U << (config->pin & 0xF));
    callbacks[config->pin & 0xF] = NULL;

    return STATUS_OK;
}

void EXTI0_IRQHandler() { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
void EXTI1_IRQHandler() { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); }
void EXTI2_IRQHandler() { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); }
void EXTI3_IRQHandler() { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }
void EXTI4_IRQHandler() { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); }
void EXTI9_5_IRQHandler() {
    for (int i = 5; i < 10; i++) {
        if (EXTI->PR1 & (1U << i)) {
            HAL_GPIO_EXTI_IRQHandler(1U << i);
        }
    }
}
void EXTI15_10_IRQHandler() {
    for (int i = 10; i < 16; i++) {
        if (EXTI->PR1 & (1U << i)) {
            HAL_GPIO_EXTI_IRQHandler(1U << i);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    int pin = 0;
    while (GPIO_Pin >>= 1) {
        pin++;
    }
    if (callbacks[pin] != NULL) {
        (callbacks[pin])();
    }
}