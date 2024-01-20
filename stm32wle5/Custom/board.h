/*
    Board specific definitions such as pin assignments
*/

#ifndef BOARD_H
#define BOARD_H

#include "stm32wlxx_hal.h"

// Port to index
#define PIN_PA0 0
#define PIN_PA1 1
#define PIN_PA2 2
#define PIN_PA3 3
#define PIN_PA4 4
#define PIN_PA5 5
#define PIN_PA6 6
#define PIN_PA7 7
#define PIN_PA8 8
#define PIN_PA9 9
#define PIN_PA10 10
#define PIN_PA11 11
#define PIN_PA12 12
#define PIN_PA13 13
#define PIN_PA14 14
#define PIN_PA15 15
#define PIN_PB0 16
#define PIN_PB1 17
#define PIN_PB2 18
#define PIN_PB3 19
#define PIN_PB4 20
#define PIN_PB5 21
#define PIN_PB6 22
#define PIN_PB7 23
#define PIN_PB8 24
#define PIN_PB9 25
#define PIN_PB10 26
#define PIN_PB11 27
#define PIN_PB12 28
#define PIN_PB13 29
#define PIN_PB14 30
#define PIN_PB15 31
#define PIN_PC0 32
#define PIN_PC1 33
#define PIN_PC2 34
#define PIN_PC3 35
#define PIN_PC4 36
#define PIN_PC5 37
#define PIN_PC6 38
#define PIN_PC7 39
#define PIN_PC8 40
#define PIN_PC9 41
#define PIN_PC10 42
#define PIN_PC11 43
#define PIN_PC12 44
#define PIN_PC13 45
#define PIN_PC14 46
#define PIN_PC15 47
#define PIN_PH3 48

// Physical pin to gpio base
__attribute__((unused)) static GPIO_TypeDef* GPIO_PIN_TO_BASE[] = {
    GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA,
    GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB,
    GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOC,
    GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC,
    GPIOC, GPIOC, GPIOC, GPIOC, GPIOH,
};

// Physical pin to gpio num
__attribute__((unused)) static uint32_t GPIO_PIN_TO_NUM[] = {
    GPIO_PIN_0,  GPIO_PIN_1,  GPIO_PIN_2,  GPIO_PIN_3,  GPIO_PIN_4,
    GPIO_PIN_5,  GPIO_PIN_6,  GPIO_PIN_7,  GPIO_PIN_8,  GPIO_PIN_9,
    GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14,
    GPIO_PIN_15, GPIO_PIN_0,  GPIO_PIN_1,  GPIO_PIN_2,  GPIO_PIN_3,
    GPIO_PIN_4,  GPIO_PIN_5,  GPIO_PIN_6,  GPIO_PIN_7,  GPIO_PIN_8,
    GPIO_PIN_9,  GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13,
    GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_0,  GPIO_PIN_1,  GPIO_PIN_2,
    GPIO_PIN_3,  GPIO_PIN_4,  GPIO_PIN_5,  GPIO_PIN_6,  GPIO_PIN_7,
    GPIO_PIN_8,  GPIO_PIN_9,  GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12,
    GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_3,
};

#endif
