/*
    Define the map from ports on the IMXRT1062 to a GPIO base number and pin
    number
*/

#ifndef GPIO_PORT_MAP_H
#define GPIO_PORT_MAP_H

#include "imxrt1062/MIMXRT1062/drivers/fsl_gpio.h"

typedef struct {
    GPIO_Type *base;
    uint8_t pin;
} gpioPortToPin_t;

/*
static const gpioPortToPin_t GPIO_AD_B0_00 = {.base = GPIO6, .pin = 0};
static const gpioPortToPin_t GPIO_AD_B0_01 = {.base = GPIO6, .pin = 1};
static const gpioPortToPin_t GPIO_AD_B0_02 = {.base = GPIO6, .pin = 2};
static const gpioPortToPin_t GPIO_AD_B0_03 = {.base = GPIO6, .pin = 3};
static const gpioPortToPin_t GPIO_AD_B0_04 = {.base = GPIO6, .pin = 4};
static const gpioPortToPin_t GPIO_AD_B0_05 = {.base = GPIO6, .pin = 5};
static const gpioPortToPin_t GPIO_AD_B0_06 = {.base = GPIO6, .pin = 6};
static const gpioPortToPin_t GPIO_AD_B0_07 = {.base = GPIO6, .pin = 7};
static const gpioPortToPin_t GPIO_AD_B0_08 = {.base = GPIO6, .pin = 8};
static const gpioPortToPin_t GPIO_AD_B0_09 = {.base = GPIO6, .pin = 9};
static const gpioPortToPin_t GPIO_AD_B0_10 = {.base = GPIO6, .pin = 10};
static const gpioPortToPin_t GPIO_AD_B0_11 = {.base = GPIO6, .pin = 11};
static const gpioPortToPin_t GPIO_AD_B0_12 = {.base = GPIO6, .pin = 12};
static const gpioPortToPin_t GPIO_AD_B0_13 = {.base = GPIO6, .pin = 13};
static const gpioPortToPin_t GPIO_AD_B0_14 = {.base = GPIO6, .pin = 14};
static const gpioPortToPin_t GPIO_AD_B0_15 = {.base = GPIO6, .pin = 15};
static const gpioPortToPin_t GPIO_AD_B1_00 = {.base = GPIO6, .pin = 16};
static const gpioPortToPin_t GPIO_AD_B1_01 = {.base = GPIO6, .pin = 17};
static const gpioPortToPin_t GPIO_AD_B1_02 = {.base = GPIO6, .pin = 18};
static const gpioPortToPin_t GPIO_AD_B1_03 = {.base = GPIO6, .pin = 19};
static const gpioPortToPin_t GPIO_AD_B1_04 = {.base = GPIO6, .pin = 20};
static const gpioPortToPin_t GPIO_AD_B1_05 = {.base = GPIO6, .pin = 21};
static const gpioPortToPin_t GPIO_AD_B1_06 = {.base = GPIO6, .pin = 22};
static const gpioPortToPin_t GPIO_AD_B1_07 = {.base = GPIO6, .pin = 23};
static const gpioPortToPin_t GPIO_AD_B1_08 = {.base = GPIO6, .pin = 24};
static const gpioPortToPin_t GPIO_AD_B1_09 = {.base = GPIO6, .pin = 25};
static const gpioPortToPin_t GPIO_AD_B1_10 = {.base = GPIO6, .pin = 26};
static const gpioPortToPin_t GPIO_AD_B1_11 = {.base = GPIO6, .pin = 27};
static const gpioPortToPin_t GPIO_AD_B1_12 = {.base = GPIO6, .pin = 28};
static const gpioPortToPin_t GPIO_AD_B1_13 = {.base = GPIO6, .pin = 29};
static const gpioPortToPin_t GPIO_AD_B1_14 = {.base = GPIO6, .pin = 30};
static const gpioPortToPin_t GPIO_AD_B1_15 = {.base = GPIO6, .pin = 31};

static const gpioPortToPin_t GPIO_B0_00 = {.base = GPIO7, .pin = 0};
static const gpioPortToPin_t GPIO_B0_01 = {.base = GPIO7, .pin = 1};
static const gpioPortToPin_t GPIO_B0_02 = {.base = GPIO7, .pin = 2};
static const gpioPortToPin_t GPIO_B0_03 = {.base = GPIO7, .pin = 3};
static const gpioPortToPin_t GPIO_B0_04 = {.base = GPIO7, .pin = 4};
static const gpioPortToPin_t GPIO_B0_05 = {.base = GPIO7, .pin = 5};
static const gpioPortToPin_t GPIO_B0_06 = {.base = GPIO7, .pin = 6};
static const gpioPortToPin_t GPIO_B0_07 = {.base = GPIO7, .pin = 7};
static const gpioPortToPin_t GPIO_B0_08 = {.base = GPIO7, .pin = 8};
static const gpioPortToPin_t GPIO_B0_09 = {.base = GPIO7, .pin = 9};
static const gpioPortToPin_t GPIO_B0_10 = {.base = GPIO7, .pin = 10};
static const gpioPortToPin_t GPIO_B0_11 = {.base = GPIO7, .pin = 11};
static const gpioPortToPin_t GPIO_B0_12 = {.base = GPIO7, .pin = 12};
static const gpioPortToPin_t GPIO_B0_13 = {.base = GPIO7, .pin = 13};
static const gpioPortToPin_t GPIO_B0_14 = {.base = GPIO7, .pin = 14};
static const gpioPortToPin_t GPIO_B0_15 = {.base = GPIO7, .pin = 15};
static const gpioPortToPin_t GPIO_B1_00 = {.base = GPIO7, .pin = 16};
static const gpioPortToPin_t GPIO_B1_01 = {.base = GPIO7, .pin = 17};
static const gpioPortToPin_t GPIO_B1_02 = {.base = GPIO7, .pin = 18};
static const gpioPortToPin_t GPIO_B1_03 = {.base = GPIO7, .pin = 19};
static const gpioPortToPin_t GPIO_B1_04 = {.base = GPIO7, .pin = 20};
static const gpioPortToPin_t GPIO_B1_05 = {.base = GPIO7, .pin = 21};
static const gpioPortToPin_t GPIO_B1_06 = {.base = GPIO7, .pin = 22};
static const gpioPortToPin_t GPIO_B1_07 = {.base = GPIO7, .pin = 23};
static const gpioPortToPin_t GPIO_B1_08 = {.base = GPIO7, .pin = 24};
static const gpioPortToPin_t GPIO_B1_09 = {.base = GPIO7, .pin = 25};
static const gpioPortToPin_t GPIO_B1_10 = {.base = GPIO7, .pin = 26};
static const gpioPortToPin_t GPIO_B1_11 = {.base = GPIO7, .pin = 27};
static const gpioPortToPin_t GPIO_B1_12 = {.base = GPIO7, .pin = 28};
static const gpioPortToPin_t GPIO_B1_13 = {.base = GPIO7, .pin = 29};
static const gpioPortToPin_t GPIO_B1_14 = {.base = GPIO7, .pin = 30};
static const gpioPortToPin_t GPIO_B1_15 = {.base = GPIO7, .pin = 31};

static const gpioPortToPin_t GPIO_SD_B1_00 = {.base = GPIO8, .pin = 0};
static const gpioPortToPin_t GPIO_SD_B1_01 = {.base = GPIO8, .pin = 1};
static const gpioPortToPin_t GPIO_SD_B1_02 = {.base = GPIO8, .pin = 2};
static const gpioPortToPin_t GPIO_SD_B1_03 = {.base = GPIO8, .pin = 3};
static const gpioPortToPin_t GPIO_SD_B1_04 = {.base = GPIO8, .pin = 4};
static const gpioPortToPin_t GPIO_SD_B1_05 = {.base = GPIO8, .pin = 5};
static const gpioPortToPin_t GPIO_SD_B1_06 = {.base = GPIO8, .pin = 6};
static const gpioPortToPin_t GPIO_SD_B1_07 = {.base = GPIO8, .pin = 7};
static const gpioPortToPin_t GPIO_SD_B1_08 = {.base = GPIO8, .pin = 8};
static const gpioPortToPin_t GPIO_SD_B1_09 = {.base = GPIO8, .pin = 9};
static const gpioPortToPin_t GPIO_SD_B1_10 = {.base = GPIO8, .pin = 10};
static const gpioPortToPin_t GPIO_SD_B1_11 = {.base = GPIO8, .pin = 11};
static const gpioPortToPin_t GPIO_SD_B0_00 = {.base = GPIO8, .pin = 12};
static const gpioPortToPin_t GPIO_SD_B0_01 = {.base = GPIO8, .pin = 13};
static const gpioPortToPin_t GPIO_SD_B0_02 = {.base = GPIO8, .pin = 14};
static const gpioPortToPin_t GPIO_SD_B0_03 = {.base = GPIO8, .pin = 15};
static const gpioPortToPin_t GPIO_SD_B0_04 = {.base = GPIO8, .pin = 16};
static const gpioPortToPin_t GPIO_SD_B0_05 = {.base = GPIO8, .pin = 17};
static const gpioPortToPin_t GPIO_EMC_32 = {.base = GPIO8, .pin = 18};
static const gpioPortToPin_t GPIO_EMC_33 = {.base = GPIO8, .pin = 19};
static const gpioPortToPin_t GPIO_EMC_34 = {.base = GPIO8, .pin = 20};
static const gpioPortToPin_t GPIO_EMC_35 = {.base = GPIO8, .pin = 21};
static const gpioPortToPin_t GPIO_EMC_36 = {.base = GPIO8, .pin = 22};
static const gpioPortToPin_t GPIO_EMC_37 = {.base = GPIO8, .pin = 23};
static const gpioPortToPin_t GPIO_EMC_38 = {.base = GPIO8, .pin = 24};
static const gpioPortToPin_t GPIO_EMC_39 = {.base = GPIO8, .pin = 25};
static const gpioPortToPin_t GPIO_EMC_40 = {.base = GPIO8, .pin = 26};
static const gpioPortToPin_t GPIO_EMC_41 = {.base = GPIO8, .pin = 27};

static const gpioPortToPin_t GPIO_EMC_00 = {.base = GPIO9, .pin = 0};
static const gpioPortToPin_t GPIO_EMC_01 = {.base = GPIO9, .pin = 1};
static const gpioPortToPin_t GPIO_EMC_02 = {.base = GPIO9, .pin = 2};
static const gpioPortToPin_t GPIO_EMC_03 = {.base = GPIO9, .pin = 3};
static const gpioPortToPin_t GPIO_EMC_04 = {.base = GPIO9, .pin = 4};
static const gpioPortToPin_t GPIO_EMC_05 = {.base = GPIO9, .pin = 5};
static const gpioPortToPin_t GPIO_EMC_06 = {.base = GPIO9, .pin = 6};
static const gpioPortToPin_t GPIO_EMC_07 = {.base = GPIO9, .pin = 7};
static const gpioPortToPin_t GPIO_EMC_08 = {.base = GPIO9, .pin = 8};
static const gpioPortToPin_t GPIO_EMC_09 = {.base = GPIO9, .pin = 9};
static const gpioPortToPin_t GPIO_EMC_10 = {.base = GPIO9, .pin = 10};
static const gpioPortToPin_t GPIO_EMC_11 = {.base = GPIO9, .pin = 11};
static const gpioPortToPin_t GPIO_EMC_12 = {.base = GPIO9, .pin = 12};
static const gpioPortToPin_t GPIO_EMC_13 = {.base = GPIO9, .pin = 13};
static const gpioPortToPin_t GPIO_EMC_14 = {.base = GPIO9, .pin = 14};
static const gpioPortToPin_t GPIO_EMC_15 = {.base = GPIO9, .pin = 15};
static const gpioPortToPin_t GPIO_EMC_16 = {.base = GPIO9, .pin = 16};
static const gpioPortToPin_t GPIO_EMC_17 = {.base = GPIO9, .pin = 17};
static const gpioPortToPin_t GPIO_EMC_18 = {.base = GPIO9, .pin = 18};
static const gpioPortToPin_t GPIO_EMC_19 = {.base = GPIO9, .pin = 19};
static const gpioPortToPin_t GPIO_EMC_20 = {.base = GPIO9, .pin = 20};
static const gpioPortToPin_t GPIO_EMC_21 = {.base = GPIO9, .pin = 21};
static const gpioPortToPin_t GPIO_EMC_22 = {.base = GPIO9, .pin = 22};
static const gpioPortToPin_t GPIO_EMC_23 = {.base = GPIO9, .pin = 23};
static const gpioPortToPin_t GPIO_EMC_24 = {.base = GPIO9, .pin = 24};
static const gpioPortToPin_t GPIO_EMC_25 = {.base = GPIO9, .pin = 25};
static const gpioPortToPin_t GPIO_EMC_26 = {.base = GPIO9, .pin = 26};
static const gpioPortToPin_t GPIO_EMC_27 = {.base = GPIO9, .pin = 27};
static const gpioPortToPin_t GPIO_EMC_28 = {.base = GPIO9, .pin = 28};
static const gpioPortToPin_t GPIO_EMC_29 = {.base = GPIO9, .pin = 29};
static const gpioPortToPin_t GPIO_EMC_30 = {.base = GPIO9, .pin = 30};
static const gpioPortToPin_t GPIO_EMC_31 = {.base = GPIO9, .pin = 31};
*/

#define GPIO_AD_B0_00_BASE GPIO6
#define GPIO_AD_B0_01_BASE GPIO6
#define GPIO_AD_B0_02_BASE GPIO6
#define GPIO_AD_B0_03_BASE GPIO6
#define GPIO_AD_B0_04_BASE GPIO6
#define GPIO_AD_B0_05_BASE GPIO6
#define GPIO_AD_B0_06_BASE GPIO6
#define GPIO_AD_B0_07_BASE GPIO6
#define GPIO_AD_B0_08_BASE GPIO6
#define GPIO_AD_B0_09_BASE GPIO6
#define GPIO_AD_B0_10_BASE GPIO6
#define GPIO_AD_B0_11_BASE GPIO6
#define GPIO_AD_B0_12_BASE GPIO6
#define GPIO_AD_B0_13_BASE GPIO6
#define GPIO_AD_B0_14_BASE GPIO6
#define GPIO_AD_B0_15_BASE GPIO6
#define GPIO_AD_B1_00_BASE GPIO6
#define GPIO_AD_B1_01_BASE GPIO6
#define GPIO_AD_B1_02_BASE GPIO6
#define GPIO_AD_B1_03_BASE GPIO6
#define GPIO_AD_B1_04_BASE GPIO6
#define GPIO_AD_B1_05_BASE GPIO6
#define GPIO_AD_B1_06_BASE GPIO6
#define GPIO_AD_B1_07_BASE GPIO6
#define GPIO_AD_B1_08_BASE GPIO6
#define GPIO_AD_B1_09_BASE GPIO6
#define GPIO_AD_B1_10_BASE GPIO6
#define GPIO_AD_B1_11_BASE GPIO6
#define GPIO_AD_B1_12_BASE GPIO6
#define GPIO_AD_B1_13_BASE GPIO6
#define GPIO_AD_B1_14_BASE GPIO6
#define GPIO_AD_B1_15_BASE GPIO6

#define GPIO_B0_00_BASE GPIO7
#define GPIO_B0_01_BASE GPIO7
#define GPIO_B0_02_BASE GPIO7
#define GPIO_B0_03_BASE GPIO7
#define GPIO_B0_04_BASE GPIO7
#define GPIO_B0_05_BASE GPIO7
#define GPIO_B0_06_BASE GPIO7
#define GPIO_B0_07_BASE GPIO7
#define GPIO_B0_08_BASE GPIO7
#define GPIO_B0_09_BASE GPIO7
#define GPIO_B0_10_BASE GPIO7
#define GPIO_B0_11_BASE GPIO7
#define GPIO_B0_12_BASE GPIO7
#define GPIO_B0_13_BASE GPIO7
#define GPIO_B0_14_BASE GPIO7
#define GPIO_B0_15_BASE GPIO7
#define GPIO_B1_00_BASE GPIO7
#define GPIO_B1_01_BASE GPIO7
#define GPIO_B1_02_BASE GPIO7
#define GPIO_B1_03_BASE GPIO7
#define GPIO_B1_04_BASE GPIO7
#define GPIO_B1_05_BASE GPIO7
#define GPIO_B1_06_BASE GPIO7
#define GPIO_B1_07_BASE GPIO7
#define GPIO_B1_08_BASE GPIO7
#define GPIO_B1_09_BASE GPIO7
#define GPIO_B1_10_BASE GPIO7
#define GPIO_B1_11_BASE GPIO7
#define GPIO_B1_12_BASE GPIO7
#define GPIO_B1_13_BASE GPIO7
#define GPIO_B1_14_BASE GPIO7
#define GPIO_B1_15_BASE GPIO7

#define GPIO_SD_B1_00_BASE GPIO8
#define GPIO_SD_B1_01_BASE GPIO8
#define GPIO_SD_B1_02_BASE GPIO8
#define GPIO_SD_B1_03_BASE GPIO8
#define GPIO_SD_B1_04_BASE GPIO8
#define GPIO_SD_B1_05_BASE GPIO8
#define GPIO_SD_B1_06_BASE GPIO8
#define GPIO_SD_B1_07_BASE GPIO8
#define GPIO_SD_B1_08_BASE GPIO8
#define GPIO_SD_B1_09_BASE GPIO8
#define GPIO_SD_B1_10_BASE GPIO8
#define GPIO_SD_B1_11_BASE GPIO8
#define GPIO_SD_B0_00_BASE GPIO8
#define GPIO_SD_B0_01_BASE GPIO8
#define GPIO_SD_B0_02_BASE GPIO8
#define GPIO_SD_B0_03_BASE GPIO8
#define GPIO_SD_B0_04_BASE GPIO8
#define GPIO_SD_B0_05_BASE GPIO8
#define GPIO_EMC_32_BASE GPIO8
#define GPIO_EMC_33_BASE GPIO8
#define GPIO_EMC_34_BASE GPIO8
#define GPIO_EMC_35_BASE GPIO8
#define GPIO_EMC_36_BASE GPIO8
#define GPIO_EMC_37_BASE GPIO8
#define GPIO_EMC_38_BASE GPIO8
#define GPIO_EMC_39_BASE GPIO8
#define GPIO_EMC_40_BASE GPIO8
#define GPIO_EMC_41_BASE GPIO8

#define GPIO_EMC_00_BASE GPIO9
#define GPIO_EMC_01_BASE GPIO9
#define GPIO_EMC_02_BASE GPIO9
#define GPIO_EMC_03_BASE GPIO9
#define GPIO_EMC_04_BASE GPIO9
#define GPIO_EMC_05_BASE GPIO9
#define GPIO_EMC_06_BASE GPIO9
#define GPIO_EMC_07_BASE GPIO9
#define GPIO_EMC_08_BASE GPIO9
#define GPIO_EMC_09_BASE GPIO9
#define GPIO_EMC_10_BASE GPIO9
#define GPIO_EMC_11_BASE GPIO9
#define GPIO_EMC_12_BASE GPIO9
#define GPIO_EMC_13_BASE GPIO9
#define GPIO_EMC_14_BASE GPIO9
#define GPIO_EMC_15_BASE GPIO9
#define GPIO_EMC_16_BASE GPIO9
#define GPIO_EMC_17_BASE GPIO9
#define GPIO_EMC_18_BASE GPIO9
#define GPIO_EMC_19_BASE GPIO9
#define GPIO_EMC_20_BASE GPIO9
#define GPIO_EMC_21_BASE GPIO9
#define GPIO_EMC_22_BASE GPIO9
#define GPIO_EMC_23_BASE GPIO9
#define GPIO_EMC_24_BASE GPIO9
#define GPIO_EMC_25_BASE GPIO9
#define GPIO_EMC_26_BASE GPIO9
#define GPIO_EMC_27_BASE GPIO9
#define GPIO_EMC_28_BASE GPIO9
#define GPIO_EMC_29_BASE GPIO9
#define GPIO_EMC_30_BASE GPIO9
#define GPIO_EMC_31_BASE GPIO9

#define GPIO_AD_B0_00_PIN 0
#define GPIO_AD_B0_01_PIN 1
#define GPIO_AD_B0_02_PIN 2
#define GPIO_AD_B0_03_PIN 3
#define GPIO_AD_B0_04_PIN 4
#define GPIO_AD_B0_05_PIN 5
#define GPIO_AD_B0_06_PIN 6
#define GPIO_AD_B0_07_PIN 7
#define GPIO_AD_B0_08_PIN 8
#define GPIO_AD_B0_09_PIN 9
#define GPIO_AD_B0_10_PIN 10
#define GPIO_AD_B0_11_PIN 11
#define GPIO_AD_B0_12_PIN 12
#define GPIO_AD_B0_13_PIN 13
#define GPIO_AD_B0_14_PIN 14
#define GPIO_AD_B0_15_PIN 15
#define GPIO_AD_B1_00_PIN 16
#define GPIO_AD_B1_01_PIN 17
#define GPIO_AD_B1_02_PIN 18
#define GPIO_AD_B1_03_PIN 19
#define GPIO_AD_B1_04_PIN 20
#define GPIO_AD_B1_05_PIN 21
#define GPIO_AD_B1_06_PIN 22
#define GPIO_AD_B1_07_PIN 23
#define GPIO_AD_B1_08_PIN 24
#define GPIO_AD_B1_09_PIN 25
#define GPIO_AD_B1_10_PIN 26
#define GPIO_AD_B1_11_PIN 27
#define GPIO_AD_B1_12_PIN 28
#define GPIO_AD_B1_13_PIN 29
#define GPIO_AD_B1_14_PIN 30
#define GPIO_AD_B1_15_PIN 31

#define GPIO_B0_00_PIN 0
#define GPIO_B0_01_PIN 1
#define GPIO_B0_02_PIN 2
#define GPIO_B0_03_PIN 3
#define GPIO_B0_04_PIN 4
#define GPIO_B0_05_PIN 5
#define GPIO_B0_06_PIN 6
#define GPIO_B0_07_PIN 7
#define GPIO_B0_08_PIN 8
#define GPIO_B0_09_PIN 9
#define GPIO_B0_10_PIN 10
#define GPIO_B0_11_PIN 11
#define GPIO_B0_12_PIN 12
#define GPIO_B0_13_PIN 13
#define GPIO_B0_14_PIN 14
#define GPIO_B0_15_PIN 15
#define GPIO_B1_00_PIN 16
#define GPIO_B1_01_PIN 17
#define GPIO_B1_02_PIN 18
#define GPIO_B1_03_PIN 19
#define GPIO_B1_04_PIN 20
#define GPIO_B1_05_PIN 21
#define GPIO_B1_06_PIN 22
#define GPIO_B1_07_PIN 23
#define GPIO_B1_08_PIN 24
#define GPIO_B1_09_PIN 25
#define GPIO_B1_10_PIN 26
#define GPIO_B1_11_PIN 27
#define GPIO_B1_12_PIN 28
#define GPIO_B1_13_PIN 29
#define GPIO_B1_14_PIN 30
#define GPIO_B1_15_PIN 31

#define GPIO_SD_B1_00_PIN 0
#define GPIO_SD_B1_01_PIN 1
#define GPIO_SD_B1_02_PIN 2
#define GPIO_SD_B1_03_PIN 3
#define GPIO_SD_B1_04_PIN 4
#define GPIO_SD_B1_05_PIN 5
#define GPIO_SD_B1_06_PIN 6
#define GPIO_SD_B1_07_PIN 7
#define GPIO_SD_B1_08_PIN 8
#define GPIO_SD_B1_09_PIN 9
#define GPIO_SD_B1_10_PIN 10
#define GPIO_SD_B1_11_PIN 11
#define GPIO_SD_B0_00_PIN 12
#define GPIO_SD_B0_01_PIN 13
#define GPIO_SD_B0_02_PIN 14
#define GPIO_SD_B0_03_PIN 15
#define GPIO_SD_B0_04_PIN 16
#define GPIO_SD_B0_05_PIN 17
#define GPIO_EMC_32_PIN 18
#define GPIO_EMC_33_PIN 19
#define GPIO_EMC_34_PIN 20
#define GPIO_EMC_35_PIN 21
#define GPIO_EMC_36_PIN 22
#define GPIO_EMC_37_PIN 23
#define GPIO_EMC_38_PIN 24
#define GPIO_EMC_39_PIN 25
#define GPIO_EMC_40_PIN 26
#define GPIO_EMC_41_PIN 27

#define GPIO_EMC_00_PIN 0
#define GPIO_EMC_01_PIN 1
#define GPIO_EMC_02_PIN 2
#define GPIO_EMC_03_PIN 3
#define GPIO_EMC_04_PIN 4
#define GPIO_EMC_05_PIN 5
#define GPIO_EMC_06_PIN 6
#define GPIO_EMC_07_PIN 7
#define GPIO_EMC_08_PIN 8
#define GPIO_EMC_09_PIN 9
#define GPIO_EMC_10_PIN 10
#define GPIO_EMC_11_PIN 11
#define GPIO_EMC_12_PIN 12
#define GPIO_EMC_13_PIN 13
#define GPIO_EMC_14_PIN 14
#define GPIO_EMC_15_PIN 15
#define GPIO_EMC_16_PIN 16
#define GPIO_EMC_17_PIN 17
#define GPIO_EMC_18_PIN 18
#define GPIO_EMC_19_PIN 19
#define GPIO_EMC_20_PIN 20
#define GPIO_EMC_21_PIN 21
#define GPIO_EMC_22_PIN 22
#define GPIO_EMC_23_PIN 23
#define GPIO_EMC_24_PIN 24
#define GPIO_EMC_25_PIN 25
#define GPIO_EMC_26_PIN 26
#define GPIO_EMC_27_PIN 27
#define GPIO_EMC_28_PIN 28
#define GPIO_EMC_29_PIN 29
#define GPIO_EMC_30_PIN 30
#define GPIO_EMC_31_PIN 31

#endif