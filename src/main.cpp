#include <Arduino.h>
#include <stdint.h>
#include <teensy_sd.h>

extern "C" {
#include "gpio/gpio.h"
#include "lsm6dsox/lsm6dsox.h"
#include "ms5637/ms5637.h"
#include "time.h"
}

#define LED_PIN 13

uint32_t lastTime = 0;

I2cDevice baroConf = {
    .address = 0x76,
    .clk = I2C_SPEED_STANDARD,
    .periph = I2C3,
};
SpiDevice imuConf = {
    .clk = 1000000,
    .cpol = 0,
    .cpha = 0,
    .cs = 0,
    .periph = P_SPI2,
};

void setup() {
    initTimers();
    gpio_mode(LED_PIN, GPIO_OUTPUT);
    gpio_mode(2, GPIO_INPUT);
    delay(200);
    ms5637_init(&baroConf);
    lsm6dsox_init(&imuConf);
    lsm6dsox_config_accel(&imuConf, LSM6DSOX_XL_RATE_208_HZ,
                          LSM6DSOX_XL_RANGE_8_G);
    lsm6dsox_config_gyro(&imuConf, LSM6DSOX_G_RATE_208_HZ,
                         LSM6DSOX_G_RANGE_500_DPS);
    SDDevice device{.cs = BUILTIN_SDCARD};
    sd_init(&device);
}

void loop() {
    while (gpio_read(2) == GPIO_HIGH) {
        gpio_write(LED_PIN, GPIO_HIGH);
    }
    while (MILLIS() - lastTime < 10) {
    }
    lastTime = MILLIS();
    gpio_write(LED_PIN, GPIO_LOW);
    Accel accelData = lsm6dsox_read_accel(&imuConf);
    Gyro gyroData = lsm6dsox_read_gyro(&imuConf);
    BaroData baroData = ms5637_read(&baroConf, OSR_256);

    // Accel accelData = {.accelX = 0.123, .accelY = 0.211, .accelZ = 9.814};
    // Gyro gyroData = {.gyroX = 0.025, .gyroY = 0.011, .gyroZ = 0.0352};
    // BaroData baroData = {.temperature = 23.55, .pressure = 995.35};
    sd_write(MILLIS(), &accelData, &gyroData, &baroData);
}