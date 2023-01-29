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

I2cDevice baroConf = {
    .address = 0x76,
    .clk = I2C_SPEED_STANDARD,
    .periph = I2C0,
};
SpiDevice imuConf = {
    .clk = 100000,
    .cpol = 0,
    .cpha = 0,
    .cs = 0,
    .periph = P_SPI2,
};

void setup() {
    initTimers();
    Serial.begin(9600);
    gpio_mode(LED_PIN, GPIO_OUTPUT);
    delay(200);
    ms5637_init(&baroConf);
    lsm6dsox_init(&imuConf);
    lsm6dsox_config_accel(&imuConf, LSM6DSOX_XL_RATE_52_HZ,
                          LSM6DSOX_XL_RANGE_2_G);
    lsm6dsox_config_gyro(&imuConf, LSM6DSOX_G_RATE_52_HZ,
                         LSM6DSOX_G_RANGE_250_DPS);
    SDDevice device{.cs = BUILTIN_SDCARD};
    sd_init(&device);
}

void loop() {
    /*
    uint16_t val = 0x4841;
    for (size_t i = 0; i < 16; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
        digitalWrite(LED_PIN, (val >> i) & 1);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }
    */
    Serial.println(MILLIS());
    gpio_write(LED_PIN, GPIO_HIGH);
    DELAY(500);
    gpio_write(LED_PIN, GPIO_LOW);
    DELAY(500);
    Accel accelData = lsm6dsox_read_accel(&imuConf);
    Serial.printf("Accel X: %f g, Accel Y: % f g, Accel Z: %f g\n",
                  accelData.accelX, accelData.accelY, accelData.accelZ);
    Gyro gyroData = lsm6dsox_read_gyro(&imuConf);
    Serial.printf("Gyro X: %f dps, Gyro Y: % f dps, Gyro Z: %f dps\n",
                  gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ);
    BaroData baroData = ms5637_read(&baroConf, OSR_256);
    Serial.printf("Pressure: %f mbar, Temperature %f degC\n", baroData.pressure,
                  baroData.temperature);

    // Accel accelData = {.accelX = 0.123, .accelY = 0.211, .accelZ = 9.814};
    // Gyro gyroData = {.gyroX = 0.025, .gyroY = 0.011, .gyroZ = 0.0352};
    // BaroData baroData = {.temperature = 23.55, .pressure = 995.35};
    sd_write(MILLIS(), &accelData, &gyroData, &baroData);
}