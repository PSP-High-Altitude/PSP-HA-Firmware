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
#define TARGET_RATE 10  // Hz
#define SERIAL_ENABLED

#ifdef SERIAL_ENABLED
#define PRINT(...) Serial.printf(__VA_ARGS__);
#define PRINT_STATUS(name, status)               \
    if (status == OK)                            \
        Serial.printf("%s returned OK\n", name); \
    else                                         \
        Serial.printf("%s returned error code %d\n", name, status);
#else
#define PRINT(...)
#define PRINT_STATUS(name, status) (void)status;  // To suppress unused var
#endif

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
SDDevice sdDev{.cs = BUILTIN_SDCARD};

void setup() {
#ifdef SERIAL_ENABLED
    Serial.begin(9600);
    delay(1000);
#endif

    PRINT("Starting initialization\n")

    // Initialize timers
    initTimers();

    // Initialize GPIO
    Status led_init_status = gpio_mode(LED_PIN, GPIO_OUTPUT);  // Status LED
    Status pause_init_status = gpio_mode(2, GPIO_INPUT);       // Pause button
    PRINT_STATUS("led_init", led_init_status)
    PRINT_STATUS("pause_init", pause_init_status)

    // Initialize barometer
    Status baro_init_status = ms5637_init(&baroConf);
    PRINT_STATUS("baro_init", baro_init_status)

    // Initialize IMU
    Status imu_init_status = lsm6dsox_init(&imuConf);
    PRINT_STATUS("imu_init", imu_init_status)

    Status imu_config_accel = lsm6dsox_config_accel(
        &imuConf, LSM6DSOX_XL_RATE_208_HZ, LSM6DSOX_XL_RANGE_8_G);
    PRINT_STATUS("imu_config_accel", imu_config_accel)

    Status imu_config_gyro = lsm6dsox_config_gyro(
        &imuConf, LSM6DSOX_G_RATE_208_HZ, LSM6DSOX_G_RANGE_500_DPS);
    PRINT_STATUS("imu_config_gyro", imu_config_gyro)

    // Initialize storage
    Status sd_init_status = sd_init(&sdDev);
    PRINT_STATUS("sd_init", sd_init_status)
}

void loop() {
    // LED on -> I/O access is occurring
    gpio_write(LED_PIN, GPIO_LOW);
    while (gpio_read(2) == GPIO_HIGH) {
    }
    while (MILLIS() - lastTime < 1000 / TARGET_RATE) {
    }
    lastTime = MILLIS();
    gpio_write(LED_PIN, GPIO_HIGH);

    Accel accelData = lsm6dsox_read_accel(&imuConf);
    Gyro gyroData = lsm6dsox_read_gyro(&imuConf);
    BaroData baroData = ms5637_read(&baroConf, OSR_256);

    Status sd_write_status =
        sd_write(MILLIS(), &accelData, &gyroData, &baroData);
    PRINT("%d ", MILLIS())
    PRINT_STATUS("sd_write", sd_write_status)

    if (sd_write_status != OK) {
        while (sd_reinit(&sdDev) != OK) {
        }
    }
}
