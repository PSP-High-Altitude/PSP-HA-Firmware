#include <Arduino.h>
#include <stdint.h>
// #include <teensy_sd.h>

extern "C" {
#include "gpio/gpio.h"
#include "ms5637/ms5637.h"
}

#define LED_PIN 13

I2cDevice baroConf = {
    .address = 0x76,
    .clk = I2C_SPEED_FAST,
    .periph = I2C0,
};

void setup() {
    gpio_mode(LED_PIN, GPIO_OUTPUT);
    ms5637_init(&baroConf);
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
    gpio_write(LED_PIN, GPIO_HIGH);
    delay(500);
    gpio_write(LED_PIN, GPIO_LOW);
    delay(500);

    BaroData data = ms5637_read(&baroConf, OSR_4096);
    Serial.printf("Pressure: %d mbar, Temperature %d degC\n", data.pressure,
                  data.temperature);

    // SDDevice device{.cs = 1};
    // sd_init(&device);
}