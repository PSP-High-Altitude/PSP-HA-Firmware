#include <Arduino.h>
#include <stdint.h>
// #include <teensy_sd.h>

#include "i2c/i2c.h"
#include "spi/spi.h"

#define LED_PIN 13

void setup() { pinMode(LED_PIN, OUTPUT); }

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
    digitalWrite(LED_PIN, HIGH);
    delay(2000);
    digitalWrite(LED_PIN, LOW);
    delay(2000);

    // SDDevice device{.cs = 1};
    // sd_init(&device);
}