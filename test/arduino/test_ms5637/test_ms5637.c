#include <Arduino.h>
#include <unity.h>

#include "i2c/i2c.h"
#include "ms5637/ms5637.h"
#include "status.h"

void setUp() {}

void tearDown() {}

void test_ms5637_init() {
    I2cDevice device = {
        .address = 0b1110110,
        .clk = I2C_SPEED_FAST,
        .periph = P_I2C1,
    };
    TEST_ASSERT_EQUAL(ms5637_init(&device), STATUS_OK);
}

void test_ms5637_default_read() {
    I2cDevice device = {
        .address = 0b1110110,
        .clk = I2C_SPEED_FAST,
        .periph = P_I2C1,
    };
    TEST_ASSERT_EQUAL(ms5637_init(&device), STATUS_OK);

    BaroData data = ms5637_read(&device, OSR_8192);
    TEST_ASSERT_FLOAT_WITHIN(0.1, data.pressure, 1100.02);
    TEST_ASSERT_FLOAT_WITHIN(0.1, data.temperature, 20.00);
}

void runUnityTests() {
    UNITY_BEGIN();
    RUN_TEST(test_ms5637_init);
    RUN_TEST(test_ms5637_default_read);
    UNITY_END();
}

/**
 * For Arduino framework
 */
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Wait ~2 seconds before the Unity test runner
    // establishes connection with a board Serial interface
    delay(2000);

    runUnityTests();
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}
