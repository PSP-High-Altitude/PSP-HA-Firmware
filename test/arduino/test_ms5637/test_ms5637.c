#include <Arduino.h>
#include <unity.h>

#include "ms5637/ms5637.h"
#include "status.h"

void setUp() {}

void tearDown() {}

void test_ms5637_init() { TEST_ASSERT_EQUAL(ms5637_init(), OK); }

void runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ms5637_init);
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
