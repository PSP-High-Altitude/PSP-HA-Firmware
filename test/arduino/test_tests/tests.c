#include <Arduino.h>
#include <unity.h>

void setUp() {}

void tearDown() {}

void test_2_plus_2() {
    uint32_t a = 2;
    uint32_t b = 2;
    TEST_ASSERT_EQUAL_UINT32(4, a + b);
}

void runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_2_plus_2);
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
