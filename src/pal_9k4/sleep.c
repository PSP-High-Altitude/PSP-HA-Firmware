#include "sleep.h"

#include "board.h"
#include "gpio/gpio.h"
#include "timer.h"

void pre_sleep(uint32_t* xModifiableIdleTime) {
    gpio_write(PIN_RED, GPIO_HIGH);
    g_last_tickless_idle_entry_us = MICROS();

// If we're using core deep sleep, indicate that to FreeRTOS and enter
#ifdef USE_HAL_SLEEP
    *xModifiableIdleTime = 0;
    HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
#endif  // USE_HAL_SLEEP
}

void post_sleep(uint32_t* xExpectedIdleTime) {
    g_total_tickless_idle_us += MICROS() - g_last_tickless_idle_entry_us;
    gpio_write(PIN_RED, GPIO_LOW);
}
