#include "main.h"

#include "backup.h"
#include "board_config.h"
#include "button_event.h"
#include "buttons.h"
#include "clocks.h"
#include "data.h"
#include "gpio/gpio.h"
#include "malloc.h"
#include "pspcom.h"
#include "pyros.h"
#include "rtc/rtc.h"
#include "status.h"
#include "stdio.h"
#include "stm32h7xx.h"
#include "tasks/buzzer.h"
#include "tasks/control.h"
#include "tasks/gps.h"
#include "tasks/sensors.h"
#include "tasks/storage.h"
#include "timer.h"
#include "usb.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

uint8_t mtp_mode = 0;

/*****************/
/* HELPER MACROS */
/*****************/

#define PANIC(msg, ...)                                       \
    do {                                                      \
        gpio_write(PIN_RED, GPIO_HIGH);                       \
        DELAY(1000);                                          \
        gpio_write(PIN_RED, GPIO_LOW);                        \
        DELAY(1000);                                          \
        /* Print might itself cause a fault, so do it last */ \
        PAL_LOGE("PANIC @ %s:%d: ", __FILE__, __LINE__);      \
        PAL_LOGE(msg, ##__VA_ARGS__);                         \
    } while (1)

#define TASK_CREATE(func, pri, ss)                                    \
    do {                                                              \
        static TaskHandle_t s_##func##_handle;                        \
        if (xTaskCreate((void *)func,             /* Task function */ \
                        #func,                    /* Task name */     \
                        ss,                       /* Stack size */    \
                        &s_##func##_handle,       /* Parameters */    \
                        tskIDLE_PRIORITY + (pri), /* Priority */      \
                        &s_##func##_handle        /* Task handle */   \
                        ) != pdPASS) {                                \
            PANIC("failed to launch task %s\n", #func);               \
        }                                                             \
    } while (0)

/******************/
/* MAIN FUNCTIONS */
/******************/

/**
 * FreeRTOS init task for initializating all application-related
 * components like interfaces, program state, etc.
 *
 * After initialization, this task runs periodically to perform
 * maintenance functions and handle outside events.
 */
void task_init() {
    // Suspend all tasks until initialization is complete
    vTaskSuspendAll();

    uint32_t init_error = 0;  // Set if error occurs during initialization
    uint32_t num_inits = 8;   // Number of inits the error code refers to

    mtp_mode = backup_get_ptr()->flag_mtp_pressed;

    PAL_LOGI("Starting initialization...\n");

    buttons_init();
    init_error |= (EXPECT_OK(storage_init(), "init storage") != STATUS_OK) << 0;
    init_error |= (EXPECT_OK(usb_init(), "init usb") != STATUS_OK) << 1;
    init_error |= (EXPECT_OK(buzzer_init(), "init buzzer") != STATUS_OK) << 2;
    init_error |= (EXPECT_OK(sensors_init(), "init sensors") != STATUS_OK) << 3;
    init_error |= (EXPECT_OK(gps_init(), "init GPS") != STATUS_OK) << 4;
    init_error |= (EXPECT_OK(control_init(), "init control") != STATUS_OK) << 5;
    init_error |= (EXPECT_OK(pyros_init(), "init pyros") != STATUS_OK) << 6;
    init_error |= (EXPECT_OK(pspcom_init(), "init pspcom") != STATUS_OK) << 7;

    // Play init tune
    gpio_write(PIN_RED, GPIO_LOW);
    gpio_write(PIN_YELLOW, GPIO_LOW);
    gpio_write(PIN_GREEN, GPIO_LOW);
    gpio_write(PIN_BLUE, GPIO_LOW);
    buzzer_play(BUZZER_SOUND_INIT);
    // buzzer_play(BUZZER_SOUND_SONG);

    // Beep out the failure code (if any)
    for (int i = 0; i < num_inits; i++) {
        if ((init_error >> i) & 1) {
            buzzer_play(BUZZER_SOUND_LONG_DESCENDING_BEEP);
        } else {
            buzzer_play(BUZZER_SOUND_LONG_BEEP);
        }
    }

    PAL_LOGI("Initialization complete\n");

    if (!mtp_mode) {
        // Start tasks if we are in normal mode
        PAL_LOGI("Launching flight tasks\n");
        TASK_CREATE(task_pyros, +9, 2048);
        TASK_CREATE(task_control, +8, 2048);
        TASK_CREATE(task_sensors, +7, 2048);
        TASK_CREATE(task_pspcom_tx, +6, 2048);
        TASK_CREATE(task_gps, +5, 2048);
        TASK_CREATE(task_storage, +4, 4096);
        TASK_CREATE(task_pspcom_rx, +3, 2048);
        TASK_CREATE(task_buzzer, +2, 512);
        TASK_CREATE(task_usb, +1, 4096);
    } else {
        PAL_LOGI("Started USB MSC mode\n");
        TASK_CREATE(task_buzzer, +2, 512);
        TASK_CREATE(task_usb, +1, 4096);
    }

#ifdef DEBUG_MEMORY_USAGE
    TASK_CREATE(debug_memory_usage_task, +1, 512);
#endif

    xTaskResumeAll();

    // After starting tasks, set the transmit frequency
    // if (pspcom_change_frequency(config_get_ptr()->telemetry_frequency_hz) !=
    //     STATUS_OK) {
    //     PAL_LOGE("Failed to set telemetry frequency!\n");
    // } else {
    //     PAL_LOGI("Telemetry frequency set to %.3f MHz!\n",
    //              config_get_ptr()->telemetry_frequency_hz / 1e6);
    // }

    while (1) {
        if (mtp_mode) {
            gpio_write(PIN_RED, GPIO_HIGH);
            gpio_write(PIN_YELLOW, GPIO_HIGH);
            gpio_write(PIN_GREEN, GPIO_HIGH);
            gpio_write(PIN_BLUE, GPIO_HIGH);
            DELAY(1000);
            gpio_write(PIN_RED, GPIO_LOW);
            gpio_write(PIN_YELLOW, GPIO_LOW);
            gpio_write(PIN_GREEN, GPIO_LOW);
            gpio_write(PIN_BLUE, GPIO_LOW);
            DELAY(1000);
        } else {
#ifdef HWIL_TEST
            gpio_write(PIN_RED, GPIO_HIGH);
            DELAY(100);
            gpio_write(PIN_RED, GPIO_LOW);
            DELAY(100);
#else
            DELAY(1000 * 60);
            PAL_LOGI("<3\n");
#endif  // HWIL_TEST
        }
    }
}

/**
 * Entry point function.
 */
int main(void) {
    // Perform critical bare-metal initialization
    HAL_Init();
    SystemClock_Config();
    init_timers();
    backup_init();
    rtc_init();
    button_event_init();

    // Light all LEDs to indicate initialization
    gpio_write(PIN_RED, GPIO_HIGH);
    gpio_write(PIN_YELLOW, GPIO_HIGH);
    gpio_write(PIN_GREEN, GPIO_HIGH);
    gpio_write(PIN_BLUE, GPIO_HIGH);

    // Set pull-ups on the button pins
    gpio_mode(PIN_PAUSE, GPIO_INPUT_PULLUP);

    // Launch FreeRTOS kernel and init task
    TASK_CREATE(task_init, -1, 8192);

    PAL_LOGI("Starting scheduler\n");

    vTaskStartScheduler();

    PANIC("kernel exited\n");
}

/**********************/
/* EXCEPTION HANDLERS */
/**********************/

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName) {
    PANIC("stack overflow in task '%s'", pcTaskName);
}

extern void xPortSysTickHandler(void);

void SysTick_Handler(void) {
    /* Clear overflow flag */
    SysTick->CTRL;

    /* Update backup system timestamp */
    backup_get_ptr()->timestamp = MICROS();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        /* Call tick handler */
        xPortSysTickHandler();
    }
}

void Error_Handler(void) { PANIC("unexpected exception\n"); }

void NMI_Handler(void) { PANIC("unexpected exception\n"); }

void HardFault_Handler(void) { PANIC("unexpected exception\n"); }

void MemManage_Handler(void) { PANIC("unexpected exception\n"); }

void BusFault_Handler(void) { PANIC("unexpected exception\n"); }

void UsageFault_Handler(void) { PANIC("unexpected exception\n"); }

void DebugMon_Handler(void) { PANIC("unexpected exception\n"); }
