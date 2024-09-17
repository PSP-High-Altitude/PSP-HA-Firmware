#include "main.h"

#include "buzzer.h"
#include "clocks.h"
#include "data.h"
#include "gpio/gpio.h"
#include "pspcom.h"
#include "pyros.h"
#include "status.h"
#include "stdio.h"
#include "stm32h7xx.h"
#include "storage.h"
#include "timer.h"
#include "usb.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

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
        printf("PANIC @ %s:%d: ", __FILE__, __LINE__);        \
        printf(msg, ##__VA_ARGS__);                           \
    } while (1)

#define TASK_CREATE(func, pri, ss)                                    \
    do {                                                              \
        static TaskHandle_t s_##func##_handle;                        \
        if (xTaskCreate(func,                     /* Task function */ \
                        #func,                    /* Task name */     \
                        ss,                       /* Stack size */    \
                        NULL,                     /* Parameters */    \
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
void init_task() {
    // Suspend all tasks until initialization is complete
    vTaskSuspendAll();

    uint32_t init_error = 0;  // Set if error occurs during initialization

    printf("Starting initialization...\n");

    init_error |= (EXPECT_OK(init_usb(0), "init usb") != STATUS_OK) << 0;
    init_error |= (EXPECT_OK(init_storage(), "init storage") != STATUS_OK) << 1;
    // init_error |= (EXPECT_OK(init_sensors(), "init sensors") != STATUS_OK) <<
    // 2; init_error |= (EXPECT_OK(init_pyros(), "init pyros") != STATUS_OK) <<
    // 4; init_error |= (EXPECT_OK(pspcom_init(), "init pspcom") != STATUS_OK)
    // << 5;

    // One beep for initialization complete
    gpio_write(PIN_RED, GPIO_LOW);
    buzzer_set(BUZZER_FREQ_1KHZ);
    DELAY(200);
    buzzer_set(BUZZER_FREQ_2KHZ);
    DELAY(200);
    buzzer_set(BUZZER_FREQ_4KHZ);
    DELAY(200);
    buzzer_clear();
    DELAY(1000);

    // Beep out the failure code (if any)
    for (int i = 0; i < init_error; i++) {
        buzzer_set(BUZZER_FREQ_4KHZ);
        gpio_write(PIN_RED, GPIO_HIGH);
        DELAY(100);
        buzzer_clear();
        gpio_write(PIN_RED, GPIO_LOW);
        DELAY(100);
    }

    printf("Initialization complete\n");

    // Start tasks if we are in normal mode
    // if (usb_mode == 1) {
    //    printf("Launching tasks\n");
    //    TASK_CREATE(pyros_task, +5, 2048);
    //    TASK_CREATE(read_sensors_task, +4, 2048);
    //    TASK_CREATE(state_est_task, +3, 2048);
    //    TASK_CREATE(pspcom_process_bytes, +3, 2048);
    //    TASK_CREATE(pspcom_send_standard, +2, 2048);
    //    TASK_CREATE(read_gps_task, +2, 2048);
    //    TASK_CREATE(storage_task, +1, 4096);
    //} else {
    //    // MTP mode data queuing task
    //    TASK_CREATE(mtp_readwrite_file_task, +1, 2048);
    //}
#ifdef DEBUG_MEMORY_USAGE
    TASK_CREATE(debug_memory_usage_task, +1, 512);
#endif

    xTaskResumeAll();

    while (1) {
        DELAY(0xFFFF);
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

    // Light all LEDs to indicate initialization
    gpio_write(PIN_RED, GPIO_HIGH);
    gpio_write(PIN_YELLOW, GPIO_HIGH);
    gpio_write(PIN_GREEN, GPIO_HIGH);
    gpio_write(PIN_BLUE, GPIO_HIGH);

    // Set pull-ups on the button pins
    gpio_mode(PIN_PAUSE, GPIO_INPUT_PULLUP);

    // Launch FreeRTOS kernel and init task
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    TASK_CREATE(init_task, -1, 8192);

    printf("Starting scheduler\n");

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

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        /* Call tick handler */
        xPortSysTickHandler();
    }
}

void Error_Handler(void) { PANIC("Unexpected exception\n"); }

void NMI_Handler(void) { PANIC("Unexpected exception\n"); }

void HardFault_Handler(void) { PANIC("Unexpected exception\n"); }

void MemManage_Handler(void) { PANIC("Unexpected exception\n"); }

void BusFault_Handler(void) { PANIC("Unexpected exception\n"); }

void UsageFault_Handler(void) { PANIC("Unexpected exception\n"); }

void DebugMon_Handler(void) { PANIC("Unexpected exception\n"); }
