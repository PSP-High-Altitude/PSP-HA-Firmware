#include "main.h"

#include "adc/adc.h"
#include "backup/backup.h"
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
static ADCDevice adc2;
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

void adc_test(TaskHandle_t *handle_ptr) {
    DELAY(5000);

    // Close PA0 analog switch
    adc_set_pa0_sw(true);
    // Set INP16 alternate to Vbat/4
    adc_set_ch16_alt_sw(true);
    // Set INP17 alternate to Vrefint
    adc_set_ch17_alt_sw(true);
    gpio_mode(PIN_PA0, GPIO_ANALOG);
    gpio_mode(PIN_PC0, GPIO_ANALOG);
    adc2.periph = P_ADC2;
    adc2.resolution_bits = 16;
    adc2.sample_time_cycles_x2 = 5;
    adc2.channel_mask = 0x00030401;  // INP0, 10, 16, 17
    Status ret = adc_init(&adc2);
    printf("ADC init: %d\n", ret);

    while (1) {
        gpio_write(PIN_BLUE, GPIO_HIGH);
        adc_conv(&adc2);
        DELAY(100);
        uint16_t adc_data0, adc_data10, adc_data16, adc_data17;
        adc_ch_read(&adc2, 0, &adc_data0);
        adc_ch_read(&adc2, 1, &adc_data10);
        adc_ch_read(&adc2, 2, &adc_data16);
        adc_ch_read(&adc2, 3, &adc_data17);
        double vin = (double)adc_data10 * 3.3 / 65535;
        double vpy = (double)adc_data0 * 3.3 / 65535;
        double vbat = (double)adc_data16 * 3.3 / 65535 * 4;
        double vrefint = (double)adc_data17 * 3.3 / 65535;
        printf("VIN: %g V, VPY: %g V, VBAT: %g V, VREFINT: %g V\n", vin, vpy,
               vbat, vrefint);
        gpio_write(PIN_BLUE, GPIO_LOW);
        DELAY(100);
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

    usb_init();
    TASK_CREATE(task_usb, 3, 8192);
    TASK_CREATE(adc_test, 2, 1024);

    vTaskStartScheduler();
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
