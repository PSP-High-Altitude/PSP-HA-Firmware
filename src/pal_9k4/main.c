#include "main.h"

#include <errno.h>
#include <sys/unistd.h>

#include "USB_Device/App/usb_device.h"
#include "USB_Device/App/usbd_cdc_if.h"
#include "clocks.h"
#include "data.h"
#include "gpio/gpio.h"
#include "nand_flash.h"
#include "pspcom.h"
#include "pyros.h"
#include "sensors.h"
#include "state.h"
#include "status.h"
#include "stm32h7xx.h"
#include "storage.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
static TaskHandle_t s_read_sensors_handle;
static TaskHandle_t s_read_gps_handle;
static TaskHandle_t s_storage_task_handle;
static TaskHandle_t s_standard_telem_handle;
static TaskHandle_t s_state_est_task_handle;
static TaskHandle_t s_process_commands_handle;
#ifdef PSPCOM_SENSORS
static TaskHandle_t s_sensor_telem_handle;
#endif

int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

#ifdef DEBUG
    uint64_t start_time = MILLIS();
    USBD_StatusTypeDef rc = USBD_OK;
    do {
        rc = CDC_Transmit_HS((uint8_t *)data, len);
    } while (USBD_BUSY == rc && MILLIS() - start_time < 10);

    if (USBD_FAIL == rc) {
        return 0;
    }
#endif
    return len;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    init_timers();
    MX_USB_DEVICE_Init();
    gpio_mode(PIN_PAUSE, GPIO_INPUT_PULLUP);
    gpio_write(PIN_RED, GPIO_HIGH);
    init_pyros();

    uint8_t init_error = 0;  // Set if error occurs during initialization

    DELAY(4700);
    printf("Starting initialization...\n");
    init_error += (init_storage() != STATUS_OK);
    init_error += (init_sensors() != STATUS_OK);
    init_error += (init_state_est() != STATUS_OK);
    init_error += (EXPECT_OK(pspcom_init(), "pspcom init") != STATUS_OK);

    // One beep for error, two for success
    gpio_write(PIN_BUZZER, GPIO_HIGH);
    DELAY(100);
    gpio_write(PIN_BUZZER, GPIO_LOW);
    DELAY(100);
    if (!init_error) {
        gpio_write(PIN_BUZZER, GPIO_HIGH);
        DELAY(100);
        gpio_write(PIN_BUZZER, GPIO_LOW);
    }

    gpio_write(PIN_RED, GPIO_LOW);
    printf("Initialization complete\n");

    // https://www.freertos.org/RTOS-Cortex-M3-M4.html
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    xTaskCreate(storage_task,           // Task function
                "storage_task",         // Task name
                2048,                   // Stack size
                NULL,                   // Parameters
                tskIDLE_PRIORITY + 1,   // Priority
                &s_storage_task_handle  // Task handle
    );

    xTaskCreate(read_sensors_task,      // Task function
                "read_sensors_task",    // Task name
                2048,                   // Stack size
                NULL,                   // Parameters
                tskIDLE_PRIORITY + 4,   // Priority
                &s_read_sensors_handle  // Task handle
    );

    xTaskCreate(read_gps_task,         // Task function
                "read_gps_task",       // Task name
                2048,                  // Stack size
                NULL,                  // Parameters
                tskIDLE_PRIORITY + 3,  // Priority
                &s_read_gps_handle     // Task handle
    );

    xTaskCreate(state_est_task,           // Task function
                "state_est_task",         // Task name
                2048,                     // Stack size
                NULL,                     // Parameters
                tskIDLE_PRIORITY + 3,     // Priority
                &s_state_est_task_handle  // Task handle
    );

    /*
        xTaskCreate(pspcom_send_gps,       // Task function
                    "gps_telem",           // Task name
                    2048,                  // Stack size
                    (void *)&s_last_fix,   // Parameters
                    tskIDLE_PRIORITY + 2,  // Priority
                    &s_gps_telem_handle    // Task handle
        );

        xTaskCreate(pspcom_send_status,     // Task function
                    "status_telem",         // Task name
                    2048,                   // Stack size
                    NULL,                   // Parameters
                    tskIDLE_PRIORITY + 2,   // Priority
                    &s_status_telem_handle  // Task handle
        );
    */
    xTaskCreate(pspcom_send_standard,     // Task function
                "status_telem",           // Task name
                2048,                     // Stack size
                NULL,                     // Parameters
                tskIDLE_PRIORITY + 2,     // Priority
                &s_standard_telem_handle  // Task handle
    );

    xTaskCreate(pspcom_process_bytes,       // Task function
                "process_commands",         // Task name
                2048,                       // Stack size
                NULL,                       // Parameters
                tskIDLE_PRIORITY + 3,       // Priority
                &s_process_commands_handle  // Task handle
    );

#ifdef PSPCOM_SENSORS
    xTaskCreate(pspcom_send_sensor,           // Task function
                "sensor_telem",               // Task name
                2048,                         // Stack size
                (void *)&s_last_sensor_data,  // Parameters
                tskIDLE_PRIORITY + 1,         // Priority
                &s_sensor_telem_handle        // Task handle
    );
#endif

    vTaskStartScheduler();

    while (1) {
        printf("kernel exited\n");
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName) {
    while (1) {
        printf("stack overflow in task '%s'", pcTaskName);
    }
}

extern void xPortSysTickHandler(void);

void SysTick_Handler(void) {
    /* Clear overflow flag */
    SysTick->CTRL;

    // Detect pause condition
    if (!gpio_read(PIN_PAUSE)) {
        pause_storage();
    } else {
        start_storage();
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        /* Call tick handler */
        xPortSysTickHandler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

void NMI_Handler(void) { printf("nmi\n"); }

void HardFault_Handler(void) {
    printf("hard fault\n");
    while (1) {
    }
}

void MemManage_Handler(void) {
    printf("memmanage\n");
    while (1) {
    }
}

void BusFault_Handler(void) {
    printf("bus fault\n");
    while (1) {
    }
}

void UsageFault_Handler(void) {
    printf("usage fault\n");
    while (1) {
    }
}

void DebugMon_Handler(void) {}

void OTG_HS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS); }
