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

// Sorry about preprocessor abuse, but this really does make the code cleaner
#define TASK_STACK_SIZE 2048
#define TASK_CREATE(func, pri)                                          \
    do {                                                                \
        static TaskHandle_t s_##func##_handle;                          \
        if (xTaskCreate(func,                     /* Task function */   \
                        #func,                    /* Task name */       \
                        TASK_STACK_SIZE,          /* Stack size */      \
                        NULL,                     /* Parameters */      \
                        tskIDLE_PRIORITY + (pri), /* Priority */        \
                        &s_##func##_handle        /* Task handle */     \
                        ) != pdPASS) {                                  \
            printf("FATAL: failed to launch task %s at %s:%d\n", #func, \
                   __FILE__, __LINE__);                                 \
            while (1) {                                                 \
                gpio_write(PIN_RED, GPIO_HIGH);                         \
                DELAY(1000);                                            \
                gpio_write(PIN_RED, GPIO_LOW);                          \
                DELAY(1000);                                            \
            }                                                           \
        }                                                               \
    } while (0)

// Serial debug stuff
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

void handle_pause() {
    static bool s_last_paused;

    if (!gpio_read(PIN_PAUSE)) {
        s_last_paused = true;

        pause_sensors();
        pause_storage();
    } else if (s_last_paused) {
        start_storage();
        start_sensors();

        s_last_paused = false;
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    init_timers();
    MX_USB_DEVICE_Init();
    gpio_mode(PIN_PAUSE, GPIO_INPUT_PULLUP);
    gpio_write(PIN_RED, GPIO_HIGH);

    uint32_t init_error = 0;  // Set if error occurs during initialization

    DELAY(4700);
    printf("Starting initialization...\n");
    init_error |= (EXPECT_OK(init_storage(), "init storage") != STATUS_OK) << 0;
    init_error |= (EXPECT_OK(init_sensors(), "init sensors") != STATUS_OK) << 1;
    init_error |= (EXPECT_OK(init_state_est(), "init state") != STATUS_OK) << 2;
    init_error |= (EXPECT_OK(init_pyros(), "init pyros") != STATUS_OK) << 3;
    init_error |= (EXPECT_OK(pspcom_init(), "init pspcom") != STATUS_OK) << 4;

    // One beep for initialization complete
    gpio_write(PIN_RED, GPIO_LOW);
    gpio_write(PIN_BUZZER, GPIO_HIGH);
    DELAY(200);
    gpio_write(PIN_BUZZER, GPIO_LOW);
    DELAY(200);

    // Beep out the failure code (if any)
    for (int i = 0; i < init_error; i++) {
        gpio_write(PIN_BUZZER, GPIO_HIGH);
        gpio_write(PIN_RED, GPIO_HIGH);
        DELAY(100);
        gpio_write(PIN_BUZZER, GPIO_LOW);
        gpio_write(PIN_RED, GPIO_LOW);
        DELAY(100);
    }

    printf("Initialization complete\n");

    // https://www.freertos.org/RTOS-Cortex-M3-M4.html
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    printf("Launching tasks\n");

    TASK_CREATE(pyros_task, +5);
    TASK_CREATE(read_sensors_task, +4);
    TASK_CREATE(state_est_task, +3);
    TASK_CREATE(pspcom_process_bytes, +3);
    TASK_CREATE(pspcom_send_standard, +2);
    TASK_CREATE(read_gps_task, +2);
    TASK_CREATE(storage_task, +1);

    printf("Starting scheduler\n");

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
    handle_pause();

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
