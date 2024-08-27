#include "main.h"

#include <errno.h>
#include <sys/unistd.h>

#include "clocks.h"
#include "data.h"
#include "debug_tasks.h"
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
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_mtp_if.h"

I2cDevice acc_i2c_device = {
    .address = 0x18, .clk = I2C_SPEED_FAST, .periph = P_I2C1};

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
uint8_t usb_initialized = 0;
char usb_serial_buffer[SERIAL_BUFFER_SIZE];
uint32_t usb_serial_buffer_idx = 0;
uint8_t usb_mode = 1;  // 0 = MTP, 1 = Normal

// Sorry about preprocessor abuse, but this really does make the code cleaner
#define TASK_CREATE(func, pri, ss)                                      \
    do {                                                                \
        static TaskHandle_t s_##func##_handle;                          \
        if (xTaskCreate(func,                     /* Task function */   \
                        #func,                    /* Task name */       \
                        ss,                       /* Stack size */      \
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

    if (usb_initialized == 0) {
        uint32_t copy_size =
            MIN(len, SERIAL_BUFFER_SIZE - usb_serial_buffer_idx);
        memcpy(usb_serial_buffer + usb_serial_buffer_idx, data,
               copy_size * sizeof(char));
        usb_serial_buffer_idx += copy_size;
        return len;
    }

#ifdef DEBUG
    uint64_t start_time = MILLIS();
    USBD_StatusTypeDef rc = USBD_OK;

    do {
        rc = CDC_Transmit_HS((uint8_t *)usb_serial_buffer,
                             usb_serial_buffer_idx);
    } while (USBD_BUSY == rc && MILLIS() - start_time < 10);

    start_time = MILLIS();

    do {
        rc = CDC_Transmit_HS((uint8_t *)data, len);
    } while (USBD_BUSY == rc && MILLIS() - start_time < 10);

    usb_serial_buffer_idx = 0;

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

        // pause_sensors();
        pause_storage();
    } else if (s_last_paused) {
        start_storage();
        // start_sensors();

        s_last_paused = false;
    }
}

void init_task() {
    // Suspend all tasks until initialization is complete
    vTaskSuspendAll();

    memset(usb_serial_buffer, 0, SERIAL_BUFFER_SIZE);
    uint32_t init_error = 0;  // Set if error occurs during initialization

    printf("Starting initialization...\n");
    init_error |=
        (EXPECT_OK(init_storage(usb_mode), "init storage") != STATUS_OK) << 0;
    if (usb_mode == 1) {
        init_error |= (EXPECT_OK(init_sensors(), "init sensors") != STATUS_OK)
                      << 1;
        init_error |= (EXPECT_OK(init_state_est(), "init state") != STATUS_OK)
                      << 2;
        init_error |= (EXPECT_OK(init_pyros(), "init pyros") != STATUS_OK) << 3;
        init_error |= (EXPECT_OK(pspcom_init(), "init pspcom") != STATUS_OK)
                      << 4;
    }

    MX_USB_DEVICE_Init(usb_mode);

    DELAY(4700);
    usb_initialized = 1;

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

    // Start tasks if we are in normal mode
    if (usb_mode == 1) {
        printf("Launching tasks\n");
        TASK_CREATE(pyros_task, +5, 2048);
        TASK_CREATE(read_sensors_task, +4, 2048);
        TASK_CREATE(state_est_task, +3, 2048);
        TASK_CREATE(pspcom_process_bytes, +3, 2048);
        TASK_CREATE(pspcom_send_standard, +2, 2048);
        TASK_CREATE(read_gps_task, +2, 2048);
        TASK_CREATE(storage_task, +1, 4096);
    } else {
        // MTP mode data queuing task
        TASK_CREATE(mtp_readwrite_file_task, +1, 2048);
    }
#ifdef DEBUG_MEMORY_USAGE
    TASK_CREATE(debug_memory_usage_task, +1, 512);
#endif

    xTaskResumeAll();

    while (1) {
        DELAY(0xFFFF);
    }
}

int main(void) {
    uxTopUsedPriority = configMAX_PRIORITIES - 1;
    HAL_Init();
    SystemClock_Config();
    init_timers();
    gpio_mode(PIN_PAUSE, GPIO_INPUT_PULLUP);
    gpio_write(PIN_RED, GPIO_HIGH);

    gpio_mode(PIN_USB_MODE, GPIO_INPUT_PULLUP);
    DELAY(1);
    usb_mode = gpio_read(PIN_USB_MODE);

    // https://www.freertos.org/RTOS-Cortex-M3-M4.html
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    TASK_CREATE(init_task, -1, 8192);

    printf("Starting scheduler\n");

    vTaskStartScheduler();

    while (1) {
        Gyro gyr = bmi088_gyro_read(&gyro_i2c_device);
        Accel acc = bmi088_acc_read(&acc_i2c_device);

        printf("Gyro: %g %g %g\n", gyr.gyroX, gyr.gyroY, gyr.gyroZ);
        printf("Acc: %g %g %g\n", acc.accelX, acc.accelY, acc.accelZ);
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
    while (1) {
    }
}

void MemManage_Handler(void) {
    while (1) {
    }
}

void BusFault_Handler(void) {
    while (1) {
    }
}

void UsageFault_Handler(void) {
    while (1) {
    }
}

void DebugMon_Handler(void) {}

void OTG_HS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS); }
