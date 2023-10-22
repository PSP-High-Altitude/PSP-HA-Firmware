#include <errno.h>
#include <sys/unistd.h>

#include "USB_Device/App/usb_device.h"
#include "USB_Device/App/usbd_cdc_if.h"
#include "adxl372/adxl372.h"
#include "board.h"
#include "clocks.h"
#include "data.h"
#include "fatfs/sd.h"
#include "gpio/gpio.h"
#include "iis2mdc/iis2mdc.h"
#include "lfs.h"
#include "lsm6dsox/lsm6dsox.h"
#include "max_m10s.h"
#include "ms5637/ms5637.h"
#include "mt29f2g.h"
#include "qspi/qspi.h"
#include "status.h"
#include "stm32g474xx.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#ifdef USE_SPI_CRC
#undef USE_SPI_CRC
#endif
#define USE_SPI_CRC 0

#define PIN_RED PIN_PC0
#define PIN_YELLOW PIN_PC1
#define PIN_GREEN PIN_PC2
#define PIN_BLUE PIN_PC3
#define PIN_PROG PIN_PB8
#define PIN_BUZZER PIN_PB9

#define TARGET_INTERVAL 25  // ms

#define LOG_FIFO_LEN 256

#define DEBUG

extern PCD_HandleTypeDef hpcd_USB_FS;

volatile static struct {
    SensorData queue[LOG_FIFO_LEN];
    size_t ridx;  // Next index that will be read from
    size_t widx;  // Next index that will be written to
    // ridx == widx -> FIFO is empty
    // (ridx == widx - 1) mod LOG_FIFO_LEN -> FIFO is full
} fifo;

static I2cDevice s_mag_conf = {
    .address = 0x1E,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C3,
};
static I2cDevice s_baro_conf = {
    .address = 0x76,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C3,
};
static I2cDevice s_gps_conf = {
    .address = 0x42,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C2,
};
static SpiDevice s_imu_conf = {
    .clk = SPI_SPEED_1MHz,
    .cpol = 0,
    .cpha = 0,
    .cs = 0,
    .periph = P_SPI1,
};
static SpiDevice s_sd_conf = {
    .clk = SPI_SPEED_10MHz,
    .cpol = 0,
    .cpha = 0,
    .cs = 0,
    .periph = P_SPI4,
};
static SpiDevice s_acc_conf = {
    .clk = SPI_SPEED_1MHz,
    .cpol = 0,
    .cpha = 0,
    .cs = 0,
    .periph = P_SPI2,
};

int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

#ifdef DEBUG
    uint64_t start_time = MILLIS();
    USBD_StatusTypeDef rc = USBD_OK;
    do {
        rc = CDC_Transmit_FS((uint8_t *)data, len);
    } while (USBD_BUSY == rc && MILLIS() - start_time < 10);

    if (USBD_FAIL == rc) {
        return 0;
    }
#endif
    return len;
}

static TaskHandle_t s_read_sensors_handle;
static TaskHandle_t s_read_gps_handle;
static TaskHandle_t s_store_data_handle;

static TickType_t s_last_sensor_read_ticks;

void read_sensors() {
    s_last_sensor_read_ticks = xTaskGetTickCount();
    while (1) {
        if (!((fifo.widx == fifo.ridx - 1) ||
              (fifo.ridx == 0 && fifo.widx == LOG_FIFO_LEN - 1))) {
            // FIFO is not full
            SensorData log = {
                .timestamp = xTaskGetTickCount(),
                .acch = adxl372_read_accel(&s_acc_conf),
                .accel = lsm6dsox_read_accel(&s_imu_conf),
                .gyro = lsm6dsox_read_gyro(&s_imu_conf),
                .mag = iis2mdc_read(&s_mag_conf),
                .baro = ms5637_read(&s_baro_conf, OSR_256),
            };
            fifo.queue[fifo.widx] = log;
            if (fifo.widx == LOG_FIFO_LEN - 1) {
                fifo.widx = 0;
            } else {
                fifo.widx += 1;
            }
            xTaskNotifyGive(s_store_data_handle);
        }
        vTaskDelayUntil(&s_last_sensor_read_ticks,
                        pdMS_TO_TICKS(TARGET_INTERVAL));
    }
}

static GPS_Fix_TypeDef s_last_fix;
volatile static int s_fix_avail = 0;

void read_gps() {
    while (1) {
        while (s_fix_avail) {
            vTaskDelay(1);
        }
        Status code = max_m10s_poll_fix(&s_gps_conf, &s_last_fix);
        if (code == STATUS_OK) {
            gpio_write(PIN_BLUE, GPIO_HIGH);
            s_fix_avail = 1;
        } else {
            gpio_write(PIN_BLUE, GPIO_LOW);
            printf("GPS read failed with code %d\n", code);
        }
    }
}

void store_data() {
    while (1) {
        uint32_t notif_value;
        xTaskNotifyWait(0, 0xffffffffUL, &notif_value, 100);

        // If PROG switch is set, unmount SD card and wait
        if (gpio_read(PIN_PROG)) {
            sd_deinit();
            printf("SD safe to remove\n");
            gpio_write(PIN_BLUE, GPIO_LOW);
            gpio_write(PIN_GREEN, GPIO_LOW);
            while (gpio_read(PIN_PROG)) {
                gpio_write(PIN_GREEN, GPIO_HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_write(PIN_GREEN, GPIO_LOW);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            printf("Remounting SD\n\n");
            sd_reinit();
            fifo.ridx = fifo.widx;
            continue;
        }

        if (notif_value == 0) {
            continue;
        }

        gpio_write(PIN_YELLOW, GPIO_HIGH);

        TickType_t start_ticks = xTaskGetTickCount();

        uint32_t entries_read = 0;
        while (fifo.ridx != fifo.widx) {
            Status code = sd_write_sensor_data(&fifo.queue[fifo.ridx]);
            if (code != STATUS_OK) {
                printf("SD sensor write error %d\n", code);
                gpio_write(PIN_GREEN, GPIO_LOW);
                break;
            }
            gpio_write(PIN_GREEN, GPIO_HIGH);
            if (fifo.ridx == LOG_FIFO_LEN - 1) {
                fifo.ridx = 0;
            } else {
                fifo.ridx += 1;
            }
            entries_read += 1;
            if (entries_read == LOG_FIFO_LEN) {
                gpio_write(PIN_RED, GPIO_HIGH);
                break;
            }
            gpio_write(PIN_RED, GPIO_LOW);
        }

        if (s_fix_avail) {
            Status code = sd_write_gps_data(xTaskGetTickCount(), &s_last_fix);
            if (code != STATUS_OK) {
                printf("SD GPS write error %d\n", code);
            }
            s_fix_avail = 0;
        }

        sd_flush();

        TickType_t elapsed_time = xTaskGetTickCount() - start_ticks;

        gpio_write(PIN_YELLOW, GPIO_LOW);
        printf("%lu entries read in %lu ticks\n", entries_read, elapsed_time);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    init_timers();
    gpio_write(PIN_PA4, GPIO_HIGH);
    gpio_write(PIN_PB12, GPIO_HIGH);
    gpio_write(PIN_PE4, GPIO_HIGH);
    gpio_write(PIN_RED, GPIO_HIGH);
    MX_USB_Device_Init();
    DELAY(1000);
    printf("Starting initialization...\n");

    // Initialize magnetometer
    if (iis2mdc_init(&s_mag_conf, IIS2MDC_ODR_50_HZ) == STATUS_OK) {
        printf("Magnetometer initialization successful\n");
    } else {
        printf("Magnetometer initialization failed\n");
    }

    // Initialize barometer
    if (ms5637_init(&s_baro_conf) == STATUS_OK) {
        printf("Barometer initialization successful\n");
    } else {
        printf("Barometer initialization failed\n");
    }

    // Initialize IMU
    if (lsm6dsox_init(&s_imu_conf) == STATUS_OK) {
        printf("IMU initialization successful\n");
    } else {
        printf("IMU initialization failed\n");
    }

    if (lsm6dsox_config_accel(&s_imu_conf, LSM6DSOX_XL_RATE_208_HZ,
                              LSM6DSOX_XL_RANGE_16_G) == STATUS_OK) {
        printf("IMU accel range set successfully\n");
    } else {
        printf("IMU configuration failed\n");
    }

    if (lsm6dsox_config_gyro(&s_imu_conf, LSM6DSOX_G_RATE_208_HZ,
                             LSM6DSOX_G_RANGE_500_DPS) == STATUS_OK) {
        printf("IMU gyro range set successfully\n");
    } else {
        printf("IMU configuration failed\n");
    }

    // Initialize accelerometer
    if (adxl372_init(&s_acc_conf, ADXL372_200_HZ, ADXL372_OUT_RATE_400_HZ,
                     ADXL372_MEASURE_MODE)) {
        printf("Accelerometer initialization successful\n");
    } else {
        printf("Accelerometer initialization failed\n");
    }

    // Initialize GPS
    if (max_m10s_init(&s_gps_conf) == STATUS_OK) {
        printf("GPS initialization successful\n");
    } else {
        printf("GPS initialization failed\n");
    }

    // Initialize SD card
    if (sd_init(&s_sd_conf) == STATUS_OK) {
        printf("SD card initialization successful\n");
    } else {
        printf("SD card initialization failed\n");
    }

    // https://www.freertos.org/RTOS-Cortex-M3-M4.html
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    gpio_write(PIN_RED, GPIO_LOW);

    xTaskCreate(store_data,            // Task function
                "store_data",          // Task name
                2048,                  // Stack size
                NULL,                  // Parameters
                tskIDLE_PRIORITY + 2,  // Priority
                &s_store_data_handle   // Task handle
    );

    xTaskCreate(read_sensors,           // Task function
                "read_sensors",         // Task name
                2048,                   // Stack size
                NULL,                   // Parameters
                tskIDLE_PRIORITY + 3,   // Priority
                &s_read_sensors_handle  // Task handle
    );

    xTaskCreate(read_gps,              // Task function
                "read_gps",            // Task name
                2048,                  // Stack size
                NULL,                  // Parameters
                tskIDLE_PRIORITY + 2,  // Priority
                &s_read_gps_handle     // Task handle
    );

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
    HAL_IncTick();

    /* Clear overflow flag */
    SysTick->CTRL;

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

void NMI_Handler(void) {}

void HardFault_Handler(void) {
    while (1) {
        printf("hard fault\n");
    }
}

void MemManage_Handler(void) {
    while (1) {
        printf("memmanage\n");
    }
}

void BusFault_Handler(void) {
    while (1) {
        printf("bus fault\n");
    }
}

void UsageFault_Handler(void) {
    while (1) {
        printf("usage fault\n");
    }
}

void DebugMon_Handler(void) {}

void USB_LP_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }
