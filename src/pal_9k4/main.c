#include "main.h"

#include <errno.h>
#include <sys/unistd.h>

#include "USB_Device/App/usb_device.h"
#include "USB_Device/App/usbd_cdc_if.h"
#include "clocks.h"
#include "data.h"
#include "gpio/gpio.h"
#include "iis2mdc/iis2mdc.h"
#include "kx134/kx134.h"
#include "lfs.h"
#include "lsm6dsox/lsm6dsox.h"
#include "max_m10s.h"
#include "ms5637/ms5637.h"
#include "mt29f2g.h"
#include "pb.h"
#include "pspcom.h"
#include "sd.h"
#include "status.h"
#include "stm32h7xx.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#ifdef USE_SPI_CRC
#undef USE_SPI_CRC
#endif
#define USE_SPI_CRC 0

#define PIN_RED PIN_PA0
#define PIN_YELLOW PIN_PA1
#define PIN_GREEN PIN_PA2
#define PIN_BLUE PIN_PA3
#define PIN_PROG PIN_PA13
#define PIN_BUZZER PIN_PE0

#define TARGET_INTERVAL 40  // ms

#define LOG_FIFO_LEN 256

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
SensorFrame s_last_sensor_data;
GPS_Fix_TypeDef s_last_fix;
static TickType_t s_last_sensor_read_ticks;
volatile static int s_fix_avail = 0;

static TaskHandle_t s_read_sensors_handle;
static TaskHandle_t s_read_gps_handle;
static TaskHandle_t s_store_data_handle;
static TaskHandle_t s_gps_telem_handle;
static TaskHandle_t s_status_telem_handle;
static TaskHandle_t s_process_commands_handle;
#ifdef PSPCOM_SENSORS
static TaskHandle_t s_sensor_telem_handle;
#endif

volatile static struct {
    SensorFrame queue[LOG_FIFO_LEN];
    size_t head;  // Next index that will be read from
    size_t tail;  // Next index that will be written to
    size_t count;
    // ridx == widx -> FIFO is empty
    // (ridx == widx - 1) mod LOG_FIFO_LEN -> FIFO is full
} fifo;

void init_fifo() {
    fifo.head = 0;
    fifo.tail = 0;
    fifo.count = 0;
}

SensorFrame read_fifo() {
    SensorFrame ret = fifo.queue[fifo.head];
    fifo.head = (fifo.head + 1) % LOG_FIFO_LEN;
    fifo.count--;
    return ret;
}

void write_fifo(SensorFrame data) {
    if (fifo.count == LOG_FIFO_LEN) {
        fifo.head = (fifo.head + 1) % LOG_FIFO_LEN;
        fifo.count--;
    }
    fifo.queue[fifo.tail] = data;
    fifo.tail = (fifo.tail + 1) % LOG_FIFO_LEN;
    fifo.count++;
}

static I2cDevice s_mag_conf = {
    .address = 0x1E,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C1,
};
static I2cDevice s_baro_conf = {
    .address = 0x76,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C1,
};
static I2cDevice s_gps_conf = {
    .address = 0x42,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C2,
};
static SpiDevice s_imu_conf = {
    .clk = SPI_SPEED_10MHz,
    .cpol = 0,
    .cpha = 0,
    .cs = 0,
    .periph = P_SPI1,
};
static SdDevice s_sd_conf = {
    .clk = SD_SPEED_10MHz,
    .periph = P_SD4,
};
static SpiDevice s_acc_conf = {
    .clk = SPI_SPEED_10MHz,
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
        rc = CDC_Transmit_HS((uint8_t *)data, len);
    } while (USBD_BUSY == rc && MILLIS() - start_time < 10);

    if (USBD_FAIL == rc) {
        return 0;
    }
#endif
    return len;
}

void read_sensors() {
    s_last_sensor_read_ticks = xTaskGetTickCount();
    while (1) {
        BaroData baro =
            ms5637_read(&s_baro_conf, OSR_256);    // Baro read takes longest
        uint64_t timestamp = xTaskGetTickCount();  // So measure timestamp after
        Accel acch = kx134_read_accel(&s_acc_conf);
        Accel accel = lsm6dsox_read_accel(&s_imu_conf);
        Gyro gyro = lsm6dsox_read_gyro(&s_imu_conf);
        Mag mag = iis2mdc_read(&s_mag_conf);

        s_last_sensor_data.timestamp = timestamp;

        s_last_sensor_data.acc_h_x = acch.accelX;
        s_last_sensor_data.acc_h_y = acch.accelY;
        s_last_sensor_data.acc_h_z = acch.accelZ;

        s_last_sensor_data.acc_i_x = accel.accelX;
        s_last_sensor_data.acc_i_y = accel.accelY;
        s_last_sensor_data.acc_i_z = accel.accelZ;

        s_last_sensor_data.rot_i_x = gyro.gyroX;
        s_last_sensor_data.rot_i_y = gyro.gyroY;
        s_last_sensor_data.rot_i_z = gyro.gyroZ;

        s_last_sensor_data.mag_i_x = mag.magX;
        s_last_sensor_data.mag_i_y = mag.magY;
        s_last_sensor_data.mag_i_z = mag.magZ;

        s_last_sensor_data.temperature = baro.temperature;
        s_last_sensor_data.pressure = baro.pressure;

        write_fifo(s_last_sensor_data);
        xTaskNotifyGive(s_store_data_handle);
        vTaskDelayUntil(&s_last_sensor_read_ticks,
                        pdMS_TO_TICKS(TARGET_INTERVAL));
    }
}

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

GpsFrame gps_fix_to_pb_frame(uint64_t timestamp,
                             const GPS_Fix_TypeDef *gps_fix) {
    GpsFrame gps_frame;

    // Copy UTC Time
    gps_frame.timestamp = timestamp;
    gps_frame.year = gps_fix->year;
    gps_frame.month = gps_fix->month;
    gps_frame.day = gps_fix->day;
    gps_frame.hour = gps_fix->hour;
    gps_frame.min = gps_fix->min;
    gps_frame.sec = gps_fix->sec;

    // Pack validity flags into a single uint64_t
    gps_frame.valid_flags = ((uint64_t)gps_fix->date_valid << 0) |
                            ((uint64_t)gps_fix->time_valid << 1) |
                            ((uint64_t)gps_fix->time_resolved << 2) |
                            ((uint64_t)gps_fix->fix_type << 3) |
                            ((uint64_t)gps_fix->fix_valid << 8) |
                            ((uint64_t)gps_fix->diff_used << 9) |
                            ((uint64_t)gps_fix->psm_state << 10) |
                            ((uint64_t)gps_fix->hdg_veh_valid << 14) |
                            ((uint64_t)gps_fix->carrier_phase << 15) |
                            ((uint64_t)gps_fix->invalid_llh << 19);

    // Copy Navigation info
    gps_frame.num_sats = gps_fix->num_sats;
    gps_frame.lon = gps_fix->lon;
    gps_frame.lat = gps_fix->lat;
    gps_frame.height = gps_fix->height;
    gps_frame.height_msl = gps_fix->height_msl;
    gps_frame.accuracy_horiz = gps_fix->accuracy_horiz;
    gps_frame.accuracy_vertical = gps_fix->accuracy_vertical;
    gps_frame.vel_north = gps_fix->vel_north;
    gps_frame.vel_east = gps_fix->vel_east;
    gps_frame.vel_down = gps_fix->vel_down;
    gps_frame.ground_speed = gps_fix->ground_speed;
    gps_frame.hdg = gps_fix->hdg;
    gps_frame.accuracy_speed = gps_fix->accuracy_speed;
    gps_frame.accuracy_hdg = gps_fix->accuracy_hdg;

    return gps_frame;
}

void store_data() {
    while (1) {
        uint32_t notif_value;
        xTaskNotifyWait(0, 0xffffffffUL, &notif_value, 100);

        // If PROG switch is set, unmount SD card and wait
        if (!gpio_read(PIN_PROG)) {
            sd_deinit();
            printf("SD safe to remove\n");
            gpio_write(PIN_BLUE, GPIO_LOW);
            gpio_write(PIN_GREEN, GPIO_LOW);
            while (!gpio_read(PIN_PROG)) {
                gpio_write(PIN_GREEN, GPIO_HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_write(PIN_GREEN, GPIO_LOW);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            printf("Remounting SD\n\n");
            sd_reinit();
            continue;
        }

        if (notif_value == 0) {
            continue;
        }

        gpio_write(PIN_YELLOW, GPIO_HIGH);

        // TickType_t start_ticks = xTaskGetTickCount();

        uint32_t entries_read = 0;
        while (fifo.count > 0) {
            SensorFrame log = read_fifo();
            Status code = sd_write_sensor_data(&log);
            if (code != STATUS_OK) {
                // printf("SD sensor write error %d\n", code);
                gpio_write(PIN_GREEN, GPIO_LOW);
                break;
            }
            gpio_write(PIN_GREEN, GPIO_HIGH);
            entries_read += 1;
            if (entries_read == LOG_FIFO_LEN) {
                gpio_write(PIN_RED, GPIO_HIGH);
                break;
            }
            gpio_write(PIN_RED, GPIO_LOW);
        }

        if (s_fix_avail) {
            // This is an expensive copy but ideally we don't want to expose the
            // raw GPS format to the SD HAL
            GpsFrame gps_frame =
                gps_fix_to_pb_frame(xTaskGetTickCount(), &s_last_fix);
            Status code = sd_write_gps_data(&gps_frame);
            if (code != STATUS_OK) {
                // printf("SD GPS write error %d\n", code);
            }
            s_fix_avail = 0;
        }

        sd_flush();

        // TickType_t elapsed_time = xTaskGetTickCount() - start_ticks;

        gpio_write(PIN_YELLOW, GPIO_LOW);
        // printf("%lu entries read in %lu ticks\n", entries_read,
        // elapsed_time);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    init_timers();
    init_fifo();
    gpio_mode(PIN_PROG, GPIO_INPUT_PULLUP);
    gpio_write(PIN_PA4, GPIO_HIGH);
    gpio_write(PIN_PD8, GPIO_HIGH);
    gpio_write(PIN_PE4, GPIO_HIGH);
    MX_USB_DEVICE_Init();
    gpio_write(PIN_RED, GPIO_HIGH);

    gpio_write(PIN_BUZZER, GPIO_HIGH);
    DELAY(100);
    gpio_write(PIN_BUZZER, GPIO_LOW);
    DELAY(100);
    gpio_write(PIN_BUZZER, GPIO_HIGH);
    DELAY(100);
    gpio_write(PIN_BUZZER, GPIO_LOW);

    DELAY(4700);
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
    if (kx134_init(&s_acc_conf, KX134_OUT_RATE_200_HZ, KX134_RANGE_64_G) ==
        STATUS_OK) {
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

    // Initialize PSPCOM connection
    if (pspcom_init() == STATUS_OK) {
        printf("PSPCOM initialization successful\n");
    } else {
        printf("PSPCOM initialization failed\n");
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

    xTaskCreate(pspcom_send_gps,       // Task function
                "gps_telem",           // Task name
                2048,                  // Stack size
                (void *)&s_last_fix,   // Parameters
                tskIDLE_PRIORITY + 1,  // Priority
                &s_gps_telem_handle    // Task handle
    );

    xTaskCreate(pspcom_send_status,     // Task function
                "status_telem",         // Task name
                2048,                   // Stack size
                NULL,                   // Parameters
                tskIDLE_PRIORITY + 1,   // Priority
                &s_status_telem_handle  // Task handle
    );

    xTaskCreate(pspcom_process_bytes,       // Task function
                "process_commands",         // Task name
                2048,                       // Stack size
                NULL,                       // Parameters
                tskIDLE_PRIORITY + 2,       // Priority
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

void NMI_Handler(void) { printf("nmi\n"); }

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

void OTG_HS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS); }
