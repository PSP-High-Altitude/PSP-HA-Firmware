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
#include "lsm6dsox/lsm6dsox.h"
#include "max_m10s.h"
#include "ms5637/ms5637.h"
#include "nand_flash.h"
#include "pspcom.h"
#include "pyros.h"
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

static SensorFrame s_last_sensor_frame;
static GPS_Fix_TypeDef s_last_fix;

PAL_Data_Typedef s_last_data = {&s_last_sensor_frame, &s_last_fix};

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
    .clk = I2C_SPEED_STANDARD,
    .periph = P_I2C2,
};
static SpiDevice s_imu_conf = {
    .clk = SPI_SPEED_10MHz,
    .cpol = 0,
    .cpha = 0,
    .cs = 0,
    .periph = P_SPI1,
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
    TickType_t last_sensor_read_ticks = xTaskGetTickCount();
    while (1) {
        BaroData baro =
            ms5637_read(&s_baro_conf, OSR_256);  // Baro read takes longest
        uint64_t timestamp = MICROS();           // So measure timestamp after
        Accel acch = kx134_read_accel(&s_acc_conf);
        Accel accel = lsm6dsox_read_accel(&s_imu_conf);
        Gyro gyro = lsm6dsox_read_gyro(&s_imu_conf);
        Mag mag = iis2mdc_read(&s_mag_conf);

        // Copy data
        s_last_sensor_frame.timestamp = timestamp;

        s_last_sensor_frame.acc_h_x = acch.accelX;
        s_last_sensor_frame.acc_h_y = acch.accelY;
        s_last_sensor_frame.acc_h_z = acch.accelZ;

        s_last_sensor_frame.acc_i_x = accel.accelX;
        s_last_sensor_frame.acc_i_y = accel.accelY;
        s_last_sensor_frame.acc_i_z = accel.accelZ;

        s_last_sensor_frame.rot_i_x = gyro.gyroX;
        s_last_sensor_frame.rot_i_y = gyro.gyroY;
        s_last_sensor_frame.rot_i_z = gyro.gyroZ;

        s_last_sensor_frame.mag_i_x = mag.magX;
        s_last_sensor_frame.mag_i_y = mag.magY;
        s_last_sensor_frame.mag_i_z = mag.magZ;

        s_last_sensor_frame.temperature = baro.temperature;
        s_last_sensor_frame.pressure = baro.pressure;

        queue_sensor_store(&s_last_sensor_frame);
        update_latest_sensor_frame(&s_last_sensor_frame);
        vTaskDelayUntil(&last_sensor_read_ticks,
                        pdMS_TO_TICKS(TARGET_INTERVAL));
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

void read_gps() {
    gpio_write(PIN_BLUE, GPIO_LOW);
    while (1) {
        EXPECT_OK(max_m10s_poll_fix(&s_gps_conf, &s_last_fix), "GPS read");

        gpio_write(PIN_BLUE, s_last_fix.fix_valid != 0);

        GpsFrame gps_frame = gps_fix_to_pb_frame(MICROS(), &s_last_fix);
        queue_gps_store(&gps_frame);
    }
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

    // Initialize sensor data
    s_last_sensor_frame.timestamp = 0;

    // Initialize magnetometer
    if (iis2mdc_init(&s_mag_conf, IIS2MDC_ODR_100_HZ) == STATUS_OK) {
        printf("Magnetometer initialization successful\n");
    } else {
        printf("Magnetometer initialization failed\n");
        init_error = 1;
    }

    // Initialize barometer
    if (ms5637_init(&s_baro_conf) == STATUS_OK) {
        printf("Barometer initialization successful\n");
    } else {
        printf("Barometer initialization failed\n");
        init_error = 1;
    }

    // Initialize IMU
    if (lsm6dsox_init(&s_imu_conf) == STATUS_OK) {
        printf("IMU initialization successful\n");
    } else {
        printf("IMU initialization failed\n");
        init_error = 1;
    }

    if (lsm6dsox_config_accel(&s_imu_conf, LSM6DSOX_XL_RATE_208_HZ,
                              LSM6DSOX_XL_RANGE_16_G) == STATUS_OK) {
        printf("IMU accel range set successfully\n");
    } else {
        printf("IMU configuration failed\n");
        init_error = 1;
    }

    if (lsm6dsox_config_gyro(&s_imu_conf, LSM6DSOX_G_RATE_208_HZ,
                             LSM6DSOX_G_RANGE_500_DPS) == STATUS_OK) {
        printf("IMU gyro range set successfully\n");
    } else {
        printf("IMU configuration failed\n");
        init_error = 1;
    }

    // Initialize accelerometer
    if (kx134_init(&s_acc_conf, KX134_OUT_RATE_200_HZ, KX134_RANGE_64_G) ==
        STATUS_OK) {
        printf("Accelerometer initialization successful\n");
    } else {
        printf("Accelerometer initialization failed\n");
        init_error = 1;
    }

    // Initialize GPS
    if (max_m10s_init(&s_gps_conf) == STATUS_OK) {
        printf("GPS initialization successful\n");
    } else {
        printf("GPS initialization failed\n");
        init_error = 1;
    }

    // Initialize SD card
    if (init_storage() == STATUS_OK) {
        printf("SD card initialization successful\n");
    } else {
        printf("SD card initialization failed\n");
        init_error = 1;
    }

    /*
    // Initialize flash memory
    if (nand_flash_init() == STATUS_OK) {
        printf("Flash initialization successful\n");
    } else {
        printf("Flash initialization failed\n");
        init_error = 1;
    }
    */

    // Initialize PSPCOM connection
    if (pspcom_init() == STATUS_OK) {
        printf("PSPCOM initialization successful\n");
    } else {
        printf("PSPCOM initialization failed\n");
        init_error = 1;
    }

    // Initialize state estimation
    if (init_state_est() == STATUS_OK) {
        printf("State estimation init successful\n");
    } else {
        printf("State estimation init failed\n");
        init_error = 1;
    }

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

    // https://www.freertos.org/RTOS-Cortex-M3-M4.html
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    xTaskCreate(storage_task,           // Task function
                "storage_task",         // Task name
                2048,                   // Stack size
                NULL,                   // Parameters
                tskIDLE_PRIORITY + 1,   // Priority
                &s_storage_task_handle  // Task handle
    );

    xTaskCreate(read_sensors,           // Task function
                "read_sensors",         // Task name
                2048,                   // Stack size
                NULL,                   // Parameters
                tskIDLE_PRIORITY + 4,   // Priority
                &s_read_sensors_handle  // Task handle
    );

    xTaskCreate(read_gps,              // Task function
                "read_gps",            // Task name
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
                (void *)&s_last_data,     // Parameters
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
