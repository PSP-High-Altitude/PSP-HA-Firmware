#include "sensors.h"

#include <stdio.h>

#include "board.h"
#include "board_config.h"
#include "control.h"
#include "spi/spi.h"
#include "storage.h"
#include "timer.h"

// Sensor drivers
#include "bmi088/bmi088.h"
#include "i2c/i2c.h"
#include "iis2mdc/iis2mdc.h"
#include "kx134/kx134.h"
#include "ms5637/ms5637.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

#ifdef HWIL_TEST
#include "hwil/hwil.h"
#endif

/*********************/
/* PERIPHERAL CONFIG */
/*********************/

// MS5637 Barometer
static I2cDevice s_baro_conf = {
    .address = 0x76,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C1,
    .scl = PIN_PB8,
    .sda = PIN_PB7,
};

//// BMI088 Inertial Measurement Unit (Accelerometer)
// static I2cDevice s_imu_acc_conf = {
//     .address = 0x18,
//     .clk = I2C_SPEED_FAST,
//     .periph = P_I2C1,
//     .scl = PIN_PB8,
//     .sda = PIN_PB7,
// };
//
//// BMI088 Inertial Measurement Unit (Gyroscope)
// static I2cDevice s_imu_rot_conf = {
//     .address = 0x68,
//     .clk = I2C_SPEED_FAST,
//     .periph = P_I2C1,
//     .scl = PIN_PB8,
//     .sda = PIN_PB7,
// };

// KX134 High-G Accelerometer
static I2cDevice s_acc_conf = {
    .address = 0x1F,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C1,
    .scl = PIN_PB8,
    .sda = PIN_PB7,
};

// IIS2MDC Magnetometer
static I2cDevice s_mag_conf = {
    .address = 0x1E,
    .clk = I2C_SPEED_FAST,
    .periph = P_I2C1,
    .scl = PIN_PB8,
    .sda = PIN_PB7,
};

/********************/
/* STATIC VARIABLES */
/********************/
static TaskHandle_t* s_handle_ptr = NULL;
static BoardConfig* s_config_ptr = NULL;

/********************/
/* HELPER FUNCTIONS */
/********************/

/*****************/
/* API FUNCTIONS */
/*****************/
Status sensors_init() {
    ASSERT_OK(ms5637_init(&s_baro_conf), "Barometer initialization failed\n");
    ASSERT_OK(kx134_init(&s_acc_conf, KX134_OUT_RATE_200_HZ, KX134_RANGE_64_G),
              "Accelerometer initialization failed\n");
    ASSERT_OK(iis2mdc_init(&s_mag_conf, IIS2MDC_ODR_100_HZ),
              "Magnetometer initialization failed\n");
    // ASSERT_OK(bmi088_init(&s_imu_acc_conf, &s_imu_rot_conf,
    //                       BMI088_GYRO_RATE_200_HZ, BMI088_ACC_RATE_200_HZ,
    //                       BMI088_GYRO_RANGE_2000_DPS, BMI088_ACC_RANGE_24_G),
    //           "IMU initialization failed\n");

    s_config_ptr = config_get_ptr();
    if (s_config_ptr == NULL) {
        ASSERT_OK(STATUS_STATE_ERROR, "unable to get ptr to config\n");
    }

    return STATUS_OK;
}

Status sensors_start_read() {
    if (s_handle_ptr == NULL) {
        return STATUS_ERROR;
    }
    xTaskNotifyGive(*s_handle_ptr);
    return STATUS_OK;
}

void task_sensors(TaskHandle_t* handle_ptr) {
    s_handle_ptr = handle_ptr;

    while (1) {
        uint32_t notif_value;
        xTaskNotifyWait(0 /* Don't clear any bits on entry */,
                        ULONG_MAX /* Clear all bits on exit */, &notif_value,
                        pdMS_TO_TICKS(s_config_ptr->sensor_loop_period_ms));

        // Read all the sensors, measuring the timestamp after the barometer
        // read since everything else is really fast
        BaroData baro = ms5637_read(&s_baro_conf, OSR_256);
        uint64_t timestamp = MICROS();
        Accel acch = kx134_read_accel(&s_acc_conf);
        Accel accel = {0};  // bmi088_acc_read(&s_imu_acc_conf);
        Gyro gyro = {0};    // bmi088_gyro_read(&s_imu_rot_conf);
        Mag mag = iis2mdc_read(&s_mag_conf);

        // Copy data into a sensor frame
        SensorFrame sensor_frame;
        sensor_frame.timestamp = timestamp;

        sensor_frame.acc_h_x = acch.accelX;
        sensor_frame.acc_h_y = acch.accelY;
        sensor_frame.acc_h_z = acch.accelZ;

        sensor_frame.acc_i_x = accel.accelX;
        sensor_frame.acc_i_y = accel.accelY;
        sensor_frame.acc_i_z = accel.accelZ;

        sensor_frame.rot_i_x = gyro.gyroX;
        sensor_frame.rot_i_y = gyro.gyroY;
        sensor_frame.rot_i_z = gyro.gyroZ;

        sensor_frame.mag_i_x = mag.magX;
        sensor_frame.mag_i_y = mag.magY;
        sensor_frame.mag_i_z = mag.magZ;

        sensor_frame.temperature = baro.temperature;
        sensor_frame.pressure = baro.pressure;

#ifdef HWIL_TEST
        // If we're doing a HWIL test, overwrite the actual sensor frame with
        // one from the test data based on the current system timestamp
        gpio_write(PIN_RED, GPIO_HIGH);
        SensorFrame hwil_sensor_frame;
        if (get_hwil_sensor_frame(&hwil_sensor_frame) == STATUS_OK) {
            sensor_frame = hwil_sensor_frame;
        }
#endif  // HWIL_TEST

        update_sensors_for_control(&sensor_frame);
        queue_sensors_for_storage(&sensor_frame);
    }
}
