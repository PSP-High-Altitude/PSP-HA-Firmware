#include "sensors.h"

#include <stdio.h>

#include "i2c/i2c.h"
#include "iis2mdc/iis2mdc.h"
#include "kx134/kx134.h"
#include "lsm6dsox/lsm6dsox.h"
#include "max_m10s.h"
#include "ms5637/ms5637.h"
#include "spi/spi.h"
#include "state.h"
#include "storage.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

#ifdef HWIL_TEST
#include "hwil/hwil.h"
#endif

/*********************/
/* PERIPHERAL CONFIG */
/*********************/
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

/********************/
/* STATIC VARIABLES */
/********************/
static bool s_pause_sensors;

static SensorFrame s_last_sensor_frame;
static GPS_Fix_TypeDef s_last_gps_fix;

/********************/
/* HELPER FUNCTIONS */
/********************/
GpsFrame gps_fix_to_pb_frame(uint64_t timestamp,
                             const GPS_Fix_TypeDef* gps_fix) {
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

/*****************/
/* API FUNCTIONS */
/*****************/
Status init_sensors() {
    ASSERT_OK(iis2mdc_init(&s_mag_conf, IIS2MDC_ODR_100_HZ), "mag init");
    ASSERT_OK(ms5637_init(&s_baro_conf), "baro init");
    ASSERT_OK(lsm6dsox_init(&s_imu_conf), "IMU init");
    ASSERT_OK(lsm6dsox_config_accel(&s_imu_conf, LSM6DSOX_XL_RATE_208_HZ,
                                    LSM6DSOX_XL_RANGE_16_G),
              "IMU accel config");
    ASSERT_OK(lsm6dsox_config_gyro(&s_imu_conf, LSM6DSOX_G_RATE_208_HZ,
                                   LSM6DSOX_G_RANGE_500_DPS),
              "IMU gyro config");
    ASSERT_OK(kx134_init(&s_acc_conf, KX134_OUT_RATE_200_HZ, KX134_RANGE_64_G),
              "accel init");
    ASSERT_OK(max_m10s_init(&s_gps_conf), "GPS init");

    // Initialize pause flag
    s_pause_sensors = false;

    return STATUS_OK;
}

void pause_sensors() { s_pause_sensors = true; }

void start_sensors() { s_pause_sensors = false; }

SensorFrame* get_last_sensor_frame() { return &s_last_sensor_frame; }

GPS_Fix_TypeDef* get_last_gps_fix() { return &s_last_gps_fix; }

void read_sensors_task() {
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

#ifdef HWIL_TEST
        SensorFrame hwil_sensor_frame;
        if (get_hwil_sensor_frame(&hwil_sensor_frame) == STATUS_OK) {
            s_last_sensor_frame = hwil_sensor_frame;
        }
#endif  // HWIL_TEST

        queue_sensor_store(&s_last_sensor_frame);
        update_latest_sensor_frame(&s_last_sensor_frame);

        if (s_pause_sensors) {
            vTaskDelayUntil(&last_sensor_read_ticks,
                            pdMS_TO_TICKS(SENSOR_SLEEP_READ_INTERVAL_MS));
        } else {
            vTaskDelayUntil(&last_sensor_read_ticks,
                            pdMS_TO_TICKS(SENSOR_NORMAL_READ_INTERVAL_MS));
        }
    }
}

void read_gps_task() {
    gpio_write(PIN_BLUE, GPIO_LOW);
    TickType_t last_gps_read_ticks = xTaskGetTickCount();

    while (1) {
        EXPECT_OK(max_m10s_poll_fix(&s_gps_conf, &s_last_gps_fix), "GPS read");

#ifdef HWIL_TEST
        GPS_Fix_TypeDef hwil_gps_fix;
        if (get_hwil_gps_fix(&hwil_gps_fix) == STATUS_OK) {
            s_last_gps_fix = hwil_gps_fix;
        }
#endif

        gpio_write(PIN_BLUE, s_last_gps_fix.fix_valid != 0);

        GpsFrame gps_frame = gps_fix_to_pb_frame(MICROS(), &s_last_gps_fix);
        update_latest_gps_fix(&s_last_gps_fix);
        queue_gps_store(&gps_frame);

        if (s_pause_sensors) {
            vTaskDelayUntil(&last_gps_read_ticks,
                            pdMS_TO_TICKS(GPS_SLEEP_READ_INTERVAL_MS));
        } else {
            vTaskDelayUntil(&last_gps_read_ticks,
                            pdMS_TO_TICKS(GPS_NORMAL_READ_INTERVAL_MS));
        }
    }
}
