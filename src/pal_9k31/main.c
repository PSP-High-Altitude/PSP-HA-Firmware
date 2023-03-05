#include <errno.h>
#include <sys/unistd.h>

#include "USB_Device/App/usb_device.h"
#include "USB_Device/App/usbd_cdc_if.h"
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
#include "status.h"
#include "timer.h"

#ifdef USE_SPI_CRC
#undef USE_SPI_CRC
#endif
#define USE_SPI_CRC 0

#define LED_PIN PIN_PC1
#define BUZZER_PIN PIN_PB9

#define TARGET_RATE 100

#define DEBUG

lfs_t lfs;
lfs_file_t file;

Status init_flash();

extern PCD_HandleTypeDef hpcd_USB_FS;

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

int main(void) {
    HAL_Init();
    SystemClock_Config();
    init_timers();
    gpio_write(PIN_PA4, GPIO_HIGH);
    gpio_write(PIN_PB12, GPIO_HIGH);
    gpio_write(PIN_PE4, GPIO_HIGH);
    DELAY(1000);
    MX_USB_Device_Init();
    DELAY(1000);
    printf("Starting initialization...\n");

    /*
    gpio_write(BUZZER_PIN, GPIO_HIGH);
    DELAY(50);
    gpio_write(BUZZER_PIN, GPIO_LOW);
    DELAY(50);
    gpio_write(BUZZER_PIN, GPIO_HIGH);
    DELAY(50);
    gpio_write(BUZZER_PIN, GPIO_LOW);
    DELAY(50);
    */

    /*
    if (init_flash() != STATUS_OK) {
        printf("Flash initialization failed.\n");
    } else {
        uint32_t boot_count = 0;
        lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));
        boot_count += 1;
        lfs_file_rewind(&lfs, &file);
        lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));
        lfs_file_close(&lfs, &file);

        printf("Welcome to PAL 9000, boot %lu\n", boot_count);
    }
    */

    I2cDevice mag_conf = {
        .address = 0x1E,
        .clk = I2C_SPEED_FAST,
        .periph = P_I2C3,
    };
    I2cDevice baro_conf = {
        .address = 0x76,
        .clk = I2C_SPEED_FAST,
        .periph = P_I2C3,
    };
    I2cDevice gps_conf = {
        .address = 0x42,
        .clk = I2C_SPEED_FAST,
        .periph = P_I2C2,
    };
    SpiDevice imu_conf = {
        .clk = SPI_SPEED_1MHz,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = P_SPI1,
    };
    SpiDevice sd_conf = {
        .clk = SPI_SPEED_1MHz,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = P_SPI4,
    };

    // Initialize magnetometer
    if (iis2mdc_init(&mag_conf, IIS2MDC_ODR_50_HZ) == STATUS_OK) {
        printf("Magnetometer initialization successful\n");
    } else {
        printf("Magnetometer initialization failed\n");
    }

    // Initialize barometer
    if (ms5637_init(&baro_conf) == STATUS_OK) {
        printf("Barometer initialization successful\n");
    } else {
        printf("Barometer initialization failed\n");
    }

    // Initialize IMU
    if (lsm6dsox_init(&imu_conf) == STATUS_OK) {
        printf("IMU initialization successful\n");
    } else {
        printf("IMU initialization failed\n");
    }

    if (lsm6dsox_config_accel(&imu_conf, LSM6DSOX_XL_RATE_208_HZ,
                              LSM6DSOX_XL_RANGE_8_G) == STATUS_OK) {
        printf("IMU accel range set successfully\n");
    } else {
        printf("IMU configuration failed\n");
    }

    if (lsm6dsox_config_gyro(&imu_conf, LSM6DSOX_G_RATE_208_HZ,
                             LSM6DSOX_G_RANGE_500_DPS) == STATUS_OK) {
        printf("IMU gyro range set successfully\n");
    } else {
        printf("IMU configuration failed\n");
    }

    // Initialize GPS
    if (max_m10s_init(&gps_conf) == STATUS_OK) {
        printf("GPS initialization successful\n");
    } else {
        printf("GPS initialization failed\n");
    }

    // Initialize SD card
    if (sd_init(&sd_conf) == STATUS_OK) {
        printf("SD card initialization successful\n");
    } else {
        printf("SD card initialization failed\n");
    }

    printf("\n");

    while (1) {
        gpio_write(LED_PIN, GPIO_HIGH);
        DELAY(1000);
        gpio_write(LED_PIN, GPIO_LOW);
        DELAY(1000);
        Accel accel = lsm6dsox_read_accel(&imu_conf);
        Gyro gyro = lsm6dsox_read_gyro(&imu_conf);
        BaroData baro = ms5637_read(&baro_conf, OSR_256);
        Mag mag = iis2mdc_read(&mag_conf);
        GPS_Fix_TypeDef fix;

        if (isnan(baro.pressure)) {
            printf("Barometer read error\n");
        } else {
            printf("Temperature %6f (deg C)\n", baro.temperature);
            printf("Pressure %6f (mbar)\n", baro.pressure);
        }
        if (isnan(accel.accelX) || isnan(gyro.gyroX)) {
            printf("IMU read error\n");
        } else {
            printf("Acceleration - x: %6f, y: %6f, z: %6f (g)\n", accel.accelX,
                   accel.accelY, accel.accelZ);
            printf("Rotation - x: %6f, y: %6f, z: %6f (deg/s)\n", gyro.gyroX,
                   gyro.gyroY, gyro.gyroZ);
        }
        if (isnan(mag.magX)) {
            printf("Magnetometer read error\n");
        } else {
            printf("Magnetic Field - x: %6f, y: %6f, z: %6f (G)\n", mag.magX,
                   mag.magY, mag.magZ);
        }
        if (max_m10s_poll_fix(&gps_conf, &fix) == STATUS_OK) {
            printf(
                "Latitude: %f (deg), Longitude: %f (deg), Altitude: %f (m)\n",
                fix.lat, fix.lon, fix.height_msl);
        } else {
            printf("GPS read error\n");
        }
        printf("\n");
    }
    /*
    printf("PAL 9000 initialization took %lld milliseconds\n", MILLIS());

    uint32_t lastTime = 0;
    while (1) {
        while (MILLIS() - lastTime < 1000 / TARGET_RATE) {
        }
        lastTime = MILLIS();

        lsm6dsox_read_accel(&imu_conf);
        lsm6dsox_read_gyro(&imu_conf);
        ms5637_read(&baro_conf, OSR_256);
        iis2mdc_read(&mag_conf);
    }
    */
}

Status init_flash() {
    const struct lfs_config cfg = {
        .read = &mt29f2g_read,
        .prog = &mt29f2g_prog,
        .erase = &mt29f2g_erase,
        .sync = &mt29f2g_sync,

        .read_size = 16,
        .prog_size = 16,
        .block_size = 139264,
        .block_count = 2048,
        .cache_size = 16,
        .lookahead_size = 16,
        .block_cycles = 750,
    };
    int err = lfs_mount(&lfs, &cfg);

    if (err) {
        if (lfs_format(&lfs, &cfg)) {
            return STATUS_ERROR;
        }
        if (lfs_mount(&lfs, &cfg)) {
            return STATUS_ERROR;
        }
    }
    return STATUS_OK;
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }

void NMI_Handler(void) {}

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

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void USB_LP_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }