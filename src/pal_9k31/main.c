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
#include "timer.h"

#ifdef USE_SPI_CRC
#undef USE_SPI_CRC
#endif
#define USE_SPI_CRC 0

#define PIN_RED PIN_PC0
#define PIN_YELLOW PIN_PC1
#define PIN_GREEN PIN_PC2
#define PIN_BLUE PIN_PC3
#define BUZZER_PIN PIN_PB9

#define TARGET_INTERVAL 1000  // ms

#define DEBUG

TIM_HandleTypeDef tim6_handle;

lfs_t lfs;
lfs_file_t file;

Status init_flash_fs();

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

void init_tim6() {
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM_Base_InitTypeDef tim6_conf = {
        .Prescaler = 16800,
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = TARGET_INTERVAL * 10,
        .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE,
    };
    tim6_handle.Init = tim6_conf;
    tim6_handle.Instance = TIM6;
    tim6_handle.Channel = TIM_CHANNEL_1;
    TIM_MasterConfigTypeDef tim6_master_conf = {
        .MasterOutputTrigger = TIM_TRGO_RESET,
        .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
    };

    HAL_TIM_Base_Init(&tim6_handle);
    HAL_TIMEx_MasterConfigSynchronization(&tim6_handle, &tim6_master_conf);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void set_tim6_it(uint8_t setting) {
    if (setting) {
        HAL_TIM_Base_Start_IT(&tim6_handle);
    } else {
        HAL_TIM_Base_Stop_IT(&tim6_handle);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    init_timers();
    init_tim6();
    gpio_write(PIN_PA4, GPIO_HIGH);
    gpio_write(PIN_PB12, GPIO_HIGH);
    gpio_write(PIN_PE4, GPIO_HIGH);
    gpio_write(PIN_RED, GPIO_HIGH);
    MX_USB_Device_Init();
    DELAY(1000);
    printf("Starting initialization...\n");
    if (mt29f2g_init() == STATUS_OK) {
        printf("Flash chip initialization successful\n");
    } else {
        printf("Flash chip initialization failed\n");
    }

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
    if (init_flash_fs() != STATUS_OK) {
        printf("Flash filesystem initialization failed.\n");
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
        .clk = SPI_SPEED_10MHz,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = P_SPI4,
    };
    SpiDevice acc_conf = {
        .clk = SPI_SPEED_1MHz,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = P_SPI2,
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

    // Initialize accelerometer
    if (adxl372_init(&acc_conf, ADXL372_200_HZ, ADXL372_OUT_RATE_400_HZ,
                     ADXL372_MEASURE_MODE)) {
        printf("Accelerometer initialization successful");
    } else {
        printf("Accelrometer initialization failed");
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
    gpio_write(PIN_RED, GPIO_LOW);

    Accel accel, acch;
    Gyro gyro;
    BaroData baro;
    Mag mag;
    GPS_Fix_TypeDef fix;
    uint64_t last_time = MILLIS();
    uint64_t read_time = MILLIS();

    set_tim6_it(1);

    while (1) {
        accel = lsm6dsox_read_accel(&imu_conf);
        gyro = lsm6dsox_read_gyro(&imu_conf);
        acch = adxl372_read_accel(&acc_conf);
        baro = ms5637_read(&baro_conf, OSR_256);
        mag = iis2mdc_read(&mag_conf);

        read_time = last_time = MILLIS();
        gpio_write(PIN_YELLOW, GPIO_HIGH);

        if (isnan(baro.pressure)) {
            printf("Barometer read error\n");
        } else {
            printf("Read baro in %d ms\n", (int)(MILLIS() - read_time));
            printf("Temperature %6f (deg C)\n", baro.temperature);
            printf("Pressure %6f (mbar)\n", baro.pressure);
        }
        read_time = MILLIS();

        if (isnan(accel.accelX) || isnan(gyro.gyroX)) {
            printf("IMU read error\n");
        } else {
            printf("Read IMU in %d ms\n", (int)(MILLIS() - read_time));
            printf("Acceleration - x: %6f, y: %6f, z: %6f (g)\n", accel.accelX,
                   accel.accelY, accel.accelZ);
            printf("Rotation - x: %6f, y: %6f, z: %6f (deg/s)\n", gyro.gyroX,
                   gyro.gyroY, gyro.gyroZ);
        }
        read_time = MILLIS();

        if (isnan(mag.magX)) {
            printf("Magnetometer read error\n");
        } else {
            printf("Read mag in %d ms\n", (int)(MILLIS() - read_time));
            printf("Magnetic Field - x: %6f, y: %6f, z: %6f (G)\n", mag.magX,
                   mag.magY, mag.magZ);
        }
        read_time = MILLIS();

        if (isnan(acch.accelX)) {
            printf("Accelerometer read error\n");
        } else {
            printf("Read acc in %d ms\n", (int)(MILLIS() - read_time));
            printf("Acceleration - x: %6f, y: %6f, z: %6f (g)\n", acch.accelX,
                   acch.accelY, acch.accelZ);
        }
        read_time = MILLIS();

        if (max_m10s_poll_fix(&gps_conf, &fix) == STATUS_OK) {
            printf("Read GPS in %d ms\n", (int)(MILLIS() - read_time));
            printf(
                "Latitude: %f (deg), Longitude: %f (deg), Altitude: %f "
                "(m)\n",
                fix.lat, fix.lon, fix.height_msl);
            printf("Sats: %d, Valid fix: %d\n", fix.num_sats, fix.fix_valid);
        } else {
            printf("GPS read error\n");
        }
        read_time = MILLIS();

        if (sd_write(MILLIS(), &accel, &gyro, &baro, &mag, &fix) == STATUS_OK) {
            printf("Wrote SD card in %d ms\n", (int)(MILLIS() - read_time));
            printf("Logged successfully\n");
            gpio_write(PIN_GREEN, GPIO_HIGH);
        } else {
            printf("SD write error\n");
            gpio_write(PIN_GREEN, GPIO_LOW);
        }
        printf("\n");

        gpio_write(PIN_YELLOW, GPIO_LOW);

        // If PROG switch is set, unmount SD card and wait
        if (gpio_read(PIN_PB8)) {
            sd_deinit();
            while (gpio_read(PIN_PB8)) {
                gpio_write(PIN_GREEN, GPIO_HIGH);
                DELAY(500);
                gpio_write(PIN_GREEN, GPIO_LOW);
                DELAY(500);
            }
            sd_reinit();
        }
    }
}

Status init_flash_fs() {
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

void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&tim6_handle);
    gpio_write(PIN_BLUE, GPIO_HIGH);
    DELAY(TARGET_INTERVAL / 2);
    gpio_write(PIN_BLUE, GPIO_LOW);
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

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void USB_LP_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }