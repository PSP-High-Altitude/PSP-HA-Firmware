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
#define PIN_PROG PIN_PB8
#define PIN_BUZZER PIN_PB9

#define TARGET_INTERVAL 30  // ms

#define LOG_FIFO_LEN 256

#define DEBUG

TIM_HandleTypeDef tim6_handle;

lfs_t lfs;
lfs_file_t file;

Status init_flash_fs();

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

volatile uint32_t s_last_sensor_read_us;

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
        printf("Accelerometer initialization successful");
    } else {
        printf("Accelrometer initialization failed");
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

    set_tim6_it(1);
    printf("\n");

    GPS_Fix_TypeDef fix;

    fifo.ridx = 0;
    fifo.widx = 0;

    uint64_t start_time = MICROS();

    gpio_write(PIN_RED, GPIO_LOW);

    while (1) {
        gpio_write(PIN_YELLOW, GPIO_HIGH);

        // Empty the FIFO to the SD card
        start_time = MICROS();
        uint32_t entries_read = 0;
        while (fifo.ridx != fifo.widx) {
            sd_write_sensor_data(&fifo.queue[fifo.ridx]);
            if (fifo.ridx == LOG_FIFO_LEN - 1) {
                fifo.ridx = 0;
            } else {
                fifo.ridx += 1;
            }
            entries_read += 1;
            if (entries_read == LOG_FIFO_LEN) {
                break;
            }
        }
        printf("%lu FIFO entries read in %lu ms\n", entries_read,
               (uint32_t)(MICROS() - start_time) / 1000);
        printf("Last sensor read took %lu us\n",
               (uint32_t)s_last_sensor_read_us);

        // Check if we have a GPS fix
        start_time = MICROS();
        if (max_m10s_poll_fix(&s_gps_conf, &fix) == STATUS_OK) {
            printf("Sats: %d, Valid fix: %d\n", fix.num_sats, fix.fix_valid);
            sd_write_gps_data(MILLIS(), &fix);
        }
        printf("GPS read in %lu ms\n",
               (uint32_t)(MICROS() - start_time) / 1000);

        // Flush I/O buffers
        start_time = MICROS();
        if (sd_flush() != STATUS_OK) {
            printf("SD flush to disk failed\n");
            gpio_write(PIN_GREEN, GPIO_LOW);
            sd_deinit();
            sd_reinit();
        } else {
            gpio_write(PIN_GREEN, GPIO_HIGH);
        }
        printf("SD flush in %lu ms\n\n",
               (uint32_t)(MICROS() - start_time) / 1000);

        gpio_write(PIN_YELLOW, GPIO_LOW);

        if (fix.fix_valid) {
            gpio_write(PIN_BLUE, GPIO_HIGH);
        } else {
            gpio_write(PIN_BLUE, GPIO_LOW);
        }

        // If PROG switch is set, unmount SD card and wait
        if (gpio_read(PIN_PROG)) {
            set_tim6_it(0);
            sd_deinit();
            printf("SD safe to remove\n");
            while (gpio_read(PIN_PROG)) {
                gpio_write(PIN_GREEN, GPIO_HIGH);
                DELAY(500);
                gpio_write(PIN_GREEN, GPIO_LOW);
                DELAY(500);
            }
            printf("Remounting SD\n\n");
            sd_reinit();
            set_tim6_it(1);
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
    if (!((fifo.widx == fifo.ridx - 1) ||
          (fifo.ridx == 0 && fifo.widx == LOG_FIFO_LEN - 1))) {
        // FIFO is not full
        uint64_t start_time = MICROS();
        SensorData log = {
            .timestamp = MILLIS(),
            .acch = adxl372_read_accel(&s_acc_conf),
            .accel = lsm6dsox_read_accel(&s_imu_conf),
            .gyro = lsm6dsox_read_gyro(&s_imu_conf),
            .mag = iis2mdc_read(&s_mag_conf),
            .baro = ms5637_read(&s_baro_conf, OSR_256),
        };
        s_last_sensor_read_us = MICROS() - start_time;
        fifo.queue[fifo.widx] = log;
        if (fifo.widx == LOG_FIFO_LEN - 1) {
            fifo.widx = 0;
        } else {
            fifo.widx += 1;
        }
    }
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