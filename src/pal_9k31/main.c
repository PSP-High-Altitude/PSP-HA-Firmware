#include <errno.h>
#include <sys/unistd.h>

#include "USB_Device/App/usb_device.h"
#include "USB_Device/App/usbd_cdc_if.h"
#include "adxl372/adxl372.h"
#include "board.h"
#include "clocks.h"
#include "cmsis_os.h"
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

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4,
};

osThreadId_t blink_red_handle;
const osThreadAttr_t blink_red_attributes = {
    .name = "blink_red",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4,
};

osThreadId_t blink_green_handle;
const osThreadAttr_t blink_green_attributes = {
    .name = "blink_green",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4,
};

osThreadId_t blink_blue_handle;
const osThreadAttr_t blink_blue_attributes = {
    .name = "blink_blue",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4,
};

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

void blink_red_1hz() {
    while (1) {
        gpio_write(PIN_RED, GPIO_HIGH);
        vTaskDelay(500);
        gpio_write(PIN_RED, GPIO_LOW);
        vTaskDelay(500);
    }
}

void blink_green_2hz() {
    while (1) {
        gpio_write(PIN_GREEN, GPIO_HIGH);
        vTaskDelay(250);
        gpio_write(PIN_GREEN, GPIO_LOW);
        vTaskDelay(250);
    }
}

void blink_blue_4hz() {
    while (1) {
        gpio_write(PIN_BLUE, GPIO_HIGH);
        vTaskDelay(125);
        gpio_write(PIN_BLUE, GPIO_LOW);
        vTaskDelay(125);
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

    osKernelInitialize();
    blink_red_handle = osThreadNew(blink_red_1hz, NULL, &blink_red_attributes);
    blink_green_handle =
        osThreadNew(blink_green_2hz, NULL, &blink_green_attributes);
    blink_blue_handle =
        osThreadNew(blink_blue_4hz, NULL, &blink_blue_attributes);

    osKernelStart();

    while (1) {
        printf("kernel exited\n");
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