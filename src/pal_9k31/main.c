#include <errno.h>
#include <sys/unistd.h>

#include "USB_Device/App/usb_device.h"
#include "USB_Device/App/usbd_cdc_if.h"
#include "board.h"
#include "clocks.h"
#include "gpio/gpio.h"
#include "lsm6dsox/lsm6dsox.h"
#include "ms5637/ms5637.h"
#include "timer.h"

#define LED_PIN PIN_PC0

#define TARGET_RATE 100

int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

    HAL_StatusTypeDef status = CDC_Transmit_FS((uint8_t *)data, len);

    return (status == HAL_OK ? len : 0);
}

int main(void) {
    HAL_Init();
    init_clocks();
    init_timers();

    MX_USB_Device_Init();

    uint32_t lastTime = 0;

    I2cDevice baroConf = {
        .address = 0x76,
        .clk = I2C_SPEED_STANDARD,
        .periph = P_I2C3,
    };
    SpiDevice imuConf = {
        .clk = SPI_SPEED_1MHz,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = P_SPI1,
    };

    // Initialize barometer
    ms5637_init(&baroConf);

    // Initialize IMU
    lsm6dsox_init(&imuConf);

    lsm6dsox_config_accel(&imuConf, LSM6DSOX_XL_RATE_208_HZ,
                          LSM6DSOX_XL_RANGE_8_G);

    lsm6dsox_config_gyro(&imuConf, LSM6DSOX_G_RATE_208_HZ,
                         LSM6DSOX_G_RANGE_500_DPS);

    /*while (1) {
        gpio_write(LED_PIN, GPIO_HIGH);
        DELAY(1000);
        gpio_write(LED_PIN, GPIO_LOW);
        DELAY(1000);
    }*/

    printf("PAL 9000 initialization took %lld milliseconds\n", MILLIS());

    while (1) {
        while (MILLIS() - lastTime < 1000 / TARGET_RATE) {
        }
        lastTime = MILLIS();

        lsm6dsox_read_accel(&imuConf);
        lsm6dsox_read_gyro(&imuConf);
        ms5637_read(&baroConf, OSR_256);
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
