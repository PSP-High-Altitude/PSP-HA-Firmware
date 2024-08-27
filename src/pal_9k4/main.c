#include "main.h"

#include <errno.h>
#include <sys/unistd.h>

#include "USB_Device/App/usb_device.h"
#include "USB_Device/App/usbd_cdc_if.h"
#include "backup.h"
#include "bmi088/bmi088.h"
#include "clocks.h"
#include "data.h"
#include "gpio/gpio.h"
#include "nand_flash.h"
#include "pspcom.h"
#include "pyros.h"
#include "sensors.h"
#include "state.h"
#include "status.h"
#include "stm32h7xx.h"
#include "storage.h"
#include "timer.h"

I2cDevice acc_i2c_device = {
    .address = 0x18, .clk = I2C_SPEED_FAST, .periph = P_I2C1};

I2cDevice gyro_i2c_device = {
    .address = 0x68, .clk = I2C_SPEED_FAST, .periph = P_I2C1};

int main(void) {
    if (bmi088_init(&acc_i2c_device, &gyro_i2c_device, BMI088_GYRO_RATE_2000_HZ,
                    BMI088_ACC_RATE_1600_HZ, BMI088_GYRO_RANGE_2000_DPS,
                    BMI088_ACC_RANGE_24_G) != STATUS_OK) {
        printf("Error initializing BMI088\n");
    }

    while (1) {
        Gyro gyr = bmi088_gyro_read(&gyro_i2c_device);
        Accel acc = bmi088_acc_read(&acc_i2c_device);

        printf("Gyro: %g %g %g\n", gyr.gyroX, gyr.gyroY, gyr.gyroZ);
        printf("Acc: %g %g %g\n", acc.accelX, acc.accelY, acc.accelZ);
    }
}