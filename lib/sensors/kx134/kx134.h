#ifndef KX134_H
#define KX134_H

#include <stdint.h>

#include "data.h"
#include "spi/spi.h"
#include "status.h"

/*
 * register Definitions (below):
 */

/**
 * This register holds the device ID. Used to check a connection
 * to the device as this address should always contain the same
 * value: 0x46
 *
 * Bits:
 * [7:0] - Device Id = 0x46
 *
 * All the bits in this register have read only access.
 */
#define KX134_WHO_AM_I 0x13

/**
 * Command test register. Holds 0x55 after reset.
 *
 * Bits:
 * [7:0] - Command test response = 0x55
 *
 * All the bits in this register have read only access.
 */
#define KX134_COTR 0x12

/**
 * Status register.
 *
 * Bits:
 * [0] - Wake. 1 = wake, 0 = sleep
 * [3:1] - Reserved.
 * [4] - Interrupt status
 * [7:5] - Reserved.
 *
 * All bits for this register have read only access.
 */
#define KX134_STATUS 0x19

/**
 * KX134 XOUT_L register. An incrementing read of this register
 * will read all the data registers in order.
 *
 * All of these registers are read only.
 */
#define KX134_DATA 0x08

/**
 * CNTL1
 *
 * Bits:
 * [0] - TPE. 0 = disable, 1 = enable
 * [1] - Reserved.
 * [2] - TDTE. 0 = disable, 1 = enable
 * [4:3] - GSEL. 0 = 8g, 1 = 16g, 2 = 32g, 3 = 64g
 * [5] - DRDYE. 0 = disable, 1 = enable
 * [6] - RES. 0 = low power, 1 = high performance
 * [7] - PC1. 0 = standby, 1 = high performance or low power
 */
#define KX134_CNTL1 0x1B

/**
 * CNTL2
 *
 * Bits:
 * [0] - FUM. 0 = disable, 1 = enable
 * [1] - FDM. 0 = disable, 1 = enable
 * [2] - UPM. 0 = disable, 1 = enable
 * [3] - DOM. 0 = disable, 1 = enable
 * [4] - RIM. 0 = disable, 1 = enable
 * [5] - LEM. 0 = disable, 1 = enable
 * [6] - COTC. write 1 for command test
 * [7] - SRST. write 1 to reset all registers
 */
#define KX134_CNTL2 0x1C

/**
 * CNTL3
 *
 * Bits:
 * [2:0] - OWUF. WUF ODR = 0.78125Hz * 2^(OWUF)
 * [5:3] - OTDT. TDT ODR = 12.5Hz * 2^(OTDT)
 * [7:6] - OTP. Tilt ODR = 1.5625Hz * 2^(OTP)
 */
#define KX134_CNTL3 0x1D

/**
 * CNTL4
 *
 * Bits:
 * [2:0] - OBTS. BTS ODR = 0.78125Hz * 2^(OBTS)
 * [3] - PR_MODE. 0 = normal, 1 = pulse reject
 * [4] - BTSE. 0 = disable, 1 = enable
 * [5] - WUFE. 0 = disable, 1 = enable
 * [6] - TH_MODE. 0 = absolute, 1 = relative
 * [7] - C_MODE. 0 = reset counter, 1 = decrement counter
 */
#define KX134_CNTL4 0x1E

/**
 * CNTL5
 *
 * Bits:
 * [0] - MAN_SLEEP. 0 = default, 1 = force sleep
 * [1] - MAN_WAKE. 0 = default, 1 = force wake
 * [3:2] - Reserved.
 * [4] - ADPE. 0 = disable, 1 = enable
 * [7:5] - Reserved.
 */
#define KX134_CNTL5 0x1F

/**
 * CNTL6
 *
 * Bits:
 * [1:0] - I2C_ALC. 0.5s * 2^(I2C_ALC)
 * [6:2] - Reserved.
 * [7] - I2C_ALE. 0 = disable, 1 = enable
 */
#define KX134_CNTL6 0x20

/**
 * ODCNTL
 *
 * Bits:
 * [3:0] - OSA. ODR = 0.78125Hz * 2^(OSA)
 * [4] - Reserved.
 * [5] - FSTUP. Fast start: 0 = disable, 1 = enable
 * [6] - LPRO. LP filter roll off: 0 = ODR/9, 1 = ODR/2
 * [7] - Reserved.
 */
#define KX134_ODCNTL 0x21

/**
 * INC1 - INT1 control
 *
 * Bits:
 * [0] - SPI3E. 3-wire SPI: 0 = disable, 1 = enable
 * [1] - STPOL. self-test polarity: 0 = pos, 1 = neg
 * [2] - Reserved.
 * [3] - IEL1. 0 = latched, 1 = pulsed
 * [4] - IEA1. 0 = active low, 1 = active high
 * [5] - IEN1. 0 = disable, 1 = enable
 * [7:6] - PW1. int pulse width (refer to datasheet)
 */
#define KX134_INC1 0x22

/**
 * INC4 - INT1 enables
 *
 * Bits:
 * [0] - TPI1. Tilt: 0 = disable, 1 = enable
 * [1] - WUFI1. Wake-up: 0 = disable, 1 = enable
 * [2] - TDT1. Tap/double tap: 0 = disable, 1 = enable
 * [3] - BTSI1. Back-to-sleep: 0 = disable, 1 = enable
 * [4] - DRDYI1. Data ready: 0 = disable, 1 = enable
 * [5] - WMI1. Watermark: 0 = disable, 1 = enable
 * [6] - BFI1. Buffer full: 0 = disable, 1 = enable
 * [7] - FFI1. Free fall: 0 = disable, 1 = enable
 */
#define KX134_INC4 0x25

/**
 * INC5 - INT2 control
 *
 * Bits:
 * [0] - ACLR1. auto-clear 1: 0 = disable, 1 = enable
 * [1] - ACLR2. auto-clear 2: 0 = disable, 1 = enable
 * [2] - Reserved.
 * [3] - IEL2. 0 = latched, 1 = pulsed
 * [4] - IEA2. 0 = active low, 1 = active high
 * [5] - IEN2. 0 = disable, 1 = enable
 * [7:6] - PW2. int pulse width (refer to datasheet)
 */
#define KX134_INC5 0x26

/**
 * INC6 - INT2 enables
 *
 * Bits:
 * [0] - TPI2. Tilt: 0 = disable, 1 = enable
 * [1] - WUFI2. Wake-up: 0 = disable, 1 = enable
 * [2] - TDT2. Tap/double tap: 0 = disable, 1 = enable
 * [3] - BTSI2. Back-to-sleep: 0 = disable, 1 = enable
 * [4] - DRDYI2. Data ready: 0 = disable, 1 = enable
 * [5] - WMI2. Watermark: 0 = disable, 1 = enable
 * [6] - BFI2. Buffer full: 0 = disable, 1 = enable
 * [7] - FFI2. Free fall: 0 = disable, 1 = enable
 */
#define KX134_INC6 0x27

// KX134 Ranges
typedef enum {
    KX134_OUT_RATE_0_78125_HZ = 0x0,
    KX134_OUT_RATE_1_5625_HZ = 0x1,
    KX134_OUT_RATE_3_125_HZ = 0x2,
    KX134_OUT_RATE_6_25_HZ = 0x3,
    KX134_OUT_RATE_12_5_HZ = 0x4,
    KX134_OUT_RATE_25_HZ = 0x5,
    KX134_OUT_RATE_50_HZ = 0x6,
    KX134_OUT_RATE_100_HZ = 0x7,
    KX134_OUT_RATE_200_HZ = 0x8,
    KX134_OUT_RATE_400_HZ = 0x9,
    KX134_OUT_RATE_800_HZ = 0xA,
    KX134_OUT_RATE_1600_HZ = 0xB,
    KX134_OUT_RATE_3200_HZ = 0xC,
    KX134_OUT_RATE_6400_HZ = 0xD,
    KX134_OUT_RATE_12800_HZ = 0xE,
    KX134_OUT_RATE_25600_HZ = 0xF,
} Kx134OutputDataRate;

// KX134 Ranges
typedef enum {
    KX134_RANGE_8_G = 0x0 << 3,
    KX134_RANGE_16_G = 0x1 << 3,
    KX134_RANGE_32_G = 0x2 << 3,
    KX134_RANGE_64_G = 0x3 << 3,
} Kx134Range;

// Initalize the sensor
Status kx134_init(SpiDevice* device, Kx134OutputDataRate rate,
                  Kx134Range range);

// Read the acceleration registers
Accel kx134_read_accel(SpiDevice* device);

// Set the accelerometer range and measurement rate
Status kx134_config(SpiDevice* device, Kx134OutputDataRate rate,
                    Kx134Range range);

#endif  // KX134_H