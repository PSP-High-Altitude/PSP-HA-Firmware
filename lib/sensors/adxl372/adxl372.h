#ifndef ADXL372_H
#define ADXL372_H

#include <stdint.h>

#include "data.h"
#include "spi/spi.h"
#include "status.h"

/*
 * Registery Definitions (below):
 */

/*
 * Acronym key:
 * NAC - not a concern. This is used for an address which is not
 * important for our purposes.
 */

/**
 * This registery holds the device ID. Used to check a connection
 * to the device as this address should always contain the same
 * value: 0xFA
 *
 * Bits:
 * [7:0] - Device Id = 0xFA
 *
 * All the bits in this registery have read only access.
 */
#define ADXL372_PARTID 0x02
/**
 * This c is uesd to determine various conditions of the
 * ADXL 372 sensor.
 *
 * Bits:
 * [0] - DATA_RDY Marks wheather the data is ready to be read or
 * if the data is still being written to the registeries. (Status
 * is high after all data has completed)
 * [3:1] (NAC) - FIFO stuff
 * [4] (NAC) - (Reserved)
 * [5] (NAC) - nonvolatile memory (or NVM) is busy
 * [6] (NAC?) - Awake stauts
 * [7] (NAC) - SEU event
 *
 * All bits for this registery have read only access.
 */
#define ADXL372_STATUS 0x04
/**
 * The ADXL 372 is set up so the MSB are set up in the first
 * registery and the LSB are in the second (only in bits [3:0])
 *
 * The registery for X, Y, and Z have back to back adresses so
 * the registery for YDATA_H is directly after the registery for
 * XDATA_L where YDATA_H is the MSB registery for Y and XDATA_L
 * is the LSB registery for X.
 *
 * All of these registeries are read only.
 */
#define ADXL372_DATA 0x08  // Address of first X accel data
/**
 * High Pass filter settings. Not sure if this is needed.
 */
#define ADXL372_HPF 0x38
/**
 * This registery controls the FIFO operations. Use this reg to
 * disable FIFO by setting the bits [2:1] to 0.
 */
#define ADXL372_FIFO_CTL 0x3A
/**
 * Controls the timing fucntions of the ADXL 372.
 *
 * Bits:
 * [0] and [1] (NAC) - Used to enable external trigger and clock
 * respectivly. Leave these as thier default value of 0 to ensure
 * everything remains internal.
 * [4:2] (NAC) - The wake up rates for Wake-Up mode. The pre-set
 * values for the wake up rates are def in the emun
 * Adxl372TimerWakeupRate
 * [7:5] - The output data rates (our main use for this address).
 * Possible rates are defined in the Adxl372OutputDataRate emun.
 *
 * All bits for this registery have read/write access.
 */
#define ADXL372_TIMING 0x3D
/**
 * This registery controls how the measurements are taken.
 *
 * Bits:
 * [2:0] - Set Bandwidth. See Preset values in Adxl372Bandwidths
 * [3] - Use this to set normal operations or to operate with low
 * noise (0 is Normal operations/ 1 is Low noise operations).
 * [5:4] (NAC) - Link/Loop processing (wake-up mode)
 * [6] (NAC) - Autosleep (wake-up mode)
 * [7] (NAC) - Disable Overange
 *
 * All bits for this registery have read/write access.
 */
#define ADXL372_MEASURE 0x3E
/**
 * Controls the power and mode of the ADXL 372.
 *
 * Bits:
 * [1:0] - Mode of operation. Preset values are founs in
 * Adxl372Modes.
 * [2] and [3] - Disables the high-pass and low-pass filter
 * respectivly.
 * [4] (NAC) - Filter Settleing period
 * [5] (NAC) - Instant on threshhold (0 is low / 1 is high)
 * [6] (NAC) - (Reserved - Read only)
 * [7] (NAC) - I2C speed select (1 = high speed mode).
 *
 * All non-reserved bits have read/write access.
 */
#define ADXL372_POWER_CTL 0x3F
/**
 * This registery resets the device when 0x52 is writen here.
 */
#define ADXL372_RESET 0x41

// Emuns for preset values
typedef enum {
    ADXL372_OUT_RATE_400_HZ = 0x00,
    ADXL372_OUT_RATE_800_HZ = 0x20,
    ADXL372_OUT_RATE_1_600_HZ = 0x40,
    ADXL372_OUT_RATE_3_200_HZ = 0x60,
    ADXL372_OUT_RATE_6_400_HZ = 0x80,
} Adxl372OutputDataRate;

typedef enum {
    ADXL372_TIMER_RATE_52_MS = 0x00,
    ADXL372_TIMER_RATE_104_MS = 0x04,
    ADXL372_TIMER_RATE_208_MS = 0x08,
    ADXL372_TIMER_RATE_512_MS = 0x0C,
    ADXL372_TIMER_RATE_2_048_MS = 0x10,
    ADXL372_TIMER_RATE_4_096_MS = 0x14,
    ADXL372_TIMER_RATE_8_192_MS = 0x18,
    ADXL372_TIMER_RATE_24_576_MS = 0x1C,
} Adxl372TimerWakeupRate;

typedef enum {
    ADXL372_MEASURE_MODE = 0x03,
    ADXL372_INSTANT_ON = 0x02,
    ADXL372_WAKE_UP = 0x01,
    ADXL372_STANDBY = 0x00,
} Adxl372Modes;

typedef enum {
    ADXL372_200_HZ = (1 << 3) | 0x00,    // 000
    ADXL372_400_HZ = (1 << 3) | 0x01,    // 001
    ADXL372_800_HZ = (1 << 3) | 0x02,    // 010
    ADXL372_1_600_HZ = (1 << 3) | 0x03,  // 011
    ADXL372_3_200_HZ = (1 << 3) | 0x04,  // 100
} Adxl372Bandwidths;

// Initalize the sensor
Status adxl372_init(SpiDevice* device, Adxl372Bandwidths bandwidth,
                    Adxl372OutputDataRate rate, Adxl372Modes mode);

// Read the acceleration registers
Accel adxl372_read_accel(SpiDevice* device);

// Set the accelerometer range and measurement rate
Status adxl372_config(SpiDevice* device, Adxl372OutputDataRate rate);

#endif  // ADXL372_H