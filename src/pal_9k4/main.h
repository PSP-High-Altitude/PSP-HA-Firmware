#ifndef MAIN_H
#define MAIN_H

#include "board.h"
#include "flight_estimation.h"

#define DEBUG
// #define DEBUG_STORAGE
// #define PSPCOM_SENSORS

// Sensor read interval (ms)
#define TARGET_INTERVAL 5  // 200 Hz
#define AVG_BUFFER_SIZE \
    (AVERAGING_PERIOD_MS / TARGET_INTERVAL)  // For state estimation

// Pyro fire length (ms)
#define PYRO_FIRE_LENGTH 1000

// Pyro retries (if we make it to the ground and the pyro didn't fire someone
// approaching could get injured)
#define PYRO_MAX_RETRIES 20
#define PYRO_RETRY_INTERVAL 1000

// Sensor FIFO length
#define LOG_FIFO_LEN 256

// Misc
#ifdef USE_SPI_CRC
#undef USE_SPI_CRC
#endif
#define USE_SPI_CRC 0

#endif  // MAIN_H