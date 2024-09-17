#ifndef MAIN_H
#define MAIN_H

#include "board.h"

volatile int uxTopUsedPriority;

#define DEBUG
// #define DEBUG_STORAGE
// #define PSPCOM_SENSORS
// #define FLIGHT_MODE

// Misc
#ifdef USE_SPI_CRC
#undef USE_SPI_CRC
#endif
#define USE_SPI_CRC 0

// USB
#define SERIAL_BAUD_RATE 115200
#define SERIAL_STOP_BITS 0  // 0 - 1, 1 - 1.5, 2 - 2
#define SERIAL_PARITY 0     // 0 - None, 1 - Odd, 2 - Even, 3 - Mark, 4 - Space
#define SERIAL_DATA_BITS 8  // 5, 6, 7, 8 or 16
#define SERIAL_BUFFER_SIZE 4096
// #define PIN_USB_MODE PIN_PB7

// Storage
#define NAND_ALLOW_REFORMAT
#define NAND_MAX_FLIGHTS 10

// MTP
#define MTP_SEL_PIN PIN_PB5
#define MTP_SEL_DELAY_MS 5000

// Extra debugging
// #define DEBUG_MEMORY_USAGE

#endif  // MAIN_H