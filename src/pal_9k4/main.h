#ifndef MAIN_H
#define MAIN_H

#include "board.h"
#include "flight_estimation.h"

#define DEBUG
// #define DEBUG_STORAGE
// #define PSPCOM_SENSORS

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
#define SERIAL_BUFFER_SIZE 2048
#define PIN_USB_MODE PIN_PB7

// Storage
#define NAND_ALLOW_REFORMAT
#define NAND_MAX_FLIGHTS 10

#endif  // MAIN_H