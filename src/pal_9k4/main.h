#ifndef MAIN_H
#define MAIN_H

#include "board.h"

#define DEBUG
#define DEBUG_STORAGE
// #define PSPCOM_SENSORS

// Pin configuration
#define PIN_FIREMAIN PIN_PG12
#define PIN_FIREDRG PIN_PG11
#define PIN_FIREAUX PIN_PG10
#define PIN_CONTMAIN PIN_PG9
#define PIN_CONTDRG PIN_PD7
#define PIN_CONTAUX PIN_PD6
#define PIN_RED PIN_PA0
#define PIN_YELLOW PIN_PA1
#define PIN_GREEN PIN_PA2
#define PIN_BLUE PIN_PA3
#define PIN_BUZZER PIN_PE0
#define PIN_PAUSE PIN_PB7  // SDA4

// Sensor read interval (ms)
#define TARGET_INTERVAL 5  // 200 Hz

// Sensor FIFO length
#define LOG_FIFO_LEN 256

// Misc
#ifdef USE_SPI_CRC
#undef USE_SPI_CRC
#endif
#define USE_SPI_CRC 0

#endif  // MAIN_H