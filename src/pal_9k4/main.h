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

#endif  // MAIN_H