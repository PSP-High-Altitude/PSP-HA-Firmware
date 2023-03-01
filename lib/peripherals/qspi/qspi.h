#ifndef QSPI_H
#define QSPI_H

#include <stdbool.h>

#include "status.h"

typedef enum {
    QSPI_BK1 = 0,
    QSPI_BK2 = 1,
} QSpiBank;

typedef enum {
    QSPI_SPEED_INVALID = 0,
    QSPI_SPEED_1MHz = 1000000,
    QSPI_SPEED_2MHz = 2000000,
    QSPI_SPEED_10MHz = 10000000,
    QSPI_SPEED_20MHz = 20000000,
} QSpiSpeed;

typedef struct {
    QSpiSpeed clk;
    bool cpol;
    bool cpha;
    QSpiBank bank;
} QSpiDevice;

#endif