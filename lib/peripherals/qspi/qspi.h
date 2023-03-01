#ifndef QSPI_H
#define QSPI_H

#include "status.h"

typedef enum {
    P_QSPI1 = 1,
} QSpiPeriph;

typedef enum {
    P_QSPI_BK1 = 1,
    P_QSPI_BK2 = 2,
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
    QSpiPeriph periph;
    QSpiBank bank;
} QSpiDevice;

#endif