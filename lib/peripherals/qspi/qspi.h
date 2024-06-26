#ifndef QSPI_H
#define QSPI_H

#include <stdbool.h>

#include "status.h"
#include "stm32g4xx_hal.h"

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
    QSpiBank bank;
} QSpiDevice;

Status qspi_auto_poll_cmd(QSpiDevice* dev, QSPI_CommandTypeDef* cmd,
                          QSPI_AutoPollingTypeDef* cfg);
Status qspi_cmd(QSpiDevice* dev, QSPI_CommandTypeDef* cmd);
Status qspi_write(QSpiDevice* dev, QSPI_CommandTypeDef* cmd, uint8_t* tx_buf);
Status qspi_read(QSpiDevice* dev, QSPI_CommandTypeDef* cmd, uint8_t* rx_buf);

#endif