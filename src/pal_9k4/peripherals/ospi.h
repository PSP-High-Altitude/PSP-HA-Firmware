#ifndef OSPI_H
#define OSPI_H

#include <stdbool.h>

#include "status.h"
#include "stm32h7xx_hal.h"

typedef enum {
    OSPI_PORT1_3_0 = 0,
    OSPI_PORT1_7_4 = 1,
} OSpiBank;

typedef enum {
    OSPI_SPEED_INVALID = 0,
    OSPI_SPEED_1MHz = 1000000,
    OSPI_SPEED_5MHz = 5000000,
    OSPI_SPEED_10MHz = 10000000,
    OSPI_SPEED_20MHz = 20000000,
    OSPI_SPEED_40MHz = 40000000,
    OSPI_SPEED_80MHz = 80000000,
} OSpiSpeed;

typedef struct {
    OSpiSpeed clk;
    OSpiBank bank;
} OSpiDevice;

Status ospi_auto_poll_cmd(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                          OSPI_AutoPollingTypeDef* cfg);
Status ospi_cmd(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd);
Status ospi_write(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                  uint8_t* tx_buf);
Status ospi_read(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd, uint8_t* rx_buf);

#endif