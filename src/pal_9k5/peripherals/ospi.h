#ifndef OSPI_H
#define OSPI_H

#include <stdbool.h>

#include "status.h"
#include "stm32h7xx_hal.h"

typedef enum {
    P_OSPI1 = 0,
    P_OSPI2 = 1,
} OSpiPeriph;

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
    OSpiPeriph periph;
    OSpiSpeed clk;
    uint8_t sck;
    uint8_t ncs;

    // IO pins can be the actual [3:0] pins or the [7:4] pins
    uint8_t io0;
    uint8_t io1;
    uint8_t io2;
    uint8_t io3;

    // 2^(device_size + 1) bytes
    uint8_t device_size;
} OSpiDevice;

typedef struct {
    uint8_t inst;
    uint8_t inst_mode;
    uint8_t addr_mode;
    uint32_t n_addr;
    uint32_t addr;
    uint8_t n_dummy;
    uint8_t data_mode;
    uint32_t n_data;
} OSpiCommand;

typedef enum {
    OSPI_MATCH_AND,
    OSPI_MATCH_OR,
} OSpiMatchMode;

Status ospi_auto_poll_cmd(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd,
                          OSPI_AutoPollingTypeDef* cfg, uint64_t timeout);
Status ospi_cmd(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd);
Status ospi_write(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd, uint8_t* tx_buf,
                  uint64_t timeout);
Status ospi_read(OSpiDevice* dev, OSPI_RegularCmdTypeDef* cmd, uint8_t* rx_buf,
                 uint64_t timeout);

#endif