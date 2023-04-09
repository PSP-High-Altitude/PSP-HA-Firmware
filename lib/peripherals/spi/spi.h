#ifndef SPI_H
#define SPI_H

// Standard library
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// PSPHAA library
#include "gpio/gpio.h"
#include "status.h"

typedef enum {
    P_SPI1 = 0,
    P_SPI2 = 1,
    P_SPI3 = 2,
    P_SPI4 = 3,
} SpiPeriph;

typedef enum {
    SPI_SPEED_INVALID = 0,
    SPI_SPEED_100kHz = 100000,
    SPI_SPEED_500kHz = 500000,
    SPI_SPEED_1MHz = 1000000,
    SPI_SPEED_10MHz = 10000000,
    SPI_SPEED_20MHz = 20000000,
} SpiSpeed;

typedef struct {
    SpiSpeed clk;
    bool cpol;
    bool cpha;
    uint8_t cs;
    SpiPeriph periph;
} SpiDevice;

Status spi_setup(SpiDevice* dev);

Status spi_set_cs(SpiDevice* dev, GpioValue val);

Status spi_exchange_nosetup(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                            uint16_t len);

Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                    uint16_t len);

#endif // SPI_H
