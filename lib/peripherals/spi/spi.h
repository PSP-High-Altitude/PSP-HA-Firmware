#ifndef SPI_H
#define SPI_H

// Standard library
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// PSPHAA library
#include <status.h>

typedef enum {
    P_SPI1 = 1,
    P_SPI2 = 2,
    P_SPI3 = 3,
    P_SPI4 = 4,
} SpiPeriph;

typedef enum {
    SPI_SPEED_INVALID = 0,
    SPI_SPEED_100kHz = 100000,
    SPI_SPEED_500kHz = 500000,
    SPI_SPEED_1MHz = 1000000,
    SPI_SPEED_10MHz = 10000000,
} SpiSpeed;

typedef struct {
    SpiSpeed clk;
    bool cpol;
    bool cpha;
    uint8_t cs;
    SpiPeriph periph;
} SpiDevice;

Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                    uint8_t len);

#endif // SPI_H
