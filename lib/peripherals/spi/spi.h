#ifndef SPI_H
#define SPI_H

// Standard library
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// PSPHAA library
#include <status.h>

typedef enum {
    P_SPI0 = 0,
    P_SPI1 = 1,
    P_SPI2 = 2,
    P_SPI3 = 3,
} SpiPeriph;

typedef struct {
    uint32_t clk;  // Clock target (in Hz)
    bool cpol;
    bool cpha;
    uint8_t cs;
    SpiPeriph periph;
} SpiDevice;

Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                    uint8_t len);

#endif // SPI_H
