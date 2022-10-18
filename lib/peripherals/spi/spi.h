#ifndef SPI_H
#define SPI_H

// Standard library
#include <stddef.h>
#include <stdint.h>

// PSPHAA library
#include <status.h>

// Local headers
#include "device.h"

Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                    uint8_t len);

#endif // SPI_H
