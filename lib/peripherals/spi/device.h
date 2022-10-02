#ifndef DEVICE_H
#define DEVICE_H

#include <stdint.h>

enum SpiPeriph {
    SPI0,
    SPI1,
    SPI2,
    SPI3,
};

typedef struct {
    uint32_t clk; // Clock target (in Hz)
    bool cpol;
    bool cpha;
    uint8_t cs;
    SpiPeriph periph;
} SpiDevice;

#endif // DEVICE_H
