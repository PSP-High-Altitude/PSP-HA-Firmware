#include "spi/spi.h"

#include <math.h>

#include "imxrt1062/MIMXRT1062/drivers/fsl_clock.h"
#include "imxrt1062/MIMXRT1062/drivers/fsl_iomuxc.h"
#include "imxrt1062/MIMXRT1062/drivers/fsl_lpspi.h"

static bool spi_enabled[] = {0, 0, 0, 0};

static Status spi_setup(SpiDevice* dev) {
    if (spi_enabled[dev->periph]) {
        return OK;
    }
    LPSPI_Type* base = NULL;
    switch (dev->periph) {
        case SPI0:
            base = LPSPI1;
            break;
        case SPI1:
            base = LPSPI2;
            break;
        case SPI2:
            base = LPSPI3;
            break;
        case SPI3:
            base = LPSPI4;
            break;
    }
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_00_LPSPI4_PCS0, 1);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_01_LPSPI4_SDI, 1);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_02_LPSPI4_SDO, 1);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_03_LPSPI4_SCK, 1);
    lpspi_master_config_t conf;
    LPSPI_MasterGetDefaultConfig(&conf);
    conf.baudRate = dev->clk;
    conf.cpha = dev->cpha;
    conf.cpol = dev->cpol;
    conf.whichPcs = dev->cs;
    LPSPI_MasterInit(base, &conf, CLOCK_GetClockRootFreq(kCLOCK_LpspiClkRoot));
    spi_enabled[dev->periph] = 1;
    return OK;
}

Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                    uint8_t len) {
    spi_setup(dev);
    LPSPI_Type* base = NULL;
    switch (dev->periph) {
        case SPI0:
            base = LPSPI1;
            break;
        case SPI1:
            base = LPSPI2;
            break;
        case SPI2:
            base = LPSPI3;
            break;
        case SPI3:
            base = LPSPI4;
            break;
    }
    lpspi_transfer_t transfer = {
        .txData = tx_buf,
        .rxData = rx_buf,
        .dataSize = len,
    };
    if (LPSPI_MasterTransferBlocking(base, &transfer) != kStatus_Success) {
        return ERROR;
    }
    return OK;
}