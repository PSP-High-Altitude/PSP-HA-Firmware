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
            switch (dev->cs) {
                case 0:
                    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_01_LPSPI1_PCS0,
                                     1);  // pin 44
                    break;
                default:
                    return PARAMETER_ERROR;
            }
            IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_LPSPI1_SDI, 1);  // pin 42
            IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_02_LPSPI1_SDO, 1);  // pin 43
            IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_00_LPSPI1_SCK, 1);  // pin 45
            break;
        case SPI1:
            return PARAMETER_ERROR;
            break;
        case SPI2:
            base = LPSPI3;
            switch (dev->cs) {
                case 0:
                    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_LPSPI3_PCS0,
                                     1);  // pin 0
                    break;
                default:
                    return PARAMETER_ERROR;
            }
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_LPSPI3_SDI, 1);  // pin 1
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_LPSPI3_SDO, 1);  // pin 26
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_LPSPI3_SCK, 1);  // pin 27
            break;
        case SPI3:
            base = LPSPI4;
            switch (dev->cs) {
                case 0:
                    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_00_LPSPI4_PCS0,
                                     1);  // pin 10
                    break;
                case 1:
                    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_03_LPSPI4_PCS1,
                                     1);  // pin 37
                    break;
                case 2:
                    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_02_LPSPI4_PCS2,
                                     1);  // pin 36
                    break;
                default:
                    return PARAMETER_ERROR;
            }
            IOMUXC_SetPinMux(IOMUXC_GPIO_B0_01_LPSPI4_SDI, 1);  // pin 12
            IOMUXC_SetPinMux(IOMUXC_GPIO_B0_02_LPSPI4_SDO, 1);  // pin 11
            IOMUXC_SetPinMux(IOMUXC_GPIO_B0_03_LPSPI4_SCK, 1);  // pin 13
            break;
        default:
            return PARAMETER_ERROR;
    }
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
    if (spi_setup(dev) != OK) {
        return PARAMETER_ERROR;
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