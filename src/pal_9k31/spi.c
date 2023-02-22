#include "spi/spi.h"

#include "board.h"
#include "stm32g4xx_hal.h"

static SPI_HandleTypeDef* spi_handles[] = {NULL, NULL, NULL, NULL};

static Status spi_setup(SpiDevice* dev) {
    if (spi_handles[dev->periph] != NULL) {
        return STATUS_OK;
    }
    SPI_TypeDef* base = NULL;
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    switch (dev->periph) {
        case P_SPI1:
            base = SPI1;
            pin_conf.Alternate = GPIO_AF5_SPI1;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PA4];
            HAL_GPIO_Init(GPIOA, &pin_conf);  // NSS: pin PA4
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PA5];
            HAL_GPIO_Init(GPIOA, &pin_conf);  // SCK: pin PA5
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PA6];
            HAL_GPIO_Init(GPIOA, &pin_conf);  // MISO: pin PA6
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PA7];
            HAL_GPIO_Init(GPIOA, &pin_conf);  // MOSI: pin PA7
        case P_SPI2:
            base = SPI2;
            pin_conf.Alternate = GPIO_AF5_SPI2;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB12];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // NSS: pin PB12
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB13];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // SCK: pin PB13
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB14];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // MISO: pin PB14
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PB15];
            HAL_GPIO_Init(GPIOB, &pin_conf);  // MOSI: pin PB15
            break;
        case P_SPI3:
            base = SPI1;
            return STATUS_PARAMETER_ERROR;
            break;
        case P_SPI4:
            base = SPI4;
            pin_conf.Alternate = GPIO_AF5_SPI4;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE4];
            HAL_GPIO_Init(GPIOE, &pin_conf);  // NSS: pin PE4
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE2];
            HAL_GPIO_Init(GPIOE, &pin_conf);  // SCK: pin PE2
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE5];
            HAL_GPIO_Init(GPIOE, &pin_conf);  // MISO: pin PE5
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE6];
            HAL_GPIO_Init(GPIOE, &pin_conf);  // MOSI: pin PE6
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }
    uint32_t prescale = 0;
    switch (dev->clk) {
        case SPI_SPEED_100kHz:
            prescale = SPI_BAUDRATEPRESCALER_256;
            break;
        case SPI_SPEED_500kHz:
            prescale = SPI_BAUDRATEPRESCALER_64;
            break;
        case SPI_SPEED_1MHz:
            prescale = SPI_BAUDRATEPRESCALER_32;
            break;
        case SPI_SPEED_10MHz:
            prescale = SPI_BAUDRATEPRESCALER_4;
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }
    SPI_InitTypeDef init_conf = {
        .BaudRatePrescaler = prescale,
        .CLKPhase = dev->cpha ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE,
        .CLKPolarity = dev->cpol ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW,
        .NSS = SPI_NSS_HARD_OUTPUT,
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .DataSize = SPI_DATASIZE_8BIT,
    };
    SPI_HandleTypeDef handle = {
        .Init = init_conf,
        .Instance = base,
    };
    HAL_SPI_Init(&handle);
    spi_handles[dev->periph] = &handle;
    return STATUS_OK;
}

Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                    uint8_t len) {
    if (spi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_SPI_TransmitReceive(spi_handles[dev->periph], tx_buf, rx_buf, len,
                                100) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}