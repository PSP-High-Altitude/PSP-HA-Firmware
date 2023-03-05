#include "spi/spi.h"

#include "board.h"
#include "gpio/gpio.h"
#include "stdio.h"
#include "stm32g4xx_hal.h"
#include "timer.h"

static SPI_HandleTypeDef spi1_handle = {.State = 0};
static SPI_HandleTypeDef spi2_handle = {.State = 0};
static SPI_HandleTypeDef spi3_handle = {.State = 0};
static SPI_HandleTypeDef spi4_handle = {.State = 0};
static SPI_HandleTypeDef* spi_handles[] = {&spi1_handle, &spi2_handle,
                                           &spi3_handle, &spi4_handle};
static uint8_t cs_pin[4] = {PIN_PA4, PIN_PB12, 0, PIN_PE4};

static Status spi_setup(SpiDevice* dev) {
    if (dev->periph < 0 || dev->periph > 3) {
        return STATUS_PARAMETER_ERROR;
    }
    if (spi_handles[dev->periph]->State != 0) {
        return STATUS_OK;
    }
    SPI_TypeDef* base = NULL;
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    switch (dev->periph) {
        case P_SPI1:
            base = SPI1;
            pin_conf.Alternate = GPIO_AF5_SPI1;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PA5] | GPIO_PIN_TO_NUM[PIN_PA6] |
                           GPIO_PIN_TO_NUM[PIN_PA7];  // SCK: pin PA5, MISO: pin
                                                      // PA6, MOSI: pin PA7
            gpio_write(cs_pin[0], GPIO_HIGH);         // NSS: pin PA4
            HAL_GPIO_DeInit(GPIOA, pin_conf.Pin);
            HAL_GPIO_Init(GPIOA, &pin_conf);
            HAL_GPIO_LockPin(GPIOA, pin_conf.Pin);
            break;
        case P_SPI2:
            base = SPI2;
            pin_conf.Alternate = GPIO_AF5_SPI2;
            pin_conf.Pin =
                GPIO_PIN_TO_NUM[PIN_PB13] | GPIO_PIN_TO_NUM[PIN_PB14] |
                GPIO_PIN_TO_NUM[PIN_PB15];     // SCK: pin PB13, MISO: pin
                                               // PB14, MOSI: pin PB15
            gpio_write(cs_pin[1], GPIO_HIGH);  // NSS: pin PB12
            HAL_GPIO_DeInit(GPIOB, pin_conf.Pin);
            HAL_GPIO_Init(GPIOB, &pin_conf);
            HAL_GPIO_LockPin(GPIOB, pin_conf.Pin);
            break;
        case P_SPI3:
            base = SPI1;
            return STATUS_PARAMETER_ERROR;
            break;
        case P_SPI4:
            base = SPI4;
            pin_conf.Alternate = GPIO_AF5_SPI4;
            pin_conf.Pin = GPIO_PIN_TO_NUM[PIN_PE2] | GPIO_PIN_TO_NUM[PIN_PE5] |
                           GPIO_PIN_TO_NUM[PIN_PE6];  // SCK: pin PE2, MISO: pin
                                                      // PE5, MOSI: pin PE6
            gpio_write(cs_pin[3], GPIO_HIGH);         // NSS: pin PE4
            HAL_GPIO_DeInit(GPIOE, pin_conf.Pin);
            HAL_GPIO_Init(GPIOE, &pin_conf);
            HAL_GPIO_LockPin(GPIOE, pin_conf.Pin);
            break;
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
        .NSS = SPI_NSS_SOFT,
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .DataSize = SPI_DATASIZE_8BIT,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCLength = SPI_CRC_LENGTH_DATASIZE,
        .NSSPMode = SPI_NSS_PULSE_DISABLE,
    };
    SPI_HandleTypeDef* handle = spi_handles[dev->periph];
    (*handle).Init = init_conf;
    (*handle).Instance = base;
    if (HAL_SPI_Init(handle) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

uint8_t spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                     uint8_t len) {
    uint8_t status;
    if (spi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    gpio_write(cs_pin[dev->periph], GPIO_LOW);
    if ((status = HAL_SPI_TransmitReceive(spi_handles[dev->periph], tx_buf,
                                          rx_buf, len, 100)) != HAL_OK) {
        gpio_write(cs_pin[dev->periph], GPIO_HIGH);
        return status;
    }
    gpio_write(cs_pin[dev->periph], GPIO_HIGH);
    return STATUS_OK;
}

/*
void SPI1_IRQHandler(void) { HAL_SPI_IRQHandler(&spi1_handle); }

void SPI2_IRQHandler(void) { HAL_SPI_IRQHandler(&spi2_handle); }

void SPI4_IRQHandler(void) { HAL_SPI_IRQHandler(&spi4_handle); }

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI1) {
        gpio_write(cs_pin[0], GPIO_HIGH);
    } else if (hspi->Instance == SPI2) {
        gpio_write(cs_pin[2], GPIO_HIGH);
    } else if (hspi->Instance == SPI4) {
        gpio_write(cs_pin[3], GPIO_HIGH);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI1) {
        gpio_write(cs_pin[0], GPIO_HIGH);
    } else if (hspi->Instance == SPI2) {
        gpio_write(cs_pin[2], GPIO_HIGH);
    } else if (hspi->Instance == SPI4) {
        gpio_write(cs_pin[3], GPIO_HIGH);
    }
}
*/

SPI_HandleTypeDef* get_handle(SpiDevice* dev) {
    return spi_handles[dev->periph];
}