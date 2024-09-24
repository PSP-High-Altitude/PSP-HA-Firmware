#include "spi/spi.h"

#include "gpio/gpio.h"
#include "pal_9k5/board.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

#define SPI_PIN_AF_COUNT 10

enum {
    MOSI = 1,
    MISO = 2,
    SCK = 3,
};

typedef struct {
    uint8_t pin;
    uint8_t function;  // 0 - Invalid, 1 - MOSI, 2 - MISO, 3 - SCK
    uint8_t af;
} SpiPinAf;

const SpiPinAf spi_pin_af[6][SPI_PIN_AF_COUNT] = {
    {
        // SPI1
        {PIN_PA5, SCK, GPIO_AF5_SPI1},
        {PIN_PA6, MISO, GPIO_AF5_SPI1},
        {PIN_PA7, MOSI, GPIO_AF5_SPI1},
        {PIN_PB3, SCK, GPIO_AF5_SPI1},
        {PIN_PB4, MISO, GPIO_AF5_SPI1},
        {PIN_PB5, MOSI, GPIO_AF5_SPI1},
        {PIN_PD7, MOSI, GPIO_AF5_SPI1},
        {PIN_PG9, MISO, GPIO_AF5_SPI1},
        {PIN_PG11, SCK, GPIO_AF5_SPI1},
        {0},
    },
    {
        // SPI2
        {PIN_PA9, SCK, GPIO_AF5_SPI2},
        {PIN_PA12, SCK, GPIO_AF5_SPI2},
        {PIN_PB10, SCK, GPIO_AF5_SPI2},
        {PIN_PB13, SCK, GPIO_AF5_SPI2},
        {PIN_PB14, MISO, GPIO_AF5_SPI2},
        {PIN_PB15, MOSI, GPIO_AF5_SPI2},
        {PIN_PC1, MOSI, GPIO_AF5_SPI2},
        {PIN_PC2, MISO, GPIO_AF5_SPI2},
        {PIN_PC3, MOSI, GPIO_AF5_SPI2},
        {PIN_PD3, SCK, GPIO_AF5_SPI2},
    },
    {
        // SPI3
        {PIN_PB2, MOSI, GPIO_AF7_SPI3},
        {PIN_PB3, SCK, GPIO_AF6_SPI3},
        {PIN_PB4, MISO, GPIO_AF6_SPI3},
        {PIN_PB5, MOSI, GPIO_AF7_SPI3},
        {PIN_PC10, SCK, GPIO_AF6_SPI3},
        {PIN_PC11, MISO, GPIO_AF6_SPI3},
        {PIN_PC12, MOSI, GPIO_AF6_SPI3},
        {PIN_PD6, MOSI, GPIO_AF5_SPI3},
        {0},
        {0},
    },
    {
        // SPI4
        {PIN_PE2, SCK, GPIO_AF5_SPI4},
        {PIN_PE5, MISO, GPIO_AF5_SPI4},
        {PIN_PE6, MOSI, GPIO_AF5_SPI4},
        {PIN_PE12, SCK, GPIO_AF5_SPI4},
        {PIN_PE13, MISO, GPIO_AF5_SPI4},
        {PIN_PE14, MOSI, GPIO_AF5_SPI4},
        {0},
        {0},
        {0},
        {0},
    },
    {
        // SPI5
        {PIN_PF7, SCK, GPIO_AF5_SPI5},
        {PIN_PF8, MISO, GPIO_AF5_SPI5},
        {PIN_PF9, MOSI, GPIO_AF5_SPI5},
        {PIN_PF11, MOSI, GPIO_AF5_SPI5},
        {PIN_PH6, SCK, GPIO_AF5_SPI5},
        {PIN_PH7, MISO, GPIO_AF5_SPI5},
        {PIN_PJ10, MOSI, GPIO_AF5_SPI5},
        {PIN_PJ11, MISO, GPIO_AF5_SPI5},
        {PIN_PK0, SCK, GPIO_AF5_SPI5},
        {0},
    },
    {
        // SPI6
        {PIN_PA5, SCK, GPIO_AF8_SPI6},
        {PIN_PA6, MISO, GPIO_AF8_SPI6},
        {PIN_PA7, MOSI, GPIO_AF8_SPI6},
        {PIN_PB3, SCK, GPIO_AF8_SPI6},
        {PIN_PB4, MISO, GPIO_AF8_SPI6},
        {PIN_PB5, MOSI, GPIO_AF8_SPI6},
        {PIN_PC12, SCK, GPIO_AF5_SPI6},
        {PIN_PG12, MISO, GPIO_AF5_SPI6},
        {PIN_PG13, SCK, GPIO_AF5_SPI6},
        {PIN_PG14, MOSI, GPIO_AF5_SPI6},
    },
};

static SPI_TypeDef* spi_base[6] = {SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};
static SPI_HandleTypeDef spi1_handle = {.State = 0};
static SPI_HandleTypeDef spi2_handle = {.State = 0};
static SPI_HandleTypeDef spi3_handle = {.State = 0};
static SPI_HandleTypeDef spi4_handle = {.State = 0};
static SPI_HandleTypeDef spi5_handle = {.State = 0};
static SPI_HandleTypeDef spi6_handle = {.State = 0};
static SPI_HandleTypeDef* spi_handles[] = {&spi1_handle, &spi2_handle,
                                           &spi3_handle, &spi4_handle,
                                           &spi5_handle, &spi6_handle};

static Status get_pin(uint8_t periph, uint8_t pin, uint8_t function,
                      uint32_t* af) {
    for (int i = 0; i < SPI_PIN_AF_COUNT; i++) {
        // Invalid function means we ran out of pins
        if (spi_pin_af[periph][i].function == 0) {
            return STATUS_ERROR;
        }

        // Check if the pin is a match
        if (spi_pin_af[periph][i].pin == pin &&
            spi_pin_af[periph][i].function == function) {
            *af = spi_pin_af[periph][i].af;
            return STATUS_OK;
        }
    }

    return STATUS_ERROR;
}

Status spi_setup(SpiDevice* dev) {
    // Check if the peripheral is valid
    if (dev->periph < P_SPI1 || dev->periph > P_SPI6) {
        return STATUS_PARAMETER_ERROR;
    }

    // Check if the peripheral is already initialized
    if (spi_handles[dev->periph]->State != 0) {
        return STATUS_OK;
    }

    // Enable clocks
    switch (dev->periph) {
        case P_SPI1:
            __HAL_RCC_SPI1_CLK_ENABLE();
            break;
        case P_SPI2:
            __HAL_RCC_SPI2_CLK_ENABLE();
            break;
        case P_SPI3:
            __HAL_RCC_SPI3_CLK_ENABLE();
            break;
        case P_SPI4:
            __HAL_RCC_SPI4_CLK_ENABLE();
            break;
        case P_SPI5:
            __HAL_RCC_SPI5_CLK_ENABLE();
            break;
        case P_SPI6:
            __HAL_RCC_SPI6_CLK_ENABLE();
            break;
    }

    // Get the base address of the peripheral
    SPI_TypeDef* base = spi_base[dev->periph];

    // Create the pin configuration
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };

    // Get and configure the MOSI pin
    if (get_pin(dev->periph, dev->mosi, MOSI, &pin_conf.Alternate) !=
        STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    pin_conf.Pin = PAL_GPIO_PIN(dev->mosi);
    HAL_GPIO_Init(PAL_GPIO_PORT(dev->mosi), &pin_conf);

    // Get and configure the MISO pin
    if (get_pin(dev->periph, dev->miso, MISO, &pin_conf.Alternate) !=
        STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    pin_conf.Pin = PAL_GPIO_PIN(dev->miso);
    HAL_GPIO_Init(PAL_GPIO_PORT(dev->miso), &pin_conf);

    // Get and configure the SCK pin
    if (get_pin(dev->periph, dev->sck, SCK, &pin_conf.Alternate) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    pin_conf.Pin = PAL_GPIO_PIN(dev->sck);
    HAL_GPIO_Init(PAL_GPIO_PORT(dev->sck), &pin_conf);

    // Get and configure the CS pin
    if (dev->cs < 0 || dev->cs > PIN_MAX) {
        return STATUS_PARAMETER_ERROR;
    }
    gpio_write(dev->cs, GPIO_HIGH);

    uint32_t prescale = 0;
    switch (dev->clk) {
        case SPI_SPEED_100kHz:
            prescale = SPI_BAUDRATEPRESCALER_256;
            break;
        case SPI_SPEED_500kHz:
            prescale = SPI_BAUDRATEPRESCALER_128;
            break;
        case SPI_SPEED_1MHz:
            prescale = SPI_BAUDRATEPRESCALER_64;
            break;
        case SPI_SPEED_10MHz:
            prescale = SPI_BAUDRATEPRESCALER_4;
            break;
        case SPI_SPEED_20MHz:
            prescale = SPI_BAUDRATEPRESCALER_2;
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

Status spi_set_cs(SpiDevice* dev, GpioValue val) {
    return gpio_write(dev->cs, val);
}

Status spi_exchange_nosetup(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                            uint16_t len) {
    if (HAL_SPI_TransmitReceive(spi_handles[dev->periph], tx_buf, rx_buf, len,
                                100) != HAL_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    return STATUS_OK;
}

Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                    uint16_t len) {
    if (spi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    spi_set_cs(dev, GPIO_LOW);
    if (spi_exchange_nosetup(dev, tx_buf, rx_buf, len) != STATUS_OK) {
        spi_set_cs(dev, GPIO_HIGH);
        return STATUS_HARDWARE_ERROR;
    }
    spi_set_cs(dev, GPIO_HIGH);
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
