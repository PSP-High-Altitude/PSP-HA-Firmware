#include "spi/spi.h"

#include <math.h>

#include "imxrt1062/registers.h"

#define BASE_CLK 1320000000
#define CLK_DIV 7

static SpiDevice g_spi_state[4] = {
    {
        .clk = 0,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = SPI0,
    },
    {
        .clk = 0,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = SPI1,
    },
    {
        .clk = 0,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = SPI2,
    },
    {
        .clk = 0,
        .cpol = 0,
        .cpha = 0,
        .cs = 0,
        .periph = SPI3,
    },
};

static LPSPI_t* g_spi_base[4] = {LPSPI1, LPSPI2, LPSPI3, LPSPI4};

typedef struct {
    int mosi;
    uint8_t mosi_mode;
    int miso;
    uint8_t miso_mode;
    int sck;
    uint8_t sck_mode;
} SpiPins;

static SpiPins g_spi_pins[4] = {
    {
        .mosi = 43,
        .mosi_mode = 4,
        .miso = 42,
        .miso_mode = 4,
        .sck = 45,
        .sck_mode = 4,
    },
    {
        .mosi = -1,
        .mosi_mode = 0,
        .miso = -1,
        .miso_mode = 0,
        .sck = -1,
        .sck_mode = 0,
    },
    {
        .mosi = 26,
        .mosi_mode = 2,
        .miso = 39,
        .miso_mode = 2,
        .sck = 27,
        .sck_mode = 2,
    },
    {
        .mosi = 11,
        .mosi_mode = 3,
        .miso = 12,
        .miso_mode = 3,
        .sck = 13,
        .sck_mode = 3,
    },
};

typedef struct {
    volatile uint32_t* MUX_REG_ADDR;
    uint8_t pcs_num;
    uint32_t alt_mode;
} LPSPI_PCS;

// PCS pin configurations

const LPSPI_PCS g_pcs_pins[4][5] = {
    {
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_30),
            .pcs_num = 0,
            .alt_mode = 3,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B0_01),
            .pcs_num = 0,
            .alt_mode = 4,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_31),
            .pcs_num = 1,
            .alt_mode = 3,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_40),
            .pcs_num = 2,
            .alt_mode = 2,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_41),
            .pcs_num = 3,
            .alt_mode = 2,
        },
    },
    {
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B1_06),
            .pcs_num = 0,
            .alt_mode = 4,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_01),
            .pcs_num = 0,
            .alt_mode = 2,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_EMC_14),
            .pcs_num = 1,
            .alt_mode = 4,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B1_10),
            .pcs_num = 2,
            .alt_mode = 4,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_SD_B1_11),
            .pcs_num = 3,
            .alt_mode = 4,
        },
    },
    {
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B1_12),
            .pcs_num = 0,
            .alt_mode = 2,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_03),
            .pcs_num = 0,
            .alt_mode = 7,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_04),
            .pcs_num = 1,
            .alt_mode = 7,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_05),
            .pcs_num = 2,
            .alt_mode = 7,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_AD_B0_06),
            .pcs_num = 3,
            .alt_mode = 7,
        },
    },
    {
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_04),
            .pcs_num = 0,
            .alt_mode = 1,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_B0_00),
            .pcs_num = 0,
            .alt_mode = 3,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_03),
            .pcs_num = 1,
            .alt_mode = 2,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_02),
            .pcs_num = 2,
            .alt_mode = 2,
        },
        {
            .MUX_REG_ADDR = &(IOMUXC->SW_MUX_CTL_PAD_GPIO_B1_11),
            .pcs_num = 3,
            .alt_mode = 6,
        },
    },
};

int8_t checkCS(SpiDevice* dev, uint8_t pinNum) {
    for (int i = 0; i < 5; i++) {
        if (g_pcs_pins[dev->periph][i].MUX_REG_ADDR ==
            PIN[pinNum].MUX_REG_ADDR) {
            *(g_pcs_pins[dev->periph][i].MUX_REG_ADDR) &= 0xFFFFFFF0;
            *(g_pcs_pins[dev->periph][i].MUX_REG_ADDR) |=
                g_pcs_pins[dev->periph][i].alt_mode;
            return g_pcs_pins[dev->periph][i].pcs_num;
        }
    }
    return -1;
}

static Status spi_setup(SpiDevice* dev) {
    LPSPI_t* spi_base = g_spi_base[dev->periph];

    CCM->CCGR1 |= 0xC0;  // Enable clock gate

    g_spi_pins[dev->periph].mosi &= 0xFFFFFFF0;
    g_spi_pins[dev->periph].mosi |= g_spi_pins[dev->periph].mosi_mode;
    g_spi_pins[dev->periph].miso &= 0xFFFFFFF0;
    g_spi_pins[dev->periph].miso |= g_spi_pins[dev->periph].miso_mode;
    g_spi_pins[dev->periph].sck &= 0xFFFFFFF0;
    g_spi_pins[dev->periph].sck |= g_spi_pins[dev->periph].sck_mode;

    spi_base->CR |= 0x00000002;     // Reset SPI
    spi_base->CR &= ~(0x00000002);  // Clear reset SPI
    spi_base->CFGR1 |= 0x00000001;  // Set Master Mode
    spi_base->CCR |=
        0x0C0C0F00 |
        CLK_DIV;  // Set Clock Divider to 7 -> Clock Speed is now 14.67 Mhz
    uint32_t prescaler = ceil(log2(((double)BASE_CLK / CLK_DIV)) / dev->clk);
    if (prescaler > 7) {
        return ERROR;
    }
    spi_base->TCR |= (prescaler << 27);  // set computed prescaler
    spi_base->CR |= 0x00000001;          // Enable SPI

    return 0;
}

Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf,
                    uint8_t len) {
    LPSPI_t* spi_base = g_spi_base[dev->periph];
    if (len > 256 || len > 4 * (1 << ((spi_base->PARAM >> 8) & 0xFF))) {
        return ERROR;
    }  // the length cannot be greater than the maximum frame size or the size
       // of the rx FIFO
    if (spi_base->SR & 0x01000000) {
        return BUSY;
    }  // return BUSY if bus is busy

    if (g_spi_state[dev->periph].clk == 0) {
        if (spi_setup(dev) != OK) {
            return ERROR;
        }  // If the peripheral is not yet set up, set it up
        g_spi_state[dev->periph].clk = dev->clk;
        g_spi_state[dev->periph].cpha = dev->cpha;
        g_spi_state[dev->periph].cpol = dev->cpol;
        g_spi_state[dev->periph].cs = dev->cs;
    }
    if (g_spi_state[dev->periph].cpha != dev->cpha ||
        g_spi_state[dev->periph].cpol != dev->cpol) {
        spi_base->TCR = (spi_base->TCR & 0x3FFFFFFFF) | (dev->cpol << 31) |
                        (dev->cpha << 30);  // If cpol or cpha changed, set them
    }

    // Find and set PCS
    uint8_t cs_num = dev->cs;
    cs_num = checkCS(dev, cs_num);
    if (cs_num < 0) {
        return ERROR;
    }
    spi_base->TCR |= (cs_num << 24);
    spi_base->TCR = (spi_base->TCR & 0xFFFFF000) | (len * 4 - 1);

    uint32_t tx_data = 0;
    uint8_t num_words = len / 4 + (len % 4 != 0);
    for (int word = 0; word < num_words; word++) {
        if (word == num_words - 1 && len % 4 != 0) {
            for (int byte = 0; byte < len % 4; byte++) {
                tx_data |= (uint32_t) * (tx_buf + (word * 4) + byte)
                           << (8 * (len % 4 - 1 -
                                    byte));  // Combine write data into one word
            }
        } else {
            for (int byte = 0; byte < 4; byte++) {
                tx_data |=
                    (uint32_t) * (tx_buf + (word * 4) + byte)
                    << (8 * (3 - byte));  // Combine write data into one word
            }
        }
        while (spi_base->FSR & 0x1F == (1 << (spi_base->PARAM & 0xFF))) {
        }                         // Wait for the tx FIFO to empty
        spi_base->TDR = tx_data;  // Write data to be sent
    }

    while (!(spi_base->SR & 0x00000400) || !(spi_base->FSR & 0x001F0000)) {
    }  // Wait until transfer complete and words are in the rx buffer
    uint32_t rx_data;
    for (int word = 0; word < num_words; word++) {
        rx_data = spi_base->RDR;  // Read data from the receive FIFO
        if (word == num_words - 1 && len % 4 != 0) {
            for (int byte = 0; byte < len % 4; byte++) {
                *(rx_buf + (word * 4) + byte) =
                    (uint8_t)(rx_data >> ((len % 4 - 1 - byte) *
                                          8));  // Split each read word into
                                                // bytes and store in the rx_buf
            }
        } else {
            for (int byte = 0; byte < 4; byte++) {
                *(rx_buf + (word * 4) + byte) =
                    (uint8_t)(rx_data >> ((3 - byte) *
                                          8));  // Split each read word into
                                                // bytes and store in the rx_buf
            }
        }
    }

    return OK;
}