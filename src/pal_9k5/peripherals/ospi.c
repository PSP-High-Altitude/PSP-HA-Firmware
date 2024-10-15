#include "ospi.h"

#include "pal_9k5/board.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

#define OSPI_PIN_AF_COUNT 44

enum {
    OSPI_CLK = 1,
    OSPI_NCS = 2,
    OSPI_IO0 = 3,
    OSPI_IO1 = 4,
    OSPI_IO2 = 5,
    OSPI_IO3 = 6,
    OSPI_IO4 = 7,
    OSPI_IO5 = 8,
    OSPI_IO6 = 9,
    OSPI_IO7 = 10,
};

typedef struct {
    uint8_t pin;
    uint8_t function;
    uint8_t af;
} OspiPinAf;

// List of all pin options for each peripheral
const OspiPinAf ospi_pin_af[2][OSPI_PIN_AF_COUNT] = {
    {
        // P1
        {PIN_PA1, OSPI_IO3, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PA2, OSPI_IO0, GPIO_AF6_OCTOSPIM_P1},
        {PIN_PA3, OSPI_IO2, GPIO_AF6_OCTOSPIM_P1},
        {PIN_PA3, OSPI_CLK, GPIO_AF12_OCTOSPIM_P1},
        {PIN_PA6, OSPI_IO3, GPIO_AF6_OCTOSPIM_P1},
        {PIN_PA7, OSPI_IO2, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PB0, OSPI_IO1, GPIO_AF4_OCTOSPIM_P1},
        {PIN_PB1, OSPI_IO0, GPIO_AF4_OCTOSPIM_P1},
        {PIN_PB2, OSPI_CLK, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PB6, OSPI_NCS, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PB10, OSPI_NCS, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PB12, OSPI_IO0, GPIO_AF12_OCTOSPIM_P1},
        {PIN_PB13, OSPI_IO2, GPIO_AF4_OCTOSPIM_P1},
        {PIN_PC0, OSPI_IO4, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PC2, OSPI_IO5, GPIO_AF4_OCTOSPIM_P1},
        {PIN_PC2, OSPI_IO2, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PC3, OSPI_IO6, GPIO_AF4_OCTOSPIM_P1},
        {PIN_PC3, OSPI_IO0, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PC9, OSPI_IO0, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PC10, OSPI_IO1, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PC11, OSPI_NCS, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PD4, OSPI_IO4, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PD5, OSPI_IO5, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PD6, OSPI_IO6, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PD7, OSPI_IO7, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PD11, OSPI_IO0, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PD12, OSPI_IO1, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PD13, OSPI_IO3, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PE2, OSPI_IO2, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PE7, OSPI_IO4, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PE8, OSPI_IO5, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PE9, OSPI_IO6, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PE10, OSPI_IO7, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PE11, OSPI_NCS, GPIO_AF11_OCTOSPIM_P1},
        {PIN_PF6, OSPI_IO3, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PF7, OSPI_IO2, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PF8, OSPI_IO0, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PF9, OSPI_IO1, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PF10, OSPI_CLK, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PG6, OSPI_NCS, GPIO_AF10_OCTOSPIM_P1},
        {PIN_PG9, OSPI_IO6, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PG14, OSPI_IO7, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PH2, OSPI_IO4, GPIO_AF9_OCTOSPIM_P1},
        {PIN_PH3, OSPI_IO5, GPIO_AF9_OCTOSPIM_P1},
    },
    {
        // P2
        {PIN_PF0, OSPI_IO0, GPIO_AF9_OCTOSPIM_P2},
        {PIN_PF1, OSPI_IO1, GPIO_AF9_OCTOSPIM_P2},
        {PIN_PF2, OSPI_IO2, GPIO_AF9_OCTOSPIM_P2},
        {PIN_PF3, OSPI_IO3, GPIO_AF9_OCTOSPIM_P2},
        {PIN_PF4, OSPI_CLK, GPIO_AF9_OCTOSPIM_P2},
        {PIN_PG0, OSPI_IO4, GPIO_AF9_OCTOSPIM_P2},
        {PIN_PG1, OSPI_IO5, GPIO_AF9_OCTOSPIM_P2},
        {PIN_PG10, OSPI_IO6, GPIO_AF3_OCTOSPIM_P2},
        {PIN_PG11, OSPI_IO7, GPIO_AF9_OCTOSPIM_P2},
        {PIN_PG12, OSPI_NCS, GPIO_AF3_OCTOSPIM_P2},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
        {0},
    },
};

static OCTOSPI_TypeDef* ospi_base[2] = {OCTOSPI1, OCTOSPI2};
static OSPI_HandleTypeDef ospi1_handle = {0};
static OSPI_HandleTypeDef ospi2_handle = {0};
static OSPI_HandleTypeDef* ospi_handles[2] = {&ospi1_handle, &ospi2_handle};
static MDMA_HandleTypeDef hmdma_octospi;

static Status get_pin(uint8_t periph, uint8_t pin, uint8_t function,
                      uint32_t* af) {
    for (int i = 0; i < OSPI_PIN_AF_COUNT; i++) {
        // Invalid function means we ran out of pins
        if (ospi_pin_af[periph][i].function == 0) {
            return STATUS_ERROR;
        }

        // Check if the pin is a match
        if (ospi_pin_af[periph][i].pin == pin &&
            ospi_pin_af[periph][i].function == function) {
            *af = ospi_pin_af[periph][i].af;
            return STATUS_OK;
        }
    }

    return STATUS_ERROR;
}

static Status ospi_setup(OSpiDevice* dev) {
    // Check if the peripheral is valid
    if (dev->periph < P_OSPI1 || dev->periph > P_OSPI2) {
        return STATUS_PARAMETER_ERROR;
    }

    // Check if the peripheral has already been initialized
    // Also, only one OSPI can be used at a time
    if (ospi1_handle.State != 0 || ospi2_handle.State != 0) {
        return STATUS_OK;
    }

    __HAL_RCC_OCTOSPIM_CLK_ENABLE();

    // Enable clock
    switch (dev->periph) {
        case P_OSPI1:
            __HAL_RCC_OSPI1_CLK_ENABLE();
            __HAL_RCC_OSPI1_FORCE_RESET();
            __HAL_RCC_OSPI1_RELEASE_RESET();
            break;
        case P_OSPI2:
            __HAL_RCC_OSPI2_CLK_ENABLE();
            __HAL_RCC_OSPI2_FORCE_RESET();
            __HAL_RCC_OSPI2_RELEASE_RESET();
            break;
    }

    OCTOSPI_TypeDef* base = ospi_base[dev->periph];
    GPIO_InitTypeDef pin_conf = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };

    // Get the CLK pin
    if (get_pin(dev->periph, dev->sck, OSPI_CLK, &pin_conf.Alternate) !=
        STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    pin_conf.Pin = PAL_GPIO_PIN(dev->sck);
    HAL_GPIO_Init(PAL_GPIO_PORT(dev->sck), &pin_conf);

    // Get the NCS pin
    if (get_pin(dev->periph, dev->ncs, OSPI_NCS, &pin_conf.Alternate) !=
        STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    pin_conf.Pin = PAL_GPIO_PIN(dev->ncs);
    HAL_GPIO_Init(PAL_GPIO_PORT(dev->ncs), &pin_conf);

    // Get the IO pins
    int port = 0;
    uint8_t io_pins[4] = {dev->io0, dev->io1, dev->io2, dev->io3};
try_gpio_port:
    for (int i = 0; i < 4; i++) {
        // Get each pin AF, if one doesn't match, we will break and go to
        // the higher port and try again.
        if (get_pin(dev->periph, io_pins[i], OSPI_IO0 + i + (port * 4),
                    &pin_conf.Alternate) != STATUS_OK) {
            port++;
            if (port < 2)
                goto try_gpio_port;
            else
                return STATUS_PARAMETER_ERROR;
        }
        pin_conf.Pin = PAL_GPIO_PIN(io_pins[i]);
        HAL_GPIO_Init(PAL_GPIO_PORT(io_pins[i]), &pin_conf);
    }

    // Set clock speed
    uint32_t prescale = 0;
    switch (dev->clk) {
        case OSPI_SPEED_1MHz:
            prescale = 80;
            break;
        case OSPI_SPEED_5MHz:
            prescale = 16;
            break;
        case OSPI_SPEED_10MHz:
            prescale = 8;
            break;
        case OSPI_SPEED_20MHz:
            prescale = 4;
            break;
        case OSPI_SPEED_40MHz:
            prescale = 2;
            break;
        case OSPI_SPEED_80MHz:
            prescale = 1;
            break;
        default:
            return STATUS_PARAMETER_ERROR;
    }

    OSPI_InitTypeDef init_conf = {
        .ClockPrescaler = prescale,
        .FifoThreshold = 1,
        .DualQuad = HAL_OSPI_DUALQUAD_DISABLE,
        .SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE,
        .DeviceSize = dev->device_size,
        .MemoryType = HAL_OSPI_MEMTYPE_MICRON,
        .ChipSelectHighTime = 1,
        .ClockMode = HAL_OSPI_CLOCK_MODE_0,
        .WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED,
        .DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE,
        .ChipSelectBoundary = 0,
        .DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED,
        .FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE,
        .MaxTran = 0,
        .Refresh = 0,
    };
    ospi_handles[dev->periph]->Init = init_conf;
    ospi_handles[dev->periph]->Instance = base;
    if (HAL_OSPI_Init(ospi_handles[dev->periph]) != HAL_OK) {
        return STATUS_ERROR;
    }
    OSPIM_CfgTypeDef sOspiManagerCfg = {0};
    sOspiManagerCfg.ClkPort = 1;
    sOspiManagerCfg.NCSPort = 1;
    const uint32_t low_pin_map[2][2] = {
        {HAL_OSPIM_IOPORT_1_LOW, HAL_OSPIM_IOPORT_1_HIGH},
        {HAL_OSPIM_IOPORT_2_LOW, HAL_OSPIM_IOPORT_2_HIGH},
    };
    sOspiManagerCfg.IOLowPort = low_pin_map[dev->periph][port];
    sOspiManagerCfg.DQSPort = 0;
    if (HAL_OSPIM_Config(ospi_handles[dev->periph], &sOspiManagerCfg,
                         HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return STATUS_ERROR;
    }

    hmdma_octospi.Instance = MDMA_Channel0;
    hmdma_octospi.Init.Request = (dev->periph == P_OSPI1)
                                     ? MDMA_REQUEST_OCTOSPI1_FIFO_TH
                                     : MDMA_REQUEST_OCTOSPI2_FIFO_TH;
    hmdma_octospi.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
    hmdma_octospi.Init.Priority = MDMA_PRIORITY_LOW;
    hmdma_octospi.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    hmdma_octospi.Init.SourceInc = MDMA_SRC_INC_BYTE;
    hmdma_octospi.Init.DestinationInc = MDMA_DEST_INC_BYTE;
    hmdma_octospi.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
    hmdma_octospi.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
    hmdma_octospi.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma_octospi.Init.BufferTransferLength = 1;
    hmdma_octospi.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma_octospi.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    hmdma_octospi.Init.SourceBlockAddressOffset = 0;
    hmdma_octospi.Init.DestBlockAddressOffset = 0;
    if (HAL_MDMA_Init(&hmdma_octospi) != HAL_OK) {
        return STATUS_ERROR;
    }

    if (HAL_MDMA_ConfigPostRequestMask(&hmdma_octospi, 0, 0) != HAL_OK) {
        return STATUS_ERROR;
    }

    __HAL_LINKDMA(ospi_handles[dev->periph], hmdma, hmdma_octospi);

    HAL_NVIC_SetPriority(
        (dev->periph == P_OSPI1) ? OCTOSPI1_IRQn : OCTOSPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ((dev->periph == P_OSPI1) ? OCTOSPI1_IRQn
                                                : OCTOSPI2_IRQn);
    HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(MDMA_IRQn);

    return STATUS_OK;
}

Status ospi_cmd(OSpiDevice* dev, OSpiCommand* cmd) {
    uint32_t data_mode = HAL_OSPI_DATA_1_LINE;
    switch (cmd->data_mode) {
        case OSPI_DATA_NONE:
            data_mode = HAL_OSPI_DATA_NONE;
            break;
        case OSPI_DATA_1_LINE:
            data_mode = HAL_OSPI_DATA_1_LINE;
            break;
        case OSPI_DATA_2_LINES:
            data_mode = HAL_OSPI_DATA_2_LINES;
            break;
        case OSPI_DATA_4_LINES:
            data_mode = HAL_OSPI_DATA_4_LINES;
            break;
    }

    uint32_t addr_mode = HAL_OSPI_ADDRESS_1_LINE;
    switch (cmd->addr_mode) {
        case OSPI_ADDR_NONE:
            addr_mode = HAL_OSPI_ADDRESS_NONE;
            break;
        case OSPI_ADDR_1_LINE:
            addr_mode = HAL_OSPI_ADDRESS_1_LINE;
            break;
        case OSPI_ADDR_2_LINES:
            addr_mode = HAL_OSPI_ADDRESS_2_LINES;
            break;
        case OSPI_ADDR_4_LINES:
            addr_mode = HAL_OSPI_ADDRESS_4_LINES;
            break;
    }

    uint32_t inst_mode = HAL_OSPI_INSTRUCTION_1_LINE;
    switch (cmd->inst_mode) {
        case OSPI_INST_1_LINE:
            inst_mode = HAL_OSPI_INSTRUCTION_1_LINE;
            break;
        case OSPI_INST_2_LINES:
            inst_mode = HAL_OSPI_INSTRUCTION_2_LINES;
            break;
        case OSPI_INST_4_LINES:
            inst_mode = HAL_OSPI_INSTRUCTION_4_LINES;
            break;
    }

    uint32_t addr_size = HAL_OSPI_ADDRESS_8_BITS;
    switch (cmd->n_addr) {
        case OSPI_ADDR_1_BYTE:
            addr_size = HAL_OSPI_ADDRESS_8_BITS;
            break;
        case OSPI_ADDR_2_BYTES:
            addr_size = HAL_OSPI_ADDRESS_16_BITS;
            break;
        case OSPI_ADDR_3_BYTES:
            addr_size = HAL_OSPI_ADDRESS_24_BITS;
            break;
        case OSPI_ADDR_4_BYTES:
            addr_size = HAL_OSPI_ADDRESS_32_BITS;
            break;
    }

    OSPI_RegularCmdTypeDef hal_cmd = {
        .OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
        .FlashId = HAL_OSPI_FLASH_ID_1,
        .Instruction = cmd->inst,
        .InstructionMode = inst_mode,
        .InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
        .InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
        .Address = cmd->addr,
        .AddressMode = addr_mode,
        .AddressSize = addr_size,
        .AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,
        .AlternateBytes = 0,
        .AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
        .AlternateBytesSize = 0,
        .AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE,
        .DataMode = data_mode,
        .DummyCycles = cmd->n_dummy,
        .NbData = cmd->n_data,
        .SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
        .DQSMode = HAL_OSPI_DQS_DISABLE,

    };

    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (HAL_OSPI_Command(ospi_handles[dev->periph], &hal_cmd, 100) != HAL_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    return STATUS_OK;
}

Status ospi_auto_poll_cmd(OSpiDevice* dev, OSpiCommand* cmd, OSpiAutopoll* cfg,
                          uint64_t timeout) {
    if (ospi_setup(dev) != STATUS_OK) {
        return STATUS_PARAMETER_ERROR;
    }
    if (cmd->n_data != 1) {
        return STATUS_PARAMETER_ERROR;
    }

    uint64_t start_time = MILLIS();
    while (MILLIS() - start_time < timeout) {
        // Poll the device
        uint32_t rx_buf;
        if (ospi_cmd(dev, cmd) != STATUS_OK) {
            return STATUS_HARDWARE_ERROR;
        }
        if (HAL_OSPI_Receive_DMA(ospi_handles[dev->periph],
                                 (uint8_t*)&rx_buf) != HAL_OK) {
            return STATUS_HARDWARE_ERROR;
        }

        // Wait for the data to come
        while (HAL_OSPI_GetState(ospi_handles[dev->periph]) !=
               HAL_OSPI_STATE_READY) {
            DELAY(cfg->interval);
        }

        // Check if the data matches
        rx_buf = (cfg->match_mode == OSPI_MATCH_AND) ? (rx_buf & cfg->mask)
                                                     : (rx_buf | cfg->mask);
        if (rx_buf == cfg->match) {
            return STATUS_OK;
        }
    }
    return STATUS_TIMEOUT_ERROR;
}

Status ospi_write(OSpiDevice* dev, OSpiCommand* cmd, uint8_t* tx_buf,
                  uint64_t timeout) {
    if (ospi_cmd(dev, cmd) != STATUS_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (HAL_OSPI_Transmit_DMA(ospi_handles[dev->periph], tx_buf) != HAL_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    uint64_t start_time = MILLIS();
    while (HAL_OSPI_GetState(ospi_handles[dev->periph]) !=
           HAL_OSPI_STATE_READY) {
        if (MILLIS() - start_time > timeout) {
            return STATUS_TIMEOUT_ERROR;
        }
        DELAY(0);
    }
    return STATUS_OK;
}

Status ospi_read(OSpiDevice* dev, OSpiCommand* cmd, uint8_t* rx_buf,
                 uint64_t timeout) {
    if (ospi_cmd(dev, cmd) != STATUS_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    if (HAL_OSPI_Receive_DMA(ospi_handles[dev->periph], rx_buf) != HAL_OK) {
        return STATUS_HARDWARE_ERROR;
    }
    uint64_t start_time = MILLIS();
    while (HAL_OSPI_GetState(ospi_handles[dev->periph]) !=
           HAL_OSPI_STATE_READY) {
        if (MILLIS() - start_time > timeout) {
            return STATUS_TIMEOUT_ERROR;
        }
        DELAY(0);
    }
    return STATUS_OK;
}

void OCTOSPI1_IRQHandler(void) { HAL_OSPI_IRQHandler(&ospi1_handle); }

void OCTOSPI2_IRQHandler(void) { HAL_OSPI_IRQHandler(&ospi2_handle); }

void MDMA_IRQHandler(void) { HAL_MDMA_IRQHandler(&hmdma_octospi); }