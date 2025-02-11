#include "adc/adc.h"

#include "stm32h7xx.h"
#include "string.h"
#include "timer.h"
#include "utils.h"

#define INT_ABS(x) ((x) < 0 ? -(x) : (x))

static ADC_TypeDef* adc_base[3] = {ADC1, ADC2, ADC3};
static int adc12_sample_times_x2[8] = {3, 5, 17, 33, 129, 112, 775, 1621};
static int adc3_sample_times_x2[8] = {5, 13, 25, 49, 95, 185, 495, 1281};
RAM_D2 static uint32_t adc1_sample_result[16];
RAM_D2 static uint32_t adc2_sample_result[16];
RAM_D2 static uint32_t adc3_sample_result[16];
static int channel_count[3] = {0, 0, 0};

Status adc_init(ADCDevice* dev) {
    // Check parameters
    // Peripheral
    if (dev->periph < P_ADC1 || dev->periph > P_ADC3) {
        return STATUS_PARAMETER_ERROR;
    }
    // Resolution
    // Allowed values for ADC12: 8, 10, 12, 14, 16
    if (dev->periph < P_ADC3 &&
        (dev->resolution_bits < 8 || dev->resolution_bits > 16 ||
         dev->resolution_bits % 2 != 0)) {
        return STATUS_PARAMETER_ERROR;
    }
    // Allowed values for ADC3: 6, 8, 10, 12
    if (dev->periph >= P_ADC3 &&
        (dev->resolution_bits < 6 || dev->resolution_bits > 12 ||
         dev->resolution_bits % 2 != 0)) {
        return STATUS_PARAMETER_ERROR;
    }

    // Clear results
    if (dev->periph == P_ADC1) {
        memset(adc1_sample_result, 0, sizeof(adc1_sample_result));
    } else if (dev->periph == P_ADC2) {
        memset(adc2_sample_result, 0, sizeof(adc2_sample_result));
    } else if (dev->periph == P_ADC3) {
        memset(adc3_sample_result, 0, sizeof(adc3_sample_result));
    }

    // Enable clocks
    if (dev->periph == P_ADC1) {
        __HAL_RCC_ADC12_CLK_ENABLE();
    } else if (dev->periph == P_ADC2) {
        __HAL_RCC_ADC12_CLK_ENABLE();
    } else if (dev->periph == P_ADC3) {
        __HAL_RCC_ADC3_CLK_ENABLE();
    }

    // Disable peripheral
    adc_base[dev->periph]->CR |= ADC_CR_ADDIS;
    Status result = STATUS_OK;
    // Wait on a timeout for the ADC to disable
    WAIT_COND_BLOCKING_TIMEOUT((adc_base[dev->periph]->CR & ADC_CR_ADEN) == 0,
                               200, result);
    if (result != STATUS_OK) {
        return result;
    }

    // Configure clocks
    if (dev->periph == P_ADC1) {
        ADC12_COMMON->CCR &= ~ADC_CCR_PRESC;
        ADC12_COMMON->CCR |= ADC_CCR_PRESC_3;  // div 32
    } else if (dev->periph == P_ADC2) {
        ADC12_COMMON->CCR &= ~ADC_CCR_PRESC;
        ADC12_COMMON->CCR |= ADC_CCR_PRESC_3;  // div 32
    } else if (dev->periph == P_ADC3) {
        ADC3_COMMON->CCR &= ~ADC_CCR_PRESC;
        ADC3_COMMON->CCR |= ADC_CCR_PRESC_3;  // div 32
    }

    // Startup
    adc_base[dev->periph]->CR &= ~ADC_CR_DEEPPWD;
    adc_base[dev->periph]->CR |= ADC_CR_ADVREGEN;
    DELAY_MICROS(10);  // Wait for voltage regulator to stabilize

    // Calibration
    adc_base[dev->periph]->CR |= ADC_CR_ADCAL;
    WAIT_COND_BLOCKING_TIMEOUT((adc_base[dev->periph]->CR & ADC_CR_ADCAL) == 0,
                               200, result);

    // Set resolution, alignment, dma
    adc_base[dev->periph]->CFGR &= ~ADC_CFGR_RES;
    if (dev->periph < P_ADC3) {
        adc_base[dev->periph]->CFGR &= ~ADC_CFGR_RES;
        adc_base[dev->periph]->CFGR |= ((16 - dev->resolution_bits) >> 1) << 2;
        adc_base[dev->periph]->CFGR &= ~ADC_CFGR_DMNGT;
        adc_base[dev->periph]->CFGR |= ADC_CFGR_DMNGT_0;  // DMA one shot
    } else {
        adc_base[dev->periph]->CFGR &= ~ADC3_CFGR_RES;
        adc_base[dev->periph]->CFGR |= ((12 - dev->resolution_bits) >> 1) << 3;
        adc_base[dev->periph]->CFGR &= ~ADC3_CFGR_ALIGN;   // Right alignment
        adc_base[dev->periph]->CFGR |= ADC3_CFGR_DMAEN;    // DMA enable
        adc_base[dev->periph]->CFGR &= ~ADC3_CFGR_DMACFG;  // one shot mode
    }

    // Set closest sample time
    int min_diff_tsamp = __INT_MAX__;
    int min_diff_tsamp_idx = 0;
    for (int i = 0; i < 8; i++) {
        int diff_tsamp = 0;
        if (dev->periph < P_ADC3) {
            diff_tsamp =
                INT_ABS(dev->sample_time_cycles_x2 - adc12_sample_times_x2[i]);
        } else {
            diff_tsamp =
                INT_ABS(dev->sample_time_cycles_x2 - adc3_sample_times_x2[i]);
        }
        if (diff_tsamp < min_diff_tsamp) {
            min_diff_tsamp = diff_tsamp;
            min_diff_tsamp_idx = i;
        }
    }

    // Configure channels
    channel_count[dev->periph] = 0;
    for (int i = 0; i < 20; i++) {
        if (dev->channel_mask & (1 << i)) {
            if (channel_count[dev->periph] > 15) {
                // Only 16 channels in a sequence
                return STATUS_PARAMETER_ERROR;
            }
            // Enter the channel into the sequence
            switch (channel_count[dev->periph]) {
                case 0:
                    adc_base[dev->periph]->SQR1 &= ~ADC_SQR1_SQ1;
                    adc_base[dev->periph]->SQR1 |= (i & 0x1F)
                                                   << ADC_SQR1_SQ1_Pos;
                    break;
                case 1:
                    adc_base[dev->periph]->SQR1 &= ~ADC_SQR1_SQ2;
                    adc_base[dev->periph]->SQR1 |= (i & 0x1F)
                                                   << ADC_SQR1_SQ2_Pos;
                    break;
                case 2:
                    adc_base[dev->periph]->SQR1 &= ~ADC_SQR1_SQ3;
                    adc_base[dev->periph]->SQR1 |= (i & 0x1F)
                                                   << ADC_SQR1_SQ3_Pos;
                    break;
                case 3:
                    adc_base[dev->periph]->SQR1 &= ~ADC_SQR1_SQ4;
                    adc_base[dev->periph]->SQR1 |= (i & 0x1F)
                                                   << ADC_SQR1_SQ4_Pos;
                    break;
                case 4:
                    adc_base[dev->periph]->SQR2 &= ~ADC_SQR2_SQ5;
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ5_Pos;
                    break;
                case 5:
                    adc_base[dev->periph]->SQR2 &= ~ADC_SQR2_SQ6;
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ6_Pos;
                    break;
                case 6:
                    adc_base[dev->periph]->SQR2 &= ~ADC_SQR2_SQ7;
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ7_Pos;
                    break;
                case 7:
                    adc_base[dev->periph]->SQR2 &= ~ADC_SQR2_SQ8;
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ8_Pos;
                    break;
                case 8:
                    adc_base[dev->periph]->SQR2 &= ~ADC_SQR2_SQ9;
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ9_Pos;
                    break;
                case 9:
                    adc_base[dev->periph]->SQR3 &= ~ADC_SQR3_SQ10;
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ10_Pos;
                    break;
                case 10:
                    adc_base[dev->periph]->SQR3 &= ~ADC_SQR3_SQ11;
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ11_Pos;
                    break;
                case 11:
                    adc_base[dev->periph]->SQR3 &= ~ADC_SQR3_SQ12;
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ12_Pos;
                    break;
                case 12:
                    adc_base[dev->periph]->SQR3 &= ~ADC_SQR3_SQ13;
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ13_Pos;
                    break;
                case 13:
                    adc_base[dev->periph]->SQR3 &= ~ADC_SQR3_SQ14;
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ14_Pos;
                    break;
                case 14:
                    adc_base[dev->periph]->SQR4 &= ~ADC_SQR4_SQ15;
                    adc_base[dev->periph]->SQR4 |= (i & 0x1F)
                                                   << ADC_SQR4_SQ15_Pos;
                    break;
                case 15:
                    adc_base[dev->periph]->SQR4 &= ~ADC_SQR4_SQ16;
                    adc_base[dev->periph]->SQR4 |= (i & 0x1F)
                                                   << ADC_SQR4_SQ16_Pos;
            }
            channel_count[dev->periph]++;

            // Set sample time
            if (dev->periph < P_ADC3) {
                if (i < 10) {
                    adc_base[dev->periph]->SMPR1 &= ~(0x7 << (3 * i));
                    adc_base[dev->periph]->SMPR1 |= min_diff_tsamp_idx
                                                    << (3 * i);
                } else {
                    adc_base[dev->periph]->SMPR2 &= ~(0x7 << (3 * (i - 10)));
                    adc_base[dev->periph]->SMPR2 |= min_diff_tsamp_idx
                                                    << (3 * (i - 10));
                }
            } else {
                if (i < 10) {
                    adc_base[dev->periph]->SMPR1 &= ~(0x7 << (3 * i));
                    adc_base[dev->periph]->SMPR1 |= min_diff_tsamp_idx
                                                    << (3 * i);
                } else {
                    adc_base[dev->periph]->SMPR2 &= ~(0x7 << (3 * (i - 10)));
                    adc_base[dev->periph]->SMPR2 |= min_diff_tsamp_idx
                                                    << (3 * (i - 10));
                }
            }

            // Preselect channel
            if (dev->periph < P_ADC3) {
                adc_base[dev->periph]->PCSEL_RES0 |= 1 << i;
            }
        }
    }

    // If no channels were selected, return an error
    if (channel_count[dev->periph] == 0) {
        return STATUS_PARAMETER_ERROR;
    }

    // Enter the sequence length
    adc_base[dev->periph]->SQR1 &= ~ADC_SQR1_L;
    adc_base[dev->periph]->SQR1 |= ((channel_count[dev->periph] - 1) & 0xF)
                                   << ADC_SQR1_L_Pos;

    // Single conversion mode
    adc_base[dev->periph]->CFGR &= ~ADC_CFGR_CONT;

    // Set software trigger
    adc_base[dev->periph]->CFGR &= ~ADC_CFGR_EXTEN;

    // Interrupts
    adc_base[dev->periph]->IER = 0;
    adc_base[dev->periph]->IER |= ADC_IER_EOCIE;
    adc_base[dev->periph]->IER |= ADC_IER_EOSIE;
    adc_base[dev->periph]->IER |= ADC_IER_EOSMPIE;
    if (dev->periph < P_ADC3) {
        NVIC_SetPriority(ADC_IRQn, 0);
        NVIC_EnableIRQ(ADC_IRQn);
    } else {
        NVIC_SetPriority(ADC3_IRQn, 0);
        NVIC_EnableIRQ(ADC3_IRQn);
    }

    // DMA channel
    __HAL_RCC_DMA1_CLK_ENABLE();

    DMA1_Stream2->CR &= ~DMA_SxCR_EN;
    DMA1_Stream2->PAR = (uint32_t)&adc_base[dev->periph]->DR;
    DMA1_Stream2->CR &=
        ~(DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_TRBUFF | DMA_SxCR_CT |
          DMA_SxCR_DBM | DMA_SxCR_PL | DMA_SxCR_PINCOS | DMA_SxCR_MSIZE |
          DMA_SxCR_PSIZE | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_MINC |
          DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR | DMA_SxCR_PFCTRL |
          DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE |
          DMA_SxCR_EN);
    DMA1_Stream2->CR |= DMA_SxCR_MINC;     // Memory increment mode
    DMA1_Stream2->CR &= ~DMA_SxCR_PSIZE;   // Clear peripheral data size
    DMA1_Stream2->CR |= DMA_SxCR_PSIZE_1;  // Peripheral data size: 32 bits
    DMA1_Stream2->CR |= DMA_SxCR_MSIZE_1;  // Memory data size: 32 bits

    NVIC_SetPriority(DMA1_Stream2_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    // DMAMUX
    switch (dev->periph) {
        case P_ADC1:
            DMA1_Stream2->M0AR = (uint32_t)adc1_sample_result;
            DMAMUX1_Channel2->CCR = 9U << DMAMUX_CxCR_DMAREQ_ID_Pos;
            break;
        case P_ADC2:
            DMA1_Stream2->M0AR = (uint32_t)adc2_sample_result;
            DMAMUX1_Channel2->CCR = 10U << DMAMUX_CxCR_DMAREQ_ID_Pos;
            break;
        case P_ADC3:
            DMA1_Stream2->M0AR = (uint32_t)adc3_sample_result;
            DMAMUX1_Channel2->CCR = 115U << DMAMUX_CxCR_DMAREQ_ID_Pos;
            break;
    }

    // Enable ADC
    adc_base[dev->periph]->CR |= ADC_CR_ADEN;
    WAIT_COND_BLOCKING_TIMEOUT(
        (adc_base[dev->periph]->ISR & ADC_ISR_ADRDY) != 0, 200, result);
    if (result != STATUS_OK) {
        return result;
    }

    return STATUS_OK;
}

Status adc_conv(ADCDevice* dev) {
    // Check parameters
    if (dev->periph < P_ADC1 || dev->periph > P_ADC3) {
        return STATUS_PARAMETER_ERROR;
    }

    // Enable DMA
    DMA1_Stream2->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR |= DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                   DMA_LIFCR_CDMEIF2;
    switch (dev->periph) {
        case P_ADC1:
            DMA1_Stream2->M0AR = (uint32_t)adc1_sample_result;
            break;
        case P_ADC2:
            DMA1_Stream2->M0AR = (uint32_t)adc2_sample_result;
            break;
        case P_ADC3:
            DMA1_Stream2->M0AR = (uint32_t)adc3_sample_result;
            break;
    }
    DMA1_Stream2->NDTR = channel_count[dev->periph];  // Number of data items
    DMA1_Stream2->CR |= DMA_SxCR_EN;

    // Start conversion
    adc_base[dev->periph]->CR |= ADC_CR_ADSTART;

    return STATUS_OK;
}

Status adc_ch_read(ADCDevice* dev, int chan, uint16_t* data) {
    if (chan < 0 || chan > 15) {
        return STATUS_PARAMETER_ERROR;
    }
    if (dev->periph == P_ADC1) {
        *data = (uint16_t)adc1_sample_result[chan];
    } else if (dev->periph == P_ADC2) {
        *data = (uint16_t)adc2_sample_result[chan];
    } else if (dev->periph == P_ADC3) {
        *data = (uint16_t)adc3_sample_result[chan];
    } else {
        return STATUS_PARAMETER_ERROR;
    }
    return STATUS_OK;
}

void adc_set_pc3_sw(bool state) {
    if (state) {
        SYSCFG->PMCR &= ~SYSCFG_PMCR_PC3SO;
    } else {
        SYSCFG->PMCR |= SYSCFG_PMCR_PC3SO;
    }
}
void adc_set_pc2_sw(bool state) {
    if (state) {
        SYSCFG->PMCR &= ~SYSCFG_PMCR_PC2SO;
    } else {
        SYSCFG->PMCR |= SYSCFG_PMCR_PC2SO;
    }
}

void adc_set_pa1_sw(bool state) {
    if (state) {
        SYSCFG->PMCR &= ~SYSCFG_PMCR_PA1SO;
    } else {
        SYSCFG->PMCR |= SYSCFG_PMCR_PA1SO;
    }
}

void adc_set_pa0_sw(bool state) {
    if (state) {
        SYSCFG->PMCR &= ~SYSCFG_PMCR_PA0SO;
    } else {
        SYSCFG->PMCR |= SYSCFG_PMCR_PA0SO;
    }
}

void adc_set_ch16_alt_sw(bool state) {
    if (state) {
        SYSCFG->ADC2ALT |= SYSCFG_ADC2ALT_ADC2_ROUT0;
        ADC3_COMMON->CCR |= ADC_CCR_VBATEN;
    } else {
        SYSCFG->ADC2ALT &= ~SYSCFG_ADC2ALT_ADC2_ROUT0;
        ADC3_COMMON->CCR &= ~ADC_CCR_VBATEN;
    }
}

void adc_set_ch17_alt_sw(bool state) {
    if (state) {
        SYSCFG->ADC2ALT |= SYSCFG_ADC2ALT_ADC2_ROUT1;
        ADC3_COMMON->CCR |= ADC_CCR_VREFEN;
    } else {
        SYSCFG->ADC2ALT &= ~SYSCFG_ADC2ALT_ADC2_ROUT1;
        ADC3_COMMON->CCR &= ~ADC_CCR_VREFEN;
    }
}

void DMA1_Stream2_IRQHandler(void) {
    // Clear interrupts
    if (DMA1->LISR & DMA_LISR_TCIF2) {
        DMA1->LIFCR |= DMA_LIFCR_CTCIF2;
    }
    if (DMA1->LISR & DMA_LISR_HTIF2) {
        DMA1->LIFCR |= DMA_LIFCR_CHTIF2;
    }
    if (DMA1->LISR & DMA_LISR_TEIF2) {
        DMA1->LIFCR |= DMA_LIFCR_CTEIF2;
    }
    if (DMA1->LISR & DMA_LISR_DMEIF2) {
        DMA1->LIFCR |= DMA_LIFCR_CDMEIF2;
    }
}

void ADC_IRQHandler(void) {
    // Clear interrupts
    if (ADC1->ISR & ADC_ISR_EOC) {
        ADC1->ISR |= ADC_ISR_EOC;
    }
    if (ADC1->ISR & ADC_ISR_EOS) {
        ADC1->ISR |= ADC_ISR_EOS;
    }
    if (ADC1->ISR & ADC_ISR_EOSMP) {
        ADC1->ISR |= ADC_ISR_EOSMP;
    }
    if (ADC2->ISR & ADC_ISR_EOC) {
        ADC2->ISR |= ADC_ISR_EOC;
    }
    if (ADC2->ISR & ADC_ISR_EOS) {
        ADC2->ISR |= ADC_ISR_EOS;
    }
    if (ADC2->ISR & ADC_ISR_EOSMP) {
        ADC2->ISR |= ADC_ISR_EOSMP;
    }
}

void ADC3_IRQHandler(void) {
    // Clear interrupts
    if (ADC3->ISR & ADC_ISR_EOC) {
        ADC3->ISR |= ADC_ISR_EOC;
    }
    if (ADC3->ISR & ADC_ISR_EOS) {
        ADC3->ISR |= ADC_ISR_EOS;
    }
    if (ADC3->ISR & ADC_ISR_EOSMP) {
        ADC3->ISR |= ADC_ISR_EOSMP;
    }
}