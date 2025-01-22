#include "adc/adc.h"

#include "stm32h7xx.h"
#include "string.h"
#include "utils.h"

#define INT_ABS(x) ((x) < 0 ? -(x) : (x))

static ADC_TypeDef* adc_base[3] = {ADC1_BASE, ADC2_BASE, ADC3_BASE};
static int adc12_sample_times_x2[8] = {3, 5, 17, 33, 129, 112, 775, 1621};
static int adc3_sample_times_x2[8] = {5, 13, 25, 49, 95, 185, 495, 1281};
static uint32_t adc1_sample_result[16];
static uint32_t adc2_sample_result[16];
static uint32_t adc3_sample_result[16];

Status adc_init(ADCDevice* dev) {
    // Check parameters
    // Peripheral
    if (dev->periph < ADC1 || dev->periph > ADC3) {
        return STATUS_PARAMETER_ERROR;
    }
    // Resolution
    // Allowed values for ADC12: 8, 10, 12, 14, 16
    if (dev->periph < ADC3 &&
        (dev->resolution_bits < 8 || dev->resolution_bits > 16 ||
         dev->resolution_bits % 2 != 0)) {
        return STATUS_PARAMETER_ERROR;
    }
    // Allowed values for ADC3: 6, 8, 10, 12
    if (dev->periph >= ADC3 &&
        (dev->resolution_bits < 6 || dev->resolution_bits > 12 ||
         dev->resolution_bits % 2 != 0)) {
        return STATUS_PARAMETER_ERROR;
    }

    // Clear results
    if (dev->periph == ADC1) {
        memset(adc1_sample_result, 0, sizeof(adc1_sample_result));
    } else if (dev->periph == ADC2) {
        memset(adc2_sample_result, 0, sizeof(adc2_sample_result));
    } else if (dev->periph == ADC3) {
        memset(adc3_sample_result, 0, sizeof(adc3_sample_result));
    }

    // Enable clocks
    if (dev->periph == ADC1) {
        __HAL_RCC_ADC12_CLK_ENABLE();
    } else if (dev->periph == ADC2) {
        __HAL_RCC_ADC12_CLK_ENABLE();
    } else if (dev->periph == ADC3) {
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

    // Set resolution and alignment
    adc_base[dev->periph]->CFGR &= ~ADC_CFGR_RES;
    if (dev->periph < ADC3) {
        adc_base[dev->periph]->CFGR |= ((16 - dev->resolution_bits) >> 1) << 2;
    } else {
        adc_base[dev->periph]->CFGR |= ((12 - dev->resolution_bits) >> 1) << 3;
        adc_base[dev->periph]->CFGR &= ADC3_CFGR_ALIGN;
    }

    // Set closest sample time
    int min_diff_tsamp = __INT_MAX__;
    int min_diff_tsamp_idx = 0;
    for (int i = 0; i < 8; i++) {
        int diff_tsamp = 0;
        if (dev->periph < ADC3) {
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
    int channel_count = 0;
    for (int i = 0; i < 16; i++) {
        if (dev->channel_mask & (1 << i)) {
            if (channel_count > 15) {
                // Only 16 channels in a sequence
                return STATUS_PARAMETER_ERROR;
            }
            // Enter the channel into the sequence
            switch (channel_count) {
                case 0:
                    adc_base[dev->periph]->SQR1 |= (i & 0x1F)
                                                   << ADC_SQR1_SQ1_Pos;
                    break;
                case 1:
                    adc_base[dev->periph]->SQR1 |= (i & 0x1F)
                                                   << ADC_SQR1_SQ2_Pos;
                    break;
                case 2:
                    adc_base[dev->periph]->SQR1 |= (i & 0x1F)
                                                   << ADC_SQR1_SQ3_Pos;
                    break;
                case 3:
                    adc_base[dev->periph]->SQR1 |= (i & 0x1F)
                                                   << ADC_SQR1_SQ4_Pos;
                    break;
                case 4:
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ5_Pos;
                    break;
                case 5:
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ6_Pos;
                    break;
                case 6:
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ7_Pos;
                    break;
                case 7:
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ8_Pos;
                    break;
                case 8:
                    adc_base[dev->periph]->SQR2 |= (i & 0x1F)
                                                   << ADC_SQR2_SQ9_Pos;
                    break;
                case 9:
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ10_Pos;
                    break;
                case 10:
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ11_Pos;
                    break;
                case 11:
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ12_Pos;
                    break;
                case 12:
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ13_Pos;
                    break;
                case 13:
                    adc_base[dev->periph]->SQR3 |= (i & 0x1F)
                                                   << ADC_SQR3_SQ14_Pos;
                    break;
                case 14:
                    adc_base[dev->periph]->SQR4 |= (i & 0x1F)
                                                   << ADC_SQR4_SQ15_Pos;
                    break;
                case 15:
                    adc_base[dev->periph]->SQR4 |= (i & 0x1F)
                                                   << ADC_SQR4_SQ16_Pos;
            }
            channel_count++;

            // Preselect channel
            if (dev->periph < ADC3) {
                adc_base[dev->periph]->PCSEL_RES0 |= 1 << i;
            }
        }
    }

    // Enter the sequence length
    adc_base[dev->periph]->SQR1 &= ~ADC_SQR1_L;
    adc_base[dev->periph]->SQR1 |= ((channel_count - 1) & 0xF)
                                   << ADC_SQR1_L_Pos;

    // Single conversion mode
    adc_base[dev->periph]->CFGR &= ~ADC_CFGR_CONT;

    // Set software trigger
    adc_base[dev->periph]->CFGR &= ~ADC_CFGR_EXTEN;

    // Enable EOC interrupt
    adc_base[dev->periph]->IER |= ADC_IER_EOCIE;
    if (dev->periph < ADC3) {
        NVIC_SetPriority(ADC_IRQn, 0);
        NVIC_EnableIRQ(ADC_IRQn);
    } else {
        NVIC_SetPriority(ADC3_IRQn, 0);
        NVIC_EnableIRQ(ADC3_IRQn);
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
    if (dev->periph < ADC1 || dev->periph > ADC3) {
        return STATUS_PARAMETER_ERROR;
    }

    // Start conversion
    adc_base[dev->periph]->CR |= ADC_CR_ADSTART;

    return STATUS_OK;
}

Status adc_ch_read(ADCDevice* dev, int chan, uint16_t* data) {
    if (chan < 0 || chan > 15) {
        return STATUS_PARAMETER_ERROR;
    }
    if (dev->periph == ADC1) {
        *data = (uint16_t)adc1_sample_result[chan];
    } else if (dev->periph == ADC2) {
        *data = (uint16_t)adc2_sample_result[chan];
    } else if (dev->periph == ADC3) {
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
        SYSCFG->ADC2ALT &= ~SYSCFG_ADC2ALT_ADC2_ROUT0;
    } else {
        SYSCFG->ADC2ALT |= SYSCFG_ADC2ALT_ADC2_ROUT0;
    }
}

void adc_set_ch17_alt_sw(bool state) {
    if (state) {
        SYSCFG->ADC2ALT &= ~SYSCFG_ADC2ALT_ADC2_ROUT1;
    } else {
        SYSCFG->ADC2ALT |= SYSCFG_ADC2ALT_ADC2_ROUT1;
    }
}