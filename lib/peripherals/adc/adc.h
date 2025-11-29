#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>

#include "status.h"

typedef enum {
    P_ADC1 = 0,
    P_ADC2 = 1,
    P_ADC3 = 2,
} ADCPeriph;

typedef struct {
    ADCPeriph periph;           // ADC peripheral
    int resolution_bits;        // Sample resolution in bits
    int sample_time_cycles_x2;  // Sample time in ADC clock cycles * 2
    uint32_t channel_mask;      // Bitmask of channels to sample
} ADCDevice;

// Initialize an ADC peripheral and return the status.
Status adc_init(ADCDevice* dev);

// Start an ADC conversion and return the status.
Status adc_conv(ADCDevice* dev);

// Read the last value of an ADC channel and return the status.
// Data are right aligned in the 16-bit word.
Status adc_ch_read(ADCDevice* dev, int chan, uint16_t* data);

// Open (false) or close (true) the analog switch between pin PC3 and PC3_C.
// SYSCFG clock must be active.
void adc_set_pc3_sw(bool state);

// Open (false) or close (true) the analog switch between pin PC2 and PC2_C.
// SYSCFG clock must be active.
void adc_set_pc2_sw(bool state);

// Open (false) or close (true) the analog switch between pin PA1 and PA1_C.
// SYSCFG clock must be active.
void adc_set_pa1_sw(bool state);

// Open (false) or close (true) the analog switch between pin PA0 and PA0_C.
// SYSCFG clock must be active.
void adc_set_pa0_sw(bool state);

// Set ADC2 channel 16 alternate to either dac1_out1 (false) or VBAT/4 (true).
// SYSCFG clock must be active.
void adc_set_ch16_alt_sw(bool state);

// Set ADC2 channel 17 alternate to either dac1_out2 (false) or VREFINT (true).
// SYSCFG clock must be active.
void adc_set_ch17_alt_sw(bool state);

#endif  // ADC_H