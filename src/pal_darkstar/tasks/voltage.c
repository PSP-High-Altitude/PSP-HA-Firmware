#include "voltage.h"

#include "adc/adc.h"
#include "board.h"
#include "gpio/gpio.h"
#include "stdio.h"
#include "timer.h"

/***************************/
/*   Voltages to sample    */
/*-------------------------*/
/* INP0    Vpyro     PA0_C */
/* INP4    Pyro A3   PC4   */
/* INP5    Pyro MN   PB1   */
/* INP7    Pyro A1   PA7   */
/* INP9    Pyro DG   PB0   */
/* INP10   Vin       PC0   */
/* INP16   Vbat/4          */
/* INP18   PYRO A2   PA4   */
/***************************/
static ADCDevice s_vadc = {
    .periph = P_ADC2,
    .resolution_bits = 16,
    .sample_time_cycles_x2 = 30,
    .channel_mask = 0x000506B1,
};

Status voltage_init() {
    // Close PA0 analog switch
    adc_set_pa0_sw(true);
    // Set INP16 alternate to Vbat/4
    adc_set_ch16_alt_sw(true);

    // GPIO modes
    gpio_mode(PIN_PA0, GPIO_ANALOG);
    gpio_mode(PIN_PC4, GPIO_ANALOG);
    gpio_mode(PIN_PB1, GPIO_ANALOG);
    gpio_mode(PIN_PA7, GPIO_ANALOG);
    gpio_mode(PIN_PB0, GPIO_ANALOG);
    gpio_mode(PIN_PC0, GPIO_ANALOG);
    gpio_mode(PIN_PA4, GPIO_ANALOG);

    // Initialize ADC
    if (adc_init(&s_vadc) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

void task_voltage(TaskHandle_t *handle_ptr) {
    while (1) {
        adc_conv(&s_vadc);
        DELAY(100);
        uint16_t adc_vpy, adc_a3, adc_mn, adc_a1;
        adc_ch_read(&s_vadc, 0, &adc_vpy);
        adc_ch_read(&s_vadc, 1, &adc_a3);
        adc_ch_read(&s_vadc, 2, &adc_mn);
        adc_ch_read(&s_vadc, 3, &adc_a1);
        double vpy = (double)adc_vpy * 3.3 / 65535;
        double a3 = (double)adc_a3 * (470.0 / (470.0 + 10.0)) * 3.3 / 65535;
        double mn = (double)adc_mn * (470.0 / (470.0 + 10.0)) * 3.3 / 65535;
        double dg = (double)adc_a1 * (470.0 / (470.0 + 10.0)) * 3.3 / 65535;
        printf("VPY: %.3f V, V_A3: %.3f V, V_MN: %.3f V, V_A1: %.3f V\n", vpy,
               a3, mn, dg);
        DELAY(100);
    }
}