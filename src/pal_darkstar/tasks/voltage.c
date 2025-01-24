#include "voltage.h"

#include "adc/adc.h"
#include "board.h"
#include "gpio/gpio.h"

Status voltage_init() {
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

    ADCDevice vadc = {
        .periph = P_ADC2,
        .resolution_bits = 16,
        .sample_time_cycles_x2 = 5,
        .channel_mask = 0x000506B1,
    };

    // Close PA0 analog switch
    adc_set_pa0_sw(true);
    // Set INP16 alternate to Vbat/4
    adc_set_ch16_alt_sw(true);

    // GPIO modes
    gpio_mode(PIN_PA0, GPIO_INPUT);
    gpio_mode(PIN_PC4, GPIO_INPUT);
    gpio_mode(PIN_PB1, GPIO_INPUT);
    gpio_mode(PIN_PA7, GPIO_INPUT);
    gpio_mode(PIN_PB0, GPIO_INPUT);
    gpio_mode(PIN_PC0, GPIO_INPUT);
    gpio_mode(PIN_PA4, GPIO_INPUT);

    // Initialize ADC
    if (adc_init(&vadc) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}