#include "pyros.h"

#include "gpio/gpio.h"
#include "main.h"

void init_pyros(void) {
    // Initialize the pyros
    gpio_mode(PIN_CONTMAIN, GPIO_INPUT);
    gpio_mode(PIN_CONTDRG, GPIO_INPUT);
    gpio_mode(PIN_CONTAUX, GPIO_INPUT);
    gpio_write(PIN_FIREMAIN, 0);
    gpio_write(PIN_FIREDRG, 0);
    gpio_write(PIN_FIREAUX, 0);
}

void fire_pyro(uint8_t pyro) {
    // Fire the specified pyro
    switch (pyro) {
        case MAIN_PYRO:
            printf("Firing main pyro\n");
            break;
        case DROGUE_PYRO:
            printf("Firing drogue pyro\n");
            break;
        case AUX_PYRO(0):
            printf("Firing aux pyro 0\n");
            break;
        default:
            break;
    }
}