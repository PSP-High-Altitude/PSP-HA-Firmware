#include "pyros.h"

#include "FreeRTOS.h"
#include "gpio/gpio.h"
#include "main.h"
#include "queue.h"
#include "timer.h"

/********************/
/* STATIC VARIABLES */
/********************/
static QueueHandle_t s_pyro_queue_handle;

static uint32_t s_retries_left[] = {
    [PYRO_MAIN] = PYRO_MAX_RETRIES,
    [PYRO_DRG] = PYRO_MAX_RETRIES,
    [PYRO_AUX] = PYRO_MAX_RETRIES,
};

/********************/
/* HELPER FUNCTIONS */
/********************/

static Status get_pyro_pins(Pyro pyro, uint8_t* fire_pin, uint8_t* cont_pin) {
    switch (pyro) {
        case PYRO_MAIN:
            *fire_pin = PIN_FIREMAIN;
            *cont_pin = PIN_CONTMAIN;
            return STATUS_OK;
        case PYRO_DRG:
            *fire_pin = PIN_FIREDRG;
            *cont_pin = PIN_CONTDRG;
            return STATUS_OK;
        case PYRO_AUX:
            *fire_pin = PIN_FIREAUX;
            *cont_pin = PIN_CONTAUX;
            return STATUS_OK;
        default:
            return STATUS_PARAMETER_ERROR;
    }
}

/*****************/
/* API FUNCTIONS */
/*****************/

Status init_pyros() {
    // Initialize the pyros
    gpio_mode(PIN_CONTMAIN, GPIO_INPUT);
    gpio_mode(PIN_CONTDRG, GPIO_INPUT);
    gpio_mode(PIN_CONTAUX, GPIO_INPUT);
    gpio_write(PIN_FIREMAIN, GPIO_LOW);
    gpio_write(PIN_FIREDRG, GPIO_LOW);
    gpio_write(PIN_FIREAUX, GPIO_LOW);

    s_pyro_queue_handle = xQueueCreate(PYRO_QUEUE_LEN, sizeof(Pyro));

    return STATUS_OK;
}

Status fire_pyro(Pyro pyro) {
    if (xQueueSend(s_pyro_queue_handle, &pyro, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    // We want the pyro to fire immediately, so force a context switch (this
    // assumes that the pyro task priority is the highest in the system)
    taskYIELD();

    return STATUS_OK;
}

void pyros_task() {
    while (1) {
        Pyro pyro;
        if (xQueueReceive(s_pyro_queue_handle, &pyro, portMAX_DELAY) !=
            pdPASS) {
            // If we somehow time out here, just go back to the start because we
            // do NOT want to accidentally trigger pyros
            printf("pyro restart recv\n");
            continue;
        }

        // Get the pins (dummy init values to appease compiler)
        uint8_t fire_pin = 0;
        uint8_t cont_pin = 0;
        Status pin_status = EXPECT_OK(get_pyro_pins(pyro, &fire_pin, &cont_pin),
                                      "pyro pin mapping");

        // If the pin is somehow invalid, something weird is going on so just
        // abort and wait for a new command
        if (pin_status != STATUS_OK) {
            printf("pyro invalid pin\n");
            continue;
        }

        // Fire the pyro
        printf("*** FIRING %d ***\n", pyro);
        gpio_write(fire_pin, GPIO_HIGH);
        DELAY(PYRO_FIRE_LENGTH_MS);
        gpio_write(fire_pin, GPIO_LOW);

        // If we still detect continuity on the pin, retry if we still have
        // retries left by readding the command to the queue
        if (gpio_read(cont_pin) == GPIO_HIGH && s_retries_left[pyro] > 0) {
            printf("Pyro retrying (%lu retries left)\n", s_retries_left[pyro]);

            while (xQueueSend(s_pyro_queue_handle, &pyro, 1) != pdPASS) {
                // This should be impossible with a long enough queue, but retry
                // a few times anyway just in case
                s_retries_left[pyro] -= 1;
            }

            s_retries_left[pyro] -= 1;
        }
    }
}
