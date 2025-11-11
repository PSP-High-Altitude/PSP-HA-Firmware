#include "pyros.h"

#include "FreeRTOS.h"
#include "gpio/gpio.h"
#include "main.h"
#include "queue.h"
#include "status.h"
#include "timer.h"

/********************/
/* STATIC VARIABLES */
/********************/
static QueueHandle_t s_pyro_queue_handle;

static uint32_t s_retries_left[] = {
    [PYRO_MAIN] = PYRO_MAX_RETRIES, [PYRO_DRG] = PYRO_MAX_RETRIES,
    [PYRO_A1] = PYRO_MAX_RETRIES,   [PYRO_A2] = PYRO_MAX_RETRIES,
    [PYRO_A3] = PYRO_MAX_RETRIES,
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
        case PYRO_A1:
            *fire_pin = PIN_FIREA1;
            *cont_pin = PIN_CONTA1;
            return STATUS_OK;
        case PYRO_A2:
            *fire_pin = PIN_FIREA2;
            *cont_pin = PIN_CONTA2;
            return STATUS_OK;
        case PYRO_A3:
            *fire_pin = PIN_FIREA3;
            *cont_pin = PIN_CONTA3;
            return STATUS_OK;
        default:
            return STATUS_PARAMETER_ERROR;
    }
}

/*****************/
/* API FUNCTIONS */
/*****************/

Status pyros_init() {
    // Initialize the pyros
    gpio_mode(PIN_CONTMAIN, GPIO_INPUT);
    gpio_mode(PIN_CONTDRG, GPIO_INPUT);
    gpio_mode(PIN_CONTA1, GPIO_INPUT);
    gpio_mode(PIN_CONTA2, GPIO_INPUT);
    gpio_mode(PIN_CONTA3, GPIO_INPUT);
    gpio_write(PIN_FIREMAIN, GPIO_LOW);
    gpio_write(PIN_FIREDRG, GPIO_LOW);
    gpio_write(PIN_FIREA1, GPIO_LOW);
    gpio_write(PIN_FIREA2, GPIO_LOW);
    gpio_write(PIN_FIREA3, GPIO_LOW);

    s_pyro_queue_handle = xQueueCreate(PYRO_QUEUE_LEN, sizeof(Pyro));

    return STATUS_OK;
}

Status pyros_fire(Pyro pyro) {
    if (xQueueSend(s_pyro_queue_handle, &pyro, 0) != pdPASS) {
        return STATUS_BUSY;
    }

    // We want the pyro to fire immediately, so force a context switch (this
    // assumes that the pyro task priority is the highest in the system)
    taskYIELD();

    return STATUS_OK;
}

bool pyros_cont(Pyro pyro) {
    // Get the pins (dummy init values to appease compiler)
    uint8_t fire_pin = 0;
    uint8_t cont_pin = 0;

    EXPECT_OK(get_pyro_pins(pyro, &fire_pin, &cont_pin), "pyro pin mapping");

    return gpio_read(cont_pin);
}

void task_pyros() {
    while (1) {
        Pyro pyro;
        if (xQueueReceive(s_pyro_queue_handle, &pyro, portMAX_DELAY) !=
            pdPASS) {
            // If we somehow time out here, just go back to the start because we
            // do NOT want to accidentally trigger pyros
            PAL_LOGW("Pyro receive timeout\n");
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
            PAL_LOGE("Invalid pyro pin; aborting!\n");
            continue;
        }

        // Fire the pyro
        PAL_LOGI("*** FIRING %d ***\n", pyro);
        gpio_write(fire_pin, GPIO_HIGH);
        DELAY(PYRO_FIRE_LENGTH_MS);
        gpio_write(fire_pin, GPIO_LOW);
        DELAY(PYRO_CHECK_DELAY_MS);

        // If we still detect continuity on the pin, retry if we still have
        // retries left by readding the command to the queue (for round robin)
        if (gpio_read(cont_pin) == GPIO_HIGH && s_retries_left[pyro] > 0) {
            PAL_LOGW("Pyro retrying (%lu retries left)\n",
                     s_retries_left[pyro] - 1);

            while (xQueueSend(s_pyro_queue_handle, &pyro, 1) != pdPASS) {
                // This should be impossible with a long enough queue,
                // but retry a few times anyway just in case
                s_retries_left[pyro] -= 1;
            }

            s_retries_left[pyro] -= 1;
        }
    }
}
