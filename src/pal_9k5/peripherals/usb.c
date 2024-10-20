#include "usb.h"

#include <errno.h>
#include <sys/unistd.h>

#include "FreeRTOS.h"
#include "Regex.h"
#include "backup.h"
#include "button_event.h"
#include "commands.h"
#include "fifos.h"
#include "gpio/gpio.h"
#include "main.h"
#include "rtc/rtc.h"
#include "tasks/storage.h"
#include "terminal/terminal.h"
#include "timer.h"
#include "tusb.h"

static bool s_usb_initialized = false;
static uint32_t s_usb_initialized_time = 0;

#ifdef DEBUG
static uint8_t s_usb_serial_buffer[CFG_TUD_CDC_TX_BUFSIZE];
static FIFO_t s_usb_serial_fifo = {
    .buffer = s_usb_serial_buffer,
    .size = CFG_TUD_CDC_TX_BUFSIZE,
    .circ = 0,
    .head = 0,
    .tail = 0,
    .count = 0,
};
uint8_t ser_out_buf[CFG_TUD_CDC_TX_BUFSIZE];

#endif

Status usb_init() {
#ifdef DEBUG
    // Low level Init
    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
    NVIC_SetPriority(OTG_HS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

    // Initialize terminal interface and add commands
    terminal_init();
    terminal_add_cmd(regex_help, cmd_help);
    terminal_add_cmd(regex_set_datetime, cmd_set_datetime);
    terminal_add_cmd(regex_get_datetime, cmd_get_datetime);
    terminal_add_cmd(regex_invalidate_config, cmd_invalidate_config);
    terminal_add_cmd(regex_erase_flash_chip, cmd_erase_flash_chip);
    terminal_add_cmd(regex_set_frequency, cmd_set_frequency);
#endif

    return STATUS_OK;
}

// Serial debug stuff -- used by printf
int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

    /***************************/
    /*       NAND Section      */
    /***************************/

    storage_write_log(data, len);
#ifdef DEBUG
    /***************************/
    /*       USB Section       */
    /***************************/

    // If the USB isn't yet initialized, buffer the writes in an internal buffer
    // so that they can be output when the interface actually gets initialized
    if (!s_usb_initialized || (MILLIS() - s_usb_initialized_time < 1000) ||
        xPortIsInsideInterrupt()) {
        int32_t copy_size =
            fifo_enqueuen(&s_usb_serial_fifo, (uint8_t *)data, len);
        return copy_size;
    }

    // If the USB is initialized, write the data to the USB interface.
    int new_len = s_usb_serial_fifo.count;
    fifo_dequeuen(&s_usb_serial_fifo, ser_out_buf, new_len);
    tud_cdc_write(ser_out_buf, new_len);

    // Send data
    tud_cdc_write(data, len);

#endif

    gpio_write(PIN_RED, GPIO_LOW);
    return len;
}

void tud_cdc_rx_cb(uint8_t itf) {
    int len = tud_cdc_available();

    // char str[*len + 1];
    char *str = malloc(len + 1);
    tud_cdc_read(str, len);
    str[len] = '\0';

    terminal_process(str);

    free(str);
}

void task_usb(void *param) {
    (void)param;

    // Configure DM DP Pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_PWREx_EnableUSBVoltageDetector();

    // init device stack on configured roothub port
    // This should be called after scheduler/kernel is started.
    // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS
    // queue API.
    tud_init(BOARD_TUD_RHPORT);

    s_usb_initialized = true;

    // Allows us to delay a little so the USB host can
    // establish a connection before we start sending data.
    s_usb_initialized_time = MILLIS();

    // RTOS forever loop
    while (1) {
        // put this thread to waiting state until there is new events
        tud_task();

        // following code only run if tud_task() process at least 1 event
        tud_cdc_write_flush();
    }
}

void OTG_HS_IRQHandler(void) { tud_int_handler(0); }
