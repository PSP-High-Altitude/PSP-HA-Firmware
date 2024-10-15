#include "usb.h"

#include <errno.h>
#include <sys/unistd.h>

#include "FreeRTOS.h"
#include "Regex.h"
#include "backup.h"
#include "button_event.h"
#include "fifos.h"
#include "gpio/gpio.h"
#include "main.h"
#include "rtc/rtc.h"
#include "tasks/storage.h"
#include "timer.h"
#include "tusb.h"

TaskHandle_t s_usb_device_handle;

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

    // If the USB is initialized, write the data to the USB interface
    // and flush the buffer.
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

    // Help command
    Regex regex_help;
    regexCompile(&regex_help, "^help[\n]*$");

    // Set datetime command
    Regex regex_set_datetime;
    regexCompile(&regex_set_datetime,
                 "^set_datetime [0-9]{4}\\-[0-9]{2}\\-[0-9]{2} "
                 "[0-9]{2}:[0-9]{2}:[0-9]{2}[\n]*$");

    // Get datetime command
    Regex regex_get_datetime;
    regexCompile(&regex_get_datetime, "^get_datetime[\n]*$");

    // Help command
    Regex regex_invalidate_config;
    regexCompile(&regex_invalidate_config, "^invalidate_config[\n]*$");

    // Test all commands
    Matcher match = regexMatch(&regex_help, str);
    if (match.isFound) {
        printf(
            "Commands:\n"
            "  help                                  this command\n"
            "  set_datetime YYYY-MM-DD HH:MM:SS      sets the RTC time\n"
            "  get_datetime                          gets the RTC time\n"
            "  invalidate_config                     invalidates the config\n");
        free(str);
        return;
    }
    match = regexMatch(&regex_set_datetime, str);
    if (match.isFound) {
        int year, month, day, hour, minute, second;
        sscanf(str, "set_datetime %d-%d-%d %d:%d:%d", &year, &month, &day,
               &hour, &minute, &second);
        RTCDateTime dt = {year, month, day, hour, minute, second};
        rtc_set_datetime(dt);
        printf("Time set!\n");
        free(str);
        return;
    }
    match = regexMatch(&regex_get_datetime, str);
    if (match.isFound) {
        RTCDateTime dt = rtc_get_datetime(dt);
        printf("Current time: %04ld-%02ld-%02ld %02ld:%02ld:%02ld\n", dt.year,
               dt.month, dt.day, dt.hour, dt.minute, dt.second);
        free(str);
        return;
    }
    match = regexMatch(&regex_invalidate_config, str);
    if (match.isFound) {
        config_invalidate();
        free(str);
        return;
    }
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
