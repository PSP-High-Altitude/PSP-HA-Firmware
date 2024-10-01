#include "usb.h"

#include <errno.h>
#include <sys/unistd.h>

#include "FreeRTOS.h"
#include "backup.h"
#include "button_event.h"
#include "gpio/gpio.h"
#include "main.h"
#include "timer.h"
#include "timers.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_mtp_if.h"

static void mtp_button_handler();

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

static bool s_usb_initialized = false;
static uint32_t s_usb_initialized_time = 0;

#ifdef DEBUG
static char s_usb_serial_buffer[SERIAL_BUFFER_SIZE];
static size_t s_usb_serial_buffer_idx = 0;
#endif

static ButtonEventConfig g_mtp_button = {
    .pin = PIN_MTP,
    .rising = true,
    .falling = false,
    .event_handler = mtp_button_handler,
};

static xTimerHandle g_mtp_button_timer;

// If the MTP button is not pressed
static void mtp_button_timeout(TimerHandle_t timer) {
    // Handle button timeout
    button_event_destroy(&g_mtp_button);
    printf("MTP mode was not selected!\n");
}

// If the MTP button is pressed
static void mtp_button_handler() {
    // Handle button press
    button_event_destroy(&g_mtp_button);
    printf("MTP mode selected!\n");
    xTimerStopFromISR(g_mtp_button_timer, 0);

    DELAY_MICROS(1000000);
    get_backup_ptr()->flag_mtp_pressed = 1;
    NVIC_SystemReset();
}

Status usb_init() {
    MX_USB_DEVICE_Init(!get_backup_ptr()->flag_mtp_pressed);
    s_usb_initialized = true;

    // Allows us to delay a little so the USB host can
    // establish a connection before we start sending data.
    s_usb_initialized_time = MILLIS();

    // Wait asynchronously to enter MTP mode
    if (!get_backup_ptr()->flag_mtp_pressed) {
        button_event_create(&g_mtp_button);
        g_mtp_button_timer =
            xTimerCreate("mtp_button_timeout", pdMS_TO_TICKS(5000), pdFALSE,
                         NULL, mtp_button_timeout);
        xTimerStart(g_mtp_button_timer, 0);
    }

    // Clear the flag so we go to normal mode next time
    get_backup_ptr()->flag_mtp_pressed = 0;

    return STATUS_OK;
}

// Serial debug stuff -- used by printf
int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

#ifdef DEBUG
    // If the USB isn't yet initialized, buffer the writes in an internal buffer
    // so that they can be output when the interface actually gets initialized.
    if (!s_usb_initialized || (MILLIS() - s_usb_initialized_time < 1000)) {
        int32_t copy_size = len;
        if (s_usb_serial_buffer_idx + copy_size >= SERIAL_BUFFER_SIZE) {
            copy_size = SERIAL_BUFFER_SIZE - s_usb_serial_buffer_idx;
        }
        memcpy(s_usb_serial_buffer + s_usb_serial_buffer_idx, data,
               copy_size * sizeof(char));
        s_usb_serial_buffer_idx += copy_size;
        return copy_size;
    }

    uint64_t start_time = MILLIS();
    USBD_StatusTypeDef rc = USBD_OK;

    // Empty the serial buffer if it has content.
    if (s_usb_serial_buffer_idx > 0) {
        do {
            rc = CDC_Transmit_HS((uint8_t *)s_usb_serial_buffer,
                                 s_usb_serial_buffer_idx);
        } while (USBD_BUSY == rc && MILLIS() - start_time < 10);

        s_usb_serial_buffer_idx = 0;
    }

    // Finally, transmit the newly provided data.
    start_time = MILLIS();

    do {
        rc = CDC_Transmit_HS((uint8_t *)data, len);
    } while (USBD_BUSY == rc && MILLIS() - start_time < 10);

    if (USBD_FAIL == rc) {
        return 0;
    }
#endif

    return len;
}

void OTG_HS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS); }
