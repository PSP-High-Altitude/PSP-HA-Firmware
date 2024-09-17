#include "usb.h"

#include <errno.h>
#include <sys/unistd.h>

#include "main.h"
#include "timer.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_mtp_if.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

static bool s_usb_initialized = false;
static uint32_t s_usb_initialized_time = 0;

#ifdef DEBUG
static char s_usb_serial_buffer[SERIAL_BUFFER_SIZE];
static size_t s_usb_serial_buffer_idx = 0;
#endif

Status init_usb(bool mtp_mode) {
    MX_USB_DEVICE_Init(!mtp_mode);
    s_usb_initialized = true;

    // Allows us to delay a little so the USB host can
    // establish a connection before we start sending data.
    s_usb_initialized_time = MILLIS();

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
    if (!s_usb_initialized || (MILLIS() - s_usb_initialized_time < 500)) {
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
