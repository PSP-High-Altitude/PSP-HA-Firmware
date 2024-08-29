#include "usb.h"

#include <errno.h>
#include <sys/unistd.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_mtp_if.h"

static bool s_usb_initialized = false;
static char s_usb_serial_buffer[SERIAL_BUFFER_SIZE];
static size_t s_usb_serial_buffer_idx = 0;

Status init_usb(bool mtp_mode) {
    MX_USB_DEVICE_Init(!mtp_mode);
    s_usb_initialized = true;
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
    if (!s_usb_initialized) {
        int32_t copy_size = len;
        if (s_usb_serial_buffer_idx + copy_size >= SERIAL_BUFFER_SIZE) {
            copy_size = SERIAL_BUFFER_SIZE - s_usb_serial_buffer_idx;
        }
        memcpy(s_usb_serial_buffer + s_usb_serial_buffer_idx, data,
               copy_size * sizeof(char));
        s_usb_serial_buffer_idx += copy_size;
        return copy_size;
    }

    // Empty the serial buffer if it has content.
    if (s_usb_serial_buffer_idx > 0) {
        uint64_t start_time = MILLIS();
        USBD_StatusTypeDef rc = USBD_OK;

        do {
            rc = CDC_Transmit_HS((uint8_t *)s_usb_serial_buffer,
                                 s_usb_serial_buffer_idx);
        } while (USBD_BUSY == rc && MILLIS() - start_time < 10);

        s_usb_serial_buffer_idx = 0;
    }

    // Finally, transmit the newly provided data.
    uint64_t start_time = MILLIS();

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
