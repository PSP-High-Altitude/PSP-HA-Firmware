#ifndef USB_H
#define USB_H

#include <stdbool.h>
#include <stdint.h>

#include "status.h"

Status usb_init();

void usb_device_task(void *param);

extern int _write(int file, char *data, int len);

#endif  // USB_H
