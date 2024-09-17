#ifndef STORAGE_H
#define STORAGE_H

#include "sd.h"
#include "status.h"

Status init_storage() {
    // Initialize FATFS
    ASSERT_OK(diskio_init(NULL), "diskio init");
    // ASSERT_OK(nand_flash_init(), "nand init");

    return STATUS_OK;
}

#endif  // STORAGE_H
