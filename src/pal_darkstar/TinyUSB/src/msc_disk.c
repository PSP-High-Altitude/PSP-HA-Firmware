/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "fatfs/diskio.h"
#include "tasks/storage.h"
#include "tusb.h"

#if CFG_TUD_MSC

#define MAX_BLOCK_SIZE (512)

static uint8_t s_buf[MAX_BLOCK_SIZE];

static uint32_t s_block_count;
static uint16_t s_block_size;
static bool s_ejected = false;

static void get_disk_capacity() {
    DWORD count;
    DWORD size;

    // Here block == sector (for MMC)
    disk_ioctl(0, GET_SECTOR_COUNT, &count);
    disk_ioctl(0, GET_SECTOR_SIZE, &size);

    // Do this to ensure no resizing shenanigans
    s_block_count = count;
    s_block_size = size;
}

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16,
// 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                        uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;

    const char vid[] = "PSPHA";
    const char pid[] = "PAL Darkstar";
    const char rev[] = "1.0";

    memcpy(vendor_id, vid, strlen(vid));
    memcpy(product_id, pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    // If ejected, always not ready
    if (s_ejected) {
        return false;
    }

    // Tell the storage task to stop for MSC
    storage_pause(STORAGE_PAUSE_MSC);

    // Wait for storage task to clean up
    if (storage_is_active()) {
        // SCIS SCI 04-01 is Not Ready - becoming ready
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x01);
        return false;
    }

    // Do this here to make sure the cached values are correct
    // In case read/writes are issued before capacity read
    get_disk_capacity();

    // Make sure we can actually support this
    if (s_block_size > MAX_BLOCK_SIZE) {
        tud_msc_set_sense(lun, SCSI_SENSE_HARDWARE_ERROR, 0x00, 0x00);
        return false;
    }

    return true;
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and
// SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size Application update
// block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count,
                         uint16_t* block_size) {
    (void)lun;

    get_disk_capacity();

    *block_count = s_block_count;
    *block_size = s_block_size;
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start,
                           bool load_eject) {
    (void)lun;
    (void)power_condition;

    if (load_eject && !start) {
        storage_start(STORAGE_PAUSE_MSC);
        s_ejected = true;
    }

    return true;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void* buffer, uint32_t bufsize) {
    (void)lun;

    DRESULT res;

    uint32_t btr = bufsize;
    uint32_t br = 0;

    // If we have a large aligned access, do a burst transfer
    if (offset == 0 && bufsize > s_block_size) {
        uint32_t blocks_to_read = bufsize / s_block_size;
        res = disk_read(0, buffer, lba, blocks_to_read);
        if (res != RES_OK) {
            return br;
        }
        lba += blocks_to_read;
        br += blocks_to_read * s_block_size;
        btr -= br;
    }

    while (btr) {
        // Read a single block into the intermediate buffer
        res = disk_read(0, s_buf, lba, 1);
        if (res != RES_OK) {
            return br;
        }

        // Size of data to copy from the block
        uint32_t btc = btr;
        if (offset + btc > s_block_size) {
            btc = s_block_size - offset;
        }

        // Copy the appropriate part of the block to the buffer
        memcpy((BYTE*)buffer + br, s_buf + offset, btc);

        btr -= btc;
        br += btc;

        // Move to the next block
        offset = 0;
        lba++;
    }

    return br;
}

bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;

    return false;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t* buffer, uint32_t bufsize) {
    (void)lun;

    DRESULT res;

    uint32_t btw = bufsize;
    uint32_t bw = 0;

    // If we have a large aligned access, do a burst transfer
    if (offset == 0 && bufsize > s_block_size) {
        uint32_t blocks_to_write = bufsize / s_block_size;
        res = disk_write(0, buffer, lba, blocks_to_write);
        if (res != RES_OK) {
            return bw;
        }
        lba += blocks_to_write;
        bw += blocks_to_write * s_block_size;
        btw -= bw;
    }

    while (btw) {
        // Size of data to copy into the intermediate buffer
        uint32_t btc = btw;
        if (offset + btc > s_block_size) {
            btc = s_block_size - offset;
        }

        // If we're not copying a whole block, read it first
        res = disk_read(0, s_buf, lba, 1);
        if (res != RES_OK) {
            return bw;
        }

        // Copy the appropriate part of the block to the buffer
        memcpy(s_buf + offset, (BYTE*)buffer + bw, btc);

        // Write a single block from the intermediate buffer
        res = disk_write(0, s_buf, lba, 1);
        if (res != RES_OK) {
            return bw;
        }

        btw -= btc;
        bw += btc;

        // Move to the next block
        offset = 0;
        lba++;
    }

    return bw;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer,
                        uint16_t bufsize) {
    // read10 & write10 has their own callback and MUST not be handled here

    void const* response = NULL;
    int32_t resplen = 0;

    // most scsi handled is input
    bool in_xfer = true;

    switch (scsi_cmd[0]) {
        default:
            // Set Sense = Invalid Command Operation
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

            // negative means error -> tinyusb could stall and/or response with
            // failed status
            resplen = -1;
            break;
    }

    // return resplen must not larger than bufsize
    if (resplen > bufsize) resplen = bufsize;

    if (response && (resplen > 0)) {
        if (in_xfer) {
            memcpy(buffer, response, (size_t)resplen);
        } else {
            // SCSI output
        }
    }

    return (int32_t)resplen;
}

#endif
