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

#include "nand_flash.h"
#include "tusb.h"

#if CFG_TUD_MSC

extern int g_nand_ready;

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16,
// 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                        uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;

    const char vid[] = "PSPHA";
    const char pid[] = "PAL 9000 Ver. 5";
    const char rev[] = "1.0";

    memcpy(vendor_id, vid, strlen(vid));
    memcpy(product_id, pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;

    // RAM disk is ready until ejected
    if (!g_nand_ready) {
        // Additional Sense 3A-00 is NOT_FOUND
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
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

    nand_flash_capacity(block_count, block_size);
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start,
                           bool load_eject) {
    (void)lun;
    (void)power_condition;

    if (load_eject) {
        if (start) {
            // load disk storage
        } else {
            // unload disk storage
        }
    }

    return true;
}

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void* buffer, uint32_t bufsize) {
    (void)lun;

    uint32_t block_count;
    uint16_t block_size;
    nand_flash_capacity(&block_count, &block_size);

    int to_read = bufsize;
    int read_curr = 0;
    BYTE temp_buffer[block_size];

    while (to_read) {
        // Read the block
        if (nand_flash_raw_read(temp_buffer, lba, 1) != STATUS_OK) {
            return read_curr;
        }

        // Size of data to copy from the block
        int data_size = MIN(to_read, block_size - offset);

        // Copy the appropriate part of the block to the buffer
        memcpy((BYTE*)buffer + read_curr, temp_buffer + offset, data_size);

        to_read -= data_size;
        read_curr += data_size;

        // Move to the next block
        offset = 0;
        lba++;
    }

    return read_curr;
}

bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;

#ifdef CFG_EXAMPLE_MSC_READONLY
    return false;
#else
    return true;
#endif
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t* buffer, uint32_t bufsize) {
    (void)lun;

    uint32_t block_count;
    uint16_t block_size;
    nand_flash_capacity(&block_count, &block_size);

    int to_write = bufsize;
    int wrote_curr = 0;
    BYTE temp_buffer[block_size];

    while (to_write) {
        if (offset) {
            // Read the block if offset
            if (nand_flash_raw_read(temp_buffer, lba, 1) != STATUS_OK) {
                return wrote_curr;
            }
        }

        // Size of data to copy to the buffer
        int data_size = MIN(to_write, block_size - offset);

        // Copy the buffer to the working buffer
        memcpy(temp_buffer + offset, (BYTE*)buffer + wrote_curr, data_size);

        // Pad the rest of the block with ones
        memset(temp_buffer + offset + data_size, 0xFF,
               block_size - offset - data_size);

        // Write the block
        if (nand_flash_raw_write(temp_buffer, lba, 1) != STATUS_OK) {
            return wrote_curr;
        }

        to_write -= data_size;
        wrote_curr += data_size;

        // Move to the next block
        offset = 0;
        lba++;
    }

    return wrote_curr;
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
