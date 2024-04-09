/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2018 Aleph One Ltd.
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "yaffs2/yaffs_driver.h"

#include "mt29f4g.h"

int drv_write_chunk_fn(struct yaffs_dev *dev, int nand_chunk, const u8 *data,
                       int data_len, const u8 *oob, int oob_len);
int drv_read_chunk_fn(struct yaffs_dev *dev, int nand_chunk, u8 *data,
                      int data_len, u8 *oob, int oob_len,
                      enum yaffs_ecc_result *ecc_result);
int drv_erase_fn(struct yaffs_dev *dev, int block_no);
int drv_mark_bad_fn(struct yaffs_dev *dev, int block_no);
int drv_check_bad_fn(struct yaffs_dev *dev, int block_no);
int drv_initialise_fn(struct yaffs_dev *dev);
int drv_deinitialise_fn(struct yaffs_dev *dev);

int yaffs_driver_init(struct yaffs_dev *dev) {
    memset(dev, 0, sizeof(struct yaffs_dev));
    dev->param.name = "/";
    dev->param.total_bytes_per_chunk = MT29F4G_PAGE_SIZE;
    dev->param.chunks_per_block = MT29F4G_PAGE_PER_BLOCK;
    dev->param.spare_bytes_per_chunk = MT29F4G_SPARE_SIZE;
    dev->param.n_reserved_blocks = 5;
    dev->param.start_block = 0;
    dev->param.end_block = MT29F4G_BLOCK_COUNT - 1;
    dev->param.n_caches = 0;
    dev->param.cache_bypass_aligned = 0;
    dev->param.use_nand_ecc = 0;
    dev->param.no_tags_ecc = 1;
    dev->param.is_yaffs2 = 1;

    dev->drv.drv_write_chunk_fn = drv_write_chunk_fn;
    dev->drv.drv_read_chunk_fn = drv_read_chunk_fn;
    dev->drv.drv_erase_fn = drv_erase_fn;
    dev->drv.drv_mark_bad_fn = drv_mark_bad_fn;
    dev->drv.drv_check_bad_fn = drv_check_bad_fn;
    dev->drv.drv_initialise_fn = drv_initialise_fn;
    dev->drv.drv_deinitialise_fn = drv_deinitialise_fn;

    yaffs_add_device(dev);

    return YAFFS_OK;
}

int drv_write_chunk_fn(struct yaffs_dev *dev, int nand_chunk, const u8 *data,
                       int data_len, const u8 *oob, int oob_len) {
    uint32_t row_addr = nand_chunk;
    uint32_t col_addr = 0;
    uint8_t status;

    // Write data
    if (data) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
        if (mt29f4g_write_partial_page(data, row_addr, col_addr, data_len) !=
            STATUS_OK) {
            return YAFFS_FAIL;
        }
#pragma GCC diagnostic pop
        status = mt29f4g_status();
        if ((status & (0x1 << 3))) {
            return YAFFS_FAIL;
        }
    }

    // Write tags
    if (oob) {
        col_addr = MT29F4G_SPARE_OFFSET;  // Metadata I
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
        if (mt29f4g_write_partial_page(oob, row_addr, col_addr, oob_len) !=
            STATUS_OK) {
            return YAFFS_FAIL;
        }
#pragma GCC diagnostic pop
        status = mt29f4g_status();
        if ((status & (0x1 << 3))) {
            return YAFFS_FAIL;
        }
    }

    return YAFFS_OK;
}

int drv_read_chunk_fn(struct yaffs_dev *dev, int nand_chunk, u8 *data,
                      int data_len, u8 *oob, int oob_len,
                      enum yaffs_ecc_result *ecc_result) {
    uint32_t row_addr = nand_chunk;
    uint32_t col_addr = 0;
    // uint8_t status;

    *ecc_result = YAFFS_ECC_RESULT_NO_ERROR;

    // Read data
    if (data) {
        if (mt29f4g_read_within_page(data, row_addr, col_addr, data_len) !=
            STATUS_OK) {
            return YAFFS_FAIL;
        }
        /*
            status = mt29f4g_status();
            status = (status >> 4) & 0x7;
            switch (status) {
                case 0b000:
                    if (*ecc_result <= YAFFS_ECC_RESULT_NO_ERROR)
                        *ecc_result = YAFFS_ECC_RESULT_NO_ERROR;
                    break;
                case 0b001:
                case 0b011:
                case 0b101:
                    if (*ecc_result <= YAFFS_ECC_RESULT_FIXED)
                        *ecc_result = YAFFS_ECC_RESULT_FIXED;
                    break;
                case 0b010:
                    printf("data_len %d, status: %x\n", data_len, status);
                    if (*ecc_result <= YAFFS_ECC_RESULT_UNFIXED)
                        *ecc_result = YAFFS_ECC_RESULT_UNFIXED;
                    break;
                default:
                    printf("status: %x\n", status);
                    if (*ecc_result <= YAFFS_ECC_RESULT_UNFIXED)
                        *ecc_result = YAFFS_ECC_RESULT_UNFIXED;
                    break;
            }
                    */
    }

    // Read oob
    if (oob) {
        col_addr = MT29F4G_SPARE_OFFSET;  // Metadata I
        if (mt29f4g_read_within_page(oob, row_addr, col_addr, oob_len) !=
            STATUS_OK) {
            return YAFFS_FAIL;
        }

        /*
            status = mt29f4g_status();
            status = (status >> 4) & 0x7;
            switch (status) {
                case 0b000:
                    if (*ecc_result <= YAFFS_ECC_RESULT_NO_ERROR)
                        *ecc_result = YAFFS_ECC_RESULT_NO_ERROR;
                    break;
                case 0b001:
                case 0b011:
                case 0b101:
                    if (*ecc_result <= YAFFS_ECC_RESULT_FIXED)
                        *ecc_result = YAFFS_ECC_RESULT_FIXED;
                    break;
                case 0b010:
                    printf("oob_len %d, status: %x\n", oob_len, status);
                    if (*ecc_result <= YAFFS_ECC_RESULT_UNFIXED)
                        *ecc_result = YAFFS_ECC_RESULT_UNFIXED;
                    break;
                default:
                    printf("status: %x\n", status);
                    if (*ecc_result <= YAFFS_ECC_RESULT_UNFIXED)
                        *ecc_result = YAFFS_ECC_RESULT_UNFIXED;
                    break;
            }
                    */
    }

    return YAFFS_OK;
}

int drv_erase_fn(struct yaffs_dev *dev, int block_no) {
    if (mt29f4g_erase_blocks(block_no, block_no) != STATUS_OK) {
        return YAFFS_FAIL;
    }

    return YAFFS_OK;
}

int drv_mark_bad_fn(struct yaffs_dev *dev, int block_no) {
    uint8_t buffer[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    if (mt29f4g_write_partial_page(buffer, block_no * MT29F4G_PAGE_PER_BLOCK,
                                   MT29F4G_SPARE_OFFSET, 8) != STATUS_OK) {
        return YAFFS_FAIL;
    }

    uint8_t status = mt29f4g_status();
    if ((status & (0x1 << 3))) {
        return YAFFS_FAIL;
    }

    return YAFFS_OK;
}

int drv_check_bad_fn(struct yaffs_dev *dev, int block_no) {
    uint8_t buffer[8];
    if (mt29f4g_read_within_page(buffer, block_no * MT29F4G_PAGE_PER_BLOCK,
                                 MT29F4G_SPARE_OFFSET, 8) != STATUS_OK) {
        return YAFFS_FAIL;
    }

    uint8_t status = mt29f4g_status();
    if ((status & (0x1 << 3))) {
        return YAFFS_FAIL;
    }

    if (buffer[0] == 0xAA && buffer[1] == 0xAA && buffer[2] == 0xAA &&
        buffer[3] == 0xAA && buffer[4] == 0xAA && buffer[5] == 0xAA &&
        buffer[6] == 0xAA && buffer[7] == 0xAA) {
        return YAFFS_FAIL;
    }

    return YAFFS_OK;
}

int drv_initialise_fn(struct yaffs_dev *dev) { return YAFFS_OK; }

int drv_deinitialise_fn(struct yaffs_dev *dev) { return YAFFS_OK; }