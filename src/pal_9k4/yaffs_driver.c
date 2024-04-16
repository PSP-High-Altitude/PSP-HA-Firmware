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

#define MT29F4G_PAGES_PER_VIRTUAL_PAGE 1

#define MIN(a, b) ((a) < (b) ? (a) : (b))

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
    dev->param.total_bytes_per_chunk =
        MT29F4G_PAGE_SIZE * MT29F4G_PAGES_PER_VIRTUAL_PAGE;
    dev->param.chunks_per_block =
        MT29F4G_PAGE_PER_BLOCK / MT29F4G_PAGES_PER_VIRTUAL_PAGE;
    dev->param.spare_bytes_per_chunk = MT29F4G_SPARE_SIZE / 2;
    dev->param.n_reserved_blocks = 5;
    dev->param.start_block = 0;
    dev->param.end_block = MT29F4G_BLOCK_COUNT - 1;
    dev->param.n_caches = 0;
    dev->param.cache_bypass_aligned = 0;
    dev->param.use_nand_ecc = 0;
    dev->param.no_tags_ecc = 0;
    dev->param.is_yaffs2 = 1;
    dev->param.wide_tnodes_disabled = 0;

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
    uint32_t row_addr = nand_chunk * MT29F4G_PAGES_PER_VIRTUAL_PAGE;
    uint8_t status;

    // Write data and tags
    for (int i = 0; i < MT29F4G_PAGES_PER_VIRTUAL_PAGE; i++) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
        if (mt29f4g_write_page_and_spare(data, MIN(data_len, MT29F4G_PAGE_SIZE),
                                         oob, MIN(oob_len, MT29F4G_SPARE_SIZE),
                                         row_addr + i) != STATUS_OK) {
            return YAFFS_FAIL;
        }
#pragma GCC diagnostic pop
        status = mt29f4g_status();
        if ((status & (0x1 << 3))) {
            return YAFFS_FAIL;
        }

        data += MIN(data_len, MT29F4G_PAGE_SIZE);
        oob = NULL;
        data_len -= MT29F4G_PAGE_SIZE;
        if (data_len <= 0) {
            break;
        }
    }

    return YAFFS_OK;
}

int drv_read_chunk_fn(struct yaffs_dev *dev, int nand_chunk, u8 *data,
                      int data_len, u8 *oob, int oob_len,
                      enum yaffs_ecc_result *ecc_result) {
    uint32_t row_addr = nand_chunk * MT29F4G_PAGES_PER_VIRTUAL_PAGE;
    uint32_t col_addr = 0;
    // uint8_t status;

    *ecc_result = YAFFS_ECC_RESULT_NO_ERROR;

    for (int i = 0; i < MT29F4G_PAGES_PER_VIRTUAL_PAGE; i++) {
        //  Read data
        if (data && data_len > 0) {
            col_addr = 0;
            if (mt29f4g_read_within_page(data, row_addr + i, col_addr,
                                         MIN(data_len, MT29F4G_PAGE_SIZE)) !=
                STATUS_OK) {
                return YAFFS_FAIL;
            }
        }

        // Read oob
        if (oob && oob_len > 0) {
            col_addr = MT29F4G_SPARE_OFFSET;
            if (mt29f4g_read_within_page(oob, row_addr + i, col_addr,
                                         MIN(oob_len, MT29F4G_SPARE_SIZE)) !=
                STATUS_OK) {
                return YAFFS_FAIL;
            }
        }

        data += MIN(data_len, MT29F4G_PAGE_SIZE);
        oob = NULL;
        data_len -= MT29F4G_PAGE_SIZE;
        if (data_len <= 0) {
            break;
        }
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
    uint8_t val = 'Y';
    if (mt29f4g_write_partial_page(&val, block_no * MT29F4G_PAGE_PER_BLOCK,
                                   MT29F4G_SPARE_OFFSET + 5, 1) != STATUS_OK) {
        return YAFFS_FAIL;
    }

    uint8_t status = mt29f4g_status();
    if ((status & (0x1 << 3))) {
        return YAFFS_FAIL;
    }

    return YAFFS_OK;
}

int drv_check_bad_fn(struct yaffs_dev *dev, int block_no) {
    uint8_t val;
    if (mt29f4g_read_within_page(&val, block_no * MT29F4G_PAGE_PER_BLOCK,
                                 MT29F4G_SPARE_OFFSET + 5, 1) != STATUS_OK) {
        return YAFFS_FAIL;
    }

    if (val == 'Y') {
        return YAFFS_FAIL;
    }

    return YAFFS_OK;
}

int drv_initialise_fn(struct yaffs_dev *dev) { return YAFFS_OK; }

int drv_deinitialise_fn(struct yaffs_dev *dev) { return YAFFS_OK; }