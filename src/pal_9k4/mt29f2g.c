#include "mt29f2g.h"

#include "ospi.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

static OSpiDevice dev = {
    .bank = OSPI_PORT1_7_4,
    .clk = OSPI_SPEED_20MHz,
};
static uint8_t chip_ready = 0;

static Status poll_oip() {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = MT29F2G_FEATURE_STATUS;
    cmd.AddressSize = MT29F2G_CMD_GET_FEATURES.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_GET_FEATURES.n_dummy;
    cmd.NbData = MT29F2G_CMD_GET_FEATURES.n_data;
    cmd.Instruction = MT29F2G_CMD_GET_FEATURES.op_code;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    OSPI_AutoPollingTypeDef auto_conf = mt29f2g_default_auto;
    auto_conf.Mask = MT29F2G_STATUS_OIP;
    auto_conf.Match = 0U;
    return ospi_auto_poll_cmd(&dev, &cmd, &auto_conf);
}

/*
static Status poll_crbsy() {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = MT29F2G_FEATURE_STATUS;
    cmd.AddressSize = MT29F2G_CMD_GET_FEATURES.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_GET_FEATURES.n_dummy;
    cmd.NbData = MT29F2G_CMD_GET_FEATURES.n_data;
    cmd.Instruction = MT29F2G_CMD_GET_FEATURES.op_code;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    OSPI_AutoPollingTypeDef auto_conf = mt29f2g_default_auto;
    auto_conf.Mask = MT29F2G_STATUS_CRBSY;
    auto_conf.Match = 0U;
    return ospi_auto_poll_cmd(&dev, &cmd, &auto_conf);
}
*/

static Status read_page(uint32_t row_addr) {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = row_addr;
    cmd.AddressSize = MT29F2G_CMD_PAGE_READ.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_PAGE_READ.n_dummy;
    cmd.NbData = MT29F2G_CMD_PAGE_READ.n_data;
    cmd.Instruction = MT29F2G_CMD_PAGE_READ.op_code;
    cmd.DataMode = HAL_OSPI_DATA_NONE;
    return ospi_cmd(&dev, &cmd);
}

/*
static Status read_page_random(uint32_t row_addr) {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = row_addr;
    cmd.AddressSize = MT29F2G_CMD_READ_PAGE_CACHE_RANDOM.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_READ_PAGE_CACHE_RANDOM.n_dummy;
    cmd.NbData = MT29F2G_CMD_READ_PAGE_CACHE_RANDOM.n_data;
    cmd.Instruction = MT29F2G_CMD_READ_PAGE_CACHE_RANDOM.op_code;
    return ospi_cmd(&dev, &cmd);
}
*/

/*
static Status read_page_cache_last() {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.AddressSize = MT29F2G_CMD_READ_PAGE_CACHE_LAST.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_READ_PAGE_CACHE_LAST.n_dummy;
    cmd.NbData = MT29F2G_CMD_READ_PAGE_CACHE_LAST.n_data;
    cmd.Instruction = MT29F2G_CMD_READ_PAGE_CACHE_LAST.op_code;
    return ospi_cmd(&dev, &cmd);
}
*/

static Status read_cache_x4(uint32_t col_addr, uint32_t plane, uint8_t *buffer,
                            uint32_t size) {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = (plane << 12) + col_addr;
    cmd.AddressSize = MT29F2G_CMD_READ_FROM_CACHE_X4.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_READ_FROM_CACHE_X4.n_dummy;
    cmd.Instruction = MT29F2G_CMD_READ_FROM_CACHE_X4.op_code;
    cmd.NbData = size;
    cmd.DataMode = HAL_OSPI_DATA_4_LINES;
    return (ospi_read(&dev, &cmd, buffer));
}

static Status write_enable() {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.AddressSize = MT29F2G_CMD_WRITE_ENABLE.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_WRITE_ENABLE.n_dummy;
    cmd.NbData = MT29F2G_CMD_WRITE_ENABLE.n_data;
    cmd.Instruction = MT29F2G_CMD_WRITE_ENABLE.op_code;
    cmd.DataMode = HAL_OSPI_DATA_NONE;
    return ospi_cmd(&dev, &cmd);
}

static Status reset() {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.AddressSize = MT29F2G_CMD_RESET.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_RESET.n_dummy;
    cmd.NbData = MT29F2G_CMD_RESET.n_data;
    cmd.Instruction = MT29F2G_CMD_RESET.op_code;
    cmd.DataMode = HAL_OSPI_DATA_NONE;
    return ospi_cmd(&dev, &cmd);
}

static Status program_load_x4(uint32_t col_addr, uint32_t plane,
                              uint8_t *buffer, uint32_t size) {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = (plane << 12) + col_addr;
    cmd.AddressSize = MT29F2G_CMD_PROGRAM_LOAD_X4.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_PROGRAM_LOAD_X4.n_dummy;
    cmd.Instruction = MT29F2G_CMD_PROGRAM_LOAD_X4.op_code;
    cmd.NbData = size;
    cmd.DataMode = HAL_OSPI_DATA_4_LINES;
    return (ospi_write(&dev, &cmd, buffer));
}

static Status program_execute(uint32_t row_addr) {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = row_addr;
    cmd.AddressSize = MT29F2G_CMD_PROGRAM_EXECUTE.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_PROGRAM_EXECUTE.n_dummy;
    cmd.NbData = MT29F2G_CMD_PROGRAM_EXECUTE.n_data;
    cmd.Instruction = MT29F2G_CMD_PROGRAM_EXECUTE.op_code;
    cmd.DataMode = HAL_OSPI_DATA_NONE;
    return ospi_cmd(&dev, &cmd);
}

static Status block_erase(uint32_t block_addr) {
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = block_addr;
    cmd.AddressSize = MT29F2G_CMD_BLOCK_ERASE.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_BLOCK_ERASE.n_dummy;
    cmd.NbData = MT29F2G_CMD_BLOCK_ERASE.n_data;
    cmd.Instruction = MT29F2G_CMD_BLOCK_ERASE.op_code;
    cmd.DataMode = HAL_OSPI_DATA_NONE;
    return ospi_cmd(&dev, &cmd);
}

Status mt29f2g_init() {
    if (chip_ready) {
        return STATUS_OK;
    }
    if (reset() != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (poll_oip() != STATUS_OK) {
        return STATUS_ERROR;
    }

    uint8_t tx_buf[] = {0x40};
    OSPI_RegularCmdTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = 0xB0;
    cmd.AddressSize = MT29F2G_CMD_SET_FEATURES.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_SET_FEATURES.n_dummy;
    cmd.NbData = MT29F2G_CMD_SET_FEATURES.n_data;
    cmd.Instruction = MT29F2G_CMD_SET_FEATURES.op_code;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    if (ospi_write(&dev, &cmd, tx_buf) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (read_page(0x1) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (poll_oip() != STATUS_OK) {
        return STATUS_ERROR;
    }

    uint8_t rx_buf[4];
    if (read_cache_x4(0x0, 0x0, rx_buf, 4) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (rx_buf[0] != 0x4F || rx_buf[1] != 0x4E || rx_buf[2] != 0x46 ||
        rx_buf[3] != 0x49) {
        return STATUS_ERROR;
    }

    tx_buf[0] = 0x00;
    cmd = mt29f2g_default_cmd;
    cmd.Address = 0xB0;
    cmd.AddressSize = MT29F2G_CMD_SET_FEATURES.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_SET_FEATURES.n_dummy;
    cmd.NbData = MT29F2G_CMD_SET_FEATURES.n_data;
    cmd.Instruction = MT29F2G_CMD_SET_FEATURES.op_code;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    if (ospi_write(&dev, &cmd, tx_buf) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (reset() != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (poll_oip() != STATUS_OK) {
        return STATUS_ERROR;
    }

    chip_ready = 1;
    return STATUS_OK;
}

Status mt29f2g_read_pages(uint8_t *buffer, uint32_t page, uint32_t num_pages) {
    for (int i = 0; i < num_pages; i++) {
        if (read_page(page + i) != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (poll_oip() != STATUS_OK) {
            return STATUS_ERROR;
        }
        uint8_t plane =
            ((page + i) >> 6) & 0x1;  // plane is the LSB of the block address
        if (read_cache_x4(0, plane, buffer, 2048) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }
    return STATUS_OK;
}
Status mt29f2g_write_pages(uint8_t *buffer, uint32_t page, uint32_t num_pages) {
    for (int i = 0; i < num_pages; i++) {
        if (write_enable() != STATUS_OK) {
            return STATUS_ERROR;
        }
        uint8_t plane =
            ((page + i) >> 6) & 0x1;  // plane is the LSB of the block address
        if (program_load_x4(0, plane, buffer, 2048) != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (program_execute(page + i) != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (poll_oip() != STATUS_OK) {
            return STATUS_ERROR;
        }
    }
    return STATUS_OK;
}

Status mt29f2g_erase_blocks(uint32_t block_start, uint32_t block_end) {
    uint32_t block_addr_curr = block_start >> 6;
    uint32_t block_addr_end = block_end >> 6;
    while (block_addr_curr <= block_addr_end) {
        if (write_enable() != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (block_erase(block_addr_curr) != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (poll_oip() != STATUS_OK) {
            return STATUS_ERROR;
        }
        block_addr_curr++;
    }
    return STATUS_OK;
}

Status mt29f2g_sync() {
    if (poll_oip() != STATUS_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

int lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
             void *buffer, lfs_size_t size) {
    uint32_t row_addr = (block << 6) + (off / 2048);
    if (mt29f2g_read_pages(buffer, row_addr, size / 2048) != STATUS_OK) {
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

int lfs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
             const void *buffer, lfs_size_t size) {
    uint32_t row_addr = (block << 6) + (off / 2048);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
    if (mt29f2g_write_pages(buffer, row_addr, size / 2048) != STATUS_OK) {
        return LFS_ERR_IO;
    }
#pragma GCC diagnostic pop
    return LFS_ERR_OK;
}

int lfs_erase(const struct lfs_config *c, lfs_block_t block) {
    if (mt29f2g_erase_blocks(block, block) != STATUS_OK) {
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

int lfs_sync(const struct lfs_config *c) {
    if (mt29f2g_sync() != STATUS_OK) {
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

struct lfs_config *mt29f2g_get_lfs_config() {
    static struct lfs_config lfs_cfg;
    lfs_cfg.read = lfs_read;
    lfs_cfg.prog = lfs_prog;
    lfs_cfg.erase = lfs_erase;
    lfs_cfg.sync = lfs_sync;
    lfs_cfg.read_size = 2048;
    lfs_cfg.prog_size = 2048;
    lfs_cfg.block_size = 131072;
    lfs_cfg.block_count = 2048;
    lfs_cfg.cache_size = 2048;
    lfs_cfg.lookahead_size = 2048;
    lfs_cfg.block_cycles = 500;

    return &lfs_cfg;
}