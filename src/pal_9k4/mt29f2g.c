#include "mt29f2g.h"

#include "qspi/qspi.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "timer.h"

static QSpiDevice dev = {
    .bank = QSPI_BK1,
    .clk = QSPI_SPEED_20MHz,
};

static Status poll_oip() {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = MT29F2G_FEATURE_STATUS;
    cmd.AddressSize = MT29F2G_CMD_GET_FEATURES.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_GET_FEATURES.n_dummy;
    cmd.NbData = MT29F2G_CMD_GET_FEATURES.n_data;
    cmd.Instruction = MT29F2G_CMD_GET_FEATURES.op_code;
    cmd.DataMode = QSPI_DATA_1_LINE;
    QSPI_AutoPollingTypeDef auto_conf = mt29f2g_default_auto;
    auto_conf.Mask = MT29F2G_STATUS_OIP;
    auto_conf.Match = 0U;
    return qspi_auto_poll_cmd(&dev, &cmd, &auto_conf);
}

static Status poll_crbsy() {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = MT29F2G_FEATURE_STATUS;
    cmd.AddressSize = MT29F2G_CMD_GET_FEATURES.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_GET_FEATURES.n_dummy;
    cmd.NbData = MT29F2G_CMD_GET_FEATURES.n_data;
    cmd.Instruction = MT29F2G_CMD_GET_FEATURES.op_code;
    cmd.DataMode = QSPI_DATA_1_LINE;
    QSPI_AutoPollingTypeDef auto_conf = mt29f2g_default_auto;
    auto_conf.Mask = MT29F2G_STATUS_CRBSY;
    auto_conf.Match = 0U;
    return qspi_auto_poll_cmd(&dev, &cmd, &auto_conf);
}

static Status read_page(uint32_t row_addr) {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = row_addr;
    cmd.AddressSize = MT29F2G_CMD_PAGE_READ.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_PAGE_READ.n_dummy;
    cmd.NbData = MT29F2G_CMD_PAGE_READ.n_data;
    cmd.Instruction = MT29F2G_CMD_PAGE_READ.op_code;
    return qspi_cmd(&dev, &cmd);
}

static Status read_page_random(uint32_t row_addr) {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = row_addr;
    cmd.AddressSize = MT29F2G_CMD_READ_PAGE_CACHE_RANDOM.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_READ_PAGE_CACHE_RANDOM.n_dummy;
    cmd.NbData = MT29F2G_CMD_READ_PAGE_CACHE_RANDOM.n_data;
    cmd.Instruction = MT29F2G_CMD_READ_PAGE_CACHE_RANDOM.op_code;
    return qspi_cmd(&dev, &cmd);
}

static Status read_page_cache_last() {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.AddressSize = MT29F2G_CMD_READ_PAGE_CACHE_LAST.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_READ_PAGE_CACHE_LAST.n_dummy;
    cmd.NbData = MT29F2G_CMD_READ_PAGE_CACHE_LAST.n_data;
    cmd.Instruction = MT29F2G_CMD_READ_PAGE_CACHE_LAST.op_code;
    return qspi_cmd(&dev, &cmd);
}

static Status read_cache_x4(uint32_t col_addr, uint8_t plane, void *buffer,
                            uint32_t size) {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = (plane << 12) + col_addr;
    cmd.AddressSize = MT29F2G_CMD_READ_FROM_CACHE_X4.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_READ_FROM_CACHE_X4.n_dummy;
    cmd.NbData = MT29F2G_CMD_READ_FROM_CACHE_X4.n_data;
    cmd.Instruction = MT29F2G_CMD_READ_FROM_CACHE_X4.op_code;
    cmd.NbData = size;
    cmd.DataMode = QSPI_DATA_4_LINES;
    return (qspi_read(&dev, &cmd, (uint8_t *)buffer));
}

static Status write_enable() {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.AddressSize = MT29F2G_CMD_WRITE_ENABLE.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_WRITE_ENABLE.n_dummy;
    cmd.NbData = MT29F2G_CMD_WRITE_ENABLE.n_data;
    cmd.Instruction = MT29F2G_CMD_WRITE_ENABLE.op_code;
    return qspi_cmd(&dev, &cmd);
}

static Status reset() {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.AddressSize = MT29F2G_CMD_RESET.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_PAGE_READ.n_dummy;
    cmd.NbData = MT29F2G_CMD_RESET.n_data;
    cmd.Instruction = MT29F2G_CMD_RESET.op_code;
    return qspi_cmd(&dev, &cmd);
}

static Status program_load_x4(uint32_t col_addr, uint8_t plane,
                              const void *buffer, uint32_t size) {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = (plane << 12) + col_addr;
    cmd.AddressSize = MT29F2G_CMD_PROGRAM_LOAD_X4.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_PROGRAM_LOAD_X4.n_dummy;
    cmd.NbData = MT29F2G_CMD_PROGRAM_LOAD_X4.n_data;
    cmd.Instruction = MT29F2G_CMD_PROGRAM_LOAD_X4.op_code;
    cmd.NbData = size;
    cmd.DataMode = QSPI_DATA_4_LINES;
    return (qspi_write(&dev, &cmd, (uint8_t *)buffer));
}

static Status program_execute(uint32_t row_addr) {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = row_addr;
    cmd.AddressSize = MT29F2G_CMD_PROGRAM_EXECUTE.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_PROGRAM_EXECUTE.n_dummy;
    cmd.NbData = MT29F2G_CMD_PROGRAM_EXECUTE.n_data;
    cmd.Instruction = MT29F2G_CMD_PROGRAM_EXECUTE.op_code;
    return qspi_cmd(&dev, &cmd);
}

static Status block_erase(uint32_t row_addr) {
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = row_addr;
    cmd.AddressSize = MT29F2G_CMD_BLOCK_ERASE.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_BLOCK_ERASE.n_dummy;
    cmd.NbData = MT29F2G_CMD_BLOCK_ERASE.n_data;
    cmd.Instruction = MT29F2G_CMD_BLOCK_ERASE.op_code;
    return qspi_cmd(&dev, &cmd);
}

Status mt29f2g_init() {
    if (reset() != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (poll_oip() != STATUS_OK) {
        return STATUS_ERROR;
    }

    uint8_t tx_buf[] = {0x40};
    QSPI_CommandTypeDef cmd = mt29f2g_default_cmd;
    cmd.Address = 0xB0;
    cmd.AddressSize = MT29F2G_CMD_SET_FEATURES.n_addr;
    cmd.DummyCycles = MT29F2G_CMD_SET_FEATURES.n_dummy;
    cmd.NbData = MT29F2G_CMD_SET_FEATURES.n_data;
    cmd.Instruction = MT29F2G_CMD_SET_FEATURES.op_code;
    cmd.DataMode = QSPI_DATA_1_LINE;
    if (qspi_write(&dev, &cmd, tx_buf) != STATUS_OK) {
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
    cmd.DataMode = QSPI_DATA_1_LINE;
    if (qspi_write(&dev, &cmd, tx_buf) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (reset() != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (poll_oip() != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

int mt29f2g_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                 void *buffer, lfs_size_t size) {
    uint8_t num_pages = ((size + off) / 2176) - (off / 2176);
    if (num_pages > 2) {
        uint8_t page = (off / 2176);
        uint32_t row_addr = ((block / 2) << 6) + page;
        uint16_t col_addr = off % 2176;
        uint8_t plane = block % 2;
        uint32_t buf_offset = 0;
        if (read_page(row_addr) != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (poll_oip() != STATUS_OK) {
            return LFS_ERR_IO;
        }
        page++;
        row_addr++;
        while (page < (size + off) / 2176) {
            if (read_page_random(row_addr) != STATUS_OK) {
                return LFS_ERR_IO;
            }
            if (poll_oip() != STATUS_OK) {
                return LFS_ERR_IO;
            }
            if (read_cache_x4(col_addr, plane, buffer + buf_offset,
                              2176 - col_addr) != STATUS_OK) {
                return LFS_ERR_IO;
            }
            if (poll_crbsy() != STATUS_OK) {
                return LFS_ERR_IO;
            }
            buf_offset += 2176 - col_addr;
            col_addr = 0;
            page++;
            row_addr++;
        }
        if (read_page_cache_last() != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (poll_oip() != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (read_cache_x4(col_addr, plane, buffer + buf_offset,
                          (size + off) % 2176) != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (poll_crbsy() != STATUS_OK) {
            return LFS_ERR_IO;
        }
    } else {
        uint8_t page = (off / 2176);
        uint32_t row_addr = ((block / 2) << 6) + page;
        uint16_t col_addr = off % 2176;
        uint8_t plane = block % 2;
        uint32_t buf_offset = 0;
        while (page < (size + off) / 2176 - 1) {
            if (read_page(row_addr) != STATUS_OK) {
                return LFS_ERR_IO;
            }
            if (poll_oip() != STATUS_OK) {
                return LFS_ERR_IO;
            }
            if (read_cache_x4(col_addr, plane, buffer + buf_offset,
                              2176 - col_addr) != STATUS_OK) {
                return LFS_ERR_IO;
            }
            if (poll_crbsy() != STATUS_OK) {
                return LFS_ERR_IO;
            }
            buf_offset += 2176 - col_addr;
            col_addr = 0;
            page++;
            row_addr++;
        }
        if (read_page(row_addr) != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (poll_oip() != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (read_cache_x4(col_addr, plane, buffer + buf_offset,
                          (size + off) % 2176) != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (poll_crbsy() != STATUS_OK) {
            return LFS_ERR_IO;
        }
    }
    return 0;
}

int mt29f2g_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                 const void *buffer, lfs_size_t size) {
    uint8_t page = (off / 2176);
    uint32_t row_addr = ((block / 2) << 6) + page;
    uint16_t col_addr = off % 2176;
    uint8_t plane = block % 2;
    uint32_t buf_offset = 0;
    while (page < (size + off) / 2176 - 1) {
        if (write_enable() != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (program_load_x4(col_addr, plane, buffer + buf_offset,
                            2176 - col_addr) != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (program_execute(row_addr) != STATUS_OK) {
            return LFS_ERR_IO;
        }
        if (poll_oip() != STATUS_OK) {
            return LFS_ERR_IO;
        }
        buf_offset += 2176 - col_addr;
        col_addr = 0;
        page++;
        row_addr++;
    }
    if (write_enable() != STATUS_OK) {
        return LFS_ERR_IO;
    }
    if (program_load_x4(col_addr, plane, buffer + buf_offset,
                        (size + off) % 2176) != STATUS_OK) {
        return LFS_ERR_IO;
    }
    if (program_execute(row_addr) != STATUS_OK) {
        return LFS_ERR_IO;
    }
    if (poll_oip() != STATUS_OK) {
        return LFS_ERR_IO;
    }
    return 0;
}

int mt29f2g_erase(const struct lfs_config *c, lfs_block_t block) {
    uint32_t row_addr = ((block / 2) << 6);
    if (write_enable() != STATUS_OK) {
        return LFS_ERR_IO;
    }
    if (block_erase(row_addr) != STATUS_OK) {
        return LFS_ERR_IO;
    }
    if (poll_oip() != STATUS_OK) {
        return LFS_ERR_IO;
    }
    return 0;
}

int mt29f2g_sync(const struct lfs_config *c) { return 0; }