#ifndef MT29F4G_H
#define MT29F4G_H

#include "ospi.h"
#include "status.h"
#include "stdint.h"

#define MT29F4G_PAGE_SIZE 4096
#define MT29F4G_PAGE_SIZE_LOG2 12
#define MT29F4G_SPARE_SIZE 256
#define MT29F4G_SPARE_OFFSET 0x1000
#define MT29F4G_METADATA_I_OFF 0x1040
#define MT29F4G_METADATA_I_SIZE 64
#define MT29F4G_PAGE_PER_BLOCK 64
#define MT29F4G_PAGE_PER_BLOCK_LOG2 6
#define MT29F4G_BLOCK_COUNT 2048
#define MT29F4G_BLOCK_COUNT_LOG2 11
#define MT29F4G_STATUS_ECC_SHIFT 4
#define MT29F4G_STATUS_ECC_MASK 0x07
#define MT29F4G_STATUS_PFAIL 3
#define MT29F4G_STATUS_EFAIL 2

Status mt29f4g_init();

Status mt29f4g_read_pages(uint8_t *buffer, uint32_t page, uint32_t num_pages);

Status mt29f4g_read_within_page(uint8_t *buffer, uint32_t page, uint32_t offset,
                                uint32_t size);

Status mt29f4g_read_page_spare(uint8_t *buffer, uint32_t page);

Status mt29f4g_write_pages(uint8_t *buffer, uint32_t page, uint32_t num_pages);

Status mt29f4g_write_page_and_spare(uint8_t *buffer, uint32_t data_len,
                                    uint8_t *spare, uint32_t spare_len,
                                    uint32_t page);

// Only 4 partial programs are allowed per page
Status mt29f4g_write_partial_page(uint8_t *buffer, uint32_t page,
                                  uint32_t offset, uint32_t size);

Status mt29f4g_erase_blocks(uint32_t block_start, uint32_t block_end);

Status mt29f4g_erase_chip();

Status mt29f4g_sync();

uint8_t mt29f4g_status();

typedef struct {
    const uint8_t op_code;
    const uint32_t n_addr;
    const uint32_t n_dummy;
    uint32_t n_data;
} MT29F4G_CmdTypeDef;

static const OSpiCommand mt29f4g_default_cmd = {
    .addr_mode = OSPI_ADDR_1_LINE,
    .data_mode = OSPI_DATA_1_LINE,
    .inst_mode = OSPI_INST_1_LINE,
};

static const OSpiAutopoll mt29f4g_default_auto = {
    .match_mode = OSPI_MATCH_AND,
    .interval = 0,
};

__attribute__((unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_RESET = {
    .op_code = 0xFF,
    .n_addr = 0,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_GET_FEATURES = {
    .op_code = 0x0F,
    .n_addr = OSPI_ADDR_1_BYTE,
    .n_dummy = 0,
    .n_data = 1,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_SET_FEATURES = {
    .op_code = 0x1F,
    .n_addr = OSPI_ADDR_1_BYTE,
    .n_dummy = 0,
    .n_data = 1,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_READ_ID = {
    .op_code = 0x9F,
    .n_addr = 0,
    .n_dummy = 8,
    .n_data = 2,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_PAGE_READ = {
    .op_code = 0x13,
    .n_addr = OSPI_ADDR_3_BYTES,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_READ_PAGE_CACHE_RANDOM = {
    .op_code = 0x30,
    .n_addr = 0,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_READ_PAGE_CACHE_LAST = {
    .op_code = 0x3F,
    .n_addr = OSPI_ADDR_NONE,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_READ_FROM_CACHE_X1 = {
    .op_code = 0x0B,
    .n_addr = OSPI_ADDR_2_BYTES,
    .n_dummy = 8,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_READ_FROM_CACHE_X2 = {
    .op_code = 0x3B,
    .n_addr = OSPI_ADDR_2_BYTES,
    .n_dummy = 8,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_READ_FROM_CACHE_X4 = {
    .op_code = 0x6B,
    .n_addr = OSPI_ADDR_2_BYTES,
    .n_dummy = 8,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_READ_FROM_CACHE_DUAL_IO = {
    .op_code = 0xBB,
    .n_addr = OSPI_ADDR_2_BYTES,
    .n_dummy = 8,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_READ_FROM_CACHE_QUAD_IO = {
    .op_code = 0xEB,
    .n_addr = OSPI_ADDR_2_BYTES,
    .n_dummy = 16,
    .n_data = 0,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_WRITE_ENABLE = {
    .op_code = 0x06,
    .n_addr = 0,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_WRITE_DISABLE = {
    .op_code = 0x04,
    .n_addr = 0,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_BLOCK_ERASE = {
    .op_code = 0xD8,
    .n_addr = OSPI_ADDR_3_BYTES,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_PROGRAM_EXECUTE = {
    .op_code = 0x10,
    .n_addr = OSPI_ADDR_3_BYTES,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_PROGRAM_LOAD_X1 = {
    .op_code = 0x02,
    .n_addr = OSPI_ADDR_2_BYTES,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((
    unused)) static MT29F4G_CmdTypeDef MT29F4G_CMD_PROGRAM_LOAD_X4 = {
    .op_code = 0x32,
    .n_addr = OSPI_ADDR_2_BYTES,
    .n_dummy = 0,
    .n_data = 0,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef
    MT29F4G_CMD_PROGRAM_LOAD_RANDOM_DATA_X1 = {
        .op_code = 0x84,
        .n_addr = OSPI_ADDR_2_BYTES,
        .n_dummy = 0,
        .n_data = 0,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef
    MT29F4G_CMD_PROGRAM_LOAD_RANDOM_DATA_X4 = {
        .op_code = 0x34,
        .n_addr = OSPI_ADDR_2_BYTES,
        .n_dummy = 0,
        .n_data = 0,
};
__attribute__((unused)) static MT29F4G_CmdTypeDef
    MT29F4G_CMD_PERMANENT_BLOCK_LOCK_PROTECTION = {
        .op_code = 0x2C,
        .n_addr = OSPI_ADDR_3_BYTES,
        .n_dummy = 0,
        .n_data = 0,
};

#define MT29F4G_FEATURE_STATUS 0xC0

#define MT29F4G_STATUS_OIP 0x01
#define MT29F4G_STATUS_CRBSY 0x80

#endif