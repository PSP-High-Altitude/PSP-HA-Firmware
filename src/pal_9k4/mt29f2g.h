#ifndef MT29F2G_H
#define MT29F2G_H

#include "littlefs/lfs.h"
#include "ospi.h"
#include "status.h"
#include "stdint.h"
#include "stm32h7xx_hal.h"

typedef struct {
    const uint8_t op_code;
    const uint32_t n_addr;
    const uint32_t n_dummy;
    uint32_t n_data;
} MT29F2G_CmdTypeDef;

Status mt29f2g_init();

Status mt29f2g_read_pages(uint8_t *buffer, uint32_t page, uint32_t num_pages);

Status mt29f2g_write_pages(uint8_t *buffer, uint32_t page, uint32_t num_pages);

Status mt29f2g_erase_blocks(uint32_t block_start, uint32_t block_end);

Status mt29f2g_sync();

struct lfs_config *mt29f2g_get_lfs_config();

int lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
             void *buffer, lfs_size_t size);

int lfs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
             const void *buffer, lfs_size_t size);

int lfs_erase(const struct lfs_config *c, lfs_block_t block);

int lfs_sync(const struct lfs_config *c);

static const OSPI_RegularCmdTypeDef mt29f2g_default_cmd = {
    .OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
    .FlashId = HAL_OSPI_FLASH_ID_1,
    .InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
    .DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE,
    .AddressMode = HAL_OSPI_ADDRESS_1_LINE,
    .AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
    .AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS,
    .AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE,
    .DataMode = HAL_OSPI_DATA_1_LINE,
    .InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
    .SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
    .InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
    .AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,
    .DQSMode = HAL_OSPI_DQS_DISABLE,
};

static const OSPI_AutoPollingTypeDef mt29f2g_default_auto = {
    .AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE,
    .MatchMode = HAL_OSPI_MATCH_MODE_AND,
    .Interval = 0x10,
};

__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_RESET = {
    .op_code = 0xFF, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_GET_FEATURES = {
    .op_code = 0x0F,
    .n_addr = HAL_OSPI_ADDRESS_8_BITS,
    .n_dummy = 0,
    .n_data = 1};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_SET_FEATURES = {
    .op_code = 0x1F,
    .n_addr = HAL_OSPI_ADDRESS_8_BITS,
    .n_dummy = 0,
    .n_data = 1};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_ID = {
    .op_code = 0x9F, .n_addr = 0, .n_dummy = 8, .n_data = 2};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PAGE_READ = {
    .op_code = 0x13,
    .n_addr = HAL_OSPI_ADDRESS_24_BITS,
    .n_dummy = 0,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_PAGE_CACHE_RANDOM = {
    .op_code = 0x30, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_PAGE_CACHE_LAST = {
    .op_code = 0x3F,
    .n_addr = HAL_OSPI_ADDRESS_NONE,
    .n_dummy = 0,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X1 = {
    .op_code = 0x0B,
    .n_addr = HAL_OSPI_ADDRESS_16_BITS,
    .n_dummy = 8,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X2 = {
    .op_code = 0x3B,
    .n_addr = HAL_OSPI_ADDRESS_16_BITS,
    .n_dummy = 8,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X4 = {
    .op_code = 0x6B,
    .n_addr = HAL_OSPI_ADDRESS_16_BITS,
    .n_dummy = 8,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_DUAL_IO = {
    .op_code = 0xBB,
    .n_addr = HAL_OSPI_ADDRESS_16_BITS,
    .n_dummy = 8,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_QUAD_IO = {
    .op_code = 0xEB,
    .n_addr = HAL_OSPI_ADDRESS_16_BITS,
    .n_dummy = 16,
    .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_WRITE_ENABLE = {
    .op_code = 0x06, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_WRITE_DISABLE = {
    .op_code = 0x04, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_BLOCK_ERASE = {
    .op_code = 0xD8,
    .n_addr = HAL_OSPI_ADDRESS_24_BITS,
    .n_dummy = 0,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_EXECUTE = {
    .op_code = 0x10,
    .n_addr = HAL_OSPI_ADDRESS_24_BITS,
    .n_dummy = 0,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_X1 = {
    .op_code = 0x02,
    .n_addr = HAL_OSPI_ADDRESS_16_BITS,
    .n_dummy = 0,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_X4 = {
    .op_code = 0x32,
    .n_addr = HAL_OSPI_ADDRESS_16_BITS,
    .n_dummy = 0,
    .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_RANDOM_DATA_X1 =
    {.op_code = 0x84,
     .n_addr = HAL_OSPI_ADDRESS_16_BITS,
     .n_dummy = 0,
     .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_RANDOM_DATA_X4 =
    {.op_code = 0x34,
     .n_addr = HAL_OSPI_ADDRESS_16_BITS,
     .n_dummy = 0,
     .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef
    MT29F2G_CMD_PERMANENT_BLOCK_LOCK_PROTECTION = {
        .op_code = 0x2C,
        .n_addr = HAL_OSPI_ADDRESS_24_BITS,
        .n_dummy = 0,
        .n_data = 0};

#define MT29F2G_FEATURE_STATUS 0xC0

#define MT29F2G_STATUS_OIP 0x01
#define MT29F2G_STATUS_CRBSY 0x80

#endif