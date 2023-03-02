#ifndef MT29F2G_H
#define MT29F2G_H

#include "littlefs/lfs.h"
#include "status.h"
#include "stdint.h"
#include "stm32g4xx_hal.h"

typedef struct {
    const uint8_t op_code;
    const uint32_t n_addr;
    const uint32_t n_dummy;
    uint32_t n_data;
} MT29F2G_CmdTypeDef;

int mt29f2g_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                 void *buffer, lfs_size_t size);

int mt29f2g_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                 const void *buffer, lfs_size_t size);

int mt29f2g_erase(const struct lfs_config *c, lfs_block_t block);

int mt29f2g_sync(const struct lfs_config *c);

static const QSPI_CommandTypeDef mt29f2g_default_cmd = {
    .AddressMode = QSPI_ADDRESS_1_LINE,
    .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
    .DataMode = QSPI_DATA_4_LINES,
    .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
    .DdrMode = QSPI_DDR_MODE_DISABLE,
};

static const QSPI_AutoPollingTypeDef mt29f2g_default_auto = {
    .AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE,
    .MatchMode = QSPI_MATCH_MODE_AND,
    .StatusBytesSize = 1,
    .Interval = 8,
};

__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_RESET = {
    .op_code = 0xFF, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_GET_FEATURES = {
    .op_code = 0x0F, .n_addr = QSPI_ADDRESS_8_BITS, .n_dummy = 0, .n_data = 1};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_SET_FEATURES = {
    .op_code = 0x1F, .n_addr = QSPI_ADDRESS_8_BITS, .n_dummy = 0, .n_data = 1};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_ID = {
    .op_code = 0x9F, .n_addr = 0, .n_dummy = 8, .n_data = 2};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PAGE_READ = {
    .op_code = 0x13, .n_addr = QSPI_ADDRESS_24_BITS, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_PAGE_CACHE_RANDOM = {
    .op_code = 0x30, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_PAGE_CACHE_LAST = {
    .op_code = 0x3F, .n_addr = QSPI_ADDRESS_NONE, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X1 = {
    .op_code = 0x0B, .n_addr = QSPI_ADDRESS_16_BITS, .n_dummy = 8, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X2 = {
    .op_code = 0x3B, .n_addr = QSPI_ADDRESS_16_BITS, .n_dummy = 8, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X4 = {
    .op_code = 0x6B, .n_addr = QSPI_ADDRESS_16_BITS, .n_dummy = 8, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_DUAL_IO = {
    .op_code = 0xBB, .n_addr = QSPI_ADDRESS_16_BITS, .n_dummy = 8, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_QUAD_IO = {
    .op_code = 0xEB,
    .n_addr = QSPI_ADDRESS_16_BITS,
    .n_dummy = 16,
    .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_WRITE_ENABLE = {
    .op_code = 0x06, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_WRITE_DISABLE = {
    .op_code = 0x04, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_BLOCK_ERASE = {
    .op_code = 0xD8, .n_addr = QSPI_ADDRESS_24_BITS, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_EXECUTE = {
    .op_code = 0x10, .n_addr = QSPI_ADDRESS_24_BITS, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_X1 = {
    .op_code = 0x02, .n_addr = QSPI_ADDRESS_16_BITS, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_X4 = {
    .op_code = 0x32, .n_addr = QSPI_ADDRESS_16_BITS, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef
    MT29F2G_CMD_PROGRAM_LOAD_RANDOM_DATA_X1 = {.op_code = 0x84,
                                               .n_addr = QSPI_ADDRESS_16_BITS,
                                               .n_dummy = 0,
                                               .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef
    MT29F2G_CMD_PROGRAM_LOAD_RANDOM_DATA_X4 = {.op_code = 0x34,
                                               .n_addr = QSPI_ADDRESS_16_BITS,
                                               .n_dummy = 0,
                                               .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef
    MT29F2G_CMD_PERMANENT_BLOCK_LOCK_PROTECTION = {
        .op_code = 0x2C,
        .n_addr = QSPI_ADDRESS_24_BITS,
        .n_dummy = 0,
        .n_data = 0};

#define MT29F2G_FEATURE_STATUS 0xC0

#define MT29F2G_STATUS_OIP 0x01
#define MT29F2G_STATUS_CRBSY 0x80

#endif