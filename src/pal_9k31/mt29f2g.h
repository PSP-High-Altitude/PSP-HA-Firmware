#ifndef MT29F2G_H
#define MT29F2G_H

#include "littlefs/lfs.h"
#include "status.h"
#include "stdint.h"

typedef struct {
    const uint8_t op_code;
    const uint8_t n_addr;
    const uint8_t n_dummy;
    uint16_t n_data;
} MT29F2G_CmdTypeDef;

int8_t mt29f2g_read(const struct lfs_config *c, lfs_block_t block,
                    lfs_off_t off, void *buffer, lfs_size_t size);

int8_t mt29f2g_prog(const struct lfs_config *c, lfs_block_t block,
                    lfs_off_t off, const void *buffer, lfs_size_t size);

int8_t mt29f2g_erase(const struct lfs_config *c, lfs_block_t block);

int8_t mt29f2g_sync(const struct lfs_config *c);

__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_RESET = {
    .op_code = 0xFF, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_GET_FEATURES = {
    .op_code = 0x0F, .n_addr = 1, .n_dummy = 0, .n_data = 1};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_SET_FEATURES = {
    .op_code = 0x1F, .n_addr = 1, .n_dummy = 0, .n_data = 1};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_ID = {
    .op_code = 0x9F, .n_addr = 0, .n_dummy = 1, .n_data = 2};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PAGE_READ = {
    .op_code = 0x13, .n_addr = 3, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_PAGE_CACHE_RANDOM = {
    .op_code = 0x30, .n_addr = 3, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X1 = {
    .op_code = 0x0B, .n_addr = 2, .n_dummy = 1, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X2 = {
    .op_code = 0x3B, .n_addr = 2, .n_dummy = 1, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_X4 = {
    .op_code = 0x6B, .n_addr = 2, .n_dummy = 1, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_DUAL_IO = {
    .op_code = 0xBB, .n_addr = 2, .n_dummy = 1, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_READ_FROM_CACHE_QUAD_IO = {
    .op_code = 0xEB, .n_addr = 2, .n_dummy = 2, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_WRITE_ENABLE = {
    .op_code = 0x06, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_WRITE_DISABLE = {
    .op_code = 0x04, .n_addr = 0, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_BLOCK_ERASE = {
    .op_code = 0xD8, .n_addr = 3, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_EXECUTE = {
    .op_code = 0x10, .n_addr = 3, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_X1 = {
    .op_code = 0x02, .n_addr = 2, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_X4 = {
    .op_code = 0x32, .n_addr = 2, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_RANDOM_DATA_X1 =
    {.op_code = 0x84, .n_addr = 2, .n_dummy = 0, .n_data = 0};
__attribute__((
    unused)) static MT29F2G_CmdTypeDef MT29F2G_CMD_PROGRAM_LOAD_RANDOM_DATA_X4 =
    {.op_code = 0x34, .n_addr = 2, .n_dummy = 0, .n_data = 0};
__attribute__((unused)) static MT29F2G_CmdTypeDef
    MT29F2G_CMD_PERMANENT_BLOCK_LOCK_PROTECTION = {
        .op_code = 0x2C, .n_addr = 3, .n_dummy = 0, .n_data = 0};

#endif