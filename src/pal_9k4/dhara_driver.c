#include <stdio.h>

#include "dhara/nand.h"
#include "memory.h"
#include "mt29f4g.h"

const char *dhara_strerror(dhara_error_t err) {
    switch (err) {
        case DHARA_E_NONE:
            return "Dhara: No error";
        case DHARA_E_BAD_BLOCK:
            return "Dhara: Bad block";
        case DHARA_E_ECC:
            return "Dhara: ECC error";
        case DHARA_E_TOO_BAD:
            return "Dhara: Block too bad to recover";
        case DHARA_E_RECOVER:
            return "Dhara: Block recovered";
        case DHARA_E_JOURNAL_FULL:
            return "Dhara: Journal full";
        case DHARA_E_NOT_FOUND:
            return "Dhara: Not found";
        case DHARA_E_MAP_FULL:
            return "Dhara: Map full";
        case DHARA_E_CORRUPT_MAP:
            return "Dhara: Corrupt map";
        default:
            return "Dhara: Unknown error";
    }
}

int dhara_nand_is_bad(const struct dhara_nand *n, dhara_block_t b) {
    uint8_t mark = 0xFF;
    if (mt29f4g_read_within_page(&mark, b * MT29F4G_PAGE_PER_BLOCK,
                                 MT29F4G_METADATA_I_OFF, 1) != STATUS_OK) {
        return 1;
    }

    uint8_t ecc = (mt29f4g_status() >> MT29F4G_STATUS_ECC_SHIFT) &
                  MT29F4G_STATUS_ECC_MASK;
    if (ecc == 0b010) {
        return 1;
    }

    if (mark == 'X') {
        return 1;
    }

    return 0;
}

void dhara_nand_mark_bad(const struct dhara_nand *n, dhara_block_t b) {
    uint8_t mark = 'X';
    mt29f4g_write_partial_page(&mark, b * MT29F4G_PAGE_PER_BLOCK,
                               MT29F4G_METADATA_I_OFF, 1);
}

int dhara_nand_erase(const struct dhara_nand *n, dhara_block_t b,
                     dhara_error_t *err) {
    if (mt29f4g_erase_blocks(b, b) != STATUS_OK) {
        dhara_set_error(err, DHARA_E_BAD_BLOCK);
        return -1;
    }

    uint8_t efail = (mt29f4g_status() >> MT29F4G_STATUS_EFAIL) & 0x1;
    if (efail) {
        dhara_set_error(err, DHARA_E_BAD_BLOCK);
        return -1;
    }

    return 0;
}

int dhara_nand_prog(const struct dhara_nand *n, dhara_page_t p,
                    const uint8_t *data, dhara_error_t *err) {
    int partial_page_shift = MT29F4G_PAGE_SIZE_LOG2 - n->log2_page_size;
    int num_partial_pages = 1U << partial_page_shift;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
    if (mt29f4g_write_partial_page(
            data, p >> partial_page_shift,
            (p % num_partial_pages) * (1U << n->log2_page_size),
            1U << n->log2_page_size) != STATUS_OK) {
        dhara_set_error(err, DHARA_E_BAD_BLOCK);
        return -1;
    }
#pragma GCC diagnostic pop
    uint8_t pfail = (mt29f4g_status() >> MT29F4G_STATUS_PFAIL) & 0x1;
    if (pfail) {
        dhara_set_error(err, DHARA_E_BAD_BLOCK);
        return -1;
    }

    return 0;
}

int dhara_nand_is_free(const struct dhara_nand *n, dhara_page_t p) {
    int partial_page_shift = MT29F4G_PAGE_SIZE_LOG2 - n->log2_page_size;
    int num_partial_pages = 1U << partial_page_shift;
    uint8_t buffer[1U << n->log2_page_size];
    if (mt29f4g_read_within_page(
            buffer, p >> partial_page_shift,
            (p % num_partial_pages) * (1U << n->log2_page_size),
            1U << n->log2_page_size) != STATUS_OK) {
        return 0;
    }

    uint8_t ecc = (mt29f4g_status() >> MT29F4G_STATUS_ECC_SHIFT) &
                  MT29F4G_STATUS_ECC_MASK;
    if (ecc == 0b010) {
        return 0;
    }

    for (size_t i = 0; i < (1U << n->log2_page_size); i++) {
        if (buffer[i] != 0xFF) {
            return 0;
        }
    }

    return 1;
}

int dhara_nand_read(const struct dhara_nand *n, dhara_page_t p, size_t offset,
                    size_t length, uint8_t *data, dhara_error_t *err) {
    int partial_page_shift = MT29F4G_PAGE_SIZE_LOG2 - n->log2_page_size;
    int num_partial_pages = 1U << partial_page_shift;
    if (mt29f4g_read_within_page(
            data, p >> partial_page_shift,
            (p % num_partial_pages) * (1U << n->log2_page_size) + offset,
            length) != STATUS_OK) {
        return -1;
    }

    uint8_t ecc = (mt29f4g_status() >> MT29F4G_STATUS_ECC_SHIFT) &
                  MT29F4G_STATUS_ECC_MASK;
    if (ecc == 0b010) {
        dhara_set_error(err, DHARA_E_ECC);
        return -1;
    }

    return 0;
}

int dhara_nand_copy(const struct dhara_nand *n, dhara_page_t src,
                    dhara_page_t dst, dhara_error_t *err) {
    int partial_page_size = 1U << n->log2_page_size;
    uint8_t buffer[partial_page_size];
    int ret = dhara_nand_read(n, src, 0, partial_page_size, buffer, err);
    if (ret != 0) {
        return -1;
    }
    ret = dhara_nand_prog(n, dst, buffer, err);
    if (ret != 0) {
        return -1;
    }

    return 0;
}