/*------------------------------------------------------------------------*/
/*
/  Copyright (C) 2018, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

#include "fatfs/diskio.h"

#include "FreeRTOS.h"
#include "dhara/map.h"
#include "gpio/gpio.h"
#include "nand/mt29f4g.h"
#include "rtc/rtc.h"
#include "sdmmc/sdmmc.h"
#include "string.h"
#include "task.h"
#include "timer.h"

#ifndef FF_VOLUME_STRS
const char *VolumeStr[FF_VOLUMES] = {"MMC, NAND"};
#endif

RAM_D2 static uint8_t rw_buf[512];  // DMA happy buffer

/* MMC/SD command */
#define CMD0 (0)           /* GO_IDLE_STATE */
#define CMD1 (1)           /* SEND_OP_COND (MMC) */
#define ACMD41 (0x80 + 41) /* SEND_OP_COND (SDC) */
#define CMD8 (8)           /* SEND_IF_COND */
#define CMD9 (9)           /* SEND_CSD */
#define CMD10 (10)         /* SEND_CID */
#define CMD12 (12)         /* STOP_TRANSMISSION */
#define ACMD13 (0x80 + 13) /* SD_STATUS (SDC) */
#define CMD16 (16)         /* SET_BLOCKLEN */
#define CMD17 (17)         /* READ_SINGLE_BLOCK */
#define CMD18 (18)         /* READ_MULTIPLE_BLOCK */
#define CMD23 (23)         /* SET_BLOCK_COUNT (MMC) */
#define ACMD23 (0x80 + 23) /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24 (24)         /* WRITE_BLOCK */
#define CMD25 (25)         /* WRITE_MULTIPLE_BLOCK */
#define CMD32 (32)         /* ERASE_ER_BLK_START */
#define CMD33 (33)         /* ERASE_ER_BLK_END */
#define CMD38 (38)         /* ERASE */
#define CMD55 (55)         /* APP_CMD */
#define CMD58 (58)         /* READ_OCR */

static volatile DSTATUS Stat[FF_VOLUMES] = {
    STA_NOINIT, STA_NOINIT}; /* Physical drive status */

#define SD

/*-----------------------------------------------------------------------*/
/* CRC functions (from https://github.com/hazelnusse/crc7)               */
/*-----------------------------------------------------------------------*/

static uint8_t s_crc_table[256];

void generate_crc_table() {
    int i, j;
    uint8_t CRCPoly = 0x89;  // the value of our CRC-7 polynomial

    // generate a table value for all 256 possible byte values
    for (i = 0; i < 256; ++i) {
        s_crc_table[i] = (i & 0x80) ? i ^ CRCPoly : i;
        for (j = 1; j < 8; ++j) {
            s_crc_table[i] <<= 1;
            if (s_crc_table[i] & 0x80) s_crc_table[i] ^= CRCPoly;
        }
    }
}

// adds a message byte to the current CRC-7 to get a the new CRC-7
uint8_t crc_add(uint8_t crc, uint8_t message_byte) {
    return s_crc_table[(crc << 1) ^ message_byte];
}

// returns the CRC-7 for a message of "length" bytes
uint8_t get_crc(uint8_t message[], int length) {
    int i;
    uint8_t crc = 0;

    for (i = 0; i < length; ++i) {
        crc = crc_add(crc, message[i]);
    }

    return crc;
}

// returns the CRC-7 for a message of "length" bytes
uint8_t get_cmd_crc(BYTE cmd, DWORD arg) {
    uint8_t crc = 0;
    crc = crc_add(crc, cmd | 0x40);        /* Start bit + CMD */
    crc = crc_add(crc, (BYTE)(arg >> 24)); /* Argument[31..24] */
    crc = crc_add(crc, (BYTE)(arg >> 16)); /* Argument[23..16] */
    crc = crc_add(crc, (BYTE)(arg >> 8));  /* Argument[15..8] */
    crc = crc_add(crc, (BYTE)arg);         /* Argument[7..0] */
    return (crc << 1) | 0x01;              /* Set stop bit */
}

/*-----------------------------------------------------------------------*/
/* Peripheral controls (Platform dependent)                              */
/*-----------------------------------------------------------------------*/
struct dhara_map s_map;
struct dhara_nand s_nand;
static dhara_error_t s_map_error = DHARA_E_NONE;
static uint8_t s_map_buffer[1U << MT29F4G_PAGE_SIZE_LOG2];

static SdmmcDevice s_sd_sdmmc_device;

/* Initialize MMC interface */
Status diskio_init(SdmmcDevice *device) {
    generate_crc_table();

    // Create a local copy of MMC device (actual setup will be done later)
    s_sd_sdmmc_device.periph = device->periph;
    s_sd_sdmmc_device.clk = device->clk;

    // Initialize FTL for NAND
    memset(&s_map, 0, sizeof(s_map));
    memset(&s_nand, 0, sizeof(s_nand));
    memset(&s_map_buffer, 0, sizeof(s_map_buffer));
    s_nand.log2_page_size = MT29F4G_PAGE_SIZE_LOG2;
    s_nand.log2_ppb = MT29F4G_PAGE_PER_BLOCK_LOG2;
    s_nand.num_blocks = MT29F4G_BLOCK_COUNT;

    return STATUS_OK;
}

DWORD get_fattime() {
    RTCDateTime datetime = rtc_get_datetime();

    return (DWORD)(datetime.year - 1980) << 25 |  // Year
           (DWORD)(datetime.month) << 21 |        // Month
           (DWORD)(datetime.day) << 16 |          // Day
           (DWORD)(datetime.hour) << 11 |         // Hour
           (DWORD)(datetime.minute) << 5 |        // Minute
           (DWORD)(datetime.second) >> 1;         // Second
}

/*-----------------------------------------------------------------------*/
/* Check status with timeout                                             */
/*-----------------------------------------------------------------------*/
static Status sdmmc_check_status(uint64_t timeout) {
    SdmmcState res;
    uint64_t start_time = xTaskGetTickCount();
    do {
        res = sdmmc_status(&s_sd_sdmmc_device);
        // Yield until timeout
        DELAY(0);
    } while ((res != SD_CARD_TRANSFER) &&
             (xTaskGetTickCount() - start_time < timeout));
    return res == SD_CARD_TRANSFER ? STATUS_OK : STATUS_ERROR;
}

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE drv /* Physical drive number (0) */
) {
    if (drv == 0) {
        if (Stat[0] == STA_NOINIT) {
            // Initialize the SDMMC device
            if (Stat[0] & STA_NODISK)
                return Stat[0]; /* Is card existing in the soket? */

            if (sdmmc_setup(&s_sd_sdmmc_device) != STATUS_OK) {
                PAL_LOGE("Failed to initialize MMC hardware\n");
                Stat[0] = STA_NOINIT;
                return Stat[0];
            }
            Stat[0] &= ~STA_NOINIT; /* Clear STA_NOINIT flag */
            return Stat[0];
        }
    } else if (drv == 1) {
        if ((Stat[1] & STA_NOINIT) == 0)
            return Stat[1]; /* Check if drive is ready */

        if (mt29f4g_init() != STATUS_OK) {
            PAL_LOGE("Failed to initialize NAND hardware\n");
            Stat[1] = STA_NOINIT;
            return Stat[1];
        }

        dhara_map_init(&s_map, &s_nand, s_map_buffer, 4);
        dhara_map_resume(&s_map, &s_map_error);
        dhara_map_sync(&s_map, &s_map_error);

        Stat[1] &= ~STA_NOINIT; /* Clear STA_NOINIT flag */
        return Stat[1];
    } else {
        return STA_NODISK;
    }

    return Stat[0];
}

/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(BYTE drv /* Physical drive number (0) */
) {
    if (drv > 1) return STA_NOINIT;

    return Stat[drv]; /* Return disk status */
}

/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(
    BYTE drv,     /* Physical drive number (0) */
    BYTE *buff,   /* Pointer to the data buffer to store read data */
    LBA_t sector, /* Start sector number (LBA) */
    UINT count    /* Number of sectors to read (1..128) */
) {
    DWORD sect = (DWORD)sector;

    if (drv == 0) {
        while (count--) {
            if (sdmmc_read_blocks(&s_sd_sdmmc_device, (uint8_t *)rw_buf, sect++,
                                  1) != STATUS_OK) {
                return RES_ERROR;
            }
            memcpy(buff, rw_buf, 512);
            buff += 512;
        }
        return RES_OK;
    } else if (drv == 1) {
        unsigned int i = 0;
        while (i < count) {
            if (dhara_map_read(&s_map, sect, (uint8_t *)buff, &s_map_error) !=
                0) {
                return RES_ERROR;
            }
            i++;
            sect++;
            buff += (1U << s_nand.log2_page_size);
        }
        return RES_OK;
    }

    return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE drv,         /* Physical drive number (0) */
                   const BYTE *buff, /* Ponter to the data to write */
                   LBA_t sector,     /* Start sector number (LBA) */
                   UINT count        /* Number of sectors to write (1..128) */
) {
    DWORD sect = (DWORD)sector;

    if (drv == 0) {
        while (count--) {
            memcpy(rw_buf, buff, 512);
            buff += 512;
            if (sdmmc_write_blocks(&s_sd_sdmmc_device, (uint8_t *)rw_buf,
                                   sect++, 1) != STATUS_OK) {
                return RES_ERROR;
            }
        }
        return RES_OK;
    } else if (drv == 1) {
        unsigned int i = 0;
        while (i < count) {
            if (dhara_map_write(&s_map, sect, (uint8_t *)buff, &s_map_error) !=
                0) {
                return RES_ERROR;
            }
            i++;
            sect++;
            buff += (1U << s_nand.log2_page_size);
        }
        return RES_OK;
    }

    return RES_PARERR;
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(BYTE drv,  /* Physical drive number (0) */
                   BYTE cmd,  /* Control command code */
                   void *buff /* Pointer to the conrtol data */
) {
    SdmmcInfo info;
    DWORD st, ed;
    DRESULT res;
    LBA_t *dp;

    if (drv == 0) {
        if (Stat[0] & STA_NOINIT)
            return RES_NOTRDY; /* Check if drive is ready */

        res = RES_ERROR;

        switch (cmd) {
            case CTRL_SYNC: /* Wait for end of internal write process of the
                             * drive
                             */
                if (sdmmc_check_status(200) != STATUS_OK) {
                    res = RES_ERROR;
                    break;
                }
                res = RES_OK;
                break;

            case GET_SECTOR_COUNT: /* Get drive capacity in unit of sector
                                    * (DWORD)
                                    */
                if (sdmmc_info(&s_sd_sdmmc_device, &info) != STATUS_OK) {
                    res = RES_ERROR;
                    break;
                }
                *(DWORD *)buff = info.LogBlockNbr;
                res = RES_OK;
                break;

            case GET_SECTOR_SIZE: /* Get sector size (WORD) */
                if (sdmmc_info(&s_sd_sdmmc_device, &info) != STATUS_OK) {
                    res = RES_ERROR;
                    break;
                }
                *(WORD *)buff = info.LogBlockSize;
                res = RES_OK;
                break;

            case GET_BLOCK_SIZE: /* Get erase block size in unit of sector
                                  * (DWORD)
                                  */
                if (sdmmc_info(&s_sd_sdmmc_device, &info) != STATUS_OK) {
                    res = RES_ERROR;
                    break;
                }
                *(WORD *)buff = info.LogBlockSize / 512;
                res = RES_OK;
                break;

            case CTRL_TRIM: /* Erase a block of sectors (used when _USE_ERASE ==
                             * 1)
                             */
                dp = buff;
                st = (DWORD)dp[0];
                ed = (DWORD)dp[1];
                if (sdmmc_erase(&s_sd_sdmmc_device, st, ed) != STATUS_OK) {
                    res = RES_ERROR;
                    break;
                }
                if (sdmmc_check_status(30000) != STATUS_OK) {
                    res = RES_ERROR;
                    break;
                }
                res = RES_OK;
                break;

            default:
                res = RES_PARERR;
        }
        return res;
    } else if (drv == 1) {
        switch (cmd) {
            case CTRL_SYNC:
                if (dhara_map_sync(&s_map, &s_map_error) != 0) {
                    return RES_ERROR;
                }
                return RES_OK;
            case GET_SECTOR_COUNT:
                *(DWORD *)buff = (DWORD)dhara_map_capacity(&s_map);
                return RES_OK;
            case GET_SECTOR_SIZE:
                *(DWORD *)buff = 1U << s_nand.log2_page_size;
                return RES_OK;
            case GET_BLOCK_SIZE:
                *(DWORD *)buff = 1U << s_nand.log2_ppb;
                return RES_OK;
            case CTRL_TRIM:
                dp = buff;
                st = (DWORD)dp[0];
                ed = (DWORD)dp[1];
                for (int i = st; i < ed; i++) {
                    if (dhara_map_trim(&s_map, i, &s_map_error) != 0) {
                        return RES_ERROR;
                    }
                }
                return RES_OK;
            default:
                return RES_PARERR;
        }
    }

    return RES_PARERR;
}
