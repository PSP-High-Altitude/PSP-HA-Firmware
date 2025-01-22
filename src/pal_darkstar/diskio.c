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
#include "dhara/nand.h"
#include "gpio/gpio.h"
#include "mt29f4g.h"
#include "sd.h"
#include "sdmmc/sdmmc.h"
#include "semphr.h"
#include "spi/spi.h"
#include "stdio.h"
#include "task.h"

// SD or SPI
#define SD

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
static volatile UINT Timer1,
    Timer2; /* 1kHz decrement timer stopped at zero (disk_timerproc()) */

#ifndef SD
BYTE CardType; /* Card type flags */
#endif

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
/* SPI controls (Platform dependent)                                     */
/*-----------------------------------------------------------------------*/
struct dhara_nand s_nand;
struct dhara_map s_map;
static uint8_t s_map_buffer[1U << MT29F4G_PAGE_SIZE_LOG2];
static dhara_error_t s_map_error = DHARA_E_NONE;

static SemaphoreHandle_t s_mutex = NULL;

/* Initialize NAND interface */
Status diskio_init(SdDevice *device) {
    generate_crc_table();

    // Initialize NAND
    memset(&s_map, 0, sizeof(s_map));
    memset(&s_map_buffer, 0, sizeof(s_map_buffer));
    memset(&s_nand, 0, sizeof(s_nand));

    // Virtual pages
    // s_nand.log2_page_size = MT29F4G_PAGE_SIZE_LOG2 - 2;  // 4 partial pages
    // s_nand.log2_ppb =
    //    MT29F4G_PAGE_PER_BLOCK_LOG2 + 2;  // 64*4 partial pages per block
    // s_nand.num_blocks = MT29F4G_BLOCK_COUNT;

    // No virtual pages
    s_nand.log2_page_size = MT29F4G_PAGE_SIZE_LOG2;
    s_nand.log2_ppb = MT29F4G_PAGE_PER_BLOCK_LOG2;
    s_nand.num_blocks = MT29F4G_BLOCK_COUNT;

    // Create mutex
    s_mutex = xSemaphoreCreateRecursiveMutex();
    configASSERT(s_mutex);

    return STATUS_OK;
}

DWORD get_fattime() { return 0; }

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE drv /* Physical drive number (0) */
) {
    if (drv == 1) {
        if ((Stat[1] & STA_NOINIT) == 0)
            return Stat[1]; /* Check if drive is ready */

        if (mt29f4g_init() != STATUS_OK) {
            printf("Failed to initialize NAND hardware\n");
            Stat[1] = STA_NOINIT;
            return Stat[1];
        }

        dhara_map_init(&s_map, &s_nand, s_map_buffer, 4);
        dhara_map_resume(&s_map, &s_map_error);
        dhara_map_sync(&s_map, &s_map_error);

        Stat[1] &= ~STA_NOINIT; /* Clear STA_NOINIT flag */
        return Stat[1];
    }
    return STA_NOINIT;
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
/* Locking functions                                                     */
/*-----------------------------------------------------------------------*/

static BaseType_t diskioENTER_CRITICAL() {
    BaseType_t ctx = 0;

    // Acquire the mutex before proceeding if the scheduler is running
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        while (xSemaphoreTakeRecursive(s_mutex, 1) != pdPASS);
    }

    return ctx;
}

static void diskioEXIT_CRITICAL(BaseType_t ctx) {
    // Release the mutex regardless of whether the scheduler is running, since
    // it's technically possible for the scheduler to be stopped with held
    // locks, and we don't want a lockup in that case
    xSemaphoreGiveRecursive(s_mutex);
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
    // Non reentrant, and USB will try it
    BaseType_t ctx = diskioENTER_CRITICAL();

    DWORD sect = (DWORD)sector;
    if (drv == 1) {
        unsigned int i = 0;
        while (i < count) {
            if (dhara_map_read(&s_map, sect, (uint8_t *)buff, &s_map_error) !=
                0) {
                diskioEXIT_CRITICAL(ctx);
                return RES_ERROR;
            }
            i++;
            sect++;
            buff += (1U << s_nand.log2_page_size);
        }
        diskioEXIT_CRITICAL(ctx);
        return RES_OK;
    }

    diskioEXIT_CRITICAL(ctx);
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
    // Non reentrant, and USB will try it
    BaseType_t ctx = diskioENTER_CRITICAL();

    DWORD sect = (DWORD)sector;
    if (drv == 1) {
        unsigned int i = 0;
        while (i < count) {
            if (dhara_map_write(&s_map, sect, (uint8_t *)buff, &s_map_error) !=
                0) {
                diskioEXIT_CRITICAL(ctx);
                return RES_ERROR;
            }
            i++;
            sect++;
            buff += (1U << s_nand.log2_page_size);
        }
        diskioEXIT_CRITICAL(ctx);
        return RES_OK;
    }

    diskioEXIT_CRITICAL(ctx);
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
    DWORD st, ed;
    LBA_t *dp;

    // Non reentrant, and USB will try it
    BaseType_t ctx = diskioENTER_CRITICAL();

    if (drv == 1) {
        switch (cmd) {
            case CTRL_SYNC:
                if (dhara_map_sync(&s_map, &s_map_error) != 0) {
                    diskioEXIT_CRITICAL(ctx);
                    return RES_ERROR;
                }
                diskioEXIT_CRITICAL(ctx);
                return RES_OK;
            case GET_SECTOR_COUNT:
                *(DWORD *)buff = (DWORD)dhara_map_capacity(&s_map);
                diskioEXIT_CRITICAL(ctx);
                return RES_OK;
            case GET_SECTOR_SIZE:
                *(DWORD *)buff = 1U << s_nand.log2_page_size;
                diskioEXIT_CRITICAL(ctx);
                return RES_OK;
            case GET_BLOCK_SIZE:
                *(DWORD *)buff = 1U << s_nand.log2_ppb;
                diskioEXIT_CRITICAL(ctx);
                return RES_OK;
            case CTRL_TRIM:
                dp = buff;
                st = (DWORD)dp[0];
                ed = (DWORD)dp[1];
                for (int i = st; i < ed; i++) {
                    if (dhara_map_trim(&s_map, i, &s_map_error) != 0) {
                        diskioEXIT_CRITICAL(ctx);
                        return RES_ERROR;
                    }
                }
                diskioEXIT_CRITICAL(ctx);
                return RES_OK;
            default:
                diskioEXIT_CRITICAL(ctx);
                return RES_PARERR;
        }
    }
    diskioEXIT_CRITICAL(ctx);
    return RES_PARERR;
}
