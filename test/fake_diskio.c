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
#include "sd.h"

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

static volatile DSTATUS Stat = STA_NOINIT; /* Physical drive status */
static volatile UINT Timer1,
    Timer2; /* 1kHz decrement timer stopped at zero (disk_timerproc()) */

static BYTE CardType; /* Card type flags */

// Create a 256 MiB buffer for simulating writes and reads
#define FAKE_CARD_SIZE ((uint64_t)(1 << 28))
static BYTE fake_card[FAKE_CARD_SIZE];

#define FAKE_SECTOR_SIZE 512

DWORD get_fattime() { return 0; }

Status diskio_init(SdDevice *device) { return STATUS_OK; }

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE drv /* Physical drive number (0) */
) {
    // BYTE n, cmd, ty, ocr[4];

    if (drv) return STA_NOINIT; /* Supports only drive 0 */

    if (Stat & STA_NODISK) return Stat; /* Is card existing in the soket? */

    CardType = CT_SDC2; /* Card type */

    if (CardType) {          /* OK */
        Stat &= ~STA_NOINIT; /* Clear STA_NOINIT flag */
    } else {                 /* Failed */
        Stat = STA_NOINIT;
    }

    return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(BYTE drv /* Physical drive number (0) */
) {
    if (drv) return STA_NOINIT; /* Supports only drive 0 */

    return Stat; /* Return disk status */
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

    if (drv || !count) return RES_PARERR;     /* Check parameter */
    if (Stat & STA_NOINIT) return RES_NOTRDY; /* Check if drive is ready */

    for (UINT read = 0; read < count; read++) {
        for (int i = 0; i < FAKE_SECTOR_SIZE; i++) {
            size_t ridx = (sect + read) * FAKE_SECTOR_SIZE + i;

            if (ridx >= FAKE_CARD_SIZE) {
                return RES_ERROR;
            }

            buff[read * FAKE_SECTOR_SIZE + i] = fake_card[ridx];
        }
    }

    return RES_OK; /* Return result */
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

    if (drv || !count) return RES_PARERR;     /* Check parameter */
    if (Stat & STA_NOINIT) return RES_NOTRDY; /* Check drive status */
    if (Stat & STA_PROTECT) return RES_WRPRT; /* Check write protect */

    for (UINT written = 0; written < count; written++) {
        for (int i = 0; i < FAKE_SECTOR_SIZE; i++) {
            size_t widx = (sect + written) * FAKE_SECTOR_SIZE + i;

            if (widx >= FAKE_CARD_SIZE) {
                return RES_ERROR;
            }

            fake_card[widx] = buff[written * FAKE_SECTOR_SIZE + i];
        }
    }

    return RES_OK; /* Return result */
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(BYTE drv,  /* Physical drive number (0) */
                   BYTE cmd,  /* Control command code */
                   void *buff /* Pointer to the conrtol data */
) {
    DRESULT res;
    BYTE n, csd[16];
    DWORD st, ed, csize;
    LBA_t *dp;

    if (drv) return RES_PARERR;               /* Check parameter */
    if (Stat & STA_NOINIT) return RES_NOTRDY; /* Check if drive is ready */

    res = RES_ERROR;

    switch (cmd) {
        case CTRL_SYNC: /* Wait for end of internal write process of the drive
                         */
            res = RES_OK;
            break;

        case GET_SECTOR_COUNT: /* Get drive capacity in unit of sector (DWORD)
                                */
            *(LBA_t *)buff = FAKE_CARD_SIZE / FAKE_SECTOR_SIZE;
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE: /* Get erase block size in unit of sector (DWORD)
                              */
            *(DWORD *)buff = 1;
            res = RES_OK;
            break;

        case CTRL_TRIM: /* Erase a block of sectors (used when _USE_ERASE == 1)
                         */
            res = RES_OK;  // Technically this won't work but there's also no
                           // reason for the driver to read an erased block
            break;

        default:
            res = RES_PARERR;
    }

    return res;
}
