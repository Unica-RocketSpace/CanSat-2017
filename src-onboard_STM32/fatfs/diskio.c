/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "diskio.h"     /* FatFs lower layer API */

#include <sd.h>

/* Definitions of physical drive number for each drive */
//#define DEV_RAM     0   /* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC     0   /* Example: Map MMC/SD card to physical drive 1 */
//#define DEV_USB     2   /* Example: Map USB MSD to physical drive 2 */


static int mmc_status = STA_NOINIT;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE pdrv       /* Physical drive nmuber to identify the drive */
)
{
   if (DEV_MMC != pdrv)
      return STA_NODISK;

   return (DSTATUS)mmc_status;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv               /* Physical drive nmuber to identify the drive */
)
{
   if (DEV_MMC != pdrv)
      return STA_NODISK;

   sd_init();
   sd_error_t err = sd_startup();
   if (SD_ERROR_NONE == err)
      mmc_status &= ~STA_NOINIT;

   return (DSTATUS)mmc_status;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,      /* Physical drive nmuber to identify the drive */
    BYTE *buff,     /* Data buffer to store read data */
    DWORD sector,   /* Start sector in LBA */
    UINT count      /* Number of sectors to read */
)
{
   if (pdrv != DEV_MMC)
      return RES_PARERR;

   if (mmc_status & (STA_NOINIT | STA_NODISK))
      return RES_NOTRDY;

   //led_set(true);

   sd_error_t err = sd_block_read_multi(sector, buff, count);
   DRESULT retval = RES_OK;
   if (err)
      retval = RES_ERROR;

   //led_set(false);
   return retval;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
    BYTE pdrv,          /* Physical drive nmuber to identify the drive */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Start sector in LBA */
    UINT count          /* Number of sectors to write */
)
{

   if (pdrv != DEV_MMC)
      return RES_PARERR;

   if (mmc_status & (STA_NOINIT | STA_NODISK))
      return RES_NOTRDY;

   //led_set(true);

   sd_error_t err = sd_block_write_multi(sector, buff, count);
   DRESULT retval = RES_OK;
   if (err != SD_ERROR_NONE)
      retval = RES_ERROR;

   //led_set(false);
   return retval;
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE pdrv,      /* Physical drive nmuber (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
   (void)buff;

   if (pdrv != DEV_MMC)
      return RES_PARERR;

   if (CTRL_SYNC == cmd)
      return RES_OK;
   else
      return RES_ERROR; // не должна быть вызвана эта функция.
}

