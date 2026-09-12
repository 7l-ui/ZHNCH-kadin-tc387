#include "diskio.h"
#include "sd_simple.h"

#define FATFS_PDRV_SD   0

extern void sd_simple_reset(void);

static uint8_t g_sd_ready = 0;

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != FATFS_PDRV_SD)
    {
        return STA_NOINIT;
    }

    return g_sd_ready ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != FATFS_PDRV_SD)
    {
        return STA_NOINIT;
    }

    if (sd_simple_init() != 0)
    {
        g_sd_ready = 0;
        return STA_NOINIT;
    }

    g_sd_ready = 1;
    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if ((pdrv != FATFS_PDRV_SD) || (buff == 0) || (count == 0))
    {
        return RES_PARERR;
    }

    if (!g_sd_ready && (disk_initialize(pdrv) != 0))
    {
        return RES_NOTRDY;
    }

    if (sd_simple_read((uint8_t *)buff, (uint32_t)sector, (uint32_t)count) != 0)
    {
        g_sd_ready = 0;
        sd_simple_reset();
        return RES_ERROR;
    }

    return RES_OK;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if ((pdrv != FATFS_PDRV_SD) || (buff == 0) || (count == 0))
    {
        return RES_PARERR;
    }

    if (!g_sd_ready && (disk_initialize(pdrv) != 0))
    {
        return RES_NOTRDY;
    }

    if (sd_simple_write((uint8_t *)buff, (uint32_t)sector, (uint32_t)count) != 0)
    {
        g_sd_ready = 0;
        sd_simple_reset();
        return RES_ERROR;
    }

    return RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != FATFS_PDRV_SD)
    {
        return RES_PARERR;
    }

    if (!g_sd_ready && (disk_initialize(pdrv) != 0))
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
        case CTRL_SYNC:
            return RES_OK;

        case GET_SECTOR_SIZE:
            if (buff == 0)
            {
                return RES_PARERR;
            }
            *(WORD *)buff = 512;
            return RES_OK;

        case GET_BLOCK_SIZE:
            if (buff == 0)
            {
                return RES_PARERR;
            }
            *(DWORD *)buff = 1;
            return RES_OK;

        case GET_SECTOR_COUNT:
        {
            sd_simple_info_struct info;
            if ((buff == 0) || (sd_simple_get_info(&info) != 0))
            {
                return RES_ERROR;
            }

            if (info.block_count != 0)
            {
                *(DWORD *)buff = info.block_count;
            }
            else if (info.capacity_mb != 0)
            {
                *(DWORD *)buff = (DWORD)(((uint64_t)info.capacity_mb * 1024ULL * 1024ULL) / 512UL);
            }
            else
            {
                *(DWORD *)buff = 0;
            }
            return RES_OK;
        }

        default:
            return RES_PARERR;
    }
}

DWORD get_fattime(void)
{
    return ((DWORD)(2026 - 1980) << 25)
         | ((DWORD)1 << 21)
         | ((DWORD)1 << 16)
         | ((DWORD)0 << 11)
         | ((DWORD)0 << 5)
         | ((DWORD)0 >> 1);
}

