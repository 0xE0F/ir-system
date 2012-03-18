#include <stm32f10x.h>
#include <IR/IR.h>

#include <Storage/ffconf.h>
#include <Storage/ff.h>
#include <Storage/diskio.h>

#include <Storage/Storage.h>

/* ============================================= */
// Public API
/* ============================================= */

static FATFS _FatFs;

// Инициализация хранилища
StorageStatus InitStorage(void)
{
	FRESULT result;

	WORD status = (WORD)disk_initialize(0);
	if (status)
	{
		if (status == STA_NODISK)
			return StorageNoDevice;
		if (status == STA_NOINIT)
			return StorageInternalError;
		if (status == STA_PROTECT)
			return StorageWriteProtect;
	}

	result = f_mount(0, &_FatFs);
	if (result != FR_OK)
		return StorageInternalError;

	return StorageNoError;
}

// Сохранение кода в хранилище
StorageStatus Save(IRCode *code)
{
	return StorageInternalError;
}

// Чтение из хранилища
StorageStatus Open(const uint32_t id, IRCode *resutl)
{
	return StorageInternalError;
}

/* ============================================= */
// Debug fucntions
/* ============================================= */

// Отображение статуса устройства
void PrintStorageStatus(void)
{
	unsigned long rLong;
	WORD rWord;
	BYTE rByte;

	if (disk_ioctl(0, GET_SECTOR_COUNT, &rLong) == RES_OK)
		{ printf("Drive size: %u sectors\r\n", rLong); }
	if (disk_ioctl(0, GET_SECTOR_SIZE, &rWord) == RES_OK)
		{ printf("Sector size: %u\r\n", rWord); }
	if (disk_ioctl(0, GET_BLOCK_SIZE, &rLong) == RES_OK)
		{ printf("Erase block size: %u sectors\r\n", rLong); }
	if (disk_ioctl(0, MMC_GET_TYPE, &rByte) == RES_OK)
		{ printf("MMC/SDC type: %u\r\n", rByte); }

	FATFS *fs = &_FatFs;
	rByte = f_getfree("", (DWORD*)&rLong, &fs);
	if (!rByte)
	{
		printf("FAT type = %u (%s)\n\rNumber of FATs = %u\n\r",
				(WORD)_FatFs.fs_type, (_FatFs.fs_type==FS_FAT12) ? "FAT12" : (_FatFs.fs_type==FS_FAT16) ? "FAT16" : "FAT32",
				(WORD)_FatFs.n_fats);
	}
	else
	{
		printf("Cannot get more info. Error in getfree: %d\n\r", rByte);
	}
}

FILINFO Finfo;

static
FRESULT scan_files (
    char* path        /* Start node to be scanned (also used as work area) */
)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;   /* This function is assuming non-Unicode cfg. */
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif


    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                sprintf(&path[i], "/%s", fn);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
                printf("%s/%s\n\r", path, fn);
            }
        }
    }

    return res;
}

// Отображение содержимого хранилища
void PrintConentStorage(void)
{
	char path[255] = {"0:"};
	scan_files(path);
}

