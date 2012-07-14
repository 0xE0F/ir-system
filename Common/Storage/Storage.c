#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <IR/IR.h>

#include <Storage/ffconf.h>
#include <Storage/ff.h>
#include <Storage/diskio.h>

#include <Storage/Storage.h>

#include <Terminal.h>

static FATFS _FatFs;						/* �������� ������� */
static uint8_t StorageInit;					/* ���� ������������� ��������� */

static void MakeFileName(uint32_t number, char *str);


/** ������������� ��������� */
bool InitStorage(void)
{
	FRESULT result;
	StorageInit = 0;
	printf("Initialazing storage...");
	WORD status = (WORD)disk_initialize(0);
	if (status) {
		if (status == STA_NODISK) {
			print("no disk\n\r");
		}
		if (status == STA_NOINIT) {
			print("interal error\n\r");
		}
		if (status == STA_PROTECT) {
			print("write protect\n\r");
		}

		return false;
	}

	result = f_mount(0, &_FatFs);
	if (result != FR_OK) {
		print("Unable to mount fs\n\r");
		return false;
	}

	StorageInit = 1;
	print("done\n\r");
	return true;
}

// ���������� ���� � ���������
bool Save(IRCode *code, const uint16_t number)
{
	FIL file;
	FRESULT res;
	UINT bw;
	uint16_t crc = 0xFFFF;

	char fName[8 + 1 + 3 + 1] = {'\0'}; // 8 name, + .bin + \0

	if (!StorageInit) {
		print("Storage not init\n\r");
		return false;
	}

	crc = GetCrc(code);
	code->Crc = crc;

	MakeFileName(number, fName);
	res = f_open(&file, fName, FA_CREATE_ALWAYS | FA_WRITE);
	f_sync(&file);
	if (res) {
		printf("Create file error: %d\n\r", res);
		return false;
	}

	res = f_write(&file, code, sizeof(IRCode), &bw);
	f_sync(&file);
	if (res || bw != sizeof(IRCode)) {
		printf("Write file error: %d, bytes to write: %d\n\r", res, bw);
		return false;
	}
	f_close(&file);

	return true;
}

// ������ �� ���������
bool Open(IRCode *code, const uint16_t number)
{
	char fName[8 + 1 + 3 + 1] = {'\0'}; // 8 name, + .bin + \0
	FRESULT res;
	FIL file;
	UINT btr = 0;
	uint16_t crc = 0xFFFF;

	if (!StorageInit) {
		print("Storage not init\n\r");
		return false;
	}

	if (!code) {
		print("Null pointer reference\n\r");
		return false;
	}

	MakeFileName(number, fName);
	res = f_open(&file, fName, FA_OPEN_EXISTING | FA_READ);
//	f_sync(&file);

	if (res) {
		printf("Error open file: %s, res: %d\n\r", fName, res);
		return false;
	}

	res = f_read(&file, code, sizeof(IRCode), &btr);
//	f_sync(&file);

	if (res || btr != sizeof(IRCode)) {
		printf("Error reading file: %d, bytes to read: %d\n\r", res, btr);
		return false;
	}

	f_close(&file);

	crc = GetCrc(code);
	return crc == code->Crc;
}


static void MakeFileName(uint32_t number, char *str)
{
	if (!str)
		return ;

	sprintf(str, "%08X.bin", (unsigned int)number);
}

/* ============================================= */
// Debug fucntions
/* ============================================= */

// ����������� ������� ����������
void PrintStorageStatus(void)
{
	unsigned long rLong;
	WORD rWord;
	BYTE rByte;

	if (disk_ioctl(0, GET_SECTOR_COUNT, &rLong) == RES_OK) {
		printf("Drive size: %u sectors\r\n", (unsigned int)rLong);
	} if (disk_ioctl(0, GET_SECTOR_SIZE, &rWord) == RES_OK)	{
		printf("Sector size: %u\r\n", rWord);
	} if (disk_ioctl(0, GET_BLOCK_SIZE, &rLong) == RES_OK) {
		printf("Erase block size: %u sectors\r\n", (unsigned int)rLong);
	} if (disk_ioctl(0, MMC_GET_TYPE, &rByte) == RES_OK) {
		printf("MMC/SDC type: %u\r\n", rByte);
	}

	FATFS *fs = &_FatFs;
	rByte = f_getfree("", (DWORD*)&rLong, &fs);

	if (!rByte) {
		printf("FAT type = %u (%s)\n\rNumber of FATs = %u\n\r",
				(WORD)_FatFs.fs_type, (_FatFs.fs_type==FS_FAT12) ? "FAT12" : (_FatFs.fs_type==FS_FAT16) ? "FAT16" : "FAT32",
				(WORD)_FatFs.n_fats);
	} else {
		printf("Cannot get more info. Error in getfree: %d\n\r", rByte);
	}
}

/** ��� ������� ��������� ������ ��� ������������ ������ */
typedef void (*action_on_enum_t) (char *path, char *fname);

/** ������������ ������ */
static FRESULT EnumerateFiles ( char* path, action_on_enum_t func)
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
                res = EnumerateFiles(path, func);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
                if (func)
                	func(path, fn);
            }
        }
    }

    return res;
}

/** */
static void PrintCallBack(char *path, char *fname)
{
	printf("%s/%s\n\r", path, fname);
}

/** ������� ��������� ������ ��� ������� */
static void EraseCallBack(char *path, char *fname)
{
	FIL file;
	FRESULT res;
	char fullName[259] = {'\0'};
	size_t len = strlen(path) + strlen(fname);

	if (len > ( sizeof(fullName) / sizeof(fullName[0])) )
		return;

	sprintf(fullName, "%s/%s", path, fname);
	res = f_open(&file, fullName, FA_OPEN_EXISTING | FA_WRITE);
	f_sync(&file);

	if (res) {
		printf("Error open file: %s, res: %d\n\r", fullName, res);
		return;
	}

	res = f_truncate(&file);
	if (res != FR_OK) {
		printf("Error truncate file: %s, res: %d\n\r", fullName, res);
	}
	f_sync(&file);
	f_close(&file);

}

// ����������� ����������� ���������
void PrintConentStorage(void)
{
	char path[259] = {"0:"};
	EnumerateFiles(path, PrintCallBack);
}

/** ������� ��������� */
void EraseStorage(void)
{
	char path[259] = {"0:"};
	EnumerateFiles(path, EraseCallBack);
}
