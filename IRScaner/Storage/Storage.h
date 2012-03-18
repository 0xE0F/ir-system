#ifndef __STORAGE__H__
#define __STORAGE__H__

/* ============================================= */
// Public API
/* ============================================= */

typedef enum {StorageNoError, StorageNoDevice, StorageWriteProtect, StorageNoFreeSpace, StorageNotFound, StorageInternalError} StorageStatus;

// Инициализация хранилища
StorageStatus InitStorage(void);

// Сохранение кода в хранилище
StorageStatus Save(IRCode *code);

// Чтение из хранилища
StorageStatus Open(const uint32_t id, IRCode *resutl);

/* ============================================= */
// Debug fucntions
/* ============================================= */

// Отображение статуса устройства
void PrintStorageStatus(void);

// Отображение содержимого хранилища
void PrintConentStorage(void);


#endif //__STORAGE__H__
