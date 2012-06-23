#ifndef __STORAGE__H__
#define __STORAGE__H__

/* Инициализация хранилища */
bool InitStorage(void);

/* Сохранение кода в хранилище */
bool Save(IRCode *code);

/* Чтение из хранилища */
bool Open(const uint32_t id, IRCode *result);

/* ============================================= */
// Debug fucntions
/* ============================================= */

// Отображение статуса устройства
void PrintStorageStatus(void);

// Отображение содержимого хранилища
void PrintConentStorage(void);

/* Вклчюение отладочного режима хранилища */
void SetStorageDebugMode(bool mode);


#endif //__STORAGE__H__
