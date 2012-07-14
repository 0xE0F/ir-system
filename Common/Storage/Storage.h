#ifndef __STORAGE__H__
#define __STORAGE__H__

/* Инициализация хранилища */
bool InitStorage(void);

/* Сохранение кода в хранилище */
bool Save(IRCode *code, const uint16_t number);

/* Чтение из хранилища */
bool Open(IRCode *code, const uint16_t number);

/** Очистить хранилище */
void EraseStorage(void);

/** Тип функции обратного вызова при перечеслении файлов */
typedef void (*action_on_enum_t) (char *path, char *fname);

/** Пеерчисление файлов */
FRESULT EnumerateFiles ( char* path, action_on_enum_t func);

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
