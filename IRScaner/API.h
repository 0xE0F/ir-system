/*
 *	Заголовочный файл с основной функциональность.
 */
#ifndef _IRDA_API_H_
#define _IRDA_API_H_

#define STORAGE_PAGE_SIZE 2048
#define INTERVAL_SIZE (4)
#define IR_CODE_HEADER_SIZE (4 * 4)
// Один интервал
#pragma pack(1)
typedef struct
{
	uint32_t Time;
	uint8_t  Value;
} Interval;

#define INTERVALS_MAX ((STORAGE_PAGE_SIZE - IR_CODE_HEADER_SIZE) / INTERVAL_SIZE) //

// Хранение кода
typedef struct
{
	uint32_t ID;
	uint32_t Flags;
	uint32_t Frequency;
	uint32_t IntervalsCount;
	uint32_t Intervals[INTERVALS_MAX];
	//Interval Intervals[INTERVALS_MAX];
} IRCode;
#pragma pack(0)

#if (INTERVALS_MAX*INTERVAL_SIZE + IR_CODE_HEADER_SIZE) > STORAGE_PAGE_SIZE
#error Size of IRCode NOT EQUAL STORAGE_PAGE_SIZE
#endif

extern const uint32_t MaxChannelNumber;

// Сканирование кода
void Scan(IRCode *irCode);
// Остановка сканирования
void StopScan();
// Производится сканирование
volatile uint32_t IsScanning();

// Чтение кода из устроства хранениея
uint32_t ReadCodeFromStorage(IRCode *code, uint32_t id);
// Запись кода в устройство хранения
uint32_t WriteCodeToStorage(IRCode *code, uint32_t id);

// Выдача кода в соответствующий канал
// 0 - принято в очередь
// 1 - ошибка (код не найден или неизвесный номер канала)
uint32_t WriteToChannel(IRCode *code, uint32_t channelID);

// Получение стандартной частоты из интервала 30 кГц - 60 кГц
// 0 - если частота вне диапазона
uint32_t GetFrequencyInterval(uint32_t value);

// Установка несущей частоты
void SetCarrierFrequency(const uint32_t value);

// отладчный вывод кода
void DebugPrint(IRCode *code);

#endif // _IRDA_API_H_
