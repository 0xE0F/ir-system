/*
 *	Заголовочный файл с основной функциональность.
 */
#ifndef _IRDA_API_H_
#define _IRDA_API_H_

#define STORAGE_PAGE_SIZE 512

// Один интервал
#pragma pack(1)
typedef struct
{
	uint16_t Time;
	uint8_t  Value;
} OnePulse;

#define MaxPulses 166 //((512 - 4 * 3) / sizeof(OnePulse))

// Хранение кода
typedef struct
{
	uint32_t ID;
	uint32_t Frequency;
	uint32_t PulsesCount;
	OnePulse Pulses[MaxPulses];
	uint16_t Flags;
} IRCode;
#pragma pack(0)

#if (MaxPulses*3 + 3*4+2) != STORAGE_PAGE_SIZE
#error Size of IRCode NOT EQUAL STORAGE_PAGE_SIZE
#endif


// Запись кода со сканера
uint32_t StartRecord(IRCode *irCode);
// Остановка записи
uint32_t StopRecord();

// Чтение кода из устроства хранениея
uint32_t ReadCode(IRCode *code, uint32_t id);
// Запись кода в устройство хранения
uint32_t WriteCode(IRCode *code, uint32_t id);

// Выдача кода в соответствующий канал
uint32_t WriteToChannel(IRCode *code, uint32_t channelID);

// отладчный вывод кода
void DebugPrint(IRCode *code);

// Получение стандартной частоты из интервала 30 кГц - 60 кГц
// 0 - если частота вне диапазона
uint32_t GetFrequencyInterval(uint32_t value);

// Установка несущей частоты
void SetCarrierFrequency(const uint32_t value);

#endif // _IRDA_API_H_
