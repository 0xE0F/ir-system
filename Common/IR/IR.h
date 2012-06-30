/*
 * Общие определения для инфракрасной части
 */
#ifndef __IR__H__
#define __IR__H__

#define STORAGE_PAGE_SIZE 2048
#define INTERVAL_SIZE (4)
#define IR_CODE_HEADER_SIZE (4 * 4)

#define INTERVALS_MAX ((STORAGE_PAGE_SIZE - IR_CODE_HEADER_SIZE) / INTERVAL_SIZE) // Максимальное количество интервалов

#pragma pack(1)

// Хранение кода
typedef struct
{
	uint32_t ID;
	uint16_t Flags;
	uint16_t Crc;
	uint32_t Frequency;
	uint32_t IntervalsCount;
	uint32_t Intervals[INTERVALS_MAX];	// Интрвал - старший бит - значение уровня, остальные - время в мкс
} IRCode;

#pragma pack(0)

#if (INTERVALS_MAX*INTERVAL_SIZE + IR_CODE_HEADER_SIZE) > STORAGE_PAGE_SIZE
#error Size of IRCode NOT EQUAL STORAGE_PAGE_SIZE
#endif

enum { InvalidFrequencyValue = 0 };

extern const uint32_t DetectFreqMin; /* Минимально детектируемая частота */
extern const uint32_t DetectFreqMax; /* Максимальная детектируемая частота */

extern uint32_t LengthDelataMax; 		/* Максимальная разница в длине кодов */
extern uint32_t IntervalDelataMax;	/* Максимальная разница между интервалами */

extern bool DebugModeIr; /* Режим отладки ИК кодов */

// Получение стандартной частоты из интервала 30 кГц - 60 кГц
uint32_t GetFrequencyInterval(uint32_t value);

//Установка значения времени
void SetTime(uint32_t *interval, const uint32_t time);

// Установка значения интервала
void SetValue(uint32_t *interval, const uint8_t value);

// Получение значения времени интервала
uint32_t GetTime(uint32_t interval);

// Получение значения интервала
uint8_t GetValue(uint32_t interval);

bool CheckIRCode(IRCode *code); /* Проверка на допустимость кода */

bool IsEqual(IRCode *left, IRCode *rigth); /* Сравнение двух кодов */

void DebugPrint(IRCode *code); 	/* отладчный вывод кода */

#endif // __IR__H__
