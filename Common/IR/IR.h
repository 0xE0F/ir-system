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
	uint32_t Flags;
	uint32_t Frequency;
	uint32_t IntervalsCount;
	uint32_t Intervals[INTERVALS_MAX];	// Интрвал - старший бит - значение уровня, остальные - время в мкс
} IRCode;

#pragma pack(0)

#if (INTERVALS_MAX*INTERVAL_SIZE + IR_CODE_HEADER_SIZE) > STORAGE_PAGE_SIZE
#error Size of IRCode NOT EQUAL STORAGE_PAGE_SIZE
#endif

enum { InvalidFrequencyValue = 0 };

// Минимально детектируемая частота
extern const uint32_t DetectFreqMin;
// Максимально детектируемая частота
extern const uint32_t DetectFreqMax;

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

// отладчный вывод кода
void DebugPrint(IRCode *code);

typedef enum  { CS_OK, CS_ERROR, CS_WARNING} CheckStatus;
// Проверка кода
CheckStatus CheckIRCode(IRCode *code);

// Тип - результат сравнения кодов
typedef enum {CodesEqual = 0, FirstEmpty, SecondEmpty, LengthNotEqual, CodesValueNotEqual, CodesIntervalNotEqual, FreqNotEqual} CompareCodesResult;

// Сравнение кодов
CompareCodesResult CompareIrCode(IRCode *left, IRCode *rigth);
#endif // __IR__H__
