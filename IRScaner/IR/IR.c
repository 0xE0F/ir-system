#include <stm32f10x.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "IR.h"

// Минимально детектируемая частота
const uint32_t DetectFreqMin = 29000;
// Максимально детектируемая частота
const uint32_t DetectFreqMax = 60500;

// Стандартные значения частот
static const uint32_t StandartFrequency[] = {30000, 32000, 34000, 36000, 38000, 40000, 42000, 44000, 46000, 48000, 50000, 52000, 54000, 56000, 58000, 60000};
// Длина буфера стандартных частот
static const uint32_t StandartFrequencyCount = sizeof(StandartFrequency) / sizeof(uint32_t);

// Маска времени
const uint32_t TimeMask = 0x7FFFFFFF;

// Маска значения
const uint32_t ValueMask = 0x80000000;

extern signed int printf(const char *pFormat, ...);

// Получение стандартной частоты из интервала 30 кГц - 60 кГц
// 0 - вне диапазона
uint32_t GetFrequencyInterval(uint32_t value)
{
	uint32_t d1 = 0;
	uint32_t d2 = 0;

	for (uint32_t i = 0; i < StandartFrequencyCount-1; i++ )
	{
		d1 = abs(StandartFrequency[i] - value);
		d2 = abs(StandartFrequency[i+1] - value);

		if ((d1 >= 1001) && (d2 >= 1001))
			continue;
		return ((d1 < d2) ? StandartFrequency[i] : StandartFrequency[i+1]);
	}

	return 0;
}

void SetTime(uint32_t *interval, const uint32_t time)
{
	(*interval) &= ~TimeMask;
	(*interval) = (*interval | (time & TimeMask));
}

void SetValue(uint32_t *interval, const uint8_t value)
{
	(*interval) = (value > 0) ? (*interval | ValueMask) : (*interval & (~ValueMask));
}

uint32_t GetTime(uint32_t interval) { return interval & TimeMask; }

uint8_t GetValue(uint32_t interval) { return (interval & ValueMask) == ValueMask; }

// отладчный вывод кода
void DebugPrint(IRCode *code)
{
	if (!code)
	{
		return;
	}

	printf("ID: 0x%08X\n\r", (unsigned int)code->ID);
	printf("Frequency: %05d Hz\n\r", (unsigned int)code->Frequency);
	printf("Flags: 0x%04X\n\r", (unsigned short)code->Flags);
	printf("Count: %u\n\r", (unsigned int)code->IntervalsCount);

	if (code->IntervalsCount > INTERVALS_MAX)
	{
		printf("Intervals count out if range\n\r");
		return;
	}

	for(uint32_t i=0; i < code->IntervalsCount; i++)
	{
		printf("[%03u] %01X -> [%08u us]\n\r", (unsigned int)i, (unsigned char)GetValue(code->Intervals[i]), (unsigned int)GetTime(code->Intervals[i]));
	}
	printf("==> ");
}

CheckStatus CheckIRCode(IRCode *code)
{
	if (!code)
		return CS_ERROR;

	if (code->IntervalsCount >= INTERVALS_MAX)
		return CS_ERROR;

	if ((code->Frequency < DetectFreqMin) || (code->Frequency > DetectFreqMax))
		return CS_ERROR;

	if ((code->Frequency < 36000) || (code->Frequency >38000))
		return CS_WARNING;

	return CS_OK;
}
