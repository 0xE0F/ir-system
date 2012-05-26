#include <stm32f10x.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "IR.h"

// Минимально детектируемая частота
const uint32_t DetectFreqMin = 29000;
// Максимально детектируемая частота
const uint32_t DetectFreqMax = 60500;

// Стандартные значения частот
static const uint32_t StandartFrequency[] = {30000, 32000, 34000, 36000, 38000, 40000, 42000, 44000, 46000, 48000, 50000, 52000, 54000, 56000, 58000, 60000};
// Длина буфера стандартных частот
static const uint32_t StandartFrequencyCount = sizeof(StandartFrequency) / sizeof(uint32_t);

uint32_t LengthDelataMax = 20; 		/* Максимальная разница в длине кодов */

uint32_t IntervalDelataMax = 500;	/* Максимальная разница между интервалами */

const uint32_t TimeMask = 0x7FFFFFFF;	/* Маска времени */

const uint32_t ValueMask = 0x80000000; /* Маска значения */

extern signed int printf(const char *pFormat, ...);

bool DebugModeIr = false; /* Флаг отладочного режима */

uint32_t min(uint32_t l, uint32_t r) { return (l < r) ? l : r ; }

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
	printf("\n\r");
}

bool CheckIRCode(IRCode *code)
{
	if (!code)
		return false;

	if (code->IntervalsCount >= INTERVALS_MAX)
		return false;

	if ((code->Frequency < DetectFreqMin) || (code->Frequency > DetectFreqMax))
		return false;

	if ((code->Frequency < 36000) || (code->Frequency >38000))
		return false;

	return true;
}

bool IsEqual(IRCode *left, IRCode *rigth)
{
	if (DebugModeIr)
	{
		printf("\n\r");
		if (left)
			printf("Id:0x%08X  Flags:0x%08X  Freq:%05u  Count:%u\n\r", (unsigned int)left->ID, (unsigned int)left->Flags, (unsigned int)left->Frequency, (unsigned int)left->IntervalsCount);
		if (rigth)
			printf("Id:0x%08X  Flags:0x%08X  Freq:%05u  Count:%u\n\r", (unsigned int)rigth->ID, (unsigned int)rigth->Flags, (unsigned int)rigth->Frequency, (unsigned int)rigth->IntervalsCount);
	}

	if (!CheckIRCode(left))
		return false;

	if (!CheckIRCode(rigth))
		return false;

	if (abs(left->IntervalsCount - rigth->IntervalsCount) > LengthDelataMax)
		return false;

	if (left->Frequency != rigth->Frequency)
		return false;

	uint32_t difference = 0;
	uint32_t intervalsDiff = 0;

	const uint32_t intervalsCount = min(left->IntervalsCount, rigth->IntervalsCount);
	for (uint32_t i = 0; i < intervalsCount; i++)
	{
		intervalsDiff = abs(GetTime(left->Intervals[i]) - GetTime(rigth->Intervals[i]));
		if (DebugModeIr)
			printf("%01u:%08u    %01u:%08u    D:%u", (unsigned int)GetValue(left->Intervals[i]), (unsigned int)GetTime(left->Intervals[i]), (unsigned int)GetValue(rigth->Intervals[i]), (unsigned int)GetTime(rigth->Intervals[i]), (unsigned int)intervalsDiff);

		if (GetValue(left->Intervals[i]) != GetValue(rigth->Intervals[i]))
		{
			difference++;
			if (DebugModeIr) printf("  !");
		}

		if ( intervalsDiff > IntervalDelataMax )
		{
			difference++;
			if (DebugModeIr) printf("  *");
		}
		if (DebugModeIr) printf("\n\r");
	}

	if (DebugModeIr) printf("Diff: %u\n\r", (unsigned int) difference);

	return (difference == 0);
}
