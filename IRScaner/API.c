#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "API.h"

// Стандартные значения частот
static uint32_t StandartFrequency[] = {30000, 32000, 34000, 36000, 38000, 40000, 42000, 44000, 46000, 48000, 50000, 52000, 54000, 56000, 58000, 60000};
// Длина буфера стандартных частот
static uint32_t const StandartFrequencyCount = sizeof(StandartFrequency) / sizeof(uint32_t);

static void InitCarrierTimer(void);

static uint32_t IsCarrierTimerInit = 0;

const uint32_t MaxChannelNumber = 3;

extern signed int printf(const char *pFormat, ...);

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

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

// Установка несущей частоты
void SetCarrierFrequency(const uint32_t value)
{
	uint16_t period = 0;

	if (!IsCarrierTimerInit)
		InitCarrierTimer();

	TIM_Cmd(TIM4, DISABLE);
	TIM_ARRPreloadConfig(TIM4, DISABLE);

	period = (uint16_t) (SystemCoreClock / value) - 1;  // Период
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_Pulse = (uint16_t) (((uint32_t) 5 * (period - 1)) / 10);
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

static void InitCarrierTimer(void)
{
	/* Compute the prescaler value */
	uint16_t period = (uint16_t) (SystemCoreClock / 30000) - 1;  // Период
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// Time base
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* OCM */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = (uint16_t) (((uint32_t) 5 * (period - 1)) / 10);
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Initialize LED which connected to PC8,9, Enable the Clock*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	IsCarrierTimerInit = 1;
}

// Запись кода со сканера
uint32_t StartRecord(IRCode *irCode)
{

}

// отладчный вывод кода
void DebugPrint(IRCode *code)
{
	if (!code)
	{
		printf("Illegal IRCode pointer\n\r");
		return;
	}

	printf("ID: 0x%08X\n\r", code->ID);
	printf("Frequency: %05d Hz\n\r", code->Frequency);
	printf("Flags: 0x%04X\n\r", code->Flags);
	printf("Count: %u\n\r", code->IntervalsCount);

	if (code->IntervalsCount > PULSES_MAX)
	{
		printf("Intervals count out if range\n\r");
		return;
	}

	for(uint32_t i=0; i < code->IntervalsCount; i++)
		printf("[%03u] [%02X] [%08u]\n\r", i, code->Intervals[i].Value, code->Intervals[i].Time);

	printf("==>\n\r\n\r");
}


