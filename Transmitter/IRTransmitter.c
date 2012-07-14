#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "../Common/IR/IR.h"
#include "IRTransmitter.h"

#define CHANNEL_0_PIN_NUMBER (GPIO_Pin_6)
#define CHANNEL_1_PIN_NUMBER (GPIO_Pin_7)
#define CHANNEL_2_PIN_NUMBER (GPIO_Pin_8)
#define CHANNEL_3_PIN_NUMBER (GPIO_Pin_9)

#define CHANNELS_PORT GPIOC

// Максимальный номер канала
const uint32_t MaxChannelNumber = 3;

static IRCode *TransmittingCode = NULL;

// Флаги инициализации устройств
static uint32_t InitFlags = 0;

// Флаг инициализации несущего генератора
static const uint32_t CarrierTimerInit = 0x1;

// Флаг инициализации выходных каналов
static const uint32_t OutputChannelInit = 0x2;

// Флаг инициализации несущего генератора
static const uint32_t WorkTimersInit = 0x4;

// Флаг процесса отправки кода
static volatile bool Transmitting = 0;

// Текущий отправляемый интервал
static __IO uint32_t TransmittingIndex = 0;

// остаток интервала
static __IO uint32_t TransmittingTime = 0;

// Текущий отправляемый канал
static uint8_t TransmittingChannel = ~0;

// Максимальное значение таймаута для отправляемого кода
static const uint32_t OutTimeoutMax = 0x0000FFFF;

// Струтуры настройки таймера на захват длительности импульсов
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

// Инициализация несущего генератора
static void InitCarrierTimer(void); //TIM4

// Инициализация рабочих таймеров
static void InitTimers(void); //TIM7

// Получение первого не пустого значения в спске интервалов
static uint32_t GetFirstNonEmptyIndex(const IRCode *code);

// Установка нового интервала на отправку
static void SetOutInterval(const uint32_t interval);

// Отправка очередной части интервала
static void SetNextPartOutInterval(void);

// Инициализация выходных каналов
static void InitOuputChannel(void);

// Проверка текущего состояния передатчика
volatile bool IsIrTransmitting() { return Transmitting; }

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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	InitFlags |= CarrierTimerInit;

	InitOuputChannel();
}

static void InitTimers(void)
{
	/* Compute the prescaler value */
	uint16_t prescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	// Time base
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, DISABLE);

	NVIC_SetPriority(TIM7_IRQn, 2);

	InitFlags |= WorkTimersInit;
}

/** Установка несущей частоты */
void SetCarrierFrequency(const uint32_t value)
{
	uint16_t period = 0;

	if ((InitFlags & CarrierTimerInit) != CarrierTimerInit)
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


void TIM7_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

	if (!TransmittingCode)
		return;

	if (TransmittingTime) {
		SetNextPartOutInterval();
	} else {
		if (++TransmittingIndex < TransmittingCode->IntervalsCount) {
			SetOutInterval(TransmittingCode->Intervals[TransmittingIndex]);
		} else {
			TIM_Cmd(TIM7, DISABLE);
			NVIC_DisableIRQ(TIM7_IRQn);
			Transmitting = 0;
			SetOutValueToChannel(TransmittingChannel, 0);
		}
	}
}

static void InitOuputChannel(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = CHANNEL_0_PIN_NUMBER | CHANNEL_1_PIN_NUMBER | CHANNEL_2_PIN_NUMBER | CHANNEL_3_PIN_NUMBER;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CHANNELS_PORT, &GPIO_InitStructure);

	InitFlags |= OutputChannelInit;
}

// Выдача кода в соответствующий канал
StatusCode SendCodeToChannel(IRCode *code, uint32_t channelID)
{
	if ((InitFlags & OutputChannelInit) != OutputChannelInit)
		InitOuputChannel();

	if ((InitFlags & WorkTimersInit) != WorkTimersInit)
		InitTimers();

	if (Transmitting)
		return StatusCode_Busy;
	if (!code)
		return StatusCode_NullArgumentReference;
	if (channelID > MaxChannelNumber)
		return StatusCode_ArgumentOutOfRange;

	TransmittingCode = code;
	TransmittingChannel = channelID;

	TransmittingIndex = GetFirstNonEmptyIndex(TransmittingCode);
	if (TransmittingIndex >= TransmittingCode->IntervalsCount)
		return StatusCode_InternalError;

	SetCarrierFrequency(GetFrequencyInterval(TransmittingCode->Frequency));
	SetOutInterval(TransmittingCode->Intervals[TransmittingIndex]);
	Transmitting = 1;

	TIM_Cmd(TIM7, ENABLE);
	NVIC_EnableIRQ(TIM7_IRQn);

	return StatusCode_Ok;
}

uint32_t GetFirstNonEmptyIndex(const IRCode *code)
{
	if (!code)
		return ~0;
	if (code->IntervalsCount == 0)
		return ~0;

	for(size_t i=0; i < code->IntervalsCount; i++) {
		if (GetTime(code->Intervals[i]) != 0)
			return i;
	}
	return ~0;
}

static void SetOutInterval(const uint32_t interval)
{
	TransmittingTime = GetTime(interval);
	SetOutValueToChannel(TransmittingChannel, GetValue(interval));
	SetNextPartOutInterval();
}

static void SetNextPartOutInterval(void)
{
	if (TransmittingTime > OutTimeoutMax) {
		TIM7->CNT = 0;
		TransmittingTime -= OutTimeoutMax;
	} else {
		TIM7->CNT = OutTimeoutMax -TransmittingTime;
		TransmittingTime = 0;
	}
}

void SetOutValueToChannel(const uint32_t channelId, const uint8_t value)
{
	uint16_t pin = 0;
	switch(channelId)
	{
		case 0:
			pin = CHANNEL_0_PIN_NUMBER;
			break;
		case 1:
			pin = CHANNEL_1_PIN_NUMBER;
			break;
		case 2:
			pin = CHANNEL_2_PIN_NUMBER;
			break;
		case 3:
			pin = CHANNEL_3_PIN_NUMBER;
			break;

	}

	if (value) {
		CHANNELS_PORT->BSRR |= pin;
	} else {
		CHANNELS_PORT->BRR |= pin;
	}
}
