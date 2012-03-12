#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_exti.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "IR.h"
#include "IRTransmitter.h"

#define CHANNEL_0_PIN_NUMBER (GPIO_Pin_6)
#define CHANNEL_1_PIN_NUMBER (GPIO_Pin_7)
#define CHANNEL_2_PIN_NUMBER (GPIO_Pin_8)
#define CHANNEL_3_PIN_NUMBER (GPIO_Pin_9)

#define CHANNELS_PORT GPIOC

// Максимальный номер канала
const uint32_t MaxChannelNumber = 3;

static IRCode *_SendingCode = NULL;

// Флаги инициализации устройств
static uint32_t _InitFlags = 0;

// Флаг инициализации несущего генератора
static const uint32_t _CarrierTimerInit = 0x1;

// Флаг инициализации выходных каналов
static const uint32_t _OutputChannelInit = 0x2;

// Флаг инициализации несущего генератора
static const uint32_t _WorkTimersInit = 0x4;

// Флаг процесса отправки кода
static __IO uint32_t _IsSending = 0;

// Текущий отправляемый интервал
static __IO uint32_t _SendingIndex = 0;

// остаток интервала
static __IO uint32_t _SendingTime = 0;

// Текущий отправляемый канал
static uint8_t _SendingChannel = ~0;

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
volatile uint32_t IsSending() { return _IsSending != 0; }

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

	_InitFlags |= _CarrierTimerInit;
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

	_InitFlags |= _WorkTimersInit;
}

// Установка несущей частоты
void SetCarrierFrequency(const uint32_t value)
{
	uint16_t period = 0;

	if ((_InitFlags & _CarrierTimerInit) != _CarrierTimerInit)
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

	if (!_SendingCode)
		return;

	if (_SendingTime)
	{
		SetNextPartOutInterval();
	}
	else
	{
		if (++_SendingIndex < _SendingCode->IntervalsCount)
		{
			SetOutInterval(_SendingCode->Intervals[_SendingIndex]);
		}
		else
		{
			TIM_Cmd(TIM7, DISABLE);
			NVIC_DisableIRQ(TIM7_IRQn);
			_IsSending = 0;
			SetOutValueToChannel(_SendingChannel, 0);
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

	_InitFlags |= _OutputChannelInit;
}

// Выдача кода в соответствующий канал
StatusCode SendCodeToChannel(IRCode *code, uint32_t channelID)
{
	if ((_InitFlags & _OutputChannelInit) != _OutputChannelInit)
		InitOuputChannel();

	if ((_InitFlags & _WorkTimersInit) != _WorkTimersInit)
		InitTimers();

	if (_IsSending)
		return StatusCode_Busy;
	if (!code)
		return StatusCode_NullArgumentReference;
	if (channelID > MaxChannelNumber)
		return StatusCode_ArgumentOutOfRange;

	_SendingCode = code;
	_SendingChannel = channelID;

	_SendingIndex = GetFirstNonEmptyIndex(_SendingCode);
	if (_SendingIndex >= _SendingCode->IntervalsCount)
		return StatusCode_InternalError;

	SetCarrierFrequency(GetFrequencyInterval(_SendingCode->Frequency));
	SetOutInterval(_SendingCode->Intervals[_SendingIndex]);
	_IsSending = 1;

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

	for(size_t i=0; i < code->IntervalsCount; i++)
	{
		if (GetTime(code->Intervals[i]) != 0)
			return i;
	}
	return ~0;
}

static void SetOutInterval(const uint32_t interval)
{
	_SendingTime = GetTime(interval);
	SetOutValueToChannel(_SendingChannel, GetValue(interval));
	SetNextPartOutInterval();
}

static void SetNextPartOutInterval(void)
{
	if (_SendingTime > OutTimeoutMax)
	{
		TIM7->CNT = 0;
		_SendingTime -= OutTimeoutMax;
	}
	else
	{
		TIM7->CNT = OutTimeoutMax -_SendingTime;
		_SendingTime = 0;
	}
}

void SetOutValueToChannel(const uint32_t channelId, const uint8_t value)
{
	#warning When you fix me ?

	if (channelId == 0)
	{
		if (value)
		{
			CHANNELS_PORT->BSRR |= CHANNEL_0_PIN_NUMBER;
		}
		else
		{
			CHANNELS_PORT->BRR |= CHANNEL_0_PIN_NUMBER;
		}
	}
	if (channelId == 1)
	{
		if (value)
		{
			CHANNELS_PORT->BSRR |= CHANNEL_1_PIN_NUMBER;
		}
		else
		{
			CHANNELS_PORT->BRR |= CHANNEL_1_PIN_NUMBER;
		}
	}
	if (channelId == 2)
	{
		if (value)
		{
			CHANNELS_PORT->BSRR |= CHANNEL_2_PIN_NUMBER;
		}
		else
		{
			CHANNELS_PORT->BRR |= CHANNEL_2_PIN_NUMBER;
		}
	}
	if (channelId == 3)
	{
		if (value)
		{
			CHANNELS_PORT->BSRR |= CHANNEL_3_PIN_NUMBER;
		}
		else
		{
			CHANNELS_PORT->BRR |= CHANNEL_3_PIN_NUMBER;
		}
	}
}
