#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_exti.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "API.h"

static const uint32_t DetectFreqMin = 29000;
static const uint32_t DetectFreqMax = 60500;

// Стандартные значения частот
static const uint32_t StandartFrequency[] = {30000, 32000, 34000, 36000, 38000, 40000, 42000, 44000, 46000, 48000, 50000, 52000, 54000, 56000, 58000, 60000};
// Длина буфера стандартных частот
static const uint32_t StandartFrequencyCount = sizeof(StandartFrequency) / sizeof(uint32_t);

// Флаги инициализации устройств
static uint32_t _InitFlags = 0;
// Флаг инициализации несущего генератора
static const uint32_t IsCarrierTimerInit = 1;
// Флаг инициализации несущего генератора
static const uint32_t IsWorkTimersInit = 2;
// Флаг инициализации внешних прерываний
static const uint32_t IsEXTIInit = 4;

// Максимальный номер канала
const uint32_t MaxChannelNumber = 3;

static IRCode *_WorkCode = NULL;
static __IO uint32_t _IsScanning = 0;

static __IO uint32_t _CurrentTime = 0;		// Текущее время
static const uint32_t TimeToStop = 5000000; // Время принудительной остановки, мкс
static const uint32_t TimeIncrement = 0xFFFF / 1000000; // Размер интервала: Разрядность регистра 16 бит / таймер включен ан период в 1 мкс.

static __IO uint16_t IC2Value = 0;
static __IO uint16_t DutyCycle = 0;
static __IO uint32_t Frequency = 0;
static __IO uint32_t DetectFreq = 0;

static __IO uint8_t _SilentCounter = 0;			// Счетчик интервал тишины
static const uint8_t SilentDetectFactor = 15;  // Коэффициент при котором считается что далее сигнала нет
static __IO uint32_t _SilentStartIndex = 0; // Индекс с которого зафиксирована тишина

// Инициализация несущего генератора
static void InitCarrierTimer(void);
// Инициализация рабочих таймеров
static void InitWorkTimers(void);
// Инициализация внешних прерываний
static void InitEXTI(void);

extern signed int printf(const char *pFormat, ...);


#define GET_IR_DATA_BIT (GPIOC->IDR & GPIO_Pin_0)

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

	if ((_InitFlags & IsCarrierTimerInit) != IsCarrierTimerInit)
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	_InitFlags |= IsCarrierTimerInit;
}

static void InitWorkTimers(void)
{
	/* Compute the prescaler value */
	uint16_t prescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_ICInitTypeDef  TIM_ICInitStructure;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

	/* Select the TIM3 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 3);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 4);

	_InitFlags |= IsWorkTimersInit;
}


static void InitEXTI(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_SetPriority(EXTI0_IRQn, 1);

	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	_InitFlags |= IsEXTIInit;
}

// Запись кода со сканера
void Scan(IRCode *irCode)
{
	if ((_InitFlags & IsWorkTimersInit) != IsWorkTimersInit)
		InitWorkTimers();
	if ((_InitFlags & IsEXTIInit) != IsEXTIInit)
		InitEXTI();
	if (irCode == NULL)
		return;

	_WorkCode = irCode;
	_WorkCode->Frequency = 0;
	_WorkCode->IntervalsCount = 0;
	_WorkCode->Intervals[_WorkCode->IntervalsCount++].Value = GET_IR_DATA_BIT;
	_CurrentTime = 0;
	_SilentCounter = 0;
	_SilentStartIndex = 0;
	_IsScanning = 1;

	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM3, ENABLE);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

// Остановка записи
void StopScan()
{
	NVIC_DisableIRQ(EXTI0_IRQn);
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
	if (_WorkCode)
	{
		_WorkCode->Frequency = GetFrequencyInterval(DetectFreq);
	}
	_IsScanning = 0;
}

volatile uint32_t IsScanning()
{
	return _IsScanning != 0;
}

void EXTI0_IRQHandler()
{
	TIM_Cmd(TIM2, DISABLE);
	if (_WorkCode)
	{
		uint32_t intervalsCount = _WorkCode->IntervalsCount;
		_WorkCode->Intervals[intervalsCount-1].Time = TIM2->CNT;
		_WorkCode->Intervals[intervalsCount++].Value = GET_IR_DATA_BIT;
		_WorkCode->IntervalsCount = intervalsCount;
		TIM2->CNT =0;
		TIM_Cmd(TIM2, ENABLE);
		if (intervalsCount >= INTERVALS_MAX)
		{
			StopScan();
		}
		_SilentCounter = 0;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void TIM2_IRQHandler()
{
	if (_WorkCode)
	{
		uint32_t intervalsCount = _WorkCode->IntervalsCount;
		_WorkCode->Intervals[intervalsCount-1].Time = 0xFFFF;
		_WorkCode->Intervals[intervalsCount].Value = _WorkCode->Intervals[intervalsCount-1].Value;
		intervalsCount++;
		_WorkCode->IntervalsCount = intervalsCount;

		_CurrentTime += TimeIncrement;
		if ((_CurrentTime > TimeToStop) || (intervalsCount >= INTERVALS_MAX))
		{
			StopScan();
		}

//		if (_SilentCounter >= SilentDetectFactor)
//		{
//			_WorkCode->IntervalsCount = SilentDetectFactor;
//			StopScan();
//		}
		if (_SilentCounter == 0)
		{
			_SilentStartIndex = intervalsCount;
		}
		_SilentCounter++;
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM3_IRQHandler(void)
{
  /* Clear TIM3 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM3);

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = (TIM_GetCapture1(TIM3) * 100) / IC2Value;

    /* Frequency computation */
    Frequency = SystemCoreClock / IC2Value;
    if ((Frequency > DetectFreqMin) && (Frequency < DetectFreqMax))
    {
    	DetectFreq = Frequency;
    }
  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }
}

// отладчный вывод кода
void DebugPrint(IRCode *code)
{
	if (!code)
	{
		printf("Illegal IRCode pointer\n\r");
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
		printf("[%03u] [%02X] [%08u]\n\r", (unsigned int)i, (unsigned char)(code->Intervals[i].Value), (unsigned short)(code->Intervals[i].Time));

	printf("==>\n\r\n\r");
}


