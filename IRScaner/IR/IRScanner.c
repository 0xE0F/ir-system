#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_exti.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "IR.h"
#include "IRScanner.h"

// Флаги инициализации устройств
static uint32_t _InitFlags = 0;

// Флаг инициализации внешних прерываний
static const uint32_t _EXTIInit = 0x1;

// Флаг инициализации несущего генератора
static const uint32_t _WorkTimersInit = 0x2;

static IRCode *_ScanningCode = NULL;

static __IO uint32_t _IsScanning = 0;			// Флаг процесса сканирования

static __IO uint32_t _CurrentTime = 0;			// Текущее время
static const uint32_t TimeToStop = 5000000; 	// Время принудительной остановки, мкс
static const uint32_t TimeIncrement = 0xFFFF; 	// таймер включен на период в 1 мкс.

static __IO uint16_t _IC2Value = 0;
static __IO uint16_t _DutyCycle = 0;
static __IO uint32_t _SharedFrequency = 0;
static __IO uint32_t _DetectFrequency = 0;

#define GET_IR_DATA_BIT ((GPIOC->IDR & GPIO_Pin_0) ? 0 : 1)

// Инициализация внешних прерываний
static void InitEXTI(void);

// Инициализация рабочих таймеров
static void InitTimers(void); //TIM2 - in, TIM3 - freq meas

// Остановка сканирования
static void StopScan();

volatile uint32_t IsScanning() { return _IsScanning != 0; }

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

	NVIC_SetPriority(EXTI0_IRQn, 1);

	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	_InitFlags |= _EXTIInit;
}

static void InitTimers(void)
{
	/* Compute the prescaler value */
	uint16_t prescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);


	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

	/* Select the TIM3 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 4);
	NVIC_SetPriority(TIM3_IRQn, 3);

	_InitFlags |= _WorkTimersInit;
}

// Запись кода со сканера
void Scan(IRCode *irCode)
{
	if ((_InitFlags & _WorkTimersInit) != _WorkTimersInit)
		InitTimers();

	if ((_InitFlags & _EXTIInit) != _EXTIInit)
		InitEXTI();

	if (irCode == NULL)
		return;

	_ScanningCode = irCode;

	memset(_ScanningCode, 0, sizeof(IRCode));

	SetValue(&(_ScanningCode->Intervals[_ScanningCode->IntervalsCount]), GET_IR_DATA_BIT);

	_CurrentTime = 0;
	_DetectFrequency = 0;

	TIM_SetCounter(TIM2, 0);
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, DISABLE);

	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearFlag(EXTI_Line0);

	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);

	EXTI->IMR |= EXTI_IMR_MR0;

	_IsScanning = 1;
}

// Остановка записи
static void StopScan()
{
	NVIC_DisableIRQ(TIM3_IRQn);
	NVIC_DisableIRQ(EXTI0_IRQn);

	EXTI->IMR &= ~EXTI_IMR_MR0;

	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM3, DISABLE);

	if (_ScanningCode)
	{
		_ScanningCode->Frequency = GetFrequencyInterval(_DetectFrequency);
		_ScanningCode->Flags = _DutyCycle;
	}

	_IsScanning = 0;
}

void EXTI0_IRQHandler()
{
	if ((_ScanningCode) && (_IsScanning))
	{
		TIM_Cmd(TIM3, DISABLE);

		uint32_t index = _ScanningCode->IntervalsCount;
		uint32_t timeout = TIM_GetCounter(TIM3) & 0x0000FFFF;
		uint32_t time = GetTime(_ScanningCode->Intervals[index]);

		SetTime( &(_ScanningCode->Intervals[index++]),time+timeout);
		SetTime( &(_ScanningCode->Intervals[index]), 0);
		SetValue( &(_ScanningCode->Intervals[index]), GET_IR_DATA_BIT);
		_ScanningCode->IntervalsCount = index;

		TIM_SetCounter(TIM3, 0);

		TIM_Cmd(TIM3, ENABLE);
		_CurrentTime += timeout;
		if ((_CurrentTime > TimeToStop) || (_ScanningCode->IntervalsCount >= INTERVALS_MAX))
		{
			StopScan();
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void TIM3_IRQHandler()
{
	if ((_ScanningCode) && (_IsScanning))
	{
		uint32_t time = GetTime(_ScanningCode->Intervals[_ScanningCode->IntervalsCount]);
		SetTime(&(_ScanningCode->Intervals[_ScanningCode->IntervalsCount]), time + UINT16_MAX);
		_CurrentTime += TimeIncrement;
		if ((_CurrentTime > TimeToStop) || (_ScanningCode->IntervalsCount >= INTERVALS_MAX))
		{
			StopScan();
		}
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void TIM2_IRQHandler(void)
{
	/* Clear TIM3 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

  /* Get the Input Capture value */
  _IC2Value = TIM_GetCapture2(TIM2);

  if (_IC2Value != 0)
  {
    /* Duty cycle computation */
    _DutyCycle = (TIM_GetCapture1(TIM2) * 100) / _IC2Value;

    /* Frequency computation */
    _SharedFrequency = SystemCoreClock / _IC2Value;
    if ((_SharedFrequency > DetectFreqMin) && (_SharedFrequency < DetectFreqMax))
    {
    	_DetectFrequency = _SharedFrequency;
    }
  }
  else
  {
    _DutyCycle = 0;
    _SharedFrequency = 0;
  }
}
