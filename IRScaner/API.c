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

// ����������� �������� ������
static const uint32_t StandartFrequency[] = {30000, 32000, 34000, 36000, 38000, 40000, 42000, 44000, 46000, 48000, 50000, 52000, 54000, 56000, 58000, 60000};
// ����� ������ ����������� ������
static const uint32_t StandartFrequencyCount = sizeof(StandartFrequency) / sizeof(uint32_t);

// ����� ������������� ���������
static uint32_t _InitFlags = 0;
// ���� ������������� �������� ����������
static const uint32_t IsCarrierTimerInit = 0x1;
// ���� ������������� �������� ����������
static const uint32_t IsWorkTimersInit = 0x2;
// ���� ������������� ������� ����������
static const uint32_t IsEXTIInit = 0x4;
// ���� ������������� ��������� �������
static const uint32_t IsOutputTimerInit = 0x8;
// ���� ������������� �������� �������
static const uint32_t IsOutputChannelInit = 0x10;

// ������������ ����� ������
const uint32_t MaxChannelNumber = 3;

static IRCode *_ScanningCode = NULL;
static IRCode *_SendingCode = NULL;

// ���� �������� ������������
static __IO uint32_t _IsScanning = 0;
// ���� �������� �������� ����
static __IO uint32_t _IsSending = 0;

// ������� ������������ ��������
static __IO uint32_t _SendingIndex = 0;
// ������� ���������
static __IO uint32_t _SendingTime = 0;
// ������� ������������ �����
static uint8_t _SendingChannel = ~0;
// ������������ �������� �������� ��� ������������� ����
static const uint32_t OutTimeoutMax = 0x0000FFFF;

static __IO uint32_t _CurrentTime = 0;		// ������� �����
static const uint32_t TimeToStop = 5000000; // ����� �������������� ���������, ���
static const uint32_t TimeIncrement = 0xFFFF; // ������ ������� �� ������ � 1 ���.

static __IO uint16_t _IC2Value = 0;
static __IO uint16_t _DutyCycle = 0;
static __IO uint32_t _SharedFrequency = 0;
static __IO uint32_t _DetectFrequency = 0;

// �������� ��������� ������� �� ������ ������������ ���������
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

// ������������� �������� ����������
static void InitCarrierTimer(void); //TIM4
// ������������� ������� ��������
static void InitWorkTimers(void); //TIM1 - out, TIM2 - in, TIM3 - freq meas
// ������������� ������� ����������
static void InitEXTI(void);

#define GET_IR_DATA_BIT ((GPIOC->IDR & GPIO_Pin_0) ? 0 : 1)

#define CHANNEL_0_PIN_NUMBER (GPIO_Pin_6)
#define CHANNEL_1_PIN_NUMBER (GPIO_Pin_7)
#define CHANNEL_2_PIN_NUMBER (GPIO_Pin_8)
#define CHANNEL_3_PIN_NUMBER (GPIO_Pin_9)

#define CHANNELS_PORT GPIOC

//��������� �������� �������
inline void SetTime(uint32_t *interval, const uint32_t time);
// ��������� �������� ���������
inline void SetValue(uint32_t *interval, const uint8_t value);
// ��������� �������� ������� ���������
inline uint32_t GetTime(uint32_t interval);
// ��������� �������� ���������
inline uint8_t GetValue(uint32_t interval);
// ����� �������
const uint32_t TimeMask = 0x7FFFFFFF;
// ����� ��������
const uint32_t ValueMask = 0x80000000;

// ��������� ������� �� ������� �������� � ����� ����������
static uint32_t GetFirstNonEmptyIndex(const IRCode *code);
// ��������� ������ ��������� �� ��������
static void SetOutInterval(const uint32_t interval);
// �������� ��������� ����� ���������
static void SetNextPartOutInterval(void);

// ������������� �������� �������
static void InitOuputChannel(void);

extern signed int printf(const char *pFormat, ...);

inline void SetTime(uint32_t *interval, const uint32_t time)
{
	(*interval) &= ~TimeMask;
	(*interval) = (*interval | (time & TimeMask));
}

inline void SetValue(uint32_t *interval, const uint8_t value)
{
	(*interval) = (value > 0) ? (*interval | ValueMask) : (*interval & (~ValueMask));
}

inline uint32_t GetTime(uint32_t interval)
{
	return interval & TimeMask;
}

inline uint8_t GetValue(uint32_t interval)
{
	return (interval & ValueMask) == ValueMask;
}

// ��������� ����������� ������� �� ��������� 30 ��� - 60 ���
// 0 - ��� ���������
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

// ��������� ������� �������
void SetCarrierFrequency(const uint32_t value)
{
	uint16_t period = 0;

	if ((_InitFlags & IsCarrierTimerInit) != IsCarrierTimerInit)
		InitCarrierTimer();

	TIM_Cmd(TIM4, DISABLE);
	TIM_ARRPreloadConfig(TIM4, DISABLE);

	period = (uint16_t) (SystemCoreClock / value) - 1;  // ������
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
	uint16_t period = (uint16_t) (SystemCoreClock / 30000) - 1;  // ������
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

	//--
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	// Time base
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, DISABLE);

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 3);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 4);
	NVIC_SetPriority(TIM7_IRQn, 2);

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

// ������ ���� �� �������
void Scan(IRCode *irCode)
{
	if ((_InitFlags & IsWorkTimersInit) != IsWorkTimersInit)
		InitWorkTimers();

	if ((_InitFlags & IsEXTIInit) != IsEXTIInit)
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
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM2, DISABLE);

	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearFlag(EXTI_Line0);

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);

	EXTI->IMR |= EXTI_IMR_MR0;

	_IsScanning = 1;
}

// ��������� ������
void StopScan()
{
	NVIC_DisableIRQ(EXTI0_IRQn);
	NVIC_DisableIRQ(TIM2_IRQn);

	EXTI->IMR &= ~EXTI_IMR_MR0;

	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM3, DISABLE);

	if (_ScanningCode)
	{
		_ScanningCode->Frequency = GetFrequencyInterval(_DetectFrequency);
	}

	_IsScanning = 0;
}

volatile uint32_t IsScanning()
{
	return _IsScanning != 0;
}

volatile uint32_t IsSending()
{
	return _IsSending != 0;
}

void EXTI0_IRQHandler()
{
	if ((_ScanningCode) && (_IsScanning))
	{
		TIM_Cmd(TIM2, DISABLE);

		uint32_t index = _ScanningCode->IntervalsCount;
		uint32_t timeout = TIM_GetCounter(TIM2) & 0x0000FFFF;
		uint32_t time = GetTime(_ScanningCode->Intervals[index]);

		SetTime( &(_ScanningCode->Intervals[index++]),time+timeout);
		SetTime( &(_ScanningCode->Intervals[index]), 0);
		SetValue( &(_ScanningCode->Intervals[index]), GET_IR_DATA_BIT);
		_ScanningCode->IntervalsCount = index;

		TIM_SetCounter(TIM2, 0);

		TIM_Cmd(TIM2, ENABLE);
		_CurrentTime += timeout;
		if ((_CurrentTime > TimeToStop) || (_ScanningCode->IntervalsCount >= INTERVALS_MAX))
		{
			StopScan();
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void TIM2_IRQHandler()
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
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM3_IRQHandler(void)
{
  /* Clear TIM3 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

  /* Get the Input Capture value */
  _IC2Value = TIM_GetCapture2(TIM3);

  if (_IC2Value != 0)
  {
    /* Duty cycle computation */
    _DutyCycle = (TIM_GetCapture1(TIM3) * 100) / _IC2Value;

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
// ��������� ����� ����
void DebugPrint(IRCode *code)
{
	if (!code)
	{
		if (!_ScanningCode)
			return;
		code = _ScanningCode;
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

static void InitOuputChannel(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = CHANNEL_0_PIN_NUMBER | CHANNEL_1_PIN_NUMBER | CHANNEL_2_PIN_NUMBER | CHANNEL_3_PIN_NUMBER;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CHANNELS_PORT, &GPIO_InitStructure);

	_InitFlags |= IsOutputChannelInit;
}

// ������ ���� � ��������������� �����
// 0 - ������� � �������
// !0 - ��� ������ (��� �� ������ ��� ���������� ����� ������)
StatusCode SendCodeToChannel(IRCode *code, uint32_t channelID)
{
	if ((_InitFlags & IsOutputChannelInit) != IsOutputChannelInit)
		InitOuputChannel();

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
