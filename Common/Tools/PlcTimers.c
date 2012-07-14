#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>

#include "PlcTimers.h"

enum { TimersCount = 7};	/* Количество PLC таймеров */

static uint32_t _Timers[TimersCount];

// Инициализация тамеров
void InitPlcTimers(void)
{
	uint16_t prescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 100; /* 1000 000 / 100 = 10 kHz */
	TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn, 8);

	TIM_Cmd(TIM6, ENABLE);
}

// Проверка на таймаут тамера с timerId
uint8_t IsTimeout(const uint8_t timerId)
{
	return IsTimeoutEx(timerId, 0);
}

// Проверка на таймаут таймера с timerId и автоматической перезагрузкой значения
uint8_t IsTimeoutEx(const uint8_t timerId, const uint32_t reloadValue)
{
	uint8_t res = 0;

	if (timerId >= TimersCount)
		return res;

	uint32_t oldValue;
	do
	{
		oldValue = __LDREXW(_Timers + timerId);
		if (oldValue == 0)
		{
			res = 1;
		}
		else
		{
			__CLREX();
			return 0;
		}
	} while(__STREXW(reloadValue, _Timers +timerId));
	return res;
}

// Установка значения таймера
void SetTimerValue(const uint8_t timerId, const uint32_t value)
{
	if (timerId >= TimersCount)
		return;
	do
	{
		__LDREXW(_Timers + timerId);
	} while(__STREXW(value, _Timers +timerId));
}

void TIM6_IRQHandler()
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		for (uint32_t i=0; i < TimersCount; i++)
		{
			if (_Timers[i] != 0)
				_Timers[i]--;
		}
	}
}
