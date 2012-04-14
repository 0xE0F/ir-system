#include <stdio.h>

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>

#include "RingBuffer.h"
#include <IR/IR.h>

#include <NetWork/NetWork.h>

static uint8_t _DevAddr = 0; 							/* Адрес устройства */

static RingBuffer *_NetWorkBuffer;						/* Буфер данных устройства */
static const uint32_t NetWorkBufferSize = STORAGE_PAGE_SIZE + 1 + 1 + 2 + 2 + 2;			/* Размер буфера кода + заголовок команды (ADDR + CODE + PARAM + CRC16) */

static NetworkState _State = Receive;

static void SetMode(NetworkState state)
{
	if (state == Receive)
	{
		RX_TX_RECEIVE_MODE;
	}
	else
	{
		RX_TX_TRANSMIT_MODE;
	}

	RB_Clear(_NetWorkBuffer);
	_State = state;
}

void InitNetWork(uint32_t baudrate, uint8_t address)
{
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = 8;
	USART_InitStructure.USART_Parity = USART_Parity_No;

	_NetWorkBuffer = MakeRingBuffer(NetWorkBufferSize);
	_DevAddr = address;

	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 8);

	USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE))
	{
		if (_State == Receive)
			RB_Write(_NetWorkBuffer, USART_ReceiveData(USART2));
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}

	if (USART_GetITStatus(USART2, USART_IT_TC))
	{
		if (_State == Transmit)
		{
			uint8_t ch;
			if (RB_Read(_NetWorkBuffer, &ch))
				USART_SendData(USART2, ch);
		}
		USART_ClearITPendingBit(USART2, USART_IT_TC);
	}
}

// Обработка сетевых данных
void NetWorkProcess(void)
{
	uint8_t c;
	if ((!RB_IsEmpty(_NetWorkBuffer)) && RB_Read(_NetWorkBuffer, &c))
		USART_SendData(USART2, c);

}

// Передача данных в сеть.
// Если возвращаемое значение не ноль - произошла ошибка (интерфейс занят, недействиетльный буффер, слишком большой размер данных)
uint8_t Send(const uint8_t *pBuf, const uint32_t count)
{

	return 1;
}

