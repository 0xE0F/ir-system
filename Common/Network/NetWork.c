#include <stdio.h>

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>

#include "RingBuffer.h"
#include <../IR/IR.h>
#include <PlcTimers.h>
#include <Crc.h>
#include <Terminal.h>

#include <../NetWork/NetWork.h>

#if defined DEVICE_IR_READER
	static const uint8_t DeviceType = 2;
#elif defined DEVICE_IR_WRITER
	static const uint8_t DeviceType = 1;
#else
	#error "Device type not selected"
#endif

enum { RequestDeviceType = 0x0F };

static uint8_t _DevAddr = 0; 							/* Адрес устройства */

static CircularBuffer _NetWorkBuffer;						/* Буфер данных устройства */
static const uint32_t NetWorkBufferSize = STORAGE_PAGE_SIZE + 1 + 1 + 2 + 2 + 2;			/* Размер буфера кода + заголовок команды (ADDR + CODE + PARAM + CRC16) */

static NetworkState _State = Receive;		/* Текущее состояние */

static int IsCrcValid(uint8_t addr, uint8_t cmd, uint16_t crc);

/* Установка состояния */
static void SetMode(NetworkState state)
{
	USART_Cmd(USART2, DISABLE);
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	USART_ClearITPendingBit(USART2, USART_IT_TC);

	if (state == Receive)
	{
		RX_TX_RECEIVE_MODE;
		USART_ITConfig(USART2, USART_IT_TC, DISABLE);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	}
	else
	{
		RX_TX_TRANSMIT_MODE;
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	}

	cbClear(&_NetWorkBuffer);
	_State = state;
	USART_Cmd(USART2, ENABLE);
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
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_WordLength = 8;
	USART_InitStructure.USART_Parity = USART_Parity_No;

//	cbInit(&_NetWorkBuffer, NetWorkBufferSize);
	_DevAddr = address;

	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 8);


	SetMode(Receive);
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE))
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		if (_State == Receive)
		{
			uint8_t ch = USART_ReceiveData(USART2);
			cbWrite(&_NetWorkBuffer, &ch);
			SetTimerValue(RX_TIMEOUT_TIMER, RX_TIMEOUT_VALUE);
		}
	}

	if (USART_GetITStatus(USART2, USART_IT_TC))
	{
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		if (_State == Transmit)
		{
			uint8_t ch;
			if (!cbIsEmpty(&_NetWorkBuffer))
			{
				cbRead(&_NetWorkBuffer, &ch);
				USART_SendData(USART2, ch);
			}
			else
			{
				SetMode(Receive);
			}
		}
	}
}

// Обработка сетевых данных
void NetWorkProcess(void)
{
	if (_State == Receive)
	{
		if ((!cbIsEmpty(&_NetWorkBuffer)) && IsTimeout(RX_TIMEOUT_TIMER))
		{
			uint8_t addr, func;

			cbRead(&_NetWorkBuffer, &addr);
			if (addr != _DevAddr)
			{
				cbClear(&_NetWorkBuffer);
				return;
			}

			cbRead(&_NetWorkBuffer, &func);
			switch (func)
			{
				case RequestDeviceType:
				{
					uint8_t ch;
					uint16_t crc;
					cbRead(&_NetWorkBuffer, &ch);
					crc = ch;
					cbRead(&_NetWorkBuffer, &ch);
					crc |= ((ch << 8) & 0xFF00);

					if (IsCrcValid(addr, func, crc))
					{
						uint8_t answer[] = {addr, RequestDeviceType, DeviceType, 0, 0};
						crc = Crc16(answer, 3);
						answer[3] = crc & 0xFF;
						answer[4] = (crc >> 8) & 0xFF;
						Send(answer, 5);
					}
					else
					{
						cbClear(&_NetWorkBuffer);
					}
					break;
				}

				default:
					break;
			}
		}
	}
	else
	{

	}
}

static int IsCrcValid(uint8_t addr, uint8_t cmd, uint16_t crc)
{
	uint8_t buf[] = {addr, cmd, crc & 0xFF, (crc >> 8) & 0xFF};
	uint16_t calc_crc = Crc16(buf, 2);
	printf("In: %04X  Calc: %04X\n\r", crc, calc_crc);
	return crc == calc_crc;
}
// Передача данных в сеть.
// Если возвращаемое значение не ноль - произошла ошибка (интерфейс занят, недействиетльный буффер, слишком большой размер данных)
uint8_t Send(const uint8_t *pBuf, uint32_t count)
{
	if (!pBuf)
		return 2;
	if (count >= NetWorkBufferSize)
		count = NetWorkBufferSize;

	SetMode(Transmit);

	for(uint32_t i=0; i < count; i++)
	{
		uint8_t ch = *(pBuf + i);
		cbWrite(&_NetWorkBuffer, &ch);
	}

	if ( !cbIsEmpty(&_NetWorkBuffer) )
	{
		uint8_t ch;
		cbRead(&_NetWorkBuffer, &ch);
		USART_SendData(USART2, ch);
	}

	return 0;
}

