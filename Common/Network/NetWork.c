#include <stdio.h>
#include <stdbool.h>

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

enum { cmdDeviceType = 0x0F, cmdOnScan = 0x01, cmdOffScan = 0x02,  cmdSendCode = 0x03, cmdDeleteCode = 0x04, cmdDeleteAll = 0x05, cmdReadCode = 0x07, cmdSaveCode = 0x08, cmdError = 0x0A};

static uint8_t DeviceAddress = 0; 							/* Адрес устройства */
static uint8_t DeviceType;									/* Тип устройства */

static CircularBuffer NetworkBuffer;						/* Буфер данных устройства */
enum {NetWorkBufferSize = 64 };
//STORAGE_PAGE_SIZE + 1 + 1 + 2 + 2 + 2;			/* Размер буфера кода + заголовок команды (ADDR + CODE + PARAM + CRC16) */
static uint8_t WorkBuffer[NetWorkBufferSize];				/* Рабочий буфер устройства */

static NetworkState NetState = Receive;		/* Текущее состояние */

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

	cbClear(&NetworkBuffer);
	NetState = state;
	USART_Cmd(USART2, ENABLE);
}

void InitNetWork(uint32_t baudrate, uint8_t address, uint8_t deviceType)
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

	cbInit(&NetworkBuffer, NetWorkBufferSize);
	DeviceAddress = address;
	DeviceType = deviceType;

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
		if (NetState == Receive)
		{
			uint8_t ch = USART_ReceiveData(USART2);
			cbWrite(&NetworkBuffer, &ch);
			SetTimerValue(RX_TIMEOUT_TIMER, RX_TIMEOUT_VALUE);
		}
	}

	if (USART_GetITStatus(USART2, USART_IT_TC))
	{
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		if (NetState == Transmit)
		{
			uint8_t ch;
			if (!cbIsEmpty(&NetworkBuffer))
			{
				cbRead(&NetworkBuffer, &ch);
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
	if (NetState == Receive)
	{
		if ((!cbIsEmpty(&NetworkBuffer)) && IsTimeout(RX_TIMEOUT_TIMER))
		{
			uint8_t addr, func, ch;
			uint16_t count = 0, crc;

			cbRead(&NetworkBuffer, &addr);
			if (addr != DeviceAddress)
			{
				cbClear(&NetworkBuffer);
				return;
			}

			if (cbIsEmpty(&NetworkBuffer))
				return;

			cbRead(&NetworkBuffer, &func);

			WorkBuffer[count++] = addr;
			WorkBuffer[count++] = func;

			while(!cbIsEmpty(&NetworkBuffer) && (count < NetWorkBufferSize))
			{
				cbRead(&NetworkBuffer, &ch);
				WorkBuffer[count++] = ch;
			}

			crc = WorkBuffer[--count];
			crc |= (WorkBuffer[--count] << 8) & 0xFF00;

			if (crc != Crc16(WorkBuffer, count))
				return;

			switch (func)
			{
				case cmdDeviceType:
					RequestDeviceType();
					break;

				case cmdOnScan:
					break;

				case cmdOffScan:
					break;

				case cmdSaveCode:
					break;

				case cmdReadCode:
					break;

				case cmdSendCode:
					break;

				case cmdDeleteCode:
					break;

				case cmdDeleteAll:
					break;

				default:
					break;
			}
		}
	}
	else
	{

	}
}


// Передача данных в сеть.
// Если возвращаемое значение не ноль - произошла ошибка (интерфейс занят, недействиетльный буффер, слишком большой размер данных)
bool Send(const uint8_t *pBuf, uint32_t count)
{
	if (!pBuf)
		return false;

	if (count >= NetWorkBufferSize)
		count = NetWorkBufferSize;

	SetMode(Transmit);

	for(uint32_t i=0; i < count; i++)
	{
		uint8_t ch = *(pBuf + i);
		cbWrite(&NetworkBuffer, &ch);
	}

	if ( !cbIsEmpty(&NetworkBuffer) )
	{
		uint8_t ch;
		cbRead(&NetworkBuffer, &ch);
		USART_SendData(USART2, ch);
	}

	return true;
}

/* Запрос типа устройства */
void RequestDeviceType(void)
{
	uint8_t answer[] = {DeviceAddress, cmdDeviceType, DeviceType, 0, 0};
	uint16_t crc = Crc16(answer, 3);
	answer[3] = crc & 0xFF;
	answer[4] = (crc >> 8) & 0xFF;
	Send(answer, sizeof(answer)/sizeof(answer[0]));
}

