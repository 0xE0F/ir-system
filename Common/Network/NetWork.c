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

static uint8_t DeviceAddress = 0; 							/* Адрес устройства */
static uint8_t DeviceType;									/* Тип устройства */

static CircularBuffer NetworkBuffer;						/* Буфер данных устройства */
enum {NetWorkBufferSize = STORAGE_PAGE_SIZE + 64 };
//STORAGE_PAGE_SIZE + 1 + 1 + 2 + 2 + 2;			/* Размер буфера кода + заголовок команды (ADDR + CODE + PARAM + CRC16) */
static uint8_t WorkBuffer[NetWorkBufferSize];				/* Рабочий буфер устройства */

static NetworkState NetState = Receive;		/* Текущее состояние */

static bool TransmitComplite = false;

/* Состоянее драйвер отправки */
NetworkState GetNetworkState(void) { return NetState ;}

/* Получение адреса устройства */
uint8_t GetDeviceAddress(void) { return DeviceAddress; }

/* Получение типа устройства */
uint8_t GetDeviceType(void) { return DeviceType; }

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
		NetworkLedOff();
	}
	else
	{
		RX_TX_TRANSMIT_MODE;
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		USART_ITConfig(USART2, USART_IT_TC, ENABLE);
		NetworkLedOn();
		TransmitComplite = false;
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

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

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = RX_CONTROL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RX_CONTROL_PORT, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = TX_CONTROL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TX_CONTROL_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = 8;
	USART_InitStructure.USART_Parity = USART_Parity_No;

	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 8);

	cbInit(&NetworkBuffer, NetWorkBufferSize);
	DeviceAddress = address;
	DeviceType = deviceType;

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
				SetTimerValue(TRANSMIT_TIMER, TRANSMIT_VALUE);
				TransmitComplite = true;
//				SetMode(Receive);
			}
		}
	}
}

// Обработка сетевых данных
void ProcessNetwork(void)
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

			crc = (WorkBuffer[--count] << 8) & 0xFF00;
			crc |= WorkBuffer[--count] & 0x00FF;

			if (crc != Crc16(WorkBuffer, count, 0xFFFF))
				return;

			switch (func)
			{
				case cmdDeviceType:
					RequestDeviceType();
					break;

				case cmdOnScan:
				{
					uint16_t id = WorkBuffer[3];
					uint8_t mode = WorkBuffer[4];
					id |= (WorkBuffer[2] << 8) & 0xFF00;
					RequestOnScan(id, mode);
					break;
				}


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
		if ((TransmitComplite) && (IsTimeout(TRANSMIT_TIMER)))
		{
			SetMode(Receive);
		}
	}
}

/* Запрос типа устройства */
void RequestDeviceType(void)
{
	uint8_t answer[] = {DeviceAddress, cmdDeviceType, DeviceType};
	Answer(answer, sizeof(answer)/sizeof(answer[0]), NULL, 0, true);
}

/* Ответ на запрос */
/* Формируется путем сложения заголовка и тела сообщения
 * calcCrc определяет нужли ли добавлять в конце CRC
 * */
bool Answer(uint8_t *header, const size_t headerSize, uint8_t *msg, const size_t msgSize, const bool calcCrc)
{
	size_t totalSize = headerSize + msgSize + (calcCrc ? sizeof(uint16_t) : 0);

	if (!header)
		return false;

	if (totalSize > NetWorkBufferSize)
		return false;

	SetMode(Transmit);

	for (size_t i = 0; i < headerSize; i++)
		cbWrite(&NetworkBuffer, (header+i));

	for (size_t i = 0; i < msgSize; i++)
		cbWrite(&NetworkBuffer, (msg+i));

	if (calcCrc)
	{
		uint16_t crc = Crc16(header, headerSize, 0xFFFF);
		crc = Crc16(msg, msgSize, crc);
		uint8_t bCrc = crc & 0xFF;
		cbWrite(&NetworkBuffer, &bCrc);
		bCrc = (crc >> 8)& 0xFF;
		cbWrite(&NetworkBuffer, &bCrc);
	}

	if ( !cbIsEmpty(&NetworkBuffer) )
	{
		uint8_t ch;
		cbRead(&NetworkBuffer, &ch);
		USART_SendData(USART2, ch);
	}

//	printf("[Send :%u]\n\r", totalSize);
	return true;
}

/* Ответ кодом ошибки */
bool AnswerError(Errors error)
{
	uint8_t errorHeader[] = {GetDeviceAddress(), cmdError, error};
	return Answer(errorHeader, sizeof(errorHeader) / sizeof(errorHeader[0]), NULL, 0, true);
}


