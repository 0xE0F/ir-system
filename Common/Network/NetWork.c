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

static uint8_t DeviceType;									/* ��� ���������� */
static uint8_t DeviceAddress = 0; 							/* ����� ���������� */

static CircularBuffer NetworkBuffer;						/* ����� ������ ���������� */
enum {NetWorkBufferSize = STORAGE_PAGE_SIZE + 1024 };
static uint8_t WorkBuffer[NetWorkBufferSize];				/* ������� ����� ���������� */

static volatile NetworkState NetState = Receive;			/* ������� ��������� */

static bool TransmitComplite = false;						/* ��������� �������� */

enum {NetworkHandlersMaxCount = 32 };
static network_action_t NetworkHandlers[NetworkHandlersMaxCount];

static void InitNetworkHandlers(void)
{
	for (size_t index = 0; index < NetworkHandlersMaxCount; ++index) {
		NetworkHandlers[index] = NULL;
	}
}

/* ���������� �� ��� ������ ?*/
bool IsNetworkBusy(void) { return NetState == Transmit; }

/* ��������� ������� �������� */
NetworkState GetNetworkState(void) { return NetState ;}

/* ��������� ������ ���������� */
uint8_t GetDeviceAddress(void) { return DeviceAddress; }

/* ��������� ���� ���������� */
uint8_t GetDeviceType(void) { return DeviceType; }

/* ��������� ��������� */
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

void InitNetWork(uint32_t baudrate, ParityMode parity, uint8_t address, uint8_t deviceType)
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

	switch(parity) {
		case pmEven:
			USART_InitStructure.USART_WordLength =USART_WordLength_9b;
			USART_InitStructure.USART_Parity = USART_Parity_Even;
			break;

		case pmOdd:
			USART_InitStructure.USART_WordLength =USART_WordLength_9b;
			USART_InitStructure.USART_Parity = USART_Parity_Odd;
			break;

		case pmNone:
		default:
			USART_InitStructure.USART_WordLength =USART_WordLength_8b;
			USART_InitStructure.USART_Parity = USART_Parity_No;
	}

	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 8);

	cbInit(&NetworkBuffer, NetWorkBufferSize);
	DeviceAddress = address;
	DeviceType = deviceType;
	InitNetworkHandlers();
	RegisterNetworkHandler(cmdDeviceType, DeviceTypeHandler);

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

/** ����������� � ������ ������������ */
void RegisterNetworkHandler(uint8_t func, network_action_t action)
{
	if (func < NetworkHandlersMaxCount) {
		NetworkHandlers[func] = action;
	}
}

void UnregisterNetworkHandler(uint8_t func)
{
	if (func < NetworkHandlersMaxCount) {
		NetworkHandlers[func] = NULL;
	}
}

// ��������� ������� ������
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

			if (func < NetworkHandlersMaxCount) {
				network_action_t action = NetworkHandlers[func];
				if (action != NULL) {
					 action(WorkBuffer + 2, count);
				}
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

/* ������ ���� ���������� */
void DeviceTypeHandler(uint8_t *buffer, const size_t count)
{
	uint8_t answer[] = {DeviceAddress, cmdDeviceType, DeviceType};
	Answer(answer, sizeof(answer)/sizeof(answer[0]), NULL, 0, true);
}

/* ����� �� ������ */
/* ����������� ����� �������� ��������� � ���� ���������
 * calcCrc ���������� ����� �� ��������� � ����� CRC
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

	return true;
}

/* ����� ����� ������ */
bool AnswerError(Errors error)
{
	uint8_t errorHeader[] = {GetDeviceAddress(), cmdError, error};
	return Answer(errorHeader, sizeof(errorHeader) / sizeof(errorHeader[0]), NULL, 0, true);
}

/** ������ ����� �� ������ */
uint16_t GetUInt16(uint8_t *buf)
{
	uint16_t value = *buf;
	return value | ( ( *(buf + 1) << 8 ) & 0xFF00 );
}

uint16_t GetUInt32(uint8_t *buf)
{
	uint32_t value = *buf;
	value |= ( *(buf + 1) << 8 ) & 0x0000FF00;
	value |= ( *(buf + 2) << 16 ) & 0x00FF0000;

	return value | ( ( *(buf + 3) << 24 ) & 0xFF000000 );
}

void StoreUInt16(uint8_t *buf, const uint16_t value)
{
	*buf = value & 0xFF;
	*(buf + 1)  = (value >> 8 ) & 0xFF;
}

void StoreUInt32(uint8_t *buf, const uint32_t value)
{
	*buf = value & 0xFF;
	*(buf + 1)  = (value >> 8 ) & 0xFF;
	*(buf + 2)  = (value >> 16 ) & 0xFF;
	*(buf + 3)  = (value >> 24 ) & 0xFF;
}

