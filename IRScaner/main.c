#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>

#include <stdio.h>
#include <string.h>
#include "microrl/microrl.h"
#include "Terminal.h"
#include "IR/IR.h"
#include "IR/IRScanner.h"
#include "IR/IRTransmitter.h"

#include "RingBuffer.h"

// create microrl object and pointer on it
microrl_t rl;
microrl_t * prl = &rl;
RingBuffer *uartBuffer;

IRCode DebugCode;

static void InitLeds(void);
static void InitUART(uint32_t baudrate);

inline void ReceiveLedOn(void) { GPIOC->BSRR |= GPIO_Pin_8;}
inline void ReceiveLedOff(void) { GPIOC->BRR |= GPIO_Pin_8;}
inline void ReceiveLedInv(void) { GPIOC->ODR ^= GPIO_Pin_8;}

inline void TransmitLedOn(void) { GPIOC->BSRR |= GPIO_Pin_9;}
inline void TransmitLedOff(void) { GPIOC->BRR |= GPIO_Pin_9;}
inline void TransmitLedInv(void) { GPIOC->ODR ^= GPIO_Pin_9;}

enum CurrentState { IDLE, SCANNING, TRANSMITTING};

static enum CurrentState _CurrentState = IDLE;

int main(void)
{
	uartBuffer = MakeRingBuffer(16);

	InitUART(115200);
	InitLeds();

	// call init with ptr to microrl instance and print callback
	microrl_init (prl, print);
	// set callback for execute
	microrl_set_execute_callback (prl, execute);

#ifdef _USE_COMPLETE
	// set callback for completion
	microrl_set_complite_callback (prl, complet);
#endif
	// set callback for Ctrl+C
	microrl_set_sigint_callback (prl, sigint);
	while (1)
	{
		uint8_t c;
		// put received char from stdin to microrl lib
		if ((!RB_IsEmpty(uartBuffer)) && RB_Read(uartBuffer, &c))
			microrl_insert_char (prl, c);

		switch(_CurrentState)
		{
			case IDLE:
				if (IsScanning())
				{
					ReceiveLedOn();
					_CurrentState = SCANNING;
					print("Scanning ...\n\r");
				}
				if (IsSending())
				{
					TransmitLedOn();
					print("Transmitting ...");
					_CurrentState = TRANSMITTING;
				}
				break;

			case SCANNING:
				if (!IsScanning())
				{
					_CurrentState = IDLE;
					ReceiveLedOff();
					DebugPrint(&DebugCode);
					printf("Done \n");
				}
				break;

			case TRANSMITTING:
				if (!IsSending())
				{
					_CurrentState = IDLE;
					TransmitLedOff();
					printf("done.\n");
				}
				break;
		}
	}
}

static void InitUART(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = 8;
	USART_InitStructure.USART_Parity = USART_Parity_No;

	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 7);

	USART_Cmd(USART1, ENABLE);
}

static void InitLeds(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void USART1_IRQHandler(void)
{
	RB_Write(uartBuffer, USART_ReceiveData(USART1));
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

