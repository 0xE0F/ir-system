#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>

#include <stdio.h>
#include <stdbool.h>

#include <string.h>
#include <../Microrl/microrl.h>
#include <Terminal.h>
#include <../IR/IR.h>
#include <IRScanner.h>
#include <RingBuffer.h>

#include <../NetWork/NetWork.h>
#include <PlcTimers.h>

static microrl_t rl;
static microrl_t * prl = &rl;
static CircularBuffer terminalBuffer; /* Буфер терминала */
static const unsigned TerminalBufferSize = 16;
IRCode IrCodes[2];

/* Рабочие состояния */
enum States { Idle, Start, ScanFirst, WaitFirst, ScanSecond, WaitSecond, Check, SendCode, WaitSend};

static enum States State = Idle;

void RunScan(void)
{
	if (State != Idle)
		return;
	State = Start;
}

static void InitLeds(void);
static void InitTerminalUART(uint32_t baudrate);

inline void ReceiveLedOn(void) { GPIOC->BSRR |= GPIO_Pin_8;}
inline void ReceiveLedOff(void) { GPIOC->BRR |= GPIO_Pin_8;}
inline void ReceiveLedInv(void) { GPIOC->ODR ^= GPIO_Pin_8;}

inline void TransmitLedOn(void) { GPIOC->BSRR |= GPIO_Pin_9;}
inline void TransmitLedOff(void) { GPIOC->BRR |= GPIO_Pin_9;}
inline void TransmitLedInv(void) { GPIOC->ODR ^= GPIO_Pin_9;}

void ProcessScan(void);

/* Инициализация терминала */
void InitTerminal(uint32_t baudrate)
{
	cbInit(&terminalBuffer, TerminalBufferSize);

	InitTerminalUART(baudrate);

	microrl_init (prl, print);
	microrl_set_execute_callback (prl, execute);
	microrl_set_complite_callback (prl, complet);
	microrl_set_sigint_callback (prl, sigint);
}

int main(void)
{
	InitTerminal(115200);
	InitLeds();

	while (1)
	{
		if (!cbIsEmpty(&terminalBuffer))
		{
			uint8_t c;
			cbRead(&terminalBuffer, &c);
			microrl_insert_char (prl, c);
		}

		ProcessScan();
	}
}

static void InitTerminalUART(uint32_t baudrate)
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
	uint8_t ch = USART_ReceiveData(USART1);
	cbWrite(&terminalBuffer, &ch);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

//RAMFUNC void SysTick_Handler(void)
//{
//	static uint16_t cnt=0;
//	static uint8_t flip=0, cntdiskio=0;
//
//	cnt++;
//	if( cnt >= 500 ) {
//		cnt = 0;
//		/* alive sign */
//		if ( flip ) {
//			GPIOC->BSRR |= GPIO_Pin_9;
//		} else {
//			GPIOC->BRR |= GPIO_Pin_9;		}
//		flip = !flip;
//	}
//
//	cntdiskio++;
//	if ( cntdiskio >= 10 ) {
//		cntdiskio = 0;
//		disk_timerproc(); /* to be called every 10ms */
//	}
//}

void ProcessScan(void)
{
	switch(State)
	{
		case Idle:
			break;

		case Start:
		{
			State = ScanFirst;
			printf("Start scan IR code:\n\r");
			break;
		}

		case ScanFirst:
		{
			Scan(&(IrCodes[0]));
			State = WaitFirst;
			printf("\tWaiting first code...");
			break;
		}

		case WaitFirst:
			if (!IsScanning())
			{
				State = ScanSecond;
				printf("done\n\r");
			}
			break;

		case ScanSecond:
		{
			Scan(&(IrCodes[1]));
			State = WaitSecond;
			printf("\tWaiting second code...");
			break;
		}

		case WaitSecond:
			if (!IsScanning())
			{
				State = Check;
				printf("done\n\r");
			}
			break;

		case Check:
		{
			uint8_t cmp;
			printf("\tCheck IR codes...");
			cmp = CompareIrCode(&(IrCodes[0]), &(IrCodes[1]));
			printf("%u\n\r", cmp);
			State = SendCode;
			break;
		}

		case SendCode:
			State = WaitSend;
			printf("\tSending IR code...");
			break;

		case WaitSend:
			State = Idle;
			printf("OK\n\r");
			break;

		default:
			break;

	}
}
