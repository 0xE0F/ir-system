#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>

#include <stdio.h>
#include <stdint.h>
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
static enum States State = Idle; /* Текущее состояние */

bool BlinkIrReceiveLed = false;
bool BlinkIrScanCompliteLed = false;

static void InitLeds(void);
static void InitTerminalUART(uint32_t baudrate);

#define IR_WAIT_1_LED_PORT GPIOC
#define IR_WAIT_2_LED_PORT GPIOD
#define POWER_LED_PORT GPIOB
#define NETWORK_LED_PORT GPIOB

#define IR_WAIT_1_LED_PIN GPIO_Pin_12
#define IR_WAIT_2_LED_PIN GPIO_Pin_2
#define POWER_LED_PIN GPIO_Pin_4
#define NETWORK_LED_PIN GPIO_Pin_3

void IrWaitFirstCodeLedOn(void) {  IR_WAIT_1_LED_PORT->BRR |= IR_WAIT_1_LED_PIN;}
void IrWaitFirstCodeLedOff(void) { IR_WAIT_1_LED_PORT->BSRR |= IR_WAIT_1_LED_PIN;}
void IrWaitFirstCodeLedInv(void) { IR_WAIT_1_LED_PORT->ODR ^= IR_WAIT_1_LED_PIN;}

void IrWaitSecondCodeLedOn(void) {  IR_WAIT_2_LED_PORT->BRR |= IR_WAIT_2_LED_PIN;}
void IrWaitSecondCodeLedOff(void) { IR_WAIT_2_LED_PORT->BSRR |= IR_WAIT_2_LED_PIN;}
void IrWaitSecondCodeLedInv(void) { IR_WAIT_2_LED_PORT->ODR ^= IR_WAIT_2_LED_PIN;}

void PowerLedOn(void) { POWER_LED_PORT->BRR |= POWER_LED_PIN;}
void PowerLedOff(void) { POWER_LED_PORT->BSRR |= POWER_LED_PIN;}
void PowerLedInv(void) { POWER_LED_PORT->ODR ^= POWER_LED_PIN;}

void NetworkLedOn(void) { NETWORK_LED_PORT->BRR |= NETWORK_LED_PIN;}
void NetworkLedOff(void) { NETWORK_LED_PORT->BSRR |= NETWORK_LED_PIN;}
void NetworkLedInv(void) { NETWORK_LED_PORT->ODR ^= NETWORK_LED_PIN;}

void ProcessScan(void);

/* Запуск проесса сканирования */
void RunScan(void)
{
	if (State != Idle)
		return;
	State = Start;
}

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

size_t GetJumpersValue(void)
{
	size_t value = 0, tmp = 0;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	tmp |= (~(GPIOB->IDR >> 12)) & 0xF;
	value = (tmp << 3) & 0x08;
	value |= (tmp << 1) & 0x04;
	value |= (tmp >> 1) & 0x02;
	value |= (tmp >> 3) & 0x01;

	return value;
}

int main(void)
{
	InitTerminal(115200);
	InitPlcTimers();
	InitLeds();

	IrWaitFirstCodeLedOff();
	IrWaitSecondCodeLedOff();
	PowerLedOff();
	NetworkLedOff();

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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = IR_WAIT_1_LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IR_WAIT_1_LED_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = IR_WAIT_2_LED_PIN;
	GPIO_Init(IR_WAIT_2_LED_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POWER_LED_PIN;
	GPIO_Init(POWER_LED_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = NETWORK_LED_PIN;
	GPIO_Init(NETWORK_LED_PORT, &GPIO_InitStructure);
}

void USART1_IRQHandler(void)
{
	uint8_t ch = USART_ReceiveData(USART1);
	cbWrite(&terminalBuffer, &ch);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

void ProcessScan(void)
{
	switch(State)
	{
		case Idle:
			BlinkIrReceiveLed = false;
			BlinkIrScanCompliteLed = false;
			IrWaitFirstCodeLedOff();
			IrWaitSecondCodeLedOff();
			break;

		case Start:
		{
			State = ScanFirst;
			printf("Start scan IR code:\n\r");
			break;
		}

		case ScanFirst:
		{
			BlinkIrReceiveLed = true;
			Scan(&(IrCodes[0]));
			State = WaitFirst;
			printf("\tWaiting first code...");
			break;
		}

		case WaitFirst:
			if (!IsScanning())
			{
				State = ScanSecond;
				BlinkIrReceiveLed = false;
				IrWaitFirstCodeLedOn();
				printf("done\n\r");
			}
			break;

		case ScanSecond:
		{
			BlinkIrScanCompliteLed = true;
			Scan(&(IrCodes[1]));
			State = WaitSecond;
			printf("\tWaiting second code...");
			break;
		}

		case WaitSecond:
			if (!IsScanning())
			{
				State = Check;
				BlinkIrScanCompliteLed = false;
				IrWaitSecondCodeLedOn();
				printf("done\n\r");
			}
			break;

		case Check:
		{
			bool cmp = false;
			printf("\tCheck IR codes: ");
			cmp = IsEqual(&(IrCodes[0]), &(IrCodes[1]));
			printf("%s\n\r", cmp ? "Equal" : "Error");
			State = SendCode;
			if (!cmp)
			{
				IrWaitFirstCodeLedOff();
				IrWaitSecondCodeLedOff();
			}
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

	if (IsTimeoutEx(BLINK_TIMER, BLINK_VALUE))
	{
		if (BlinkIrReceiveLed)
			IrWaitFirstCodeLedInv();
		if (BlinkIrScanCompliteLed)
			IrWaitSecondCodeLedInv();
	}

	if (IsTimeoutEx(SLOW_BLINK_TIMER, SLOW_BLINK_VALUE))
		PowerLedInv();

}
