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
#include <../Tools/Crc.h>
#include <PlcTimers.h>

static microrl_t rl;
static microrl_t * prl = &rl;
static CircularBuffer terminalBuffer; /* Буфер терминала */
static const unsigned TerminalBufferSize = 16;
IRCode IrCodes[2];

/* Рабочие состояния */
enum States { Idle, Start, ScanFirst, WaitFirst, ScanSecond, WaitSecond, Check, SendCode, SendError, WaitSend};
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

static uint16_t Id = 0;
static size_t CodeToSendIndex = 0;

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
	InitNetWork(9600, GetJumpersValue(), dtScaner);

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
		ProcessNetwork();
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
			bool res = false;
			printf("\tCheck IR codes: ");
			res = IsEqual(&(IrCodes[0]), &(IrCodes[1]));
			printf("%s\n\r", res ? "Equal" : "Error");

			if (res)
			{
				CodeToSendIndex =  (IrCodes[0].Intervals < IrCodes[1].Intervals) ? 0 : 1;
				State = SendCode;
			}
			else
			{
				IrWaitFirstCodeLedOff();
				IrWaitSecondCodeLedOff();
				State = SendError;
			}
			break;
		}

		case SendCode:
		{
			uint8_t answer[] = {GetDeviceAddress(), cmdOnScan, (Id >> 8) & 0xFF, Id & 0x00FF, scOnlyTransmit, 0, 0};
			uint8_t *msg = (uint8_t *)&(IrCodes[CodeToSendIndex]);
			if (GetNetworkState() == Transmit)
				break;

			answer[6] = sizeof(IRCode) & 0xFF;
			answer[5] = (sizeof(IRCode) >> 8) & 0xFF;

			printf("\tSending IR code...");

			if (Answer(answer, sizeof(answer)/sizeof(answer[0]), msg, sizeof(IRCode), true));
				State = WaitSend;

			break;
		}

		case SendError:
		{
			printf("\tSending error code...");
			if (AnswerError(errNotEqual))
				State = WaitSend;
			break;
		}

		case WaitSend:
			if (GetNetworkState() == Receive)
			{
				State = Idle;
				printf("OK\n\r");
			}
			break;


		default:
			break;
	}

//	if (IsTimeoutEx(BLINK_TIMER, BLINK_VALUE))
//	{
//		if (BlinkIrReceiveLed)
//			IrWaitFirstCodeLedInv();
//		if (BlinkIrScanCompliteLed)
//			IrWaitSecondCodeLedInv();
//	}
//
//	if (IsTimeoutEx(SLOW_BLINK_TIMER, SLOW_BLINK_VALUE))
//		PowerLedInv();

//	if (IsTimeoutEx(SND_TIMER, SND_VALUE))
//	{
//		const uint8_t buf[] = {0x01, 0x10, 0x02, 0x20};
//		Send(buf, sizeof(buf) / sizeof(buf[0]));
//	}

}

/* Запрос на сканирование кода */
void RequestOnScan(uint16_t id, ScanMode mode)
{
	Id = id;

	switch(mode)
	{
		case smInternal:
			print("Scan code to internal memory not implementation\n\r");
			break;

		case smNetwork:
			printf("Start scan code ID:%u\n\r", Id);
			RunScan();
			break;

		case smInternalAndNetwork:
			print("Scan code to internal memory and network not implementation\n\r");
			break;

		default:
			print("Unknow save mode\n\r");
			break;
	}
}

