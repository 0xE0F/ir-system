#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <../Microrl/microrl.h>
#include "Terminal.h"
#include <../IR/IR.h>
#include "IRTransmitter.h"

#include "RingBuffer.h"

#include <Storage/ffconf.h>
#include <Storage/ff.h>
#include <Storage/diskio.h>
#include <Storage/Storage.h>
#include <../NetWork/NetWork.h>
#include "PlcTimers.h"

microrl_t rl;						 /* Реализация терминала */
microrl_t * prl = &rl;				 /* */
static CircularBuffer terminalBuffer; /* Буфер терминала */
static const unsigned TerminalBufferSize = 16; /* Размер буфера терминала */

static void InitLeds(void);
static void InitTerminalUART(uint32_t baudrate);

#define ERROR_LED_PORT GPIOC
#define TRANSMITT_LED_PORT GPIOD
#define POWER_LED_PORT GPIOB
#define NETWORK_LED_PORT GPIOB

#define ERROR_LED_PIN GPIO_Pin_12
#define TRANSMITT_LED_PIN GPIO_Pin_2
#define POWER_LED_PIN GPIO_Pin_4
#define NETWORK_LED_PIN GPIO_Pin_3

void ErrorLedOn(void) {  ERROR_LED_PORT->BRR |= ERROR_LED_PIN;}
void ErrorLedOff(void) { ERROR_LED_PORT->BSRR |= ERROR_LED_PIN;}
void ErrorLedInv(void) { ERROR_LED_PORT->ODR ^= ERROR_LED_PIN;}

void TransmittLedOn(void) {  TRANSMITT_LED_PORT->BRR |= TRANSMITT_LED_PIN;}
void TransmittLedOff(void) { TRANSMITT_LED_PORT->BSRR |= TRANSMITT_LED_PIN;}
void TransmittLedInv(void) { TRANSMITT_LED_PORT->ODR ^= TRANSMITT_LED_PIN;}

void PowerLedOn(void) { POWER_LED_PORT->BRR |= POWER_LED_PIN;}
void PowerLedOff(void) { POWER_LED_PORT->BSRR |= POWER_LED_PIN;}
void PowerLedInv(void) { POWER_LED_PORT->ODR ^= POWER_LED_PIN;}

void NetworkLedOn(void) { NETWORK_LED_PORT->BRR |= NETWORK_LED_PIN;}
void NetworkLedOff(void) { NETWORK_LED_PORT->BSRR |= NETWORK_LED_PIN;}
void NetworkLedInv(void) { NETWORK_LED_PORT->ODR ^= NETWORK_LED_PIN;}


static void InitTerminal(uint32_t baudrate) {
	cbInit(&terminalBuffer, TerminalBufferSize);

	InitTerminalUART(baudrate);

	microrl_init (prl, print);
	microrl_set_execute_callback (prl, execute);
	microrl_set_complite_callback (prl, complet);
	microrl_set_sigint_callback (prl, sigint);
}

size_t GetJumpersValue(void) {
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

int main(void) {
 	for (size_t i = 0; i < SystemCoreClock/100; i++);

 	InitLeds();
	PowerLedOn();
	InitTerminal(115200);

	/** Инициализация системного таймера с периодом в 1 мс */
	if (SysTick_Config(SystemCoreClock / 1000)) {
		ErrorLedOn();
		printf("Error init system timer\n\r");
		while (true);
	}

	InitPlcTimers();
	InitNetWork(115200, GetJumpersValue(), dtTransmitter);

 	SetStorageDebugMode(true);
	if ( !InitStorage() ) {
		while(true) {
			if (IsTimeoutEx(BLINK_TIMER, BLINK_VALUE)) {
				ErrorLedInv();
			}
		}
	}
 	SetStorageDebugMode(false);

	while (true) {
		if ( !cbIsEmpty(&terminalBuffer) ) {
			uint8_t c;
			cbRead(&terminalBuffer, &c);
			microrl_insert_char (prl, c);
		}

		ProcessNetwork();
		if (IsTimeoutEx(BLINK_TIMER, BLINK_VALUE)) {
			NetworkLedInv();
		}
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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ERROR_LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ERROR_LED_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = TRANSMITT_LED_PIN;
	GPIO_Init(TRANSMITT_LED_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = POWER_LED_PIN;
	GPIO_Init(POWER_LED_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = NETWORK_LED_PIN;
	GPIO_Init(NETWORK_LED_PORT, &GPIO_InitStructure);

	ErrorLedOff();
	TransmittLedOff();
	NetworkLedOff();
	PowerLedOff();
}

/** Прием данных по первому последовательному порту */
void USART1_IRQHandler(void)
{
	uint8_t ch = USART_ReceiveData(USART1);
	cbWrite(&terminalBuffer, &ch);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

/** Тик системного таймера дла отрабоки таймаутов дискового ввода вывода*/
RAMFUNC void SysTick_Handler(void)
{
	static uint8_t cntdiskio=0;

	cntdiskio++;
	if ( cntdiskio >= 10 ) {
		cntdiskio = 0;
		disk_timerproc(); /* to be called every 10ms */
	}
}

/** Запрос на сканирование кода */
void RequestOnScan(uint16_t id, ScanMode mode) { }

/** Запрос на выключение сканирвоания */
void RequestOffScan(void) { }

