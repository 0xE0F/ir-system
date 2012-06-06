/*
 * Компоненты для работы  с сетью
 */
#ifndef __NETWORK__H__
#define __NETWORK__H__

#define RX_CONTROL_PORT GPIOC
#define TX_CONTROL_PORT GPIOC
#define TX_CONTROL_PIN GPIO_Pin_10
#define RX_CONTROL_PIN GPIO_Pin_11


#define RX_TX_TRANSMIT_MODE	RX_CONTROL_PORT->BSRR |= RX_CONTROL_PIN; \
							TX_CONTROL_PORT->BSRR |= TX_CONTROL_PIN;

#define RX_TX_RECEIVE_MODE	TX_CONTROL_PORT->BRR |= TX_CONTROL_PIN; \
							RX_CONTROL_PORT->BRR |= RX_CONTROL_PIN;



enum { dtScaner = 0x02, dtTransmitter = 0x01 }; /* Типы устройств */
enum { errNotEqual = 1 };
enum { cmdDeviceType = 0x0F, cmdOnScan = 0x01, cmdOffScan = 0x02,  cmdSendCode = 0x03, cmdDeleteCode = 0x04, cmdDeleteAll = 0x05, cmdReadCode = 0x07, cmdSaveCode = 0x08, cmdError = 0x0A};

void InitNetWork(uint32_t baudrate, uint8_t address, uint8_t deviceType); /* Инициализация сети */

void ProcessNetwork(void); /* Обработка сетевых данных */

// Передача данных в сеть.
// Если возвращаемое значение не ноль - произошла ошибка (интерфейс занят, недействиетльный буффер, слишком большой размер данных)
bool Send(const uint8_t *pBuf, uint32_t count);

// Состояние в сети
typedef enum  { Receive, Transmit } NetworkState;

/* Состоянее драйвер отправки */
NetworkState GetNetworkState(void);

/* Получение адреса устройства */
uint8_t GetDeviceAddress(void);

/* Получение типа устройства */
uint8_t GetDeviceType(void);

// Режимы сканирования
typedef enum  { smInternal, smNetwork, smInternalAndNetwork} ScanMode;

void RequestDeviceType(void); /* Запрос типа устройства */
void RequestOnScan(uint16_t id, ScanMode mode); /* Запрос на сканирование кода */
void RequestOffScan(void);		/* Запрос на выключение сканирвоания */
void RequestSendCode(uint16_t id, uint8_t channel);		/* Запрос на отправку кода в канал */
void RequestSaveCode(void);			/* Запрос на сохранение кода в хранилище */
void RequestReadCode(void);			/* Запрос на чтение коад из хранилища */
void RequestDeleteCode(void);		/* Запрос на удаление кода из хранилища */
void RequestDelateAllCodes(void);	/* Запрос на удаление всех кодов в хранилище */

extern void NetworkLedOn(void);
extern void NetworkLedOff(void);
extern void NetworkLedInv(void);

#endif //__NETWORK__H__
