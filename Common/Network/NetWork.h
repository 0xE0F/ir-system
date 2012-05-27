/*
 * Компоненты для работы  с сетью
 */
#ifndef __NETWORK__H__
#define __NETWORK__H__

#define RX_TX_CONTROL_PORT GPIOA
#define RX_TX_CONTROL_PIN GPIO_Pin_3
#define RX_TX_RECEIVE_MODE RX_TX_CONTROL_PORT->BSRR |= RX_TX_CONTROL_PIN;
#define RX_TX_TRANSMIT_MODE RX_TX_CONTROL_PORT->BRR |= RX_TX_CONTROL_PIN;

void InitNetWork(uint32_t baudrate, uint8_t address, uint8_t deviceType); /* Инициализация сети */

void NetWorkProcess(void); /* Обработка сетевых данных */

// Передача данных в сеть.
// Если возвращаемое значение не ноль - произошла ошибка (интерфейс занят, недействиетльный буффер, слишком большой размер данных)
bool Send(const uint8_t *pBuf, uint32_t count);

// Состояние в сети
typedef enum  { Receive, Transmit} NetworkState;

void RequestDeviceType(void); /* Запрос типа устройства */
void RequestOnScan(uint16_t id, uint8_t mode); /* Запрос на сканирование кода */
void RequestOffScan(void);		/* Запрос на выключение сканирвоания */
void RequestSendCode(uint16_t id, uint8_t channel);		/* Запрос на отправку кода в канал */
void RequestSaveCode(void);			/* Запрос на сохранение кода в хранилище */
void RequestReadCode(void);			/* Запрос на чтение коад из хранилища */
void RequestDeleteCode(void);		/* Запрос на удаление кода из хранилища */
void RequestDelateAllCodes(void);	/* Запрос на удаление всех кодов в хранилище */

#endif //__NETWORK__H__
