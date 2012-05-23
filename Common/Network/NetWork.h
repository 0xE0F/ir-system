/*
 * Компоненты для работы  с сетью
 */
#ifndef __NETWORK__H__
#define __NETWORK__H__

#define RX_TX_CONTROL_PORT GPIOA
#define RX_TX_CONTROL_PIN GPIO_Pin_3
#define RX_TX_RECEIVE_MODE RX_TX_CONTROL_PORT->BSRR |= RX_TX_CONTROL_PIN;
#define RX_TX_TRANSMIT_MODE RX_TX_CONTROL_PORT->BRR |= RX_TX_CONTROL_PIN;

// Инициализация сети
void InitNetWork(uint32_t baudrate, uint8_t address);

// Обработка сетевых данных
void NetWorkProcess(void);

// Передача данных в сеть.
// Если возвращаемое значение не ноль - произошла ошибка (интерфейс занят, недействиетльный буффер, слишком большой размер данных)
uint8_t Send(const uint8_t *pBuf, uint32_t count);

// Состояние в сети
typedef enum  { Receive, Transmit} NetworkState;


#endif //__NETWORK__H__
