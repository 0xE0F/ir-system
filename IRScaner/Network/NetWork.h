/*
 * Компоненты для работы  с сетью
 */
#ifndef __NETWORK__H__
#define __NETWORK__H__

// Инициализация сети
void InitNetWork(uint32_t baudrate, uint8_t address);

// Обработка сетевых данных
void NetWorkProcess(void);

#endif //__NETWORK__H__
