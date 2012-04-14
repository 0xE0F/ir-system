/*
 * Служба программных таймеров
 */
#ifndef __PLC__TIMERS__H__
#define __PLC__TIMERS__H__

#define TIMERS_COUNT 4	/* Количество PLC таймеров */

// Инициализация тамеров
void InitPlcTimers(void);

// Проверка на таймаут тамера с timerId
uint8_t IsTimeout(const uint8_t timerId);

// Проверка на таймаут таймера с timerId и автоматической перезагрузкой значения
uint8_t IsTimeoutEx(const uint8_t timerId, const uint32_t reloadValue);

// Установка значения таймера
void SetTimerValue(const uint8_t timerId, const uint32_t value);

#endif //__PLC__TIMERS__H__
