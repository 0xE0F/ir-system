/*
 * Служба программных таймеров
 */
#ifndef __PLC__TIMERS__H__
#define __PLC__TIMERS__H__


#define PLC_TIMER_PERIOD	0.0001	/* Разрешение программного таймера */

#define RX_TIMEOUT_TIMER 0			/* Индекс таймера приема */
#define RX_TIMEOUT_VALUE (0.010 / PLC_TIMER_PERIOD) /* 5 мс - таймаут приема данных */

#define BLINK_TIMER 1			/* Индекс таймера мигания */
#define BLINK_VALUE (0.25 / PLC_TIMER_PERIOD) /* 250 мс - интервал мигания  */

#define SLOW_BLINK_TIMER 2			/* Индекс таймера медленного мигания */
#define SLOW_BLINK_VALUE (0.5 / PLC_TIMER_PERIOD) /* 250 мс - интервал мигания  */

#define TRANSMIT_TIMER 3			/* Индекс таймера медленного мигания */
#define TRANSMIT_VALUE (0.0002 / PLC_TIMER_PERIOD) /* 250 мс - интервал мигания  */

#define SND_TIMER 4			/* Индекс таймера медленного мигания */
#define SND_VALUE (1 / PLC_TIMER_PERIOD) /* 250 мс - интервал мигания  */

#define TR_TIMER  5			/** Таймер между отправкой пакетов с кодами */

#define IR_SND_TIMER 6 		/** Таймер интервалов между отправкой IR кодов */

/* Инициализация тамеров */
void InitPlcTimers(void);

/* Проверка на таймаут тамера с timerId */
uint8_t IsTimeout(const uint8_t timerId);

/* Проверка на таймаут таймера с timerId и автоматической перезагрузкой значения */
uint8_t IsTimeoutEx(const uint8_t timerId, const uint32_t reloadValue);

/* Установка значения таймера */
void SetTimerValue(const uint8_t timerId, const uint32_t value);

#endif //__PLC__TIMERS__H__
