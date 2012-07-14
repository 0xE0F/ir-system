/*
 * ������ ����������� ��������
 */
#ifndef __PLC__TIMERS__H__
#define __PLC__TIMERS__H__


#define PLC_TIMER_PERIOD	0.0001	/* ���������� ������������ ������� */

#define RX_TIMEOUT_TIMER 0			/* ������ ������� ������ */
#define RX_TIMEOUT_VALUE (0.010 / PLC_TIMER_PERIOD) /* 5 �� - ������� ������ ������ */

#define BLINK_TIMER 1			/* ������ ������� ������� */
#define BLINK_VALUE (0.25 / PLC_TIMER_PERIOD) /* 250 �� - �������� �������  */

#define SLOW_BLINK_TIMER 2			/* ������ ������� ���������� ������� */
#define SLOW_BLINK_VALUE (0.5 / PLC_TIMER_PERIOD) /* 250 �� - �������� �������  */

#define TRANSMIT_TIMER 3			/* ������ ������� ���������� ������� */
#define TRANSMIT_VALUE (0.0002 / PLC_TIMER_PERIOD) /* 250 �� - �������� �������  */

#define SND_TIMER 4			/* ������ ������� ���������� ������� */
#define SND_VALUE (1 / PLC_TIMER_PERIOD) /* 250 �� - �������� �������  */

#define TR_TIMER  5			/** ������ ����� ��������� ������� � ������ */

#define IR_SND_TIMER 6 		/** ������ ���������� ����� ��������� IR ����� */

/* ������������� ������� */
void InitPlcTimers(void);

/* �������� �� ������� ������ � timerId */
uint8_t IsTimeout(const uint8_t timerId);

/* �������� �� ������� ������� � timerId � �������������� ������������� �������� */
uint8_t IsTimeoutEx(const uint8_t timerId, const uint32_t reloadValue);

/* ��������� �������� ������� */
void SetTimerValue(const uint8_t timerId, const uint32_t value);

#endif //__PLC__TIMERS__H__
