/*
 * ������ ����������� ��������
 */
#ifndef __PLC__TIMERS__H__
#define __PLC__TIMERS__H__

#define TIMERS_COUNT 4	/* ���������� PLC �������� */

#define PLC_TIMER_PERIOD	0.0001	/* ���������� ������������ ������� */

#define RX_TIMEOUT_TIMER 0			/* ������ ������� ������ */
#define RX_TIMEOUT_VALUE (0.005 / PLC_TIMER_PERIOD) /* 5 �� - ������� ������ ������ */

/* ������������� ������� */
void InitPlcTimers(void);

/* �������� �� ������� ������ � timerId */
uint8_t IsTimeout(const uint8_t timerId);

/* �������� �� ������� ������� � timerId � �������������� ������������� �������� */
uint8_t IsTimeoutEx(const uint8_t timerId, const uint32_t reloadValue);

/* ��������� �������� ������� */
void SetTimerValue(const uint8_t timerId, const uint32_t value);

#endif //__PLC__TIMERS__H__
