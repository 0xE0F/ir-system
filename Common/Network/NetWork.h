/*
 * ���������� ��� ������  � �����
 */
#ifndef __NETWORK__H__
#define __NETWORK__H__

#define RX_TX_CONTROL_PORT GPIOA
#define RX_TX_CONTROL_PIN GPIO_Pin_3
#define RX_TX_RECEIVE_MODE RX_TX_CONTROL_PORT->BSRR |= RX_TX_CONTROL_PIN;
#define RX_TX_TRANSMIT_MODE RX_TX_CONTROL_PORT->BRR |= RX_TX_CONTROL_PIN;

void InitNetWork(uint32_t baudrate, uint8_t address, uint8_t deviceType); /* ������������� ���� */

void NetWorkProcess(void); /* ��������� ������� ������ */

// �������� ������ � ����.
// ���� ������������ �������� �� ���� - ��������� ������ (��������� �����, ���������������� ������, ������� ������� ������ ������)
bool Send(const uint8_t *pBuf, uint32_t count);

// ��������� � ����
typedef enum  { Receive, Transmit} NetworkState;

void RequestDeviceType(void); /* ������ ���� ���������� */
void RequestOnScan(uint16_t id, uint8_t mode); /* ������ �� ������������ ���� */
void RequestOffScan(void);		/* ������ �� ���������� ������������ */
void RequestSendCode(uint16_t id, uint8_t channel);		/* ������ �� �������� ���� � ����� */
void RequestSaveCode(void);			/* ������ �� ���������� ���� � ��������� */
void RequestReadCode(void);			/* ������ �� ������ ���� �� ��������� */
void RequestDeleteCode(void);		/* ������ �� �������� ���� �� ��������� */
void RequestDelateAllCodes(void);	/* ������ �� �������� ���� ����� � ��������� */

#endif //__NETWORK__H__
