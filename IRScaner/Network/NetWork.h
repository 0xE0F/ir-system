/*
 * ���������� ��� ������  � �����
 */
#ifndef __NETWORK__H__
#define __NETWORK__H__

#define RX_TX_CONTROL_PORT GPIOA
#define RX_TX_CONTROL_PIN GPIO_Pin_3
#define RX_TX_RECEIVE_MODE RX_TX_CONTROL_PORT->BSRR |= RX_TX_CONTROL_PIN;
#define RX_TX_TRANSMIT_MODE RX_TX_CONTROL_PORT->BRR |= RX_TX_CONTROL_PIN;

// ������������� ����
void InitNetWork(uint32_t baudrate, uint8_t address);

// ��������� ������� ������
void NetWorkProcess(void);

// �������� ������ � ����.
// ���� ������������ �������� �� ���� - ��������� ������ (��������� �����, ���������������� ������, ������� ������� ������ ������)
uint8_t Send(const uint8_t *pBuf, uint32_t count);

// ��������� � ����
typedef enum  { Receive, Transmit} NetworkState;


#endif //__NETWORK__H__
