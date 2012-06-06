/*
 * ���������� ��� ������  � �����
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



enum { dtScaner = 0x02, dtTransmitter = 0x01 }; /* ���� ��������� */
enum { errNotEqual = 1 };
enum { cmdDeviceType = 0x0F, cmdOnScan = 0x01, cmdOffScan = 0x02,  cmdSendCode = 0x03, cmdDeleteCode = 0x04, cmdDeleteAll = 0x05, cmdReadCode = 0x07, cmdSaveCode = 0x08, cmdError = 0x0A};

void InitNetWork(uint32_t baudrate, uint8_t address, uint8_t deviceType); /* ������������� ���� */

void ProcessNetwork(void); /* ��������� ������� ������ */

// �������� ������ � ����.
// ���� ������������ �������� �� ���� - ��������� ������ (��������� �����, ���������������� ������, ������� ������� ������ ������)
bool Send(const uint8_t *pBuf, uint32_t count);

// ��������� � ����
typedef enum  { Receive, Transmit } NetworkState;

/* ��������� ������� �������� */
NetworkState GetNetworkState(void);

/* ��������� ������ ���������� */
uint8_t GetDeviceAddress(void);

/* ��������� ���� ���������� */
uint8_t GetDeviceType(void);

// ������ ������������
typedef enum  { smInternal, smNetwork, smInternalAndNetwork} ScanMode;

void RequestDeviceType(void); /* ������ ���� ���������� */
void RequestOnScan(uint16_t id, ScanMode mode); /* ������ �� ������������ ���� */
void RequestOffScan(void);		/* ������ �� ���������� ������������ */
void RequestSendCode(uint16_t id, uint8_t channel);		/* ������ �� �������� ���� � ����� */
void RequestSaveCode(void);			/* ������ �� ���������� ���� � ��������� */
void RequestReadCode(void);			/* ������ �� ������ ���� �� ��������� */
void RequestDeleteCode(void);		/* ������ �� �������� ���� �� ��������� */
void RequestDelateAllCodes(void);	/* ������ �� �������� ���� ����� � ��������� */

extern void NetworkLedOn(void);
extern void NetworkLedOff(void);
extern void NetworkLedInv(void);

#endif //__NETWORK__H__
