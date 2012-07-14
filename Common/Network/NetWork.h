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
typedef enum { errNotEqual = 1, errBusy = 2, errFlash = 5 } Errors;
enum { cmdDeviceType = 0x0F, cmdOnScan = 0x01, cmdOffScan = 0x02,  cmdSendCode = 0x03, cmdDeleteCode = 0x04, cmdDeleteAll = 0x05, cmdReadCodes = 0x07, cmdSaveCode = 0x08, cmdError = 0x0A};

/* ��������� ��������� - ������ �������� ��� �������� � �������� */
enum {scOnlyTransmit = 1, scSaveAndTransmit = 2};
typedef enum { pmNone, pmEven, pmOdd } ParityMode;
void InitNetWork(uint32_t baudrate, ParityMode parity, uint8_t address, uint8_t deviceType); /* ������������� ���� */

void ProcessNetwork(void); /* ��������� ������� ������ */

/* ���������� �� ��� ������ ?*/
bool IsNetworkBusy(void);

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

/* ����� �� ������ */
/* ����������� ����� �������� ��������� � ���� ���������
 * calcCrc ���������� ����� �� ��������� � ����� CRC
 * */
bool Answer(uint8_t *header, const size_t headerSize, uint8_t *msg, const size_t msgSize, const bool calcCrc);

/* ����� ����� ������ */
bool AnswerError(Errors error);

void RequestDeviceType(void); /* ������ ���� ���������� */
void RequestOnScan(uint16_t id, ScanMode mode); /* ������ �� ������������ ���� */
void RequestOffScan(void);		/* ������ �� ���������� ������������ */
void RequestSendCode(uint8_t *buffer, size_t count);		/* ������ �� �������� ���� � ����� */


void RequestSaveCode(uint8_t *buffer, size_t count);
void RequestReadCodes(uint8_t *buffer, size_t count);			/* ������ �� ������ ����� �� ��������� */
void RequestDelateAllCodes(uint8_t *buffer, size_t count);	/* ������ �� �������� ���� ����� � ��������� */

/** ������ � ������ ����� �� ������ */
uint16_t GetUInt16(uint8_t *buf);
uint16_t GetUInt32(uint8_t *buf);
void StoreUInt16(uint8_t *buf, const uint16_t value);
void StoreUInt32(uint8_t *buf, const uint32_t value);

extern void NetworkLedOn(void);
extern void NetworkLedOff(void);
extern void NetworkLedInv(void);

#endif //__NETWORK__H__
