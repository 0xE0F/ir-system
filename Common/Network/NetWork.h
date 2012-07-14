/*
 * Компоненты для работы  с сетью
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



enum { dtScaner = 0x02, dtTransmitter = 0x01 }; /* Типы устройств */
typedef enum { errNotEqual = 1, errBusy = 2, errFlash = 5 } Errors;
enum { cmdDeviceType = 0x0F, cmdOnScan = 0x01, cmdOffScan = 0x02,  cmdSendCode = 0x03, cmdDeleteCode = 0x04, cmdDeleteAll = 0x05, cmdReadCodes = 0x07, cmdSaveCode = 0x08, cmdError = 0x0A};

/* Параметры хранилища - только передачи или передача и хранение */
enum {scOnlyTransmit = 1, scSaveAndTransmit = 2};
typedef enum { pmNone, pmEven, pmOdd } ParityMode;
void InitNetWork(uint32_t baudrate, ParityMode parity, uint8_t address, uint8_t deviceType); /* Инициализация сети */

void ProcessNetwork(void); /* Обработка сетевых данных */

/* Передается ли что нибудь ?*/
bool IsNetworkBusy(void);

// Состояние в сети
typedef enum  { Receive, Transmit } NetworkState;

/* Состоянее драйвер отправки */
NetworkState GetNetworkState(void);

/* Получение адреса устройства */
uint8_t GetDeviceAddress(void);

/* Получение типа устройства */
uint8_t GetDeviceType(void);

// Режимы сканирования
typedef enum  { smInternal, smNetwork, smInternalAndNetwork} ScanMode;

/* Ответ на запрос */
/* Формируется путем сложения заголовка и тела сообщения
 * calcCrc определяет нужли ли добавлять в конце CRC
 * */
bool Answer(uint8_t *header, const size_t headerSize, uint8_t *msg, const size_t msgSize, const bool calcCrc);

/* Ответ кодом ошибки */
bool AnswerError(Errors error);

void RequestDeviceType(void); /* Запрос типа устройства */
void RequestOnScan(uint16_t id, ScanMode mode); /* Запрос на сканирование кода */
void RequestOffScan(void);		/* Запрос на выключение сканирвоания */
void RequestSendCode(uint8_t *buffer, size_t count);		/* Запрос на отправку кода в канал */


void RequestSaveCode(uint8_t *buffer, size_t count);
void RequestReadCodes(uint8_t *buffer, size_t count);			/* Запрос на чтение кодов из хранилища */
void RequestDelateAllCodes(uint8_t *buffer, size_t count);	/* Запрос на удаление всех кодов в хранилище */

/** Чтение и запись типов из буфера */
uint16_t GetUInt16(uint8_t *buf);
uint16_t GetUInt32(uint8_t *buf);
void StoreUInt16(uint8_t *buf, const uint16_t value);
void StoreUInt32(uint8_t *buf, const uint32_t value);

extern void NetworkLedOn(void);
extern void NetworkLedOff(void);
extern void NetworkLedInv(void);

#endif //__NETWORK__H__
