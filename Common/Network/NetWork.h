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
enum { cmdDeviceType = 0x0F, cmdOnScan = 0x01, cmdOffScan = 0x02,  cmdSendCode = 0x03, cmdDeleteCode = 0x04, cmdDeleteAll = 0x05, cmdReadCode = 0x07, cmdSaveCode = 0x08, cmdError = 0x0A};

/* Параметры хранилища - только передачи или передача и хранение */
enum {scOnlyTransmit = 1, scSaveAndTransmit = 2};
typedef enum { pmNone, pmEven, pmOdd } ParityMode;
void InitNetWork(uint32_t baudrate, ParityMode parity, uint8_t address, uint8_t deviceType); /* Инициализация сети */

void ProcessNetwork(void); /* Обработка сетевых данных */

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

/** Записать (добавить) ИК-код в хранилище
a   08   k1  k2  n1   n2   n3   n4   l1   l2   [data]   c1   c2
	a  -  адрес устройства;
	08  - команда передачи ИК-кода в хранилище;
	k1   k2  -  порядковый номер ИК-кода, начинается с 0;
	n1   n2   n3   n4  -  идентификатор команды для мастер-хоста;
	l1   l2  - длина ИК-кода;
	[data]  - массив данных размером указанным выше размером;
	c1   c2  -  контрольная сумма, CRC16. Вычисляется по всей длине команды.
	В случае если команда уже присутствует в хранилище, в ответ отправляется сообщение об ошибке, см. п.5, код 6.
*/
void RequestSaveCode(uint8_t *buffer, size_t count);
void RequestReadCode(void);			/* Запрос на чтение коад из хранилища */
void RequestDeleteCode(void);		/* Запрос на удаление кода из хранилища */
void RequestDelateAllCodes(uint8_t *buffer, size_t count);	/* Запрос на удаление всех кодов в хранилище */

/** Чтение типов из буфера */
uint16_t GetUInt16(uint8_t *buf);
uint16_t GetUInt32(uint8_t *buf);

extern void NetworkLedOn(void);
extern void NetworkLedOff(void);
extern void NetworkLedInv(void);

#endif //__NETWORK__H__
