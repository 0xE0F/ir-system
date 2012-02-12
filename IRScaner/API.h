/*
 *	������������ ���� � �������� ����������������.
 */
#ifndef _IRDA_API_H_
#define _IRDA_API_H_

#define STORAGE_PAGE_SIZE 2048
#define INTERVAL_SIZE (4)
#define IR_CODE_HEADER_SIZE (4 * 4)
// ���� ��������
#pragma pack(1)
typedef struct
{
	uint32_t Time;
	uint8_t  Value;
} Interval;

#define INTERVALS_MAX ((STORAGE_PAGE_SIZE - IR_CODE_HEADER_SIZE) / INTERVAL_SIZE) //

// �������� ����
typedef struct
{
	uint32_t ID;
	uint32_t Flags;
	uint32_t Frequency;
	uint32_t IntervalsCount;
	uint32_t Intervals[INTERVALS_MAX];
	//Interval Intervals[INTERVALS_MAX];
} IRCode;
#pragma pack(0)

#if (INTERVALS_MAX*INTERVAL_SIZE + IR_CODE_HEADER_SIZE) > STORAGE_PAGE_SIZE
#error Size of IRCode NOT EQUAL STORAGE_PAGE_SIZE
#endif

extern const uint32_t MaxChannelNumber;

// ������������ ����
void Scan(IRCode *irCode);
// ��������� ������������
void StopScan();
// ������������ ������������
volatile uint32_t IsScanning();

// ������ ���� �� ��������� ���������
uint32_t ReadCodeFromStorage(IRCode *code, uint32_t id);
// ������ ���� � ���������� ��������
uint32_t WriteCodeToStorage(IRCode *code, uint32_t id);

// ������ ���� � ��������������� �����
// 0 - ������� � �������
// 1 - ������ (��� �� ������ ��� ���������� ����� ������)
uint32_t WriteToChannel(IRCode *code, uint32_t channelID);

// ��������� ����������� ������� �� ��������� 30 ��� - 60 ���
// 0 - ���� ������� ��� ���������
uint32_t GetFrequencyInterval(uint32_t value);

// ��������� ������� �������
void SetCarrierFrequency(const uint32_t value);

// ��������� ����� ����
void DebugPrint(IRCode *code);

#endif // _IRDA_API_H_
