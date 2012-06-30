/*
 * ����� ����������� ��� ������������ �����
 */
#ifndef __IR__H__
#define __IR__H__

#define STORAGE_PAGE_SIZE 2048
#define INTERVAL_SIZE (4)
#define IR_CODE_HEADER_SIZE (4 * 4)

#define INTERVALS_MAX ((STORAGE_PAGE_SIZE - IR_CODE_HEADER_SIZE) / INTERVAL_SIZE) // ������������ ���������� ����������

#pragma pack(1)

// �������� ����
typedef struct
{
	uint32_t ID;
	uint16_t Flags;
	uint16_t Crc;
	uint32_t Frequency;
	uint32_t IntervalsCount;
	uint32_t Intervals[INTERVALS_MAX];	// ������� - ������� ��� - �������� ������, ��������� - ����� � ���
} IRCode;

#pragma pack(0)

#if (INTERVALS_MAX*INTERVAL_SIZE + IR_CODE_HEADER_SIZE) > STORAGE_PAGE_SIZE
#error Size of IRCode NOT EQUAL STORAGE_PAGE_SIZE
#endif

enum { InvalidFrequencyValue = 0 };

extern const uint32_t DetectFreqMin; /* ���������� ������������� ������� */
extern const uint32_t DetectFreqMax; /* ������������ ������������� ������� */

extern uint32_t LengthDelataMax; 		/* ������������ ������� � ����� ����� */
extern uint32_t IntervalDelataMax;	/* ������������ ������� ����� ����������� */

extern bool DebugModeIr; /* ����� ������� �� ����� */

// ��������� ����������� ������� �� ��������� 30 ��� - 60 ���
uint32_t GetFrequencyInterval(uint32_t value);

//��������� �������� �������
void SetTime(uint32_t *interval, const uint32_t time);

// ��������� �������� ���������
void SetValue(uint32_t *interval, const uint8_t value);

// ��������� �������� ������� ���������
uint32_t GetTime(uint32_t interval);

// ��������� �������� ���������
uint8_t GetValue(uint32_t interval);

bool CheckIRCode(IRCode *code); /* �������� �� ������������ ���� */

bool IsEqual(IRCode *left, IRCode *rigth); /* ��������� ���� ����� */

void DebugPrint(IRCode *code); 	/* ��������� ����� ���� */

#endif // __IR__H__
