/*
 * ������� ��������� �����
 */

#ifndef __RING_BUFFER__H__
#define __RING_BUFFER__H__

typedef struct  {
	volatile uint16_t _ReadIndex;
	volatile uint16_t _WriteIndex;
	uint16_t _Mask;//= SIZE-1;
	uint8_t *_Buffer;
} RingBuffer;

RingBuffer *MakeRingBuffer(const uint16_t size);

// ���� �� ������
uint32_t RB_IsEmpty(RingBuffer *buffer);
// �������� ���������
uint32_t RB_IsFull(RingBuffer *buffer);
// ������ � �����
uint32_t RB_Write(RingBuffer *buffer, const uint8_t value);
// ������ �� ������
uint32_t RB_Read(RingBuffer *buffer, uint8_t *value);

#endif // __RING_BUFFER__H__
