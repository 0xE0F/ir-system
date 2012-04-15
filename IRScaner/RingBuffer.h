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

// ������� ������
void RB_Clear(RingBuffer *buffer);

// ������ � �����
uint32_t RB_Write(RingBuffer *buffer, const uint8_t value);

// ������ �� ������
uint32_t RB_Read(RingBuffer *buffer, uint8_t *value);

/* Circular buffer object */
typedef struct {
    int         size;   /* maximum number of elements           */
    int         start;  /* index of oldest element              */
    int         end;    /* index at which to write new element  */
    uint8_t   *elems;  /* vector of elements                   */
} CircularBuffer;

void cbInit(CircularBuffer *cb, int size);
void cbClear(CircularBuffer *cb);
int cbIsFull(CircularBuffer *cb);
int cbIsEmpty(CircularBuffer *cb);
void cbWrite(CircularBuffer *cb, uint8_t *elem);
void cbRead(CircularBuffer *cb, uint8_t *elem);


#endif // __RING_BUFFER__H__
