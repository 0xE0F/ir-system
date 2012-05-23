#include <stm32f10x.h>
#include <stdlib.h>

#include "RingBuffer.h"

RingBuffer *MakeRingBuffer(const uint16_t size)
{
	if ((size&(size-1))!=0)
		return 0;

	RingBuffer *rb = (RingBuffer*)malloc(sizeof(RingBuffer));
	rb->_Mask = size-1;
	rb->_ReadIndex=0;
	rb->_WriteIndex=0;
	rb->_Buffer = (uint8_t *)malloc(size);
	return rb;
}

uint32_t RB_Write(RingBuffer *buffer, const uint8_t value)
{
	if(RB_IsFull(buffer))
		return 0;
	buffer->_Buffer[buffer->_WriteIndex++ & buffer->_Mask] = value;
	return 1;
}

uint32_t RB_Read(RingBuffer *buffer, uint8_t *value)
{
	if(RB_IsEmpty(buffer))
		return 0;
	*value = buffer->_Buffer[buffer->_ReadIndex++ & buffer->_Mask];
	return 1;
}

inline uint32_t RB_IsEmpty(RingBuffer *buffer)
{
	return buffer->_ReadIndex == buffer->_WriteIndex;
}

inline uint32_t RB_IsFull(RingBuffer *buffer)
{
	return ((uint16_t)(buffer->_WriteIndex - buffer->_ReadIndex) & (uint16_t)~(buffer->_Mask)) != 0;
}

// Очистка буфера
void RB_Clear(RingBuffer *buffer)
{
	buffer->_ReadIndex=0;
	buffer->_WriteIndex=0;
}


void cbInit(CircularBuffer *cb, int size) {
    cb->size  = size + 1; /* include empty elem */
    cb->start = 0;
    cb->end   = 0;
    cb->elems = (uint8_t *)calloc(cb->size, sizeof(uint8_t));
}

void cbClear(CircularBuffer *cb)
{
    cb->start = cb->end = 0;
}

int cbIsFull(CircularBuffer *cb)
{
    return (cb->end + 1) % cb->size == cb->start;
}

int cbIsEmpty(CircularBuffer *cb)
{
    return cb->end == cb->start;
}

/* Write an element, overwriting oldest element if buffer is full. App can
   choose to avoid the overwrite by checking cbIsFull(). */
void cbWrite(CircularBuffer *cb, uint8_t *elem)
{
    cb->elems[cb->end] = *elem;
    cb->end = (cb->end + 1) % cb->size;
    if (cb->end == cb->start)
        cb->start = (cb->start + 1) % cb->size; /* full, overwrite */
}

/* Read oldest element. App must ensure !cbIsEmpty() first. */
void cbRead(CircularBuffer *cb, uint8_t *elem)
{
    *elem = cb->elems[cb->start];
    cb->start = (cb->start + 1) % cb->size;
}
