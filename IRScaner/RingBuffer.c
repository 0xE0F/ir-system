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
