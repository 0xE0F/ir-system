#include <stm32f10x.h>
#include <stdlib.h>

#include "RingBuffer.h"

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
