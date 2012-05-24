/*
 * Простой кольцевой буфер
 */
#ifndef __RING_BUFFER__H__
#define __RING_BUFFER__H__

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
