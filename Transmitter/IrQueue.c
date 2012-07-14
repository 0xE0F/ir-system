/* Circular buffer example, keeps one slot open */
#include <stm32f10x.h>
#include <stdio.h>
#include <malloc.h>
#include <IrQueue.h>

void IrQueueInit(IrQueue *cb, int size) {
    cb->size  = size + 1; /* include empty elem */
    cb->start = 0;
    cb->end   = 0;
    cb->elems = (ElemType *)calloc(cb->size, sizeof(ElemType));
}

void IrQueueFree(IrQueue *queue) {
    free(queue->elems); /* OK if null */ }

int IrQueueIsFull(IrQueue *queue) {
    return (queue->end + 1) % queue->size == queue->start; }

int IrQueueIsEmpty(IrQueue *queue) {
    return queue->end == queue->start; }

/* Write an element, overwriting oldest element if buffer is full. App can
   choose to avoid the overwrite by checking cbIsFull(). */
void IrQueueWrite(IrQueue *queue, ElemType *elem) {
    queue->elems[queue->end] = *elem;
    queue->end = (queue->end + 1) % queue->size;
    if (queue->end == queue->start)
        queue->start = (queue->start + 1) % queue->size; /* full, overwrite */
}

/* Read oldest element. App must ensure !cbIsEmpty() first. */
void IrQueueRead(IrQueue *queue, ElemType *elem) {
    *elem = queue->elems[queue->start];
    queue->start = (queue->start + 1) % queue->size;
}
