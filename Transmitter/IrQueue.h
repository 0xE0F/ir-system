/*
 * Компоненты для работы  с сетью
 */
#ifndef __IR_QUEUE__H__
#define __IR_QUEUE__H__

/* Opaque buffer element type.  This would be defined by the application. */
typedef struct {
	uint16_t Number;
	uint8_t Channel;
	uint8_t _alling;
} ElemType;

/* Circular buffer object */
typedef struct {
    int         size;   /* maximum number of elements           */
    int         start;  /* index of oldest element              */
    int         end;    /* index at which to write new element  */
    ElemType   *elems;  /* vector of elements                   */
} IrQueue;


void IrQueueInit(IrQueue *cb, int size);
void IrQueueFree(IrQueue *queue);

int IrQueueIsFull(IrQueue *queue);
int IrQueueIsEmpty(IrQueue *queue);

/* Write an element, overwriting oldest element if buffer is full. App can
   choose to avoid the overwrite by checking cbIsFull(). */
void IrQueueWrite(IrQueue *queue, ElemType *elem);

/* Read oldest element. App must ensure !cbIsEmpty() first. */
void IrQueueRead(IrQueue *queue, ElemType *elem);

#endif	// __IR_QUEUE__H__
