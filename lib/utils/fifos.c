#include "fifos.h"

// Initialize the FIFO
void fifo_init(FIFO_t *fifo) {
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

// Add an item to the FIFO
int fifo_enqueue(FIFO_t *fifo, uint8_t item) {
    if (fifo->count == fifo->size && !fifo->circ) {
        return 0;  // Buffer full
    } else if (fifo->count == fifo->size && fifo->circ) {
        fifo->head = (fifo->head + 1) % fifo->size;
        fifo->count--;
    }

    fifo->buffer[fifo->tail] = item;
    fifo->tail = (fifo->tail + 1) % fifo->size;
    fifo->count++;
    return 1;
}

// Add items to the FIFO
int fifo_enqueuen(FIFO_t *fifo, uint8_t *items, int n) {
    for (int i = 0; i < n; i++) {
        if (!fifo_enqueue(fifo, items[i])) {
            return i;
        }
    }

    return n;
}

// Remove an item from the FIFO
int fifo_dequeue(FIFO_t *fifo, uint8_t *item) {
    if (fifo->count == 0) {
        return 0;  // Buffer empty
    }

    *item = fifo->buffer[fifo->head];
    fifo->head = (fifo->head + 1) % fifo->size;
    fifo->count--;
    return 1;
}

// Remove items from the FIFO
int fifo_dequeuen(FIFO_t *fifo, uint8_t *items, int n) {
    for (int i = 0; i < n; i++) {
        if (!fifo_dequeue(fifo, items + i)) {
            return i;
        }
    }

    return n;
}

// Delete items from the FIFO without deleting
void fifo_discardn(FIFO_t *fifo, int n) {
    fifo->head = (fifo->head + n) % fifo->size;
    fifo->count -= n;
}

// Peek at the first item in the FIFO
int fifo_peek(FIFO_t *fifo, uint8_t *item) {
    if (fifo->count == 0) {
        return 0;
    }

    *item = fifo->buffer[fifo->head];
    return 1;
}

// Return the number of items readable contiguously in the FIFO
int fifo_size_contig(FIFO_t *fifo) {
    int items_till_buf_end = fifo->size - fifo->head;

    if (items_till_buf_end > fifo->count) {
        return fifo->count;
    } else {
        return items_till_buf_end;
    }
}
