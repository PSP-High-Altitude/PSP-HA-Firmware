#ifndef FIFOS_H
#define FIFOS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    uint8_t *buffer;  // Array for storing data
    int size;         // Size of the buffer
    int circ;         // Is circular?
    int head;         // Index of the first element
    int tail;         // Index of the last element
    int count;        // Number of elements in the buffer
} FIFO_t;

// Initialize the FIFO
void fifo_init(FIFO_t *fifo);

// Add an item to the FIFO
int fifo_enqueue(FIFO_t *fifo, uint8_t item);

// Add items to the FIFO
int fifo_enqueuen(FIFO_t *fifo, uint8_t *items, int n);

// Remove an item from the FIFO
int fifo_dequeue(FIFO_t *fifo, uint8_t *item);

// Remove items from the FIFO
int fifo_dequeuen(FIFO_t *fifo, uint8_t *items, int n);

// Delete items from the FIFO without deleting
void fifo_discardn(FIFO_t *fifo, int n);

// Peek at the first item in the FIFO
int fifo_peek(FIFO_t *fifo, uint8_t *item);

// Return the number of items readable contiguously in the FIFO
int fifo_size_contig(FIFO_t *fifo);

#endif  // FIFOS_H