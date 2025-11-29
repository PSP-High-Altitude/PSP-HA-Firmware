#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <stdlib.h>

#include "status.h"

typedef struct MedianFilterNode {
    struct MedianFilterNode* next;
    struct MedianFilterNode* prev;
    float sample;
} MedianFilterNode;

typedef struct {
    MedianFilterNode* data;    // Array for storing samples
    size_t capacity;           // Number of slots in the buffer
    size_t size;               // Number of samples in the buffer
    size_t tail;               // Index of the oldest sample in the buffer
    size_t head;               // Index at which new sample is to be inserted
    int bias;                  // +1 -> median right; -1 -> median left
    MedianFilterNode* min;     // Pointer to node with minimum value
    MedianFilterNode* max;     // Pointer to node with maximum value
    MedianFilterNode* median;  // Pointer to node with median value
} MedianFilter;

Status median_filter_init(MedianFilter* filter, size_t capacity);

Status median_filter_reset(MedianFilter* filter);

Status median_filter_insert(MedianFilter* filter, float sample);

float median_filter_get_median(MedianFilter* filter);

#endif  // MEDIAN_FILTER_H
