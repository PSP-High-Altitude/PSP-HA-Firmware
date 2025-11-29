#ifndef SMA_FILTER_H
#define SMA_FILTER_H

#include <stdlib.h>

#include "status.h"

typedef struct {
    float* data;      // Array for storing samples
    size_t capacity;  // Number of slots in the filter
    size_t size;      // Number of samples in the filter
    size_t tail;      // Index of the oldest sample in the filter
    size_t head;      // Index at which the newest sample is to be inserted
    float sum;        // Sum of the samples in the filter
} SmaFilter;

Status sma_filter_init(SmaFilter* filter, size_t capacity);

Status sma_filter_reset(SmaFilter* filter);

Status sma_filter_insert(SmaFilter* filter, float sample);

float sma_filter_get_mean(SmaFilter* filter);

#endif  // SMA_FILTER_H
