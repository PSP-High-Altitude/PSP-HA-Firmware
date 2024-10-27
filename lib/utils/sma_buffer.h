#ifndef SMA_BUFFER_H
#define SMA_BUFFER_H

#include <stdlib.h>

#include "status.h"

typedef struct {
    float* data;      // Array for storing samples
    size_t capacity;  // Number of slots in the buffer
    size_t size;      // Number of samples in the buffer
    size_t tidx;      // Index of the oldest sample in the buffer
    float sum;        // Sum of the samples in the buffer
} SmaFloatBuffer;

Status sma_float_buffer_init(SmaFloatBuffer* buffer, size_t capacity);

Status sma_float_buffer_insert_sample(SmaFloatBuffer* buffer, float sample);

float sma_float_buffer_get_current_avg(SmaFloatBuffer* buffer);

#endif  // SMA_BUFFER_H
