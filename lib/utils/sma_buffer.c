#include "sma_buffer.h"

#include <math.h>

Status sma_float_buffer_init(SmaFloatBuffer* buffer, size_t capacity) {
    // Try to allocate space for the buffer
    buffer->data = malloc(capacity * sizeof(float));
    if (buffer->data == NULL) {
        return STATUS_MEMORY_ERROR;
    }

    // Initialize the rest of the buffer members
    buffer->capacity = capacity;
    buffer->size = 0;
    buffer->tail = 0;
    buffer->head = 0;
    buffer->sum = 0;

    return STATUS_OK;
}

#include <stdbool.h>

Status sma_float_buffer_insert_sample(SmaFloatBuffer* buffer, float sample) {
    // If we're at capacity, remove the sample at the tail
    if (buffer->size == buffer->capacity) {
        buffer->sum -= buffer->data[buffer->tail++];
        buffer->size -= 1;
    }

    if (!isnan(sample)) {
        // Add the new sample to the buffer if it's not NAN
        buffer->data[buffer->head++] = sample;
        buffer->sum += sample;
        buffer->size += 1;
    } else {
        // If it is actually NAN, then remove a sampleif we didn't already
        // remove one due to reaching capacity (and if it's not empty)
        if (0 < buffer->size && buffer->size < buffer->capacity - 1) {
            buffer->sum -= buffer->data[buffer->tail++];
            buffer->size -= 1;
        }
    }

    // Fix indices if they overflowed
    buffer->head = (buffer->head < buffer->capacity) ? buffer->head : 0;
    buffer->tail = (buffer->tail < buffer->capacity) ? buffer->tail : 0;

    return STATUS_OK;
}

float sma_float_buffer_get_avg(SmaFloatBuffer* buffer) {
    // If the buffer is empty, return a NAN
    if (buffer->size == 0) {
        return NAN;
    }

    // Otherwise compute the average from the sum and size
    return buffer->sum / buffer->size;
}
