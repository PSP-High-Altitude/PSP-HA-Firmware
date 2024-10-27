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
    buffer->tidx = 0;
    buffer->sum = 0;

    return STATUS_OK;
}

Status sma_float_buffer_insert_sample(SmaFloatBuffer* buffer, float sample) {
    // If the new sample is a NAN, abort to prevent NANovirus infection
    if (isnan(sample)) {
        return STATUS_PARAMETER_ERROR;
    }

    // If we're at capacity, remove the sample at the tail from the sum
    if (buffer->size == buffer->capacity) {
        buffer->sum -= buffer->data[buffer->tidx];
        buffer->size -= 1;
    }

    // Add the new sample to the buffer
    buffer->data[buffer->tidx] = sample;
    buffer->sum += sample;
    buffer->size += 1;

    // Update the tail index
    buffer->tidx += 1;
    if (buffer->tidx == buffer->capacity) {
        buffer->tidx = 0;
    }

    return STATUS_OK;
}

float sma_float_buffer_get_current_avg(SmaFloatBuffer* buffer) {
    // If the buffer is empty, return a NAN
    if (buffer->size == 0) {
        return NAN;
    }

    // Otherwise compute the average from the sum and size
    return buffer->sum / buffer->size;
}
