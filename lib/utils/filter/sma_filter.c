#include "sma_filter.h"

#include <math.h>

Status sma_filter_init(SmaFilter* filter, size_t capacity) {
    // Try to allocate space for the filter
    filter->data = malloc(capacity * sizeof(float));
    if (filter->data == NULL) {
        return STATUS_MEMORY_ERROR;
    }
    filter->capacity = capacity;

    // Initialize the rest of the filter members
    sma_filter_reset(filter);

    return STATUS_OK;
}

Status sma_filter_reset(SmaFilter* filter) {
    filter->size = 0;
    filter->tail = 0;
    filter->head = 0;
    filter->sum = 0;

    return STATUS_OK;
}

Status sma_filter_insert(SmaFilter* filter, float sample) {
    // If we're at capacity, remove the sample at the tail
    if (filter->size == filter->capacity) {
        filter->sum -= filter->data[filter->tail++];
        filter->size -= 1;
    }

    if (!isnan(sample)) {
        // Add the new sample to the filter if it's not NAN
        filter->data[filter->head++] = sample;
        filter->sum += sample;
        filter->size += 1;
    } else {
        // If it is actually NAN, then remove a sample if we didn't already
        // remove one due to reaching capacity (and if it's not empty)
        if (0 < filter->size && filter->size < filter->capacity - 1) {
            filter->sum -= filter->data[filter->tail++];
            filter->size -= 1;
        }
    }

    // Fix indices if they overflowed
    filter->head = (filter->head < filter->capacity) ? filter->head : 0;
    filter->tail = (filter->tail < filter->capacity) ? filter->tail : 0;

    return STATUS_OK;
}

float sma_filter_get_mean(SmaFilter* filter) {
    // If the filter is empty, return a NAN
    if (filter->size == 0) {
        return NAN;
    }

    // Otherwise compute the average from the sum and size
    return filter->sum / filter->size;
}
