#include "sample_window.h"

#include <math.h>

Status sample_window_init(SampleWindow* window, size_t capacity) {
    // Try to allocate space for the window
    window->data = malloc(capacity * sizeof(float));
    if (window->data == NULL) {
        return STATUS_MEMORY_ERROR;
    }
    window->capacity = capacity;

    // Initialize the rest of the window members
    sample_window_reset(window);

    return STATUS_OK;
}

Status sample_window_reset(SampleWindow* window) {
    window->size = 0;
    window->tail = 0;
    window->head = 0;

    return STATUS_OK;
}

Status sample_window_insert(SampleWindow* window, float sample) {
    // If we're at capacity, remove the sample at the tail
    if (window->size == window->capacity) {
        window->size -= 1;
    }

    if (!isnan(sample)) {
        // Add the new sample to the window if it's not NAN
        window->data[window->head++] = sample;
        window->size += 1;
    } else {
        // If it is actually NAN, then remove a sample if we didn't already
        // remove one due to reaching capacity (and if it's not empty)
        if (0 < window->size && window->size < window->capacity - 1) {
            window->size -= 1;
        }
    }

    // Fix indices if they overflowed
    window->head = (window->head < window->capacity) ? window->head : 0;
    window->tail = (window->tail < window->capacity) ? window->tail : 0;

    return STATUS_OK;
}

float sample_window_get(SampleWindow* window, size_t idx) {
    // If the requested index is beyond the current window size, return NAN
    if (idx >= window->size) {
        return NAN;
    }

    // Otherwise find and return the sample at that index
    return window->data[(window->tail + idx) % window->capacity];
}
