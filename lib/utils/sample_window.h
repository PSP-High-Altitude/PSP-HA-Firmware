#ifndef SAMPLE_WINDOW_H
#define SAMPLE_WINDOW_H

#include <stdlib.h>

#include "status.h"

typedef struct {
    float* data;      // Array for storing samples
    size_t capacity;  // Number of slots in the window
    size_t size;      // Number of samples in the window
    size_t tail;      // Index of the oldest sample in the window
    size_t head;      // Index at which the newest sample is to be inserted
    float sum;        // Sum of the samples in the window
} SampleWindow;

Status sample_window_init(SampleWindow* window, size_t capacity);

Status sample_window_reset(SampleWindow* window);

Status sample_window_insert(SampleWindow* window, float sample);

float sample_window_get(SampleWindow* window, size_t idx);

#endif  // SAMPLE_WINDOW_H
