#include "median_filter.h"

#include <math.h>
#include <stdbool.h>

static Status list_insert_empty(MedianFilter* filter, float sample);
static Status list_insert_nonempty(MedianFilter* filter, float sample);
static Status list_find_median(MedianFilter* filter);
static Status list_delete(MedianFilter* filter);

Status median_filter_init(MedianFilter* filter, size_t capacity) {
    // Try to allocate space for the filter
    filter->data = malloc(capacity * sizeof(MedianFilterNode));
    if (filter->data == NULL) {
        return STATUS_MEMORY_ERROR;
    }
    filter->capacity = capacity;

    // Initialize the rest of the filter members
    median_filter_reset(filter);

    return STATUS_OK;
}

Status median_filter_reset(MedianFilter* filter) {
    filter->size = 0;
    filter->tail = 0;
    filter->head = 0;
    filter->bias = 0;
    filter->max = NULL;
    filter->min = NULL;
    filter->median = NULL;

    return STATUS_OK;
}

Status median_filter_insert(MedianFilter* filter, float sample) {
    if (isnan(sample)) {
        // If we're already empty, nothing to be done
        if (filter->size == 0) {
            return STATUS_OK;
        }

        // Otherwise delete a sample
        return list_delete(filter);
    }

    if (filter->size == 0) {
        return list_insert_empty(filter, sample);
    }

    // If we're full, delete a sample
    if (filter->size == filter->capacity) {
        Status status = list_delete(filter);
        if (status != STATUS_OK) {
            return status;
        }
    }

    return list_insert_nonempty(filter, sample);
}

float median_filter_get_median(MedianFilter* filter) {
    if (filter->size == 0) {
        return NAN;
    }

    if (filter->bias < 0) {
        // Biased left; average median and right element
        float left = filter->median->sample;
        float right = filter->median->next->sample;
        return (left + right) / 2.;
    } else if (filter->bias > 0) {
        // Biased right; average median and left element
        float right = filter->median->sample;
        float left = filter->median->prev->sample;
        return (left + right) / 2.;
    } else {
        // Not biased; median is true center
        return filter->median->sample;
    }
}

static Status list_delete(MedianFilter* filter) {
    if (filter->size == 0) {
        return STATUS_PARAMETER_ERROR;
    }

    // Get pointer to node to delete and fixup tracking info
    MedianFilterNode* node = &filter->data[filter->tail++];
    if (filter->tail == filter->capacity) {
        filter->tail = 0;
    }
    filter->size -= 1;

    // If we deleted the last element, just reset everything
    if (filter->size == 0) {
        return median_filter_reset(filter);
    }

    // Fix pointers of adjacent elements if they exist
    if (node->prev != NULL) {
        node->prev->next = node->next;
    }
    if (node->next != NULL) {
        node->next->prev = node->prev;
    }

    // If we were min or max, update those pointers as required
    if (node == filter->min) {
        filter->min = node->next;
    }
    if (node == filter->max) {
        filter->max = node->prev;
    }

    // Figure out where we're removing from
    float removed_sample = node->sample;
    float median_sample = filter->median->sample;

    bool removed_left;

    if (removed_sample < median_sample) {
        removed_left = true;
    } else if (removed_sample > median_sample) {
        removed_left = false;
    } else {
        // If the removed sample had the same value as the median, we have no
        // way to figure out the median and must do a search from scratch.
        return list_find_median(filter);
    }

    // Otherwise, figure out the new median
    if (filter->bias == 0) {
        // If the bias was previously zero, then that means we had an odd
        // number of elements with the median at the center. So, the median
        // element remains the same, but the bias shifts towards the removed.
        if (removed_left) {
            filter->bias = -1;
        } else {
            filter->bias = +1;
        }
    } else if (filter->bias == -1) {
        // If we were previously biased left, then removing on the left moves
        // the median to the right, and removing on the right keeps the median.
        filter->bias = 0;
        if (removed_left) {
            filter->median = filter->median->next;
        }
    } else if (filter->bias == +1) {
        // If we were previously biased right, then removing on the right moves
        // the median to the left, and removing on the left keeps the median.
        filter->bias = 0;
        if (!removed_left) {
            filter->median = filter->median->prev;
        }
    } else {
        // This should be impossible
        return STATUS_STATE_ERROR;
    }

    return STATUS_OK;
}

static Status list_find_median(MedianFilter* filter) {
    if (filter->size == 0) {
        return STATUS_PARAMETER_ERROR;
    }

    MedianFilterNode* node = filter->min;
    for (int i = 0; i < filter->size / 2; i++) {
        node = node->next;
    }
    filter->median = node;

    // This search method will always result in a right bias in case of an even
    // length, and as always, an odd length always implies zero bias
    if (filter->size % 2 == 0) {
        filter->bias = +1;
    } else {
        filter->bias = 0;
    }

    return STATUS_OK;
}

static Status list_insert_nonempty(MedianFilter* filter, float sample) {
    if (filter->size == 0 || filter->size == filter->capacity) {
        // This function not designed for empty insertion
        return STATUS_PARAMETER_ERROR;
    }

    // Get pointer to node to insert and fixup tracking info
    MedianFilterNode* node = &filter->data[filter->head++];
    if (filter->head == filter->capacity) {
        filter->head = 0;
    }
    filter->size += 1;

    // Figure out where the new sample should go
    bool inserted_left = true;
    MedianFilterNode* insert_before = filter->min;
    while (insert_before != NULL && insert_before->sample < sample) {
        if (insert_before == filter->median) {
            inserted_left = false;
        }
        insert_before = insert_before->next;
    }

    // Perform the insertion
    node->sample = sample;
    node->next = insert_before;
    if (insert_before == NULL) {
        // We are the new max element
        node->prev = filter->max;
        filter->max->next = node;
        filter->max = node;
    } else {
        node->prev = insert_before->prev;
        if (insert_before->prev == NULL) {
            // We are the new min element
            filter->min = node;
        } else {
            // We are in the middle
            insert_before->prev->next = node;
        }
        insert_before->prev = node;
    }

    // Otherwise, figure out the new median
    if (filter->bias == 0) {
        // If the bias was previously zero, then that means we had an odd
        // number of elements with the median at the center. So, the median
        // sample remains the same, but the bias shifts away from the inserted.
        if (inserted_left) {
            filter->bias = +1;
        } else {
            filter->bias = -1;
        }
    } else if (filter->bias == -1) {
        // If we were previously biased left, then inserting on the right moves
        // the median to the right, and inserting on the left keeps the median.
        filter->bias = 0;
        if (!inserted_left) {
            filter->median = filter->median->next;
        }
    } else if (filter->bias == +1) {
        // If we were previously biased right, then inserting on the left moves
        // the median to the left, and inserting on the right keeps the median.
        filter->bias = 0;
        if (inserted_left) {
            filter->median = filter->median->prev;
        }
    } else {
        // This should be impossible
        return STATUS_STATE_ERROR;
    }

    return STATUS_OK;
}

static Status list_insert_empty(MedianFilter* filter, float sample) {
    if (filter->size != 0) {
        return STATUS_PARAMETER_ERROR;
    }

    // Get a pointer to the node at which we're inserting
    MedianFilterNode* node = &filter->data[filter->head];

    // Only sample in the list, so next and prev are nullptrs
    node->sample = sample;
    node->next = NULL;
    node->prev = NULL;

    // This lone sample is the min, max, and median
    filter->min = node;
    filter->max = node;
    filter->median = node;

    // Update ring buffer tracking data
    filter->head += 1;
    if (filter->head >= filter->capacity) {
        filter->head = 0;
    }
    filter->size += 1;

    return STATUS_OK;
}
