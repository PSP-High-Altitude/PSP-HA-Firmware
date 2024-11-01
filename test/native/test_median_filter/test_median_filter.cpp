#include "test_median_filter.hpp"

#include <gtest/gtest.h>
#include <math.h>

extern "C" {
#include "filter/median_filter.h"
}

static void filter_consistent(MedianFilter* filter) {
    if (filter->size == 0) {
        // If size is zero, anything goes
        return;
    }

    // Check valid size
    EXPECT_LE(filter->size, filter->capacity);

    // Check valid bias
    if (filter->size % 2 == 0) {
        // Even size must have bias
        EXPECT_NE(filter->bias, 0);
    } else if (filter->size % 2 == 1 && filter->bias != 0) {
        // Odd size musn't have bias
        EXPECT_EQ(filter->bias, 0);
    }

    // Find min, max, and length
    size_t length = 0;
    float min_sample = INFINITY;
    float max_sample = -INFINITY;

    MedianFilterNode* node = filter->min;
    min_sample = node->sample;
    while (node != nullptr) {
        if (node->sample < min_sample) {
            min_sample = node->sample;
        }
        if (node->sample > max_sample) {
            max_sample = node->sample;
        }
        node = node->next;
        length++;
    }

    // Check length of list
    EXPECT_EQ(length, filter->size);

    // Check valid min
    ASSERT_NE(filter->min, nullptr);
    EXPECT_EQ(filter->min->prev, nullptr);
    EXPECT_FLOAT_EQ(filter->min->sample, min_sample);

    // Check valid max
    ASSERT_NE(filter->max, nullptr);
    EXPECT_EQ(filter->max->next, nullptr);
    EXPECT_FLOAT_EQ(filter->max->sample, max_sample);

    // Check sortedness and median
    length = 0;
    node = filter->min;
    MedianFilterNode* median = nullptr;
    float last_sample = filter->min->sample;
    while (node != nullptr) {
        if (length == filter->size / 2 - 1) {
            if (filter->bias == -1) {
                median = node;
            }
        } else if (length == filter->size / 2) {
            if (filter->bias == 0 || filter->bias == +1) {
                median = node;
            }
        }
        EXPECT_GE(node->sample, last_sample);
        EXPECT_LE(node->sample, max_sample);
        last_sample = node->sample;
        node = node->next;
        length++;
    }

    EXPECT_EQ(median, filter->median);

    // Check head-tail consistency
    EXPECT_LT(filter->head, filter->capacity);
    EXPECT_LT(filter->tail, filter->capacity);
    if (filter->size == filter->capacity) {
        EXPECT_EQ(filter->head, filter->tail);
    } else {
        EXPECT_NE(filter->head, filter->tail);
    }
}

#define INSERT_AND_CHECK(filter, v)               \
    do {                                          \
        Status status;                            \
        status = median_filter_insert(filter, v); \
        ASSERT_EQ(status, STATUS_OK);             \
        filter_consistent(filter);                \
    } while (0)

TEST(TestMedianFilter, SingleInsert) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 10);
    ASSERT_EQ(status, STATUS_OK);

    INSERT_AND_CHECK(&filter, 0.5);
    ASSERT_EQ(filter.size, 1);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 0.5);
}

TEST(TestMedianFilter, InsertMin) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 10);
    ASSERT_EQ(status, STATUS_OK);

    INSERT_AND_CHECK(&filter, 0.5);
    ASSERT_EQ(filter.size, 1);

    INSERT_AND_CHECK(&filter, -1.5);
    ASSERT_EQ(filter.size, 2);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), -0.5);
}

TEST(TestMedianFilter, InsertMax) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 10);
    ASSERT_EQ(status, STATUS_OK);

    INSERT_AND_CHECK(&filter, 0.5);
    ASSERT_EQ(filter.size, 1);

    INSERT_AND_CHECK(&filter, 1.5);
    ASSERT_EQ(filter.size, 2);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 1.0);
}

TEST(TestMedianFilter, InsertMiddle) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 10);
    ASSERT_EQ(status, STATUS_OK);

    INSERT_AND_CHECK(&filter, 1);
    ASSERT_EQ(filter.size, 1);

    INSERT_AND_CHECK(&filter, 3);
    ASSERT_EQ(filter.size, 2);

    // Insert middle center
    INSERT_AND_CHECK(&filter, 2);
    ASSERT_EQ(filter.size, 3);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 2);

    // Insert middle left
    INSERT_AND_CHECK(&filter, 1.5);
    ASSERT_EQ(filter.size, 4);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), (1.5 + 2) / 2);

    // Insert middle center
    INSERT_AND_CHECK(&filter, 1.75);
    ASSERT_EQ(filter.size, 5);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 1.75);

    // Insert middle right
    INSERT_AND_CHECK(&filter, 1.9);
    ASSERT_EQ(filter.size, 6);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), (1.75 + 1.9) / 2);

    // Insert middle left, not next to center
    INSERT_AND_CHECK(&filter, 1.6);
    ASSERT_EQ(filter.size, 7);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 1.75);

    // Insert middle right, not next to center
    INSERT_AND_CHECK(&filter, 1.8);
    ASSERT_EQ(filter.size, 8);

    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), (1.75 + 1.8) / 2);
}

TEST(TestMedianFilter, ReplaceCapacity2) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 2);
    ASSERT_EQ(status, STATUS_OK);

    INSERT_AND_CHECK(&filter, 1);
    ASSERT_EQ(filter.size, 1);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 1);

    INSERT_AND_CHECK(&filter, 2);
    ASSERT_EQ(filter.size, 2);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 1.5);

    // Replace inactive element
    INSERT_AND_CHECK(&filter, 3);
    ASSERT_EQ(filter.size, 2);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 2.5);

    // Replace active element
    INSERT_AND_CHECK(&filter, 1);
    ASSERT_EQ(filter.size, 2);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 2);

    // Replace active element again
    INSERT_AND_CHECK(&filter, 4);
    ASSERT_EQ(filter.size, 2);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 2.5);

    // Replace inactive element
    INSERT_AND_CHECK(&filter, 5);
    ASSERT_EQ(filter.size, 2);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 4.5);

    // Replace inactive element again
    INSERT_AND_CHECK(&filter, 7);
    ASSERT_EQ(filter.size, 2);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Replace active element
    INSERT_AND_CHECK(&filter, -1);
    ASSERT_EQ(filter.size, 2);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 3);
}

TEST(TestMedianFilter, ReplaceCapacity5) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 5);
    ASSERT_EQ(status, STATUS_OK);

    const static float init_inputs[] = {5, 1, 8, 3, 9};
    const static float init_medians[] = {5, 3, 5, 4, 5};

    for (int i = 0; i < 5; i++) {
        INSERT_AND_CHECK(&filter, init_inputs[i]);
        ASSERT_EQ(filter.size, i + 1);
        EXPECT_FLOAT_EQ(median_filter_get_median(&filter), init_medians[i]);
    }

    // Replace median with new median (1, 8, 3, 9, 6)
    INSERT_AND_CHECK(&filter, 6);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Replace min element with new min element (8, 3, 9, 6, -1)
    INSERT_AND_CHECK(&filter, -1);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // New min element with middle replacement (3, 9, 6, -1, -2)
    INSERT_AND_CHECK(&filter, -2);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 3);

    // New middle with median replacement (9, 6, -1, -2, 7)
    INSERT_AND_CHECK(&filter, 7);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Replace max with smaller max (6, -1, -2, 7, 8)
    INSERT_AND_CHECK(&filter, 8);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Insert near edge while median replaced (-1, -2, 7, 8, 7.5)
    INSERT_AND_CHECK(&filter, 7.5);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 7);

    // New median with edge replacement (-2, 7, 8, 7.5, 7.2)
    INSERT_AND_CHECK(&filter, 7.2);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 7.2);
}

TEST(TestMedianFilter, ReplaceDuplicateMedian) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 5);
    ASSERT_EQ(status, STATUS_OK);

    const static float init_inputs[] = {5, 1, 8, 3, 9};
    const static float init_medians[] = {5, 3, 5, 4, 5};

    for (int i = 0; i < 5; i++) {
        INSERT_AND_CHECK(&filter, init_inputs[i]);
        ASSERT_EQ(filter.size, i + 1);
        EXPECT_FLOAT_EQ(median_filter_get_median(&filter), init_medians[i]);
    }

    // Replace median with new median (1, 8, 3, 9, 6)
    INSERT_AND_CHECK(&filter, 6);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Insert a duplicate of the median (8, 3, 9, 6, 6)
    INSERT_AND_CHECK(&filter, 6);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Insert another duplicate of the median (3, 9, 6, 6, 6)
    INSERT_AND_CHECK(&filter, 6);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Make one of the edges equal the median (9, 6, 6, 6, 7)
    INSERT_AND_CHECK(&filter, 7);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Maintain one of the edges as median (6, 6, 6, 7, 10)
    INSERT_AND_CHECK(&filter, 10);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 6);

    // Add a duplicate of the new median (6, 6, 7, 10, 7)
    INSERT_AND_CHECK(&filter, 7);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 7);

    // Let the duplicates bubble through (6, 7, 10, 7, 11)
    INSERT_AND_CHECK(&filter, 11);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 7);

    // Let the duplicates bubble through (7, 10, 7, 11, 5)
    INSERT_AND_CHECK(&filter, 5);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 7);

    // Let the duplicates bubble through (10, 7, 11, 5, 3)
    INSERT_AND_CHECK(&filter, 3);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 7);

    // Let the duplicates bubble through (7, 11, 5, 3, 9)
    INSERT_AND_CHECK(&filter, 9);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 7);

    // Insert new median same as outgoing (11, 5, 3, 9, 7)
    INSERT_AND_CHECK(&filter, 7);
    ASSERT_EQ(filter.size, 5);
    EXPECT_FLOAT_EQ(median_filter_get_median(&filter), 7);
}

TEST(TestMedianFilter, LongVectorCapacity11) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 11);
    ASSERT_EQ(status, STATUS_OK);

    for (int i = 0; i < sizeof(s_tv_cap11_len100_in) / sizeof(float); i++) {
        INSERT_AND_CHECK(&filter, s_tv_cap11_len100_in[i]);
        EXPECT_FLOAT_EQ(median_filter_get_median(&filter),
                        s_tv_cap11_len100_out[i]);
    }
}

TEST(TestMedianFilter, LongVectorCapacity20) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 20);
    ASSERT_EQ(status, STATUS_OK);

    for (int i = 0; i < sizeof(s_tv_cap20_len1000_in) / sizeof(float); i++) {
        INSERT_AND_CHECK(&filter, s_tv_cap20_len1000_in[i]);
        EXPECT_FLOAT_EQ(median_filter_get_median(&filter),
                        s_tv_cap20_len1000_out[i]);
    }
}

TEST(TestMedianFilter, LongVectorDuplicates) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 5);
    ASSERT_EQ(status, STATUS_OK);

    for (int i = 0; i < sizeof(s_tv_cap5_duplicates_in) / sizeof(float); i++) {
        INSERT_AND_CHECK(&filter, s_tv_cap5_duplicates_in[i]);
        EXPECT_FLOAT_EQ(median_filter_get_median(&filter),
                        s_tv_cap5_duplicates_out[i]);
    }
}

TEST(TestMedianFilter, LongVectorNANs) {
    MedianFilter filter;

    Status status = median_filter_init(&filter, 10);
    ASSERT_EQ(status, STATUS_OK);

    for (int i = 0; i < sizeof(s_tv_cap10_NANs_in) / sizeof(float); i++) {
        INSERT_AND_CHECK(&filter, s_tv_cap10_NANs_in[i]);
        float out = median_filter_get_median(&filter);
        if (isnan(s_tv_cap10_NANs_out[i])) {
            EXPECT_TRUE(isnan(out));
        } else {
            EXPECT_FLOAT_EQ(median_filter_get_median(&filter),
                            s_tv_cap10_NANs_out[i]);
        }
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
        ;
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
