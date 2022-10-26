#include <gtest/gtest.h>

extern "C" {
#include "ms5637/ms5637.h"
#include "status.h"
}

TEST(TestMS5637, Init) { EXPECT_EQ(ms5637_init(), OK); }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
        ;
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
