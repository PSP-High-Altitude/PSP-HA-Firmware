#include <gtest/gtest.h>

TEST(TestTests, Add2To2) {
    uint32_t a = 2;
    uint32_t b = 2;
    EXPECT_EQ(a + b, 4);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
        ;
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
