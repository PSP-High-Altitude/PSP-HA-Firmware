#include <gtest/gtest.h>

extern "C" {
#include "i2c/i2c.h"
#include "ms5637/ms5637.h"
#include "status.h"
}

TEST(TestMS5637, Init) {
    I2cDevice device = {
        .address = 0b1110110,
        .clk = I2C_SPEED_FAST,
        .periph = I2C0,
    };
    EXPECT_EQ(ms5637_init(&device), OK);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
        ;
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
