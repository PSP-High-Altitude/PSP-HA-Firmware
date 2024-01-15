#include <gtest/gtest.h>

extern "C" {
#include "fatfs/ff.h"
#include "sd.h"
}

FATFS fs;             /* Filesystem object */
FIL fil;              /* File object */
FRESULT res;          /* API result code */
UINT bw;              /* Bytes written */
UINT br;              /* Bytes read */
BYTE work[FF_MAX_SS]; /* Work area (larger is better for processing time) */

static const char s_indiv_fname[] = "indiv.txt";
static const char s_indiv_payload[] = "Some string\r\n";

TEST(TestSd, MkFs) {
    res = f_mkfs("", 0, &work, sizeof(work));
    ASSERT_EQ(res, FR_OK);
}

TEST(TestSd, Mount) {
    res = f_mount(&fs, "", 0);
    ASSERT_EQ(res, FR_OK);
}

TEST(TestSd, Open) {
    res = f_open(&fil, s_indiv_fname, FA_CREATE_NEW | FA_WRITE);
    ASSERT_EQ(res, FR_OK);
}

TEST(TestSd, Write) {
    res = f_write(&fil, s_indiv_payload, sizeof(s_indiv_payload), &bw);
    EXPECT_EQ(res, FR_OK);
    EXPECT_EQ(bw, sizeof(s_indiv_payload));
}

TEST(TestSd, Close) {
    res = f_close(&fil);
    EXPECT_EQ(res, FR_OK);
}

TEST(TestSd, Read) {
    res = f_open(&fil, s_indiv_fname, FA_READ);
    ASSERT_EQ(res, FR_OK);

    char buff[32];

    res = f_read(&fil, buff, 32, &br);
    EXPECT_EQ(res, FR_OK);
    EXPECT_EQ(br, sizeof(s_indiv_payload));
    EXPECT_STREQ(buff, s_indiv_payload);

    res = f_close(&fil);
    EXPECT_EQ(res, FR_OK);
}

TEST(TestSd, Unmount) {
    res = f_unmount("");
    EXPECT_EQ(res, FR_OK);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
        ;
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
