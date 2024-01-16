#include <gtest/gtest.h>

extern "C" {
#include "fatfs/ff.h"
#include "pb_decode.h"
#include "sd.h"
#include "sensor.pb.h"
}

FATFS fs;             /* Filesystem object */
FIL fil;              /* File object */
FRESULT res;          /* API result code */
UINT bw;              /* Bytes written */
UINT br;              /* Bytes read */
BYTE work[FF_MAX_SS]; /* Work area (larger is better for processing time) */

static const char s_indiv_fname[] = "indiv.txt";
static const char s_indiv_payload[] = "Some string\r\n";

static SdDevice s_sd_device;

#define NUM_TEST_SENSOR_FRAMES 32
static SensorFrame s_test_sensor_frames[NUM_TEST_SENSOR_FRAMES];

// Function to generate random float within a specified range
float random_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

// Function to generate a random SensorFrame
SensorFrame generate_random_sensor_frame() {
    SensorFrame sensor_frame;

    // Generate random values for the fields
    sensor_frame.timestamp =
        (uint64_t)time(NULL);  // Use current time as timestamp
    sensor_frame.temperature = random_float(20.0, 30.0);
    sensor_frame.pressure = random_float(1000.0, 1100.0);
    sensor_frame.acc_h_x = random_float(-1.0, 1.0);
    sensor_frame.acc_h_y = random_float(-1.0, 1.0);
    sensor_frame.acc_h_z = random_float(-1.0, 1.0);
    sensor_frame.acc_i_x = random_float(-2.0, 2.0);
    sensor_frame.acc_i_y = random_float(-2.0, 2.0);
    sensor_frame.acc_i_z = random_float(-2.0, 2.0);
    sensor_frame.rot_i_x = random_float(-500.0, 500.0);
    sensor_frame.rot_i_y = random_float(-500.0, 500.0);
    sensor_frame.rot_i_z = random_float(-500.0, 500.0);
    sensor_frame.mag_i_x = random_float(-0.1, 0.1);
    sensor_frame.mag_i_y = random_float(-0.1, 0.1);
    sensor_frame.mag_i_z = random_float(-0.1, 0.1);

    return sensor_frame;
}

TEST(TestFatFs, MkFs) {
    res = f_mkfs("", 0, &work, sizeof(work));
    ASSERT_EQ(res, FR_OK);
}

TEST(TestFatFs, Mount) {
    res = f_mount(&fs, "", 0);
    ASSERT_EQ(res, FR_OK);
}

TEST(TestFatFs, Open) {
    res = f_open(&fil, s_indiv_fname, FA_CREATE_NEW | FA_WRITE);
    ASSERT_EQ(res, FR_OK);
}

TEST(TestFatFs, Write) {
    res = f_write(&fil, s_indiv_payload, sizeof(s_indiv_payload), &bw);
    EXPECT_EQ(res, FR_OK);
    EXPECT_EQ(bw, sizeof(s_indiv_payload));
}

TEST(TestFatFs, Close) {
    res = f_close(&fil);
    EXPECT_EQ(res, FR_OK);
}

TEST(TestFatFs, Read) {
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

TEST(TestFatFs, Unmount) {
    res = f_unmount("");
    EXPECT_EQ(res, FR_OK);
}

TEST(TestSd, Init) {
    s_sd_device.clk = SD_SPEED_10MHz;
    s_sd_device.periph = P_SD3;

    Status status = sd_init(&s_sd_device);
    ASSERT_EQ(status, STATUS_OK);
}

TEST(TestSd, WriteSensorOne) {
    Status status = sd_write_sensor_data(&(s_test_sensor_frames[0]));
    EXPECT_EQ(status, STATUS_OK);
}

TEST(TestSd, WriteSensorMulti) {
    for (int i = 1; i < NUM_TEST_SENSOR_FRAMES; i++) {
        Status status = sd_write_sensor_data(&(s_test_sensor_frames[i]));
        EXPECT_EQ(status, STATUS_OK);

        // Flush on somewhat irregular schedule
        if ((i % 7 == 0) || (i % 13 == 0)) {
            Status status = sd_flush();
            EXPECT_EQ(status, STATUS_OK);
        }
    }
}

TEST(TestSd, WriteGpsOne) {
    GpsFrame gps_frame = {
        .timestamp = 123456789,
        .year = 2022,
        .month = 1,
        .day = 14,
        .hour = 12,
        .min = 30,
        .sec = 45,
        .valid_flags = 0x0F,
        .num_sats = 10,
        .lon = -73.935242,
        .lat = 40.730610,
        .height = 100.0,
        .height_msl = 150.0,
        .accuracy_horiz = 5.0,
        .accuracy_vertical = 3.0,
        .vel_north = 0.5,
        .vel_east = -0.2,
        .vel_down = -0.1,
        .ground_speed = 0.6,
        .hdg = 90.0,
        .accuracy_speed = 0.1,
        .accuracy_hdg = 1.0,
    };

    Status status = sd_write_gps_data(&gps_frame);
    EXPECT_EQ(status, STATUS_OK);
}

TEST(TestSd, Deinit) {
    Status status = sd_deinit();
    EXPECT_EQ(status, STATUS_OK);
}

TEST(TestPb, Decode) {
    res = f_mount(&fs, "", 0);
    ASSERT_EQ(res, FR_OK);

    res = f_open(&fil, "dat_00.pb3", FA_READ);
    ASSERT_EQ(res, FR_OK);

    pb_byte_t buff[256];  // Same as SENSOR_BUF_LEN from sd.c

    // Read out header
    res = f_read(&fil, buff, 64, &br);
    ASSERT_EQ(res, FR_OK);
    ASSERT_EQ(br, 64);

    int i = 0;
    uint8_t message_size;
    res = f_read(&fil, &message_size, sizeof(message_size), &br);
    while (res == FR_OK && br > 0) {
        ASSERT_LE(message_size, 256);

        // Read the actual message
        res = f_read(&fil, buff, message_size, &br);
        ASSERT_EQ(res, FR_OK);
        ASSERT_EQ(br, message_size);

        // Decode the message
        SensorFrame frame;
        pb_istream_t istream = pb_istream_from_buffer(buff, message_size);
        bool decode_success = pb_decode(&istream, &SensorFrame_msg, &frame);

        // Check the frame against the original copy
        EXPECT_TRUE(decode_success);
        EXPECT_FALSE(
            memcmp(&frame, &(s_test_sensor_frames[i]), sizeof(SensorFrame)));

        // Increment induction variable and read next message length
        i++;
        res = f_read(&fil, &message_size, sizeof(message_size), &br);
    }

    EXPECT_EQ(i, NUM_TEST_SENSOR_FRAMES);

    res = f_close(&fil);
    EXPECT_EQ(res, FR_OK);
}

int main(int argc, char **argv) {
    // Initialize random sensor frames
    for (int i = 0; i < NUM_TEST_SENSOR_FRAMES; i++) {
        s_test_sensor_frames[i] = generate_random_sensor_frame();
    }

    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
        ;
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
