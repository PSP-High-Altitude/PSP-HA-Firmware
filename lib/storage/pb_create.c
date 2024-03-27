#include "pb_create.h"

static pb_byte_t s_sensor_buffer[SENSOR_BUF_LEN];
static pb_byte_t s_gps_buffer[GPS_BUF_LEN];
static pb_byte_t s_state_buffer[STATE_BUF_LEN];

pb_byte_t* create_sensor_buffer(SensorFrame* frame, size_t* size) {
    pb_ostream_t sensor_ostream =
        pb_ostream_from_buffer(s_sensor_buffer, SENSOR_BUF_LEN);

    bool encode_success = pb_encode_ex(&sensor_ostream, &SensorFrame_msg, frame,
                                       PB_ENCODE_DELIMITED);
    if (!encode_success) {
        return NULL;
    }

    *size = sensor_ostream.bytes_written;

    return s_sensor_buffer;
}

pb_byte_t* create_gps_buffer(GpsFrame* frame, size_t* size) {
    pb_ostream_t gps_ostream =
        pb_ostream_from_buffer(s_gps_buffer, GPS_BUF_LEN);

    bool encode_success =
        pb_encode_ex(&gps_ostream, &GpsFrame_msg, frame, PB_ENCODE_DELIMITED);
    if (!encode_success) {
        return NULL;
    }

    *size = gps_ostream.bytes_written;

    return s_gps_buffer;
}

pb_byte_t* create_state_buffer(StateFrame* frame, size_t* size) {
    pb_ostream_t state_ostream =
        pb_ostream_from_buffer(s_state_buffer, STATE_BUF_LEN);

    bool encode_success = pb_encode_ex(&state_ostream, &StateFrame_msg, frame,
                                       PB_ENCODE_DELIMITED);
    if (!encode_success) {
        return NULL;
    }

    *size = state_ostream.bytes_written;

    return s_state_buffer;
}