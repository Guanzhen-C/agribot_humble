#include "hechuan_motion.h"

#include <string.h>

static uint8_t checksum(const uint8_t *data, size_t length)
{
    uint8_t value = 0U;
    for (size_t i = 0U; i < length; i++) {
        value ^= data[i];
    }
    return value;
}

static int16_t read_i16_be(const uint8_t *data)
{
    uint16_t value = (uint16_t)((uint16_t)data[0] << 8U) | data[1];
    int32_t signed_value = (value & 0x8000U) ? (int32_t)value - 65536
                                           : (int32_t)value;
    return (int16_t)signed_value;
}

static void write_i16_be(uint8_t *data, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    data[0] = (uint8_t)(bits >> 8U);
    data[1] = (uint8_t)bits;
}

void hechuan_encode_serial(const hechuan_velocity_t *velocity,
                           uint8_t frame[HECHUAN_SERIAL_COMMAND_SIZE])
{
    memset(frame, 0, HECHUAN_SERIAL_COMMAND_SIZE);
    frame[0] = 0x7BU;
    write_i16_be(&frame[3], velocity->forward_mm_s);
    write_i16_be(&frame[5], velocity->lateral_mm_s);
    write_i16_be(&frame[7], velocity->yaw_mrad_s);
    frame[9] = checksum(frame, 9U);
    frame[10] = 0x7DU;
}

bool hechuan_decode_serial(const uint8_t *frame, size_t length,
                           hechuan_velocity_t *velocity)
{
    if (length != HECHUAN_SERIAL_COMMAND_SIZE || frame[0] != 0x7BU
        || frame[10] != 0x7DU || frame[9] != checksum(frame, 9U)
        || frame[1] != 0U || frame[2] != 0U) {
        return false;
    }
    velocity->forward_mm_s = read_i16_be(&frame[3]);
    velocity->lateral_mm_s = read_i16_be(&frame[5]);
    velocity->yaw_mrad_s = read_i16_be(&frame[7]);
    return true;
}

void hechuan_encode_velocity_can(const hechuan_velocity_t *velocity,
                                 uint8_t data[8])
{
    memset(data, 0, 8U);
    write_i16_be(&data[0], velocity->forward_mm_s);
    write_i16_be(&data[2], velocity->lateral_mm_s);
    write_i16_be(&data[4], velocity->yaw_mrad_s);
}

void hechuan_encode_steering_can(int16_t forward_mm_s,
                                 int16_t left_steering_mrad, uint8_t data[8])
{
    memset(data, 0, 8U);
    write_i16_be(&data[0], forward_mm_s);
    write_i16_be(&data[4], left_steering_mrad);
}

bool hechuan_decode_feedback(const uint8_t *frame, size_t length,
                             hechuan_feedback_t *feedback)
{
    if (length != HECHUAN_FEEDBACK_SIZE || frame[0] != 0x7BU
        || frame[23] != 0x7DU || frame[22] != checksum(frame, 22U)
        || frame[1] > 1U) {
        return false;
    }
    feedback->drive_disabled = frame[1] == 1U;
    feedback->velocity.forward_mm_s = read_i16_be(&frame[2]);
    feedback->velocity.lateral_mm_s = read_i16_be(&frame[4]);
    feedback->velocity.yaw_mrad_s = read_i16_be(&frame[6]);
    for (size_t i = 0U; i < 3U; i++) {
        feedback->acceleration_raw[i] = read_i16_be(&frame[8U + i * 2U]);
        feedback->angular_velocity_raw[i] = read_i16_be(&frame[14U + i * 2U]);
    }
    feedback->voltage_mv = read_i16_be(&frame[20]);
    return true;
}
