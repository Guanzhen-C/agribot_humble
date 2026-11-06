#ifndef HECHUAN_MOTION_H
#define HECHUAN_MOTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HECHUAN_SERIAL_COMMAND_SIZE 11U
#define HECHUAN_FEEDBACK_SIZE 24U
#define HECHUAN_CAN_COMMAND_ID 0x181U
#define HECHUAN_CAN_BITRATE 1000000U

typedef struct {
    int16_t forward_mm_s;
    int16_t lateral_mm_s;
    int16_t yaw_mrad_s;
} hechuan_velocity_t;

typedef struct {
    bool drive_disabled;
    hechuan_velocity_t velocity;
    int16_t acceleration_raw[3];
    int16_t angular_velocity_raw[3];
    int16_t voltage_mv;
} hechuan_feedback_t;

void hechuan_encode_serial(const hechuan_velocity_t *velocity,
                           uint8_t frame[HECHUAN_SERIAL_COMMAND_SIZE]);
bool hechuan_decode_serial(const uint8_t *frame, size_t length,
                           hechuan_velocity_t *velocity);
void hechuan_encode_velocity_can(const hechuan_velocity_t *velocity,
                                 uint8_t data[8]);
void hechuan_encode_steering_can(int16_t forward_mm_s,
                                 int16_t left_steering_mrad, uint8_t data[8]);
bool hechuan_decode_feedback(const uint8_t *frame, size_t length,
                             hechuan_feedback_t *feedback);

#endif
