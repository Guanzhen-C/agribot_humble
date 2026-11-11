#ifndef AGRIBOT_HIL_POWERTRAIN_H
#define AGRIBOT_HIL_POWERTRAIN_H
#include "hitch_control.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Isolated nominal tractor transmission, never a real vehicle's CAN profile. */
enum { PT_VERSION = 0x22, PT_ENGINE = 0x640, PT_TRANSMISSION = 0x641,
       PT_ENGINE_SENSOR = 0x642, PT_TRANS_SENSOR = 0x643, PT_VEHICLE_SENSOR = 0x644,
       PT_LOAD_SENSOR = 0x645 };
enum { PT_OFF, PT_POWERSHIFT_MANUAL, PT_POWERSHIFT_AUTO, PT_CVT_AUTO, PT_CVT_MANUAL };
enum { PT_STEADY, PT_FILL, PT_HANDOVER };
enum { PT_STALE = 1, PT_SENSOR_FAULT = 2, PT_OVERSPEED = 4 };
enum { PT_MOVING_MODE_CHANGE = 6 };
typedef struct {
    float engine_rpm, input_rpm, input_torque, ratio, pressure_a, pressure_b;
    float speed, output_rpm, load_force;
    unsigned gear, mode, fault;
} pt_measurement;
typedef struct {
    float throttle, clutch_a, clutch_b, ratio;
    unsigned gear_a, gear_b, mode;
    bool enabled, brake;
} pt_actuation;
typedef struct {
    pt_measurement measured;
    pt_actuation output;
    unsigned mode, requested_gear, gear, next_gear, shift_state;
    float requested_rpm, requested_ratio, speed_reference, rpm_integral, ratio_integral;
    float shift_seconds, ratio_rate;
    uint32_t profile_at, drive_at, sensor_at, shift_at, step_at, config_at;
    uint8_t profile_sequence, drive_sequence, sensor_sequence, config_sequence, output_sequence;
    uint8_t pending_sensor[3][8], fault;
    uint32_t pending_at[3];
    bool have_profile, have_drive, have_sensor, have_step, have_config, primed, drive_enabled;
    float demanded_speed;
} pt_controller;
void pt_init(pt_controller *p);
uint8_t pt_profile(pt_controller *p, const uint8_t data[8], uint32_t now);
uint8_t pt_settings(pt_controller *p, const uint8_t data[8], uint32_t now);
bool pt_drive(pt_controller *p, const uint8_t data[8], uint32_t now);
bool pt_sensor(pt_controller *p, uint16_t id, const uint8_t data[8], uint32_t now);
void pt_step(pt_controller *p, uint32_t now, bool network);
void pt_output(pt_controller *p, uint8_t engine[8], uint8_t transmission[8]);
void pt_status(const pt_controller *p, uint8_t data[8]);
void pt_make_profile(unsigned mode, unsigned gear, unsigned engine_rpm, float ratio,
                     uint8_t sequence, uint8_t data[8]);
float pt_gear_ratio(unsigned gear);
#ifdef __cplusplus
}
#endif
#endif
