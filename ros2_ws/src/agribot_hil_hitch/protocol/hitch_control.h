#ifndef AGRIBOT_HIL_HITCH_CONTROL_H
#define AGRIBOT_HIL_HITCH_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Bench-only messages: never use these IDs with either real chassis driver. */
enum { HH_VERSION = 0x21, HH_DRIVE = 0x620, HH_DRIVE_STATE = 0x621,
       HH_VALVE = 0x630, HH_SENSOR = 0x631 };
enum { HH_HOLD, HH_POSITION, HH_RAISE, HH_LOWER };
enum { HH_OK, HH_FORMAT, HH_RANGE, HH_REPLAY, HH_UNPRIMED, HH_NOT_READY };
enum { HH_COMMAND_TIMEOUT = 500, HH_SENSOR_TIMEOUT = 200 };
enum { HH_FAULT_STALE = 1, HH_FAULT_SENSOR = 2, HH_FAULT_LIMIT = 4 };

typedef struct {
    uint16_t position, target;
    int16_t velocity, valve;
    uint8_t limits, sensor_fault, mode, speed, sequence, sensor_sequence, fault;
    uint8_t native_sequence;
    bool have_sensor, have_command, primed, enabled;
    uint32_t sensor_at, command_at;
} hh_controller;

uint8_t hh_crc(const uint8_t *data, unsigned length);
bool hh_valid(const uint8_t data[8]);
void hh_seal(uint8_t data[8]);
void hh_init(hh_controller *state);
void hh_stop(hh_controller *state);
bool hh_sensor(hh_controller *state, const uint8_t data[8], uint32_t now);
uint8_t hh_command(hh_controller *state, const uint8_t data[8], uint32_t now);
void hh_step(hh_controller *state, uint32_t now, bool network_ready);
void hh_output(hh_controller *state, uint8_t data[8]);
void hh_make_command(uint8_t mode, uint16_t target, uint8_t speed,
                     uint8_t sequence, bool enabled, uint8_t data[8]);
bool hh_next(uint8_t previous, uint8_t current);

#ifdef __cplusplus
}
#endif
#endif
