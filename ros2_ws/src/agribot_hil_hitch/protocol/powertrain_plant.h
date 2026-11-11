#ifndef AGRIBOT_HIL_POWERTRAIN_PLANT_H
#define AGRIBOT_HIL_POWERTRAIN_PLANT_H
#include "powertrain.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float engine_omega, ratio, pressure_a, pressure_b, throttle;
    float wheel_torque, input_torque, input_rpm, output_rpm, speed, load_force;
    unsigned gear, mode;
    bool braking, enabled;
} pt_plant;
void pp_init(pt_plant *s);
void pp_step(pt_plant *s, const pt_actuation *act, float wheel_omega, float speed,
             float load_force, float dt);
void pp_sensor(const pt_plant *s, uint8_t sequence, uint8_t data[4][8]);
#ifdef __cplusplus
}
#endif
#endif
