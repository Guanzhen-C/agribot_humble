#include "powertrain_plant.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned now, sequence;
static float speed;
static pt_controller control;
static pt_plant plant;
static void protocol_guards(void)
{
    pt_controller p; pt_init(&p);
    uint8_t profile[8], settings[8] = {PT_VERSION, 60, 0xdc, 5, 0, 0, 255, 0};
    hh_seal(settings);
    assert(pt_settings(&p, settings, 10) == HH_OK);
    assert(fabsf(p.shift_seconds-.6f) < .001f && p.ratio_rate == 150.f);
    assert(pt_settings(&p, settings, 11) == HH_REPLAY);
    settings[6] = 0; settings[1] = 20; hh_seal(settings);
    assert(pt_settings(&p, settings, 12) == HH_OK);
    settings[6] = 1; settings[1] = 200; hh_seal(settings);
    assert(pt_settings(&p, settings, 13) == HH_OK);
    settings[6] = 2; settings[1] = 19; hh_seal(settings);
    assert(pt_settings(&p, settings, 14) == HH_RANGE);
    settings[1] = 60; settings[4] = 1; hh_seal(settings);
    assert(pt_settings(&p, settings, 15) == HH_FORMAT);
    settings[4] = 0; hh_seal(settings); p.measured.speed = .2f;
    assert(pt_settings(&p, settings, 16) == PT_MOVING_MODE_CHANGE);
    p.measured.speed = 0;
    pt_make_profile(PT_OFF, 0, 1600, 0, 255, profile);
    assert(pt_profile(&p, profile, 20) == HH_OK);
    assert(pt_profile(&p, profile, 21) == HH_REPLAY);
    pt_make_profile(PT_CVT_MANUAL, 0, 1600, 450, 0, profile);
    assert(pt_profile(&p, profile, 22) == HH_OK);
    pt_make_profile(PT_CVT_MANUAL, 0, 1600, 199, 1, profile);
    assert(pt_profile(&p, profile, 23) == HH_RANGE);
    pt_plant model; pp_init(&model); uint8_t data[4][8]; pp_sensor(&model, 1, data);
    assert(!pt_sensor(&p, PT_ENGINE_SENSOR, data[0], 30));
    assert(!pt_sensor(&p, PT_TRANS_SENSOR, data[1], 30));
    assert(!p.have_sensor);
    data[2][6] = 2; hh_seal(data[2]);
    assert(!pt_sensor(&p, PT_VEHICLE_SENSOR, data[2], 30));
    data[2][6] = 1; hh_seal(data[2]);
    assert(pt_sensor(&p, PT_VEHICLE_SENSOR, data[2], 31));
    assert(!pt_sensor(&p, PT_VEHICLE_SENSOR, data[2], 32));
    pp_sensor(&model, 2, data);
    assert(!pt_sensor(&p, PT_ENGINE_SENSOR, data[0], 40));
    assert(!pt_sensor(&p, PT_TRANS_SENSOR, data[1], 40));
    assert(!pt_sensor(&p, PT_VEHICLE_SENSOR, data[2], 141));
    assert(p.sensor_sequence == 1);
}
static void detached_clutch_residual_pressure(void)
{
    pt_plant model; pp_init(&model);
    model.pressure_a = 1000;
    pt_actuation act = {0}; act.mode = PT_POWERSHIFT_AUTO; act.enabled = true;
    act.brake = true; act.gear_b = 2; act.ratio = 480;
    pp_step(&model, &act, 0, 0, 0, .001f);
    assert(model.ratio == 640 && model.input_rpm == 0);
    model.pressure_b = 1000; act.brake = false; act.clutch_b = 1;
    pp_step(&model, &act, 1, 0, 0, .001f);
    assert(fabsf(model.ratio-480) < .01f);
    assert(fabsf(model.input_rpm-480*9.5492966f) < .01f);
}
static void run(unsigned mode, unsigned gear, float ratio, float target, float load, unsigned milliseconds)
{
    for (unsigned t = 0; t < milliseconds; ++t, ++now) {
        if (now%20 == 0) {
            uint8_t sensors[4][8], drive[8] = {0}, profile[8];
            pp_sensor(&plant, ++sequence, sensors);
            for (unsigned i = 0; i < 3; ++i) {
                bool accepted = pt_sensor(&control, PT_ENGINE_SENSOR+i, sensors[i], now);
                assert(i < 2 || accepted);
            }
            unsigned v = lroundf(target*1000);
            drive[0] = v & 255; drive[1] = v>>8; drive[4] = target > 0;
            drive[5] = HH_VERSION; drive[6] = sequence; hh_seal(drive); assert(pt_drive(&control, drive, now));
            pt_make_profile(mode, gear, 1600, ratio, sequence, profile);
            unsigned result = pt_profile(&control, profile, now);
            assert(result == HH_OK);
            pt_step(&control, now, true);
        }
        pp_step(&plant, &control.output, speed/.716006673f, speed, load, .001f);
        float resistance = speed > .001f ? 180.f+load : 0;
        float force = plant.wheel_torque/.716006673f-resistance;
        if (plant.braking) { force -= speed*12000; }
        speed = fmaxf(0, speed+.001f*force/2600.f);
        assert(isfinite(speed) && speed < 1);
        assert(plant.ratio >= 200 && plant.ratio <= 900);
    }
}
int main(void)
{
    protocol_guards();
    detached_clutch_residual_pressure();
    pt_init(&control); pp_init(&plant);
    run(PT_OFF, 0, 0, 0, 0, 500);
    run(PT_POWERSHIFT_AUTO, 0, 0, .45f, 0, 20000);
    printf("powershift speed=%f rpm=%f gear=%u ratio=%f\n", speed, control.measured.engine_rpm, control.gear, plant.ratio);
    assert(control.gear == 4 && speed > .25f && speed < .6f);
    uint8_t profile[8]; pt_make_profile(PT_CVT_AUTO, 0, 1600, 0, ++sequence, profile);
    assert(pt_profile(&control, profile, now) == PT_MOVING_MODE_CHANGE);
    run(PT_POWERSHIFT_AUTO, 0, 0, 0, 0, 6000);
    assert(speed < .02f);
    run(PT_OFF, 0, 0, 0, 0, 500);
    run(PT_CVT_AUTO, 0, 0, .45f, 0, 25000);
    printf("cvt speed=%f rpm=%f ratio=%f\n", speed, control.measured.engine_rpm, plant.ratio);
    assert(speed > .3f && speed < .6f && plant.ratio < 400);
    run(PT_CVT_AUTO, 0, 0, .45f, 600, 15000);
    printf("cvt loaded speed=%f rpm=%f ratio=%f\n", speed, control.measured.engine_rpm, plant.ratio);
    assert(speed > .25f && speed < .6f);
    pt_step(&control, now+600, true);
    assert(control.output.brake && !control.output.enabled && control.fault == PT_STALE);
    pt_make_profile(PT_CVT_AUTO, 0, 1600, 0, ++sequence, profile);
    assert(pt_profile(&control, profile, now+600) == HH_UNPRIMED);
    profile[7] ^= 1; assert(pt_profile(&control, profile, now+600) == HH_FORMAT);
    puts("powertrain nominal closed-loop + moving mode guard + watchdog + CRC PASS");
}
