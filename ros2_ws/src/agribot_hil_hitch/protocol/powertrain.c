#include "powertrain.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
static float clip(float x, float a, float b) { return fminf(b, fmaxf(a, x)); }
static float approach(float x, float target, float amount)
{ return x + clip(target-x, -amount, amount); }
static unsigned u16(const uint8_t *p) { return p[0] | (unsigned)p[1]<<8; }
static int i16(const uint8_t *p) { return (int16_t)u16(p); }
static void put(uint8_t *p, int value) { p[0] = value & 255; p[1] = (unsigned)value>>8; }
float pt_gear_ratio(unsigned gear)
{ const float ratios[] = {0.f, 640.f, 480.f, 350.f, 260.f}; return gear <= 4 ? ratios[gear] : 0.f; }
void pt_init(pt_controller *p)
{
    memset(p, 0, sizeof(*p)); p->requested_rpm = 1600.f; p->ratio_rate = 150.f;
    p->shift_seconds = .6f; p->output.ratio = 640.f; p->output.brake = true;
}
void pt_make_profile(unsigned mode, unsigned gear, unsigned rpm, float ratio, uint8_t sequence, uint8_t data[8])
{
    memset(data, 0, 8); data[0] = PT_VERSION; data[1] = mode; data[2] = gear;
    data[3] = rpm/20; put(data+4, (int)lroundf(ratio*10)); data[6] = sequence; hh_seal(data);
}
uint8_t pt_profile(pt_controller *p, const uint8_t data[8], uint32_t now)
{
    if (!hh_valid(data) || data[0] != PT_VERSION) { return HH_FORMAT; }
    unsigned mode = data[1], gear = data[2], rpm = data[3]*20;
    float ratio = u16(data+4)*.1f;
    if (mode > PT_CVT_MANUAL || gear > 4 ||
        (mode && (rpm < 800 || rpm > 2200)) ||
        (mode == PT_POWERSHIFT_MANUAL && gear == 0) ||
        (mode != PT_POWERSHIFT_MANUAL && gear != 0) ||
        (mode == PT_CVT_MANUAL ? ratio < 200 || ratio > 900 : ratio != 0)) { return HH_RANGE; }
    if (p->have_profile && now-p->profile_at < 500 && !hh_next(p->profile_sequence, data[6])) { return HH_REPLAY; }
    if (mode && !p->primed) { return HH_UNPRIMED; }
    if (mode && mode != p->mode && fabsf(p->measured.speed) > .03f) { return PT_MOVING_MODE_CHANGE; }
    if (mode != p->mode) {
        p->gear = p->next_gear = 0; p->shift_state = PT_STEADY;
        p->rpm_integral = p->ratio_integral = p->speed_reference = 0;
    }
    p->primed = true; p->have_profile = true; p->profile_at = now;
    p->profile_sequence = data[6]; p->mode = mode;
    p->requested_gear = gear; p->requested_rpm = rpm; p->requested_ratio = ratio;
    return HH_OK;
}
uint8_t pt_settings(pt_controller *p, const uint8_t data[8], uint32_t now)
{
    if (!hh_valid(data) || data[0] != PT_VERSION || data[4] || data[5]) { return HH_FORMAT; }
    float seconds = data[1]*.01f, rate = u16(data+2)*.1f;
    if (data[1] < 20 || data[1] > 200 || rate < 10 || rate > 300) { return HH_RANGE; }
    if (p->have_config && now-p->config_at < 1000 && !hh_next(p->config_sequence, data[6])) { return HH_REPLAY; }
    if (fabsf(p->measured.speed) > .03f &&
        (fabsf(seconds-p->shift_seconds) > .005f || fabsf(rate-p->ratio_rate) > .05f)) { return PT_MOVING_MODE_CHANGE; }
    p->shift_seconds = seconds; p->ratio_rate = rate;
    p->config_sequence = data[6]; p->config_at = now; p->have_config = true; return HH_OK;
}
bool pt_drive(pt_controller *p, const uint8_t data[8], uint32_t now)
{
    if (!hh_valid(data) || data[5] != HH_VERSION || data[4] > 1 ||
        i16(data) < 0 || i16(data) > 500 || abs(i16(data+2)) > 6100 ||
        (!data[4] && (i16(data) || i16(data+2)))) { return false; }
    if (p->have_drive && now-p->drive_at < 250 && !hh_next(p->drive_sequence, data[6])) { return false; }
    p->have_drive = true; p->drive_at = now; p->drive_sequence = data[6];
    p->drive_enabled = data[4]; p->demanded_speed = i16(data)*.001f; return true;
}
bool pt_sensor(pt_controller *p, uint16_t id, const uint8_t data[8], uint32_t now)
{
    if (!hh_valid(data) || id < PT_ENGINE_SENSOR || id > PT_VEHICLE_SENSOR) { return false; }
    unsigned index = id-PT_ENGINE_SENSOR;
    memcpy(p->pending_sensor[index], data, 8); p->pending_at[index] = now;
    for (unsigned i = 0; i < 3; ++i) {
        if (p->pending_sensor[i][6] != data[6] || now-p->pending_at[i] > 100) { return false; }
    }
    if (p->have_sensor && now-p->sensor_at < 200 && !hh_next(p->sensor_sequence, data[6])) { return false; }
    const uint8_t *a = p->pending_sensor[0], *b = p->pending_sensor[1], *c = p->pending_sensor[2];
    pt_measurement m = {0};
    m.engine_rpm = u16(a); m.input_rpm = u16(a+2); m.input_torque = i16(a+4)*.1f;
    m.ratio = u16(b)*.1f; m.pressure_a = u16(b+2); m.pressure_b = u16(b+4);
    m.speed = i16(c)*.001f; m.output_rpm = i16(c+2)*.1f; m.gear = c[4]&15;
    m.mode = c[4]>>4; m.fault = c[5];
    if (m.engine_rpm > 3000 || m.input_rpm > 4000 || m.ratio < 150 || m.ratio > 950 ||
        m.pressure_a > 2100 || m.pressure_b > 2100 || fabsf(m.speed) > 2 || m.gear > 4 || m.mode > 4) { return false; }
    p->measured = m; p->sensor_at = now; p->sensor_sequence = data[6]; p->have_sensor = true;
    return true;
}
void pt_step(pt_controller *p, uint32_t now, bool network)
{
    float dt = p->have_step ? clip((now-p->step_at)*.001f, 0, .1f) : .02f;
    p->have_step = true; p->step_at = now;
    pt_actuation out = {0}; out.ratio = p->output.ratio; out.brake = true;
    p->fault = 0;
    if (!network || !p->have_sensor || now-p->sensor_at >= 200 ||
        !p->have_profile || now-p->profile_at >= 500 || !p->have_drive || now-p->drive_at >= 250) { p->fault |= PT_STALE; }
    if (p->have_sensor && p->measured.fault) { p->fault |= PT_SENSOR_FAULT; }
    if (p->have_sensor && fabsf(p->measured.speed) > .65f) { p->fault |= PT_OVERSPEED; }
    if (p->fault) { p->primed = false; p->mode = PT_OFF; }
    if (!p->mode || p->fault) { p->output = out; return; }
    out.mode = p->mode; out.enabled = true;
    const float demand = p->drive_enabled ? p->demanded_speed : 0;
    p->speed_reference = approach(p->speed_reference, demand, dt*.4f);
    const bool moving = demand > .005f;
    out.brake = !moving || p->measured.speed > demand+.05f;
    unsigned wanted = p->requested_gear;
    if (p->mode == PT_POWERSHIFT_AUTO) {
        wanted = p->speed_reference > .40f ? 4 : p->speed_reference > .28f ? 3 : p->speed_reference > .17f ? 2 : 1;
    }
    if (p->mode <= PT_POWERSHIFT_AUTO) {
        if (!p->gear) { p->gear = 1; }
        if (wanted != p->gear && p->shift_state == PT_STEADY && now-p->shift_at > 700) {
            p->next_gear = wanted > p->gear ? p->gear+1 : p->gear-1;
            p->shift_state = PT_FILL; p->shift_at = now;
        }
        float phase = clip((now-p->shift_at)*.001f/p->shift_seconds, 0, 1);
        if (p->shift_state != PT_STEADY && phase >= 1) {
            p->gear = p->next_gear; p->next_gear = 0; p->shift_state = PT_STEADY;
        }
        if (p->shift_state != PT_STEADY) {
            p->shift_state = phase < .25f ? PT_FILL : PT_HANDOVER;
            float blend = clip((phase-.15f)/.85f, 0, 1);
            if (p->gear&1) {
                out.gear_a = p->gear; out.gear_b = p->next_gear;
                out.clutch_a = 1-blend; out.clutch_b = blend;
            } else {
                out.gear_b = p->gear; out.gear_a = p->next_gear;
                out.clutch_b = 1-blend; out.clutch_a = blend;
            }
        } else if (p->gear&1) { out.gear_a = p->gear; out.clutch_a = 1; }
        else { out.gear_b = p->gear; out.clutch_b = 1; }
        out.ratio = pt_gear_ratio(p->gear);
    } else {
        p->gear = 0; p->next_gear = 0; p->shift_state = PT_STEADY;
        const float circumference = 4.4988f;
        float target = p->requested_ratio;
        if (p->mode == PT_CVT_AUTO) {
            target = p->requested_rpm*circumference/(60*fmaxf(p->speed_reference, .02f));
            p->ratio_integral = clip(p->ratio_integral+(p->measured.speed-p->speed_reference)*dt*35, -100, 100);
            target += p->ratio_integral;
        }
        out.ratio = approach(p->output.ratio, clip(target, 200, 900), dt*p->ratio_rate);
        out.gear_a = 1; out.clutch_a = 1;
    }
    float rpm_target = p->requested_rpm;
    if (p->mode <= PT_POWERSHIFT_AUTO) {
        rpm_target = clip(p->speed_reference*out.ratio*60/4.4988f, 800, p->requested_rpm);
    }
    float error = rpm_target-p->measured.engine_rpm;
    p->rpm_integral = clip(p->rpm_integral+error*dt*.00025f, -.15f, .8f);
    out.throttle = clip(.16f+error*.0007f+p->rpm_integral, 0, 1);
    if (!moving) { out.clutch_a = out.clutch_b = 0; }
    if (out.brake && moving) { out.throttle = 0; out.clutch_a = out.clutch_b = 0; }
    p->output = out;
}
void pt_output(pt_controller *p, uint8_t a[8], uint8_t b[8])
{
    memset(a, 0, 8); memset(b, 0, 8);
    put(a, lroundf(p->output.throttle*1000)); put(a+2, lroundf(p->output.clutch_a*1000));
    a[4] = p->output.mode | (p->output.enabled ? 8 : 0) | (p->output.brake ? 16 : 0);
    put(b, lroundf(p->output.ratio*10)); put(b+2, lroundf(p->output.clutch_b*1000));
    b[4] = p->output.gear_a | p->output.gear_b<<4;
    a[5] = b[5] = PT_VERSION; a[6] = b[6] = ++p->output_sequence;
    hh_seal(a); hh_seal(b);
}
void pt_status(const pt_controller *p, uint8_t data[8])
{
    uint8_t out[8] = {PT_VERSION, p->mode, p->gear, p->requested_gear, p->shift_state,
                      p->fault, p->output_sequence, 0}; memcpy(data, out, 8); hh_seal(data);
}
