#include "powertrain_plant.h"
#include <math.h>
#include <string.h>
static float clip(float x, float a, float b) { return fminf(b, fmaxf(a, x)); }
static void put(uint8_t *p, int value) { p[0] = value & 255; p[1] = (unsigned)value>>8; }
void pp_init(pt_plant *s)
{ memset(s, 0, sizeof(*s)); s->engine_omega = 780.f*.104719755f; s->ratio = 640; s->braking = true; }
void pp_step(pt_plant *s, const pt_actuation *act, float wheel_omega, float speed, float load, float dt)
{
    dt = clip(dt, 0, .01f);
    float a = act->enabled && !act->brake ? act->clutch_a : 0;
    float b = act->enabled && !act->brake ? act->clutch_b : 0;
    s->pressure_a += (2000*a-s->pressure_a)*dt/.12f;
    s->pressure_b += (2000*b-s->pressure_b)*dt/.12f;
    s->throttle += ((act->enabled ? act->throttle : 0)-s->throttle)*dt/.08f;
    if (act->mode >= PT_CVT_AUTO) {
        s->ratio += clip(act->ratio-s->ratio, -dt*220, dt*220);
    }
    float ra = act->mode >= PT_CVT_AUTO ? s->ratio : pt_gear_ratio(act->gear_a);
    float rb = act->mode >= PT_CVT_AUTO ? 0 : pt_gear_ratio(act->gear_b);
    float wa = wheel_omega*ra, wb = wheel_omega*rb;
    float ta = ra ? s->pressure_a*.060f*tanhf((s->engine_omega-wa)*.05f) : 0;
    float tb = rb ? s->pressure_b*.060f*tanhf((s->engine_omega-wb)*.05f) : 0;
    float torque = .86f*(ta*ra+tb*rb);
    // Nominal actuator torque limit; scale the engine reaction consistently.
    float scale = fabsf(torque) > 1750 ? 1750/fabsf(torque) : 1;
    torque *= scale; ta *= scale; tb *= scale;
    if (!act->enabled || act->brake) { torque = ta = tb = 0; }
    float engine_torque = s->throttle*70.f*clip((285.f-s->engine_omega)/70.f, 0, 1);
    float idle = clip((780.f*.104719755f-s->engine_omega)*2, 0, 25);
    float friction = 5.f+s->engine_omega*.025f;
    s->engine_omega = clip(s->engine_omega+dt*(engine_torque+idle-friction-ta-tb)/3.f, 0, 290);
    s->wheel_torque = torque; s->input_torque = ta+tb;
    float engaged_a = ra ? s->pressure_a : 0;
    float engaged_b = rb ? s->pressure_b : 0;
    float sum = engaged_a+engaged_b;
    if (act->mode > PT_OFF && act->mode < PT_CVT_AUTO && sum > 10) {
        s->ratio = (ra*engaged_a+rb*engaged_b)/sum;
    }
    s->input_rpm = sum > 10 ? (wa*engaged_a+wb*engaged_b)/sum*9.5492966f : 0;
    s->output_rpm = wheel_omega*9.5492966f; s->speed = speed; s->load_force = load;
    s->gear = act->mode >= PT_CVT_AUTO ? 0 : s->pressure_a >= s->pressure_b ? act->gear_a : act->gear_b;
    if (sum < 50) { s->gear = 0; }
    s->mode = act->mode; s->braking = act->brake || !act->enabled; s->enabled = act->enabled;
}
void pp_sensor(const pt_plant *s, uint8_t sequence, uint8_t data[4][8])
{
    memset(data, 0, 32);
    put(data[0], lroundf(s->engine_omega*9.5492966f)); put(data[0]+2, lroundf(fabsf(s->input_rpm)));
    put(data[0]+4, lroundf(s->input_torque*10));
    put(data[1], lroundf(s->ratio*10)); put(data[1]+2, lroundf(s->pressure_a));
    put(data[1]+4, lroundf(s->pressure_b));
    put(data[2], lroundf(s->speed*1000)); put(data[2]+2, lroundf(s->output_rpm*10));
    data[2][4] = s->gear | s->mode<<4;
    put(data[3], lroundf(s->load_force)); data[3][2] = (uint8_t)clip(fabsf(s->input_torque)/70.f*100, 0, 100);
    data[3][3] = s->braking; data[3][4] = s->enabled; data[3][5] = PT_VERSION;
    for (unsigned i = 0; i < 4; ++i) { data[i][6] = sequence; hh_seal(data[i]); }
}
