#include "motor_model.h"

#define PI_F 3.14159265f

void motor_model_init(motor_model_t *m)
{
    /* Roughly a small 12 V hobby motor */
    m->R  = 2.0f;
    m->Kt = 0.02f;
    m->Ke = 0.02f;
    m->J  = 2.0e-5f;
    m->b  = 1.0e-5f;
    m->omega = 0.0f;
    m->angle = 0.0f;
}

void motor_model_step(motor_model_t *m, float volts, float load_nm, float dt_s)
{
    float current = (volts - m->Ke * m->omega) / m->R;
    float torque = m->Kt * current;

    /* Load torque always opposes motion (like friction from a load) */
    float load = 0.0f;
    if (m->omega > 0.0f) load = load_nm;
    else if (m->omega < 0.0f) load = -load_nm;

    float alpha = (torque - m->b * m->omega - load) / m->J;
    m->omega += alpha * dt_s;   /* explicit Euler integration */
    m->angle += m->omega * dt_s;
}

float motor_model_rpm(const motor_model_t *m)
{
    return m->omega * 60.0f / (2.0f * PI_F);
}