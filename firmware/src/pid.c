#include "pid.h"

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

void pid_init(pid_t_ctrl *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid_reset(pid);
}

void pid_reset(pid_t_ctrl *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->first_run = 1;
}

float pid_update(pid_t_ctrl *pid, float setpoint, float measurement, float dt_s)
{
    float error = setpoint - measurement;

    /* D term on the error; skipped on the first call to avoid a spike */
    float derivative = 0.0f;
    if (!pid->first_run && dt_s > 0.0f) {
        derivative = (error - pid->prev_error) / dt_s;
    }
    pid->first_run = 0;
    pid->prev_error = error;

    float p = pid->kp * error;
    float d = pid->kd * derivative;
    float candidate_integral = pid->integral + pid->ki * error * dt_s;
    float out = p + candidate_integral + d;

    /* Anti-windup (conditional integration): only keep the new integral
     * if the output is not saturated, or if the error drives it back. */
    if (out > pid->out_max) {
        if (error < 0.0f) pid->integral = candidate_integral;
    } else if (out < pid->out_min) {
        if (error > 0.0f) pid->integral = candidate_integral;
    } else {
        pid->integral = candidate_integral;
    }

    return clampf(p + pid->integral + d, pid->out_min, pid->out_max);
}