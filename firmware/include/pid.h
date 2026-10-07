#ifndef PID_H
#define PID_H

/* PID controller with output limits and anti-windup. */
typedef struct {
    float kp, ki, kd;        /* gains */
    float out_min, out_max;  /* output limits */
    float integral;          /* integrator state */
    float prev_error;        /* for the D term */
    int   first_run;
} pid_t_ctrl;

void  pid_init(pid_t_ctrl *pid, float kp, float ki, float kd, float out_min, float out_max);
void  pid_reset(pid_t_ctrl *pid);
float pid_update(pid_t_ctrl *pid, float setpoint, float measurement, float dt_s);

#endif /* PID_H */