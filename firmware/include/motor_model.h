#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

/*
 * Simple brushed DC motor model.
 *   current:  i = (V - Ke * w) / R            (inductance neglected)
 *   torque:   T_motor = Kt * i
 *   motion:   J * dw/dt = T_motor - b * w - T_load
 */
typedef struct {
    float R;        /* winding resistance [Ohm] */
    float Kt;       /* torque constant [Nm/A] */
    float Ke;       /* back-EMF constant [V s/rad] */
    float J;        /* rotor inertia [kg m^2] */
    float b;        /* viscous friction [Nm s/rad] */
    float omega;    /* speed [rad/s] (state) */
    float angle;    /* shaft angle [rad] (state) */
} motor_model_t;

void  motor_model_init(motor_model_t *m);
void  motor_model_step(motor_model_t *m, float volts, float load_nm, float dt_s);
float motor_model_rpm(const motor_model_t *m);

#endif /* MOTOR_MODEL_H */