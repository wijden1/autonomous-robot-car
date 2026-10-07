#ifndef HAL_H
#define HAL_H

#include <stdint.h>

/*
 * Hardware Abstraction Layer (HAL)
 *
 * The control code only talks to the hardware through these functions.
 * hal_sim.c implements them with a simulated DC motor; a later hal_stm32.c
 * can implement them with real PWM, encoder timer and CAN peripherals,
 * without changing the control code.
 */

void     hal_init(void);
void     hal_motor_set_voltage(float volts);   /* clamped to +/- HAL_SUPPLY_VOLTAGE */
int32_t  hal_encoder_read_ticks(void);         /* free-running tick counter */
void     hal_set_load_torque(float nm);        /* simulation only: disturbance */
void     hal_sim_step(float dt_s);             /* simulation only: advance the physics */

#define HAL_SUPPLY_VOLTAGE   12.0f
#define HAL_TICKS_PER_REV    1024

#endif /* HAL_H */