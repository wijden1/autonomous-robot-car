/*
 * Simulated hardware: a DC motor model with an encoder.
 * Shared state is protected with FreeRTOS critical sections, because the
 * plant task and the control task access it from different tasks.
 */
#include "hal.h"
#include "motor_model.h"
#include "FreeRTOS.h"
#include "task.h"

#define PI_F 3.14159265f

static motor_model_t motor;
static float applied_voltage = 0.0f;
static float load_torque = 0.0f;

void hal_init(void)
{
    motor_model_init(&motor);
}

void hal_motor_set_voltage(float volts)
{
    if (volts > HAL_SUPPLY_VOLTAGE) volts = HAL_SUPPLY_VOLTAGE;
    if (volts < -HAL_SUPPLY_VOLTAGE) volts = -HAL_SUPPLY_VOLTAGE;
    taskENTER_CRITICAL();
    applied_voltage = volts;
    taskEXIT_CRITICAL();
}

int32_t hal_encoder_read_ticks(void)
{
    taskENTER_CRITICAL();
    float angle = motor.angle;
    taskEXIT_CRITICAL();
    /* The encoder only sees whole ticks -> realistic quantisation */
    return (int32_t)(angle / (2.0f * PI_F) * HAL_TICKS_PER_REV);
}

void hal_set_load_torque(float nm)
{
    taskENTER_CRITICAL();
    load_torque = nm;
    taskEXIT_CRITICAL();
}

void hal_sim_step(float dt_s)
{
    taskENTER_CRITICAL();
    motor_model_step(&motor, applied_voltage, load_torque, dt_s);
    taskEXIT_CRITICAL();
}