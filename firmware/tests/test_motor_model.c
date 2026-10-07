/* Unit tests for the DC motor model. */
#include <stdio.h>
#include <math.h>
#include "motor_model.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (cond) printf("  PASS  %s\n", msg); \
                              else { printf("  FAIL  %s\n", msg); failures++; } } while (0)

static float run(float volts, float load, float seconds)
{
    motor_model_t m;
    motor_model_init(&m);
    for (int i = 0; i < (int)(seconds * 1000); i++) motor_model_step(&m, volts, load, 0.001f);
    return m.omega;
}

int main(void)
{
    printf("Motor model tests\n");
    CHECK(fabsf(run(0.0f, 0.0f, 1.0f)) < 1e-6f, "no voltage -> no motion");

    float w6 = run(6.0f, 0.0f, 2.0f);
    float w12 = run(12.0f, 0.0f, 2.0f);
    CHECK(w6 > 0.0f, "positive voltage -> positive speed");
    CHECK(fabsf(w12 / w6 - 2.0f) < 0.01f, "steady-state speed is proportional to voltage");
    CHECK(run(-6.0f, 0.0f, 2.0f) < 0.0f, "negative voltage -> reverse");
    CHECK(run(6.0f, 0.01f, 2.0f) < w6, "load torque reduces speed");

    printf("%s (%d failure(s))\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}