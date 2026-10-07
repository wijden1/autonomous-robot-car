/* Unit tests for the PID controller (no FreeRTOS, no hardware needed). */
#include <stdio.h>
#include <math.h>
#include "pid.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (cond) printf("  PASS  %s\n", msg); \
                              else { printf("  FAIL  %s\n", msg); failures++; } } while (0)

static void test_proportional_only(void)
{
    pid_t_ctrl pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    float out = pid_update(&pid, 10.0f, 4.0f, 0.01f);
    CHECK(fabsf(out - 12.0f) < 1e-4f, "P only: output = Kp * error");
}

static void test_output_is_clamped(void)
{
    pid_t_ctrl pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, -12.0f, 12.0f);
    CHECK(pid_update(&pid, 1000.0f, 0.0f, 0.01f) == 12.0f, "output clamped to max");
    CHECK(pid_update(&pid, -1000.0f, 0.0f, 0.01f) == -12.0f, "output clamped to min");
}

static void test_integral_accumulates(void)
{
    pid_t_ctrl pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    float out = 0.0f;
    for (int i = 0; i < 100; i++) out = pid_update(&pid, 1.0f, 0.0f, 0.01f);
    CHECK(fabsf(out - 1.0f) < 1e-3f, "I only: integral of error 1 over 1 s = 1");
}

static void test_anti_windup(void)
{
    pid_t_ctrl pid;
    pid_init(&pid, 0.0f, 10.0f, 0.0f, -1.0f, 1.0f);
    /* Long saturation: the integrator must not grow without limit */
    for (int i = 0; i < 1000; i++) pid_update(&pid, 100.0f, 0.0f, 0.01f);
    CHECK(pid.integral <= 1.0f + 1e-3f, "anti-windup: integral stays bounded while saturated");
    /* When the error changes sign, the output must react immediately */
    float out = pid_update(&pid, -100.0f, 0.0f, 0.01f);
    CHECK(out < 1.0f, "anti-windup: output leaves saturation right away");
}

static void test_reset(void)
{
    pid_t_ctrl pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    for (int i = 0; i < 50; i++) pid_update(&pid, 1.0f, 0.0f, 0.01f);
    pid_reset(&pid);
    CHECK(pid.integral == 0.0f, "reset clears the integrator");
}

int main(void)
{
    printf("PID tests\n");
    test_proportional_only();
    test_output_is_clamped();
    test_integral_accumulates();
    test_anti_windup();
    test_reset();
    printf("%s (%d failure(s))\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}