/* Unit tests for CAN packing/unpacking (must match can/robot_car.dbc). */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "can_protocol.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (cond) printf("  PASS  %s\n", msg); \
                              else { printf("  FAIL  %s\n", msg); failures++; } } while (0)

int main(void)
{
    printf("CAN protocol tests\n");

    /* Same values as can/can_test.py -> must give the same bytes as cantools */
    wheel_cmd_t cmd = { 1000, -250, 7 };
    can_frame_t f;
    can_pack_wheel_cmd(&cmd, &f);
    const uint8_t expected[5] = { 0xE8, 0x03, 0x06, 0xFF, 0x07 };
    CHECK(f.id == 0x100 && f.len == 5, "WHEEL_CMD has ID 0x100 and 5 bytes");
    CHECK(memcmp(f.data, expected, 5) == 0, "WHEEL_CMD bytes match the DBC (E8 03 06 FF 07)");

    wheel_cmd_t back;
    CHECK(can_unpack_wheel_cmd(&f, &back) && back.setpoint_left_rpm == 1000 &&
          back.setpoint_right_rpm == -250 && back.alive_counter == 7, "WHEEL_CMD round trip");

    f.id = 0x123;
    CHECK(!can_unpack_wheel_cmd(&f, &back), "frames with another ID are rejected");

    motor_status_t st = { -1234, 3.22f, MOTOR_STATE_WATCHDOG, 200 }, st_back;
    can_pack_motor_status(CAN_ID_MOTOR_STATUS_LEFT, &st, &f);
    CHECK(f.id == 0x201 && f.len == 6, "MOTOR_STATUS_LEFT has ID 0x201 and 6 bytes");
    CHECK(f.data[2] == 0x42 && f.data[3] == 0x01, "voltage 3.22 V is sent as raw 322 (factor 0.01)");
    CHECK(can_unpack_motor_status(&f, &st_back) && st_back.speed_rpm == -1234 &&
          fabsf(st_back.voltage - 3.22f) < 0.006f && st_back.state == MOTOR_STATE_WATCHDOG &&
          st_back.alive_counter == 200, "MOTOR_STATUS round trip");

    printf("%s (%d failure(s))\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}