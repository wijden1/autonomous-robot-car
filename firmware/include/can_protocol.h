#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

/*
 * CAN messages of the robot car (must match can/robot_car.dbc).
 * All signals are little-endian.
 */
#define CAN_ID_WHEEL_CMD          0x100   /* ROS 2 -> motors, 5 bytes */
#define CAN_ID_MOTOR_STATUS_LEFT  0x201   /* left motor  -> ROS 2, 6 bytes */
#define CAN_ID_MOTOR_STATUS_RIGHT 0x202   /* right motor -> ROS 2, 6 bytes */

#define MOTOR_STATE_OK        0
#define MOTOR_STATE_WATCHDOG  1

typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} can_frame_t;

typedef struct {
    int16_t setpoint_left_rpm;
    int16_t setpoint_right_rpm;
    uint8_t alive_counter;
} wheel_cmd_t;

typedef struct {
    int16_t speed_rpm;
    float   voltage;        /* sent with factor 0.01 */
    uint8_t state;
    uint8_t alive_counter;
} motor_status_t;

void can_pack_wheel_cmd(const wheel_cmd_t *msg, can_frame_t *frame);
int  can_unpack_wheel_cmd(const can_frame_t *frame, wheel_cmd_t *msg);   /* 1 = ok */

void can_pack_motor_status(uint32_t id, const motor_status_t *msg, can_frame_t *frame);
int  can_unpack_motor_status(const can_frame_t *frame, motor_status_t *msg);

#endif /* CAN_PROTOCOL_H */