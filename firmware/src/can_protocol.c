#include "can_protocol.h"

/* Little-endian helpers: byte 0 = low byte, byte 1 = high byte */
static void put_i16(uint8_t *buf, int16_t value)
{
    uint16_t u = (uint16_t)value;
    buf[0] = (uint8_t)(u & 0xFF);
    buf[1] = (uint8_t)(u >> 8);
}

static int16_t get_i16(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

void can_pack_wheel_cmd(const wheel_cmd_t *msg, can_frame_t *frame)
{
    frame->id = CAN_ID_WHEEL_CMD;
    frame->len = 5;
    put_i16(&frame->data[0], msg->setpoint_left_rpm);
    put_i16(&frame->data[2], msg->setpoint_right_rpm);
    frame->data[4] = msg->alive_counter;
}

int can_unpack_wheel_cmd(const can_frame_t *frame, wheel_cmd_t *msg)
{
    if (frame->id != CAN_ID_WHEEL_CMD || frame->len < 5) return 0;
    msg->setpoint_left_rpm = get_i16(&frame->data[0]);
    msg->setpoint_right_rpm = get_i16(&frame->data[2]);
    msg->alive_counter = frame->data[4];
    return 1;
}

void can_pack_motor_status(uint32_t id, const motor_status_t *msg, can_frame_t *frame)
{
    frame->id = id;
    frame->len = 6;
    put_i16(&frame->data[0], msg->speed_rpm);
    float raw = msg->voltage * 100.0f;                 /* factor 0.01 */
    put_i16(&frame->data[2], (int16_t)(raw >= 0 ? raw + 0.5f : raw - 0.5f));
    frame->data[4] = msg->state;
    frame->data[5] = msg->alive_counter;
}

int can_unpack_motor_status(const can_frame_t *frame, motor_status_t *msg)
{
    if (frame->len < 6) return 0;
    msg->speed_rpm = get_i16(&frame->data[0]);
    msg->voltage = get_i16(&frame->data[2]) / 100.0f;
    msg->state = frame->data[4];
    msg->alive_counter = frame->data[5];
    return 1;
}