#ifndef HAL_CAN_H
#define HAL_CAN_H

#include "can_protocol.h"

/*
 * CAN part of the HAL. hal_can_socketcan.c implements it with Linux
 * SocketCAN (vcan0 or a real CAN adapter); on an STM32 a hal_can_stm32.c
 * would use the bxCAN peripheral instead.
 */
int hal_can_init(const char *interface);        /* 0 = ok */
int hal_can_send(const can_frame_t *frame);     /* 0 = ok */
int hal_can_receive(can_frame_t *frame);        /* 1 = frame received, 0 = none (never blocks) */

#endif /* HAL_CAN_H */