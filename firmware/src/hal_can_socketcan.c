/* CAN HAL for Linux SocketCAN (vcan0 or a real USB-CAN adapter). */
#include "hal_can.h"

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

static int sock = -1;

int hal_can_init(const char *interface)
{
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("[can] socket");
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "[can] interface %s not found (run scripts/setup_vcan.sh)\n", interface);
        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[can] bind");
        return -1;
    }

    /* Non-blocking: the control loop must never wait for the bus */
    fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
    return 0;
}

int hal_can_send(const can_frame_t *frame)
{
    struct can_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id = frame->id;
    f.can_dlc = frame->len;
    memcpy(f.data, frame->data, frame->len);

    ssize_t n;
    do {
        n = write(sock, &f, sizeof(f));
    } while (n < 0 && errno == EINTR);   /* FreeRTOS tick signals can interrupt calls */
    return n == (ssize_t)sizeof(f) ? 0 : -1;
}

int hal_can_receive(can_frame_t *frame)
{
    struct can_frame f;
    ssize_t n;
    do {
        n = read(sock, &f, sizeof(f));
    } while (n < 0 && errno == EINTR);
    if (n != (ssize_t)sizeof(f)) return 0;   /* nothing waiting (EAGAIN) */

    frame->id = f.can_id & CAN_SFF_MASK;
    frame->len = f.can_dlc > 8 ? 8 : f.can_dlc;
    memcpy(frame->data, f.data, frame->len);
    return 1;
}