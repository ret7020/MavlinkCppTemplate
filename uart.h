#pragma once

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "mavlink/all/mavlink.h"

int open_uart(const char *port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("[UART] device open error: ");
        return -1;
    }

    // Configure
    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("[MAVLINK] Error from tcgetattr: ");
        close(fd);
        return -1;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // It just works with such flags
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("[UART] Setting params error: ");
        close(fd);
        return -1;
    }
    return fd;
}

void tx_mavlink(int uart_fd, mavlink_message_t *msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    write(uart_fd, buffer, len);
}

