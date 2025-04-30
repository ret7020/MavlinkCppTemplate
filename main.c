#include <stdio.h>
#include "config.h"
#include "uart.h"
#include "mav_wrappers.h"

int main() {
    int fd = open_uart(UART_PORT_DEV, UART_BAUD);
    if (fd < 0) return -1;

    // Sending initial heartbeat
    send_heartbeat(fd);

    // Waiting for FC heartbeat
    if (wait_heartbeat(fd) < 0) { // warn; blocking
        printf("[MAIN] Heartbeat from FC receive error\n");
        return -1;
    }

    printf("[MAIN] Heartbeat from FC received\n");
    printf("[MAIN] Setting offboard mode\n");
    set_offboard_mode(fd);

    printf("[MAIN] Refresh loop started\n");

    while (1)
    {
        send_vpe(fd, 1, 1, 1, 1, 1, 1);
        send_position_target(fd, 1, 1, 1, 1);
    };

    close(fd);
    return 0;
}