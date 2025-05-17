#pragma once

#include "config.h"
#include "mavlink/all/mavlink.h"
#include "uart.h"
#include <chrono>
#include <stdio.h>

long initial_msec = 0;
long time_boot_ms = 0;
long px4_time_boot_ms = 0;

void send_heartbeat(int uart_fd, int system_id = SYSTEM_ID, int component_id = COMPONENT_ID) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                               MAV_MODE_AUTO_ARMED, 0, MAV_STATE_ACTIVE);
    tx_mavlink(uart_fd, &msg);
}

int wait_heartbeat(int fd) {
    char buf[256];

    while (1) {
        int len = read(fd, buf, sizeof(buf));
        if (len > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        return 1;
                    }
                }
            }
        } else {
            perror("[MAVLINK] Len error");
            return -1;
        }
    }
}

mavlink_local_position_ned_t wait_local_position_ned(int fd, int upd_boot_time = false) {
    char buf[256];
    mavlink_local_position_ned_t pned;

    while (1) {
        int len = read(fd, buf, sizeof(buf));
        if (len > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                        mavlink_msg_local_position_ned_decode(&msg, &pned);
                        // printf("[MAVLINK] Local position ned RX\n");
                        if (upd_boot_time) {
                            px4_time_boot_ms = pned.time_boot_ms;
                        }
                        return pned;
                    }
                }
            }
        } else {
            perror("[MAVLINK] Len error");
            return pned;
        }
    }
}

void set_offboard_mode(int uart_fd, int system_id = SYSTEM_ID, int component_id = COMPONENT_ID) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(system_id,    // system_id
                                  component_id, // component_id
                                  &msg, 1, 1, MAV_CMD_DO_SET_MODE,
                                  0, // confirmation
                                  1, // custom mode for PX4
                                  6, // sub mode for PX4
                                  0, 0, 0, 0, 0);

    tx_mavlink(uart_fd, &msg);
}

void set_arm_disarm(int uart_fd, float mode, int system_id = SYSTEM_ID, int component_id = COMPONENT_ID) {
    // mode == 1 - arm
    // mode == 0 - disarm

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(system_id, component_id, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, mode, 0, 0, 0,
                                  0, 0, 0);

    tx_mavlink(uart_fd, &msg);
}

void send_vpe(int uart_fd, float x, float y, float z, float roll, float pitch, float yaw, int system_id = SYSTEM_ID,
              int component_id = COMPONENT_ID) {
    mavlink_message_t msg;
    mavlink_vision_position_estimate_t vision{};

    auto now = std::chrono::high_resolution_clock::now();
    auto now_msec = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    if (initial_msec == 0) {
        time_boot_ms = 0;
        initial_msec = now_msec;
    } else {
        time_boot_ms = now_msec - initial_msec + px4_time_boot_ms;
    }

    vision.usec = time_boot_ms;
    vision.x = x;
    vision.y = y;
    vision.z = z;
    vision.roll = roll;
    vision.pitch = pitch;
    vision.yaw = yaw;

    mavlink_msg_vision_position_estimate_encode(system_id, component_id, &msg, &vision);

    tx_mavlink(uart_fd, &msg);
}

void send_position_target(int uart_fd, float x, float y, float z, float yaw=0, int system_id = SYSTEM_ID,
                          int component_id = COMPONENT_ID) {
    auto now = std::chrono::high_resolution_clock::now();
    auto now_msec = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    if (initial_msec == 0) {
        time_boot_ms = 0;
        initial_msec = now_msec;
    } else {
        time_boot_ms = now_msec - initial_msec + px4_time_boot_ms;
    }

    mavlink_message_t msg;
    // yaw ignored
    mavlink_msg_set_position_target_local_ned_pack(system_id, component_id, &msg, time_boot_ms, 1, 1,
                                                   MAV_FRAME_LOCAL_NED, 3576, x, y, z, 0, 0, 0, 0, 0, 0,
                                                   yaw, 0);

    tx_mavlink(uart_fd, &msg);
}
