#pragma once

#include "mavlink/all/mavlink.h"
#include "config.h"
#include "uart.h"

void send_heartbeat(int uart_fd, int system_id=SYSTEM_ID, int component_id=COMPONENT_ID) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        system_id,
        component_id, 
        &msg, 
        MAV_TYPE_QUADROTOR, 
        MAV_AUTOPILOT_ARDUPILOTMEGA, 
        MAV_MODE_AUTO_ARMED, 
        0, 
        MAV_STATE_ACTIVE
    );
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

void set_offboard_mode(int uart_fd, int system_id=SYSTEM_ID, int component_id=COMPONENT_ID) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        system_id,     // system_id
        component_id,  // component_id
        &msg,
        1,
        1,
        MAV_CMD_DO_SET_MODE,
        0,            // confirmation
        1,            // custom mode for PX4
        6,            // sub mode for PX4
        0, 0, 0, 0, 0
    );

    tx_mavlink(uart_fd, &msg);
}

void send_vpe(int uart_fd, float x, float y, float z, float roll, float pitch, float yaw, int system_id=SYSTEM_ID, int component_id=COMPONENT_ID) {
    mavlink_message_t msg;
    mavlink_vision_position_estimate_t vision{};
    
    vision.usec = 0;
    vision.x = x;
    vision.y = y;
    vision.z = z;
    vision.roll = roll;
    vision.pitch = pitch;
    vision.yaw = yaw;

    mavlink_msg_vision_position_estimate_encode(system_id, component_id, &msg, &vision);
    
    tx_mavlink(uart_fd, &msg);
}

void send_position_target(int uart_fd, float x, float y, float z, float yaw, int system_id=SYSTEM_ID, int component_id=COMPONENT_ID) {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        system_id,
        component_id,
        &msg,
        0, 
        1, 
        1, 
        MAV_FRAME_LOCAL_NED,
        0b0000101111111000, 
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        yaw, 0
    );

    tx_mavlink(uart_fd, &msg);
}
