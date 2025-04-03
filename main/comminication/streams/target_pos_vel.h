#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_target_pos_vel(mavlink_message_t *_msg, uint8_t *_buffer, target_t *_target)
{
    mavlink_msg_target_pos_vel_pack(0, 0, _msg,
        _target->latitude,
        _target->longitude,
        _target->altitude,
        _target->velocity_x_ms,
        _target->velocity_y_ms,
        _target->velocity_z_ms,
        _target->throttle);

    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}