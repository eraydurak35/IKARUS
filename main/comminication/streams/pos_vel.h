#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_pos_vel(mavlink_message_t *_msg, uint8_t *_buffer, gnss_t *_gnss, states_t *_state)
{
    const float vel_xy = sqrtf(_state->vel_forward_ms * _state->vel_forward_ms + _state->vel_right_ms * _state->vel_right_ms);
    
    mavlink_msg_pos_vel_pack(0, 0, _msg, 
        _gnss->latitude, 
        _gnss->longitude,
        _state->altitude_m, 
        _state->vel_forward_ms, 
        _state->vel_right_ms, 
        _state->vel_up_ms, 
        vel_xy);

    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}