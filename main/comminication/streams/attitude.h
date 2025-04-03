#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_attitude(mavlink_message_t *_msg, uint8_t *_buffer, states_t *_state)
{
    mavlink_msg_attitude_pack(0, 0, _msg,
    _state->pitch_deg,
    _state->roll_deg,
    _state->heading_deg);

    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}