#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_target_attitude(mavlink_message_t *_msg, uint8_t *_buffer, target_t *_target)
{
    mavlink_msg_target_attitude_pack(0, 0, _msg, 
        _target->pitch_deg, 
        _target->roll_deg, 
        _target->heading_deg, 
        _target->pitch_dps, 
        _target->roll_dps,
        _target->yaw_dps);


    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}