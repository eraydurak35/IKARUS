#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_optical_flow(mavlink_message_t *_msg, uint8_t *_buffer, pmw3901_t *_flow)
{   
    mavlink_msg_optical_flow_pack(0, 0, _msg,
        _flow->quality, 
        _flow->velocity_x_ms, 
        _flow->velocity_y_ms);

    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}