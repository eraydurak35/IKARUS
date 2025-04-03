#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_range_finder(mavlink_message_t *_msg, uint8_t *_buffer, range_finder_t *_range)
{   
    mavlink_msg_range_finder_pack(0, 0, _msg, _range->range_cm);

    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}