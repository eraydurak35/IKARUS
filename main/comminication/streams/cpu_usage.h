#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_cpu_usage(mavlink_message_t *_msg, uint8_t *_buffer, cpu_usage_t *_cpu)
{
    mavlink_msg_cpu_usage_pack(0, 0, _msg,
        _cpu->core0_percent, 
        _cpu->core1_percent);

    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}