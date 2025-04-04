#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_rc_channels(mavlink_message_t *_msg, uint8_t *_buffer, radio_control_t *_radio)
{
    mavlink_msg_rc_channels_pack(0, 0, _msg,
        _radio->channel[0],
        _radio->channel[1],
        _radio->channel[2],
        _radio->channel[3],
        _radio->channel[4],
        _radio->channel[5],
        _radio->channel[6],
        _radio->channel[7],
        _radio->channel[8],
        _radio->channel[9],
        _radio->channel[10],
        _radio->channel[11],
        _radio->channel[12],
        _radio->channel[13]);


    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}