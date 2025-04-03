#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_gnss(mavlink_message_t *_msg, uint8_t *_buffer, gnss_t *_gnss)
{
    mavlink_msg_gnss_pack(0, 0, _msg,
        _gnss->fix,
        _gnss->satCount,
        _gnss->latitude,
        _gnss->longitude,
        _gnss->altitude_mm,
        _gnss->northVel_mms,
        _gnss->eastVel_mms,
        _gnss->downVel_mms,
        _gnss->headingOfMotion,
        _gnss->hdop,
        _gnss->vdop);

    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}