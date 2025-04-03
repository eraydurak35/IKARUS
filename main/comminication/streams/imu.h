#pragma once

#include "../../mavlink/Ikarus_messages/mavlink.h"
#include "../esp_now_comm.h"
#include "typedefs.h"

void stream_message_imu(mavlink_message_t *_msg, uint8_t *_buffer, states_t *_state, imu_t *_imu, magnetometer_t *_mag)
{
    mavlink_msg_imu_pack(0, 0, _msg,
    _state->pitch_dps * 100,
    _state->roll_dps * 100,
    _state->yaw_dps * 100,
    _imu->accel_ms2[X] * 400,
    _imu->accel_ms2[Y] * 400,
    _imu->accel_ms2[Z] * 400,
    _mag->axis[X],
    _mag->axis[Y],
    _mag->axis[Z],
    _imu->temp_mC);

    const uint16_t len = mavlink_msg_to_send_buffer(_buffer, _msg);

    if (len < ESP_NOW_MAX_DATA_LEN)
    {
        ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, _buffer, len));
    }

}