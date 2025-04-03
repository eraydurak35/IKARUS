#pragma once

#include "stdint.h"
#include "imu.h"
#include "barometer.h"
#include "attitude.h"
#include "target_attitude.h"
#include "gnss.h"

typedef enum
{
    HEARTBEAT_MSG_OFFSET,
    IMU_MSG_OFFSET,
    BAROMETER_MSG_OFFSET,
    ATTITUDE_MSG_OFFSET,
    TARGET_ATTITUDE_MSG_OFFSET,
    GNSS_MSG_OFFSET,

}message_offsets_t;

typedef enum
{
    HEARTBEAT_MSG_INTERVAL = 10,
    IMU_MSG_INTERVAL = 2,
    BAROMETER_MSG_INTERVAL = 5,
    ATTITUDE_MSG_INTERVAL = 2,
    TARGET_ATTITUDE_MSG_INTERVAL = 2,
    GNSS_MSG_INTERVAL = 10,

}message_intervals_t;

static config_t *config_ptr = NULL;
static waypoint_t *waypoint_ptr = NULL;
static telemetry_t *telemetry_ptr = NULL;
static flight_t *flight_ptr = NULL;
static states_t *state_ptr = NULL;
static imu_t *imu_ptr = NULL;
static magnetometer_t *mag_ptr = NULL;
static bmp390_t *baro_ptr = NULL;
static target_t *target_ptr = NULL;
static gnss_t *gnss_ptr = NULL;
static pmw3901_t *flow_ptr = NULL;
static range_finder_t *range_ptr = NULL;
static gamepad_t *gamepad_ptr = NULL;

static uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN] = {0};
static mavlink_message_t mavlink_msg;

void start_mavlink_stream(config_t *cfg, waypoint_t *wp, telemetry_t *telem, flight_t *flt, states_t *stt, imu_t *imu, magnetometer_t *mag, bmp390_t *baro, gnss_t *gnss, pmw3901_t *flow, range_finder_t *range, target_t *target, gamepad_t *gmpd)
{
    config_ptr = cfg;
    waypoint_ptr = wp;
    gamepad_ptr = gmpd;
    telemetry_ptr = telem;
    flight_ptr = flt;
    state_ptr = stt;
    imu_ptr = imu;
    mag_ptr = mag;
    baro_ptr = baro;
    target_ptr = target;
    gnss_ptr = gnss;
    flow_ptr = flow;
    range_ptr = range;
}


// 10 Hz
void run_mavlink_stream()
{
    static uint32_t counter = 0;

    counter++;

    if ((counter - HEARTBEAT_MSG_OFFSET) % HEARTBEAT_MSG_INTERVAL == 0) {
        
    }
    if ((counter - IMU_MSG_OFFSET) % IMU_MSG_INTERVAL == 0) {
        stream_message_imu(&mavlink_msg, mavlink_buffer, state_ptr, imu_ptr, mag_ptr);
    }
    if ((counter - BAROMETER_MSG_OFFSET) % BAROMETER_MSG_INTERVAL == 0) {
        stream_message_barometer(&mavlink_msg, mavlink_buffer, baro_ptr);
    }
    if ((counter - ATTITUDE_MSG_OFFSET) % ATTITUDE_MSG_INTERVAL == 0) {
        stream_message_attitude(&mavlink_msg, mavlink_buffer, state_ptr);
    }
    if ((counter - TARGET_ATTITUDE_MSG_OFFSET) % TARGET_ATTITUDE_MSG_INTERVAL == 0) {
        stream_message_target_attitude(&mavlink_msg, mavlink_buffer, target_ptr);
    }
    if ((counter - GNSS_MSG_OFFSET) % GNSS_MSG_INTERVAL == 0) {
        stream_message_gnss(&mavlink_msg, mavlink_buffer, gnss_ptr);
    }
}