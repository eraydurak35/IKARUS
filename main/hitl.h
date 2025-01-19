#ifndef HITL_H
#define HITL_H

#include <stdio.h>
#include <typedefs.h>
#include <mavlink/common/mavlink.h>

/* extern mavlink_hil_gps_t hil_gps_msg;
extern mavlink_hil_sensor_t hil_sensor_msg; */

uint8_t hitl_read();
void hitl_get_sensors(imu_t *imu, bmp390_t *baro);

void mavlink_send_actuators(float act1, float act2, float act3, float act4);

void mavlink_send_heartbeat();

#endif