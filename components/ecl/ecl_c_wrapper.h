#pragma once

#ifdef __cplusplus
extern "C" {
#endif

uint8_t ekf_init(uint64_t init_time_us);

void ekf_set_imu_data(float gyx, float gyy, float gyz, float acx, float acy, float acz, uint64_t time_us);

void ekf_set_mag_data(float mag_x, float mag_y, float mag_z, uint64_t time_us);

void ekf_set_baro_data(float altitude, uint64_t time_us);

void ekf_update();

float ekf_get_pitch_deg();

float ekf_get_roll_deg();

float ekf_get_heading_deg();

void ekf_print_status();

float ekf_get_velocity_north();

float ekf_get_velocity_east();

float ekf_get_velocity_down();

float ekf_get_position_north();

float ekf_get_position_east();

float ekf_get_position_down();

uint8_t ekf_is_attitude_valid();

#ifdef __cplusplus
}
#endif