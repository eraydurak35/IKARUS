#include <cstdint>
#include "ecl_c_wrapper.h"
#include "../components/ecl/ecl.h"
#include "../components/ecl/EKF/ekf.h"
#include "../components/ecl/EKF/common.h"

extern "C" {

    Ekf ekf;
    static estimator::imuSample imu_sample;
    static estimator::magSample mag_sample;
    static estimator::baroSample baro_sample;
    static matrix::Eulerf euler = Eulerf(matrix::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f)); // Initialize with identity quaternion
    static matrix::Vector3f velocity_ned = matrix::Vector3f(0.0f, 0.0f, 0.0f);
    static matrix::Vector3f position_ned = matrix::Vector3f(0.0f, 0.0f, 0.0f);
    static uint64_t prev_imu_sample_time_us = 0.0f;

    uint8_t ekf_init(uint64_t init_time_us) 
    {
        prev_imu_sample_time_us = init_time_us;
        return (uint8_t)ekf.init(init_time_us);
    }

    void ekf_set_imu_data(float gyx, float gyy, float gyz, 
                        float acx, float acy, float acz, uint64_t time_us)
    {
        imu_sample.time_us = time_us;
        imu_sample.delta_ang_dt = (time_us - prev_imu_sample_time_us) * 1e-6f; // Convert microseconds to seconds
        imu_sample.delta_vel_dt = (time_us - prev_imu_sample_time_us) * 1e-6f; // Convert microseconds to seconds
        imu_sample.delta_ang = matrix::Vector3f(math::radians(gyx) * imu_sample.delta_ang_dt, math::radians(gyy) * imu_sample.delta_ang_dt, math::radians(gyz) * imu_sample.delta_ang_dt);
        imu_sample.delta_vel = matrix::Vector3f(acx * imu_sample.delta_vel_dt, acy * imu_sample.delta_vel_dt, acz * imu_sample.delta_vel_dt);
        imu_sample.delta_vel_clipping[0] = false;
        imu_sample.delta_vel_clipping[1] = false;
        imu_sample.delta_vel_clipping[2] = false;
        ekf.setIMUData(imu_sample);

        prev_imu_sample_time_us = time_us;
    }

    void ekf_set_mag_data(float mag_x, float mag_y, float mag_z, uint64_t time_us) 
    {
        mag_sample.time_us = time_us;
        mag_sample.mag = matrix::Vector3f(mag_x, mag_y, mag_z);
        ekf.setMagData(mag_sample);
    }

    void ekf_set_baro_data(float altitude, uint64_t time_us) 
    {
        baro_sample.time_us = time_us;
        baro_sample.hgt = altitude;
        ekf.setBaroData(baro_sample);
    }

    void ekf_update()
    {
        ekf.update();
        euler = matrix::Eulerf(ekf.getQuaternion());
        velocity_ned = ekf.getVelocity();
        position_ned = ekf.getPosition();
    }

    float ekf_get_pitch_deg()
    {
        return math::degrees(euler.theta());
    }

    float ekf_get_roll_deg()
    {
        return math::degrees(euler.phi());
    }

    float ekf_get_heading_deg()
    {
        if (euler.psi() < 0.0f) return math::degrees(euler.psi()) + 360.0f;
        return math::degrees(euler.psi());
    }

    void ekf_print_status()
    {
        ekf.print_status();
    }

    float ekf_get_velocity_north()
    {
        return velocity_ned(0);
    }

    float ekf_get_velocity_east()
    {
        return velocity_ned(1);
    }

    float ekf_get_velocity_down()
    {
        return velocity_ned(2);
    }

    float ekf_get_position_north()
    {
        return position_ned(0);
    }

    float ekf_get_position_east()
    {
        return position_ned(1);
    }

    float ekf_get_position_down()
    {
        return position_ned(2);
    }

}