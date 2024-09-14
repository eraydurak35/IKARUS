#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdio.h>
#include "sensors/bmp390.h"
#include "typedefs.h"

#define RANGE_BARO_TRANS_START_ALT 4.0f
#define RANGE_BARO_TRANS_END_ALT 6.0f
#define IN_FLT_MAG_ALLN_ALT 2.0f
#define GYRO_MOVEMENT_DETECT_THRESHOLD 15.0f
#define GYRO_PREDICTED_BIAS_DPS_MAX 2.0f

void ahrs_init(config_t *cfg, states_t *sta, imu_t *icm, magnetometer_t *hmc, bmp390_t *baro, pmw3901_t *flw, range_finder_t *rng, flight_t *flt);
void ahrs_predict();
void ahrs_correct();
void earth_frame_acceleration();
void altitude_predict();
void altitude_correct();
void optical_flow_velocity_XY();

#endif /*STATE_ESTIMATOR_H*/