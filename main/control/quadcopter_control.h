#ifndef QUADCOPTER_CONTROL_H
#define QUADCOPTER_CONTROL_H

#include "typedefs.h"

#define QUAD_MAX_TARGET_THROTTLE 800
#define QUAD_IDLE_THROTTLE 300
#define EARTH_RADIUS_CM 637100000.0f
#define EARTH_2_RADIUS_CM 1274200000.0f
#define MAX_ANGULAR_ACCEL 650.0f
#define MAX_VEL_Z_ACCEL 5.0f
// 140Hz cutoff --> ~1ms latency
// 80Hz cutoff --> ~2ms latency
// 50Hz cutoff --> ~3ms latency
// 30Hz cutoff --> ~5ms latency
// 25Hz cutoff --> ~6ms latency
#define D_TERM_CUTOFF_FREQ 80.0f
#define P_YAW_CUTOFF_FREQ 50.0f
#define POS_I_CTRL_MAX 5.0f
#define POS_CTRL_MAX_PITCH_ROLL_DEG 15.0f
#define MIN_THROTTLE 100.0f
#define MAX_THROTTLE 1000.0f
#define MAX_I 100.0f
#define MAX_PID 400.0f //300
#define LEASH_LENGHT 400.0f
#define MAX_WP_COUNT 50

void quadcopter_control_init(radio_control_t *rc, telemetry_t *tlm, flight_t *flt, target_t *trg, states_t *stt, config_t *cfg, waypoint_t *wp, gnss_t *gnss);
void quadcopter_flight_control();
uint8_t gnss_sanity_check();


#endif