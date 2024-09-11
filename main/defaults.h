#ifndef DEFAULTS_H
#define DEFAULTS_H

#include "typedefs.h"

#define DFLT_MAX_PITCH_ANGLE         20.0f
#define DFLT_MAX_ROLL_ANGLE          20.0f
#define DFLT_MAX_PITCH_RATE          150.0f
#define DFLT_MAX_ROLL_RATE           150.0f
#define DFLT_MAX_YAW_RATE            150.0f 
#define DFLT_PITCH_RATE_SCALE        5.0f
#define DFLT_ROLL_RATE_SCALE         5.0f
#define DFLT_YAW_RATE_SCALE          1.0f
#define DFLT_ALT_VEL_SCALE           0.4f
#define DFLT_MAX_VERTICAL_VELOCITY   1.0f
#define DFLT_MAX_HORIZONTAL_VELOCITY 1.0f
#define DFLT_TAKEOFF_ALTITUDE        1.0f
#define DFLT_VOLTAGE_SENS_GAIN       3.41f  // 3.41  // 11.5
#define DFLT_MAG_DECLINATION_DEG     0.0f
#define DFLT_HOVER_THROTTLE          460.0f
#define DFLT_PITCH_P                 3.0f
#define DFLT_PITCH_I                 1.0f
#define DFLT_PITCH_D                 20.0f
#define DFLT_ROLL_P                  3.0f
#define DFLT_ROLL_I                  1.0f
#define DFLT_ROLL_D                  20.0f
#define DFLT_YAW_P                   10.0f
#define DFLT_YAW_I                   2.0f
#define DFLT_POS_P                   12.0f
#define DFLT_POS_I                   15.0f
#define DFLT_ALT_P                   180.0f // 100
#define DFLT_ALT_I                   50.0f   // 2
#define DFLT_ALT_D                   0.0f
#define DFLT_AHRS_FILTER_BETA        0.3f
#define DFLT_AHRS_FILTER_ZETA        0.003f

void load_default_config(config_t *cfg);


#endif