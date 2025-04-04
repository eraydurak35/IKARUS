#ifndef DEFAULTS_H
#define DEFAULTS_H

#include "typedefs.h"
#include <setup.h>

#if SETUP_ENABLE_HITL == true

#define DFLT_MC_MAX_PITCH_DEG               25.0f
#define DFLT_MC_MAX_ROLL_DEG                25.0f
#define DFLT_MAX_PITCH_RATE          150.0f
#define DFLT_MAX_ROLL_RATE           150.0f
#define DFLT_MAX_YAW_RATE            100.0f 
#define DFLT_PITCH_RATE_SCALE        4.5f
#define DFLT_ROLL_RATE_SCALE         4.5f
#define DFLT_YAW_RATE_SCALE          1.0f
#define DFLT_ALT_VEL_SCALE           0.4f
#define DFLT_MAX_VERTICAL_VELOCITY   1.0f
#define DFLT_MAX_HORIZONTAL_VELOCITY 1.0f
#define DFLT_TAKEOFF_ALTITUDE        1.0f
#define DFLT_VOLTAGE_SENS_GAIN       3.41f  // 3.41  // 11.5
#define DFLT_MAG_DECLINATION_DEG     0.0f
#define DFLT_HOVER_THROTTLE          400.0f
#define DFLT_PITCH_P                 0.2f
#define DFLT_PITCH_I                 0.0f
#define DFLT_PITCH_D                 2.5f
#define DFLT_ROLL_P                  0.2f
#define DFLT_ROLL_I                  0.0f
#define DFLT_ROLL_D                  2.5f
#define DFLT_YAW_P                   5.0f
#define DFLT_YAW_I                   0.0f
#define DFLT_POS_P                   12.0f
#define DFLT_POS_I                   15.0f
#define DFLT_ALT_P                   180.0f // 100
#define DFLT_ALT_I                   50.0f   // 2
#define DFLT_ALT_D                   0.0f
#define DFLT_AHRS_FILTER_BETA        0.05f
#define DFLT_AHRS_FILTER_ZETA        0.0f
#define DFLT_LPF_CUTOFF_HZ           80.0f
#define DFLT_NOTCH_1_FREQ            262.0f
#define DFLT_NOTCH_1_BNDWDTH         45.0f

#else


#define DFLT_MC_MAX_PITCH_DEG           20.0f
#define DFLT_MC_MAX_ROLL_DEG            20.0f
#define DFLT_MC_MAX_PITCH_RTE          150.0f
#define DFLT_MC_MAX_ROLL_RTE           150.0f
#define DFLT_MC_MAX_YAW_RTE            150.0f 
#define DFLT_MC_PITCH_ATT_P             5.0f
#define DFLT_MC_ROLL_ATT_P              5.0f
#define DFLT_MC_YAW_ATT_P               1.0f
#define DFLT_MC_Z_POS_P                 0.4f
#define DFLT_MC_MAX_Z_VEL              1.0f
#define DFLT_MC_MAX_XY_VEL               1.0f
#define DFLT_TAKEOFF_ALTITUDE           1.0f
#define DFLT_VOLTAGE_SENS_GAIN          3.41f  // 3.41  // 11.5
#define DFLT_MAG_DECLINATION_DEG        0.0f
#define DFLT_MC_HOVER_THROTTLE          460.0f
#define DFLT_MC_PITCH_P                 3.0f
#define DFLT_MC_PITCH_I                 1.0f
#define DFLT_MC_PITCH_D                 20.0f
#define DFLT_MC_ROLL_P                  3.0f
#define DFLT_MC_ROLL_I                  1.0f
#define DFLT_MC_ROLL_D                  20.0f
#define DFLT_MC_YAW_P                   10.0f
#define DFLT_MC_YAW_I                   2.0f
#define DFLT_MC_XY_VEL_P                   12.0f
#define DFLT_MC_XY_VEL_I                   15.0f
#define DFLT_MC_Z_VEL_P                   180.0f // 100
#define DFLT_MC_Z_VEL_I                   50.0f   // 2
#define DFLT_MC_Z_VEL_D                   0.0f
#define DFLT_AHRS_BETA                  0.01f
#define DFLT_AHRS_ZETA                 0.001f
#define DFLT_LPF_CUTOFF_HZ           80.0f
#define DFLT_NOTCH_1_FREQ            262.0f
#define DFLT_NOTCH_1_BNDWDTH         45.0f
#define DFLT_MC_XY_POS_P                1.0f
#define DFLT_MC_WP_THRESHOLD          5.0f

#endif

void load_default_config(config_t *cfg);


#endif