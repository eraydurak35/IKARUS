#include "param.h"
#include "defaults.h"

/* Parameter description */
PARAM_DEFINE_FLOAT(MC_PITCH_ATT_P,     5.0f,    0.5f,   10.0f);
/* Parameter description */
PARAM_DEFINE_FLOAT(MC_ROLL_ATT_P,      5.0f,    0.5f,   10.0f);
/* Parameter description */
PARAM_DEFINE_FLOAT(MC_YAW_ATT_P,       1.0f,    0.5f,   10.0f);

PARAM_DEFINE_FLOAT(MC_PITCH_RATE_P,    3.0f,    1.0f,   10.0f);
PARAM_DEFINE_FLOAT(MC_ROLL_RATE_P,     3.0f,    1.0f,   10.0f);
PARAM_DEFINE_FLOAT(MC_YAW_RATE_P,     10.0f,    1.0f,   20.0f);

PARAM_DEFINE_FLOAT(MC_PITCH_RATE_I,    1.0f,    0.1f,    5.0f);
PARAM_DEFINE_FLOAT(MC_ROLL_RATE_I,     1.0f,    0.1f,    5.0f);
PARAM_DEFINE_FLOAT(MC_YAW_RATE_I,      2.0f,    0.1f,    5.6f);

PARAM_DEFINE_FLOAT(MC_PITCH_RATE_D,   20.0f,    2.0f,   50.0f);
PARAM_DEFINE_FLOAT(MC_ROLL_RATE_D,    20.0f,    2.0f,   50.0f);

PARAM_DEFINE_FLOAT(MC_XY_POS_P,        1.0f,    0.0f,    5.0f);
PARAM_DEFINE_FLOAT(MC_Z_POS_P,         0.4f,    0.0f,    5.0f);

PARAM_DEFINE_FLOAT(MC_XY_VEL_P,       12.0f,    2.0f,   20.0f);
PARAM_DEFINE_FLOAT(MC_XY_VEL_I,       15.0f,    2.0f,   20.0f);

PARAM_DEFINE_FLOAT(MC_Z_VEL_P,       180.0f,    2.0f,  250.0f);
PARAM_DEFINE_FLOAT(MC_Z_VEL_I,        50.0f,    2.0f,  100.0f);
PARAM_DEFINE_FLOAT(MC_Z_VEL_D,         0.0f,    0.0f,   10.0f);

PARAM_DEFINE_FLOAT(MC_MAX_PITCH_DEG,  20.0f,    0.0f,   50.0f);
PARAM_DEFINE_FLOAT(MC_MAX_ROLL_DEG,   20.0f,    0.0f,   50.0f);

PARAM_DEFINE_FLOAT(MC_MAX_PITCH_RTE, 150.0f,    0.0f,  200.0f);
PARAM_DEFINE_FLOAT(MC_MAX_ROLL_RTE,  150.0f,    0.0f,  200.0f);
PARAM_DEFINE_FLOAT(MC_MAX_YAW_RTE,   150.0f,    0.0f,  200.0f);

PARAM_DEFINE_FLOAT(MC_MAX_XY_VEL,      1.0f,    0.0f,   10.0f);
PARAM_DEFINE_FLOAT(MC_MAX_Z_VEL,       1.0f,    0.0f,    5.0f);

PARAM_DEFINE_FLOAT(MC_WP_THRESHOLD,    5.0f,    0.0f,  100.0f);
PARAM_DEFINE_FLOAT(MC_HOVER_THR,     300.0f,  100.0f,  800.0f);
PARAM_DEFINE_FLOAT(MC_TKOF_ALT,        1.5f,    1.0f,   50.0f);
