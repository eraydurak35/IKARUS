#include "param.h"
#include "defaults.h"


PARAM_DEFINE_FLOAT(AHRS_BETA,       0.01f,      0.0f,   2.0f);
PARAM_DEFINE_FLOAT(AHRS_ZETA,       0.001f,     0.0f,   1.0f);
PARAM_DEFINE_FLOAT(EST_Z_POS_BETA,  0.1f,       0.0f,   1.0f);
PARAM_DEFINE_FLOAT(EST_MAG_DECLIN,  0.0f,      -20.0f,  20.0f);
PARAM_DEFINE_FLOAT(EST_Z_VEL_BETA,  0.001f,     0.0f,   1.0f);
PARAM_DEFINE_FLOAT(EST_Z_VEL_ZETA,  0.0001f,    0.0f,   1.0f);
PARAM_DEFINE_FLOAT(EST_XY_BETA,     0.0001f,    0.0f,   1.0f);
