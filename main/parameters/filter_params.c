#include "param.h"
#include "defaults.h"

PARAM_DEFINE_FLOAT(NOTCH1_FREQ,     100.0f,     40.0f,      400.0f);
PARAM_DEFINE_FLOAT(NOTCH2_FREQ,     120.0f,     40.0f,      400.0f);
PARAM_DEFINE_FLOAT(NOTCH1_BW,       40.0f,      20.0f,      120.0f);
PARAM_DEFINE_FLOAT(NOTCH2_BW,       40.0f,      20.0f,      120.0f);
PARAM_DEFINE_FLOAT(LPF_CUTOFF,      200.0f,     40.0f,      400.0f);
