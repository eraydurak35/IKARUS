#include "param.h"
#include "defaults.h"

/* param_t param_notch1_freq = {
    .name = "NOTCH1_FREQ",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 40.0f },
    .max_value = { .f = 400.0f },
    .value = { .f = 100.0f }
};

param_t param_notch2_freq = {
    .name = "NOTCH2_FREQ",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 40.0f },
    .max_value = { .f = 400.0f },
    .value = { .f = 120.0f }
};

param_t param_notch1_bw = {
    .name = "NOTCH1_BW",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 20.0f },
    .max_value = { .f = 120.0f },
    .value = { .f = 40.0f }
};

param_t param_notch2_bw = {
    .name = "NOTCH2_BW",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 20.0f },
    .max_value = { .f = 120.0f },
    .value = { .f = 40.0f }
};

param_t param_lpf_cutoff = {
    .name = "LPF_CUTOFF",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 40.0f },
    .max_value = { .f = 400.0f },
    .value = { .f = 200.0f }
}; */


PARAM_DEFINE_FLOAT(NOTCH1_FREQ, 100.0f, 40.0f, 400.0f);
PARAM_DEFINE_FLOAT(NOTCH2_FREQ, 120.0f, 40.0f, 400.0f);
PARAM_DEFINE_FLOAT(NOTCH1_BW,   40.0f,  20.0f, 120.0f);
PARAM_DEFINE_FLOAT(NOTCH2_BW,   40.0f,  20.0f, 120.0f);
PARAM_DEFINE_FLOAT(LPF_CUTOFF,  200.0f, 40.0f, 400.0f);
