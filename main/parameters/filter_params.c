#include "param.h"
#include "defaults.h"

static param_t param_notch1_freq = {
    .name = "NOTCH1_FREQ",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 40.0f },
    .max_value = { .f = 400.0f },
    .value = { .f = 100.0f }
};

static param_t param_notch2_freq = {
    .name = "NOTCH2_FREQ",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 40.0f },
    .max_value = { .f = 400.0f },
    .value = { .f = 120.0f }
};

static param_t param_notch1_bw = {
    .name = "NOTCH1_BW",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 20.0f },
    .max_value = { .f = 120.0f },
    .value = { .f = 40.0f }
};

static param_t param_notch2_bw = {
    .name = "NOTCH2_BW",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 20.0f },
    .max_value = { .f = 120.0f },
    .value = { .f = 40.0f }
};

static param_t param_lpf_cutoff = {
    .name = "LPF_CUTOFF",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 20.0f },
    .max_value = { .f = 120.0f },
    .value = { .f = 200.0f }
};

