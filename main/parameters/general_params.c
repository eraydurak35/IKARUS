#include "param.h"
#include "defaults.h"


static param_t param_volt_gain = {
    .name = "VOLT_GAIN",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.5f },
    .max_value = { .f = 10.0f },
    .value = { .f = 1.0f }
};

