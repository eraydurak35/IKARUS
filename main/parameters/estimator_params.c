#include "param.h"
#include "defaults.h"


static param_t param_ahrs_beta = {
    .name = "AHRS_BETA",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 2.0f },
    .value = { .f = DFLT_AHRS_BETA }
};

static param_t param_ahrs_zeta = {
    .name = "AHRS_ZETA",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 1.0f },
    .value = { .f = DFLT_AHRS_ZETA }
};

static param_t param_est_z_pos_beta = {
    .name = "EST_Z_POS_BETA",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 1.0f },
    .value = { .f = 0.1f }
};

static param_t param_est_mag_declin = {
    .name = "EST_MAG_DECLIN",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = -20.0f },
    .max_value = { .f = 20.0f },
    .value = { .f = 0.0f }
};

static param_t param_est_z_vel_beta = {
    .name = "EST_Z_VEL_BETA",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 1.0f },
    .value = { .f = 0.001f }
};

static param_t param_est_z_vel_zeta = {
    .name = "EST_Z_VEL_ZETA",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 1.0f },
    .value = { .f = 0.0001f }
};

static param_t param_est_xy_beta = {
    .name = "EST_XY_BETA",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 1.0f },
    .value = { .f = 0.0001f }
};

