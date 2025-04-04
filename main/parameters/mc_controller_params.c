#include "param.h"
#include "defaults.h"

/* ATTITUDE CONTROLLER PARAMS */
static param_t param_mc_pitch_att_p = {
    .name = "MC_PITCH_ATT_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.5f },
    .max_value = { .f = 10.0f },
    .value = { .f = DFLT_MC_PITCH_ATT_P }
};

static param_t param_mc_roll_att_p = {
    .name = "MC_ROLL_ATT_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.5f },
    .max_value = { .f = 10.0f },
    .value = { .f = DFLT_MC_ROLL_ATT_P }
};

static param_t param_mc_yaw_att_p = {
    .name = "MC_YAW_ATT_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.5f },
    .max_value = { .f = 10.0f },
    .value = { .f = DFLT_MC_YAW_ATT_P }
};

/* RATE CONTROLLER PARAMS */
static param_t param_mc_pitch_rate_p = {
    .name = "MC_PITCH_RATE_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 1.0f },
    .max_value = { .f = 10.0f },
    .value = { .f = DFLT_MC_PITCH_P }
};

static param_t param_mc_roll_rate_p = {
    .name = "MC_ROLL_RATE_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 1.0f },
    .max_value = { .f = 10.0f },
    .value = { .f = DFLT_MC_ROLL_P }
};

static param_t param_mc_yaw_rate_p = {
    .name = "MC_YAW_RATE_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 1.0f },
    .max_value = { .f = 20.0f },
    .value = { .f = DFLT_MC_YAW_P }
};

static param_t param_mc_pitch_rate_i = {
    .name = "MC_PITCH_RATE_I",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.1f },
    .max_value = { .f = 5.0f },
    .value = { .f = DFLT_MC_PITCH_I }
};

static param_t param_mc_roll_rate_i = {
    .name = "MC_ROLL_RATE_I",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.1f },
    .max_value = { .f = 5.0f },
    .value = { .f = DFLT_MC_ROLL_I }
};

static param_t param_mc_yaw_rate_i = {
    .name = "MC_YAW_RATE_I",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.1f },
    .max_value = { .f = 5.6f },
    .value = { .f = DFLT_MC_YAW_I }
};

static param_t param_mc_pitch_rate_d = {
    .name = "MC_PITCH_RATE_D",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 2.0f },
    .max_value = { .f = 50.0f },
    .value = { .f = DFLT_MC_PITCH_D }
};

static param_t param_mc_roll_rate_d = {
    .name = "MC_ROLL_RATE_D",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 2.0f },
    .max_value = { .f = 50.0f },
    .value = { .f = DFLT_MC_ROLL_D }
};

/* POSITION CONTROLLER PARAMS */
static param_t param_mc_xy_pos_p = {
    .name = "MC_XY_POS_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 5.0f },
    .value = { .f = DFLT_MC_XY_POS_P }
};

static param_t param_mc_z_pos_p = {
    .name = "MC_Z_POS_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 5.0f },
    .value = { .f = DFLT_MC_Z_POS_P }
};

/* VELOCITY CONTROLLER PARAMS */
static param_t param_mc_xy_vel_p = {
    .name = "MC_XY_VEL_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 2.0f },
    .max_value = { .f = 20.0f },
    .value = { .f = DFLT_MC_XY_VEL_P }
};

static param_t param_mc_xy_vel_i = {
    .name = "MC_XY_VEL_I",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 2.0f },
    .max_value = { .f = 20.0f },
    .value = { .f = DFLT_MC_XY_VEL_I }
};

static param_t param_mc_z_vel_p = {
    .name = "MC_Z_VEL_P",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 2.0f },
    .max_value = { .f = 250.0f },
    .value = { .f = DFLT_MC_Z_VEL_P }
};

static param_t param_mc_z_vel_i = {
    .name = "MC_Z_VEL_I",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 2.0f },
    .max_value = { .f = 100.0f },
    .value = { .f = DFLT_MC_Z_VEL_I }
};

static param_t param_mc_z_vel_d = {
    .name = "MC_Z_VEL_D",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 10.0f },
    .value = { .f = DFLT_MC_Z_VEL_D }
};

/* MAX MIN LIMITS */

static param_t param_mc_max_pitch_deg = {
    .name = "MC_MAX_PITCH_DEG",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 50.0f },
    .value = { .f = DFLT_MC_MAX_PITCH_DEG }
};

static param_t param_mc_max_roll_deg = {
    .name = "MC_MAX_ROLL_DEG",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 50.0f },
    .value = { .f = DFLT_MC_MAX_ROLL_DEG }
};

static param_t param_mc_max_pitch_rte = {
    .name = "MC_MAX_PITCH_RTE",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 200.0f },
    .value = { .f = DFLT_MC_MAX_PITCH_RTE }
};

static param_t param_mc_max_roll_rte = {
    .name = "MC_MAX_ROLL_RTE",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 200.0f },
    .value = { .f = DFLT_MC_MAX_ROLL_RTE }
};

static param_t param_mc_max_yaw_rte = {
    .name = "MC_MAX_YAW_RTE",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 200.0f },
    .value = { .f = DFLT_MC_MAX_YAW_RTE }
};

static param_t param_mc_max_xy_vel = {
    .name = "MC_MAX_XY_VEL",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 10.0f },
    .value = { .f = DFLT_MC_MAX_XY_VEL }
};

static param_t param_mc_max_z_vel = {
    .name = "MC_MAX_Z_VEL",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 5.0f },
    .value = { .f = DFLT_MC_MAX_Z_VEL }
};

static param_t param_mc_wp_threshold = {
    .name = "MC_WP_THRESHOLD",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 0.0f },
    .max_value = { .f = 100.0f },
    .value = { .f = DFLT_MC_WP_THRESHOLD }
};

static param_t param_mc_hover_thr = {
    .name = "MC_HOVER_THR",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 100.0f },
    .max_value = { .f = 800.0f },
    .value = { .f = 300.0f }
};

static param_t param_mc_tkof_alt = {
    .name = "MC_TKOF_ALT",
    .type = PARAM_TYPE_FLOAT,
    .min_value = { .f = 1.0f },
    .max_value = { .f = 50.0f },
    .value = { .f = 1.5f }
};
