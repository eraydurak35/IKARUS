#pragma once

#include <stdint.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define NVS_NAMESPACE "param_store"

// Helper macro to get parameter metadata
#define PARAM_META(_name) _name##_

typedef enum {
    PARAM_TYPE_INT32,
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_BOOL
} param_type_t;

typedef struct {
    const char* name;
    param_type_t type;
    union {
        int32_t i;
        float f;
    } min_value;
    union {
        int32_t i;
        float f;
    } max_value;
    union {
        int32_t i;
        float f;
        uint8_t b;
    } default_value;
    union {
        int32_t i;
        float f;
        uint8_t b;
    } value;
} param_t;

typedef struct
{
    union
    {
        int32_t i;
        float f;
        uint8_t b;
    } value;
} param_ret_t;



#define PARAM_DEFINE_FLOAT(_name, _default, _min, _max) \
    param_t PARAM_##_name = { \
        .name = #_name, \
        .type = PARAM_TYPE_FLOAT, \
        .min_value.f = _min, \
        .max_value.f = _max, \
        .default_value.f = _default, \
        .value.f = _default \
    };

#define PARAM_DEFINE_INT32(_name, _default, _min, _max) \
    param_t PARAM_##_name = { \
        .name = #_name, \
        .type = PARAM_TYPE_INT32, \
        .min_value.i = _min, \
        .max_value.i = _max, \
        .default_value.i = _default, \
        .value.i = _default \
};

#define PARAM_DEFINE_BOOL(_name, _default) \
    param_t PARAM_##_name = { \
        .name = #_name, \
        .type = PARAM_TYPE_BOOL, \
        .default_value.b = _default, \
        .value.b = _default \
};
#define PARAM_EXTERN(_name) \
    extern param_t PARAM_##_name;




/***** PARAM_START *****/
PARAM_EXTERN(AHRS_BETA);
PARAM_EXTERN(AHRS_ZETA);
PARAM_EXTERN(EST_Z_POS_BETA);
PARAM_EXTERN(EST_MAG_DECLIN);
PARAM_EXTERN(EST_Z_VEL_BETA);
PARAM_EXTERN(EST_Z_VEL_ZETA);
PARAM_EXTERN(EST_XY_BETA);
PARAM_EXTERN(NOTCH1_FREQ);
PARAM_EXTERN(NOTCH2_FREQ);
PARAM_EXTERN(NOTCH1_BW);
PARAM_EXTERN(NOTCH2_BW);
PARAM_EXTERN(LPF_CUTOFF);
PARAM_EXTERN(VOLT_GAIN);
PARAM_EXTERN(MC_PITCH_ATT_P);
PARAM_EXTERN(MC_ROLL_ATT_P);
PARAM_EXTERN(MC_YAW_ATT_P);
PARAM_EXTERN(MC_PITCH_RATE_P);
PARAM_EXTERN(MC_ROLL_RATE_P);
PARAM_EXTERN(MC_YAW_RATE_P);
PARAM_EXTERN(MC_PITCH_RATE_I);
PARAM_EXTERN(MC_ROLL_RATE_I);
PARAM_EXTERN(MC_YAW_RATE_I);
PARAM_EXTERN(MC_PITCH_RATE_D);
PARAM_EXTERN(MC_ROLL_RATE_D);
PARAM_EXTERN(MC_XY_POS_P);
PARAM_EXTERN(MC_Z_POS_P);
PARAM_EXTERN(MC_XY_VEL_P);
PARAM_EXTERN(MC_XY_VEL_I);
PARAM_EXTERN(MC_Z_VEL_P);
PARAM_EXTERN(MC_Z_VEL_I);
PARAM_EXTERN(MC_Z_VEL_D);
PARAM_EXTERN(MC_MAX_PITCH_DEG);
PARAM_EXTERN(MC_MAX_ROLL_DEG);
PARAM_EXTERN(MC_MAX_PITCH_RTE);
PARAM_EXTERN(MC_MAX_ROLL_RTE);
PARAM_EXTERN(MC_MAX_YAW_RTE);
PARAM_EXTERN(MC_MAX_XY_VEL);
PARAM_EXTERN(MC_MAX_Z_VEL);
PARAM_EXTERN(MC_WP_THRESHOLD);
PARAM_EXTERN(MC_HOVER_THR);
PARAM_EXTERN(MC_TKOF_ALT);
/***** PARAM_END *****/




















/* 
// Helper macro to get parameter metadata
#define PARAM_META(_name) _name##

// Parameter getter functions declarations
int32_t param_get_int32(const char* name, int32_t default_val);
float param_get_float(const char* name, float default_val);
uint8_t param_get_bool(const char* name, uint8_t default_val);

// Modified parameter definition macros
#define PARAM_DEFINE_INT32(_name, _default, _min, _max) \
    static param_t _name## = { \
        .name = #_name, \
        .type = PARAM_TYPE_INT32, \
        .min_value.i = _min, \
        .max_value.i = _max, \
        .default_value.i = _default \
    }; \
    int32_t _name = _default;

#define PARAM_DEFINE_FLOAT(_name, _default, _min, _max) \
    static param_t _name## = { \
        .name = #_name, \
        .type = PARAM_TYPE_FLOAT, \
        .min_value.f = _min, \
        .max_value.f = _max, \
        .default_value.f = _default \
    }; \
    float _name = _default;

#define PARAM_DEFINE_BOOL(_name, _default) \
    static param_t _name## = { \
        .name = #_name, \
        .type = PARAM_TYPE_BOOL, \
        .default_value.b = _default \
    }; \
    uint8_t _name = _default;

// Function to get parameter value
static inline float param_get_value_float(const char* name) {
    return param_get_float(name, PARAM_META(MC_ROLL_P).default_value.f);
}

// PARAM_GET macro replacement as a function
float get_mc_roll_p() {
    return param_get_value_float("MC_ROLL_P");
}


param_ret_t param_get(const char* name)
{
    if (PARAM_META(name).)
}


// Function implementations (same as before)
int32_t param_get_int32(const char* name, int32_t default_val) {
    int32_t value = default_val;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_i32(handle, name, &value);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE("PARAM", "Failed to read %s from NVS: %s", name, esp_err_to_name(err));
        }
        nvs_close(handle);
    }
    return value;
}

float param_get_float(const char* name, float default_val) {
    float value = default_val;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(float);
        err = nvs_get_blob(handle, name, &value, &required_size);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE("PARAM", "Failed to read %s from NVS: %s", name, esp_err_to_name(err));
        }
        nvs_close(handle);
    }
    return value;
}

uint8_t param_get_bool(const char* name, uint8_t default_val) {
    uint8_t value = default_val;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_u8(handle, name, &value);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE("PARAM", "Failed to read %s from NVS: %s", name, esp_err_to_name(err));
        }
        nvs_close(handle);
    }
    return value;
} */