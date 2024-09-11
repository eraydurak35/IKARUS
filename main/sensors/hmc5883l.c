#include <stdio.h>
#include "sensors/hmc5883l.h"
#include "comminication/i2c.h"
#include "typedefs.h"
#include "esp_log.h"

static calibration_t *mag_calib_data = NULL;

static void parse_hmc5883l_data(magnetometer_t *hmc, uint8_t *buffer);
static void get_calibrated_result(magnetometer_t *hmc);

uint8_t hmc5883l_setup(calibration_t *mg_cal)
{
    uint8_t ret = i2c_read_byte(I2C_NUM_0, HMC5883L_ADDR ,HMC5883L_WHO_AM_I_REG);
    if (ret != HMC5883L_WHO_AM_I_RET)
    {
        ESP_LOGE("Magnetometer", "HMC5883L WHO_AM_I Failed!!");
        return 1;
    }
    mag_calib_data = mg_cal;

    vTaskDelay(10);
    i2c_write_byte(I2C_NUM_0, HMC5883L_ADDR, CONFIG_REG_A, AVERAGE_8 | ODR_75HZ | MODE_NORMAL_HMC);

    vTaskDelay(10);
    i2c_write_byte(I2C_NUM_0, HMC5883L_ADDR, CONFIG_REG_B, RES_1_9_GAUSS);

    vTaskDelay(10);
    i2c_write_byte(I2C_NUM_0, HMC5883L_ADDR, MODE_REG, HS_I2C_ENABLE | MEAS_CONTINUOUS);

    return 0;
}

void hmc5883l_read(magnetometer_t *hmc)
{
    static uint8_t buff[6] = {0};
    i2c_read_bytes(I2C_NUM_0, HMC5883L_ADDR, X_MSB_REG, buff, 6);
    parse_hmc5883l_data(hmc, buff);
    get_calibrated_result(hmc);
}

static void parse_hmc5883l_data(magnetometer_t *hmc, uint8_t *buffer)
{   
    // X Z Y order
    hmc->axis[0] = (float)((int16_t)(buffer[0] << 8 | buffer[1]));
    hmc->axis[2] = (float)((int16_t)(buffer[2] << 8 | buffer[3]));
    hmc->axis[1] = (float)((int16_t)(buffer[4] << 8 | buffer[5]));
}

static void get_calibrated_result(magnetometer_t *mag)
{
    mag->axis[X] -= mag_calib_data->offset[X];
    mag->axis[Y] -= mag_calib_data->offset[Y];
    mag->axis[Z] -= mag_calib_data->offset[Z];

    mag->axis[X] *= mag_calib_data->scale[X];
    mag->axis[Y] *= mag_calib_data->scale[Y];
    mag->axis[Z] *= mag_calib_data->scale[Z];
}