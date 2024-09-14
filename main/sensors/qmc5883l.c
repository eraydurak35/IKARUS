#include <stdio.h>
#include "sensors/qmc5883l.h"
#include "comminication/i2c.h"
#include "typedefs.h"
#include "esp_log.h"
#include "setup.h"

static calibration_t *mag_calib_data = NULL;

static void parse_qmc5883l_data(magnetometer_t *mag, uint8_t *buffer);
static void get_calibrated_result(magnetometer_t *mag);
/* static void mag_throttle_correction(magnetometer_t *mag, uint16_t throttle); */
static uint8_t qmc5883l_check_drdy_bit();

uint8_t qmc5883l_setup(calibration_t *mg_cal)
{
    uint8_t ret = i2c_read_byte(I2C_NUM_0, QMC5883L_ADDR ,QMC5883L_WHO_AM_I_REG);
    if (ret != QMC5883L_WHO_AM_I_RET)
    {
        ESP_LOGE("Magnetometer", "QMC5883L WHO_AM_I Failed!!");
        return 1;
    }

    mag_calib_data = mg_cal;

    vTaskDelay(10);
    i2c_write_byte(I2C_NUM_0, QMC5883L_ADDR, SET_RESET_REG, 0x01);

    vTaskDelay(10);
    i2c_write_byte(I2C_NUM_0, QMC5883L_ADDR, 0x0A, 0x41);

    vTaskDelay(10);
    i2c_write_byte(I2C_NUM_0, QMC5883L_ADDR, CTRL_REG_1, MODE_CONTINUOUS | ODR_50_HZ | OSR_512 | SCALE_8_GAUSS);

    return 0;
}

void qmc5883l_read(magnetometer_t *mag, uint16_t throttle)
{
    static uint8_t buff[6] = {0};

    if (qmc5883l_check_drdy_bit() == 1)
    {
        i2c_read_bytes(I2C_NUM_0, QMC5883L_ADDR, OUTPUT_X_REG, buff, 6);
        parse_qmc5883l_data(mag, buff);
        //mag_throttle_correction(mag, throttle);
        get_calibrated_result(mag);
    }
}

/* static void mag_throttle_correction(magnetometer_t *mag, uint16_t throttle)
{
    static const float correction_vector[3] = {-0.0782f, 0.512f, -0.2185f};

    mag->axis[X] += correction_vector[X] * (throttle - 200);
    mag->axis[Y] += correction_vector[Y] * (throttle - 200);
    mag->axis[Z] += correction_vector[Z] * (throttle - 200);
} */

static uint8_t qmc5883l_check_drdy_bit()
{
    uint8_t ret = i2c_read_byte(I2C_NUM_0, QMC5883L_ADDR, STATUS_REG);

    if (ret == 1 || ret == 4 || ret == 5) return 1;
    return 0;
}

static void parse_qmc5883l_data(magnetometer_t *mag, uint8_t *buffer)
{   
    // X Y Z order
    static int16_t axis[3] = {0, 0, 0};

    axis[X] = (int16_t)(buffer[0] | buffer[1] << 8);
    axis[Y] = (int16_t)(buffer[2] | buffer[3] << 8);
    axis[Z] = (int16_t)(buffer[4] | buffer[5] << 8);

    mag->axis[X] = axis[ALIGNED_MAG_X_AXIS] * ALIGNED_MAG_X_AXIS_SIGN;
    mag->axis[Y] = axis[ALIGNED_MAG_Y_AXIS] * ALIGNED_MAG_Y_AXIS_SIGN;
    mag->axis[Z] = axis[ALIGNED_MAG_Z_AXIS] * ALIGNED_MAG_Z_AXIS_SIGN;
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

