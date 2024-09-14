#include <stdio.h>
#include "sensors/icm42688p.h"
#include "comminication/spi.h"
#include "math.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "typedefs.h"
#include "setup.h"

static spi_device_handle_t icm42688p_handle;
static calibration_t *accel_calib_data = NULL;

static void parse_icm42688p_data(imu_t *imu, uint8_t *buff);
static void getCalibratedResults(imu_t *imu);

void icm42688p_setup(calibration_t *acc_cal)
{
    accel_calib_data = acc_cal;
    spi_add_device_to_bus(&icm42688p_handle, SETUP_ICM42688P_CS_PIN, SPI_FREQ);
    spi_write_byte(icm42688p_handle, ICM426XX_RA_REG_BANK_SEL, ICM426XX_BANK_SELECT0);
    // Turn gyro acc off
    spi_write_byte(icm42688p_handle, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);
    vTaskDelay(20);

    // gyro LPF 258Hz
    spi_write_byte(icm42688p_handle, ICM426XX_RA_REG_BANK_SEL, ICM426XX_BANK_SELECT1);
    spi_write_byte(icm42688p_handle, ICM426XX_RA_GYRO_CONFIG_STATIC3, 6);
    spi_write_byte(icm42688p_handle, ICM426XX_RA_GYRO_CONFIG_STATIC4, 36 & 0xFF);
    spi_write_byte(icm42688p_handle, ICM426XX_RA_GYRO_CONFIG_STATIC5, (36 >> 8) | (10 << 4));

    vTaskDelay(20);
    // acc LPF 258Hz
    spi_write_byte(icm42688p_handle, ICM426XX_RA_REG_BANK_SEL, ICM426XX_BANK_SELECT2);

    spi_write_byte(icm42688p_handle, ICM426XX_RA_ACCEL_CONFIG_STATIC2, 6 << 1);
    spi_write_byte(icm42688p_handle, ICM426XX_RA_ACCEL_CONFIG_STATIC3, 36 & 0xFF);
    spi_write_byte(icm42688p_handle, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (36 >> 8) | (10 << 4));


    vTaskDelay(20);
    // Configure gyro and acc UI Filters
    spi_write_byte(icm42688p_handle, ICM426XX_RA_REG_BANK_SEL, ICM426XX_BANK_SELECT0);
    spi_write_byte(icm42688p_handle, ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);

    // Bu çözüm gürültüyü çok az arttırıyor.
    // Fix for stalls in gyro output. See https://github.com/ArduPilot/ardupilot/pull/25332
    // first read the register default value. Only change bit[7:6] from 10 to 01 (reserved bits)

    /*
    uint8_t value = spi_read_byte(icm42688p_handle, ICM426XX_INTF_CONFIG1);
    value &= ~ICM426XX_INTF_CONFIG1_AFSR_MASK;
    value |= ICM426XX_INTF_CONFIG1_AFSR_DISABLE;
    spi_write_byte(icm42688p_handle, ICM426XX_INTF_CONFIG1, value); 
    */

    // Turn gyro acc on
    spi_write_byte(icm42688p_handle, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    vTaskDelay(20);
    spi_write_byte(icm42688p_handle, GYRO_CONFIG0, GY_FS_2000 | GY_ODR_1KHZ);
    vTaskDelay(20);
    spi_write_byte(icm42688p_handle, ACCEL_CONFIG0, AC_FS_16G | AC_ODR_1KHZ);
    vTaskDelay(20);
}


void icm42688p_read(imu_t *imu)
{
    uint8_t data_read[14];
    spi_read_bytes(icm42688p_handle, TEMP_DATA1, data_read, 14);
    parse_icm42688p_data(imu, data_read);
    getCalibratedResults(imu);
}

static void parse_icm42688p_data(imu_t *imu, uint8_t *buff)
{
    static int16_t acc_axis[3] = {0, 0, 0};
    static int16_t gyr_axis[3] = {0, 0, 0};

    imu->temp_mC = ((int16_t)(buff[0] << 8 | buff[1]) / 1.3248f) + 2500;
    acc_axis[X] = (int16_t)(buff[2] << 8 | buff[3]);
    acc_axis[Y] = (int16_t)(buff[4] << 8 | buff[5]);
    acc_axis[Z] = (int16_t)(buff[6] << 8 | buff[7]);

    gyr_axis[X] = (int16_t)(buff[8] << 8 | buff[9]);
    gyr_axis[Y] = (int16_t)(buff[10] << 8 | buff[11]);
    gyr_axis[Z] = (int16_t)(buff[12] << 8 | buff[13]);

    imu->accel_ms2[X] = (acc_axis[ALIGNED_ACC_X_AXIS] * ALIGNED_ACC_X_AXIS_SIGN) * ACC_GAIN;
    imu->accel_ms2[Y] = (acc_axis[ALIGNED_ACC_Y_AXIS] * ALIGNED_ACC_Y_AXIS_SIGN) * ACC_GAIN;
    imu->accel_ms2[Z] = (acc_axis[ALIGNED_ACC_Z_AXIS] * ALIGNED_ACC_Z_AXIS_SIGN) * ACC_GAIN;

    imu->gyro_dps[X] = (gyr_axis[ALIGNED_GYR_X_AXIS] * ALIGNED_GYR_X_AXIS_SIGN) * GYR_GAIN;
    imu->gyro_dps[Y] = (gyr_axis[ALIGNED_GYR_Y_AXIS] * ALIGNED_GYR_Y_AXIS_SIGN) * GYR_GAIN;
    imu->gyro_dps[Z] = (gyr_axis[ALIGNED_GYR_Z_AXIS] * ALIGNED_GYR_Z_AXIS_SIGN) * GYR_GAIN;
}

static void getCalibratedResults(imu_t *imu)
{
    imu->gyro_dps[X] -= imu->gyro_bias_dps[X];
    imu->gyro_dps[Y] -= imu->gyro_bias_dps[Y];
    imu->gyro_dps[Z] -= imu->gyro_bias_dps[Z];

    imu->accel_ms2[X] -= accel_calib_data->offset[X];
    imu->accel_ms2[Y] -= accel_calib_data->offset[Y];
    imu->accel_ms2[Z] -= accel_calib_data->offset[Z];

    imu->accel_ms2[X] *= accel_calib_data->scale[X];
    imu->accel_ms2[Y] *= accel_calib_data->scale[Y];
    imu->accel_ms2[Z] *= accel_calib_data->scale[Z];
}
