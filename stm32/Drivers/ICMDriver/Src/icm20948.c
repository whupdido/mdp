#include "icm20948.h"
#include <math.h>

static I2C_HandleTypeDef *icm_i2c;
float gyro_z_bias = 0.0f;

static void select_user_bank(uint8_t bank)
{
    uint8_t reg[2] = {REG_BANK_SEL, (uint8_t)(bank << 4)};
    HAL_I2C_Master_Transmit(icm_i2c, ICM20948_I2C_ADDR, reg, 2, 50);
}

uint8_t icm20948_init(I2C_HandleTypeDef *hi2c)
{
    icm_i2c = hi2c;
    uint8_t who_am_i = 0;
    uint8_t cmd[2];

    select_user_bank(0);
    HAL_I2C_Mem_Read(icm_i2c, ICM20948_I2C_ADDR, REG_WHO_AM_I, 1, &who_am_i, 1, 50);

    /* ICM-20948 WHO_AM_I default is 0xEA */
    if (who_am_i != 0xEA) {
        return 0; /* Device not detected */
    }

    /* Reset device */
    cmd[0] = REG_PWR_MGMT_1; cmd[1] = 0x80;
    HAL_I2C_Master_Transmit(icm_i2c, ICM20948_I2C_ADDR, cmd, 2, 50);
    HAL_Delay(50);

    /* Wake up & auto-select best clock source */
    cmd[0] = REG_PWR_MGMT_1; cmd[1] = 0x01;
    HAL_I2C_Master_Transmit(icm_i2c, ICM20948_I2C_ADDR, cmd, 2, 50);
    HAL_Delay(10);

    /* Enable Gyro and Accel */
    cmd[0] = REG_PWR_MGMT_2; cmd[1] = 0x00;
    HAL_I2C_Master_Transmit(icm_i2c, ICM20948_I2C_ADDR, cmd, 2, 50);

    return 1;
}

void icm20948_calib_gyro_bias(void)
{
    float sum = 0.0f;
    const int samples = 500;

    for (int i = 0; i < samples; i++) {
        sum += icm20948_read_gyro_z();
        HAL_Delay(2);
    }
    gyro_z_bias = sum / (float)samples;
}

float icm20948_read_gyro_z(void)
{
    uint8_t data[2];
    select_user_bank(0);
    if (HAL_I2C_Mem_Read(icm_i2c, ICM20948_I2C_ADDR, REG_GYRO_ZOUT_H, 1, data, 2, 20) == HAL_OK) {
        int16_t raw = (int16_t)((data[0] << 8) | data[1]);
        return ((float)raw / GYRO_SCALE_FACTOR) - gyro_z_bias;
    }
    return 0.0f;
}
