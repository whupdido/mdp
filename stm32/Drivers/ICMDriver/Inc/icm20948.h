#ifndef ICM20948_H
#define ICM20948_H

#include "main.h"

#define ICM20948_I2C_ADDR       (0x68 << 1) /* 0xD0 */

/* Bank 0 Registers */
#define REG_BANK_SEL            0x7F
#define REG_WHO_AM_I            0x00
#define REG_USER_CTRL           0x03
#define REG_PWR_MGMT_1          0x06
#define REG_PWR_MGMT_2          0x07
#define REG_GYRO_ZOUT_H         0x37
#define REG_GYRO_ZOUT_L         0x38

/* Sensitivity at default +/- 250 dps */
#define GYRO_SCALE_FACTOR       131.0f

uint8_t icm20948_init(I2C_HandleTypeDef *hi2c);
void    icm20948_calib_gyro_bias(void);
float   icm20948_read_gyro_z(void);

extern float gyro_z_bias;

#endif /* ICM20948_H */
