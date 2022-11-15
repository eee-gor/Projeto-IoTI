#include <stdio.h>
#include "driver/i2c.h"

#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

#define MPU6050_ACCEL_FULL_SCALE_2G 0x00
#define MPU6050_ACCEL_FULL_SCALE_4G 0x08
#define MPU6050_ACCEL_FULL_SCALE_8G 0x10
#define MPU6050_ACCEL_FULL_SCALE_16G 0x18

#define MPU6050_ACCEL_LSB_SENS_2G 16384
#define MPU6050_ACCEL_LSB_SENS_4G 8192
#define MPU6050_ACCEL_LSB_SENS_8G 4096
#define MPU6050_ACCEL_LSB_SENS_16G 2048

#define MPU6050_I2C_MASTER_FREQ_HZ 400000

void task_mpu6050(void *ignore);
/**
 * @brief Lê dados do acelerômetro;
 */