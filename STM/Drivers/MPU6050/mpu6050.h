/**
 * @file mpu6050.h
 * @author Amr M. Taha (amr.taha1261@gmail.com)
 * @brief This file contains the declaration of the MPU6050 sensor driver.
 * @version 0.1
 * @date 2024-02-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef MPU6050_H_
#define MPU6050_H_

#include "mpu6050_reg.h"
#include "stm32f4xx_hal.h"

typedef enum
{
    FS_SEL_250 = 0,
    FS_SEL_500 = 1,
    FS_SEL_1000 = 2,
    FS_SEL_2000 = 3
} gyroFullScaleRange_t;

typedef enum
{
    AFS_SEL_2G = 0,
    AFS_SEL_4G = 1,
    AFS_SEL_8G = 2,
    AFS_SEL_16G = 3
} accelFullScaleRange_t;

typedef enum{
    CLK_SEL_INTERNAL_8MHZ = 0,
    CLK_SEL_PLL_X = 1,
    CLK_SEL_PLL_Y = 2,
    CLK_SEL_PLL_Z = 3,
    CLK_SEL_PLL_EXT_32_768KHZ = 4,
    CLK_SEL_PLL_EXT_19_2MHZ = 5,
    CLK_SEL_STOP = 7
}clkSel_t;

typedef enum{
    LP_WAKE_CTRL_1_25HZ = 0,
    LP_WAKE_CTRL_5HZ = 1,
    LP_WAKE_CTRL_20HZ = 2,
    LP_WAKE_CTRL_40HZ = 3
}lpWakeCtrl_t;

typedef struct{
    gyroFullScaleRange_t gyroFullScaleRange;
    accelFullScaleRange_t accelFullScaleRange;
    clkSel_t clkSel;
    uint8_t enableTempSensor;
}MPU6050_Config_t;

typedef struct{
    float x;
    float y;
    float z;
}MPU6050_Accel_t;

typedef struct{
    float x;
    float y;
    float z;
}MPU6050_Gyro_t;

typedef struct{
    MPU6050_Accel_t accel;
    MPU6050_Gyro_t gyro;
    float temp;
}MPU6050_Data_t;


/**
 * @brief initialize the mpu6050
 * 
 * @param i2c pointer to the i2c handler
 */
void mpu6050_init(I2C_HandleTypeDef *i2c);

/**
 * @brief set the mpu6050 configurations
 * 
 * @param config pointer to the configuration struct
 */
void mpu6050_setConfig(MPU6050_Config_t *config);


/**
 * @brief read the mpu6050 data
 * 
 * @param data pointer to the data struct
 */
void mpu6050_readData(MPU6050_Data_t *data);

/**
 * @brief read the mpu6050 accelerometer data
 * 
 * @param accel pointer to the accelerometer struct
 */
void mpu6050_readAccel(MPU6050_Accel_t *accel);

/**
 * @brief read the mpu6050 gyro data
 * 
 * @param gyro pointer to the gyro struct
 */
void mpu6050_readGyro(MPU6050_Gyro_t *gyro);

/**
 * @brief read the mpu6050 temperature data
 * 
 * @param temp pointer to the temperature variable
 */
void mpu6050_readTemp(int16_t *temp);

/**
 * @brief get the accel data in mg
 * 
 * @param accel pointer to the accelerometer struct
 */
void mpu6050_getAccelMG(MPU6050_Accel_t *accel);

/**
 * @brief get the gyro data in dps
 * 
 * @param gyro pointer to the gyro struct
 */
void mpu6050_getGyroDPS(MPU6050_Gyro_t *gyro);

/**
 * @brief get the temperature in celsius
 * 
 * @param temp pointer to the temperature variable
 */
void mpu6050_getTempC(float *temp);


#endif