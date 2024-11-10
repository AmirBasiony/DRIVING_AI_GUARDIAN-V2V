/**
 * @file mpu6050.c
 * @author Amr M. Taha (amr.taha1261@gmail.com)
 * @brief  This file contains the implementation of the MPU6050 sensor driver.
 *          It provides functions for initializing the sensor, reading accelerometer
 *          and gyroscope data, and configuring sensor settings.
 * @version 0.1
 * @version 0.1
 * @date 2024-02-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "mpu6050.h"

#define MPU6050_ADDRESS (0x68 << 1)

static I2C_HandleTypeDef *mpu6050_i2c;
static gyroFullScaleRange_t MPU6050_gyroFullScaleRange;
static accelFullScaleRange_t MPU6050_accelFullScaleRange;


static void MPU6050_writeReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t sendbuf[2] = {reg, *data};
    HAL_I2C_Master_Transmit(mpu6050_i2c, MPU6050_ADDRESS | 0, sendbuf, 2, HAL_MAX_DELAY);
}

static void MPU6050_readReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (HAL_I2C_Master_Transmit(mpu6050_i2c, MPU6050_ADDRESS | 0, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return;
    }
    HAL_I2C_Master_Receive(mpu6050_i2c, MPU6050_ADDRESS | 1, data, len, HAL_MAX_DELAY);
}

/**
 * @brief initialize the mpu6050
 *
 * @param i2c pointer to the i2c handler
 */
void mpu6050_init(I2C_HandleTypeDef *i2c)
{
    mpu6050_i2c = i2c;

    HAL_I2C_IsDeviceReady(mpu6050_i2c, MPU6050_ADDRESS, 3, HAL_MAX_DELAY);
    // set the sample rate to 1khz
    uint8_t config_reg;
    MPU6050_readReg(CONFIG_I2C_ADDR, &config_reg, 1);
    config_reg &= 0b11111000;
    config_reg |= 0x01;
    MPU6050_writeReg(CONFIG_I2C_ADDR, &config_reg, 1);
    // set the gyro range to 250 degrees per second
    gyroConfig_reg_t gyroConfig;
    MPU6050_readReg(GYRO_CONFIG_I2C_ADDR, (uint8_t *)&gyroConfig, 1);
    MPU6050_gyroFullScaleRange = FS_SEL_250;
    gyroConfig.fs_sel = (uint8_t)MPU6050_gyroFullScaleRange;
    MPU6050_writeReg(GYRO_CONFIG_I2C_ADDR, (uint8_t *)&gyroConfig, 1);
    // set the accelerometer range to 2g
    accelConfig_reg_t accelConfig;
    MPU6050_readReg(ACCEL_CONFIG_I2C_ADDR, (uint8_t *)&accelConfig, 1);
    MPU6050_accelFullScaleRange = AFS_SEL_2G;
    accelConfig.afs_sel = (uint8_t)MPU6050_accelFullScaleRange;
    MPU6050_writeReg(ACCEL_CONFIG_I2C_ADDR, (uint8_t *)&accelConfig, 1);
    // disable interrupts & enable I2c bypass
    intPinConfig_reg_t intPinConfig;
    MPU6050_readReg(GYRO_CONFIG_I2C_ADDR, (uint8_t *)&intPinConfig, 1);
    intPinConfig.i2c_bypass_en = 1;
    intPinConfig.fsync_int_en = 0;
    intPinConfig.latch_int_en = 0;
    MPU6050_writeReg(GYRO_CONFIG_I2C_ADDR, (uint8_t *)&intPinConfig, 1);
    intEnable_reg_t intEnable;
    MPU6050_readReg(INT_ENABLE_I2C_ADDR, (uint8_t *)&intEnable, 1);
    intEnable.data_rdy_en = 0;
    intEnable.fifo_oflow_en = 0;
    intEnable.i2c_mst_int_en = 0;
    MPU6050_writeReg(INT_ENABLE_I2C_ADDR, (uint8_t *)&intEnable, 1);
    // enable i2c interface, disable i2c master mode, disable fifo
    userCtrl_reg_t userCtrl;
    MPU6050_readReg(USER_CTRL_I2C_ADDR, (uint8_t *)&userCtrl, 1);
    userCtrl.fifo_en = 0;
    userCtrl.i2c_if_dis = 0;
    userCtrl.i2c_mst_en = 0;
    MPU6050_writeReg(USER_CTRL_I2C_ADDR, (uint8_t *)&userCtrl, 1);
    // disable sleep mode, disable cycle, enable temp, clock select as 8MHz
    pwrMgmt1_reg_t pwr1;
    MPU6050_readReg(PWR_MGMT_1_I2C_ADDR, (uint8_t *)&pwr1, 1);
    pwr1.clk_sel = CLK_SEL_INTERNAL_8MHZ;
    pwr1.temp_dis = 0;
    pwr1.cycle = 0;
    pwr1.sleep = 0;
    MPU6050_writeReg(PWR_MGMT_1_I2C_ADDR, (uint8_t *)&pwr1, 1);
}

/**
 * @brief set the mpu6050 configurations
 *
 * @param config pointer to the configuration struct
 */
void mpu6050_setConfig(MPU6050_Config_t *config)
{
    gyroConfig_reg_t gyroConfig;
    accelConfig_reg_t accelConfig;
    pwrMgmt1_reg_t pwrMgmt1;
    pwrMgmt2_reg_t pwrMgmt2;
    userCtrl_reg_t userCtrl;
    // edit the scale of gyroscope
    MPU6050_readReg(GYRO_CONFIG_I2C_ADDR, (uint8_t *)&gyroConfig, 1);
    MPU6050_gyroFullScaleRange = config->gyroFullScaleRange;
    gyroConfig.fs_sel = (uint8_t)MPU6050_gyroFullScaleRange;
    MPU6050_writeReg(GYRO_CONFIG_I2C_ADDR, (uint8_t *)&gyroConfig, 1);
    // edit the scale of accel data
    MPU6050_readReg(ACCEL_CONFIG_I2C_ADDR, (uint8_t *)&accelConfig, 1);
    MPU6050_accelFullScaleRange = config->accelFullScaleRange;
    accelConfig.afs_sel = (uint8_t)MPU6050_accelFullScaleRange;
    MPU6050_writeReg(ACCEL_CONFIG_I2C_ADDR, (uint8_t *)&accelConfig, 1);
    // edit the data clock
    MPU6050_readReg(PWR_MGMT_1_I2C_ADDR, (uint8_t *)&pwrMgmt1, 1);
    pwrMgmt1.clk_sel = config->clkSel;
    // change temp sensor state
    pwrMgmt1.temp_dis = !(config->enableTempSensor);
    MPU6050_writeReg(PWR_MGMT_1_I2C_ADDR, (uint8_t *)&pwrMgmt1, 1);
    
}

/**
 * @brief read the mpu6050 data
 *
 * @param data pointer to the data struct
 */
void mpu6050_readData(MPU6050_Data_t *data){
    mpu6050_readAccel(&data->accel);
    mpu6050_readGyro(&data->gyro);
    int16_t temp;
    mpu6050_readTemp(&temp);
    data->temp = (float)temp;
}

/**
 * @brief read the mpu6050 accelerometer data
 *
 * @param accel pointer to the accelerometer struct
 */
void mpu6050_readAccel(MPU6050_Accel_t *accel){
    Mpu6050_AccelData_t accelData;
    
    // read the lower & higher register of accelX
    MPU6050_readReg(ACCEL_XOUT_L_I2C_ADDR, &accelData.raw[0], 1);
    MPU6050_readReg(ACCEL_XOUT_H_I2C_ADDR, &accelData.raw[1], 1);
    // read the lower & higher register of accelY
    MPU6050_readReg(ACCEL_YOUT_L_I2C_ADDR, &accelData.raw[2], 1);
    MPU6050_readReg(ACCEL_YOUT_H_I2C_ADDR, &accelData.raw[3], 1);
    // read the lower & higher register of accelZ
    MPU6050_readReg(ACCEL_ZOUT_L_I2C_ADDR, &accelData.raw[4], 1);
    MPU6050_readReg(ACCEL_ZOUT_H_I2C_ADDR, &accelData.raw[5], 1);

    // assigning results
    accel->x = accelData.raw16[0];
    accel->y = accelData.raw16[1];
    accel->z = accelData.raw16[2];
    
}

/**
 * @brief read the mpu6050 gyro data
 *
 * @param gyro pointer to the gyro struct
 */
void mpu6050_readGyro(MPU6050_Gyro_t *gyro){
    Mpu6050_GyroData_t gyroData;

    // read the lower & higher register of accelX
    MPU6050_readReg(GYRO_XOUT_L_I2C_ADDR, &gyroData.raw[0], 1);
    MPU6050_readReg(GYRO_XOUT_H_I2C_ADDR, &gyroData.raw[1], 1);
    // read the lower & higher register of accelY
    MPU6050_readReg(GYRO_YOUT_L_I2C_ADDR, &gyroData.raw[2], 1);
    MPU6050_readReg(GYRO_YOUT_H_I2C_ADDR, &gyroData.raw[3], 1);
    // read the lower & higher register of accelZ
    MPU6050_readReg(GYRO_ZOUT_L_I2C_ADDR, &gyroData.raw[4], 1);
    MPU6050_readReg(GYRO_ZOUT_H_I2C_ADDR, &gyroData.raw[5], 1);

    // assigning results
    gyro->x = gyroData.raw16[0];
    gyro->y = gyroData.raw16[1];
    gyro->z = gyroData.raw16[2];
}

/**
 * @brief read the mpu6050 temperature data
 *
 * @param temp pointer to the temperature variable
 */
void mpu6050_readTemp(int16_t *temp){
    Mpu6050_TempData_t tempData;

    // read the lower & higher byte of temp data
    MPU6050_readReg(TEMP_OUT_L_I2C_ADDR, &tempData.raw[0], 1);
    MPU6050_readReg(TEMP_OUT_H_I2C_ADDR, &tempData.raw[1], 1);

    // assign results
    *temp = tempData.raw16;
}

/**
 * @brief get the accel data in mg
 * 
 * @param accel pointer to the accelerometer struct
 */
void mpu6050_getAccelMG(MPU6050_Accel_t *accel){
    mpu6050_readAccel(accel);
    switch (MPU6050_accelFullScaleRange)
    {
    case AFS_SEL_2G:
        accel->x = (accel->x / 16384.0) * 1000;
        accel->y = (accel->y / 16384.0) * 1000;
        accel->z = (accel->z / 16384.0) * 1000;
        break;
    case AFS_SEL_4G:
        accel->x = (accel->x / 8192.0) * 1000;
        accel->y = (accel->y / 8192.0) * 1000;
        accel->z = (accel->z / 8192.0) * 1000;
        break;
    case AFS_SEL_8G:
        accel->x = (accel->x / 4096.0) * 1000;
        accel->y = (accel->y / 4096.0) * 1000;
        accel->z = (accel->z / 4096.0) * 1000;
        break;
    case AFS_SEL_16G:
        accel->x = (accel->x / 2048.0) * 1000;
        accel->y = (accel->y / 2048.0) * 1000;
        accel->z = (accel->z / 2048.0) * 1000;
        break;
    default:
        // 2g
        accel->x = (accel->x / 16384.0) * 1000;
        accel->y = (accel->y / 16384.0) * 1000;
        accel->z = (accel->z / 16384.0) * 1000;
        break;
    }
}

/**
 * @brief get the gyro data in dps
 * 
 * @param gyro pointer to the gyro struct
 */
void mpu6050_getGyroDPS(MPU6050_Gyro_t *gyro){
    mpu6050_readGyro(gyro);
    switch (MPU6050_gyroFullScaleRange)
    {
    case FS_SEL_250:
        gyro->x = gyro->x / 131.0;
        gyro->y = gyro->y / 131.0;
        gyro->z = gyro->z / 131.0;
        break;
    case FS_SEL_500:
        gyro->x = gyro->x / 65.5;
        gyro->y = gyro->y / 65.5;
        gyro->z = gyro->z / 65.5;
        break;
    case FS_SEL_1000:
        gyro->x = gyro->x / 32.8;
        gyro->y = gyro->y / 32.8;
        gyro->z = gyro->z / 32.8;
        break;
    case FS_SEL_2000:
        gyro->x = gyro->x / 16.4;
        gyro->y = gyro->y / 16.4;
        gyro->z = gyro->z / 16.4;
        break;
    default:
        // 250 dps
        gyro->x = gyro->x / 131.0;
        gyro->y = gyro->y / 131.0;
        gyro->z = gyro->z / 131.0;
        break;
    }

}

/**
 * @brief get the temperature in celsius
 * 
 * @param temp pointer to the temperature variable
 */
void mpu6050_getTempC(float *temp){
    int16_t tempData;
    mpu6050_readTemp(&tempData);
    *temp = ((float)tempData / 340.0) + 36.53;
}
