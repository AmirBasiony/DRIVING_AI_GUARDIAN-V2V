/**
 * @file mpu6050_reg.h
 * @author Amr M. Taha (amr.taha1261@gmail.com)
 * @brief This file contains the register definitions for the MPU6050 sensor.
 * @version 0.1
 * @date 2024-02-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef MPU6050_REG_H_
#define MPU6050_REG_H_
#include <stdint.h>

#define SMPLRT_DIV_I2C_ADDR 0x19
#define CONFIG_I2C_ADDR 0x1A

#define GYRO_CONFIG_I2C_ADDR 0x1B
typedef struct
{
    uint8_t reserved : 3;
    uint8_t fs_sel : 2;
    uint8_t zg_st : 1;
    uint8_t yg_st : 1;
    uint8_t xg_st : 1;
} gyroConfig_reg_t;



#define ACCEL_CONFIG_I2C_ADDR 0x1C
typedef struct
{
    uint8_t reserved : 3;
    uint8_t afs_sel : 2;
    uint8_t za_st : 1;
    uint8_t ya_st : 1;
    uint8_t xa_st : 1;
} accelConfig_reg_t;



#define INT_PIN_CFG_I2C_ADDR    0x37
typedef struct{
    uint8_t reserved:1;
    uint8_t i2c_bypass_en:1;    // 0: I2C interface access
    uint8_t fsync_int_en:1;
    uint8_t fsync_int_level:1;
    uint8_t int_rd_clear:1;
    uint8_t latch_int_en:1;
    uint8_t int_open:1;
    uint8_t int_level:1;
}intPinConfig_reg_t;

#define INT_ENABLE_I2C_ADDR 0x38
typedef struct {
    uint8_t data_rdy_en:1;
    uint8_t reserved1:2;
    uint8_t i2c_mst_int_en:1;
    uint8_t fifo_oflow_en:1;
    uint8_t reserved2:3;
}intEnable_reg_t;

#define INT_STATUS_I2C_ADDR 0x3A
typedef struct {
    uint8_t data_rdy_int:1;
    uint8_t reserved1:2;
    uint8_t i2c_mst_int:1;
    uint8_t fifo_oflow_int:1;
    uint8_t reserved2:3;
}intStatus_reg_t;

#define ACCEL_XOUT_H_I2C_ADDR 0x3B
#define ACCEL_XOUT_L_I2C_ADDR 0x3C
#define ACCEL_YOUT_H_I2C_ADDR 0x3D
#define ACCEL_YOUT_L_I2C_ADDR 0x3E
#define ACCEL_ZOUT_H_I2C_ADDR 0x3F
#define ACCEL_ZOUT_L_I2C_ADDR 0x40
#define TEMP_OUT_H_I2C_ADDR 0x41
#define TEMP_OUT_L_I2C_ADDR 0x42
#define GYRO_XOUT_H_I2C_ADDR 0x43
#define GYRO_XOUT_L_I2C_ADDR 0x44
#define GYRO_YOUT_H_I2C_ADDR 0x45
#define GYRO_YOUT_L_I2C_ADDR 0x46
#define GYRO_ZOUT_H_I2C_ADDR 0x47
#define GYRO_ZOUT_L_I2C_ADDR 0x48

typedef union{
    uint8_t raw[6];
    int16_t raw16[3];
}Mpu6050_AccelData_t;

typedef union{
    uint8_t raw[2];
    int16_t raw16;
}Mpu6050_TempData_t;

typedef union{
    uint8_t raw[6];
    int16_t raw16[3];
}Mpu6050_GyroData_t;

#define SIGNAL_PATH_RESET_I2C_ADDR 0x68
typedef struct{
    uint8_t temp_reset:1;
    uint8_t accel_reset:1;
    uint8_t gyro_reset:1;
    uint8_t reserved:5;
}signalPathReset_reg_t;

#define USER_CTRL_I2C_ADDR 0x6A
typedef struct{
    uint8_t sig_cond_reset:1;
    uint8_t i2c_mst_reset:1;
    uint8_t fifo_reset:1;
    uint8_t reserved:1;
    uint8_t i2c_if_dis:1;
    uint8_t i2c_mst_en:1;
    uint8_t fifo_en:1;
    uint8_t reserved2:1;
}userCtrl_reg_t;

#define PWR_MGMT_1_I2C_ADDR 0x6B
typedef struct{
    uint8_t clk_sel:3;
    uint8_t temp_dis:1;
    uint8_t reserved:1;
    uint8_t cycle:1;
    uint8_t sleep:1;
    uint8_t reset:1;
}pwrMgmt1_reg_t;

#define PWR_MGMT_2_I2C_ADDR 0x6C
typedef struct{
    uint8_t STBY_ZG:1;
    uint8_t STBY_YG:1;
    uint8_t STBY_XG:1;
    uint8_t STBY_ZA:1;
    uint8_t STBY_YA:1;
    uint8_t STBY_XA:1;
    uint8_t lp_wake_ctrl:2;
}pwrMgmt2_reg_t;

#define WHO_AM_I_I2C_ADDR 0x75
typedef struct{
    uint8_t reserved1:1;
    uint8_t who_am_i:6;
    uint8_t reserved2:1;
}whoAmI_reg_t;


#endif