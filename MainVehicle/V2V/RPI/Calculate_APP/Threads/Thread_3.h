#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "../connections/UART_Struct.h"
#include "../ipcMsg/msgipc.h"

#define MPU_ADDR 0x68
#define GYRO_CONFIG 0x1B
#define ACC_CONFIG 0x1C
#define POWER_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define GRAVITY 9.81
#define GYRO_SCALE 131.0
#define ACC_SCALE 16384.0
#define ALPHA 0.1
#define TAU 0.98
#define WINDOW_SIZE 10 // Window size for the moving average filter

extern int fd;
extern float gyroXcal, gyroYcal, gyroZcal, ax_offset, ay_offset, az_offset;
extern double ax_samples[], ay_samples[];
extern int sample_index;
extern struct timespec t_start, t_now;
extern float roll, pitch, yaw;
extern Bool_t DoneGettintInitials_viaUART;
void setup_mpu();
void calibrate_sensors();
void process_imu_values(float *roll, float *pitch, float *yaw);
float normalize_angle(float angle);
double read_accel(int reg, double *prev_filtered);
void i2c_write_byte(int reg, int value);
int i2c_read_word(int reg);
double moving_average(double new_sample, double *samples, int size);
void MPU_Readings(void);
#endif /* IMU_SENSOR_H */
