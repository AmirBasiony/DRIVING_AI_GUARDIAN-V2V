#include "HAL_mpu_6050.h"

// Kalman filter parameters
float Q = 1e-5;  // Process noise covariance
float R = 0.1;    // Measurement noise covariance
float x[3] = {0}; // Initial state (position)
float P[3] = {1, 1, 1}; // Initial covariance

// Measurement function: linear mapping from state to measurement
float H = 1;

// Kalman filter function
void kalman_filter(float* z, int len) {
    for (int i = 0; i < len; i++) {
        // Prediction step
        x[i] = x[i];      // State prediction (no change in this example)
        P[i] += Q;        // Covariance prediction

        // Measurement update step
        float K = P[i] / (P[i] + R); // Kalman gain
        x[i] += K * (z[i] - x[i]);   // State update
        P[i] *= (1 - K);             // Covariance update
    }
}

// Define the filter function with Kalman filter integration
float* kalman_filter_func(float *accel, float *gyro, float *previous_data, u8 len) 
{
    // Apply Kalman filter to the accelerometer data
    kalman_filter(accel, len);

    // Update previous_data with the filtered accelerometer values
    for (int i = 0; i < len; i++) {
        previous_data[i] = x[i];
    }

    // Return the filtered angles
    return previous_data;
}

int main () {
    // Allocate memory for prev_angles array
    float prev_angles[3] = {0.0, 0.0, 0.0};
    
    while (1) {
        i2c_init(MPU6050_Slave_Address);
        MPU6050_Init();
        // printf("\ntesting MPU6050_getFilteredAngles \n----------------------------------\n");
        // float *AnglesData = MPU6050_getFilteredAngles(prev_angles, kalman_filter_func);
        // int angleSize = sizeof(*AnglesData);
        // for (int i = 0; i < angleSize; i++)
        // {
        //     printf("%f : ", AnglesData[i]);
        // }
        printf("\n----------------------------------\ntesting MPU6050_getAccel \n");
        s32* accel = MPU6050_getAccel();
        int accelSize = sizeof(*accel);
        for (int i = 0; i < accelSize; i++)
        {
            printf("%d : ", accel[i]);
        }
        sleep(1);
    }
    return 0;
}
