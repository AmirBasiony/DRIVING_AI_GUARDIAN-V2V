#include "Thread_3.h"

int fd;
float gyroXcal = 0, gyroYcal = 0, gyroZcal = 0, ax_offset = 0, ay_offset = 0, az_offset = 0;
double ax_samples[WINDOW_SIZE] = {0};
int sample_index = 0;
struct timespec t_start, t_now;
float roll = 0, pitch = 0, yaw = 0; // Rotation in the direction of X,Y,Z respectivly
Bool_t DoneCalibration = False;
double prev_x = 0;
pthread_mutex_t DoneCalibration_mutex;

/**
 * @brief Reads and processes MPU sensor values.
 * @details This function reads data from the MPU sensor and processes IMU values. It handles both testing and non-testing scenarios.
 *
 */
void MPU_Readings(void)
{
#ifdef TESTING
    pthread_mutex_lock(&DoneCalibration_mutex);
    sleep(5);
    acceleration = 50;
    yaw = 300;
    DoneCalibration = True;
    pthread_mutex_unlock(&DoneCalibration_mutex);

#else

    // pthread_mutex_lock(&DoneGettintInitials_viaUART_mutex);
    // while (True)
    // {
    pthread_mutex_lock(&DoneCalibration_mutex);
    char *bus = "/dev/i2c-1";
    if ((fd = open(bus, O_RDWR)) < 0)
    {
        perror("Failed to open the bus.");
    }
    ioctl(fd, I2C_SLAVE, MPU_ADDR);
    setup_mpu();
    calibrate_sensors();
    // _UART_Receive_StartFlag();
    DoneCalibration = True;
    pthread_mutex_unlock(&DoneCalibration_mutex);
    while (True)
    {
        pthread_mutex_lock(&yaw_mutex);
        pthread_mutex_lock(&MainVehicle_mutex);
        process_imu_values(&roll, &pitch, &yaw);
        MainVehicle.Accelration = moving_average(read_accel(ACCEL_XOUT_H, &prev_x), ax_samples, WINDOW_SIZE);
        // printf("YAW : %.2f , ACC : %d ", yaw, acceleration);
        pthread_mutex_unlock(&yaw_mutex);
        pthread_mutex_unlock(&MainVehicle_mutex);

        usleep(5000);
    }

    close(fd);
#endif
    // }
    // pthread_mutex_unlock(&DoneGettintInitials_viaUART_mutex);
    exit(EXIT_SUCCESS);
}
/**
 * @brief Initializes the MPU by writing default values to the power management and configuration registers.
 */
void setup_mpu()
{
    i2c_write_byte(POWER_MGMT_1, 0);
    i2c_write_byte(GYRO_CONFIG, 0);
    i2c_write_byte(ACC_CONFIG, 0);
}
/**
 * @brief Calibrates the gyroscope and accelerometer by averaging multiple samples.
 */
void calibrate_sensors()
{
    printf("Calibrating...\n");
    int samples = 1000;
    for (int i = 0; i < samples; i++)
    {
        gyroXcal += i2c_read_word(GYRO_XOUT_H);
        gyroYcal += i2c_read_word(GYRO_XOUT_H + 2);
        gyroZcal += i2c_read_word(GYRO_XOUT_H + 4);
        ax_offset += i2c_read_word(ACCEL_XOUT_H);
        ay_offset += i2c_read_word(ACCEL_YOUT_H);
        az_offset += i2c_read_word(ACCEL_YOUT_H + 2);
    }
    gyroXcal /= samples;
    gyroYcal /= samples;
    gyroZcal /= samples;
    ax_offset /= samples;
    ay_offset /= samples;
    az_offset /= samples;
    printf("Calibration complete.\n");
}

/**
 * @brief Processes IMU values to update roll, pitch, and yaw.
 *
 * @param roll Pointer to the roll value.
 * @param pitch Pointer to the pitch value.
 * @param yaw Pointer to the yaw value.
 */
void process_imu_values(float *roll, float *pitch, float *yaw)
{
    int gx = i2c_read_word(GYRO_XOUT_H) - (int)gyroXcal;
    int gy = i2c_read_word(GYRO_XOUT_H + 2) - (int)gyroYcal;
    int gz = i2c_read_word(GYRO_XOUT_H + 4) - (int)gyroZcal;

    gx /= GYRO_SCALE;
    gy /= GYRO_SCALE;
    gz /= GYRO_SCALE;

    // Update timestamps and calculate dt
    clock_gettime(CLOCK_REALTIME, &t_now);
    double dt = (t_now.tv_sec - t_start.tv_sec) + (t_now.tv_nsec - t_start.tv_nsec) / 1000000000.0;
    t_start = t_now;

    // Complementary filter for roll and pitch
    float accPitch = atan2(gy, gz) * 180 / M_PI;
    float accRoll = atan2(gx, gz) * 180 / M_PI;

    *yaw += gz * dt;
    *roll = TAU * (*roll - gy * dt) + (1 - TAU) * accRoll;
    *pitch = TAU * (*pitch + gx * dt) + (1 - TAU) * accPitch;

    *yaw = normalize_angle(*yaw);
    *roll = normalize_angle(*roll);
    *pitch = normalize_angle(*pitch);
}

/**
 * @brief Reads and filters accelerometer data.
 *
 * @param reg The register address to read from.
 * @param prev_filtered Pointer to the previously filtered value.
 * @return double The filtered accelerometer value.
 */
double read_accel(int reg, double *prev_filtered)
{
    int raw = i2c_read_word(reg) - (reg == ACCEL_XOUT_H ? (int)ax_offset : (int)ay_offset);
    double g_force = raw / ACC_SCALE;
    double accel = g_force * GRAVITY * 100.0;
    double filtered = *prev_filtered + ALPHA * (accel - *prev_filtered);
    *prev_filtered = filtered;
    return filtered;
}

/**
 * @brief Writes a byte to a specific I2C register.
 *
 * @param reg The register address to write to.
 * @param value The byte value to write.
 */
void i2c_write_byte(int reg, int value)
{
    char buf[2] = {reg, value};
    write(fd, buf, 2);
}

/**
 * @brief Reads a word (2 bytes) from a specific I2C register.
 *
 * @param reg The register address to read from.
 * @return int The read word, converted to a signed integer.
 */
int i2c_read_word(int reg)
{
    char buf[2] = {0};
    char reg_buf = reg;
    write(fd, &reg_buf, 1);
    read(fd, buf, 2);
    int val = (buf[0] << 8) | buf[1];
    return (val >= 0x8000) ? -((65535 - val) + 1) : val;
}

/**
 * @brief Calculates the moving average of the given samples.
 *
 * @param new_sample The new sample to add to the moving average.
 * @param samples The array holding the past samples.
 * @param size The size of the samples array.
 * @return double The updated moving average.
 */
double moving_average(double new_sample, double *samples, int size)
{
    samples[sample_index % size] = new_sample;
    sample_index++;
    double sum = 0;
    for (int i = 0; i < size; i++)
    {
        sum += samples[i];
    }
    return sum / size;
}

/**
 * @brief Normalizes an angle to the range [0, 360] degrees.
 *
 * @param angle The angle to normalize.
 * @return float The normalized angle.
 */
float normalize_angle(float angle)
{
    float new_angle = fmod(angle, 360.0);
    if (new_angle < 0)
    {
        new_angle += 360.0;
    }
    return new_angle;
}
