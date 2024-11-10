#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define MPU6050_ADDRESS     0x68
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

int16_t combineBytes(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

int main() {
    // Open I2C bus
    int file;
    char *bus = "/dev/i2c-1";
    if ((file = open(bus, O_RDWR)) < 0) {
        printf("Failed to open the bus.\n");
        return 1;
    }

    // Set MPU6050 address
    if (ioctl(file, I2C_SLAVE, MPU6050_ADDRESS) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        return 1;
    }

    // Read accelerometer data
    while (1) {
        // Read acceleration values for X and Y axes
        int16_t accel_x, accel_y;
        uint8_t buf[6];

        // Read accelerometer data
        if (read(file, buf, 6) != 6) {
            printf("Error reading accelerometer data.\n");
            return 1;
        }

        // Combine high and low bytes for X and Y axes
        accel_x = combineBytes(buf[0], buf[1]);
        accel_y = combineBytes(buf[2], buf[3]);

        // Print acceleration values
        printf("Acceleration - X: %d, Y: %d\n", accel_x, accel_y);

        // Delay for sampling rate
        usleep(100000); // 100 ms delay, adjust as needed
    }

    // Close I2C bus
    close(file);

    return 0;
}
