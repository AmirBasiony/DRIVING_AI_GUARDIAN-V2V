/*
 * I2C.c
 *
 * Author: Hady Abdelhady
 */

#include "i2c.h"

sint16 i2c_fd; // File descriptor for I2C device

char buffer1[40] = ""; // Initialize buffer for receiving string 1
char buffer2[40] = ""; // Initialize buffer for receiving string 2
char buffer3[40] = ""; // Initialize buffer for receiving string 3
char data[120] = "";   // Initialize buffer for concatenated data
/*
 * @Fn         i2c_receive_byte
 * @brief      Receives a single byte over I2C communication.
 * @retval     The received byte.
 * @note       Reads a single byte from the I2C device, handles errors, and performs additional operations if needed.
 */

// uint8 i2c_receive_byte()
// {
//     uint8 byte;
//     // Read byte from the I2C device
//     if (read(i2c_fd, &byte, 1) != 1)
//     {
//         perror("Error reading from I2C device");
//         close(i2c_fd); // Close the file descriptor before exiting
//         exit(EXIT_FAILURE);
//     }
//     // Additional error handling or operations can be added here
//     return byte;
// }

uint8 i2c_receive_byte(uint8 last) // will remove the parameter
{
#ifdef TESTING
    // for test
    // printf("last (parameter)= %d\n", last);
    printf("receive one byte done successfully\n");
    return 0;
#else
    uint8 buffer;
    if (read(i2c_fd, &buffer, 1) != 1)
    {
        perror("Error reading from I2C device");
        close(i2c_fd); // Close the file descriptor before exiting
        exit(EXIT_FAILURE);
    }
        // printf("last (parameter)= %d\n", last);

    return buffer;
#endif
}

/*
 * @Fn         i2c_send_data
 * @brief      Sends data over I2C communication.
 * @param [in] data: the data to be sent.
 * @note       Writes data to the I2C device and handles errors.
 */

void i2c_send_byte(uint8 data)
{
// #ifdef TESTING
//     printf("Send one byte[%d] done successfully\n", data);
// #else
    // Additional setup for sending data can be added here
    // Write data to the I2C device
    if (write(i2c_fd, &data, 1) != 1)
    {
        perror("Error writing to I2C device");
        close(i2c_fd); // Close the file descriptor before exiting
        exit(EXIT_FAILURE);
    }
// #endif
}

/*
 * @Fn         i2c_init
 * @brief      Initializes the I2C communication and sets the slave address.
 * @param [in] slave_address: The I2C slave address.
 * @note       Opens the I2C device, sets the slave address, and performs additional configurations if needed.
 */
void i2c_init(int slave_address)
{
    // Open the I2C device

    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0)
    {
        perror("Error opening I2C device");
        exit(EXIT_FAILURE);
    }

    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, slave_address) < 0)
    {
        perror("Error setting I2C slave address");
        close(i2c_fd); // Close the file descriptor before exiting
        exit(EXIT_FAILURE);
    }
}

/*
 * @Fn         i2c_send_data
 * @brief      Sends data over I2C communication.
 * @param [in] data: Pointer to the data to be sent.
 * @param [in] len: Length of the data to be sent.
 * @note       Writes data to the I2C device and handles errors.
 */
// Function to send data over I2C communication
// void i2c_send_data(uint8 *data, size_t len)
// {
//     if (write(i2c_fd, data, len) != len) {
//         perror("Error writing to I2C device");
//         exit(EXIT_FAILURE);
//     }
// printf("sent string: %s\n", data);
// }

/*
 * @Fn         i2c_receive_data
 * @brief      Receives data over I2C communication.
 * @param [out] buffer: Pointer to the buffer to store received data.
 * @param [in] len: Length of the data to be received.
 * @retval     Number of bytes read or -1 on error.
 * @note       Reads data from the I2C device, handles errors, and performs additional operations if needed.
 */
// void i2c_receive_data(uint8 *buffer, size_t len)
// {
//     int bytesRead;

//     if ((bytesRead = read(i2c_fd, buffer, sizeof(buffer) - 1)) < 0)
//     {
//         perror("Failed to read string 1 from the I2C bus.");
//         return 1;
//     }
//     buffer[bytesRead] = '\0'; // Null terminate the received data
// Read data from the I2C device
// if (read(i2c_fd, buffer, len) < 0)
// {
//     perror("Error reading from I2C device");
//     close(i2c_fd); // Close the file descriptor before exiting
//     exit(EXIT_FAILURE);
// }
// }

/*
 * @Fn         i2c_close
 * @brief      Closes the I2C communication.
 * @note       Closes the I2C device file descriptor.
 */
void i2c_close()
{
#ifdef TESTING
#else
    // Close the I2C device
    close(i2c_fd);
#endif
}
