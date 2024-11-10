// /*
//     [NOTE]: Maximum number of charachters at sending via I2C is 34(BYTES)
// */


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>


// #define SEND_DATA
#define ARDUINO_I2C_ADDRESS 9

typedef enum DIRECTION
{
    VERTICAL,
    HORIZONTAL,
    NORTH,
    SOUTH,
    EAST,
    WEST
} E_Direction_t;

typedef enum MSG
{
    SLOWDOWN, // 0
    FASTER,   // 1
    SAFE,     // 2
} E_MSG_t;

typedef struct
{
    uint8_t ID;
    int16_t x;
    int16_t y;
    int16_t Acceleration;
    int16_t Angle;
    int16_t Velocity;
    E_Direction_t Direction;
    E_MSG_t msg;
} S_Connected_Car_t;

typedef struct
{
    int16_t SideDistance;
    int16_t FrontRearDistance;
    int16_t safe_distance;
} S_SUDDENBRAKE_Data_t;

typedef struct
{
    int16_t My_Dist;
    int16_t other_Dist;
    int16_t Mytime;
    int16_t Othertime;
    int16_t Current_Dist;
    int16_t point_X;
    int16_t point_Y;
} S_INTERSECTION_Data_t;

typedef struct
{
    uint8_t action;
    S_Connected_Car_t CAR;
    S_INTERSECTION_Data_t Intersection;
    S_SUDDENBRAKE_Data_t SuddenBrake;
} S_DATA_t;

// Define and initialize the global instance
S_DATA_t data = {
    /* action */ 3,
    /* CAR */ {1, 50.123456, 31.402555, 25, 150, 200, HORIZONTAL, SAFE},
    /* Intersection */ {50, 30, 200, 130, 75, 50.123456, 31.402555},
    /* SuddenBrake */ {7, 15, 20}
    };
S_DATA_t Car_Info;
int i2c_fd; // File descriptor for I2C device

// Function to send data over I2C communication
void i2c_send_data(int i2c_fd, uint8_t *data, size_t len) 
{
    if (write(i2c_fd, data, len) != len) {
        perror("Error writing to I2C device");
        close(i2c_fd); // Close the file descriptor before exiting
        exit(EXIT_FAILURE);
    }
    // printf("sent string: %s\n", data);
}
// Function to receive data over I2C communication
void i2c_receive_data(int i2c_fd, uint8_t *data, size_t len) {
    if (read(i2c_fd, data, len) != (ssize_t)len) {
        perror("Error reading from I2C device");
        close(i2c_fd);
        exit(EXIT_FAILURE);
    }
    printf("received string: %s\n", data);
}
// Function to concatenate strings into a single buffer
void concatenateStrings(char *destination, const char *source)
{
    strcat(destination, source);
}
void struct_to_string3(const S_DATA_t *data, char* buffer1,char* buffer2,char* buffer3) //
{
       sprintf(buffer1, "%u,%d,%d,%d,%d,%d,%d,%d,%d,",
        data->action,
        data->CAR.ID, data->CAR.x, data->CAR.y,
        data->CAR.Acceleration, data->CAR.Angle,
        data->CAR.Velocity, data->CAR.Direction,
        data->CAR.msg);

        sprintf(buffer2, "%d,%d,%d,%d,%d,%d,%d,",
        data->Intersection.My_Dist, data->Intersection.other_Dist,
        data->Intersection.Mytime, data->Intersection.Othertime,
        data->Intersection.Current_Dist, data->Intersection.point_X,
        data->Intersection.point_Y);

        sprintf(buffer3, "%d,%d,%d",
        data->SuddenBrake.SideDistance, data->SuddenBrake.FrontRearDistance,
        data->SuddenBrake.safe_distance);
}

void printData(const S_DATA_t *data) 
{
    printf("Action: %d\n", data->action);
    printf("Car ID: %d\n", data->CAR.ID);
    printf("Car x: %d\n", data->CAR.x);
    printf("Car y: %d\n", data->CAR.y);
    printf("Car Acceleration: %d\n", data->CAR.Acceleration);
    printf("Car Angle: %d\n", data->CAR.Angle);
    printf("Car Velocity: %d\n", data->CAR.Velocity);
    printf("Car Direction: %d\n", data->CAR.Direction);
    printf("Car Message: %d\n", data->CAR.msg);
    printf("Intersection My_Dist: %d\n", data->Intersection.My_Dist);
    printf("Intersection other_Dist: %d\n", data->Intersection.other_Dist);
    printf("Intersection Mytime: %d\n", data->Intersection.Mytime);
    printf("Intersection Othertime: %d\n", data->Intersection.Othertime);
    printf("Intersection Current_Dist: %d\n", data->Intersection.Current_Dist);
    printf("Intersection point_X: %d\n", data->Intersection.point_X);
    printf("Intersection point_Y: %d\n", data->Intersection.point_Y);
    printf("SuddenBrake SideDistance: %d\n", data->SuddenBrake.SideDistance);
    printf("SuddenBrake FrontRearDistance: %d\n", data->SuddenBrake.FrontRearDistance);
    printf("SuddenBrake safe_distance: %d\n", data->SuddenBrake.safe_distance);
}

// Function to parse received string and populate struct
void parseReceivedData(const char *receivedData, S_DATA_t *data)
{
    sscanf(receivedData, "%hhu,%hhu,%hd,%hd,%hd,%hd,%hd,%u,%u,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd",
           &data->action,
           &data->CAR.ID, &data->CAR.x, &data->CAR.y, &data->CAR.Acceleration, &data->CAR.Angle, &data->CAR.Velocity, &data->CAR.Direction, &data->CAR.msg,
           &data->Intersection.My_Dist, &data->Intersection.other_Dist, &data->Intersection.Mytime, &data->Intersection.Othertime, &data->Intersection.Current_Dist, &data->Intersection.point_X, &data->Intersection.point_Y,
           &data->SuddenBrake.SideDistance, &data->SuddenBrake.FrontRearDistance, &data->SuddenBrake.safe_distance);
}


int main() 
{
    
#ifdef SEND_DATA
    int i2c_fd;
    // Open the I2C device
    const char i2c_device[] = "/dev/i2c-1"; // I2C device path
    if ((i2c_fd = open(i2c_device, O_RDWR)) < 0) {
        perror("Error opening I2C device");
        exit(EXIT_FAILURE);
    }

    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, ARDUINO_I2C_ADDRESS) < 0) {
        perror("Error setting I2C slave address");
        close(i2c_fd);
        exit(EXIT_FAILURE);
    }

    // Serialize the struct into a Char
    char buffer1[30]; 
    char buffer2[30]; 
    char buffer3[30]; 

    struct_to_string3(&data,buffer1,buffer2,buffer3);

    while(1)
    {    
        printf("Sent buffer1 : %s\n",buffer1);
        printf("Sent buffer2 : %s\n",buffer2);
        printf("Sent buffer3 : %s\n",buffer3);
        printf("Data Sent Successfully\n");

        i2c_send_data(i2c_fd, buffer1,strlen(buffer1));
        i2c_send_data(i2c_fd, buffer2,strlen(buffer2));
        i2c_send_data(i2c_fd, buffer3,strlen(buffer3));

        sleep(2);
    }

#else
  int file;
    char filename[] = "/dev/i2c-1"; // I2C bus 1 on Raspberry Pi
    char buffer[30] = ""; // Assuming buffer size is 100 bytes
    char Received_data[100] = "";    // Initialize buffer for concatenated data
    int bytesRead;

    // Open the I2C bus
    if ((file = open(filename, O_RDWR)) < 0)
    {
        perror("Failed to open the bus.");
        return 1;
    }

    // Set the slave address
    if (ioctl(file, I2C_SLAVE, ARDUINO_I2C_ADDRESS) < 0)
    {
        perror("Failed to acquire bus access and/or talk to slave.");
        return 1;
    }
    
    while (1)
    {
        // Reset the buffer
        memset(Received_data, 0, sizeof(Received_data));

        for (int i = 0; i < 3; ++i) 
        {
            // Read data from I2C slave
            if ((bytesRead = read(file, buffer, sizeof(buffer) - 1)) < 0) {
                perror("Failed to read from the I2C bus.");
                return 1;
            }
            buffer[bytesRead] = '\0'; // Null terminate the received data

            // Concatenate the received string to Received_data
            strcat(Received_data, buffer);
        }

        // Print the concatenated data for debugging
        printf("Received data: %s\n", Received_data);

        parseReceivedData(Received_data, &Car_Info);
        
        printData(&Car_Info);
        sleep(2);
    }

#endif
    // Close the I2C device
    close(i2c_fd);

    return 0;
}