#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h> 

// Assuming E_Direction_t and E_MSG_t are defined as enums or similar
typedef enum
{
    NORTH,
    SOUTH,
    EAST,
    WEST
} E_Direction_t;
typedef enum
{
    MSG_OK,
    MSG_ERROR
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

#define UART_DEV "/dev/ttyUSB0"

// Initialize your structure here
S_DATA_t data_to_send = {
    .action = 1, // Example action
    .CAR = {.ID = 1, .x = 100, .y = 200, .Acceleration = 300, .Angle = 45, .Velocity = 60, .Direction = NORTH, .msg = MSG_OK},
    .Intersection = {.My_Dist = 400, .other_Dist = 500, .Mytime = 600, .Othertime = 700, .Current_Dist = 800, .point_X = 900, .point_Y = 1000},
    .SuddenBrake = {.SideDistance = 1100, .FrontRearDistance = 1200, .safe_distance = 1300}};



// Initialization function
int initialize(char *device)
{
    int usb_filestream = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (usb_filestream == -1)
    {
        perror("Error opening device");
        return -1;
    }

    struct termios options;
    tcgetattr(usb_filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR | ICRNL;
    tcflush(usb_filestream, TCIFLUSH);
    tcsetattr(usb_filestream, TCSANOW, &options);

    return usb_filestream;
}

// Send function
int send_data(int usb_filestream, char *buffer, int buffer_size)
{
    int count = write(usb_filestream, buffer, buffer_size);
    if (count < 0)
    {
        perror("UART TX error");
        return -1;
    }
    printf("Sent Data: %s\n",buffer);
    return count;
}
int receive_data(int usb_filestream, char *buffer, int buffer_size) 
{
    int bytes_to_read = buffer_size - 1;
    int received_bytes;

    // Flush input buffer
    tcflush(usb_filestream, TCIFLUSH);

    // Set a timeout value for waiting for data
    struct timeval timeout;
    timeout.tv_sec = 1; // 1-second timeout
    timeout.tv_usec = 100000; // 0.5-second timeout
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(usb_filestream, &readfds);

    // Wait until data is available to read or timeout occurs
    int select_result = select(usb_filestream + 1, &readfds, NULL, NULL, &timeout);
    if (select_result < 0) 
    {
        perror("Select error");
        return -1;
    } 
    else if (select_result == 0) 
    {
        printf("Timeout occurred, no data available\n");
        return 0; // Return 0 bytes received
    }

    // Data is available to read, proceed with reading
    received_bytes = read(usb_filestream, buffer, bytes_to_read);
    if (received_bytes < 0) 
    {
        perror("UART RX error");
        return -1;
    }

    buffer[received_bytes] = '\0'; // Null-terminate the string
    printf("Received %d bytes: %s\n", received_bytes, buffer);
    return received_bytes;
}



// Convert structure to string function
int struct_to_string(S_DATA_t *data, char *buffer, int buffer_size)
{
    int bytes_written = 0;
    bytes_written += sprintf(buffer + bytes_written, "<"); // Start marker

    // Serialize action
    bytes_written += sprintf(buffer + bytes_written, "%d,", data->action);

    // Serialize CAR
    bytes_written += sprintf(buffer + bytes_written, "%d,%d,%d,%d,%d,%d,%d,%d,",
                             data->CAR.ID, data->CAR.x, data->CAR.y,
                             data->CAR.Acceleration, data->CAR.Angle, data->CAR.Velocity,
                             data->CAR.Direction, data->CAR.msg);

    // Serialize Intersection
    bytes_written += sprintf(buffer + bytes_written, "%d,%d,%d,%d,%d,%d,%d,",
                             data->Intersection.My_Dist, data->Intersection.other_Dist,
                             data->Intersection.Mytime, data->Intersection.Othertime,
                             data->Intersection.Current_Dist, data->Intersection.point_X,
                             data->Intersection.point_Y);

    // Serialize SuddenBrake
    bytes_written += sprintf(buffer + bytes_written, "%d,%d,%d",
                             data->SuddenBrake.SideDistance, data->SuddenBrake.FrontRearDistance,
                             data->SuddenBrake.safe_distance);
    bytes_written += sprintf(buffer + bytes_written, ">"); // End marker

    return bytes_written;
}

int main() {
    char buffer[128];
    char received_buffer[128];

    int usb0_filestream = initialize(UART_DEV);
    if (usb0_filestream == -1)
        return -1;
    
    while (1) {
        memset(buffer, '\0', sizeof(buffer));
        memset(received_buffer, '\0', sizeof(received_buffer));
        
        // Serialize the structure into a string
        int bytes_written = struct_to_string(&data_to_send, buffer, 128);

        // Send the serialized string
        int send_result = send_data(usb0_filestream, buffer, bytes_written);
        if (send_result == -1)
            return -1;

        // Wait for a response from the ESP8266
        int received_bytes = receive_data(usb0_filestream, received_buffer, 128);
        if (received_bytes == -1)
            return -1;
        
        // Optional: Add a delay between consecutive transmissions
        // usleep(500000); // Sleep for 0.5 second
        // sleep(1); // Sleep for 1 second
    }
    
    // Close the file stream
    close(usb0_filestream);
    
    return 0;
}



/*


// int main()
// {
//     int usb0_filestream = -1;
//     char buffer[128];
//     memset(buffer, '\0', sizeof(buffer));
//     usb0_filestream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
//     if (usb0_filestream == -1)
//     {
//         perror("Error ");
//         return -1;
//     }
//     struct termios options;
//     tcgetattr(usb0_filestream, &options);
//     options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
//     options.c_iflag = IGNPAR | ICRNL;
//     tcflush(usb0_filestream, TCIFLUSH);
//     tcsetattr(usb0_filestream, TCSANOW, &options);
//     // Initialize your structure here
//     S_DATA_t data_to_send = {
//         .action = 1, // Example action
//         .CAR = {.ID = 1, .x = 100, .y = 200, .Acceleration = 300, .Angle = 45, .Velocity = 60, .Direction = NORTH, .msg = MSG_OK},
//         .Intersection = {.My_Dist = 400, .other_Dist = 500, .Mytime = 600, .Othertime = 700, .Current_Dist = 800, .point_X = 900, .point_Y = 1000},
//         .SuddenBrake = {.SideDistance = 1100, .FrontRearDistance = 1200, .safe_distance = 1300}};
//     // Serialize the structure into a string
//     int bytes_written = 0;
//     bytes_written += sprintf(buffer + bytes_written, "<"); // Start marker
//     // Serialize action
//     bytes_written += sprintf(buffer + bytes_written, "%d,", data_to_send.action);
//     // Serialize CAR
//     bytes_written += sprintf(buffer + bytes_written, "%d,%d,%d,%d,%d,%d,%d,%d,",
//                              data_to_send.CAR.ID, data_to_send.CAR.x, data_to_send.CAR.y,
//                              data_to_send.CAR.Acceleration, data_to_send.CAR.Angle, data_to_send.CAR.Velocity,
//                              data_to_send.CAR.Direction, data_to_send.CAR.msg);
//     // Serialize Intersection
//     bytes_written += sprintf(buffer + bytes_written, "%d,%d,%d,%d,%d,%d,%d,",
//                              data_to_send.Intersection.My_Dist, data_to_send.Intersection.other_Dist,
//                              data_to_send.Intersection.Mytime, data_to_send.Intersection.Othertime,
//                              data_to_send.Intersection.Current_Dist, data_to_send.Intersection.point_X,
//                              data_to_send.Intersection.point_Y);
//     // Serialize SuddenBrake
//     bytes_written += sprintf(buffer + bytes_written, "%d,%d,%d",
//                              data_to_send.SuddenBrake.SideDistance, data_to_send.SuddenBrake.FrontRearDistance,
//                              data_to_send.SuddenBrake.safe_distance);
//     bytes_written += sprintf(buffer + bytes_written, ">"); // End marker
//     // Send the serialized string
//     int count = write(usb0_filestream, buffer, bytes_written);
//     if (count < 0)
//     {
//         perror("UART TX error");
//         return -1;
//     }
//     // Wait for a response from the ESP8266
//     sleep(1); // Wait for 1 seconds for the ESP8266 to respond
//     char b[128];
//     int received_bytes = read(usb0_filestream, b, sizeof(b) - 1);
//     if (received_bytes > 0)
//     {
//         // Null-terminate the string
//         b[received_bytes] = '\0';
//         printf("Received: %s", b);
//     }
//     else
//     {
//         printf("No response received.\n");
//     }
//     // printf("%s",buffer);
//     // Rest of your code...
//     close(usb0_filestream);
//     return 0;
// }

*/