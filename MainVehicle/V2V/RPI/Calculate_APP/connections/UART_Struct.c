

#include "UART_Struct.h"
#include "../ipcMsg/msgipc.h"

char buffer[128];
/**
 * @brief Initializes the UART communication by opening the specified device.
 *
 * This function opens the UART device specified by the `device` parameter, configures
 * the UART settings, and returns the file descriptor for the UART device. It sets the
 * baud rate, character size, and other serial port options.
 *
 * @param [in] device The path to the UART device file (e.g., "/dev/ttyUSB0").
 *
 * @retval int The file descriptor for the opened UART device, or -1 if an error occurred.
 */
int UART_init(char *device)
{
    // Open the UART device in read/write mode, without controlling terminal and with non-blocking mode
    int usb_filestream = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (usb_filestream == -1)
    {
        // Print error message if device cannot be opened
        perror("Error opening device");
        return -1; // Return -1 to indicate an error
    }

    // Get the current UART options
    struct termios options;
    tcgetattr(usb_filestream, &options);

    // Set the UART options
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD; // Set baud rate to 9600, 8 data bits, local mode, and enable receiver
    options.c_iflag = IGNPAR | ICRNL;               // Ignore parity errors and convert carriage return to newline

    // Flush the input and output buffers
    tcflush(usb_filestream, TCIFLUSH);

    // Apply the new settings
    tcsetattr(usb_filestream, TCSANOW, &options);

    // Return the file descriptor for the UART device
    return usb_filestream;
}

/**
 * @brief Sends data over UART.
 * 
 * This function writes data from a buffer to a UART device specified by the file descriptor.
 * 
 * @param [in] usb_filestream File descriptor for the UART device.
 * @param [in] buffer Pointer to the buffer containing data to be sent.
 * @param [in] buffer_size Size of the data to be sent in bytes.
 */
void _UART_send_data(int usb_filestream, char *buffer, int buffer_size)
{
    // Write the data from the buffer to the UART device
    // ssize_t bytes_written = write(usb_filestream, buffer, buffer_size);
    write(usb_filestream, buffer, buffer_size);

    // Uncomment the following lines for error checking and debugging
    // if (bytes_written < 0)
    // {
    //     perror("UART TX error"); // Print an error message if the write operation fails
    //     printf("%ld\n", bytes_written); // Print the number of bytes written (for debugging purposes)
    // }
}


/**
 * @brief Receives and processes initial data from the UART device.
 * 
 * This function reads data from the UART device until a specific character 'g' is received.
 * Once received, it parses the data to extract ACK, GPS_X, and GPS_Y values.
 */
void _UART_Receive_initials()
{
    char buffer[128] = {0}; // Initialize buffer with zeros
    char ACK = 0;

    _UART_receive_data(usb0_filestream, buffer, sizeof(buffer));

    while (buffer[0] != 'g')
    {
        _UART_receive_data(usb0_filestream, buffer, sizeof(buffer));
    }
    sscanf(buffer, "%c,%e,%e", &ACK, &MainVehicle.x, &MainVehicle.y);
}

/**
 * @brief Waits for user input to start the Python process.
 * 
 * This function prompts the user to press the 's' key to start the Python process. 
 * It continues to prompt the user until the correct input is received. Once 's' is 
 * pressed, it sends a message to Python to signal the start of the process.
 */
// void _UART_Receive_StartFlag()
// {
//     char buffer[128] = {0}; // Initialize buffer with zeros
//     char ACK = '0';
//     uint16 StartpythonProcess = True;

//     printf("press [s] to start python code: ");
//     scanf(" %c", &buffer[0]);
//     ACK = buffer[0];

//     while (ACK != 's')
//     {
//         printf("press [s] to start python code: ");
//         scanf(" %c", &buffer[0]);
//         ACK = buffer[0];
//     }
//     if (ACK == 's')
//     {
//         send_message_to_PY(msgid_ACT_send, 1, &StartpythonProcess); // this would have the value of the needed speed
//     }
// }

/**
 * @brief Receives data from the UART device into a buffer.
 * 
 * This function reads data from the UART device into the provided buffer. It uses 
 * the `select()` function to wait for data to become available, with a specified 
 * timeout. After reading the data, it null-terminates the buffer string.
 * 
 * @param [in] usb_filestream File descriptor for the UART device.
 * @param [out] buffer Pointer to the buffer where received data will be stored.
 * @param [in] buffer_size Size of the buffer in bytes.
 */
void _UART_receive_data(int usb_filestream, char *buffer, int buffer_size)
{
    int bytes_to_read = buffer_size - 1; // Reserve space for null-terminator
    int received_bytes;

    // Flush the input buffer to clear any stale data
    tcflush(usb_filestream, TCIFLUSH);

    // Set timeout for select function
    struct timeval timeout;
    timeout.tv_sec = 1;       // 1-second timeout
    timeout.tv_usec = 100000; // 0.1-second timeout
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(usb_filestream, &readfds);

    // Wait until data is available or timeout occurs
    int select_result = select(usb_filestream + 1, &readfds, NULL, NULL, &timeout);
    if (select_result < 0)
    {
        // Handle select error
        perror("Select error");
    }
    else if (select_result == 0)
    {
        // No data available within the timeout period
        printf("No data available\n");
        return; // Exit function early if no data is available
    }

    // Data is available, proceed with reading
    received_bytes = read(usb_filestream, buffer, bytes_to_read);

    if (received_bytes <= 0)
    {
        // Handle read error or no data received
        printf("No Data Received\n");
    }
    else
    {
        // Null-terminate the buffer to safely use it as a string
        buffer[received_bytes] = '\0';
    }
}


/**
 * @brief Sends a structure over UART.
 * 
 * This function converts a `S_Connected_Car_t` structure to a string and sends it over UART.
 * 
 * @param [in] data Pointer to the structure to be sent.
 */
void _UART_Send_Struct(S_Connected_Car_t *data)
{
    char buffer[128];
    int size = struct_to_string(data, buffer);
    _UART_send_data(usb0_filestream, buffer, size);
}

/**
 * @brief Receives a structure from UART.
 * 
 * This function receives a string from UART and converts it to a `S_Connected_Car_t` structure.
 * 
 * @param [out] data Pointer to the structure where the received data will be stored.
 */
void _UART_Receive_Struct(S_Connected_Car_t *data)
{
    char buffer[128];
    _UART_receive_data(usb0_filestream, buffer, sizeof(buffer));
    // Optionally print received data for debugging
    // printf("ESP-NOW : %s\n", buffer);
    string_to_struct(buffer, data);
}