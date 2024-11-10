#ifndef __UART_RPI_ESP8266__
#define __UART_RPI_ESP8266__


#include "../Algo/PositionCalculations.h"

#define UART_DEV "/dev/ttyUSB0"

int UART_init(char *device);
void _UART_send_data(int usb_filestream, char *buffer, int buffer_size);
void _UART_receive_data(int usb_filestream, char *buffer, int buffer_size);
int struct_to_string(S_Connected_Car_t *data, char *buffer);
void string_to_struct(const char *receivedData, S_Connected_Car_t *data);
void _UART_Send_Struct(S_Connected_Car_t *data);
void _UART_Receive_Struct(S_Connected_Car_t *data);
void _UART_Receive_initials();
// void _UART_Receive_StartFlag();
// void Get_initials();
#endif //__UART_RPI_ESP8266__