
#include "msgipc.h"
// Define the structure for the message queue
typedef struct
{
    long mtype;
    char mtext[128];
} message_buf;

#define MSG_SIZE 128

/**
 * @brief Sends a message to the Python process using a message queue.
 * 
 * This function sends a message to a Python process through a message queue.
 * The message type and message content are specified by the parameters.
 * 
 * @param [in] msgid Message queue identifier used to send the message.
 * @param [in] mtype Message type used to identify the message in the queue.
 * @param [in] message Pointer to a uint16_t value to be sent as the message.
 */
void send_message_to_PY(uint32 msgid, long mtype, uint16 *message)
{
    message_buf sbuf;
    sbuf.mtype = mtype; // Set the message type

    // Copy the uint16_t value to the message buffer
    memcpy(sbuf.mtext, message, sizeof(*message));

    // Send the message to the message queue
    if (msgsnd(msgid, &sbuf, sizeof(*message), 0) == -1)
    {
        perror("\nSend message to PY [FAILED]");
    }
    // Uncomment the following lines for debugging
    // printf("------------------------------\n"
    //        " C -> P : Sending to Python process: %d\n"
    //        "------------------------------\n",
    //        *message);
}

// /**
//  * @brief Parses a message string into message type and displacement values.
//  * 
//  * This function extracts the message type and displacement value from a message string.
//  * 
//  * @param [in] message_str The input message string in the format "MsgType,disp".
//  * @param [out] MsgType Pointer to a uint16_t variable where the message type will be stored.
//  * @param [out] disp Pointer to a uint16_t variable where the displacement value will be stored.
//  */
// void parse_message(char *message_str, uint16 *MsgType, uint16 *disp)
// {
//     // Parse the message string using sscanf to extract MsgType and disp
//     sscanf(message_str, "%hu,%hu", MsgType, disp);
// }

// /**
//  * @brief Receives a message from the message queue and parses it.
//  * 
//  * This function receives a message from the message queue and parses it to extract the message type and displacement value.
//  * 
//  * @param [in] msgid The message queue identifier.
//  * @param [out] message_str Pointer to a buffer where the received message string will be stored.
//  * @retval Displacement value extracted from the received message.
//  */
// uint16 receiveMessage_from_PY(uint32 msgid, uint8 *message_str)
// {
//     message_buf rbuf;
//     uint16 disp = 0;

//     // Receive the message from the queue (blocking call)
//     if (msgrcv(msgid, &rbuf, sizeof(rbuf.mtext), 0, 0) == -1)
//     {
//         perror("\nControl -> Calculate [ FAILED ]");
//         return 0; // Return 0 to indicate failure
//     }
//     else
//     {
//         // Copy the received message to the provided buffer
//         strncpy((char *)message_str, rbuf.mtext, MSG_SIZE - 1);
//         message_str[MSG_SIZE - 1] = '\0'; // Null-terminate the string

//         uint16 MsgType;
//         parse_message((char *)message_str, &MsgType, &disp);

//         // Uncomment for debugging
//         // printf("MsgType: %hu, disp: %hu\n", MsgType, disp);
//     }

//     return disp; // Return the extracted displacement value
// }
