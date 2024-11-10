#include "INC/HAL_espNow.h"


uint8_t receiver_mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

typedef struct message
{
    u8 ID;
    u8 MAC[6]; 
    s16 x;
    s16 y;
    s16 Accelaration;
    s16 Velocity;
    u8 Angle;
} S_Message_t;
void Send_Messages(S_Message_t *data, u8 *peerMac)
{
    uint8_t serializedData[sizeof(S_Message_t)];
    memcpy(serializedData, data, sizeof(S_Message_t));
    esp_now_send(peerMac, serializedData, sizeof(S_Message_t));
}

S_Message_t Receive_Message(uint8_t *macAddr, uint8_t *data, int dataLen)
{
    S_Message_t receivedData; // Declare a struct to hold the received data

    if (dataLen != sizeof(S_Message_t))
    {
        Serial.println("Data Length Error");
        return;
    }
    memcpy(&receivedData, data, sizeof(S_Message_t));
    return receivedData;
}
