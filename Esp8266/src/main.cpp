
#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "Platform_Types.h"

#define RPI_ADDRESS 7856123 // needs to be configured

#define COMMUNICATION_CHANNEL 0 //(0-13, depending on the region).
#define COMMUNICATION_KEY 0     //. In this case, '0' means that no encryption key is used for this communication
#define COMMUNICATION_KEY_LEN 0

typedef struct message
{
    u8 ID;
    String MAC;
    u8 x;
    u8 y;
    u8 Velocity;
    u8 Accelaration;
    u8 Angle;
} S_Message_t;

S_Message_t rpiData, receivedData;

u8 receiver_mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
bool receivedFrom_RPI = false;

void ESPNOW_Receive_Message(u8 *macAddr, u8 *data, u8 dataLen)
{
    if (dataLen == sizeof(S_Message_t))
    {
        memcpy(&receivedData, data, sizeof(S_Message_t));
        Serial.println("Received Data:");
        Serial.println(receivedData.ID);
        Serial.println(receivedData.MAC);
        Serial.println(receivedData.x);
        Serial.println(receivedData.y);
        Serial.println(receivedData.Velocity);
        Serial.println(receivedData.Accelaration);
        Serial.println(receivedData.Angle);

        // Send the received data to the Raspberry Pi via I2C
        I2C_Send_Data_ESP_to_RPI(RPI_ADDRESS);
    }
    else
    {
        Serial.println("Data Length Error");
    }
}

void I2C_Send_Data_ESP_to_RPI(u8 Address)
{
    // Start communication by sending data to RPI (slave)
    Wire.beginTransmission(Address);
    Wire.write(receivedData.ID);

    for (u8 i = 0; i < sizeof(receivedData.MAC); i++)
    {
        Wire.write(receivedData.MAC[i]);
    }

    Wire.write(receivedData.x);
    Wire.write(receivedData.y);
    Wire.write(receivedData.Velocity);
    Wire.write(receivedData.Accelaration);
    Wire.write(receivedData.Angle);

    Wire.endTransmission();
}

void Send_Data_ESP_NOW(u8 *peerMac)
{
    // Create a message to send to the other ESP8266 peer
    S_Message_t dataToSend;
    dataToSend.ID = 1;
    dataToSend.MAC = WiFi.macAddress();
    dataToSend.x = rpiData.x;
    dataToSend.y = rpiData.y;
    dataToSend.Velocity = rpiData.Velocity;
    dataToSend.Accelaration = rpiData.Accelaration;
    dataToSend.Angle = rpiData.Angle;

    // Send the message to the other ESP8266 peer via ESP-NOW
    esp_now_send(peerMac, (u8 *)&dataToSend, sizeof(S_Message_t));
}
void Read_Data_From_RPI(u8 Address)
{
    // Start communication by requesting data from RPI (slave)
    Wire.requestFrom(Address, sizeof(S_Message_t));

    // Read the data from RPI
    for (u8 i = 0; i < sizeof(S_Message_t); i++)
    {
        ((u8 *)&rpiData)[i] = Wire.read();
    }
    receivedFrom_RPI = true;
}
void setup()
{
    Serial.begin(115200);
    Wire.begin(RPI_ADDRESS); // Initialize I2C communication as a slave with address 8
    WiFi.mode(WIFI_STA);     // Use WIFI_STA mode for ESP8266

    if (esp_now_init())
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_add_peer(receiver_mac, ESP_NOW_ROLE_CONTROLLER, COMMUNICATION_CHANNEL, COMMUNICATION_KEY, COMMUNICATION_KEY_LEN);

    // Register the receive callback
    esp_now_register_recv_cb(ESPNOW_Receive_Message);
}

void loop()
{
    // Read data from RPI via I2C
    Read_Data_From_RPI(RPI_ADDRESS);
    if (receivedFrom_RPI)
    {
        // Send data to the other ESP8266 peer via ESP-NOW
        Send_Data_ESP_NOW(receiver_mac);
        receivedFrom_RPI=false;
    }
}