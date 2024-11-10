#ifndef __MQTT__
#define __MQTT__
#include "../utils.h"
#include <MQTTClient.h>

// MQTT client and connection options
extern MQTTClient client;
extern MQTTClient_connectOptions conn_opts ;
extern volatile int rc;

// Global variables for topic and QoS
#define MAIN_VEHICLE "MainVehicle/Data"
#define DUMMY_VEHICLE "DummyVehicle/Data"
// 8883 is for encrypted and 1883 is for unencrypted data
#define SERVER_IP "tcp://test.mosquitto.org:1883"

#define QOS 1

// Function prototypes
void MQTT_initialize();
void MQTT_send_message(char *payload, S_Connected_Car_t *car);
int on_message_received(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void MQTT_receive_message();
void MQTT_cleanup();
void string_to_struct(const char *receivedData, S_Connected_Car_t *data);
int struct_to_string(S_Connected_Car_t *data, char *buffer);
#endif