#include "MQTT.h"

MQTTClient client;
MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;
volatile int rc;
Bool_t KnowDummy = False;
void MQTT_initialize()
{
    // Initialize the SSL options
    ssl_opts.enableServerCertAuth = 1; // Enable server certificate authentication
    if ((rc = MQTTClient_create(&client, SERVER_IP, "RPI_Client",
                                MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to create client, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.ssl = &ssl_opts;
    MQTTClient_setCallbacks(client, NULL, NULL, on_message_received, NULL);

    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        fprintf(stderr, "Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    if ((rc = MQTTClient_subscribe(client, DUMMY_VEHICLE, QOS)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to subscribe, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
}

void MQTT_send_message(char *payload, S_Connected_Car_t *car)
{
    // Serialize the `Vehicle` structure into a payload (for simplicity, using `sprintf`)
    sprintf(payload, "%d,%.2f,%.2f,%u,%.2f,%u,%d,%d,%d",
            car->ID, car->x, car->y, car->Accelration, car->Angle, car->Velocity, car->Direction, car->msg, car->action);

    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;

    pubmsg.payload = (void *)payload;
    pubmsg.payloadlen = strlen(payload);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;

    if ((rc = MQTTClient_publish(client, MAIN_VEHICLE, strlen(payload), payload, QOS, 0, NULL)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to publish, return code %d\n", rc);
    }
    MQTTClient_waitForCompletion(client, token, 10000);
    printf("Publication of %s on %s for client SUCCESSED...\n",
           payload, MAIN_VEHICLE);
    // printf("Message with token %d delivered\n", token);
}

int on_message_received(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    printf("topic: %s\n", topicName);
    printf("message: %.*s\n", message->payloadlen, (char *)message->payload);
    string_to_struct((char *)message->payload, &DummyVehicle);
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    if (DummyVehicle.ID == 0)
        KnowDummy=False;
    else
        KnowDummy=True;
    
    return 1;
}

// void MQTT_receive_message()
// {
//     MQTTClient_message *message = NULL;
//     char *topicName = NULL;
//     int topicLen = 0;
//     rc = MQTTClient_receive(client, &topicName, &topicLen, &message, 100); // Timeout of 100 ms
//     if (message != NULL)
//     {
//         on_message_received(NULL, topicName, topicLen, message);
//     }
// }

void MQTT_cleanup()
{
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
}

/**
 * @brief Converts a structure to a string.
 *
 * This function serializes the data from a `S_Connected_Car_t` structure into a string format,
 * which can be sent over UART or used for other purposes.
 *
 * @param [in] data Pointer to the structure containing the data to be serialized.
 * @param [out] buffer Pointer to the buffer where the resulting string will be stored.
 * @retval Number of bytes written to the buffer.
 */
int struct_to_string(S_Connected_Car_t *data, char *buffer)
{
    int bytes_written = 0;

    // Serialize the structure fields into the buffer
    bytes_written += sprintf(buffer + bytes_written, "%d,%.2f,%.2f,%d,%d,%d,%d,%d\n",
                             data->ID, data->x, data->y,
                             data->Accelration, (int32_t)data->Angle, data->Velocity,
                             data->Direction, data->msg);

    // Return the total number of bytes written
    return bytes_written;
}

/**
 * @brief Converts a string to a structure.
 *
 * This function deserializes data from a string into a `S_Connected_Car_t` structure.
 * The string must be formatted as specified in `struct_to_string`.
 *
 * @param [in] receivedData Pointer to the string containing the data to be deserialized.
 * @param [out] data Pointer to the structure where the deserialized data will be stored.
 */
void string_to_struct(const char *receivedData, S_Connected_Car_t *data)
{
    char *endPtr;

    // Deserialize the structure fields from the string
    data->ID = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->x = strtod(receivedData, &endPtr);
    receivedData = ++endPtr;
    data->y = strtod(receivedData, &endPtr);
    receivedData = ++endPtr;
    data->Accelration = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->Angle = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->Velocity = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->Direction = (E_Direction_t)(strtol(receivedData, &endPtr, 10));
    receivedData = ++endPtr;
    data->msg = (E_MSG_t)(strtol(receivedData, &endPtr, 10));
}