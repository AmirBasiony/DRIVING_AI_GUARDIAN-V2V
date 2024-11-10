#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>

// #define MASTER_SEND_DATA
// #define MASTER_RECEIVE_DATA
// #define SLAVE_SEND_DATA
#define SLAVE_RECEIVE_DATA

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
    double x;
    double y;
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
    double point_X;
    double point_Y;
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
    /* SuddenBrake */ {7, 15, 20}};
String structToString1(const S_DATA_t &data)
{
    long X_multiplier = data.CAR.x * 1000000;
    uint16_t last_4_digits_X = X_multiplier % 10000;

    long Y_multiplier = data.CAR.y * 1000000;
    uint16_t last_4_digits_Y = Y_multiplier % 10000;

    long X_collision_multiplier = data.Intersection.point_X * 1000000;
    uint16_t last_collision_4_digits_X = X_collision_multiplier % 10000;

    long Y_collision_multiplier = data.Intersection.point_Y * 1000000;
    uint16_t last_collision_4_digits_Y = Y_collision_multiplier % 10000;
    // Convert each field to string and concatenate them with a delimiter
    String str = String(data.action) + "," +
                 String(data.CAR.ID) + "," +
                 String(last_4_digits_X) + "," +
                 String(last_4_digits_Y) + "," +
                 String(data.CAR.Acceleration) + "," +
                 String(data.CAR.Angle) + "," +
                 String(data.CAR.Velocity) + "," +
                 String(data.CAR.Direction) + "," +
                 String(data.CAR.msg) + "," +
                 String(data.Intersection.My_Dist) + "," +
                 String(data.Intersection.other_Dist) + "," +
                 String(data.Intersection.Mytime) + "," +
                 String(data.Intersection.Othertime) + "," +
                 String(data.Intersection.Current_Dist) + "," +
                 String(last_collision_4_digits_X) + "," +
                 String(last_collision_4_digits_Y) + "," +
                 String(data.SuddenBrake.SideDistance) + "," +
                 String(data.SuddenBrake.FrontRearDistance) + "," +
                 String(data.SuddenBrake.safe_distance);
    return str;
}

void Send_Initial_Coordinates()
{
    String Coordinates_from_mobile = " "; // frame should be "longitude,lattitude"
    char achknowledgment = '0';
    if (Serial.available())
    {
        // read coordinations from bluetooth
        while (Coordinates_from_mobile == " ")
        {
            Coordinates_from_mobile = Serial.readStringUntil('\n');
        }
        int idx1 = Coordinates_from_mobile.indexOf(',');
        String X = Coordinates_from_mobile.substring(0, idx1);
        data.CAR.x = X.toFloat();

        int idx2 = Coordinates_from_mobile.indexOf(',', idx1 + 1);
        String Y = Coordinates_from_mobile.substring(idx1 + 1, idx2);
        data.CAR.y = Y.toFloat();
        // validate the data
        if (longitude != 0.0 && lattitude != 0.0)
        {
            data.CAR.x = longitude;
            data.CAR.y = lattitude;
        }
    }

    // wait for acknoledgment from rpi
    if (Serial.available())
    {
        while (achknowledgment == '0')
        {
            Serial.print(structToString1(data));
            achknowledgment = Serial.read();
        }
    }
}
String structToString2(const S_DATA_t &data)
{
    long X_multiplier = data.Intersection.point_X * 1000000;
    uint16_t last_4_digits_X = X_multiplier % 10000;

    long Y_multiplier = data.Intersection.point_Y * 1000000;
    uint16_t last_4_digits_Y = Y_multiplier % 10000;

    // Convert each field to string and concatenate them with a delimiter
    String str = String(data.Intersection.My_Dist) + "," +
                 String(data.Intersection.other_Dist) + "," +
                 String(data.Intersection.Mytime) + "," +
                 String(data.Intersection.Othertime) + "," +
                 String(data.Intersection.Current_Dist) + "," +
                 String(last_4_digits_X) + "," +
                 String(last_4_digits_Y) + ",";
    return str;
}
String structToString3(const S_DATA_t &data)
{
    // Convert each field to string and concatenate them with a delimiter
    String str = String(data.SuddenBrake.SideDistance) + "," +
                 String(data.SuddenBrake.FrontRearDistance) + "," +
                 String(data.SuddenBrake.safe_distance);
    return str;
}

int currentStringIndex = 0;
void sendData()
{
    String strData[3];

    // Populate the array with the three strings
    // // Convert struct to strings
    strData[0] = structToString1(data);
    strData[1] = structToString2(data);
    strData[2] = structToString3(data);

    // Check if the current string index is valid
    if (currentStringIndex >= 0 && currentStringIndex < 3)
    {
        // Convert the current string to a char array
        char charArray[strData[currentStringIndex].length() + 1];
        strData[currentStringIndex].toCharArray(charArray, sizeof(charArray));

        // Send the current string along with its length
        Wire.write(charArray, strlen(charArray) + 1); // +1 to include the null terminator

        // Increment the current string index for the next request
        currentStringIndex++;

        // Print the sent data for debugging
        Serial.print("Sending data ");
        Serial.print(currentStringIndex);
        Serial.print(": ");
        Serial.println(strData[currentStringIndex - 1]);
        Serial.println("Sending data Done Successfully ");
        if (currentStringIndex == 3)
        {
            // Reset the current string index if all strings have been sent
            currentStringIndex = 0;

            // Print a message indicating that all data has been sent
            Serial.println("All data sent.");
        }
    }
}

void parseReceivedData(const char *receivedData, S_DATA_t *data)
{
    char *endPtr;

    // Extract values for action and CAR
    data->action = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr; // Move past the comma
    data->CAR.ID = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->CAR.x = strtod(receivedData, &endPtr);
    receivedData = ++endPtr;
    data->CAR.y = strtod(receivedData, &endPtr);
    receivedData = ++endPtr;
    data->CAR.Acceleration = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->CAR.Angle = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->CAR.Velocity = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->CAR.Direction = static_cast<E_Direction_t>(strtol(receivedData, &endPtr, 10));
    receivedData = ++endPtr;
    data->CAR.msg = static_cast<E_MSG_t>(strtol(receivedData, &endPtr, 10));
    receivedData = ++endPtr;

    // Extract values for Intersection
    data->Intersection.My_Dist = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->Intersection.other_Dist = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->Intersection.Mytime = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->Intersection.Othertime = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->Intersection.Current_Dist = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->Intersection.point_X = strtod(receivedData, &endPtr);
    receivedData = ++endPtr;
    data->Intersection.point_Y = strtod(receivedData, &endPtr);
    receivedData = ++endPtr;

    // Extract values for SuddenBrake
    data->SuddenBrake.SideDistance = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->SuddenBrake.FrontRearDistance = strtol(receivedData, &endPtr, 10);
    receivedData = ++endPtr;
    data->SuddenBrake.safe_distance = strtol(receivedData, &endPtr, 10);
}

void printData(const S_DATA_t *data)
{
    Serial.print("Action: ");
    Serial.println(data->action);
    Serial.print("Car ID: ");
    Serial.println(data->CAR.ID);
    Serial.print("Car x: ");
    Serial.println(data->CAR.x);
    Serial.print("Car y: ");
    Serial.println(data->CAR.y);
    Serial.print("Car Acceleration: ");
    Serial.println(data->CAR.Acceleration);
    Serial.print("Car Angle: ");
    Serial.println(data->CAR.Angle);
    Serial.print("Car Velocity: ");
    Serial.println(data->CAR.Velocity);
    Serial.print("Car Direction: ");
    Serial.println(data->CAR.Direction);
    Serial.print("Car Message: ");
    Serial.println(data->CAR.msg);
    Serial.print("Intersection My_Dist: ");
    Serial.println(data->Intersection.My_Dist);
    Serial.print("Intersection other_Dist: ");
    Serial.println(data->Intersection.other_Dist);
    Serial.print("Intersection Mytime: ");
    Serial.println(data->Intersection.Mytime);
    Serial.print("Intersection Othertime: ");
    Serial.println(data->Intersection.Othertime);
    Serial.print("Intersection Current_Dist: ");
    Serial.println(data->Intersection.Current_Dist);
    Serial.print("Intersection point_X: ");
    Serial.println(data->Intersection.point_X);
    Serial.print("Intersection point_Y: ");
    Serial.println(data->Intersection.point_Y);
    Serial.print("SuddenBrake SideDistance: ");
    Serial.println(data->SuddenBrake.SideDistance);
    Serial.print("SuddenBrake FrontRearDistance: ");
    Serial.println(data->SuddenBrake.FrontRearDistance);
    Serial.print("SuddenBrake safe_distance: ");
    Serial.println(data->SuddenBrake.safe_distance);
}

int ReceivedStringNum = 0;
int i = 0;
void SlavereceiveEvent(int numBytes)
{
    char received_bytes[60];
    S_DATA_t received_car;
    // Serial.print("Received data from Raspberry Pi: ");

    while (Wire.available())
    {
        received_bytes[i] = Wire.read();
        // Serial.print(received_bytes[i]);
        i++;
    }
    received_bytes[i] = '\0';
    // Serial.println(received_bytes);
    ReceivedStringNum++;
    if (ReceivedStringNum == 3)
    {
        parseReceivedData(received_bytes, &received_car);
        Serial.println("---------------------------------------------------------");
        Serial.println(String("[") + i + "] I2C Message Received from Raspberry Pi Successfully");
        Serial.println("---------------------------------------------------------");
        printData(&received_car);
        ReceivedStringNum = 0;
        i = 0;
    }
}
void MastersendData()
{
    String strData[3];

    // Populate the array with the three strings
    // // Convert struct to strings
    strData[0] = structToString1(data);
    strData[1] = structToString2(data);
    strData[2] = structToString3(data);

    // Check if the current string index is valid
    if (currentStringIndex >= 0 && currentStringIndex < 3)
    {

        // Send the serialized data over I2C to the Raspberry Pi
        Wire.beginTransmission(ARDUINO_I2C_ADDRESS);
        Wire.write(strData[currentStringIndex].c_str());
        Wire.endTransmission();
        // Increment the current string index for the next request
        currentStringIndex++;

        // Print the sent data for debugging
        Serial.print("Sending data ");
        Serial.print(currentStringIndex);
        Serial.print(": ");
        Serial.println(strData[currentStringIndex - 1]);
        Serial.println("Sending data Done Successfully ");
        if (currentStringIndex == 3)
        {
            // Reset the current string index if all strings have been sent
            currentStringIndex = 0;

            // Print a message indicating that all data has been sent
            Serial.println("All data sent.");
        }
    }
}
void setup()
{

    Wire.begin(ARDUINO_I2C_ADDRESS);
    // Wire.onRequest(sendData);
    Wire.onReceive(SlavereceiveEvent);
    Serial.begin(9600);
}

void MasterReceiveData()
{
    char data[100];
    Wire.requestFrom(ARDUINO_I2C_ADDRESS, sizeof(data)); // Request data from the Raspberry Pi

    // Wait until data is available
    while (Wire.available())
        ;

    // Read the received data into the data structure
    Wire.readBytes((uint8_t *)&data, sizeof(data));

    // Print received data for debugging
    Serial.println("Received data from Raspberry Pi:");
    Serial.println(data);
    // Example: Serial.println(data.sensorValue);
}
void loop()
{
}

/*
void sendData(const S_DATA_t &data)
{
    String strData[3];

    // Populate the array with the three strings
    strData[0] = structToString1(data);
    strData[1] = structToString2(data);
    strData[2] = structToString3(data);

    // Iterate through the strings and send them to the slave device
    for (int i = 0; i < 3; i++) {
        char charArray[strData[i].length() + 1];
        strData[i].toCharArray(charArray, sizeof(charArray));

        Wire.beginTransmission(9);
        Wire.write(charArray, strlen(charArray) + 1);
        Wire.endTransmission();


        // Print the sent data for debugging
        Serial.print("Sending data ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(strData[i]);
    }
    // Print a message indicating that all data has been sent
    Serial.println("All data sent.");
}
*/