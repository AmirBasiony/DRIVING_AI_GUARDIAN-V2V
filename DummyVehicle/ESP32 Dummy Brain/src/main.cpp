/***********************includes section**************************/

#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

/************ Global Structs Section **************************/

typedef enum STATE
{
  UNKNOWN,
  NORMAL,
  DANGER,
  FOCUS
} E_State_t;
typedef enum DIRECTION
{
  NORTH,
  SOUTH,
  EAST,
  WEST
} E_Direction_t;
typedef enum MSG
{
  SAFE,
  SLOWDOWN,
  FASTER
} E_MSG_t;

typedef struct Vehicle
{
  float x;
  float y;
  uint8_t ID;
  E_MSG_t msg;
  double Angle;
  E_State_t action;
  float Velocity;
  float Accelration;
  E_Direction_t Direction;
} S_Connected_Car_t;
/************ Global Variables Section**************************/

// Motor control pins
#define motor2EnablePin 4
#define motor1EnablePin 15
#define motor1Pin1 5   // Motor 1 forward
#define motor1Pin2 2   // Motor 1 backward
#define motor2Pin1 19  // Motor 2 forward
#define motor2Pin2 18  // Motor 2 backward
#define WINDOW_SIZE 10 // Window size for the moving average filter

// Global variables for topic and QoS
#define MAIN_VEHICLE "MainVehicle/Data"
#define DUMMY_VEHICLE "DummyVehicle/Data"
#define MQTT_PORT 1883

String myString;
float yaw = 0.0;
bool t_flag = false;
Adafruit_MPU6050 mpu;
const char key = 0x50; // Encryption/Decryption key
String incomingDataString;
BluetoothSerial SerialBT;
float displacement = 0.0;
const int maxDataSize = 255; // Maximum size for the received string
unsigned long start_time = 0;
unsigned long previousTime = 0;
float total_displacement = 0.0;
double ax_samples[WINDOW_SIZE] = {0};
int sample_index = 0;

S_Connected_Car_t DummyVehicle = {0.0, 0.0, 2, SAFE, 90.0, NORMAL, 0.0, 0.0, NORTH};
S_Connected_Car_t MainVehicle = {0.0, 0.0, 1, SAFE, 90.0, NORMAL, 0.0, 0.0, NORTH};

float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gyroXcal = 0, gyroYcal = 0, gyroZcal = 0;

uint8_t CarSpeed = 0;
String Current_Bluetooth_CMD = "";
const char *ssid = "Koks";            // Your WiFi network name (SSID)
const char *password = "Karim*11111"; // Your WiFi network password
// const char *mqtt_server = "tcp://test.mosquitto.org";
// const char *mqtt_server = "ssl://test.mosquitto.org";
const char *mqtt_server = "test.mosquitto.org";

WiFiClient espClient;           // WiFi client object for network operations
PubSubClient client(espClient); // MQTT client object using the WiFi client

// Define task handles
TaskHandle_t CoordinatesTaskHandle = NULL;
TaskHandle_t ControlMotorsTaskHandle = NULL;
// TaskHandle_t MPU6050_TaskHandle = NULL;
bool ReceivingMainVehicleData_Done = false;
/*************** Functions Prototype Section *********/

void readSensorData(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp);
double calculateAngle(float Current_X, float Current_Y, float otherX, float otherY);
void string_to_struct(const char *receivedData, S_Connected_Car_t *data);
// void struct_to_string(S_Connected_Car_t *data, char *buffer, size_t buffer_size);
void struct_to_string(S_Connected_Car_t *data, char *buffer);
float calculateDistance(float myX, float myY, float otherX, float otherY);
void Calculate_Accelration_Velocity_Displacement_Yaw();
void printVehicle(const S_Connected_Car_t &vehicle);
String xor_encrypt_decrypt(const String &input);
void processBluetoothData(String data);
float normalize_angle(float angle);
E_Direction_t CurrentDirection();
// void CoordinatesCalculations();
void getInitialCoordinates();
void initializeMPU();
void printResults();

void controlMotors(void *parameter);
void MotorsSetup();
void stopMotors();
void turnRight();
void turnLeft();
void backward(uint8_t speed);
void forward(uint8_t speed);

void CoordinatesCalculationsTask(void *pvParameters);
// void MPU6050_Task(void *pvParameters);
void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void publishDummyVehicle();

/*************** Functions Definition Section *********/

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("DUMMY");
  DummyVehicle.ID = 2;
  Serial.println("\nsys Started ");
  MotorsSetup();
  stopMotors();

  getInitialCoordinates();

  // Create control motors task
  if (xTaskCreate(controlMotors, "ControlMotors", 4096, NULL, 1, &ControlMotorsTaskHandle) != pdPASS)
  {
    Serial.println("Failed to create ControlMotors task");
  }
  initializeMPU();

  setup_wifi();                             // Call function to connect to WiFi
  client.setServer(mqtt_server, MQTT_PORT); // Correct MQTT server address
  client.setCallback(callback);

  // Create MPU6050 task
  // if (xTaskCreate(MPU6050_Task, "MPU6050_Task", 8192, NULL, 3, &MPU6050_TaskHandle) != pdPASS)
  // {
  //   Serial.println("Failed to create MPU6050 task");
  // }

  // Create coordinates calculations task
  if (xTaskCreate(CoordinatesCalculationsTask, "CoordinatesCalculationsTask", 8192, NULL, 2, &CoordinatesTaskHandle) != pdPASS)
  {
    Serial.println("Failed to create CoordinatesCalculationsTask task");
  }
  Serial.println("STARTED");
  SerialBT.println("STARTED");
  previousTime = millis();
}

void loop()
{
   // Monitor free heap memory
  // Serial.print("Free heap: ");
  // Serial.println(ESP.getFreeHeap());

  // Monitor stack space for tasks
  // if (ControlMotorsTaskHandle != NULL) {
  //   Serial.print("Min free stack for ControlMotorsTask: ");
  //   Serial.println(uxTaskGetStackHighWaterMark(ControlMotorsTaskHandle));
  // }
  // if (MPU6050_TaskHandle != NULL) {
  //   Serial.print("Min free stack for MPU6050_Task: ");
  //   Serial.println(uxTaskGetStackHighWaterMark(MPU6050_TaskHandle));
  // }
  // if (CoordinatesTaskHandle != NULL) {
  //   Serial.print("Min free stack for CoordinatesCalculationsTask: ");
  //   Serial.println(uxTaskGetStackHighWaterMark(CoordinatesTaskHandle));
  // }
  if (!client.connected())
  {
    reconnect(); // Attempt to reconnect if not connected
  }

  client.loop(); // Process incoming and outgoing MQTT messages

  // Publish a DummyVehicleBuffer message
  publishDummyVehicle();
  delay(10);
}

void publishDummyVehicle()
{
  char dummyVehicleBuffer[255] = "Message from ESP32 to RPI";
  struct_to_string(&DummyVehicle, dummyVehicleBuffer);
  if (!client.publish(DUMMY_VEHICLE, dummyVehicleBuffer))
  {
    Serial.println("Publish failed");
  }
  else
  {
    Serial.println("Publish succeeded");
    SerialBT.println("Publish succeeded");
    Serial.print("Message sent [");
    SerialBT.print("Message sent [");
    Serial.print(DUMMY_VEHICLE);
    SerialBT.print(DUMMY_VEHICLE);
    Serial.print("] ");
    SerialBT.print("] ");
    Serial.println(String(dummyVehicleBuffer));
    SerialBT.println(String(dummyVehicleBuffer));
  }
}
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  SerialBT.print("Connecting to ");
  Serial.println(ssid);
  SerialBT.println(ssid);

  WiFi.begin(ssid, password); // Connect to the WiFi network
  unsigned long startTime = millis();
  unsigned long timeout = 30000; // 30 seconds timeout

  while (WiFi.status() != WL_CONNECTED && (millis() - startTime < timeout))
  {
    delay(500);
    Serial.print(".");
    SerialBT.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("");
    SerialBT.println("");
    Serial.println("WiFi connected");
    SerialBT.println("WiFi connected");
    Serial.print("IP address: ");
    SerialBT.print("IP address: ");
    Serial.println(WiFi.localIP());
    SerialBT.println(WiFi.localIP());
  }
  else
  {
    Serial.println("");
    SerialBT.println("");
    Serial.println("Failed to connect to WiFi");
    SerialBT.println("Failed to connect to WiFi");
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  SerialBT.print("Message arrived [");
  Serial.print(topic);
  SerialBT.print(topic);
  Serial.print("] ");
  SerialBT.print("] ");

  String message;
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  Serial.println(message);
  SerialBT.println(message);

  if (strcmp(topic, MAIN_VEHICLE) == 0)
  {
    char mainVehicleBuffer[255];
    message.toCharArray(mainVehicleBuffer, sizeof(mainVehicleBuffer));
    string_to_struct(mainVehicleBuffer, &MainVehicle);
    ReceivingMainVehicleData_Done = true;
  }
}

void reconnect()
{
  while (!client.connected())
  {
    if (client.connect("ESP32Client"))
    {
      if (!client.subscribe(MAIN_VEHICLE))
      {
        Serial.println("Subscription failed");
      }
      else
      {
        Serial.println("Subscription succeeded");
      }
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 100 milliseconds");
      delay(100);
    }
  }
}

// Function to print the Vehicle struct members
void printVehicle(const S_Connected_Car_t &vehicle)
{
  Serial.print("Vehicle Information:\n");
  // Serial.print("  ID: ");
  // Serial.println(vehicle.ID);
  Serial.println("-----------");
  Serial.print("  x: ");
  Serial.println(vehicle.x);
  Serial.print("  y: ");
  Serial.println(vehicle.y);

  // Serial.print("  Message: ");
  // switch (vehicle.msg)
  // {
  // case SAFE:
  //   Serial.println("SAFE");
  //   break;
  // case SLOWDOWN:
  //   Serial.println("SLOWDOWN");
  //   break;
  // case FASTER:
  //   Serial.println("FASTER");
  //   break;
  // default:
  //   Serial.println("UNKNOWN");
  //   break;
  // }

  // Serial.print("  Angle: ");
  // Serial.println(vehicle.Angle * 180 / PI);

  // Serial.print("  Action: ");
  // switch (vehicle.action)
  // {
  // case UNKNOWN:
  //   Serial.println("UNKNOWN");
  //   break;
  // case NORMAL:
  //   Serial.println("NORMAL");
  //   break;
  // case DANGER:
  //   Serial.println("DANGER");
  //   break;
  // case FOCUS:
  //   Serial.println("FOCUS");
  //   break;
  // default:
  //   Serial.println("UNKNOWN");
  //   break;
  // }

  // Serial.print("  Velocity: ");
  // Serial.println(vehicle.Velocity);
  // Serial.print("  Acceleration: ");
  // Serial.println(vehicle.Accelration);

  Serial.print("  Direction: ");
  switch (vehicle.Direction)
  {
  case NORTH:
    Serial.println("NORTH");
    break;
  case SOUTH:
    Serial.println("SOUTH");
    break;
  case EAST:
    Serial.println("EAST");
    break;
  case WEST:
    Serial.println("WEST");
    break;
  default:
    Serial.println("UNKNOWN");
    break;
  }
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

/**
 * @brief Converts a structure to a string.
 *
 * This function serializes the data from a `S_Connected_Car_t` structure into a string format,
 * which can be sent over UART or used for other purposes.
 *
 * @param [in] data Pointer to the structure containing the data to be serialized.
 * @param [out] buffer Pointer to the buffer where the resulting string will be stored.
 */
// void struct_to_string(S_Connected_Car_t *data, char *buffer, size_t buffer_size)
// {
//   snprintf(buffer, buffer_size,
//            "%d,%f,%f,%f,%d,%f,%d,%d\n",
//            data->ID, data->x, data->y,
//            data->Accelration, data->Angle, data->Velocity,
//            data->Direction, data->msg);
// }
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
void struct_to_string(S_Connected_Car_t *data, char *buffer)
{
  int bytes_written = 0;

  // Serialize the structure fields into the buffer
  bytes_written += sprintf(buffer + bytes_written, "%d,%.2f,%.2f,%d,%d,%d,%d,%d\n",
                           data->ID, data->x, data->y,
                           (int32_t)data->Accelration, (int32_t)data->Angle, (int32_t)data->Velocity,
                           (int32_t)data->Direction, (int32_t)data->msg);
}
void readSensorData(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp)
{
  mpu.getEvent(accel, gyro, temp);
  // Apply calibration offsets
  // float ax = accel.acceleration.x - ax_offset;
  // float ay = accel.acceleration.y - ay_offset;
  // float az = accel.acceleration.z - az_offset;

  // float gx = gyro.gyro.x - gyroXcal;
  // float gy = gyro.gyro.y - gyroYcal;
  // float gz = gyro.gyro.z - gyroZcal;
}

void printResults()
{
  // Serial.print("A : ");
  // Serial.print(DummyVehicle.Accelration);
  // Serial.print(" V : ");
  // Serial.print(DummyVehicle.Velocity);
  // Serial.print(" D: ");
  // Serial.print(displacement);
  Serial.print(" m, Yaw: ");
  Serial.print(yaw * 180.0 / PI); //() Convert radians to degrees
  Serial.println(" degrees");

  // SerialBT.print("A : ");
  // SerialBT.print(DummyVehicle.Accelration);
  // SerialBT.print(" V : ");
  // SerialBT.print(DummyVehicle.Velocity);
  // SerialBT.print(" Displacement : ");
  // SerialBT.print(displacement);
  SerialBT.print(" m, Yaw: ");
  SerialBT.print(yaw); // * 180.0 / PI //() Convert radians to degrees
  SerialBT.println(" degrees");
}
/**
 * @brief Calculates the moving average of the given samples.
 *
 * @param new_sample The new sample to add to the moving average.
 * @param samples The array holding the past samples.
 * @param size The size of the samples array.
 * @return double The updated moving average.
 */
double moving_average(double new_sample, double *samples, int size)
{
  samples[sample_index % size] = new_sample;
  sample_index++;
  double sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += samples[i];
  }
  return sum / size;
}
/**
 * @brief Normalizes an angle to the range [0, 360] degrees.
 *
 * @param angle The angle to normalize.
 * @return float The normalized angle.
 */
float normalize_angle(float angle)
{
  // Serial.print("Angle before mapping : ");
  // Serial.println(angle);

  // Serial.print("Yaw Inside Normalizing function : ");
  // Serial.print(yaw);
  // Serial.println(" degrees");

  // // Use modulo to wrap the angle within 0 to 360
  // angle = fmod(angle, 360.0);

  // // Ensure positive angle
  // if (angle < 0)
  //   angle += 360.0;
  while (angle < 0)
  {
    angle += 360.0;
  }
  while (angle >= 360.0)
  {
    angle -= 360.0;
  }
  // Serial.print("Angle after mapping : ");
  // Serial.println(angle);
  // delay(1000);
  return angle;
}

void Calculate_Accelration_Velocity_Displacement_Yaw()
{
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

  sensors_event_t accel, gyro, temp;
  readSensorData(&accel, &gyro, &temp);
  float ax = accel.acceleration.x - ax_offset;
  // Calculate acceleration in m/s^2
  // DummyVehicle.Accelration = accel.acceleration.x;
  DummyVehicle.Accelration = moving_average(ax, ax_samples, WINDOW_SIZE);
  // Integrate acceleration to get velocity
  DummyVehicle.Velocity = fabs(DummyVehicle.Accelration) * elapsedTime;

  // Integrate velocity to get displacement
  displacement = DummyVehicle.Velocity * elapsedTime;

  // Get the yaw angle (gyroscope z-axis)
  // Integrate gyroscope data to get yaw angle
  float gz = gyro.gyro.z - gyroZcal;

  yaw += gz * elapsedTime; // Integrate to get angle in degrees
  // yaw = 0;
  // Serial.print("Yaw before Normalizing : ");
  // Serial.print(yaw);
  // Serial.println(" degrees");

  yaw = normalize_angle(yaw); // Normalize to 0-360 degrees
  // Serial.print("Yaw After Normalizing : ");
  // Serial.println(yaw);
  // yaw = normalize_angle(gyro.gyro.z); // In radians/sec

  printResults();
  previousTime = currentTime;
}



void calibrate_sensor()
{
  Serial.println("Calibrating...");
  SerialBT.println("Calibrating...");

  int samples = 1000;
  for (int i = 0; i < samples; i++)
  {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    ax_offset += accel.acceleration.x;
    ay_offset += accel.acceleration.y;
    az_offset += accel.acceleration.z;

    gyroXcal += gyro.gyro.x;
    gyroYcal += gyro.gyro.y;
    gyroZcal += gyro.gyro.z;

    delay(2); // Small delay to ensure accurate readings
  }

  ax_offset /= samples;
  ay_offset /= samples;
  az_offset /= samples;

  gyroXcal /= samples;
  gyroYcal /= samples;
  gyroZcal /= samples;

  Serial.println("Calibration complete.");
  SerialBT.println("Calibration complete.");
  Serial.print("Accel Offsets: ");
  Serial.print(ax_offset);
  Serial.print(", ");
  Serial.print(ay_offset);
  Serial.print(", ");
  Serial.println(az_offset);
  Serial.print("Gyro Offsets: ");
  Serial.print(gyroXcal);
  Serial.print(", ");
  Serial.print(gyroYcal);
  Serial.print(", ");
  Serial.println(gyroZcal);
}

void initializeMPU()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    SerialBT.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  SerialBT.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  calibrate_sensor();

  delay(100);
}
void start_timer()
{
  start_time = millis();
}
/**
 * @brief Determines the current direction based on the yaw angle.
 *
 * This function checks the current yaw angle and returns the corresponding cardinal direction.
 *
 * @return E_Direction_t The current direction (NORTH, SOUTH, WEST, EAST, or UNKNOWN).
 */
E_Direction_t CurrentDirection()
{
  // yaw = 0;
  // double angle = yaw * 180.0 / PI;
  // Serial.print(" m, Yaw: ");
  // Serial.print(angle); //() Convert radians to degrees
  // Serial.println(" degrees");
  // SerialBT.print("Direction : ");
  if ((yaw >= 345 || yaw <= 15)) // 0
  {
    Serial.println("NORTH");
    SerialBT.println("NORTH");
    return NORTH;
  }
  else if ((yaw >= 165 && yaw <= 195)) // 180
  {
    Serial.println("SOUTH");
    SerialBT.println("SOUTH");
    return SOUTH;
  }
  else if ((yaw >= 75 && yaw <= 105)) // 90
  {
    Serial.println("WEST");
    SerialBT.println("WEST");
    return WEST;
  }
  else if ((yaw >= 255 && yaw <= 285)) // 270
  {
    Serial.println("WEST");
    SerialBT.println("EAST");
    return EAST;
  }
  SerialBT.println("UNKNOWN DIRECTION!!!!!!");
}

/*
 * @Fn         calculateDistance
 * @brief      Calculates the distance between two vehicles based on their coordinates.
 * @param [in] myX, myY: Coordinates of the current vehicle.
 * @param [in] otherX, otherY: Coordinates of another vehicle.
 * @retval     Distance between the two vehicles.
 */
float calculateDistance(float myX, float myY, float otherX, float otherY)
{
  return sqrt(pow((otherX - myX), 2) + pow((otherY - myY), 2));
}

/**
 * @brief Calculates the angle between two vehicles based on their coordinates.
 *
 * This function computes the angle between the current vehicle and another vehicle
 * using their respective coordinates. It accounts for different cases of the coordinates
 * to handle vertical and horizontal alignments and calculates the angle accordingly.
 *
 * @param [in] Current_X X-coordinate of the current vehicle.
 * @param [in] Current_Y Y-coordinate of the current vehicle.
 * @param [in] otherX X-coordinate of the other vehicle.
 * @param [in] otherY Y-coordinate of the other vehicle.
 *
 * @retval Angle between the two vehicles in radians, always non-negative.
 */
double calculateAngle(float Current_X, float Current_Y, float otherX, float otherY)
{
  double result = 0.0;                               // Variable to store the result
  double denominator = (double)(otherX - Current_X); // Difference in X-coordinates
  double numerator = (double)(otherY - Current_Y);   // Difference in Y-coordinates

  if (denominator == 0.0)
  {
    // Handle vertical alignment
    if (numerator > 0.0)
      result = M_PI / 2; // Angle is 90 degrees
    else if (numerator < 0.0)
      result = (3 * M_PI) / 2; // Angle is 270 degrees
    else
      result = NAN; // Undefined, angle is indeterminate if both differences are zero
  }
  else
  {
    // Handle general case
    if (MainVehicle.Direction != DummyVehicle.Direction)
      result = atan(denominator / numerator); // Calculate angle for different directions
    else
      result = atan(numerator / denominator); // Calculate angle for same directions
  }

  return fabs(result); // Return the absolute value of the angle
}

// void MPU6050_Task(void *pvParameters)
// {
//   while (true)
//   {
    
//     // Ensure the task is efficient and does not block other tasks
//     vTaskDelay(10 / portTICK_PERIOD_MS); // Adjust delay based on your needs
//   }
// }
/*
 * @Fn         CoordinatesCalculations
 * @brief      Performs calculations for coordinates, angle, displacement and velocity.
 * @note       Utilizes functions for calculating direction, angle, distance, and velocity.
 */
// CoordinatesTask function definition
void CoordinatesCalculationsTask(void *pvParameters)
{
  while (true)
  {
    if (ReceivingMainVehicleData_Done == true)
    {
      Calculate_Accelration_Velocity_Displacement_Yaw();
      DummyVehicle.y += displacement * cos(0);
      DummyVehicle.x += displacement * sin(0);
      DummyVehicle.Direction = CurrentDirection();
      // DummyVehicle.Direction = CurrentDirection();
      DummyVehicle.Angle = 0; // calculateAngle(DummyVehicle.x, DummyVehicle.y, MainVehicle.x, MainVehicle.y); //* (180 / M_PI) // Angle between my car and the other car
      // Ensure the task is efficient and does not block other tasks
      vTaskDelay(10 / portTICK_PERIOD_MS); // Adjust delay based on your needs
    }
    // Ensure the task is efficient and does not block other tasks
    vTaskDelay(10 / portTICK_PERIOD_MS); // Adjust delay based on your needs
  }
}

// void CoordinatesCalculations()
// {
//   Calculate_Accelration_Velocity_Displacement_Yaw();
//   DummyVehicle.y += displacement * cos(yaw);
//   DummyVehicle.x += displacement * sin(yaw);

//   DummyVehicle.Direction = CurrentDirection();
//   DummyVehicle.Angle = calculateAngle(DummyVehicle.x, DummyVehicle.y, MainVehicle.x, MainVehicle.y); //* (180 / M_PI) // Angle between my car and the other car
// }

void getInitialCoordinates()
{
  String data_from_bluetooth = "";

  // Read data until we get a string that starts with 'g'
  while (true)
  {
    SerialBT.println("waiting for initial coordinations X,Y ");
    data_from_bluetooth = SerialBT.readStringUntil('\n'); // Read data until newline character
    if (data_from_bluetooth.length() > 0 && data_from_bluetooth[0] == 'g')
    {
      Serial.println(data_from_bluetooth);
      break; // Exit loop if the string starts with 'g'
    }
  }

  // Buffer to store the data for parsing
  char buffer[128] = {0};                                  // Initialize buffer with zeros
  data_from_bluetooth.toCharArray(buffer, sizeof(buffer)); // Copy the string to the buffer
  char ack = 0;
  // Parse the data from the buffer and assign to DummyVehicle's x and y
  sscanf(buffer, "%c,%f,%f", &ack, &DummyVehicle.x, &DummyVehicle.y);

  // For debugging: print the parsed values
  SerialBT.print("X: ");
  SerialBT.println(DummyVehicle.x);
  SerialBT.print("Y: ");
  SerialBT.println(DummyVehicle.y);
}

String xor_encrypt_decrypt(const String &input)
{
  String output = "";
  for (size_t i = 0; i < input.length(); ++i)
  {
    output += char(input.charAt(i) ^ key);
  }
  return output;
}

// Function to control motors based on Bluetooth commands
void controlMotors(void *parameter)
{
  String command = "";
  while (true)
  {
    if (ReceivingMainVehicleData_Done == true)
    {
      if (SerialBT.available())
      {
        command = SerialBT.readStringUntil('\n'); // Read command from Bluetooth
        processBluetoothData(command);            // Process the command
      }
      vTaskDelay(10 / portTICK_PERIOD_MS); // Add a small delay to avoid tight looping
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Add a small delay to avoid tight looping
  }
}

void MotorsSetup()
{
  pinMode(motor2EnablePin, OUTPUT);
  pinMode(motor1EnablePin, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
}

void forward(uint8_t speed)
{

  // analogWrite(motor1EnablePin, speed);
  // analogWrite(motor2EnablePin, speed);
  analogWrite(motor1EnablePin, 150);
  analogWrite(motor2EnablePin, 150);

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void backward(uint8_t speed)
{

  // analogWrite(motor1EnablePin, speed);
  // analogWrite(motor2EnablePin, speed);

  analogWrite(motor1EnablePin, 150);
  analogWrite(motor2EnablePin, 150);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void turnRight()
{
  analogWrite(motor2EnablePin, 100);
  analogWrite(motor1EnablePin, 200);

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void turnLeft()
{
  analogWrite(motor2EnablePin, 200);
  analogWrite(motor1EnablePin, 100);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void stopMotors()
{
  CarSpeed = 0;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}
void processBluetoothData(String data)
{
  // CarSpeed = 200;
  if (data == "t")
  {
    SerialBT.println("RESTARTING");
    delay(100);
    ESP.restart();
  }
  else if (data == "f")
  {
    forward(CarSpeed);
    SerialBT.println("Moving forward");
  }
  else if (data == "b")
  {
    backward(CarSpeed);
    SerialBT.println("Moving backward");
  }
  else if (data == "r")
  {
    turnRight();
    SerialBT.println("Turning right");
  }
  else if (data == "l")
  {
    turnLeft();
    SerialBT.println("Turning left");
  }
  else if (data == "s")
  {
    stopMotors();
    SerialBT.println("Stopping motors");
  }
  else
  {
    Serial.print("UNWANTED DATA : ");
    Serial.println(data);
  }
}