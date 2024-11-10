/*************************** Includes Section **************************/
#include <esp_now.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <string.h>

/************************ Global Variables Section *********************/
BluetoothSerial SerialBT;
String myString ;//= "0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18";
uint8_t broadcastAddress[] = {0xFC, 0xB4, 0x67, 0xC3, 0x40, 0xBA};
int data[30];
esp_now_peer_info_t peerInfo;
String incomingDataString;
const char key = 0x50; // Encryption/Decryption key
const uint8_t maxNumCount = 10;
float data_x_y[maxNumCount];
String decrypted ;
/*********************** Function Prototypes Section *******************/
void split(String text);
void show();
String encrypt(const String &input);
String decrypt(const String &input);
String parseReceived_X_Y(String receivedData);
bool split_x_y(String input);
bool isValidFormat(String number);

/********************** Callback Functions Section *********************/
// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // Create a buffer to hold the received data
  char buffer[len + 1]; // Add 1 for the null terminator
  memcpy(buffer, incomingData, len);
  buffer[len] = '\0'; // Null-terminate the buffer

  // Update the global incomingDataString
  incomingDataString = String(buffer);
  decrypted = decrypt(incomingDataString);

}

// Callback when data is sent
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
//{
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//}

/**************************** Setup Function ****************************/
void setup()
{
  // Init Serial Monitor
  Serial.begin(9600);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  SerialBT.begin("RaspberryPI"); // Bluetooth device name

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback functions
  //esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  SerialBT.println("ESP IS STARTING");
  // Awaiting ACK from receiver
  char ACK = '0';
  while (ACK != '1')
  {
    String x_y;
    // SerialBT.println("Waiting for initial X & Y");
    if (SerialBT.available())
    {
      bool isvalid = 0;
      while (!isvalid)
      {
        x_y = SerialBT.readStringUntil('\n');
        isvalid = split_x_y(x_y);
      }
      String modified_x_y = parseReceived_X_Y(x_y);
      String All='g'+String(',')+modified_x_y;
      Serial.println(All);
      SerialBT.println(All);
    }
    ACK = Serial.read();
  }
  SerialBT.println("RB received initials");
}

/**************************** Loop Function *****************************/
void loop()
{
  if (Serial.available())
  {                                          // Check if data is available to read
    myString = Serial.readStringUntil('\n'); // Read the message until newline character
    String encrypted = encrypt(myString);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)encrypted.c_str(), encrypted.length() + 1);
  }

  if(Serial.readStringUntil('\n') == "r")
  {
    SerialBT.println("ESP is restarting")
     ESP.restart();
  }
  else
  {
      Serial.println(decrypted);
  }

 
   
 // encrypted="";
  //SerialBT.println(myString);
  //split(myString);
  //show();
  //myString="";
  delay(500);
}

/************************ Utility Functions *****************************/
void split(String input)
{
  String number = "";
  uint8_t numCount = 0;
  for (int i = 0; i < input.length(); i++)
  {
    char c = input.charAt(i);
    if (c != ',')
    {
      number += c;
    }
    else
    {
      data[numCount++] = number.toInt(); // Store the number in the array and increment the counter
      number = "";
    }
  }
  // Store the last number if there's no comma at the end of the string
  if (number != "")
  {
    data[numCount++] = number.toInt();
  }
}

void show()
{
  SerialBT.println("-----------------------------------------------------------");
  SerialBT.print("Action: ");
  SerialBT.println(data[0]);
  SerialBT.print("ID: ");
  SerialBT.println(data[1]);
  SerialBT.print("X: ");
  SerialBT.println(data[2]);
  SerialBT.print("Y: ");
  SerialBT.println(data[3]);
  SerialBT.print("Acceleration: ");
  SerialBT.println(data[4]);
  SerialBT.print("Angle: ");
  SerialBT.println(data[5]);
  SerialBT.print("Velocity: ");
  SerialBT.println(data[6]);
  SerialBT.print("Direction: ");
  SerialBT.println(data[7]);
  SerialBT.print("Message: ");
  SerialBT.println(data[8]);
  SerialBT.print("My Destination: ");
  SerialBT.println(data[9]);
  SerialBT.print("Other Destination: ");
  SerialBT.println(data[10]);
  SerialBT.print("My time: ");
  SerialBT.println(data[11]);
  SerialBT.print("Other time: ");
  SerialBT.println(data[12]);
  SerialBT.print("Current Distination: ");
  SerialBT.println(data[13]);
  SerialBT.print("point_x ");
  SerialBT.println(data[14]);
  SerialBT.print("point_Y ");
  SerialBT.println(data[15]);
  SerialBT.print("Side distance ");
  SerialBT.println(data[16]);
  SerialBT.print("Front distance ");
  SerialBT.println(data[17]);
  SerialBT.print("safe distance ");
  SerialBT.println(data[18]);
}

String decrypt(const String &input)
{
  String output = "";
  for (size_t i = 0; i < input.length(); ++i)
  {
    output += char(input.charAt(i) ^ key);
  }
  return output;
}

String encrypt(const String &input)
{
  String output = "";
  for (size_t i = 0; i < input.length(); ++i)
  {
    output += char(input.charAt(i) ^ key);
  }
  return output;
}

String parseReceived_X_Y(String receivedData)
{
  static char result[12]; // Assuming 5 digits for each number plus a comma and null terminator
  char *endPtr;
  double x = strtod(receivedData.c_str(), &endPtr);
  receivedData = receivedData.substring(endPtr - receivedData.c_str() + 1);
  double y = strtod(receivedData.c_str(), &endPtr);
  receivedData = receivedData.substring(endPtr - receivedData.c_str() + 1);

  long long int X_multiplier = x * 10000000;
  uint32_t last_5_digits_X = X_multiplier % 100000;

  long long int Y_multiplier = y * 10000000;
  uint32_t last_5_digits_Y = Y_multiplier % 100000;

  sprintf(result, "%d,%d", last_5_digits_X, last_5_digits_Y); // Ensure leading zeros are included if needed
                                                               // 0987654
  return String(result);
}

bool split_x_y(String input)
{
  String number = "";
  uint8_t numCount = 0;

  for (int i = 0; i < input.length(); i++)
  {
    char c = input.charAt(i);
    if (c != ',')
    {
      number += c;
    }
    else
    {
      if (isValidFormat(number))
      {
        data_x_y[numCount++] = number.toFloat(); // Store the number in the array and increment the counter
      }
      else
      {
        SerialBT.println("Data does not match the required format: xx.xxxxxxx");
        return false;
      }
      number = "";
    }
  }
  // Store the last number if there's no comma at the end of the string
  if (number != "")
  {
    if (isValidFormat(number))
    {
      data_x_y[numCount++] = number.toFloat();
    }
    else
    {
      SerialBT.println("Data does not match the required format: xx.xxxxxxx");
      return false;
    }
  }

  // Print the stored numbers for verification
  for (uint8_t i = 0; i < numCount; i++)
  {
    SerialBT.println(data_x_y[i], 7);
  }
  return true;
}

bool isValidFormat(String number)
{
  int pointIndex = number.indexOf('.');
  if (pointIndex == -1)
  {
    return false;
  }
  int digitsBeforePoint = pointIndex;
  int digitsAfterPoint = number.length() - pointIndex - 1;

  return (digitsBeforePoint == 2 && digitsAfterPoint == 7);
}/*************************** Includes Section **************************/