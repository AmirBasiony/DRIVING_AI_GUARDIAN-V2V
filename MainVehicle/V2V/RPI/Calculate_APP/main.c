
#include "Algo/PositionCalculations.h"
#include "connections/UART_Struct.h"
#include "Algo/intersection.h"
#include "Threads/Thread_1.h"
#include "Threads/Thread_2.h"
#include "Threads/Thread_3.h"
#include "Algo/SuddenBrake.h"
#include "MQTT/MQTT.h"

E_State_t danger = NORMAL;
uint32 usb0_filestream;
uint32 msgid_ACT_receive;
uint32 msgid_ACT_send;
// Bool_t DoneGettintInitials_viaUART = False;

// Mutex variables
pthread_mutex_t yaw_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t acceleration_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t displacement_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t MainVehicle_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DummyVehicle_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t intersection_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t suddenbrake_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t prev_distance_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t delta_d_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t GPS_X_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t GPS_Y_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DoneGettintInitials_viaUART_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief      Waits for initial coordinates (X, Y) via UART and sends an acknowledgment.
 * @note       This function waits for the initial coordinates, receives them over UART, and then sends an acknowledgment character ('1') back.
 */
void WaitFor_XY(void)
{
    printf("\nwaiting for initial coordinations X,Y \n");
    _UART_Receive_initials();
    char aknowledge[1];
    aknowledge[0] = '1';
    // // Send the acknowledgment character over UART
    _UART_send_data(usb0_filestream, aknowledge, 1);
}

/**
 * @brief      Initializes the system.
 * @details    This function performs the following tasks:
 *             1. Initiates UART communication.
 *             2. Creates IDs for data and action Processes.
 *             3. Gets the initial values of the car (X, Y) from UART - given to esp32 via bluetooth.
 */
void init()
{
    // Lock all necessary mutexes to ensure thread safety during initialization
    pthread_mutex_lock(&MainVehicle_mutex);
    pthread_mutex_lock(&DummyVehicle_mutex);
    pthread_mutex_lock(&GPS_X_mutex);
    pthread_mutex_lock(&GPS_Y_mutex);
    pthread_mutex_lock(&DoneGettintInitials_viaUART_mutex);

#ifdef SCENARIOS
    // Initialize MainVehicle and DummyVehicle with predefined values ------------> INTERSECTION

    MainVehicle.x = 0;
    MainVehicle.y = 0;
    MainVehicle.Direction = NORTH;
    MainVehicle.Velocity = NORMAL_SPEED;
    MainVehicle.Accelration = 5.6;

    DummyVehicle.ID = 2;
    DummyVehicle.Direction = WEST;
    DummyVehicle.x = 0;
    DummyVehicle.y = 100;
    DummyVehicle.Accelration = 0;
    DummyVehicle.Velocity = 0;

    // Initialize MainVehicle and DummyVehicle with predefined values ------------> REARSCENARIOS

    // MainVehicle.x = 0;
    // MainVehicle.y = 100;
    // MainVehicle.Direction = NORTH;
    // MainVehicle.Velocity = 0;

    // DummyVehicle.ID = 2;
    // DummyVehicle.Direction = NORTH;
    // DummyVehicle.x = 0;
    // DummyVehicle.y = 0;
    // DummyVehicle.Accelration = 0;
    // DummyVehicle.Velocity = NORMAL_SPEED;

    // // Initialize MainVehicle and DummyVehicle with predefined values ----------> FRONTSCENARIOS

    // MainVehicle.x = 0;
    // MainVehicle.y = 0;
    // MainVehicle.Direction = NORTH;
    // MainVehicle.Velocity = NORMAL_SPEED;

    // DummyVehicle.ID = 2;
    // DummyVehicle.Direction = NORTH;
    // DummyVehicle.x = 0;
    // DummyVehicle.y = 80;
    // DummyVehicle.Accelration = 0;
    // DummyVehicle.Velocity = 0;

#else
    // Initialize UART and get initial coordinates if SCENARIOS is not defined
    // usb0_filestream = UART_init(UART_DEV);
    // WaitFor_XY();
    char xy[20];
    printf("Enter X,Y Coordinates : ");
    fgets(xy, sizeof(xy), stdin);
    sscanf(xy, "%e,%e", &MainVehicle.x, &MainVehicle.y);
    printf("initial longitude's last 5 digits : %f\ninitial latitude's last 5 digits : %f\n", MainVehicle.x, MainVehicle.y);
#endif
    // Set MainVehicle ID
    MainVehicle.ID = 1;

    // Create message queue IDs for action process
    msgid_ACT_send = msgget(2, IPC_CREAT | 0666);
    msgid_ACT_receive = msgget(3, IPC_CREAT | 0666);
    // DoneGettintInitials_viaUART = True;
    MQTT_initialize();
    // Unlock the mutexes after initialization is done
    pthread_mutex_unlock(&DoneGettintInitials_viaUART_mutex);
    pthread_mutex_unlock(&MainVehicle_mutex);
    pthread_mutex_unlock(&DummyVehicle_mutex);
    pthread_mutex_unlock(&GPS_X_mutex);
    pthread_mutex_unlock(&GPS_Y_mutex);
}

/**
 * @brief      print the information of the system.
 * @details    This function prints the following :
 *             1. intersection or suddenbrake whichever is active
 *             2. acutal calculations of my vehicle.
 *             3. acutal calculations of the DummyVehicle vehicle.
 */
void printFormattedOutput(float *yaw)
{
    const char *leftHeader = "\n------------  Intersection -----------";
    const char *middleHeader = "------------ Main Vehicle  -----------";
    const char *rightHeader = "----------- Dummy Vehicle  -----------";
    char leftContent[512];
    char middleContent[512];
    char rightContent[512];

    if (MainVehicle.Direction != DummyVehicle.Direction)
    {
        leftHeader = "\n------------  Intersection -----------";
        snprintf(leftContent, sizeof(leftContent),
                 "State : %s\n"
                 "Message : %s\n"
                 "distance between vehicles : %d\n"
                 "My distance : %d\n"
                 "My time : %.2f\n"
                 "Other distance : %d\n"
                 "Other time : %.2f\n"
                 "Intersection X point : %d\n"
                 "Intersection Y point : %d\n"
                 "Action : %d\n",
                 (danger == DANGER) ? "DANGER" : (danger == FOCUS) ? "Alarm"
                                             : (danger == NORMAL)  ? "Idle"
                                                                   : "Unknown",
                 (MainVehicle.msg == FASTER) ? "FASTER" : (MainVehicle.msg == SLOWDOWN) ? "SLOWDOWN"
                                                      : (danger == NORMAL)              ? "SAFE"
                                                                                        : "Unknown",
                 my_intersection.Current_Dist,
                 my_intersection.My_Dist,
                 my_intersection.Mytime,
                 my_intersection.other_Dist,
                 my_intersection.Othertime,
                 my_intersection.point_X,
                 my_intersection.point_Y,
                 MainVehicle.action);
    }
    else
    {
        leftHeader = "--------  Sudden-Brake-Scape --------";

        snprintf(leftContent, sizeof(leftContent),
                 "State : %s\n"
                 "Message : %d\n"
                 "Front/Rear delta distance : %d\n"
                 "Front/Rear distance : %d\n"
                 "Side distance : %d\n"
                 "Danger Zone : %.2f\n"
                 "Focus Zone : %.2f\n"
                 "Car position : %s\n"
                 "relative_Velocity : %d\n"
                 "Mapped Angle : %.2f\n"
                 "Action : %d\n",
                 (danger == DANGER) ? "DANGER" : (danger == FOCUS) ? "Alarm"
                                             : (danger == NORMAL)  ? "Idle"
                                                                   : "Unknown",
                 MainVehicle.msg,
                 my_suddenbrake.FrontRear_DeltaDistance,
                 my_suddenbrake.FrontRearDistance,
                 my_suddenbrake.SideDistance,

                 ((my_suddenbrake.Car2Position == FrontCar) || (my_suddenbrake.Car2Position == RearCar))           ? FR_DangerZone
                 : ((my_suddenbrake.Car2Position == LeftSideCar) || (my_suddenbrake.Car2Position == RightSideCar)) ? LaneDangerZone
                                                                                                                   : 0.0,

                 ((my_suddenbrake.Car2Position == FrontCar) || (my_suddenbrake.Car2Position == RearCar))           ? FR_FocusZone
                 : ((my_suddenbrake.Car2Position == LeftSideCar) || (my_suddenbrake.Car2Position == RightSideCar)) ? LaneFocusZone
                                                                                                                   : 0.0,

                 (my_suddenbrake.Car2Position == FrontCar) ? "FrontCar" : (my_suddenbrake.Car2Position == RearCar)       ? "RearCar"
                                                                      : (my_suddenbrake.Car2Position == LeftSideCar)     ? "LeftSideCar"
                                                                      : (my_suddenbrake.Car2Position == RightSideCar)    ? "RightSideCar"
                                                                      : (my_suddenbrake.Car2Position == IgnoredPosition) ? "IgnoredPosition"
                                                                                                                         : "UnknownPosition",
                 my_suddenbrake.relative_Velocity,
                 my_suddenbrake.Angle,
                 MainVehicle.action);
    }

    snprintf(middleContent, sizeof(middleContent),
             "ID : %d\n"
             "X : %.2f\n"
             "Y : %.2f\n"
             "Direction : %s\n"
             "Velocity : %d\n"
             "Acceleration : %d cm/s^2\n"
             "YAW : %.2f\n"
             "Angle : %.2f\n"
             "Total Displacement %.6f\n"
             "Message : %s\n",

             MainVehicle.ID,
             MainVehicle.x,
             MainVehicle.y,
             (MainVehicle.Direction == NORTH)   ? "NORTH"
             : (MainVehicle.Direction == SOUTH) ? "SOUTH"
             : (MainVehicle.Direction == EAST)  ? "EAST"
             : (MainVehicle.Direction == WEST)  ? "WEST"
                                                : "UNKNOWN",
             MainVehicle.Velocity,
             MainVehicle.Accelration,
             *yaw,
             isnan(MainVehicle.Angle) ? 0.0 : (MainVehicle.Angle) * (180 / M_PI),
             total_displacement,
             (MainVehicle.msg == FASTER) ? "FAST" : (MainVehicle.msg == SLOWDOWN) ? "SLOW"
                                                : (MainVehicle.msg == SAFE)       ? "SAFE"
                                                                                  : "UNKNOWN");

    snprintf(rightContent, sizeof(rightContent),
             "ID : %d\n"
             "X : %.2f\n"
             "Y : %.2f\n"
             "Direction : %s\n"
             "Velocity : %d\n"
             "Acceleration : %d cm/s^2\n"
             "Angle : %.2f\n"
             "Message : %s\n"
             "Action : %d\n",
             DummyVehicle.ID,
             DummyVehicle.x,
             DummyVehicle.y,
             (DummyVehicle.Direction == NORTH)   ? "NORTH"
             : (DummyVehicle.Direction == SOUTH) ? "SOUTH"
             : (DummyVehicle.Direction == EAST)  ? "EAST"
             : (DummyVehicle.Direction == WEST)  ? "WEST"
                                                 : "UNKNOWN",
             DummyVehicle.Velocity,
             DummyVehicle.Accelration,
             isnan(DummyVehicle.Angle) ? 0.0 : (DummyVehicle.Angle) * (180 / M_PI),
             (DummyVehicle.msg == FASTER) ? "FAST" : (DummyVehicle.msg == SLOWDOWN) ? "SLOW"
                                                 : (DummyVehicle.msg == SAFE)       ? "SAFE"
                                                                                    : "UNKNOWN",
             DummyVehicle.action);

    int leftLines = 10;
    int middleLines = 10;
    int rightLines = 7;

    int maxLines = (leftLines > middleLines) ? ((leftLines > rightLines) ? leftLines : rightLines)
                                             : ((middleLines > rightLines) ? middleLines : rightLines);

    printf("%-38s | %-38s | %-38s\n", leftHeader, middleHeader, rightHeader);
    char *leftPtr = leftContent;
    char *middlePtr = middleContent;
    char *rightPtr = rightContent;
    for (int i = 0; i < maxLines; ++i)
    {
        char leftLine[39] = "";
        char middleLine[39] = "";
        char rightLine[39] = "";

        if (*leftPtr != '\0')
        {
            sscanf(leftPtr, "%38[^\n]\n", leftLine);
            leftPtr += strlen(leftLine) + 1;
        }
        if (*middlePtr != '\0')
        {
            sscanf(middlePtr, "%38[^\n]\n", middleLine);
            middlePtr += strlen(middleLine) + 1;
        }
        if (*rightPtr != '\0')
        {
            sscanf(rightPtr, "%38[^\n]\n", rightLine);
            rightPtr += strlen(rightLine) + 1;
        }

        printf("%-38s | %-38s | %-38s\n", leftLine, middleLine, rightLine);
    }

    printf("-------------------------------------- | -------------------------------------- | --------------------------------------\n");
}

// Example functions to be executed as threads
void *MPU_thread()
{
    MPU_Readings();
    return NULL;
}
void *Calculations_Thread()
{
    Calculate_Data();
    return NULL;
}
void *Scenario_Thread()
{
    Scenario_Handeling();
    return NULL;
}

void *Monitoring_Thread()
{
    while (True)
    {
        if (DoneCalibration)
        {
            pthread_mutex_lock(&yaw_mutex);
            pthread_mutex_lock(&MainVehicle_mutex);
            pthread_mutex_lock(&DummyVehicle_mutex);
            pthread_mutex_lock(&intersection_mutex);
            pthread_mutex_lock(&suddenbrake_mutex);

            printFormattedOutput(&yaw);

            pthread_mutex_unlock(&yaw_mutex);
            pthread_mutex_unlock(&MainVehicle_mutex);
            pthread_mutex_unlock(&DummyVehicle_mutex);
            pthread_mutex_unlock(&intersection_mutex);
            pthread_mutex_unlock(&suddenbrake_mutex);
        }
    }

    return NULL;
}

void *MQTT_send_Thread()
{

    while (True)
        if (DoneCalibration)
        {
            {
                // Send Main Vehcile's data
                char MQTT_message[128];
                MQTT_send_message(MQTT_message, &MainVehicle);
                // usleep(10000); // Sleep for 10 ms to prevent busy-waiting
            }
        }
    return NULL;
}
/**
 * @brief Main function to initialize the system and create threads.
 * @return Exit status of the program.
 */
int main()
{

    init();
    pthread_t tid1, tid2, tid3, tid4, tid5;
    int err;

    err = pthread_create(&tid1, NULL, MPU_thread, NULL);
    if (err != 0)
    {
        fprintf(stderr, "Error creating thread 1: %d\n", err);
        return 1;
    }

    err = pthread_create(&tid2, NULL, Calculations_Thread, NULL);
    if (err != 0)
    {
        fprintf(stderr, "Error creating thread 2: %d\n", err);
        return 1;
    }

    err = pthread_create(&tid3, NULL, Scenario_Thread, NULL);
    if (err != 0)
    {
        fprintf(stderr, "Error creating thread 3: %d\n", err);
        return 1;
    }

    // err = pthread_create(&tid4, NULL, Monitoring_Thread, NULL);
    // if (err != 0)
    // {
    //     fprintf(stderr, "Error creating thread 4: %d\n", err);
    //     return 1;
    // }

    err = pthread_create(&tid5, NULL, MQTT_send_Thread, NULL);
    if (err != 0)
    {
        fprintf(stderr, "Error creating thread 5: %d\n", err);
        return 1;
    }

    while (True)
    {
        // Get Dummy Vehicle's data
        // MQTT_receive_message();
    }

    // Optionally, wait for threads to finish (if needed)
    pthread_join(tid1, NULL);
    pthread_join(tid2, NULL);
    pthread_join(tid3, NULL);

    return 0;
}