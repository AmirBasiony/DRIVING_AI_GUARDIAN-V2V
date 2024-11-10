

#include "Thread_2.h"
#include "Thread_3.h"
#include "../ipcMsg/msgipc.h"
#include "../connections/UART_Struct.h"
#include "../MQTT/MQTT.h"

/**
 * @brief Handles different scenarios based on vehicle directions and calibration status.
 *
 * Depending on the direction and whether calibration is complete, it performs different
 * actions, such as handling intersections or sudden brakes. It also sends messages and updates
 * displacement based on vehicle velocity.
 *
 */
void Scenario_Handeling()
{
    // printf("\t\t\tMainVehicle.Velocity Before editing = %d\n", MainVehicle.Velocity);
    while (True)
    {
        pthread_mutex_lock(&yaw_mutex);
        pthread_mutex_lock(&displacement_mutex);
        pthread_mutex_lock(&acceleration_mutex);
        pthread_mutex_lock(&MainVehicle_mutex);
        pthread_mutex_lock(&DummyVehicle_mutex);
        pthread_mutex_lock(&intersection_mutex);
        pthread_mutex_lock(&suddenbrake_mutex);
        pthread_mutex_lock(&prev_distance_mutex);
        pthread_mutex_lock(&delta_d_mutex);
        pthread_mutex_lock(&DoneCalibration_mutex);

        if (DoneCalibration && KnowDummy)
        {
            if (MainVehicle.Direction != DummyVehicle.Direction)
            {
                intersection_Status();
            }
            else
            {
#ifdef REARSCENARIOS
                DummyVehicle.y += .05;
#endif
                // printf("\tBefore Calling the scenario\n\tMainVehicle.Velocity = %d\n", MainVehicle.Velocity);
                SuddenBrake_Calculations();
            }
#ifdef SCENARIOS
            // printf("action in thread_2 = %d\n", MainVehicle.action);
            // send_message_to_PY(msgid_ACT_send, MainVehicle.action, &MainVehicle.Velocity); // this would have the value of the needed speed

#else
            // _UART_Send_Struct(&MainVehicle);
            send_message_to_PY(msgid_ACT_send, MainVehicle.action, &MainVehicle.Velocity); // this would have the value of the needed speed
#endif
        }

        // Unlock mutexes
        pthread_mutex_unlock(&yaw_mutex);
        pthread_mutex_unlock(&displacement_mutex);
        pthread_mutex_unlock(&acceleration_mutex);
        pthread_mutex_unlock(&MainVehicle_mutex);
        pthread_mutex_unlock(&DummyVehicle_mutex);
        pthread_mutex_unlock(&intersection_mutex);
        pthread_mutex_unlock(&suddenbrake_mutex);
        pthread_mutex_unlock(&prev_distance_mutex);
        pthread_mutex_unlock(&delta_d_mutex);
        pthread_mutex_unlock(&DoneCalibration_mutex);
        usleep(100000); // Sleep for 100 ms to prevent busy-waiting
    }
    // Terminate the child process
    exit(EXIT_SUCCESS);
    // #endif
}