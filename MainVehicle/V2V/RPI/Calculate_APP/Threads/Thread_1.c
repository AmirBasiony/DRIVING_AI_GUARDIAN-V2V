#include "../connections/UART_Struct.h"
#include "../ipcMsg/msgipc.h"
#include "Thread_3.h"
#include "Thread_1.h"

/**
 * @brief Continuously calculates data based on sensor input and coordinates calculations.
 *
 * This function runs in an infinite loop, acquiring and processing data from sensors
 * and performing necessary calculations. It uses multiple mutexes to ensure thread safety
 * when accessing shared resources.
 */
void Calculate_Data()
{
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

#ifdef SCENARIOS
                        CoordinatesCalculations();
#else
                        // 1 - get data from ESP
                        // _UART_Receive_Struct(&DummyVehicle);
                        CoordinatesCalculations();
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
}
