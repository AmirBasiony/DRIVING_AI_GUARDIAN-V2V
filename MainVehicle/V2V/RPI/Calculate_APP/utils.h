#ifndef __UTILITIES__
#define __UTILITIES__

#include "math.h"
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/i2c-dev.h>
#include "Infrastructure/STDTYPE.h"

#define DANGERZONE 20
#define FOCUSZONE 40
#define DANGERTIME 1.2
#define FOCUSTIME 2

#define SLOW_SPEED 20
#define NORMAL_SPEED 40

// #define TESTING
// #define SCENARIOS
// #define REARSCENARIOS
typedef enum ALGO
{
    INTERSECTION = 1,
    SUDDENBRAKE,
    CALCULATING
} E_CollisionCause_t;

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
typedef struct  Vehicle 
{
    uint8 ID;
    f32 x;
    f32 y;
    uint32 Accelration;
    double Angle;
    uint16 Velocity;
    E_Direction_t Direction;
    E_MSG_t msg;
    E_State_t action;    
} S_Connected_Car_t;
typedef enum CarPosition
{
    FrontCar,       // Car is in front of MainVehicle.
    RearCar,        // Car is behind MainVehicle.
    LeftSideCar,    // Car is to the left side of MainVehicle.
    RightSideCar,   // Car is to the right side of MainVehicle.
    IgnoredPosition // Position of the car is ignored or not considered.
} E_CarPosition_t;

typedef struct
{
    int16_t SideDistance;
    int16_t FrontRear_DeltaDistance;
    int16_t FrontRearDistance;
    int16_t safe_distance;
    int16_t relative_Velocity;
    double Angle;
    E_CarPosition_t Car2Position;
} S_SUDDENBRAKE_Data_t;

typedef struct
{
    uint32 My_Dist;
    uint32 other_Dist;
    f32 Mytime;
    f32 Othertime;
    uint32 Current_Dist;
    uint32 point_X;
    uint32 point_Y;
} S_INTERSECION_Data_t;

extern pthread_mutex_t DoneGettintInitials_viaUART_mutex,GPS_X_mutex, GPS_Y_mutex, yaw_mutex, displacement_mutex, acceleration_mutex, MainVehicle_mutex, DummyVehicle_mutex, intersection_mutex, suddenbrake_mutex, prev_distance_mutex, delta_d_mutex, DoneCalibration_mutex;
extern S_Connected_Car_t DummyVehicle, MainVehicle;
extern S_INTERSECION_Data_t my_intersection;
extern S_SUDDENBRAKE_Data_t my_suddenbrake;
extern uint32 msgid_ACT_receive;
extern uint32 usb0_filestream;
extern Bool_t DoneCalibration,KnowDummy;
extern f32 total_displacement;
extern uint32 msgid_ACT_send;
extern uint16 acceleration;
extern f32 displacement;
extern E_State_t danger;
#endif