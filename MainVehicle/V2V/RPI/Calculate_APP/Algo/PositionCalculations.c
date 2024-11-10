#include "SuddenBrake.h"
#include "intersection.h"
#include "../ipcMsg/msgipc.h"
#include "../Threads/Thread_3.h"
#include "PositionCalculations.h"
#include "../MQTT/MQTT.h"

//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Global Variables ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

uint32 GPS_X = 0, GPS_Y = 0;
uint8 increment = 100;
f32 displacement = 0;
f32 total_displacement = 0;

S_Connected_Car_t DummyVehicle = {.ID = 0,
                                  .x = 0,
                                  .y = 0,
                                  .Velocity = 0,
                                  .Accelration = 0,
                                  .Direction = 0,
                                  .Angle = 0,
                                  .msg = 0,
                                  .action = 0};
S_Connected_Car_t MainVehicle = {.ID = 0,
                                 .x = 0,
                                 .y = 0,
                                 .Velocity = 0,
                                 .Accelration = 0,
                                 .Direction = 0,
                                 .Angle = 0,
                                 .msg = 0,
                                 .action = 0};

S_INTERSECION_Data_t my_intersection;
S_SUDDENBRAKE_Data_t my_suddenbrake;
int16_t Prev_Distance = 0.0;
struct timespec start_time, end_time;

//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Calculations APIs //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void start_timer()
{
    clock_gettime(CLOCK_REALTIME, &start_time);
}
Bool_t t_flag = 0;
void calculate_displacement()
{
    if (t_flag == 0)
    {
        start_timer();
        t_flag = 1;
    }
    clock_gettime(CLOCK_REALTIME, &end_time);
    double elapsed_time = (end_time.tv_sec - start_time.tv_sec) +
                          (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
    start_timer(); // Reset start time
    displacement = MainVehicle.Velocity * elapsed_time / 2.0;
    total_displacement += displacement;
    // printf("total_displacement: %.8f \n", total_displacement);
    // return displacement_cm;
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
    if ((yaw >= 345 || yaw <= 15)) // 0
        return NORTH;
    else if ((yaw >= 165 && yaw <= 195)) // 180
        return SOUTH;
    else if ((yaw >= 75 && yaw <= 105)) // 90
        return WEST;
    else if ((yaw >= 255 && yaw <= 285)) // 270
        return EAST;
    return -1;
}

/*
 * @Fn         calculateDistance
 * @brief      Calculates the distance between two vehicles based on their coordinates.
 * @param [in] myX, myY: Coordinates of the current vehicle.
 * @param [in] otherX, otherY: Coordinates of another vehicle.
 * @retval     Distance between the two vehicles.
 */
f32 calculateDistance(f32 myX, f32 myY, f32 otherX, f32 otherY)
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
double calculateAngle(f32 Current_X, f32 Current_Y, f32 otherX, f32 otherY)
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

/*
 * @Fn         Time_Of_Collision
 * @brief      Calculates the time of collision based on Newton's equation D = V0 * t + 0.5 * a * t^2.
 * @param [in] D_of_collision: Distance to collision point.
 * @param [in] acc: Acceleration (in meters per second squared).
 * @param [in] v0: Initial velocity (in meters per second).
 * @retval     Time of collision (in seconds).
 * @note       Uses the quadratic formula to find the roots of the equation.
 */
f32 Time_Of_Collision(f32 acc, f32 v0, f32 D_of_collision)
{
    f32 a_coefficient, b_coefficient, c_coefficient, discriminant, root1, root2;

    // Check if acceleration is zero
    if (acc == 0.0f)
    {
        if (v0 != 0.0f)
        {
            return D_of_collision / v0;
        }
        else
        {
            // printf("\nBoth acceleration and initial velocity are zero. Collision not possible.\n");
            return -2; // Error Non Moving Vehicle
        }
    }

    // Calculate coefficients of the quadratic equation
    a_coefficient = 0.5f * acc;
    b_coefficient = v0;
    c_coefficient = -D_of_collision;

    // Calculate discriminant
    discriminant = b_coefficient * b_coefficient - 4 * a_coefficient * c_coefficient;

    // Check discriminant for real roots
    if (discriminant > 0.0f)
    {
        // Two real roots
        root1 = (-b_coefficient + sqrtf(discriminant)) / (2 * a_coefficient);
        root2 = (-b_coefficient - sqrtf(discriminant)) / (2 * a_coefficient);
        return (root1 > 0) ? root1 : root2; // Return the positive root
    }
    else if (discriminant == 0.0f)
    {
        // One real root
        root1 = -b_coefficient / (2 * a_coefficient);
        return root1;
    }
    else
    {
        // Imaginary roots (error condition)
        printf("\nImaginary roots: collision not possible.\n");
        return -1; // Error
    }
}

/*
 * @Fn         CoordinatesCalculations
 * @brief      Performs calculations for coordinates, angle, displacement and velocity.
 * @note       Utilizes functions for calculating direction, angle, distance, and velocity.
 */
void CoordinatesCalculations()
{
    // ************** UPDATING STRUCT DATA **************//
    calculate_displacement();
    double yaw_radians = (yaw * (M_PI / 180.0));
    MainVehicle.y += displacement * cos(yaw_radians);
    MainVehicle.x += displacement * sin(yaw_radians);

    MainVehicle.Direction = CurrentDirection();
    MainVehicle.Angle = calculateAngle(MainVehicle.x, MainVehicle.y, DummyVehicle.x, DummyVehicle.y); //* (180 / M_PI) // Angle between my car and the other car
    // ************** CALCULATIONS DATA **************//

    // distance must be abs because the angle could be negative
    // in intersection i dont care about the direction anyway i only need the distance
    // in suddenbrake the actual position of the other vehicle is calculated - Unity circle -

    my_intersection.Current_Dist = calculateDistance(MainVehicle.x, MainVehicle.y, DummyVehicle.x, DummyVehicle.y);
    // printf("Sin(%.2f) : %.2f ,Cos(%.2f) : %.2f \n", MainVehicle.Angle, sin(MainVehicle.Angle), MainVehicle.Angle, cos(MainVehicle.Angle));
    my_intersection.My_Dist = fabs(my_intersection.Current_Dist * sin(MainVehicle.Angle));
    my_intersection.other_Dist = fabs(my_intersection.Current_Dist * cos(MainVehicle.Angle));
    // printf("My_Dist(%d) : other_D(%d)\n", my_intersection.My_Dist, my_intersection.other_Dist );

    my_suddenbrake.SideDistance = fabs(my_intersection.Current_Dist * cos(MainVehicle.Angle));
    my_suddenbrake.FrontRearDistance = fabs(my_intersection.Current_Dist * sin(MainVehicle.Angle));
    my_suddenbrake.FrontRear_DeltaDistance = my_suddenbrake.FrontRearDistance - Prev_Distance; // Negative value indicating decreasing in the distance between the two vehicles
    Prev_Distance = my_suddenbrake.FrontRearDistance;
}