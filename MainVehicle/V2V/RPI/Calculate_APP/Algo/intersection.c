#include "intersection.h"
#include "PositionCalculations.h"
#include "../Threads/Thread_2.h"
Bool_t intersection_checked = False;
// /**
//  * @brief      Determines the car's destination direction based on its movement.
//  * @return     E_Direction_t: The direction in which the car is moving.
//  * @details    This function calculates the car's movement direction by comparing
//  *             the current position (`MainVehicle.x`, `MainVehicle.y`) with the previous position (`prevX`, `prevY`).
//  *             It then updates the intersection points (`my_intersection.point_X`, `my_intersection.point_Y`)
//  *             based on the car's current position and a predefined distance (`my_intersection.My_Dist`).
//  */
void Check_intersectionPoint()
{
    if (MainVehicle.Direction == WEST)
        my_intersection.point_X = MainVehicle.x - my_intersection.My_Dist;
    else if (MainVehicle.Direction == EAST)
        my_intersection.point_X = MainVehicle.x + my_intersection.My_Dist;
    else if (MainVehicle.Direction == SOUTH)
        my_intersection.point_Y = MainVehicle.y - my_intersection.My_Dist;
    else if (MainVehicle.Direction == NORTH)
        my_intersection.point_Y = MainVehicle.y + my_intersection.My_Dist;
    intersection_checked = True;
}

/**
 * @brief      Checks the car's status at an intersection.
 * @details    This function retrieves the car's destination direction and current coordinates.
 *             It compares these coordinates with the intersection points to determine if the car
 *             has passed the collision point. If the car has passed the point, it calls `Intersection_Calculations()`.
 */
void intersection_Status()
{
    if (intersection_checked)
        Check_intersectionPoint();

    if ((MainVehicle.Direction == NORTH && MainVehicle.x <= my_intersection.point_X) ||
        (MainVehicle.Direction == SOUTH && MainVehicle.x >= my_intersection.point_X) ||
        (MainVehicle.Direction == EAST && MainVehicle.y <= my_intersection.point_Y) ||
        (MainVehicle.Direction == WEST && MainVehicle.y >= my_intersection.point_Y))
    {
        Intersection_Calculations();
    }
    else
        printf("No intersection point possible .. \n");
}

/**
 * @brief      Calculates collision times for the current vehicle and another vehicle at an intersection.
 * @details    The function determines the danger level based on the time difference between collision times
 *             and triggers a possible collision action accordingly.
 */
void Intersection_Calculations()
{
    danger = NORMAL;
    my_intersection.Mytime = Time_Of_Collision(MainVehicle.Accelration, MainVehicle.Velocity, my_intersection.My_Dist);
    my_intersection.Othertime = Time_Of_Collision(DummyVehicle.Accelration, DummyVehicle.Velocity, my_intersection.other_Dist);
    // printf("my time : %.2f, dummy time : %.2f \n", my_intersection.Mytime, my_intersection.Othertime);

    if ((my_intersection.Othertime == -2) && (my_intersection.Mytime <= DANGERTIME))
        danger = DANGER;
    else if ((my_intersection.Othertime == -2) && (my_intersection.Mytime <= FOCUSTIME))
        danger = FOCUS;

    else if ((my_intersection.Mytime == -2) && (my_intersection.Othertime <= DANGERTIME))
        danger = DANGER;
    else if ((my_intersection.Mytime == -2) && (my_intersection.Othertime <= FOCUSTIME))
        danger = FOCUS;

    else if (abs((sint32)(my_intersection.Mytime - my_intersection.Othertime)) < DANGERTIME)
        danger = DANGER;
    else if (abs((sint32)(my_intersection.Mytime - my_intersection.Othertime)) < FOCUSTIME)
        danger = FOCUS;
    Intersection_Action();
}

/**
 * @brief      Takes actions based on the danger level detected at an intersection.
 * @details    The function performs specific actions depending on the detected danger level,
 *             including sending messages to the other car or triggering an alarm.
 */
void Intersection_Action()
{
    switch (danger)
    {
    case DANGER: // danger zone
        MainVehicle.action = DANGER;
        if (my_intersection.Othertime == -2) // Error Non Moving Vehicle
        {
            if (MainVehicle.Velocity >= 15)
                MainVehicle.Velocity -= 15;
            else
                MainVehicle.Velocity = 0;
            MainVehicle.msg = FASTER;
            // printf(" V in action othertime is -2 : %d \n",MainVehicle.Velocity);
        }
        else if ((my_intersection.Mytime > my_intersection.Othertime) && (MainVehicle.Velocity >= 15))
        {
            MainVehicle.Velocity -= 15;
            MainVehicle.msg = FASTER;
        }
        else if ((my_intersection.Mytime <= my_intersection.Othertime) && (MainVehicle.Velocity < 100))
        {
            MainVehicle.Velocity += 15;
            // send data to the other car to slow down, and I will increase my speed
            MainVehicle.msg = SLOWDOWN;
        }
        break;
    case FOCUS: // focus zone
        MainVehicle.action = FOCUS;
        break;
    case NORMAL:
        // send that it's safe
        MainVehicle.msg = SAFE;
        MainVehicle.action = NORMAL;
        MainVehicle.Velocity = NORMAL_SPEED;
        break;
    default:
        break;
    }
}
