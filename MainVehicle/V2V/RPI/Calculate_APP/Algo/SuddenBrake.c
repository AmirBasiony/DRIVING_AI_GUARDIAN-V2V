/*
 * SuddenBrake.c
 *
 * Author: Amir Elbasiony
 */

//-----------------------------
//          Includes
//-----------------------------

#include "../utils.h"
#include "SuddenBrake.h"
#include "../ipcMsg/msgipc.h"
#include "../Threads/Thread_2.h"
#include "PositionCalculations.h"

//================================================================
//				Generic Variables
//================================================================
// int16_t NewAngle = 0.0;
// Enumeration representing the state of potential danger
// Structure representing the position of Car2
// E_CarPosition_t Car2Position;
// Variables representing relative value
// int16_t relative_Velocity = 0.0;

// int16_t Delta_D = 0.0;
f32  LaneSafeZone = 2.5;   // Distance considered safe within a lane.
f32  LaneFocusZone = 1.5;  // Intermediate focus zone requiring attention.
f32  LaneDangerZone = 1;  // Close distance indicating potential danger.

f32 FR_FocusZone = 25.0;  // Distance considered safe between the front/rear car and my car.
f32 FR_DangerZone = 15.0; // Close distance indicating potential danger.
//================================================================
//				APIs Functions Definition
//================================================================

/**
 * @Fn         -AdjustNeighboringCarAngle
 * @brief      -Calculates the angle to represent the relative position of DummyVehicle in relation to MainVehicle.
 * @retval     -Angle representing the relative position
 * Note        -The angle indicates whether DummyVehicle is in front, behind, to the left, or to the right of MainVehicle.
 */
void AdjustNeighboringCarAngle()
{
    my_suddenbrake.Angle = MainVehicle.Angle * (180.0 / M_PI);
    // printf("[Inside] my_suddenbrake.Angle: %.2f\n", my_suddenbrake.Angle);
    // Calculate relative X and Y positions
    int32_t relative_X = DummyVehicle.x - MainVehicle.x;
    int32_t relative_Y = DummyVehicle.y - MainVehicle.y;
    // printf("relative_X : %d, relative_Y : %d\n",relative_X,relative_Y);
    // printf("relative_X: %d\n",relative_X);
    // printf("relative_Y: %d\n",relative_Y);
    /*
    EX:
        MainVehicle coordinates = (1,3)
        at Fourth quarter:  DummyVehicle  coordinates = (5,-1) ,So Relative Coordinates = (4,-4)
        at Third quarter :  DummyVehicle  coordinates = (-3,-2),So Relative Coordinates = (-4,-5)
        at Second quarter:  DummyVehicle  coordinates = (-2,4) ,So Relative Coordinates = (-3,1)
        at First quarter :  DummyVehicle  coordinates = (4,5)  ,So Relative Coordinates = (3,2)
    */
    // Determine the angle based on relative X and Y positions

    if (relative_X > 0 && !relative_Y) // +ve X-axis
    {
        //   printf("+ve X-axis\n");
        my_suddenbrake.Angle = 0;
        my_suddenbrake.Car2Position = RightSideCar;
    }
    else if (!relative_X && relative_Y > 0) // +ve Y-axis
    {
        //   printf("+ve Y-axis\n");
        my_suddenbrake.Angle = 90;
        my_suddenbrake.Car2Position = FrontCar;
    }
    else if (relative_X < 0 && !relative_Y) // -ve X-axis
    {
        //   printf("-ve X-axis");
        my_suddenbrake.Angle = 180;
        my_suddenbrake.Car2Position = LeftSideCar;
    }
    else if (!relative_X && relative_Y < 0) // -ve Y-axis
    {
        //   printf("-ve Y-axis");

        my_suddenbrake.Angle = 270;
        my_suddenbrake.Car2Position = RearCar;
    }
    else if (relative_X > 0 && relative_Y < 0) // Fourth quarter (x +ve , y -ve)
    {
        // Finding the other vehicle position

        if (my_suddenbrake.Angle <= 45)
        {
            my_suddenbrake.Car2Position = RightSideCar;
        }
        else if (my_suddenbrake.Angle >= 75)
        {
            my_suddenbrake.Car2Position = RearCar;
        }
        else
        {
            my_suddenbrake.Car2Position = IgnoredPosition;
        }
        //   printf("Fourth quarter (x +ve , y -ve)");
        my_suddenbrake.Angle += 270;
    }
    else if (relative_X < 0 && relative_Y < 0) // Third quarter (x -ve , y -ve)
    {
        // Finding the other vehicle position
        //  printf("Third quarter (x -ve , y -ve)");
        if (my_suddenbrake.Angle <= 45)
        {
            my_suddenbrake.Car2Position = LeftSideCar;
        }
        else if (my_suddenbrake.Angle >= 75)
        {
            my_suddenbrake.Car2Position = RearCar;
        }
        else
        {
            my_suddenbrake.Car2Position = IgnoredPosition;
        }

        my_suddenbrake.Angle += 180;
    }
    else if (relative_X < 0 && relative_Y > 0) // Second quarter (x -ve , y +ve)
    {
        // Finding the other vehicle position
        //  printf("Second quarter (x -ve , y +ve)");

        if (my_suddenbrake.Angle <= 45)
        {
            my_suddenbrake.Car2Position = LeftSideCar;
        }
        else if (my_suddenbrake.Angle >= 75)
        {
            my_suddenbrake.Car2Position = FrontCar;
        }
        else
        {
            my_suddenbrake.Car2Position = IgnoredPosition;
        }

        my_suddenbrake.Angle += 90;
    }
    else // First Quarter (x +ve , y +ve)
    {
        // Finding the other vehicle position
        //  printf("First Quarter (x +ve , y +ve)");
        if (my_suddenbrake.Angle <= 45)
        {
            my_suddenbrake.Car2Position = RightSideCar;
        }
        else if (my_suddenbrake.Angle >= 75)
        {
            my_suddenbrake.Car2Position = FrontCar;
        }
        else
        {
            my_suddenbrake.Car2Position = IgnoredPosition;
        }
    }
    // printf("\tINSIDE After Calc My Angle: %.2f\n",my_suddenbrake.Angle);
}

/**
 * @Fn         -Update_Safedistance
 * @brief      -Update the safe distance zones based on the velocity of the rear car
 * @param [in] -None
 * @retval     -None
 */
void Update_Safedistance()
{
    if(danger != NORMAL)
    {
        if (my_suddenbrake.Car2Position == FrontCar)
        {
            FR_FocusZone = FR_FocusZone + 0.4 * MainVehicle.Velocity;
            FR_DangerZone = FR_DangerZone + 0.4 * MainVehicle.Velocity;
        }
        else if (my_suddenbrake.Car2Position == RearCar)
        {
            FR_FocusZone = FR_FocusZone + 0.4 * DummyVehicle.Velocity;
            FR_DangerZone = FR_DangerZone + 0.4 * DummyVehicle.Velocity;
        }
    }
}

/**
 * @Fn         -evaluateDanger
 * @brief      -Check for potential danger based on the current distance
 * @param [in] -minimum_distance: The minimum safe distance to be maintained
 * @retval     -None
 */
void evaluateDanger()
{
    // printf("Inside Evaluate Danger\n");
    // printf("\t\tMainVehicle.Velocity = %d\n", MainVehicle.Velocity);
    // printf("\t\tDummyVehicle.Velocity= %d\n", DummyVehicle.Velocity);
    // printf("\t\tmy_suddenbrake.relative_Velocity= %d\n", my_suddenbrake.relative_Velocity);
    Update_Safedistance();
    danger = NORMAL;
    // If too close, initiate emergency or regular braking
    if (my_suddenbrake.FrontRearDistance <= FR_DangerZone)
    {
        danger = DANGER;
        if(my_suddenbrake.Car2Position == FrontCar)
        {
            if((my_suddenbrake.FrontRear_DeltaDistance > 0) || (my_suddenbrake.relative_Velocity <= 0))
            {
                FR_DangerZone = 15;
                FR_FocusZone = 25;
            }
        }
        else
        {
            if((my_suddenbrake.FrontRear_DeltaDistance > 0 ) || (my_suddenbrake.relative_Velocity >= 0))
            {
                FR_DangerZone = 15;
                FR_FocusZone = 25;
            }
        }
    }
    else if (my_suddenbrake.FrontRearDistance <= FR_FocusZone)
    {
        danger = FOCUS;
        if(my_suddenbrake.Car2Position == FrontCar)
        {
            if((my_suddenbrake.FrontRear_DeltaDistance > 0) || (my_suddenbrake.relative_Velocity <= 0))
            {
                FR_DangerZone = 15;
                FR_FocusZone = 25;
            }
        }
        else
        {
            if((my_suddenbrake.FrontRear_DeltaDistance > 0 ) || (my_suddenbrake.relative_Velocity >= 0))
            {
                FR_DangerZone = 15;
                FR_FocusZone = 25;
            }
        }
    }
    else
    {
        if (FR_FocusZone > 25)
        {
            FR_FocusZone = FR_FocusZone - 0.4 * abs(my_suddenbrake.relative_Velocity);
        }
        else
        {
            FR_FocusZone = 25;
        }

        if (FR_DangerZone > 15)
        {
            FR_DangerZone = FR_FocusZone - 0.4 * abs(my_suddenbrake.relative_Velocity);
        }
        else
        {
            FR_DangerZone = 15;
        }
        // If a safe distance is maintained, continue normal operation
    }
    // printf("MainVehicle.x = %.2f, MainVehicle.y = %.2f\n",MainVehicle.x,MainVehicle.y);
    // printf("DummyVehicle.x = %.2f, DummyVehicle.y = %.2f\n",DummyVehicle.x,DummyVehicle.y);
    // printf("FrontAndRear Distance : %d\n",my_suddenbrake.FrontRearDistance);
    // printf("FR_DangerZone : %.2f, FR_FocusZone : %.2f \n", FR_DangerZone, FR_FocusZone);
}

/**
 * @Fn         -SideCars_Calculations
 * @brief      -Evaluate potential danger from cars on the sides
 * @retval     -None
 */
void SideCars_Calculations()
{
    danger = NORMAL;
    if (LaneSafeZone >= my_suddenbrake.SideDistance)
    {
        if (LaneDangerZone >= my_suddenbrake.SideDistance)
        {
            danger = DANGER;
        }
        else if (LaneFocusZone >= my_suddenbrake.SideDistance)
        {
            danger = FOCUS;
        }
    }
    else
    {
        // If safe distance is maintained, continue normal operation
    }
}

/**
 * @Fn         -FrontAndRearCarCalculations
 * @brief      -Evaluate potential danger from the front or rear car
 * @param [in] -Position: Position of the car (FrontCar or RearCar)
 * @retval     -None
 */
void FrontAndRearCarCalculations()
{
    // printf("\t\tMainVehicle.Velocity = %d\n", MainVehicle.Velocity);
    // printf("\t\tDummyVehicle.Velocity= %d\n", DummyVehicle.Velocity);

    // Calculate relative velocity based on car positions
    if (FrontCar == my_suddenbrake.Car2Position)
    {
        my_suddenbrake.relative_Velocity = MainVehicle.Velocity - DummyVehicle.Velocity;
    }
    else if (RearCar == my_suddenbrake.Car2Position)
    {
        my_suddenbrake.relative_Velocity = DummyVehicle.Velocity - MainVehicle.Velocity;
    }
    // printf("\t\tmy_suddenbrake.relative_Velocity= %d\n", my_suddenbrake.relative_Velocity);

    evaluateDanger();
}

/**
 * @Fn         -SuddenBrake_Calculations
 * @brief      -Evaluate the current state of the car based on the distance to the neighboring vehicles
 * @retval     -None
 * Note        -This function calls other functions to calculate the relative position and evaluate potential danger.
 */
void SuddenBrake_Calculations()
{
    // printf("\tABefore My Angle: %d\n",my_suddenbrake.Angle);
    AdjustNeighboringCarAngle();
    // Check the position status of the other car relative to my car by the angle between them
    if (my_suddenbrake.Angle <= 105.0 && my_suddenbrake.Angle >= 75.0)
    {
        // The other car is in front of my car
        my_suddenbrake.Car2Position = FrontCar;
        // Current_D = DummyVehicle.y - MainVehicle.y;
        FrontAndRearCarCalculations();
        // printf("FRONT of me \n");
    }
    else if (my_suddenbrake.Angle <= 285.0 && my_suddenbrake.Angle >= 255.0)
    {
        // The other car is behind my car
        my_suddenbrake.Car2Position = RearCar;
        // Current_D = MainVehicle.y - DummyVehicle.y;
        FrontAndRearCarCalculations();
        // printf("BEHIND me\n");
    }
    // Whether it is on the right or left side of the road lane
    else if ((my_suddenbrake.Angle <= 45.0 && my_suddenbrake.Angle >= 0.0) || (my_suddenbrake.Angle <= 360.0 && my_suddenbrake.Angle >= 315.0))
    {
        // The other car is in right side of my car
        my_suddenbrake.Car2Position = RightSideCar;
        // Current_D = DummyVehicle.x - MainVehicle.x;
        SideCars_Calculations();
        // printf("my RIGHT\n");
    }
    else if (my_suddenbrake.Angle <= 225.0 && my_suddenbrake.Angle >= 135.0)
    {
        // The other car is in left side of my car
        my_suddenbrake.Car2Position = LeftSideCar;
        // my_Collision_distance = MainVehicle.x - DummyVehicle.x;
        SideCars_Calculations();
        // printf("my LEFT\n");
    }
    // printf("\tAfter My Angle: %d\n",my_suddenbrake.Angle);
    Vehicle_ahead_Action();
}

/**
 * @Fn         -updateSpeedToAvoidRearCollision
 * @brief      -Calculates the updated speed required for a rear car to avoid colliding
 *          0    with a front car in a potentially dangerous situation.
 * @param [in] -None
 * @retval     -Updated speed to prevent rear collision.
 * Note        - (vf) is the velocity of the front car,
 *             - (a) is the acceleration of the rear car, and
 *             - (d) is the current distance.
 */
f32 updateSpeedToAvoidRearCollision()
{
    // vf2 = vi2 + 2 a d
    //  vi = sqr(vf2 - 2 a d)
    //  Initialize the new speed for the rear car, assuming it slows down to a complete stop.
    f32 newSpeed = 0;

    // Variable to hold the squared final speed after applying the braking mechanism.
    f32 finalSpeedSquared;

    // Check if the front car is at the same position as Car2.
    if (FrontCar == my_suddenbrake.Car2Position)
    {
        // Calculate the squared final speed using the front car's velocity, acceleration, and distance to the rear car.
        finalSpeedSquared = pow(DummyVehicle.Velocity, 2) - (2 * MainVehicle.Accelration * my_suddenbrake.FrontRearDistance);
    }
    // Check if the rear car is at the same position as Car2.
    else if (RearCar == my_suddenbrake.Car2Position)
    {
        // Calculate the squared final speed using the rear car's velocity, acceleration, and distance to the front car.
        finalSpeedSquared = pow(MainVehicle.Velocity, 2) - (2 * DummyVehicle.Accelration * my_suddenbrake.FrontRearDistance);
    }

    // If the final speed squared is non-negative (indicating a valid speed),
    // calculate the square root to get the actual speed.
    if (finalSpeedSquared >= 0)
    {
        newSpeed = sqrt(finalSpeedSquared);
    }
    // If the final speed squared is negative (it indicates that the rear car cannot achieve a speed
    // to maintain a safe distance given its acceleration,even if it were to decelerate to its maximum extent),
    // set the new speed to 0 to prevent imaginary speeds.
    else
    {
        newSpeed = 0;
    }

    // Return the updated speed to avoid a rear collision.
    return newSpeed;
}

/**
 * @Fn         -Vehicle_ahead_Action
 * @brief      -Take action based on the evaluated state of potential danger
 * @param [in] -isdangerous: Evaluated state of potential danger
 * @retval     -None
 * Note        -This function contains switch cases to determine the appropriate action based on the state of danger
 *               and the other car postion.
 */
void Vehicle_ahead_Action()
{
    // printf("Danger: %d\n",danger);
    switch (danger)
    {
    case DANGER:
        // printf("[ DANGER ] \n");
        MainVehicle.action = DANGER;
        switch (my_suddenbrake.Car2Position)
        {
        case FrontCar:
            if (my_suddenbrake.relative_Velocity > 0)
            {
                if (my_suddenbrake.FrontRear_DeltaDistance <= 0)
                {
                    MainVehicle.msg = FASTER;
                    // printf("Large change in distance!!!!!!!!!!!!!\n");
                    // my car needs to decrease the velocity aggressively
                    MainVehicle.Velocity = updateSpeedToAvoidRearCollision(); // for Rear Car
                    // The front needs to increase the velocity aggressively
                    // MainVehicle.msg = DummyVehicle.Velo0city + 0.4 * (my_suddenbrake.relative_Velocity);
                }
                else
                {
                    MainVehicle.msg = FASTER;
                    // the rear car needs to increase the velocity slightly
                    // MainVehicle.msg = DummyVehicle.Velocity + 0.2 * (my_suddenbrake.relative_Velocity); // for Rear Car
                    // my car needs to decrease the velocity slightly
                    MainVehicle.Velocity -= 0.2 * (my_suddenbrake.relative_Velocity);
                }
            }
            else // need to be tuned
            {
                MainVehicle.msg = FASTER;
                // MainVehicle.msg = DummyVehicle.Velocity;
                // my car needs to decrease the velocity slightly
                MainVehicle.Velocity -= 0.2 * (-my_suddenbrake.relative_Velocity);
            }
            break;
        case RearCar:
            if (my_suddenbrake.relative_Velocity > 0)
            {
                if (my_suddenbrake.FrontRear_DeltaDistance <= 0)
                {
                    MainVehicle.msg = SLOWDOWN;
                    // printf("Large change in distance!!!!!!!!!!!!!\n");
                    // The Rear needs to decrease the velocity aggressively
                    // MainVehicle.msg = updateSpeedToAvoidRearCollision(); // for Rear Car
                    // my car needs to increase the velocity aggressively
                    if(MainVehicle.Velocity < (DummyVehicle.Velocity))
                        MainVehicle.Velocity = DummyVehicle.Velocity + (DummyVehicle.Velocity * .25);
                    // else    
                    //     MainVehicle.Velocity += 0.6 * (my_suddenbrake.relative_Velocity);
                }
                else
                {
                    MainVehicle.msg = SLOWDOWN;
                    // the rear car needs to decrease the velocity slightly
                    // MainVehicle.msg = DummyVehicle.Velocity - 0.2 * (my_suddenbrake.relative_Velocity); // for Rear Car
                    // my car needs to increase the velocity slightly
                    MainVehicle.Velocity += 0.4 * (my_suddenbrake.relative_Velocity);
                }
            }
            else
            {
                MainVehicle.msg = SLOWDOWN;
                // MainVehicle.msg = DummyVehicle.Velocity;
                // my car needs to increase the velocity aggressively
                MainVehicle.Velocity += 0.4 * (-my_suddenbrake.relative_Velocity);
            }
            break;
        case LeftSideCar:
        case RightSideCar:
            if ((MainVehicle.Angle > 15 && MainVehicle.Angle <= 45) || (MainVehicle.Angle >= 135 && MainVehicle.Angle < 165))
            {
                MainVehicle.msg = SLOWDOWN;
                // printf("[SideCar]: Emergency Braking Initiated in my car!\n");
                MainVehicle.Velocity *= 2;              
                // MainVehicle.msg = DummyVehicle.Velocity - 0.4 * (DummyVehicle.Velocity); // need to modify the message
            }
            else
            {
                MainVehicle.msg = FASTER;
                MainVehicle.Velocity = 0;                                  // Emergency Brake
                // MainVehicle.msg = DummyVehicle.Velocity + 0.4 * (DummyVehicle.Velocity); // need to modify the message
            }
            break;
        case IgnoredPosition:
            break;
        }
        break;
    case FOCUS:
        MainVehicle.action = FOCUS;
        // printf(" [ FOCUS ] \n");
        // Execute less urgent actions based on car position
        switch (my_suddenbrake.Car2Position)
        {
        case FrontCar:
            // Less aggressive action in focus zone
            if (my_suddenbrake.relative_Velocity > 0)
            {
                if (my_suddenbrake.FrontRear_DeltaDistance <= 0)
                {
                    MainVehicle.msg = FASTER;
                    // printf("Large change in distance!!!!!!!!!!!!!\n");
                    // my car needs to decrease the velocity aggressively
                    MainVehicle.Velocity = updateSpeedToAvoidRearCollision(); // for Rear Car
                    // the front car needs to increase the velocity aggressively
                    // MainVehicle.msg = DummyVehicle.Velocity + 0.4 * (my_suddenbrake.relative_Velocity);
                }
                else
                {
                    MainVehicle.msg = FASTER;
                    // Adjust front car speed (needs to increase the velocity slightly)
                    // MainVehicle.msg = DummyVehicle.Velocity + 0.1 * (my_suddenbrake.relative_Velocity);
                    // my car needs to decrease the velocity slightly
                    MainVehicle.Velocity -= 0.1 * (my_suddenbrake.relative_Velocity);
                }
            }
            else
            {
                MainVehicle.msg = FASTER;
                // MainVehicle.msg = DummyVehicle.Velocity;
                // my car needs to decrease the velocity slightly
                MainVehicle.Velocity -= 0.1 * (-my_suddenbrake.relative_Velocity);
            }
            break;
        case RearCar:
            // printf("[RearCar]: The speed began to accelerate regularly!\n");
            // Less aggressive action in focus zone
            if (my_suddenbrake.relative_Velocity >= 0)
            {
                if (my_suddenbrake.FrontRear_DeltaDistance <= 0)
                {
                    MainVehicle.msg = SLOWDOWN;
                    // printf("Large change in distance!!!!!!!!!!!!!\n");
                    // The Rear needs to decrease the velocity aggressively
                    // MainVehicle.msg = updateSpeedToAvoidRearCollision(); // for Rear Car
                    // my car needs to increase the velocity aggressively
                    
                    if(MainVehicle.Velocity < (DummyVehicle.Velocity))
                        MainVehicle.Velocity = DummyVehicle.Velocity;
                    // else    
                    //     MainVehicle.Velocity += 0.1 * (my_suddenbrake.relative_Velocity);
                }
                else
                {
                    MainVehicle.msg = SLOWDOWN;
                    // the rear car needs to decrease the velocity slightly
                    // MainVehicle.msg = DummyVehicle.Velocity - 0.1 * (my_suddenbrake.relative_Velocity);
                    // my car needs to increase the velocity slightly
                    MainVehicle.Velocity += 0.1 * (my_suddenbrake.relative_Velocity);
                }
            }
            else
            {
                MainVehicle.msg = SLOWDOWN;
                // MainVehicle.msg = DummyVehicle.Velocity;
                // my car needs to increase the velocity slightly
                MainVehicle.Velocity += 0.1 * (-my_suddenbrake.relative_Velocity);
            }
            break;
        case LeftSideCar:
        case RightSideCar:
            MainVehicle.msg = SLOWDOWN;
            // MainVehicle.action = 'A';
            // printf("[SideCar]: Alarm\n");
            break;
        case IgnoredPosition:
            break;
        }
        break;
    case NORMAL:
        // printf(" [NORMAL] \n");
        MainVehicle.msg = SAFE;
        MainVehicle.action = NORMAL;
        switch (my_suddenbrake.Car2Position)
        {
        case FrontCar:
            //  printf("It's Safe Go On!!!\n");
            if (my_suddenbrake.FrontRearDistance > FR_DangerZone  && my_suddenbrake.FrontRearDistance <= FR_FocusZone)
                MainVehicle.Velocity = SLOW_SPEED;
            else if(my_suddenbrake.FrontRearDistance > FR_FocusZone)
                MainVehicle.Velocity = NORMAL_SPEED;
            else if (my_suddenbrake.FrontRearDistance <= FR_DangerZone)
                MainVehicle.Velocity = 0;
        break;
        case RearCar:
            if (my_suddenbrake.FrontRearDistance > FR_DangerZone  && my_suddenbrake.FrontRearDistance <= FR_FocusZone)
                MainVehicle.Velocity = DummyVehicle.Velocity;
            else if(my_suddenbrake.FrontRearDistance > FR_FocusZone)
            {
                if(my_suddenbrake.FrontRear_DeltaDistance > 0 || MainVehicle.Velocity > DummyVehicle.Velocity)
                {
                    if(DummyVehicle.Velocity == 0)
                        MainVehicle.Velocity = NORMAL_SPEED;
                    else
                        MainVehicle.Velocity = DummyVehicle.Velocity;
                }
            }
            else if (my_suddenbrake.FrontRearDistance <= FR_DangerZone)
                 MainVehicle.Velocity = 1.25 * DummyVehicle.Velocity;
            // MainVehicle.action = MainVehicle.Velocity;
            // MainVehicle.msg = DummyVehicle.Velocity;
            // printf("It's Safe Go On!!!\n");
            break;
        case RightSideCar:
        case LeftSideCar:
            // printf("It's Safe Go On!!!\n");
            // MainVehicle.action = 'N';
            // MainVehicle.msg = DummyVehicle.Velocity;
            break;
        case IgnoredPosition:
            // printf("Ignored Position!!!\n");
            // MainVehicle.action = 'N';
            // MainVehicle.msg = DummyVehicle.Velocity;
            break;
        }
    default:
        break;
    }
#ifdef TESTING
    // DummyVehicle.Velocity = MainVehicle.msg;

#endif
}
