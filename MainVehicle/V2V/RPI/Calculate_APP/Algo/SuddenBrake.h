/*
* SuddenBrake.h
*
* Author: Amir Elbasiony
* Description: Header file for the SuddenBrake functionality, providing declarations for functions 
*              and types used in the SuddenBrake calculations.
*/

#ifndef SUDDENBRAKE_H
#define SUDDENBRAKE_H

//------------
//Includes
//------------
#include "../utils.h"


/*
*===========
*   Macros
*===========
*/


// Constants (need to adjust based on vehicle and conditions during testing and validation)
// #define DECELERATION_RATE 8.0    // Deceleration rate in m/s^2
// #define REACTION_TIME 1.5        // Reaction time in seconds
// #define SAFE_DISTANCE_MULTIPLIER 2.0  // Safety multiplier

// Macros for different zones within a lane based on safety considerations.

extern f32  LaneSafeZone;   // Distance considered safe within a lane.
extern f32  LaneFocusZone;  // Intermediate focus zone requiring attention.
extern f32  LaneDangerZone;  // Close distance indicating potential danger.


//------------------------------------
//User type definitions (Enumeration)
//------------------------------------

// Enumeration for possible positions of a car relative to MyCar.

/*
* ==================
* Global variables.
* ==================
*/

// Distance considered safe between the front/rear car and my car.
extern f32 FR_FocusZone;    
// Close distance indicating potential danger.
extern f32 FR_DangerZone;    
// Global variables for danger assessment and Position tracking
// extern E_State_t danger;
// extern E_CarPosition_t Car2Position;

// Global variables for relative velocity and distance measurements
// extern int16_t relative_Velocity;
// extern int16_t Prev_Distance;
// extern f32 safe_distance; // safe distance for sudden brake scenario calculations
// extern int16_t Delta_D;
// extern int16_t NewAngle ;
/*
* ===============================================
* APIs for SuddenBrake Calculations and Actions.
* ===============================================
*/

// Main function to calculate the SuddenBrake logic.
void SuddenBrake_Calculations();
// Evaluates and performs actions based on the danger level ahead.
void Vehicle_ahead_Action();
// Calculates danger based on the position of front and rear cars.
void FrontAndRearCarCalculations();
// Evaluates potential danger from cars on the sides.
void SideCars_Calculations();
// Evaluates danger based on a specified minimum distance.
void evaluateDanger();
// Calculates safe following distance based on current velocity.
//void safeDistance(f32 velocity); 
// Calculates the relative angle of a neighboring car to MyCar.
void AdjustNeighboringCarAngle();
//Calculates updated speed to prevent rear collision.
f32 updateSpeedToAvoidRearCollision();
void Update_Safedistance();
#endif /* SUDDENBRAKE_H */