#ifndef __POSITION_CALCULATIONS__
#define __POSITION_CALCULATIONS__

#include "../utils.h"

extern uint32 GPS_X, GPS_Y;

void start_timer();
void calculate_displacement();
double calculateAngle(f32 Current_X, f32 Current_Y, f32 otherX, f32 otherY);
f32 Time_Of_Collision(f32 acc, f32 v0, f32 D_of_collision);
E_Direction_t CurrentDirection();
void CoordinatesCalculations();

#endif