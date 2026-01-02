#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <pins.h>

//Motor Groups
extern int motors[8];
extern int motors_fwd[4];
extern int motors_bkwd[4];
extern int motors_left[4];
extern int motors_right[4];
extern int motors_Rright[4];
extern int motors_Rleft[4];

//Motor Functions
void motorsOff();
void motorsFwd();
void motorsBkwd();
void motorsLeft();
void motorsRight();
void motorsRotateR();
void motorsRotateL();
void test();
char joystickControl(int xvalue, int yvalue);

#endif
