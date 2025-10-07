#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

//Digital Motor Pins
#define m1_fwd  2
#define m1_bkwd 3
#define m2_fwd  4
#define m2_bkwd 5
#define m3_fwd  6
#define m3_bkwd 7
#define m4_fwd  8
#define m4_bkwd 9

//Analog Pins
#define x_axis A0
#define y_axis A1

//Motor Groups
extern int motors[8];
extern int motors_fwd[4];
extern int motors_bkwd[4];
extern int motors_left[4];
extern int motors_right[4];

//Motor Functions
void motorsOff();
void motorsFwd();
void motorsBkwd();
void motorsLeft();
void motorsRight();
void test();
char joystickControl(int xvalue, int yvalue);

#endif
