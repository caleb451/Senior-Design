#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

#ifdef ESP32
  //Digital Motor Pins
  #ifdef CONTROLLER
    // Joystick analog pins only
    #define y_axis 32
    #define x_axis 33
  #endif
  #ifdef RECIEVER
    // Motor pins only
    #define m1_fwd  13
    #define m1_bkwd 12
    #define m2_fwd  14
    #define m2_bkwd 27
    #define m3_fwd  26
    #define m3_bkwd 25
    #define m4_fwd  35
    #define m4_bkwd 34
  #endif
#else
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
  #define y_axis A1
  #define x_axis A0
#endif

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
