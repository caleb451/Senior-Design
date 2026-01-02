/*
Location:   robot_program/src/main.cpp
Author:     Caleb Gomez
Date:       09/18/2025
Description:
	This uses an arduino uno to test the functionality of the robot by moving it forwards 
	and back
	
*/

#include <Arduino.h>

//Motor pins
#define m1_fwd  2
#define m1_bkwd 3
#define m2_fwd  4
#define m2_bkwd 5
#define m3_fwd  6
#define m3_bkwd 7
#define m4_fwd  8
#define m4_bkwd 9

int motors[8] = {m1_fwd, m2_fwd, m3_fwd, m4_fwd, m1_bkwd, m2_bkwd, m3_bkwd, m4_bkwd};
int motors_fwd[4] = {m1_fwd, m2_fwd, m3_fwd, m4_fwd};
int motors_bkwd[4] = {m1_bkwd, m2_bkwd, m3_bkwd, m4_bkwd};

void motorsOff() {
    for (int i = 0; i < 8; i++)
        digitalWrite(motors[i], LOW);
}

void motorsFwd() {
    motorsOff();
    for (int i = 0; i < 4; i++)
        digitalWrite(motors_fwd[i], HIGH);
}

void motorsBkwd() {
    motorsOff();
    for (int i = 0; i < 4; i++)
        digitalWrite(motors_bkwd[i], HIGH);
}

void setup(){
	for(int i = 0; i < 8; i++)
        pinMode(motors[i], OUTPUT);
    motorsOff();
}

void loop(){
	motorsFwd();
	delay(1000);
	motorsOff();
	delay(200);
	motorsBkwd();
	delay(1000);
	motorsOff();
	delay(200);
}
