/*
Author: Caleb Gomez
Date: 09/22/2025
Description:
10/07/2025
    Updated to wirelessly control the robot using joystick control while communicating with 2 different esp32s.
    This program will send the 'state' of the joystick to the esp32 on the robot.
*/

#include <Arduino.h>
#include <functions.h>

#ifdef ESP32
    #include <esp_now.h>
    #include <WiFi.h>
#endif

int xvalue = 0;
int yvalue = 0;
char state = ' ';

void setup(){
    
}

void loop(){
    xvalue = analogRead(x_axis);
    yvalue = analogRead(y_axis);

    state = joystickControl(xvalue, yvalue);
}