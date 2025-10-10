/*
Author: Caleb Gomez
Date: 09/22/2025
Description:
10/07/2025
    This program is designed to use the same functionality and base program used from the arduino uno but 
    uses the esp32_now protocol to recieve wireless communication between two esp32s.
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
bool testing = false;

int motors[8] = {m1_fwd, m2_fwd, m3_fwd, m4_fwd, m1_bkwd, m2_bkwd, m3_bkwd, m4_bkwd};
int motors_fwd[4] = {m1_fwd, m2_fwd, m3_fwd, m4_fwd};
int motors_bkwd[4] = {m1_bkwd, m2_bkwd, m3_bkwd, m4_bkwd};
int motors_left[4] = {m1_bkwd, m2_fwd, m3_fwd, m4_bkwd};
int motors_right[4] = {m1_fwd, m2_bkwd, m3_bkwd, m4_fwd};
int motors_fleft[2] = {m2_fwd, m3_fwd};
int motors_fright[2] = {m1_fwd, m4_fwd};
int motors_bleft[2] = {m2_bkwd, m3_bkwd};
int motors_bright[2] = {m1_bkwd, m4_bkwd};

void setup() {
    Serial.begin(115200);
    for(int i = 0; i < 8; i++)
        pinMode(motors[i], OUTPUT);
    motorsOff();
}

//Change testing to a true
#if testing
void loop(){
        test();
}
#endif

#if !testing
void loop() {
    xvalue = analogRead(x_axis);
    yvalue = analogRead(y_axis);

    state = joystickControl(xvalue, yvalue);

    switch(state){
        case '1':
            motorsFwd();
            break;
        case '2':
            motorsBkwd();
            break;
        case '3':
            motorsLeft();
            break;
        case '4':
            motorsRight();
            break;
        case ' ':
        default:
            motorsOff();
            state = ' ';
            break;
    }
}
#endif