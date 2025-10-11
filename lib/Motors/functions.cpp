#include "functions.h"

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

void motorsLeft() {
    motorsOff();
    for (int i = 0; i < 4; i++)
        digitalWrite(motors_left[i], HIGH);
}

void motorsRight() {
    motorsOff();
    for (int i = 0; i < 4; i++)
        digitalWrite(motors_right[i], HIGH);
}

char joystickControl(int xvalue, int yvalue){
    #if ESP32
        int high = 2500;
        int low = 1000;
    #else
        int high = 900;
        int low = 400;
    #endif
    if (((yvalue > low) && (yvalue < high)) && ((xvalue > low) && (xvalue < high))) //Idle
        return ' ';
    if ((yvalue >= high) && ((xvalue <= high) && (xvalue >= low))) //foward
        return '1';
    if ((yvalue <= low) && ((xvalue <= high) && (xvalue >= low))) //backward
        return '2';
    if ((xvalue > low) && ((yvalue < high) && (yvalue > low))) //strafe left
        return '3';
    if ((xvalue < high) && ((yvalue < high) && (yvalue > low))) //strafe right
        return '4';
    if ((xvalue < low) && (yvalue > high)) //front left
        return '5';
    if ((xvalue > high) && (yvalue > high)) //front right
        return '6';
    if ((xvalue < low) && (yvalue < low)) //back left
        return '7';
    if ((xvalue > high) && (yvalue > high)) //back right
        return '8';
    
    return ' '; // fallback idle
}

void test(){
    motorsOff();
    for (int i = 0; i < 4; i++){
        digitalWrite(motors_fwd[i], HIGH);
        delay(2000);
    }
    delay(2000);

    // motorsOff();
    // delay(2000);
    for (int i = 0; i < 4; i++){
        digitalWrite(motors_fwd[i], LOW);
        delay(2000);
    }
    delay(2000);

    // motorsOff();
    // delay(2000);
    for (int i = 0; i < 4; i++){
        digitalWrite(motors_bkwd[i], HIGH);
        delay(2000);
    }
    delay(2000);

    // motorsOff();
    // delay(2000);
    for (int i = 0; i < 4; i++){
        digitalWrite(motors_bkwd[i], LOW);
        delay(2000);
    }
    delay(2000);
}