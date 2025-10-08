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
    if ((yvalue >= 900) && ((xvalue <= 900) && (xvalue >= 400))) //foward
        return '1';
    if ((yvalue <= 400) && ((xvalue <= 900) && (xvalue >= 400))) //backward
        return '2';
    if ((xvalue >= 400) && ((yvalue <= 900) && (yvalue >= 400))) //strafe left
        return '3';
    if ((xvalue <= 900) && ((yvalue <= 900) && (yvalue >= 400))) //strafe right
        return '4';
    if ((xvalue <= 400) && (yvalue >= 900)) //front left
        return '5';
    if ((xvalue >= 900) && (yvalue >= 900)) //front right
        return '6';
    if ((xvalue <= 400) && (yvalue <= 400)) //back left
        return '7';
    if ((xvalue >= 900) && (yvalue >= 400)) //back right
        return '8';
    if (((yvalue >= 400) && (yvalue <= 900)) && ((xvalue >= 400) && (xvalue <= 900))) //Idle cause gorge is gay
        return ' ';
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