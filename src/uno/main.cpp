#include <Arduino.h>
#include <functions.h>

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

//Change this to a 1 and the other to a 0 to test motors
#if testing
void loop(){
        test();
}
#endif

#if !testing
void loop() {
    xvalue = analogRead(x_axis);
    yvalue = analogRead(y_axis);

    //This is to print the values of the joystick to see the x and y values
    // Serial.print("\n\nX-value: ");
    // Serial.print(xvalue);
    // Serial.print("\nY-value: ");
    // Serial.print(yvalue);
    // delay(100);

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