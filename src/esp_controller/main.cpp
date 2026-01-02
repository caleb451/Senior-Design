/*
Author: Caleb Gomez
Date: 09/22/2025
Description:
10/07/2025
    Updated to wirelessly control the robot using joystick control while communicating with 2 different esp32s.
    This program will send the 'state' of the joystick to the esp32 on the robot.
*/
#define CONTROLLER
#include <Arduino.h>
#include <pins.h>
#include <functions.h>

#ifdef ESP32
    #include <esp_now.h>
    #include <WiFi.h>
#endif

//MAC Address for the Controller:   F8:B3:B7:45:45:E4
//MAC Address for the Robot:        F8:B3:B7:3E:FC:84

uint8_t robotAddress[] = {0xF8, 0xB3, 0xB7, 0x3E, 0xFC, 0x84};

typedef struct controllerData{
    char state;
}controllerData;

controllerData outData;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
    Serial.println("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

int xvalue = 0;
int yvalue = 0;
char state = ' ';

void setup(){
    Serial.begin(115200);
    // Serial.println(WiFi.macAddress()); //get the MAC Adress of the Controller 

    WiFi.mode(WIFI_STA);
    Serial.println("Controller initialized");

    if(esp_now_init() != ESP_OK){
        Serial.println("Controller failed to initialize");
        return;
    }

    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, robotAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }

    Serial.println("Controller ready to send!");
    
}

void loop(){
    xvalue = analogRead(x_axis);
    yvalue = analogRead(y_axis);
    Serial.print("\n\nX-value: ");

    // Serial.print(xvalue);
    // Serial.print("\nY-value: ");
    // Serial.print(yvalue);
    // delay(100);

    state = joystickControl(xvalue, yvalue);

    outData.state = state;

    // Send the state to the robot ESP32
    esp_now_send(robotAddress, (uint8_t *) &outData, sizeof(outData));

    delay(100); // small delay to avoid flooding
}