/*
Author: Caleb Gomez
Date: 09/22/2025
Description:
10/07/2025
    This program is designed to use the same functionality and base program used from the arduino uno but 
    uses the esp32_now protocol to recieve wireless communication between two esp32s.
*/
#define RECIEVER
#include <Arduino.h>
#include <functions.h>

#ifdef ESP32
    #include <esp_now.h>
    #include <WiFi.h>
#endif



//MAC Address for the Robot: F8:B3:B7:3E:FC:84

typedef struct robotData {
    char command;
} robotData;

robotData incomingData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataBytes, int len) {
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  char state = incomingData.command;

  Serial.print("Received command: ");
  Serial.println(state);

  switch (state) {
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
    default:
      motorsOff();
      break;
  }
}

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
    //Serial.println(WiFi.macAddress()); //get the MAC Adress of the Controller 
    for(int i = 0; i < 8; i++)
        pinMode(motors[i], OUTPUT);
    motorsOff();

    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register callback to handle received messages
    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("ESP-NOW Receiver Ready");

}

//Change testing to a true
void loop() {
    
}