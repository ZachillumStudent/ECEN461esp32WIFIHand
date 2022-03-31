/*
 * This is the the code used for Glove to control the Prosthetic 3D-printed Hand. Uses an ESP32 to 
 * communicate over WIFI using the ESPNow library to allow for two-way communication between the hand 
 * and the glove. The finger tracking data is from 5 potentiometers.This does include haptic feddback 
 * tutorial with the use of a button on the hand to send the data from the hand to the glove 
 * and lock up both the hand and the glove. The glove's max etension is limited by 5 servos on the glove.
 * 
 * Serial prints are there for debugging purposes.
 * 
 * By Adam Erekson
 * 
 * Thanks to these websites and creators for example codes for ESPNow and ezButton
 * 
 * ESP32io.com
 * https://esp32io.com/tutorials/esp32-button-debounce
 * 
 * Rui Santos (ESPNow)
 * Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files.
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h> 

// Create servo object to control the servos
Servo myservo_pinky;  
Servo myservo_ring;
Servo myservo_middle;
Servo myservo_index;
Servo myservo_thumb;

int ADC_Max = 4096;

// Pin locations for the servos
int servoPin_pinky = 5;
int servoPin_ring = 18;
int servoPin_middle = 19;
int servoPin_index = 21;
int servoPin_thumb = 17;

// MAC Address of our hand receiver (each ESP32 has there own MAC Address if you use this please 
// use the MAC Addresses for your spefic board)
uint8_t broadcastAddress[] = {0xC8,0xC9,0xA3,0xC5,0xDF,0xC8};

int potPin_pinky = 36;
int potPin_ring = 39;
int potPin_middle = 34;
int potPin_index = 35;
int potPin_thumb = 32;

// Variable to store if sending data was successful
String success;

// Data structure for the send and recieve data.
// Structure must be the same for the reciever and sender.
typedef struct struct_message {
    int val_pinky;
    int val_ring;
    int val_middle;
    int val_index;
    int val_thumb;
    bool button1;
} struct_message;

// Create a struct_message called potReading to hold sensor readings
struct_message potReading;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// Peer to peer communication
esp_now_peer_info_t peerInfo;

// Variables to store manipulated sensor data
int val_pinky = 0;
int val_ring = 0;
int val_middle = 0;
int val_index = 0;
int val_thumb = 0;
bool button1 = 0;


// Callback when data is sent 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  
  // allows us to see the size of the data we are receiving from the glove.
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);

//  Serial.print("button1 Value: ");
//  Serial.println(incomingReadings.button1);
  button1 = incomingReadings.button1;

  // If the button is pushed, the pots will no longer send position updates to the hand
  // The values from the hand are manipulated to have the servos in the correct position
  // to restrict moevement on the glove.
  if (button1)
  {    
    int temp1 = 180 - incomingReadings.val_pinky;
    val_pinky = abs(temp1);
    myservo_pinky.write(val_pinky);                  // set the servo position according to the scaled value
//    Serial.print("Pinky Servo Value: ");
//    Serial.println(val_pinky);

    int temp2 = 180 - incomingReadings.val_ring;
    val_ring = abs(temp2);
    myservo_ring.write(val_ring);                  // set the servo position according to the scaled value
//    Serial.print("Ring Servo Value: ");
//    Serial.println(val_ring);

    int temp3 = 180 - incomingReadings.val_middle;
    val_middle = abs(temp3);
    myservo_middle.write(val_middle);                  // set the servo position according to the scaled value
//    Serial.print("Middle Servo Value: ");
//    Serial.println(val_middle);

    int temp4 = 180 - incomingReadings.val_index;
    val_index = abs(temp4);
    myservo_index.write(val_index);                  // set the servo position according to the scaled value
//    Serial.print("Index Servo Value: ");
//    Serial.println(val_index);

    int temp5 = 180 - incomingReadings.val_index;
    val_thumb = abs(temp5);
    myservo_thumb.write(val_thumb);                  // set the servo position according to the scaled value
//    Serial.print("Thumb Servo Value: ");
//    Serial.println(val_thumb);
  }

  // Set servos back to full to allow for full range of motion
  else
  {
    myservo_pinky.write(180);
    myservo_ring.write(180);
    myservo_middle.write(180);
    myservo_index.write(180);  
    myservo_thumb.write(180);
  }
}

// Setup function for the servos and WIFI
void setup() {

  // Timers for the servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Standard 50hz servo
  myservo_pinky.setPeriodHertz(50);
  myservo_ring.setPeriodHertz(50);
  myservo_middle.setPeriodHertz(50);
  myservo_index.setPeriodHertz(50);
  myservo_thumb.setPeriodHertz(50);

  // Attaching the servos to the pins on the ESP32
  myservo_pinky.attach(servoPin_pinky, 500, 2400);
  myservo_ring.attach(servoPin_ring, 500, 2400);
  myservo_middle.attach(servoPin_middle, 500, 2400);
  myservo_index.attach(servoPin_index, 500, 2400);
  myservo_thumb.attach(servoPin_thumb, 500, 2400);
  
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

// This loop is the main function where we look at the status of the 5 pots and then send data to the 
// hand. We give it a 50ms delay to allow the servos to get into position.
void loop() {

  // When the button on the hand is pressed the 
  if (!incomingReadings.button1)
  {
    potReading.val_pinky = analogRead(potPin_pinky);
    potReading.val_ring = analogRead(potPin_ring);
    potReading.val_middle = analogRead(potPin_middle);
    potReading.val_index = analogRead(potPin_index);
    potReading.val_thumb = analogRead(potPin_thumb);
  }
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &potReading, sizeof(potReading));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(50);
}
