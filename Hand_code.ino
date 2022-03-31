/*
 * This is the the code used for the Prosthetic 3D-printed Hand. Uses an ESP32 to communicate over WIFI
 * using the ESPNow library to allow for two-way communication between the hand and the glove. The 
 * movement is done with 5 servo motors. This does include a haptic feddback tutorial with the use of
 * a button to send the data of the hand to the glove and lock up both the hand and the glove.
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
#include <ezButton.h>

// Pin to be used for the button
#define BUTTON_PIN 5

// Create servo object to control the servos
Servo myservo_pinky;  
Servo myservo_ring;
Servo myservo_middle;
Servo myservo_index;
Servo myservo_thumb;

// Create object for the button
ezButton button1(BUTTON_PIN);

int ADC_Max = 4096;

// Pin locations for the servos
int servoPin_pinky = 21;
int servoPin_ring = 18;
int servoPin_middle = 19;
int servoPin_index = 22;
int servoPin_thumb = 23;

// MAC Address of our glove receiver (each ESP32 has there own MAC Address if you use this please 
// use the MAC Addresses for your spefic board)
uint8_t broadcastAddress[] = {0xC8,0xC9,0xA3,0xC5,0xCA,0x64};

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

// Create a struct_message called buttonReadings to hold sensor readings
struct_message buttonReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// Peer to peer communication
esp_now_peer_info_t peerInfo;

// Variables to store the values received from the glove
int val_pinky;
int val_ring;
int val_middle;
int val_index;
int val_thumb;

// This variable is used to store the state of the button for the haptic tutorial
bool buttonState = 0;

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

  // If the button is pushed, the motors will no longer receive position updates from the glove
  // The values from the glove are muiltipled to better simulate a 1 to 1 ratio of movement
  if (!buttonState)
  {
    val_pinky = map(incomingReadings.val_pinky*3.5, 0, ADC_Max, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo_pinky.write(val_pinky);                  // set the servo position according to the scaled value
//    Serial.print("Pinky Servo Value: ");
//    Serial.println(val_pinky);
  
    val_ring = map(incomingReadings.val_ring*2.5 , 0, ADC_Max, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo_ring.write(val_ring);                  // set the servo position according to the scaled value
//    Serial.print("Ring Servo Value: ");
//    Serial.println(val_ring);
  
    val_middle = map(incomingReadings.val_middle*3, 0, ADC_Max, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo_middle.write(val_middle);                  // set the servo position according to the scaled value
//    Serial.print("Middle Servo Value: ");
//    Serial.println(val_middle);
  
    val_index = map(incomingReadings.val_index*3.5, 0, ADC_Max, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo_index.write(val_index);                  // set the servo position according to the scaled value
//    Serial.print("Index Servo Value: ");
//    Serial.println(val_index);

    val_thumb = map(incomingReadings.val_thumb*3.5, 0, ADC_Max, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo_thumb.write(val_thumb);                  // set the servo position according to the scaled value
//    Serial.print("Thumb Servo Value: ");
//    Serial.println(val_thumb);
  }

  // Sending servo position info to the glove for the haptic feed back
  buttonReadings.val_pinky = val_pinky;
  buttonReadings.val_ring = val_ring;
  buttonReadings.val_middle = val_middle;
  buttonReadings.val_index = val_index;
  buttonReadings.val_thumb = val_thumb;
}

// Setup function for the servos, button, and WIFI
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

  // Sets up the button for debounce to make sure that we are getting a clean signal from the button
  button1.setDebounceTime(50);
 
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

// This loop is the main function where we look at the status of the button and then send data to the 
// glove. We give it a 50ms delay to allow the servos to get into position.
void loop() {

  // Tell the button object where to start
  button1.loop();

  // If the button is pushed then the haptic feedback will be sent.
  // If the button is pushed again, then the haptic feedback will be canaled
  if (button1.isPressed() && buttonState == LOW)
    buttonState = HIGH;
  else if (button1.isPressed() && buttonState == HIGH)
    buttonState = LOW;

  buttonReadings.button1 = buttonState;
  
  //Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &buttonReadings, sizeof(buttonReadings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(50);
}
