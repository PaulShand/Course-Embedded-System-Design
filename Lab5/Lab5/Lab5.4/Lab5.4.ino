/*
  Battery Monitor

  This example creates a Bluetooth® Low Energy peripheral with the standard battery service and
  level characteristic. The A0 pin is used to calculate the battery level.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include <ArduinoBLE.h>
#include "DFRobot_RGBLCD1602.h"

#define ADXL343_SCK 13
#define ADXL343_MISO 12
#define ADXL343_MOSI 11
#define ADXL343_CS 10


const int colorR = 255;
const int colorG = 255;
const int colorB = 255;

int LEFT = 0;
int CENTER = 0;
int RIGHT = 0;

DFRobot_RGBLCD1602 lcd(/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show


Adafruit_ADXL343 accel = Adafruit_ADXL343(ADXL343_CS, &SPI, 12345);



 // Bluetooth® Low Energy Battery Service
BLEService Motion("0a9d5f0e-70ea-11ed-a1eb-0242ac120002");

// Bluetooth® Low Energy Battery Level Characteristic
BLEStringCharacteristic movement("0a9d680a-70ea-11ed-a1eb-0242ac120002",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 40); // remote clients will be able to get notifications if this characteristic changes
BLEStringCharacteristic acce("0a9d6bc0-70ea-11ed-a1eb-0242ac120002",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 80); // remote clients will be able to get notifications if this characteristic changes
BLEStringCharacteristic sen("0a9d6e40-70ea-11ed-a1eb-0242ac120002",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 40); // remote clients will be able to get notifications if this characteristic changes

long previousMillis = 0;  // last time the battery level was checked, in ms


void setup() {
  Serial.begin(115200);    // initialize serial communication
  while (!Serial);

  pinMode(2, INPUT);
 pinMode(3, OUTPUT);
 pinMode(4, INPUT);
 pinMode(5, INPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("Lab5.4PaulShand");
  BLE.setAdvertisedService(Motion); // add the service UUID
  Motion.addCharacteristic(movement); // add the battery level characteristic
  Motion.addCharacteristic(acce); // add the battery level characteristic
  Motion.addCharacteristic(sen); // add the battery level characteristic
  BLE.addService(Motion); // Add the battery service
  //movement.writeValue(0); // set initial value for this characteristic

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_16_G);

  lcd.init();
    
    lcd.setRGB(colorR, colorG, colorB);
    
    // Print a message to the LCD.
    

    delay(1000);
}

void loop() {
  
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());


    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
        int detectpin1 = digitalRead(2);
        int detectpin2 = digitalRead(4);
        int detectpin3 = digitalRead(5);
        if(detectpin1 == LOW || detectpin2 == LOW || detectpin3 == LOW){
         digitalWrite(3, HIGH);
        }else{
         digitalWrite(3, LOW);
        }

        if(detectpin1 == LOW){
          RIGHT = 1;
        }else{
          RIGHT = 0;
        }
      
        if(detectpin2 == LOW){
          CENTER = 1;
        }else{
          CENTER = 0;
        }
      
        if(detectpin3 == LOW){
          LEFT = 1;
        }else{
          LEFT = 0;
        }

      
        sensors_event_t event;
        accel.getEvent(&event);

        char str[60];
        char str2[60];

        sprintf(str, "X:%.2fY:%.2fZ:%.2f", event.acceleration.x, event.acceleration.y,event.acceleration.z);
        sprintf(str2, "R:%d C:%d L:%d", RIGHT, CENTER,LEFT);

        if((event.acceleration.x < -0.5) || (event.acceleration.x > 0.3)){
          movement.writeValue("Motion: YES");
        }else if((event.acceleration.y < -0.5) || (event.acceleration.y > 0.3)){
          movement.writeValue("Motion: YES");
        }else if((event.acceleration.z < 9.5) || (event.acceleration.z > 10)){
          movement.writeValue("Motion: YES");
        }else{
          movement.writeValue("Motion: NO ");
        }
        
        
        acce.writeValue(str);

        
        sen.writeValue(str2);
        
        delay(250);
      }
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  
}
