//https://github.com/adafruit/Adafruit_ADXL343/blob/master/examples/sensortest/sensortest.ino
//sample code from github credit given above
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

#include "DFRobot_RGBLCD1602.h"

#define ADXL343_SCK 13
#define ADXL343_MISO 12
#define ADXL343_MOSI 11
#define ADXL343_CS 10

const int colorR = 255;
const int colorG = 255;
const int colorB = 255;

DFRobot_RGBLCD1602 lcd(/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show


Adafruit_ADXL343 accel = Adafruit_ADXL343(ADXL343_CS, &SPI, 12345);


void setup(void)
{

  
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
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

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("                ");
  
  lcd.setCursor(0, 0);
  lcd.print(event.acceleration.x);

  lcd.setCursor(6, 0);
  lcd.print(event.acceleration.y);
  
  lcd.setCursor(12, 0);
  lcd.print(event.acceleration.z);

  lcd.setCursor(0, 1);
  if((event.acceleration.x < -0.5) || (event.acceleration.x > 0.3)){
    lcd.print("Motion: YES");
  }else if((event.acceleration.y < -0.5) || (event.acceleration.y > 0.3)){
    lcd.print("Motion: YES");
  }else if((event.acceleration.z < 9.5) || (event.acceleration.z > 10)){
    lcd.print("Motion: YES");
  }else{
    lcd.print("Motion: NO ");
  }
  
  delay(250);
}
