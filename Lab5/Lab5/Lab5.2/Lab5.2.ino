
#include "DFRobot_RGBLCD1602.h"

const int colorR = 255;
const int colorG = 255;
const int colorB = 255;

DFRobot_RGBLCD1602 lcd(/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show

void setup() {
 Serial.begin(9600);
 pinMode(2, INPUT);
 pinMode(3, OUTPUT);
 pinMode(4, INPUT);
 pinMode(5, INPUT);

 lcd.init();
    
 lcd.setRGB(colorR, colorG, colorB);
    
  // Print a message to the LCD.
  lcd.print("L:    C:    R:   ");

    delay(1000);
}

void loop() {
  int detectpin1 = digitalRead(2);
  int detectpin2 = digitalRead(4);
  int detectpin3 = digitalRead(5);
  if(detectpin1 == LOW || detectpin2 == LOW || detectpin3 == LOW){
   digitalWrite(3, HIGH);
   
  }else{
   digitalWrite(3, LOW);
  }
  
  if(detectpin1 == LOW){
    lcd.setCursor(0,1);
    lcd.print("ON ");
  }else{
    lcd.setCursor(0,1);
    lcd.print("OFF");
  }

  if(detectpin2 == LOW){
    lcd.setCursor(6,1);
    lcd.print("ON ");
  }else{
    lcd.setCursor(6,1);
    lcd.print("OFF");
  }

  if(detectpin3 == LOW){
    lcd.setCursor(12,1);
    lcd.print("ON ");
  }else{
    lcd.setCursor(12,1);
    lcd.print("OFF");
  }
  
  delay(300);
}
