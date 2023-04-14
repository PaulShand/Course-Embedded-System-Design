
void setup() {
 Serial.begin(9600);
 pinMode(2, INPUT);
 pinMode(3, OUTPUT);
}

void loop() {
  int detect = digitalRead(2);
  if(detect == LOW){
   digitalWrite(3, HIGH);
  }else{
   digitalWrite(3, LOW);
  }
  delay(300);
}
