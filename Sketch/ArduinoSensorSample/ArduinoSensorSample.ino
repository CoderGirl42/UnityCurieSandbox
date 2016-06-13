int shaft = 0;
int touch = 0;
const int pinLed = 3;                        // pin of led define here
const int potentiometer = 0;
const int touchSensor = 4;

void setup() {
  Serial.begin(9600);
  pinMode(touchSensor, INPUT);
  pinMode(potentiometer, INPUT);
  pinMode(pinLed, OUTPUT);                    // set led OUTPUT
}

void loop() {
  shaft = analogRead(potentiometer);

  touch = digitalRead(touchSensor);
  
  shaft = map(shaft, 0, 1023, -180, 179); 
  
  Serial.print(shaft);
  Serial.print(",");
  Serial.println(touch);

  if(touch == HIGH) {
    analogWrite(pinLed, 254);
  } else {
    analogWrite(pinLed, 0);
  }
  
  delay(15);
}
