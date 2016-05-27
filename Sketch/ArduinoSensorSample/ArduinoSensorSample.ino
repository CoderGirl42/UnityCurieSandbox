int potentiometer = 0;
int touchSensor = 4;

int shaft;
int touch;
const int pinLed    = 3;                        // pin of led define here

void setup()
{
  Serial.begin(9600);
  pinMode(touchSensor, INPUT);
  pinMode(potentiometer, INPUT);
  pinMode(pinLed, OUTPUT);                    // set led OUTPUT
}

void loop()
{
  int oldShaft = shaft;
  shaft = analogRead(potentiometer);

  touch = digitalRead(touchSensor);
  
  shaft = map(shaft, 0, 1023, -179, 179); 
  
  Serial.print(shaft);
  Serial.print(",");
  Serial.println(touch);
  
  /*if (Serial.available() > 0) {
    int val = Serial.read();
    if(val == 's')
    
      Serial.println(shaft);
    
    if(val == 't')
      Serial.println(touch);
  }*/

  if(touch == HIGH)
  {
    analogWrite(pinLed, 254);
  }
  else
  {
    analogWrite(pinLed, 0);
  }
  
  delay(15);
}
