#define SWITCH1 9
#define SWITCH2 10
#define SWITCH3 11

void setup()
{
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);
  pinMode(SWITCH3, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

}

void loop()
{ 

    int val1 = digitalRead(SWITCH1);
    int val2 = digitalRead(SWITCH2);
    int val3 = digitalRead(SWITCH3);
  
  if ((val1 == 0) || (val2 == 0) || (val3 == 0))  {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }



 Serial.print("Switch1:  ");
 Serial.print(val1);
  Serial.print("      Switch2:  ");
 Serial.print(val2);
  Serial.print("      Switch3:  ");
 Serial.println(val3);
 
  
  delay(100);
}
