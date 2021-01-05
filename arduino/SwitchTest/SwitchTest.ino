#define SWITCH1 8
#define SWITCH2 9
#define SWITCH3 10

#define LED1 13

void setup()
{
  pinMode(SWITCH1, INPUT);
  pinMode(SWITCH2, INPUT);
  pinMode(SWITCH3, INPUT);
  pinMode(LED1, OUTPUT);

}

void loop()
{ 
  if (digitalRead(SWITCH1) == HIGH) {
    digitalWrite(LED1, HIGH);
  } else {
    digitalWrite(LED1, LOW);
  }

  if (digitalRead(SWITCH2) == HIGH) {
    digitalWrite(LED1, HIGH);
  } else {
    digitalWrite(LED1, LOW);
  }

  if (digitalRead(SWITCH3) == HIGH) {
    digitalWrite(LED1, HIGH);
  } else {
    digitalWrite(LED1, LOW);
  }




  
  delay(100);
}
