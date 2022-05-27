const int buttonPin = PUSH1;    
int voltPin = 13;   
int state = LOW;      
int reading;           
int previous = HIGH;   
#define R_LED RED_LED
#define G_LED GREEN_LED

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(voltPin, OUTPUT);
  pinMode(R_LED, OUTPUT); 
  pinMode(G_LED, OUTPUT);
  digitalWrite(R_LED, HIGH);
}

void loop()
{
  reading = digitalRead(buttonPin);
 
  if (reading == HIGH && previous == LOW ) {
    if (state == HIGH){
      state = LOW;
      digitalWrite(R_LED, HIGH);
      digitalWrite(G_LED, LOW);
    }
    else {
      state = HIGH;
      digitalWrite(voltPin, HIGH); 
      digitalWrite(G_LED, HIGH);
      digitalWrite(R_LED, LOW);  
    }
  }
  
  digitalWrite(voltPin, state);

  previous = reading;
}
