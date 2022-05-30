// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.



void setup()
{
  // LED on launchpad
  pinMode(BLUE_LED, OUTPUT);
  Serial.begin(115200);
  Serial3.begin(115200); // join i2c bus (address optional for master)
  Serial.println("setup");
}

int counter=0;
void loop()
{
//  Serial.print("-------------counter: ");
//  Serial.println(counter++, DEC);

//      Serial.println("BAD!");
  delay(1000);
  digitalWrite(BLUE_LED,HIGH);

      Serial3.write((uint8_t) 0);
      delay(9);
      Serial3.write((uint8_t) 249);
      delay(9);
      Serial3.write((uint8_t) 218);
  delay(1000);
      Serial3.write( (uint8_t) 224 );
      delay(9);
      Serial3.write( (uint8_t) 1 );
      delay(9);
      Serial3.write( (uint8_t) 252 );   
      delay(9); 
      Serial3.write( (uint8_t) 7 );
      Serial3.write( (uint8_t) 0 );
      delay(9);
      Serial3.write( (uint8_t) 252 );
      delay(9);
      Serial3.write( (uint8_t) 218 );    
      delay(9);
  digitalWrite(BLUE_LED,LOW);
}
