#include "uart.h" // to check if comp-launchpad is okay
#include "inc/hw_uart.h"
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
uint8_t all_values[10000]={0};
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
uint32_t uart_error_reg;
uint8_t lower_nibble_error_reg;
const int buttonPin = PUSH2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
int read_val;
int read_iter;
void setup() {
  // put your setup code here, to run once:
  // LED on launchpad
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  //usbdeviceaddress=USBDevAddrGet(USB0_BASE);
  Serial1.begin(115200);
  //Serial.println(UARTRxErrorGet(0x4000C000),HEX);
  //Serial.println(usbdeviceaddress,HEX);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  LEDUpdateTime= millis()+LED_UPDATE_PERIOD;
  pinMode(buttonPin, INPUT_PULLUP);     
  read_iter=0;
}

void loop() {
  // always blink LED
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
  }
  // always read serial1
  read_val=Serial1.read();
  if(read_val!=-1){
    all_values[read_iter]=(uint8_t)read_val;
    read_iter++;
    if(read_iter>10000) read_iter=0;
  }
  // button stuff causes print to Serial line
  buttonState = digitalRead(buttonPin);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    digitalWrite(BLUE_LED,HIGH);
    Serial.begin(115200);
    for(int i=0; i<10000;i++){
      Serial.print(all_values[i],HEX);
    }
    Serial.end();
  }
  else     digitalWrite(BLUE_LED,LOW);
  //uart_error_reg=UARTRxErrorGet(0x4000C000);
  //lower_nibble_error_reg=uart_error_reg & 0x0F;
  
}

void switch_LED(){
  if(is_high){
    is_high=false;
    digitalWrite(LED,LOW);
  }
  else{    
    is_high=true;
    digitalWrite(LED,HIGH);
  }
}
