#include "uart.h" // to check if comp-launchpad is okay
#include "inc/hw_uart.h"
#define LED RED_LED
#define LED_UPDATE_PERIOD 1350
char all_values[10000]={0};
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
uint32_t uart_error_reg;
uint8_t lower_nibble_error_reg;
uint32_t uart_int_status;
uint16_t uart_raw_int_status_lower;
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
  Serial.begin(115200);
  //Serial.println(UARTRxErrorGet(0x4000C000),HEX);
  //Serial.println(usbdeviceaddress,HEX);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  LEDUpdateTime= millis()+LED_UPDATE_PERIOD;
  pinMode(buttonPin, INPUT_PULLUP);     
  read_iter=0;
}

void loop() {
  // always blink LED
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
    uart_int_status=UARTIntStatus(0x4000C000,false);
    Serial.println("LED is blinking and I am spamming.....");
    if(uart_int_status & 0x07F0){
      //Serial1.print("incoming non-zero interrupts" );
      uart_raw_int_status_lower=(uint16_t) (uart_int_status & 0x0FFF);
      Serial1.write((uint8_t) (uart_raw_int_status_lower));
      Serial1.write((uint8_t)(uart_raw_int_status_lower >> 8));
      Serial1.println();
    }
  }
  // always read serial line
/*  read_val=Serial.read();
  if(read_val!=-1){
    // if an actual read, just forward it to serial1
    Serial1.print((char)read_val);
  }
*/  
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
