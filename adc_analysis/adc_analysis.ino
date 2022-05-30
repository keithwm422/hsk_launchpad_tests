#include "src/MainHSK_support_functions.h"

unsigned long i2c_SSOut=3;
TwoWire_1 *wire_ssout= new TwoWire_1(i2c_SSOut); // i2C object for the i2c port on the launchpad
bool SSOut_conn;
uint8_t address;
uint8_t channel;
uint16_t voltage;
uint8_t SSOut_LDAC=2; // pin 2 of launchpad is the LDAC pin we use here. 
bool led_is_high=true;
int LED_PIN=40;
//#define LED_PIN RED_LED
//uint16_t adv_val=(uint16_t) analogRead(A2);
uint16_t adv_val;
#define LED_IDLE_PERIOD 500
unsigned long LEDIdleTime;
#define ADC_IDLE_PERIOD 500
unsigned long ADCIdleTime;

void setup() {
  pinMode(LED_PIN,OUTPUT); //LED
  digitalWrite(LED_PIN,LOW);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("starting...");
    //I2C(3) is SSOut DACs
  wire_ssout->setModule(i2c_SSOut);
  //SSOut_conn=SSOutSetupSingle(*wire_ssout,SSOut_LDAC,2);
  SSOut_conn=SSOutSetup(*wire_ssout,SSOut_LDAC);
  voltage=0; //set the DACs to 3V for magnet test
  SSOutProgram(&voltage);
  //set_DACs_staircase(); //set the DACs for magnet test to weird ass values
  analogReadResolution(12);
  LEDIdleTime = millis() + LED_IDLE_PERIOD;
  ADCIdleTime = millis() + 750;

}

void loop() {
  // put your main code here, to run repeatedly: 
  if ((long) (millis() - LEDIdleTime) > 0) {
    LEDIdleTime = millis() + LED_IDLE_PERIOD;
    switch_LED();
    SSOutProgram(&voltage);
    voltage+=1;
    if(voltage>=4095) voltage=0;
  }

  if ((long) (millis() - ADCIdleTime) > 0) {
    ADCIdleTime = millis() + ADC_IDLE_PERIOD;
    uint16_t adv_val=(uint16_t) analogRead(A10);
    Serial.print(voltage,DEC);
    Serial.print(",");
    Serial.print(adv_val,DEC);
    Serial.println();
  }
  
}

void switch_LED(){
  if(led_is_high){
    led_is_high=false;
    digitalWrite(LED_PIN,LOW);
  }
  else{    
    led_is_high=true;
    digitalWrite(LED_PIN,HIGH);
  }
}
