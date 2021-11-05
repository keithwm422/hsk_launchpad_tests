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

#define LED_IDLE_PERIOD 2000
unsigned long LEDIdleTime;
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
  voltage=4095; //set the DACs to 3V for magnet test
  SSOutProgram(&voltage);
  //set_DACs_staircase(); //set the DACs for magnet test to weird ass values

}

void loop() {
  // put your main code here, to run repeatedly: 
  if ((long) (millis() - LEDIdleTime) > 0) {
    LEDIdleTime = millis() + LED_IDLE_PERIOD;
    switch_LED();
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
