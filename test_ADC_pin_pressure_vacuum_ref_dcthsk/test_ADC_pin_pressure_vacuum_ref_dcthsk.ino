
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"

/* ADC VARIABLES */

uint16_t ADC_gas_panel_pressure;
float pressure_value_gas_panel=-1;
float gas_panel_resistor=160.0/1000.0; //put in kOhms so can use mA
float to_voltage=3.3/4095.0;
//3.3 V FSR on ADC, R is resistor, 14.6959 at 1 atm is close to 20mA current draw, 4mA current is vacuum. Linear device. 
float gas_panel_conversion= to_voltage*(14.69 - 0.0)/(19.92*gas_panel_resistor - 4.6*gas_panel_resistor); 
// reading in the ADC value, then multiply by the gas_panel_conversion to get a pressure in psi
/* SENSOR STATES AND VARS */
#define SENSOR_UPDATE_PERIOD 4000// how often to check/write sensors
unsigned long SensorUpdateTime=0; // keeping LED to visualize no hanging
uint32_t TempRead=0;
// for Launchpad LED example of timer used for reading sensors without holding uC attention
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...test of ADC pin 25.");
  // state the ADC res to use:
  analogReadResolution(12);

    // LED on launchpad
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly: 
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
    TempRead=analogRead(TEMPSENSOR);
    ADC_gas_panel_pressure=(uint16_t) analogRead(A5); // gas panel pressure sensor
    pressure_value_gas_panel=gas_panel_conversion*ADC_gas_panel_pressure;
  }
  if((long)(millis()-SensorUpdateTime) >0){
    SensorUpdateTime+= SENSOR_UPDATE_PERIOD;
    ADC_gas_panel_pressure=(uint16_t) analogRead(A5); // gas panel pressure sensor
    pressure_value_gas_panel=gas_panel_conversion*ADC_gas_panel_pressure;
    Serial.print("Pressure is: ");
    Serial.print(pressure_value_gas_panel,4);
    Serial.println(" (psi)");
    Serial.print("ADC value was: ");
    Serial.print((int)ADC_gas_panel_pressure);
    Serial.println();
  }
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
