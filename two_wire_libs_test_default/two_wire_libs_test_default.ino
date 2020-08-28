// try out multiple I2C twowire libs
#include "driverlib/uart.h"
#include "inc/hw_nvic.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <inc/hw_i2c.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>
#include <inc/hw_pwm.h>
#include <driverlib/i2c.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/udma.h>
#include <driverlib/pwm.h>
#include <driverlib/ssi.h>
#include <driverlib/systick.h>
#include <driverlib/adc.h>
#include <string.h>
#include "Wire.h"
#define  WIRE_INTERFACES_COUNT 4
unsigned long  i2c_1=1;
unsigned long i2c_3=3;
// declare 2 two wire objs
TwoWire *wire_1= new TwoWire(i2c_1); // i2C object for the i2c port on the launchpad
TwoWire *wire_2= new TwoWire(i2c_3); // i2C object for the i2c port on the launchpad
//TwoWire Wire1(i2c_1);
//TwoWire Wire3(i2c_3);
#define PCF8574_I2C_ADDRESS_1 32
#define PCF8574_I2C_ADDRESS_2 33
char one_byte;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Hello, starting...");
  Serial.print("\n");
// begin both twowire libs (default off LEDs!)
//  wire_2->setModule(i2c_3);
//  wire_1->setModule(i2c_1);
//  Wire1.begin();
  wire_2->begin();
  wire_1->begin();
//  Wire3.begin();

}

void loop() {
  while(Serial.available()>0){
    one_byte=Serial.read();
    Serial.print("\n");
//    Serial.print("try writing i to test_prio_array? \n");
//    memcpy(test_prio_array,&i, sizeof(i));
    if(one_byte==49) Serial.print("1\n");
    else if(one_byte==50){ //2
      Serial.print(PCF1(1));
      Serial.print("2\n");
    }
    else if(one_byte==51){  //3
      Serial.print(PCF1(0));
      Serial.print("3\n");
    }
    else if(one_byte == 52){  //4
      Serial.print(PCF2(1));
      Serial.print("4\n");
    }
    else if(one_byte==53){ // 5
      Serial.print(PCF2(0));
      Serial.print("5\n");
    }
    else if(one_byte==54){  //6
// read pcf1
      PCF1_read();
      Serial.print(" 6\n");
    }
    else if(one_byte==55){  //7
//read pcf2
      PCF2_read();
      Serial.print(" 7\n");
    }
    else {
      Serial.print("AH\n");
    }
  }
  delay(3000);
}
uint8_t PCF1(uint8_t to_write){
//    uint32_t err_from_slave=I2CSend2bytes(base, PCF8574_I2C_ADDRESS_1,1, to_write);
// change to wire libs
  wire_1->beginTransmission(PCF8574_I2C_ADDRESS_1);
  wire_1->write(1);
  wire_1->write(to_write);
  return wire_1->endTransmission();

/*
  Wire1.beginTransmission(PCF8574_I2C_ADDRESS_1);
  Wire1.write(1);
  Wire1.write(to_write);
  return Wire1.endTransmission();
*/
}

uint32_t PCF2(uint8_t to_write){
//    uint32_t err_from_slave=I2CSend2bytes(base, PCF8574_I2C_ADDRESS_2,1, to_write);
// change to wire libs
  wire_2->beginTransmission(PCF8574_I2C_ADDRESS_2);
  wire_2->write(1);
  wire_2->write(to_write);
  return wire_2->endTransmission();

/*
  Wire3.beginTransmission(PCF8574_I2C_ADDRESS_2);
  Wire3.write(1);
  Wire3.write(to_write);
  return Wire3.endTransmission();
*/
}

void PCF1_read(){
  Serial.print(wire_1->requestFrom(PCF8574_I2C_ADDRESS_1, 2));    // request 2 bytes from slave address
  while(wire_1->available()){
    uint8_t c1 = wire_1->read(); // receive a byte as character
    Serial.print(c1);         // print the character
  }
}

void PCF2_read(){
  Serial.print(wire_2->requestFrom(PCF8574_I2C_ADDRESS_2, 2));    // request 2 bytes from slave address
  while(wire_2->available()){
    uint8_t c2 = wire_2->read(); // receive a byte as character
    Serial.print(c2);         // print the character
  }
}

/*
 * 
 * Keith and patrick non blocking i2C CODE
 * // Two methods to use I2C comms on th TM4C123 launchpad
// //"i2_adc.h" are functions borrowed from this source: 
// https://www.digikey.com/eewiki/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL 
// modified to work with different ports and better readability, plus my own comments.
// Also here is example of using Wire Library from arduino: Wire.h
// decide at the setup prompt which to use. 

#include "Wire_nonblocking.h"

#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
uint32_t ADCValues[1];
unsigned long i2c_port;
uint8_t to_write_two_wire[2]={0};
uint8_t to_write;
uint8_t command_byte;
uint32_t err_from_slave;
uint8_t PCF_I2C_WRITE_ADDRESS;
uint16_t write_2_bytes;
uint8_t * write_ptr;
#define PCF8574_I2C_ADDRESS 32 // needs to be 7 bit address!
#define MAX_I2C_ADDRESS 32  // This is a special kind of type that is arbitray in size. 
#define PCF8574_I2C_READ_ADDRESS 32
#define SENSOR_UPDATE_PERIOD 1000
TwoWire wire;
unsigned long sensorUpdateTime=0;
int i;
typedef enum {
  SENSOR_STATE_I2C_3 = 0,
  SENSOR_STATE_I2C_3_COMPLETE = 1,
  SENSOR_STATE_I2C_3_WAIT = 2
} SensorState;
SensorState sState = SENSOR_STATE_I2C_3;

size_t write_num_bytes;
// Energia specific code
void setup() {
    i2c_port=3;
    wire.setModule(i2c_port);  // no change
    write_ptr= (uint8_t *) to_write_two_wire;
    Serial.begin(9600);
    Serial.print("hello\n");
    i=0;
    command_byte=0;
}

void loop() {
  if (i>=255) i=0;
  to_write_two_wire[1]= (uint8_t) i;
  switch(sState) {
    case SENSOR_STATE_I2C_3:
      if(wire.getTxStatus()!=I2C_BUSY){
        Serial.print("sending, i is ");
        Serial.print(i);
        Serial.print("\n"); // new line 
        wire.beginTransmission(MAX_I2C_ADDRESS); // needs to be nonblocking?
        write_num_bytes=wire.write(write_ptr,sizeof(to_write_two_wire));
        wire.endTransmission_nonblocking(1);
        sState = (SensorState) ((unsigned char) sState + 1);
        i+=10;
        break;
      }
      else {
        sState= SENSOR_STATE_I2C_3_WAIT;
        break;
      }
    case SENSOR_STATE_I2C_3_COMPLETE:
      sensorUpdateTime = millis() + SENSOR_UPDATE_PERIOD;
      sState = SENSOR_STATE_I2C_3_WAIT;
      break;
    case SENSOR_STATE_I2C_3_WAIT:
      if ((long) (millis() - sensorUpdateTime) > 0) {
        sState = SENSOR_STATE_I2C_3;
        break;
      }
    }
}

*/
