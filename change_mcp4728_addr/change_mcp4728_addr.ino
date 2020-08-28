#include "SoftI2cMaster.h"

SoftI2cMaster wire;
uint8_t SCL_pin = 23;
uint8_t SDA_pin = 24;
uint8_t LDAC_pin = 26;
uint8_t returns;

uint8_t const_byte2=0x61; //0b 0 1 1 0 0 0 0 1
uint8_t const_byte3=0x62; //0b 0 1 1 0 0 0 1 0
uint8_t const_byte4=0x63; //0b 0 1 1 0 0 0 1 1

uint8_t curr_addr=0x60; //0b 0 1100 0 0 0
uint8_t first_byte=curr_addr << 1;
uint8_t curr_addr_to_write=(((curr_addr << 3) & 0xFF) >> 1) | const_byte2;
 
uint8_t new_addr= 0x63; //0b 0 1100 0 0 1
uint8_t new_addr_to_write1=(((new_addr << 3) & 0xFF) >> 1) | const_byte3;
uint8_t new_addr_to_write2=(((new_addr << 3) & 0xFF) >> 1) | const_byte4;


void setup() {
  pinMode(LDAC_pin,OUTPUT);
  digitalWrite(LDAC_pin,HIGH);
  // put your setup code here, to run once:
  Serial.begin(9600);
  wire.init(SCL_pin,SDA_pin);
  print_deets();
  Serial.print(wire.start(first_byte));  // R/!W is 0 so this is a write command. (and needs to be current 7-bit address with R/!W set to 0). 
  //Serial.print("\n");   // checkout page 42 of https://www.mouser.com/datasheet/2/268/22187E-12972.pdf
  Serial.print(wire.ldacwrite(curr_addr_to_write,LDAC_pin)); // should be current address bits in binary 0b 0 1 1 A2 A1 A0 0 1 
  //Serial.print("\n");
  wire.write(new_addr_to_write1); // now the new address bits 0b 0 1 1 A2 A1 A0 1 0 
  wire.write(new_addr_to_write2); // and again the new address bits 0b 0 1 1 A2 A1 A0 1 1
  wire.stop();
}

void loop() {
  // put your main code here, to run repeatedly: 
  delay(100000);  
}

void print_deets(){
    // For debugging
  Serial.print(new_addr,HEX);
  Serial.print(" is the new addr \n");
  Serial.print(curr_addr,HEX);
  Serial.print(" is the current addr \n");
  Serial.print(curr_addr_to_write,HEX);
  Serial.print(" is the current addr  with one-bit set\n");
  Serial.print(new_addr_to_write1,HEX);
  Serial.print(" is the new addr with second bit set \n");
  Serial.print(new_addr_to_write2,HEX);
  Serial.print(" is the new addr with second and first bit set \n");
  
}
