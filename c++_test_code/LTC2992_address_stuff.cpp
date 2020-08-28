// The Addresses on the Demo Circuit (DC)2561 are 8-bit write addresses for the LTC2992.
// The 7-bit addresses that are taken from LTC2992.h from LTSketchbook do not have the R/W bit set.
// To get the 8-bit addresses, bit-shift up one and bit-wise or with 0x0(w) 0x1(r).
// The code below generates the following table to show we haven't fucked this up. 

#include <stdio.h> 
#include <inttypes.h>
#include <unistd.h>

#define W_BIT 0x00
#define R_BIT 0x01

//  Address assignment (7 -bit address-for reads << 1 and | 0)

// LTC2992 I2C Address                 //  AD1       AD0
// #define LTC2992_I2C_ADDRESS 0x67    //  High      Low
// #define LTC2992_I2C_ADDRESS 0x68    //  Float     High
// #define LTC2992_I2C_ADDRESS 0x69    //  High      High
#define LTC2992_I2C_ADDRESS 0x6A       //  Float     Float
// #define LTC2992_I2C_ADDRESS 0x6B    //  Float     Low
// #define LTC2992_I2C_ADDRESS 0x6C    //  Low       High
// #define LTC2992_I2C_ADDRESS 0x6D    //  High      Float
// #define LTC2992_I2C_ADDRESS 0x6E    //  Low       Float
// #define LTC2992_I2C_ADDRESS 0x6F    //  Low    Low


void convert_7bit_address(uint16_t addr);
int main()
{
  convert_7bit_address(LTC2992_I2C_ADDRESS);
  return 0;
}

void convert_7bit_address(uint16_t addr){

  uint16_t addr_new_write = (addr << 1) | W_BIT;
  uint16_t addr_new_read = (addr << 1) | R_BIT;

  printf("#define LTC2992_I2C_WRITE_ADDRESS 0X%2X // write address\n", addr_new_write); 
  printf("#define LTC2992_I2C_READ_ADDRESS 0X%2X // read address\n", addr_new_read); 

}

