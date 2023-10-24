/*
 * DCTHSK_support_functions.cpp
 * 
 * Defines a set of functions to act as responses to received local commands for the DCT HSK board
 *
 */

/*******************************************************************************
* Defines
*******************************************************************************/
#include "DCTHSK_support_functions.h"
//#include "Wire.h"
#include "LTC2655.h"
#include "MCP3021.h"
#include "DAC7678.h"

MCP3021 pressure_adc;
LTC2655 HVDAC;
DAC7678 dac;                    // Set the DAC address to 0x48 (72 in decimal)
unsigned int Heater_vref = 5000;             // Voltage reference value for calculations (set to 5000 for internal reference)

/* Buffer for outgoing data */
uint8_t outgoingData [255] = {0};

/* Variables for performing a temperature reading + storing it in an array of uint8_t */
uint32_t TempReadC;
uint32_t * tmp;

void whatToDoIfISR(uint8_t * data)
{		
	TempReadC = analogRead(TEMPSENSOR);
	
	tmp = &TempReadC;
	
	/* Fills outgoing data buffer */
	for(int i=0; i < 4; i++)
    {
        *(data+i) = *tmp;
        *tmp = *tmp>>8;
    }
}

/* Function flow:
 * --Defines two variables, val & channel, to store a channel's pot value
 * --Fills outgoing packet header w/protocol standard
 * --Writes to the heater control board (?) commands it can understand
 * --Waits for a reply byte to verify the message was received
 * --Sends that reply byte back to the SFC for verification
 * 
 * Function params:
 * hdr_in:		Incoming header pointer, can find the data pts from the location
 * hdr_out:		Pointer to the outgoing packet header
 * stream:		Serial port where the heater control is connected
 * 
 * Function variables:
 * val:			Potentiometer value to set
 * channel:		Which potentiometer
 * 
 *  */
int whatToDoIfHeaterControl(uint8_t * data, uint8_t len)
{	
	/* If 1 byte, change all channels  to data byte*/ 
	// will write the serial.write to the linduino here
//	serial.write(254); // always write this first byte
	//needs to be about respData packets..
	int retval = 0;
	(0x01 & len) ? retval=1 : retval=2;
	return retval;
}

int whatToDoIfTestHeaterControl(uint8_t* data, uint8_t len, uint8_t * respData)
{
	/* If 1 byte, change all channels  to data byte*/

	int retval = 0;
	(0x01 & len) ? retval=3 : retval=4;
	*respData = 254;
	if (len == 1) {
		*(respData + 1) = 170;
		*(respData + 2) = *data;
	}
	else if (len == 2) {
		*(respData + 1) = 171;
		*(respData + 2) = *data;
		*(respData + 3) = *(data + 1);
	}
	else retval = EBADLEN;
//	(0x01 & len) ? serial.write(170) : serial.write(171);
	return retval;
	// now just write the next value, and if len >1 write the second byte in data also. 
//	stream.write(*data);
//	if ((int)len == 2) serial.write(*data + 1);
   	/* Fill outgoing data with that byte */
    // where do we put the data?
}
int whatToDoIfThermistors(uint8_t * respData){

  int retval=1;
  //match a read somehow from the thermistors, right now make this up?
  *respData = 0x05;
  return retval;
}

// returns 0 if one of the DAC's can't be set to 0 (and doesn't read 0).
bool PressureSetup(TwoWire& wire){
  pressure_adc.begin(wire,0); // set the address (second arg) to 0 since Mike ordered a specific addr device
  return 0;
}

bool HeaterSetup(TwoWire& wire){
  uint8_t address_in[3]={0x48,0x4A,0x4C}; // DACs are gnd, 5V, and floating on pin 2. this is the order they are declared here as well. 0x4C is last in array and is floating pin 2 DAC IC
  dac.begin(wire,&address_in[0]);                        // Initialize the DAC
  for(int i=0;i<3;i++){
    dac.setVREF(i); // now this needs to be done for all ICs
    dac.enable(i);              // Power up all DAC Channels
  }
  return 0;
}

bool HeaterExecute(unsigned char _channel, uint16_t set_value){
    dac.set(_channel,set_value);        // Write the value to all DAC channels
    return 0;    
}
uint16_t PressureRead(){
  return pressure_adc.readADC();
}

bool HVAllZero(){
  HVDAC.analogWrite(0,0); // channel 0 is VPGM, channel 1 is IPGM
  HVDAC.analogWrite(0,1); // channel 0 is VPGM, channel 1 is IPGM
  HVDAC.analogWrite(0,2); // channel 0 is VPGM, channel 1 is IPGM
  HVDAC.analogWrite(0,3); // channel 0 is VPGM, channel 1 is IPGM
  return 0;
}

// returns 0 if one of the DAC's can't be set to 0 (and doesn't read 0).
bool HVSetup(TwoWire& wire, uint8_t address_hvdac){
  HVDAC.attach(wire,address_hvdac); // address is 0x10 probs.
  HVAllZero();
  // The attach function sets all DAC channels to have internal 4.096 as reference.
  return 0;
}

bool CATChannelProgram(uint16_t data, uint8_t channel){
  //if(channel==0 || channel==1){
    HVDAC.analogWrite(data,channel); // channel 0 is VPGM, channel 1 is IPGM
    return 0;
  //}
  //else return 1;
}

bool POTChannelProgram(uint16_t data, uint8_t channel){
  //if(channel==2 || channel==3){
    HVDAC.analogWrite(data,channel); // channel 2 is VPGM, channel 3 is IPGM
    return 0;
  //}
  //else return 1;
}

