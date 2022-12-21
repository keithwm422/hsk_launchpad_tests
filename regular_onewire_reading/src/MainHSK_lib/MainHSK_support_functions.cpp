/*
 * MainHSK_support_functions.cpp
 * 
 * Defines a set of functions to act as responses to received local commands for the Main HSK board
 *
 */

/*******************************************************************************
* Defines
*******************************************************************************/
#include "MainHSK_support_functions.h"
#include "Wire.h"
#include "MCP4728.h"
/* Buffer for outgoing data */
uint8_t outgoingData [255] = {0};
byte addr_1[8]; // make this bigger to accomadate the 8 bytes per temp probe. 
/* Variables for performing a temperature reading + storing it in an array of uint8_t */
uint32_t TempRead;
uint32_t * tmp;
uint8_t default_address=96;
MCP4728 dac; // for SSOut dac comms
DS2482* DS;
byte data[12];

#define numMaxTempSensors 30
#define numChannels 8
#define numAddrBytes 8
byte allAddr[numChannels][numMaxTempSensors][numAddrBytes] = {0};

// float max and min temps for SSOut
// also use DAC resolution (VDD ref).
float f_max = 125; // 125 degrees celsius
float f_min = -55; // -55 degrees celsius
float delta_f = (f_max-f_min)/(4096); //stepsize in 12 bit resolution of the float degrees celsius
uint8_t channel_map[18]={3,0,1,2,3,0,2,0,2,0,2,0,2,0,2,0,2,0};
uint8_t address_map[18]={1,0,0,0,0,7,7,6,6,5,5,4,4,3,3,2,2,1};
void whatToDoIfISR(uint8_t * data)
{		
	TempRead = analogRead(TEMPSENSOR);
	
	tmp = &TempRead;
	
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
int whatToDoIfHeaterControl(uint8_t * data, uint8_t len, uint8_t * respData)
{	
	/* If 1 byte, change all channels  to data byte*/ 
	// will write the serial.write to the linduino here
//	serial.write(254); // always write this first byte
	//needs to be about respData packets..
	int retval = 0;
	(0x01 & len) ? retval=1 : retval=2;
	*respData = 254;
	if (retval == 1) {
		*(respData + 1) = 170;
		*(respData + 2) = *data;
	}
	else if (retval == 2) {
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

int whatToDoIfTestHeaterControl(uint8_t* data, uint8_t len)
{
	/* If 1 byte, change all channels  to data byte*/
	// will write the serial.write to the linduino here
//	serial.write(254); // always write this first byte
	//needs to be about respData packets..
	int retval = 0;
	(0x01 & len) ? retval = 1 : retval = 2;
	//	(0x01 & len) ? serial.write(170) : serial.write(171);
	return retval;
	// now just write the next value, and if len >1 write the second byte in data also. 
//	stream.write(*data);
//	if ((int)len == 2) serial.write(*data + 1);
	/* Fill outgoing data with that byte */
	// where do we put the data?
}

bool OneWireSetup(uint8_t channel, DS2482& ds1){ 
  DS=&ds1;
  DS->reset(); 
  //configure DS2482 to use active pull-up instead of pull-up resistor 
  //configure returns 0 if it cannot find DS2482 connected 
  if (!DS->configure(DS2482_CONFIG_APU)) {
    return false; 
  } 
  else {
    OneWireFindAddresses(DS);
    DS->selectChannel(channel);
    return true;
  }
}

// find all addresses on all channels
void OneWireFindAddresses(DS2482* ds1){

  DS=ds1;
  
  byte currentlyCopiedAddr[numAddrBytes]; // short term address storage

  // go through each channel
  for (int activeChan=0; activeChan<numChannels; activeChan++) {

    DS->selectChannel(activeChan);
    DS->wireResetSearch();
    
    // go through each sensor per channel
    int activeSensor = 0;
    while (activeSensor<numMaxTempSensors) {
      // copy the address if one is found
      if (DS->wireSearch(currentlyCopiedAddr)){
	memcpy((uint8_t *) &allAddr[activeChan][activeSensor],(uint8_t *) &currentlyCopiedAddr, sizeof(currentlyCopiedAddr));
	for(int addrByte=0;addrByte<numAddrBytes;addrByte++){
	  allAddr[activeChan][activeSensor][addrByte] = currentlyCopiedAddr[addrByte];
	}
	activeSensor++;
      }
      else { // if no (more) sensor found, move on
	break;
      }
    }
  }
}

void OneWireCopyAddress(byte* returnAddr, uint8_t channel, int sensor_index){
  // memcpy((uint8_t *) &returnAddr,(uint8_t *) &allAddr[channel][sensor_index], sizeof(allAddr[channel][sensor_index]));
  for(int addrByte=0;addrByte<numAddrBytes;addrByte++){
    returnAddr[addrByte] = allAddr[channel][sensor_index][addrByte];
  }
}

float OneWireReadOneChannel(uint8_t channel, int temp_probe_index, DS2482& ds1){
  DS=&ds1;
  DS->reset();
  // need to select the channel of the onewirebridge device before doing any reads.
  DS->selectChannel(channel);

  byte addr[8];
  for(int i=0; i<8;i++) addr[i]=allAddr[channel][temp_probe_index][i];
  float celsius;
  DS->wireReset();
  DS->wireSelect(addr);
  DS->wireWriteByte(0x44);
  delay(750);       // maybe 750ms is enough, maybe not
  DS->wireReset();
  DS->wireSelect(addr);
  DS->wireWriteByte(0xBE); 
  for (int i=0;i<9;i++){
    data[i]=DS->wireReadByte();
  }
  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00)
    raw = raw & ~7; // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20)
    raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40)
    raw = raw & ~1; // 11 bit res, 375 ms
  // default is 12 bit resolution, 750 ms conversion time
  celsius = (float)raw / 16.0;
  return celsius;
}

// // returns 0 if one of the DAC's can't be set to 0 (and doesn't read 0).
// bool SSOutSetupSingle(TwoWire_1& wire, uint8_t LDAC,uint8_t addr_i){
//   dac.attatch(wire, LDAC); // set the LDAC pin of launchpad and the i2c comms
//   dac.setID(default_address+addr_i);
//   delayMicroseconds(100);
//   dac.selectVref(MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD);
//   dac.analogWrite(0,0,0,0);
//   dac.readRegisters();
//   uint16_t dac_val=0;
//   for(int j=0;j<4;j++) dac_val=dac_val | dac.getDACData(j);
//   if(dac_val==0) return 1;
//   return 0;
// }

// // returns 0 if one of the DAC's can't be set to 0 (and doesn't read 0).
// bool SSOutSetup(TwoWire_1& wire, uint8_t LDAC){
//   bool return_val;
//   uint8_t i=0;
//   while(i<8){
//     SSOutSetupSingle(wire,LDAC,i);
//     i++;
//   }
//   return true;
// }

// bool SSOutProgram(uint16_t * data){
//   uint8_t i=0;
//   while(i<8){
//     dac.setID(default_address+i);
//     delayMicroseconds(100);
//     dac.analogWrite(*data,*data,*data,*data);
//     i++;
//   }
//   delayMicroseconds(100);
//   return 1;
// }

// bool SSOutChannelProgram(uint16_t data, uint8_t address, uint8_t channel){
//   dac.setID(default_address+address);
//   dac.analogWrite(channel, data);
//   return 1;
// }


// uint16_t SSOut_float_temp_to_volt(float val){
//   float number=(val-f_min)/delta_f;
//   int rounded = round(number);
//   uint16_t return_val = (uint16_t) rounded;
//   return return_val;
// //  (val-f_low)/(delta_f);  
// }

// void convert_DAC_outputs(SSOut_vals_t * SSOut_values_DAC_ptr, uint16_t * SSOut_volts){
//   for(int i=0;i<10;i++){
//     *(SSOut_volts+i)=SSOut_float_temp_to_volt(SSOut_values_DAC_ptr->DCTTemp[i]);
//   }
//   *(SSOut_volts+10)=SSOut_values_DAC_ptr->DCTPressureReference;
//   *(SSOut_volts+11)=SSOut_float_temp_to_volt(SSOut_values_DAC_ptr->Magnet1);
//   *(SSOut_volts+12)=SSOut_float_temp_to_volt(SSOut_values_DAC_ptr->Magnet2);
//   *(SSOut_volts+13)=SSOut_float_temp_to_volt(SSOut_values_DAC_ptr->Power1);
//   *(SSOut_volts+14)=SSOut_float_temp_to_volt(SSOut_values_DAC_ptr->Power2);
//   *(SSOut_volts+15)=SSOut_float_temp_to_volt(SSOut_values_DAC_ptr->RICH1);
//   *(SSOut_volts+16)=SSOut_float_temp_to_volt(SSOut_values_DAC_ptr->RICH2);
//   *(SSOut_volts+17)=SSOut_float_temp_to_volt(SSOut_values_DAC_ptr->Last);

// }

// void SSOutProgram_all(uint16_t * out){
//   uint8_t incrementer=0;
//   for (int i =0; i<8; i++){
//     dac.setID(default_address+i);
//     dac.analogWrite(*(out+incrementer),*(out+incrementer+1),*(out+incrementer+2), *(out+incrementer+3));
//     incrementer=i+4;
//   }
// }
// // the passed values are the data, the address to use is in the array address_map, and the channel is in channel_map;
// void SSOutProgram_flight(SSOut_vals_t * SSOut_values_DAC_ptr, uint16_t * SSOut_volts){
//   convert_DAC_outputs(SSOut_values_DAC_ptr, SSOut_volts);
//   for(int i=0;i<10;i++){
//     SSOutChannelProgram(*(SSOut_volts+i), address_map[i], channel_map[i]);
//   }
//}
