/*
 * CommandResponse.h
 * 
 * Declares a set of functions to act as responses to received commands and 
 * error protocols
 *
 */

#pragma once

#include <PacketSerial.h>
#include <Arduino.h>
#include "Wire.h"
#include "Wire_1.h"
#include "DS2482.h"
//#include "MainHSK_protocol.h"
/* SSOut volts */
// this struct is the order of the pinout of the Science Stack connector. Pins [15,32] are the values down below, set by the Main HSK.
typedef struct SSOut_vals {
  float DCTTemp[10];
  uint16_t DCTPressureReference;
  float Magnet1;
  float Magnet2;
  float Power1;
  float Power2;
  float RICH1;
  float RICH2;
  float Last;
} SSOut_vals_t;

/* If these commands are received */
void whatToDoIfISR(uint8_t * data);
							
int whatToDoIfHeaterControl(uint8_t * data, uint8_t len, uint8_t * respData);

int whatToDoIfTestHeaterControl(uint8_t* data, uint8_t len);

bool OneWireSetup(uint8_t channel, DS2482& ds1);

void OneWireFindAddresses(DS2482* ds1);

void OneWireCopyAddress(byte* returnAddr, uint8_t channel, int sensor_index);

float OneWireReadOneChannel(uint8_t channel, int temp_probe_index, DS2482& ds1);

bool SSOutSetupSingle(TwoWire_1& wire, uint8_t LDAC,uint8_t addr_i);

bool SSOutSetup(TwoWire_1& wire, uint8_t LDAC);

bool SSOutProgram(uint16_t * data);

bool SSOutChannelProgram(uint16_t data, uint8_t address, uint8_t channel);

// this returns the 0-5Volt 12-bit DAC value for a given float temperature. 
uint16_t SSOut_float_temp_to_volt(float val);

// this converts the values of a sensor read to a 0-5Volt 12-bit DAC value that will be used to write to a DAC channel.
void convert_DAC_outputs(SSOut_vals_t * SSOut_values_DAC_ptr, uint16_t * SSOut_volts);

void SSOutProgram_all(uint16_t * out);

void SSOutProgram_flight(SSOut_vals_t * SSOut_values_DAC_ptr, uint16_t * SSOut_volts);
