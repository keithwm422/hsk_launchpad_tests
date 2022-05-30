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
typedef struct SSOut_vals {
  float SFCTemp1;  // was a float, now needs to be a DAC output where each votlage step (5-0)/4096 <-> (125-(-55))/4096.
  float SFCTemp2;
  float DCTTemp1;
  float DCTTemp2;
  float DCTTemp3;
  float DCTTemp4;
  float DCTTemp5;
  float DCTTemp6;
  float DCTTemp7;
  float DCTTemp8;
  float DCTTemp9;
  float DCTTemp10;
  float BatteryTemp1;
  float BatteryTemp2;
  uint16_t Power1;
  uint16_t Power2;
  uint16_t Power3;
  uint16_t Power4;
} SSOut_vals_t;
/* If these commands are received */
void whatToDoIfISR(uint8_t * data);
							
int whatToDoIfHeaterControl(uint8_t * data, uint8_t len, uint8_t * respData);

int whatToDoIfTestHeaterControl(uint8_t* data, uint8_t len);

bool OneWireSetup(uint8_t channel, DS2482& ds1);

void OneWireReturnAddresses(uint8_t * array);

float OneWireReadOneChannel(uint8_t channel, int temp_probe_index, DS2482& ds1);

bool SSOutSetupSingle(TwoWire_1& wire, uint8_t LDAC,uint8_t addr_i);
bool SSOutSetup(TwoWire_1& wire, uint8_t LDAC);

bool SSOutProgram(uint16_t * data);

bool SSOutChannelProgram(uint16_t data, uint8_t address, uint8_t channel);

uint16_t SSOut_float_temp_to_volt(float val);

void convert_DAC_outputs(SSOut_vals_t * SSOut_values_DAC_ptr, uint16_t * SSOut_volts);

void SSOutProgram_all(uint16_t * out);


