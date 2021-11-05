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
//#include "Wire.h"
#include "Wire.h"
#include "DS2482.h"
#include "MainHSK_protocol.h"
/* If these commands are received */
void whatToDoIfISR(uint8_t * data);
							
int whatToDoIfHeaterControl(uint8_t * data, uint8_t len, uint8_t * respData);

int whatToDoIfTestHeaterControl(uint8_t* data, uint8_t len);

bool OneWireSetup(uint8_t channel, DS2482& ds1);

void OneWireReturnAddresses(uint8_t * array);

float OneWireReadOneChannel(uint8_t channel, int temp_probe_index, DS2482& ds1);

bool SSOutSetup(TwoWire& wire, uint8_t LDAC);

bool SSOutProgram(uint16_t * data);

bool SSOutChannelProgram(uint16_t data, uint8_t address, uint8_t channel);

uint16_t SSOut_float_temp_to_volt(float val);

void convert_DAC_outputs(SSOut_vals_t * SSOut_values_DAC_ptr, uint16_t * SSOut_volts);

void SSOutProgram_all(uint16_t * out);


