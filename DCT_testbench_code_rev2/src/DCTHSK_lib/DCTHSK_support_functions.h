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


/* If these commands are received */
void whatToDoIfISR(uint8_t * data);
							
int whatToDoIfHeaterControl(uint8_t * data, uint8_t len);

int whatToDoIfTestHeaterControl(uint8_t* data, uint8_t len, uint8_t * respData);

int whatToDoIfThermistors(uint8_t* respData);

// for Pressure ADC
bool PressureSetup(TwoWire& wire);

uint16_t PressureRead();

bool HVAllZero();

// for the HV programming pins.
bool HVSetup(TwoWire& wire, uint8_t address_hvdac);

bool CATChannelProgram(uint16_t data, uint8_t channel);

bool POTChannelProgram(uint16_t data, uint8_t channel);
