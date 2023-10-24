#include "Arduino.h"
#include "Wire.h"
#include "DAC7678.h"
#include "defines.h"


void DAC7678::begin(TwoWire& w_, uint8_t * address) {
  Wire_=&w_;
  for(int i=0;i<3;i++){
    dac7678_addresses[i]=address[i];
    reset(dac7678_addresses[i]); 		  // Sends Power-On Reset command to DAC
  }
  //reset(); 		  // Sends Power-On Reset command to DAC
  //disable();      // Make sure outputs are powered off
}

void DAC7678::setVREF(int which) {
  transmit(which, CMD_INTREF_RS, 0x00, 0x10); 
}

void DAC7678::setVREF() {
  transmit(CMD_INTREF_RS, 0x00, 0x10); 
}


void DAC7678::reset(int which) {
  // Issues Reset command (Same as Power-On reset)
  transmit(which, CMD_SOFT_RESET, 0x00, 0x00);
}

void DAC7678::offMode(uint8_t channel, uint8_t mode) {
  // Configure off mode for specified channel
  off_mode[channel] = mode;
}

void DAC7678::offMode(uint8_t mode) {
  // Configure off mode for all channels
  for (uint8_t x = 0; x <= 7; x++){
    off_mode[x] = mode;
  }
}

void DAC7678::LDAC(bool _state) {
  // Configure LDAC mode for all channels
  if (_state) {
    LDAC_reg = 0x00;        
  }
  else {
    LDAC_reg = 0xFF;
  }

  transmit(CMD_LDAC, LDAC_reg, 0x00);
}

void DAC7678::LDAC(uint8_t _channel, bool _state) {
  // Configure LDAC mode for specified channel
  if (_state) {
    LDAC_reg &= ~(1 << _channel);   
  }
  else {
    LDAC_reg |= (1 << _channel); 
  }

  transmit(CMD_LDAC, LDAC_reg, 0x00);
}

void DAC7678::select(uint8_t _channel) {
  // Select channel for update
  transmit(CMD_SELECT + _channel, 0x00, 0x00);
}

void DAC7678::enable(int which) {
  // Enables all the channels at once

  transmit(which, CMD_POWER_DOWN, 0x1F, 0xE0);
}

void DAC7678::enable() {
  // Enables all the channels at once

  transmit(CMD_POWER_DOWN, 0x1F, 0xE0);
}

void DAC7678::disable() {
  // Disables all the channels in one go
  // Doing it one by one is required in case of different off modes per channel

  for (uint8_t x = 0; x <= 7; x++) {
    disableChannel(x);
  }
}

void DAC7678::enableChannel(uint8_t channel) {
  // Enables the specific DAC output channel
  
  if (channel < 8) {

    unsigned int x = (unsigned int) (0x20 << channel);  // Channels are offset +5 bits
    uint8_t msdb = (uint8_t)(x >> 8);
    uint8_t lsdb  = x & 0xFF;

    transmit(CMD_POWER_DOWN, msdb, lsdb);
  }
}

void DAC7678::disableChannel(uint8_t channel) {
  // Disables the specific DAC output channel
  
  if (channel < 8) {

    unsigned int x = (unsigned int) (0x20 << channel);  // Channels are offset +5 bits
    uint8_t msdb = (uint8_t)(x >> 8);
    msdb |= off_mode[channel];
    uint8_t lsdb  = x & 0xFF;

    transmit(CMD_POWER_DOWN, msdb, lsdb);
  }
}

void DAC7678::set(uint16_t _value) { 
  // Sets all DAC channels to specified value 
  
  for (uint8_t x = 0; x <= 7; x++) {
    set(x, _value);
  }
}

void DAC7678::set(uint8_t _channel, uint16_t _value) {
  // Set specified channel (0-7) to the specified value
  //   Check for out of range values
  if (_value >= 4096 || _value < 0) {
    return;
  }
  if (_channel < 8) { // Check for out of range Channel #
    // Sets the variables
    uint8_t _command = CMD_IS_LDAC_BASE_ADDR + _channel;
    uint8_t msdb = _value>>4;
    uint8_t lsdb  = (_value << 4) & 0xF0;
    // Send data to DAC
    transmit(_command, msdb, lsdb);
  }
}


void DAC7678::update(uint8_t _channel, int _value) {
  // Update specified channel (0-7) to the specified value

  
  //   Check for out of range values
  if (_value >= 4096 || _value < 0) {
    return;
  }

  if (_channel < 8) { // Check for out of range Channel #
    // Sets the variables
    uint8_t _command = CMD_WRITE_BASE_ADDR + _channel;

    unsigned int x = (unsigned int) (_value << 4);  // Data is offset +4 bits
    uint8_t msdb = (uint8_t)(x >> 8);
    uint8_t lsdb  = x & 0xFF;

	// Send data to DAC
    transmit(_command, msdb, lsdb);
  }
}

void DAC7678::clrMode(int _value) {
// Configures the DAC value to output on all channels when CLR pin is brought low
// Will set the CLR Code register to output either zero, half-range, full range or to ignore the CLR pin

  uint8_t lsdb = _value << 4;

	// Send data to DAC
  transmit(CMD_CLEAR_CODE, 0x00, lsdb);
}

unsigned int DAC7678::readChan(uint8_t _command) {

  unsigned int reading = 0;

  Wire_->beginTransmission(dac7678_address); 
  Wire_->write(_command);                    // Then send a command byte for the register to be read.
  Wire_->endTransmission();
  Wire_->requestFrom(dac7678_address, 2);

  if(2 <= Wire_->available()) {       // if two bytes were received
    reading = Wire_->read();          // receive high byte
    reading = reading << 8;         // shift high byte to be high 8 bits
    reading |= Wire_->read();
    reading = reading >> 4;
  }
  return reading;
}

unsigned int DAC7678::readDAC(uint8_t _command) {

  unsigned int reading = 0;

  Wire_->beginTransmission(dac7678_address);
  Wire_->write(_command);      // Then send a command byte for the register to be read.
  Wire_->endTransmission();
  Wire_->requestFrom(dac7678_address, 2);

  if(2 <= Wire_->available()) {   // if two bytes were received
    reading = Wire_->read();      // receive high byte
    reading = reading << 8;     // shift high byte to be high 8 bits
    reading |= Wire_->read();
    reading = reading >> 4;
  }
  return reading;
}

void DAC7678::transmit(int which, uint8_t _command, uint8_t _msdb, uint8_t _lsdb) {
  // Transmit the actual command and high and low bytes to the DAC
  dac7678_address=dac7678_addresses[which];
  Wire_->beginTransmission(dac7678_address);
  Wire_->write(_command);
  Wire_->write(_msdb);
  Wire_->write(_lsdb);
  Wire_->endTransmission();
}

void DAC7678::transmit(uint8_t _command, uint8_t _msdb, uint8_t _lsdb) {
  // Transmit the actual command and high and low bytes to the DAC
  Wire_->beginTransmission(dac7678_address);
  Wire_->write(_command);
  Wire_->write(_msdb);
  Wire_->write(_lsdb);
  Wire_->endTransmission();
}

void DAC7678::enable(uint8_t _state) {
  if (_state) {
    enable();
  }
  else {
    disable();
  }
}

void DAC7678::enable(uint8_t _channel, uint8_t _state) {
  if(_state) {
    enableChannel(_channel);
  }
  else {
    disableChannel(_channel);
  }
}
