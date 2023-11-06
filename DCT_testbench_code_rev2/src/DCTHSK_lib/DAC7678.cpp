#include "Arduino.h"
#include "Wire.h"
#include "DAC7678.h"
#include "defines.h"


void DAC7678::begin(TwoWire& w_) {
  Wire_=&w_;
  //reset(dac7678_addresses[i]); 		  // Sends Power-On Reset command to DAC
  //reset(); 		  // Sends Power-On Reset command to DAC
  //disable();      // Make sure outputs are powered off
}

void DAC7678::setVREF(int which) {
  transmit(which, CMD_INTREF_RS, 0x00, 0x10); 
}

void DAC7678::reset(int which) {
  // Issues Reset command (Same as Power-On reset)
  transmit(which, CMD_SOFT_RESET, 0x00, 0x00);
}

void DAC7678::enable(int which) {
  // Enables all the channels at once

  transmit(which, CMD_POWER_DOWN, 0x1F, 0xE0);
}

void DAC7678::set(int which, uint16_t _value) { 
  // Sets all DAC channels to specified value 
  
  for (uint8_t x = 0; x <= 7; x++) {
    set(which, x, _value);
  }
}

void DAC7678::set(int which, uint8_t _channel, uint16_t _value) {
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
    transmit(which, _command, msdb, lsdb);
  }
}

void DAC7678::transmit(int which, uint8_t _command, uint8_t _msdb, uint8_t _lsdb) {
  // Transmit the actual command and high and low bytes to the DAC
  //dac7678_address=dac7678_addresses[which];
  Wire_->beginTransmission(dac7678_addresses[which]);
  Wire_->write(_command);
  Wire_->write(_msdb);
  Wire_->write(_lsdb);
  Wire_->endTransmission();
}
