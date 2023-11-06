#ifndef DAC7678_H
#define DAC7678_H

#include "Arduino.h"
#include "Wire.h"
#include "defines.h"

class DAC7678 {  

    public:
	    //void begin(TwoWire& w_,uint8_t * address);
          void begin(TwoWire& w_);
	    void reset(int which);
	    void setVREF(int which);
	    void enable(int which);
	    void set(int which, uint16_t _value);
	    void set(int which, uint8_t channel, uint16_t _value);
    private:
          //uint8_t address_in[3]={0x48,0x4A,0x4C}; // DACs are gnd, 5V, and floating on pin 2. this is the order they are declared here as well. 0x4C is last in array and is floating pin 2 DAC IC
          uint8_t dac7678_addresses[3]={0x48,0x4A,0x4C}; // 3 DAC chips in one object
	    void transmit(int which, uint8_t _command, uint8_t _msdb, uint8_t _lsdb);
          TwoWire* Wire_;
};

#endif
