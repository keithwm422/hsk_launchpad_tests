#ifndef DAC7678_H
#define DAC7678_H

#include "Arduino.h"
#include "Wire.h"
#include "defines.h"

class DAC7678 {  

    public:
	    void begin(TwoWire& w_,uint8_t * address);
	    void reset(int which);
	    void setVREF(int which);
	    void setVREF();
        void LDAC(bool _state);
        void LDAC(uint8_t _channel, bool _state); 
	    void offMode(uint8_t channel, uint8_t mode);
	    void offMode(uint8_t mode);
	    void enable(int which);
		void enable();
        void disable();
	    void enableChannel(uint8_t channel);
	    void disableChannel(uint8_t channel);
	    void set(int which, uint16_t _value);
	    void set(int which, uint8_t channel, uint16_t _value);
        void select(uint8_t _channel);
        void update(uint8_t _channel, int _value);
	    void clrMode(int _value);
	    uint8_t DAC;
	    unsigned int readChan(uint8_t _command);
	    unsigned int readDAC(uint8_t _command);
	
        // deprecated, kept for backwards compatibility
	    void enable(uint8_t state);
	    void enable(uint8_t channel, uint8_t state);

    private:
        uint8_t dac7678_addresses[3]; // 3 DAC chips in one object
        uint8_t dac7678_address;
        uint8_t off_mode[8];
        uint8_t LDAC_reg = 0xFF;
	    void transmit(int which, uint8_t _command, uint8_t _msdb, uint8_t _lsdb);
	    void transmit(uint8_t _command, uint8_t _msdb, uint8_t _lsdb);

        TwoWire* Wire_;
};

#endif
