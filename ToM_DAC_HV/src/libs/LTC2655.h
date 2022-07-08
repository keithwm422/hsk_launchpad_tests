// 7 bit address with a write command
// then need 3 bytes of data.
/*
Specs of the application of this LTC2655 to the HV module on HELIX
From Paul Smith schematic for ToM++ rev 2
V_Potential Full Scale =
((4.096 V / 2.21) / 4.64 V) X 10 kV =
3.994 kV
LSB = 0.975 V

I_Potential Full Scale =
((4.096 V / 13) / 4.64 V) X 1.5 mA =
102 uA
LSB =24.9 nA

V_Cathode Full Scale =
((4.096 V X 1.13) / 4.64 V) X 10 kV =
9.975 kV
LSB = 2.435 V

I_Cathode Full Scale =
((4.096 V X 1.13) / 4.64V) X 1.5 mA =
1.496 mA
LSB =365 nA

*/
#pragma once
#ifndef LTC2655_H
#define LTC2655_H

#include "Arduino.h"
#include "Wire.h"

class LTC2655{
public:
    // every write has a first byte (after 7-bit address) that needs 4 bit command, upper nibble
    enum class CMD{
        IN_REG_WRITE_N = 0x00,
        UPDATE_REG_N = 0x01,
        IN_REG_WRITE_N_AND_UPDATE_ALL = 0x02,
        WRITE_AND_UPDATE_N = 0x03,
        PWR_DWN_REG = 0x04,
        PWR_DWN_ALL = 0x05,
        SELECT_INTERNAL_REF = 0x06,
        SELECT_EXT_REF = 0x07,
    };
    // every write has a first byte (after 7-bit address) that needs 4 bit addr which is the channels to use, lower nibble
    enum class ADDR{
        DAC_CHA = 0x00,   // V_PGM_CATHODE
        DAC_CHB = 0x01,   // I_PGM_CATHODE
        DAC_CHC = 0x02,   // V_PGM_POTENTIAL
        DAC_CHD = 0x03,    // I_PGM_POTENTIAL
        DAC_CHALL = 0x0F,       
    };
    enum class DAC_CH {A, B, C, D};
    enum class VREF {VDD, INTERNAL_V};
    uint8_t attach(TwoWire& w, uint8_t address_in){
      wire_ = &w;
      addr_ = address_in;
      reg_[0].vref = VREF::INTERNAL_V;
      reg_[1].vref = VREF::INTERNAL_V;
      reg_[2].vref = VREF::INTERNAL_V;
      reg_[3].vref = VREF::INTERNAL_V;
      uint8_t data[3]={0};
      data[0] = (((uint8_t) CMD::SELECT_INTERNAL_REF << 4) | (uint8_t) (ADDR::DAC_CHALL)); // first the command with the address
      wire_->beginTransmission(addr_);
      wire_->write(data[0]);
      wire_->write(data[1]);
      wire_->write(data[2]);
      return wire_->endTransmission();
    }
    uint8_t analogWrite(uint16_t value, ADDR channel){
      reg_[(uint8_t) channel].data=value;
      uint8_t data[3]={0};
      // some fancy bitshifts to a uint16_t
      data[1] = (uint8_t) ((value >> 4) & 0x0FF); // should be the MSByte
      data[2] = (uint8_t) ((value & 0x0F) << 4); // should be the LSnibble
      data[0] = ((uint8_t) CMD::WRITE_AND_UPDATE_N << 4) | (uint8_t) (channel); // first the command with the address
      wire_->beginTransmission(addr_);
      wire_->write(data[0]);
      wire_->write(data[1]);
      wire_->write(data[2]);
      return wire_->endTransmission();      
    }
    uint8_t analogWrite(uint16_t value, uint8_t channel){
      return analogWrite(value, (ADDR) (channel));
    }
    uint16_t return_value(ADDR ch){
      return reg_[(uint8_t)ch].data;
    }
private:  //fns

private:  // vars
    struct DACInputData{
        VREF vref;
        uint16_t data;
    };
    const uint8_t I2C_ADDR {0x10};

    uint8_t addr_ {I2C_ADDR};

    DACInputData reg_[4];

    TwoWire* wire_;
};

#endif // LTC2655_H
