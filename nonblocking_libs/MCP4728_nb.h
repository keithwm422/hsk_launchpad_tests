
// Adapted from the github code to use the nonblocking lib Patrick and I made
// 07/03/2020
#pragma once
#ifndef MCP4728_H
#define MCP4728_H

#include "Arduino.h"
#include "Wire_nonblocking.h"

#define I2C_TRIES 100000

class MCP4728
{
public:

    enum class CMD
    {
        FAST_WRITE = 0x00,
        MULTI_WRITE = 0x40,
        SINGLE_WRITE = 0x58,
        SEQ_WRITE = 0x50,
        SELECT_VREF = 0x80,
        SELECT_GAIN = 0xC0,
        SELECT_PWRDOWN = 0xA0
    };
    enum class CMD_GENERAL
    {
        SELECT_GENERAL_CALL = 0x00,
        GENERAL_ADDR = 0x0C, //00001100
        GENERAL_WRITE_COMMAND_HDR = 0x03,
        GENERAL_WRITE_COMMAND_1= 0x01,
        GENERAL_WRITE_COMMAND_2= 0x01,
        GENERAL_RESTART = 0xC1,   //1100xxx1 (x=don't care)
        
    };
    enum class DAC_CH { A, B, C, D };
    enum class VREF { VDD, INTERNAL_2_8V };
    enum class PWR_DOWN { NORMAL, GND_1KOHM, GND_100KOHM, GND_500KOHM };
    enum class GAIN { X1, X2 };

    void attach(TwoWire& w, uint8_t pin)
    {
        wire_ = &w;
        pin_ldac_ = pin;
        pinMode(pin_ldac_, OUTPUT);
        enable(true);
        readRegisters();
    }

    void enable(bool b)
    {
        digitalWrite(pin_ldac_, !b);
    }

    uint8_t analogWrite(DAC_CH ch, uint16_t data, bool b_eep = false)
    {
        return analogWrite((uint8_t)ch, data, b_eep);
    }

    uint8_t analogWrite(uint8_t ch, uint16_t data, bool b_eep = false)
    {
        if (b_eep)
        {
            eep_[ch].data = data;
            return singleWrite(ch);
        }
        else
        {
            reg_[ch].data = data;
            return fastWrite();
        }
    }

    uint8_t analogWrite(uint16_t a, uint16_t b, uint16_t c, uint16_t d, bool b_eep = false)
    {
        if (b_eep)
        {
            reg_[0].data = eep_[0].data = a;
            reg_[1].data = eep_[1].data = b;
            reg_[2].data = eep_[2].data = c;
            reg_[3].data = eep_[3].data = d;
            return seqWrite();
        }
        else
        {
            reg_[0].data = a;
            reg_[1].data = b;
            reg_[2].data = c;
            reg_[3].data = d;
            return fastWrite();
        }
    }

    uint8_t selectVref(VREF a, VREF b, VREF c, VREF d)
    {
        reg_[0].vref = a;
        reg_[1].vref = b;
        reg_[2].vref = c;
        reg_[3].vref = d;

        uint8_t data = (uint8_t)CMD::SELECT_VREF;
        for (uint8_t i = 0; i < 4; ++i) bitWrite(data, 3 - i, (uint8_t)reg_[i].vref);

        if(writeByte(data)) return 1;
        else return 0;
    }

    uint8_t selectPowerDown(PWR_DOWN a, PWR_DOWN b, PWR_DOWN c, PWR_DOWN d)
    {
        reg_[0].pd = a;
        reg_[1].pd = b;
        reg_[2].pd = c;
        reg_[3].pd = d;

        uint8_t h = ((uint8_t)CMD::SELECT_PWRDOWN) | ((uint8_t)a << 2) | (uint8_t)b;
        uint8_t l = 0 | ((uint8_t)c << 6) | ((uint8_t)d << 4);

        if(writeTwoBytes(h,l)) return 1;
        else return 0;
    }

    uint8_t selectGain(GAIN a, GAIN b, GAIN c, GAIN d)
    {
        reg_[0].gain = a;
        reg_[1].gain = b;
        reg_[2].gain = c;
        reg_[3].gain = d;

        uint8_t data = (uint8_t)CMD::SELECT_GAIN;
        for (uint8_t i = 0; i < 4; ++i) bitWrite(data, 3 - i, (uint8_t)reg_[i].gain);

        if(writeByte(data)) return 1;
        else return 0;
    }

    uint8_t setID(uint8_t id) { 
        addr_ = id;
        return addr_;
    }
  
    // run checkStates then request then read them all. if something bad happens just return false and we know we didnt read registers
    bool readRegisters(){
        if(CheckStates()) wire_->requestFrom_nonblocking(addr_, (uint8_t) 24,(uint8_t) 1);
        else return false;
        if(CheckStates()) {
          if (wire_->available() == 24){
            for (uint8_t i = 0; i < 8; ++i){
                uint8_t data[3];
                bool isEeprom = i % 2;
                for (uint8_t i = 0; i < 3; ++i) data[i] = wire_->read();

                uint8_t ch = (data[0] & 0x30) >> 4;
                if (isEeprom){
                    read_eep_[ch].vref = (VREF)    ((data[1] & 0b10000000) >> 7);
                    read_eep_[ch].pd   = (PWR_DOWN)((data[1] & 0b01100000) >> 5);
                    read_eep_[ch].gain = (GAIN)    ((data[1] & 0b00010000) >> 4);
                    read_eep_[ch].data = (uint16_t)((data[1] & 0b00001111) << 8 | data[2]);
                }
                else{
                    read_reg_[ch].vref = (VREF)    ((data[1] & 0b10000000) >> 7);
                    read_reg_[ch].pd   = (PWR_DOWN)((data[1] & 0b01100000) >> 5);
                    read_reg_[ch].gain = (GAIN)    ((data[1] & 0b00010000) >> 4);
                    read_reg_[ch].data = (uint16_t)((data[1] & 0b00001111) << 8 | data[2]);
                }
            }
          }
          else return false;
        }
        else return false;
        return true;
    }
/*    // this function reads from the EEPROM and uses transitions of the LDAC pin
    void generalWriteAddress(uint8_t addr_old, uint8_t addr_new)
    {
        // get A2,A1,A0 first
        uint8_t addr_bits_old = addr_old & 0x07;
        uint8_t addr_bits_new = addr_new & 0x07;
        uint8_t write_2nd_byte = (0x03 << 5) | (addr_bits_old << 2) | 0x01;
        uint8_t write_3rd_byte = (0x03 << 5) | (addr_bits_new << 2) | 0x02;
        uint8_t write_4th_byte = (0x03 << 5) | (addr_bits_new << 2) | 0x03;
        // put LDAC high first
        digitalWrite(pin_ldac_, HIGH);
        // use the current address
        wire_->beginTransmission(addr_old);
        // use the old address bits
        wire_->write(write_2nd_byte);
        // now pull LDAC low 
        digitalWrite(pin_ldac_, LOW);
        // now the new ones twice
        wire_->write(write_3rd_byte);
        wire_->write(write_4th_byte);
        wire_->endTransmission();
    }
*/
    uint8_t getVref(uint8_t ch, bool b_eep = false) { return b_eep ? (uint8_t)read_eep_[ch].vref : (uint8_t)read_reg_[ch].vref; }
    uint8_t getGain(uint8_t ch, bool b_eep = false) { return b_eep ? (uint8_t)read_eep_[ch].gain: (uint8_t)read_reg_[ch].gain; }
    uint8_t getPowerDown(uint8_t ch, bool b_eep = false) { return b_eep ? (uint8_t)read_eep_[ch].pd : (uint8_t)read_reg_[ch].pd; }
    uint16_t getDACData(uint8_t ch, bool b_eep = false) { return b_eep ? (uint16_t)read_eep_[ch].data : (uint16_t)read_reg_[ch].data; }


    bool CheckStates(){
       int timeout=0;
       uint8_t rx_state;
       uint8_t tx_state;
       do{
         if(timeout >=I2C_TRIES) return false;
         timeout++;
         rx_state=RXStatus();
         tx_state=TXStatus();
       } while(rx_state==I2C_BUSY || tx_state==I2C_BUSY);
       return true;
    }

    uint8_t RXStatus(){
      return wire_->getRxStatus((uint8_t)1);
    }
    uint8_t TXStatus(){
      return wire_->getTxStatus();
    }



private:

    uint8_t fastWrite()
    {
      if(CheckStates()){
        begin();
        for (uint8_t i = 0; i < 4; ++i)
        {
            wire_->write((uint8_t)CMD::FAST_WRITE | highByte(reg_[i].data));
            wire_->write(lowByte(reg_[i].data));
        }
        end();
        return 1;
      }
      else return 0;
    }

    uint8_t multiWrite()
    {
      if(CheckStates()){
        begin();
        for (uint8_t i = 0; i < 4; ++i)
        {
            wire_->write((uint8_t)CMD::MULTI_WRITE | (i << 1));
            wire_->write(((uint8_t)reg_[i].vref << 7) | ((uint8_t)reg_[i].pd << 5) | ((uint8_t)reg_[i].gain << 4) | highByte(reg_[i].data));
            wire_->write(lowByte(reg_[i].data));
        }
        end();
        return 1;
      }
      else return 0;
    }

    uint8_t seqWrite()
    {
      if(CheckStates()){
        begin();
        wire_->write((uint8_t)CMD::SEQ_WRITE);
        for (uint8_t i = 0; i < 4; ++i)
        {
            wire_->write(((uint8_t)eep_[i].vref << 7) | ((uint8_t)eep_[i].pd << 5) | ((uint8_t)eep_[i].gain << 4) | highByte(eep_[i].data));
            wire_->write(lowByte(eep_[i].data));
        }
        end();
        return 1;
      }
      else return 0;
    }

    uint8_t singleWrite(uint8_t ch)
    {
      if(CheckStates()){
        begin();
        wire_->write((uint8_t)CMD::SINGLE_WRITE | (ch << 1));
        wire_->write(((uint8_t)eep_[ch].vref << 7) | ((uint8_t)eep_[ch].pd << 5) | ((uint8_t)eep_[ch].gain << 4) | highByte(eep_[ch].data));
        wire_->write(lowByte(eep_[ch].data));
        end();
        return 1;
      }
      else return 0;
    }

   bool writeByte(uint8_t byte_to_write){
        if(CheckStates()){
          begin();
          wire_->write(byte_to_write);
          end();
          return true;
        }
        else return false;
    }
    bool writeTwoBytes(uint8_t byte_1, uint8_t byte_2){
        if(CheckStates()){
          begin();
          wire_->write(byte_1);
          wire_->write(byte_2);
          end();
          return true;
        }
        else return false;
    }

    bool writeNBytes(uint8_t * bytes, int num_bytes){
        if(CheckStates()){
          begin();
          for(int i=0;i<num_bytes;i++) wire_->write(*(bytes+i));
          end();
          return true;
        }
        else return false;
    }

    void begin(){
      wire_->beginTransmission(addr_);
    }
    void end(){
      wire_->endTransmission_nonblocking(1);
    }

private:

    struct DACInputData
    {
        VREF vref;
        PWR_DOWN pd;
        GAIN gain;
        uint16_t data;
    };

    const uint8_t I2C_ADDR {0x60};

    uint8_t addr_ {I2C_ADDR};
    uint8_t pin_ldac_;

    DACInputData reg_[4];
    DACInputData eep_[4];
    DACInputData read_reg_[4];
    DACInputData read_eep_[4];

    TwoWire* wire_;
};

#endif // MCP4728_H
