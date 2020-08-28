/*
  DS2482 library for Arduino
  Copyright (C) 2009-2010 Paeae Technologies

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have readd a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

	crc code is from OneWire library

	-Updates:
		* fixed wireReadByte busyWait (thanks Mike Jackson)
		* Modified search function (thanks Gary Fariss)

*/
#include "Arduino.h"  // according http://blog.makezine.com/2011/12/01/arduino-1-0-is-out-heres-what-you-need-to-know/

#include "Wire_nonblocking.h"

#include "DS2482_nonblocking_v3.h"


#define PTR_STATUS 0xf0
#define PTR_READ 0xe1
#define PTR_CONFIG 0xc3


DS2482::DS2482(uint8_t addr, TwoWire& w_ds)
{
	mAddress = 0x18 | addr;
        wire_ds=&w_ds;
}

//-------helpers
uint8_t DS2482::RXStatus(){
       return wire_ds->getRxStatus((uint8_t)1);
}

uint8_t DS2482::TXStatus(){
       return wire_ds->getTxStatus();
}

bool DS2482::CheckStates(){
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
/*
bool DS2482::CheckStates(){
       int timeout=0;
       uint8_t rx_state;
       uint8_t tx_state;
       do{
         //if(timeout >=I2C_TRIES) return false;
         timeout++;
         rx_state=RXStatus();
         tx_state=TXStatus();
       } while(rx_state!=0 || tx_state!=0);
       return true;
}
*/

void DS2482::begin(){
	wire_ds->beginTransmission(mAddress);
}
void DS2482::end(){
	wire_ds->endTransmission_nonblocking(1);
}

bool DS2482::setReadPtr(uint8_t readPtr){
        if(CheckStates()){
          begin();
          wire_ds->write(0xe1);  // changed from 'send' to 'write' according http://blog.makezine.com/2011/12/01/arduino-1-0-is-out-heres-what-you-need-to-know/'
          wire_ds->write(readPtr);
          end();
          return true;
        }
        else return false;
}

bool DS2482::writeByte(uint8_t byte_to_write){
        if(CheckStates()){
          begin();
          wire_ds->write(byte_to_write);
          end();
          return true;
        }
        else return false;
}

bool DS2482::writeTwoBytes(uint8_t byte_1, uint8_t byte_2){
        if(CheckStates()){
          begin();
          wire_ds->write(byte_1);
          wire_ds->write(byte_2);
          end();
          return true;
        }
        else return false;
}
/*
bool DS2482::Request(){
        if(CheckStates()) {
          wire_ds->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
          return true;
        }
        else return false;
}

bool DS2482::readBytes(uint8_t *data){ // can return 255 if the i2c states never are idle when they need to be.
        if(Request()){
          if(CheckStates()){
            int i=0;
            while(wire_ds->available()) {
              data[i] = wire_ds->read();
              i++;
            }
            return true;
          }
          else return false;
        }
        else return false;
}
*/
uint8_t DS2482::readByte(){ // can return 255 if the i2c states never are idle when they need to be.
        if(CheckStates()) wire_ds->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
        else return 255;
        if(CheckStates()){
          if(wire_ds->available()) return wire_ds->read();
          else return 255;
        }
        else return 255;
}

/*
uint8_t DS2482::readByte(){ // can return 255 if the i2c states never are idle when they need to be.
        if(CheckStates()) wire_ds->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
        if(CheckStates()) return wire_ds->read();
          
}
*/

uint8_t DS2482::wireReadStatus(bool setPtr){
        if (setPtr){
          if(setReadPtr(PTR_STATUS)) return readByte();
          else return 253;   // 253 is different and means the readptr was tried to be set but couldnt be set
        }
        else return readByte();
}
/*
uint8_t DS2482::wireReadStatus(bool setPtr){
        if (setPtr){
          if(setReadPtr(PTR_STATUS)) return readByte();
        }
        else return readByte();
}
*/
uint8_t DS2482::busyWait(bool setReadPtr){
        // only need to set the ptr once if it needs set. so if true , setreadptr can be set to false after the first time
        uint8_t status;
        int loopCount = ONEWB_TRIES;
        bool settheptr=setReadPtr;
        do{
          status=wireReadStatus(settheptr);
          settheptr=false; // it doesnt need to be set again
          if (--loopCount <= 0){
            mTimeout = 1;
            break;
          }
          delayMicroseconds(20);
        } while(status & DS2482_STATUS_BUSY);
        return status;
}
/*
uint8_t DS2482::busyWait(bool setReadPtr){
        // only need to set the ptr once if it needs set. so if true , setreadptr can be set to false after the first time
        uint8_t status;
        int loopCount = ONEWB_TRIES;
        bool settheptr=setReadPtr;
        do{
          status=wireReadStatus(settheptr);
          settheptr=false; // it doesnt need to be set again
          if (--loopCount <= 0 || status==255 || status==253){
            mTimeout = 1;
            break;
          }
          delayMicroseconds(20);
        } while(status & DS2482_STATUS_BUSY);
        return status;
}
*/
/*
uint8_t DS2482::OneWB(bool setReadPtr){
        // only need to set the ptr once if it needs set. so if true , setreadptr can be set to false after the first time
        uint8_t status;
        int loopCount = 1000;
        bool settheptr=setReadPtr;
        do{
          status=wireReadStatus(settheptr);
          settheptr=false; // it doesnt need to be set again
          if(status==255 || status==253) break;
          if (--loopCount <= 0){
            mTimeout = 1;
            break;
          }
          delayMicroseconds(20);
        } while(status & DS2482_STATUS_BUSY);
        return status;
}
*/
// returns true if its still busy
// returns false if it is not busy, so you dont have to wait.
/*bool DS2482::busyWait(bool setReadPtr){
        //int tries = 20;
        //do{
         // tries--;
          //if(tries<= 0) break;
          //uint8_t status = OneWB(setReadPtr);
        //} while (status & DS2482_STATUS_BUSY);
        uint8_t status = OneWB(setReadPtr);
        if(status & DS2482_STATUS_BUSY) return true;
        else return false;
}
*/
//----------interface
bool DS2482::reset(){
        mTimeout = 0;
        return writeByte(0xF0); // to reset
}

bool DS2482::configure(uint8_t config){
        busyWait(true);
        writeTwoBytes(0xD2,(config | (~config)<<4));
        return readByte() == config;
}

bool DS2482::selectChannel(uint8_t channel){
        uint8_t ch, ch_read;
        switch (channel){
          case 0:
          default:
            ch = 0xf0;
            ch_read = 0xb8;
            break;
          case 1:
            ch = 0xe1;
            ch_read = 0xb1;
            break;
          case 2:
            ch = 0xd2;
            ch_read = 0xaa;
            break;
          case 3:
            ch = 0xc3;
            ch_read = 0xa3;
            break;
          case 4:
            ch = 0xb4;
            ch_read = 0x9c;
            break;
          case 5:
            ch = 0xa5;
            ch_read = 0x95;
            break;
          case 6:
            ch = 0x96;
            ch_read = 0x8e;
            break;
          case 7:
            ch = 0x87;
            ch_read = 0x87;
            break;
        };
        busyWait(true);
        writeTwoBytes(0xC3,ch);
        busyWait();
        uint8_t check = readByte();
        return check == ch_read;
}

bool DS2482::wireReset(){
        busyWait(true);
        writeByte(0xB4);
        uint8_t status = busyWait();
        return status & DS2482_STATUS_PPD ? true : false;
}

void DS2482::wireWriteByte(uint8_t b){
        busyWait(true);
        writeTwoBytes(0xA5,b);
}

uint8_t DS2482::wireReadByte(){
        busyWait(true);
        writeByte(0x96);
        busyWait();
        setReadPtr(PTR_READ);
        return readByte();
}

void DS2482::wireWriteBit(uint8_t bit){
        busyWait(true);
        writeTwoBytes(0x87,bit ? 0x80 : 0);
}

uint8_t DS2482::wireReadBit(){
        wireWriteBit(1);
        uint8_t status = busyWait(true);
        return status & DS2482_STATUS_SBR ? 1 : 0;
}

void DS2482::wireSkip(){
        wireWriteByte(0xcc);
}

void DS2482::wireSelect(uint8_t rom[8]){
        wireWriteByte(0x55);
        for (int i=0;i<8;i++)
          wireWriteByte(rom[i]);
}

#if ONEWIRE_SEARCH
void DS2482::wireResetSearch(){
        searchExhausted = 0;
        searchLastDisrepancy = 0;
        for(uint8_t i = 0; i<8; i++)
          searchAddress[i] = 0;
}

uint8_t DS2482::wireSearch(uint8_t *newAddr){
        uint8_t i;
        uint8_t direction;
        uint8_t last_zero=0;
        if (searchExhausted) return 0;
        if (!wireReset()) return 0;
        busyWait(true);
        wireWriteByte(0xf0);
        for(i=1;i<65;i++){
          int romByte = (i-1)>>3;
          int romBit = 1<<((i-1)&7);
          if (i < searchLastDisrepancy) direction = searchAddress[romByte] & romBit;
          else direction = i == searchLastDisrepancy;
          busyWait();
          writeTwoBytes(0x78,direction ? 0x80 : 0);
		//wire_ds->write(0x78);
		//wire_ds->write(direction ? 0x80 : 0);
          uint8_t status = busyWait();
          uint8_t id = status & DS2482_STATUS_SBR;
          uint8_t comp_id = status & DS2482_STATUS_TSB;
          direction = status & DS2482_STATUS_DIR;
          if (id && comp_id) return 0;
          else{
            if (!id && !comp_id && !direction) last_zero = i;
	  }
          if (direction) searchAddress[romByte] |= romBit;
          else searchAddress[romByte] &= (uint8_t)~romBit;
        }
        searchLastDisrepancy = last_zero;
        if (last_zero == 0) searchExhausted = 1;
        for (i=0;i<8;i++)
          newAddr[i] = searchAddress[i];
	return 1;
}
#endif

#if ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

uint8_t DS2482::crc8( uint8_t *addr, uint8_t len)
{
	uint8_t crc=0;

	for (uint8_t i=0; i<len;i++)
	{
		uint8_t inbyte = addr[i];
		for (uint8_t j=0;j<8;j++)
		{
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0x8C;

			inbyte >>= 1;
		}
	}
	return crc;
}

#endif
