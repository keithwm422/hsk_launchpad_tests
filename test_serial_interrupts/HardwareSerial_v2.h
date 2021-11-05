/*
  ************************************************************************
  *	HardwareSerial.h
  *
  *	Arduino core files for ARM Cortex-M4F: Tiva-C and Stellaris
  *		Copyright (c) 2012 Robert Wessels. All right reserved.
  *
  *
  ***********************************************************************

  2013-12-23 Limited size for RX and TX buffers, by spirilis
 
 
  Derived from:
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef HardwareSerial_v2_h
#define HardwareSerial_v2_h

#include <inttypes.h>
#include "Stream.h"

#define SERIAL_BUFFER_SIZE     256

#define UART1_PORTB	0 
#define UART1_PORTC	1

class HardwareSerial_v2 : public Stream
{

	private:
		unsigned char *txBuffer;
		unsigned long txBufferSize;
		volatile unsigned long txWriteIndex;
		volatile unsigned long txReadIndex;
		unsigned char *rxBuffer;
		unsigned long rxBufferSize;
		volatile unsigned long rxWriteIndex;
		volatile unsigned long rxReadIndex;
		unsigned long uartModule;
		unsigned long baudRate;
		void flushAll(void);
		void primeTransmit(unsigned long ulBase);

	public:
		HardwareSerial_v2(void);
		HardwareSerial_v2(unsigned long);
		void begin(unsigned long);
		void setBufferSize(unsigned long, unsigned long);
		void setModule(unsigned long);
		void setPins(unsigned long);
		void end(void);
		virtual int available(void);
		virtual int peek(void);
		virtual int read(void);
		virtual void flush(void);
		void UARTIntHandler(void);
    void UARTIntHandler_0(void);
    void turn_back_on_UART(void);
		virtual size_t write(uint8_t c);
		operator bool();
		using Print::write; // pull in write(str) and write(buf, size) from Print
    bool turn_off_TX;
};

extern HardwareSerial_v2 vSerial;
extern HardwareSerial_v2 vSerial1;
extern HardwareSerial_v2 vSerial2;
extern HardwareSerial_v2 vSerial3;
extern HardwareSerial_v2 vSerial4;
extern HardwareSerial_v2 vSerial5;
extern HardwareSerial_v2 vSerial6;
extern HardwareSerial_v2 vSerial7;

extern "C" void vUARTIntHandler(void);
extern "C" void vUARTIntHandler1(void);
extern "C" void vUARTIntHandler2(void);
extern "C" void vUARTIntHandler3(void);
extern "C" void vUARTIntHandler4(void);
extern "C" void vUARTIntHandler5(void);
extern "C" void vUARTIntHandler6(void);
extern "C" void vUARTIntHandler7(void);

extern void vserialEventRun(void) __attribute__((weak));
#endif
