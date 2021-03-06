/*
 * DCTHSK_protocol.cpp
 *
 * Declares the interface protocol for cross device communication.
 *
 * Housekeeping packet consists of header
 * 0-255 payload bytes
 * 1 byte CRCS (or checksum)
 */

/*****************************************************************************
 * Defines
 ****************************************************************************/
#pragma once
#include <stdint.h>

#define MIN_PERIOD      100

/*******************************************************************************
 * Typedef enums
 *******************************************************************************/

/* Command definitions */
typedef enum DCTHSK_cmd {
  // 2-248 are board-specific: these are test commands
	eIntSensorRead = 0x02,
	eHeaterControlAll = 0x03,
	eHeaterControlChannel = 0x04,
        eThermistors = 0x05,
        eHVmon = 0x06,  // to send most recent voltages and currents from both supplies
        ePacketCount = 0x07,
        eThermistorsTest = 0x08,
        ePressure = 0x09,
        eVPGMPotential=0x0A, //10
        eIPGMPotential=0x0B, //11
        eVPGMCathode=0x0C, //12
        eIPGMCathode=0x0D, //13
        eVMONPotential=0x0E, //14
        eIMONPotential=0x0F, //15
        eVMONCathode=0x10, //16
        eIMONCathode=0x11, //17

        eISR=0xA0,
} DCTHSK_cmd;
