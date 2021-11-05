/*
 * MainHSK_protocol.cpp
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
namespace MainHSK_cmd { enum {
  // 2-248 are board-specific: these are test commands
	eIntSensorRead = 0x02,
	ePacketCount = 0x03,
	eMapDevices = 0x04,
	eHeaterControl = 0x05,
	eTestHeaterControl = 0x06,
        eAutoPriorityPeriod = 0x07,
        eFlows = 0x08,
        ePressure = 0x09,
        eISR = 0xA0, //160
        eOneWireBridge_conn=0xA9,
        eOneWireBridge_read=0xAA, // 170
        eOneWireBridge_report_addresses=0xAB,
        eSSOut_conn=0xAC,
        eSSOut=0xAD,
        eSSOutGround=0xAE, // 174
        eSSOutHigh=0xAF,
        eSSOutChannel=0xB0,
        eSSOutTempTest=0xB1, //177
        eSetFlowSetPoint=0xB2, //178
        eScottLyrics = 0xB3, //179
        eFlow_1_close_valve = 0xB4, // 180
        eFlow_1_open_valve = 0xB5, //181
        eFlowDebug=0xB6, //182
        eFlowRtnGasString=0xB7,  //183
        eReadCmdPrios=0xB8, //184
        eRelaysIN = 0xB9,  //185
        eBigPacket=0xBA, // 186

};};

/* Autopriority period */
typedef struct autoPriorityPeriods {
  uint16_t lowPriorityPeriod; // auto low pri. period, in ms (0 disables, 100-655)
  uint16_t medPriorityPeriod; // as above, for med pri.
  uint16_t highPriorityPeriod;  // as above, for hi pri.
} autoPriorityPeriods_t;



/* not needed anymore
typedef enum {
  PRIO_STATE_IDLE = 0, // means it is just the same as eeprom and nothing needs to be done, regardless if locked, unlocked. 
  PRIO_STATE_UPDATE_DEFAULT = 1, // means series of packets was sent to unlock eeprom and write the new prios (just lock eeprom afterwards).  sends to updated
  PRIO_STATE_NOT_DEFAULT = 2, // prios have been changed since the eeprom was loaded, doesn't change out until commands are sent. 
  PRIO_STATE_UPDATED = 3 // write the new version, since it has been changed, occurs after update_default, sends back to idle
} PrioState;
*/
