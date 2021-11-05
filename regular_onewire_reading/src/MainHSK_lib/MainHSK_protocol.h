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
        eSetCFlowSetPoint=0xB2, //178
        eFlowCCloseValve = 0xB3, // 179
        eFlowCOpenValve = 0xB4, //180
        eFlowCDebug=0xB5, //181
        eFlowCRtnGasString=0xB6,  //182
        eSetAFlowSetPoint=0xB7, //183
        eFlowACloseValve = 0xB8, // 184
        eFlowAOpenValve = 0xB9, //185
        eFlowADebug=0xBA, //186
        eFlowARtnGasString=0xBB,  //187
        eSetMFlowSetPoint=0xBC, //188
        eFlowMCloseValve = 0xBD, // 189
        eFlowMOpenValve = 0xBE, //190
        eFlowMDebug=0xBF, //191
        eFlowMRtnGasString=0xC0,  //192
        eReadCmdPrios=0xC1, //193

};};

/* Autopriority period */
typedef struct autoPriorityPeriods {
  uint16_t lowPriorityPeriod; // auto low pri. period, in ms (0 disables, 100-655)
  uint16_t medPriorityPeriod; // as above, for med pri.
  uint16_t highPriorityPeriod;  // as above, for hi pri.
} autoPriorityPeriods_t;

/*****Main Hsk Structs*****/
/* Main launchpad */
// subhsk_id=0x01, command associated with this struct: eISR
struct sMainInternalTemp {
  float Temperature; // internal temperature sensor to uC
} __attribute__((packed));

/* Flowmeters */
// subhsk_id=0x01, command associated with this struct: eFlows
struct sMainFlows {
  double pressure; //double, flowmeter 1 pressure
  double temperature; //double, flowmeter 1 temperature
  double volume; //double, flowmeter 1 volume flow
  double mass; //double, flowmeter 1 mass flow
  double setpoint;
  double gasnum;
} __attribute__((packed));

/* 1Wire Temperatures */
// subhsk_id=0x01, command associated with this struct: eOneWireBridge_read
struct sMainTemps {
  float temps[10];
} __attribute__((packed));

struct sMainFlowsGasString {
  char response[100];
} __attribute__((packed));

/* Main Pressure Transducer */
// subhsk_id=0x01, command associated with this struct: ePressure
struct sMainPressure {
  float Pressure; //Serial Pressure Transducer pressure reading
  float Temperature; // Serial temperature internal to pressure transducer
} __attribute__((packed));

/*MainHSK*/
struct sMainLow {
  sMainInternalTemp main_internal_temp;
//  char lyrics[240];
} __attribute__((packed));

struct sMainMed {
  sMainFlows main_flows[3];
} __attribute__((packed));

struct sMainHi {
  sMainPressure main_pressure;
  sMainTemps main_temps;
} __attribute__((packed));
