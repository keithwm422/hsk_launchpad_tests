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
typedef enum MainHSK_cmd {
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
        eBadCount=0xBA, //186
        eErrCount=0xBB, //187
} MainHSK_cmd;

/* Autopriority period */
typedef struct autoPriorityPeriods {
  uint16_t lowPriorityPeriod; // auto low pri. period, in ms (0 disables, 100-655)
  uint16_t medPriorityPeriod; // as above, for med pri.
  uint16_t highPriorityPeriod;  // as above, for hi pri.
} autoPriorityPeriods_t;


/* SSOut volts */
typedef struct SSOut_vals {
  float SFCTemp1;  // was a float, now needs to be a DAC output where each votlage step (5-0)/4096 <-> (125-(-55))/4096.
  float SFCTemp2;
  float DCTTemp1;
  float DCTTemp2;
  float DCTTemp3;
  float DCTTemp4;
  float DCTTemp5;
  float DCTTemp6;
  float DCTTemp7;
  float DCTTemp8;
  float DCTTemp9;
  float DCTTemp10;
  float BatteryTemp1;
  float BatteryTemp2;
  uint16_t Power1;
  uint16_t Power2;
  uint16_t Power3;
  uint16_t Power4;
} SSOut_vals_t;
/* not needed anymore
typedef enum {
  PRIO_STATE_IDLE = 0, // means it is just the same as eeprom and nothing needs to be done, regardless if locked, unlocked. 
  PRIO_STATE_UPDATE_DEFAULT = 1, // means series of packets was sent to unlock eeprom and write the new prios (just lock eeprom afterwards).  sends to updated
  PRIO_STATE_NOT_DEFAULT = 2, // prios have been changed since the eeprom was loaded, doesn't change out until commands are sent. 
  PRIO_STATE_UPDATED = 3 // write the new version, since it has been changed, occurs after update_default, sends back to idle
} PrioState;
*/
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
	eDCTIntSensorRead = 0x02,
	eHeaterControlAll = 0x03,
	eHeaterControlChannel = 0x04,
        eThermistors = 0x05,
        eHVmon = 0x06,  // to send most recent voltages and currents from both supplies
        ePacketCount = 0x07,
        eThermistorsTest = 0x08,
        ePressure = 0x09,
        eVPGMPotential = 0x0A, //10
        eIPGMPotential = 0x0B, //11
        eVPGMCathode = 0x0C, //12
        eIPGMCathode = 0x0D, //13
        eVMONPotential = 0x0E, //14
        eIMONPotential = 0x0F, //15
        eVMONCathode = 0x10, //16
        eIMONCathode = 0x11, //17
        eHVMonConverted = 0x12, //18
        eHVMonOTEN = 0x13, //19
        eHVCatEN = 0x14, //20
        eHVPotEN = 0x15, //21
        eRampHVCat = 0x16, //22
        eRampHVPot = 0x17, //23
        eCancelRamping = 0x18, //24
        ePauseRamping = 0x19, //25
        eResumeRamping = 0x1A, //26
        eReadHeaterResponse = 0x1B, //27
        eHeaterStartupValue = 0x1C, //28
        eSetVREF = 0x1D, //29
        eDCTISR=0xA0,
} DCTHSK_cmd;

/*****DCT Hsk Structs*****/
/* DCT launchpad */
// subhsk_id=0x01, command associated with this struct: eISR
struct sDCTInternalTemp {
  float Temperature; // internal temperature sensor to uC
} __attribute__((packed));

/* DCT Thermistors */
// subhsk_id=0x03, command associated with this struct: eThermistors
struct sDCTThermistors {
  float Therms[25]; // float, Thermistors temperature
} __attribute__((packed));

/* HV supplies monitoring*/
// subhsk_id=0x03, command associated with this struct: eHVMon
struct sDCTHV {
  uint16_t CatVmon; // 12 bits ADC, Cathode HV supply Vmon
  uint16_t CatImon; // 12 bits ADC, Cathode HV supply Imon
  uint16_t PotVmon; // 12 bits ADC, Potential HV supply Vmon
  uint16_t PotImon; // 12 bits ADC, Potential HV supply Imon
} __attribute__((packed));

/* DCT Pressure Transducer */
// subhsk_id=0x03, command associated with this struct: ePressure
struct sDCTPressure {
  uint16_t Pressure_vessel; // 10 bits ADC, Pressure Transducer vessel reading from IC
  uint16_t Pressure_vacuumref; // 12 bits ADC, Pressure Transducer vacuum ref. reading on launchpad
} __attribute__((packed));

/* HV supplies monitoring*/
// subhsk_id=0x03, command associated with this struct: eHVMonConverted
struct sDCTHVConverted {
  float CatV; // 12 bits ADC converted to kV-Cathode HV supply
  float CatI; // 12 bits ADC, converted to mA-Cathode HV supply
  float PotV; // 12 bits ADC, converted to kV-Potential HV supply
  float PotI; // 12 bits ADC, converted to mA-Potential HV supply
} __attribute__((packed));

/*DCTHSK priority*/

struct sDCTLow {
  sDCTInternalTemp DCT_internal_temp;
} __attribute__((packed));

struct sDCTMed {
  sDCTHV DCT_hv;
} __attribute__((packed));

struct sDCTHi {
  sDCTThermistors DCT_thermistors;
  sDCTPressure DCT_pressure;
} __attribute__((packed));
