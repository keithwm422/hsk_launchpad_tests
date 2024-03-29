// if SPI_user.h doesnt work lets try the older tiva-c release of 1.0.3 edited SPI class
//#include "SPI_1_0_4_version/SPI_user.h"
#include "SPI_1_0_3_version/SPI_user_1_0_3.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
#include "configuration_constants.h"
#include "DCT_SPI_support_functions.h"
#include "config_functions.h"
///////////////


// *****************
// Configuration functions copied directly from the evalprom c-generated code
// *****************

void configure_channels(uint8_t cs)
{
  uint8_t channel_number;
  uint32_t channel_assignment_data;

  // ----- Channel 2: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(cs, 2, channel_assignment_data);
  // ----- Channel 4: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__2 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(cs, 4, channel_assignment_data);
  // ----- Channel 6: Assign Sense Resistor -----
  if(cs==31){ // for chip select C which will have an RTD on channel 8 using channel 6 with a 100Ohm resistor.
    channel_assignment_data = 
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x19000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 100.
  }
  else{
    channel_assignment_data = 
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  }
  assign_channel(cs, 6, channel_assignment_data);  // do this regardless of the chip select, cs.
  // ----- Channel 8: Assign Thermistor 44006 10K@25C -----
  if(cs==31){  // for chip select C need RTD not therms
    // ----- Channel 8: Assign RTD PT-100 -----
    channel_assignment_data = 
      SENSOR_TYPE__RTD_PT_100 |
      RTD_RSENSE_CHANNEL__6 |
      RTD_NUM_WIRES__3_WIRE |
      RTD_EXCITATION_MODE__NO_ROTATION_NO_SHARING |
      RTD_EXCITATION_CURRENT__500UA |
      RTD_STANDARD__AMERICAN;
  }
  else{
    channel_assignment_data = 
      SENSOR_TYPE__THERMISTOR_44006_10K_25C |
      THERMISTOR_RSENSE_CHANNEL__6 |
      THERMISTOR_DIFFERENTIAL |
      THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
      THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  }
  assign_channel(cs, 8, channel_assignment_data); // do this regardless of the chip select, cs.
  // ----- Channel 10: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(cs, 10, channel_assignment_data);
  // ----- Channel 12: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__10 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(cs, 12, channel_assignment_data);
  // ----- Channel 14: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(cs, 14, channel_assignment_data);
  // ----- Channel 16: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__14 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(cs, 16, channel_assignment_data);
  // ----- Channel 18: Assign Sense Resistor -----
  if(cs==12){  // this is chip E RTD
    channel_assignment_data = 
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x19000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 100.
  }
  else{
    channel_assignment_data = 
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  }
  assign_channel(cs, 18, channel_assignment_data);
  // ----- Channel 20: Assign Thermistor 44006 10K@25C -----
  if(cs==12){
    // ----- Channel 20: Assign RTD PT-100 -----
    channel_assignment_data = 
      SENSOR_TYPE__RTD_PT_100 |
      RTD_RSENSE_CHANNEL__18 |
      RTD_NUM_WIRES__3_WIRE |
      RTD_EXCITATION_MODE__NO_ROTATION_NO_SHARING |
      RTD_EXCITATION_CURRENT__500UA |
      RTD_STANDARD__AMERICAN;
  }
  else{
    channel_assignment_data = 
      SENSOR_TYPE__THERMISTOR_44006_10K_25C |
      THERMISTOR_RSENSE_CHANNEL__18 |
      THERMISTOR_DIFFERENTIAL |
      THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
      THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  }
  assign_channel(cs, 20, channel_assignment_data);

}

void configure_global_parameters(uint8_t cs) 
{
  // -- Set global parameters
  transfer_byte(cs, WRITE_TO_RAM, 0xF0, TEMP_UNIT__C |
    REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(cs, WRITE_TO_RAM, 0xFF, 0);
}

//  measure_channel(CHIP_SELECT, 4, TEMPERATURE);      // Ch 4: Thermistor 44006 10K@25C
//  measure_channel(CHIP_SELECT, 8, TEMPERATURE);      // Ch 8: Thermistor 44006 10K@25C
//  measure_channel(CHIP_SELECT, 12, TEMPERATURE);     // Ch 12: Thermistor 44006 10K@25C
//  measure_channel(CHIP_SELECT, 16, TEMPERATURE);     // Ch 16: Thermistor 44006 10K@25C
//  measure_channel(CHIP_SELECT, 20, TEMPERATURE);     // Ch 20: Thermistor 44006 10K@25C
