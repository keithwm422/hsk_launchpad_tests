/*
 * DCTHSK_prototype.ino
 * 
 * Initiates serial ports & follows HSK protocol for command responses and error
 * reporting. This program can be used on other devices by changing the device
 * address (myID) and the upStream serial connection (direct line to the SFC)
 *
 * CONSTANTS:
 * --PACKETMARKER is defined in Cobs_encoding.h
 * --MAX_PACKET_LENGTH is defined in PacketSerial
 * --NUM_LOCAL_CONTROLS is defined here
 * --FIRST_LOCAL_COMMAND is defined here
 * --TEST_MODE_PERIOD is defined here
 * --BAUD rates are defined here
 *
 */

#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>
//#include <Hsk_all_data_types.h>
// from magnet hsk for LTC2983

#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
//////////////////////////////////////
#define DOWNBAUD 115200 // Baudrate to the SFC
#define UPBAUD 115200    // Baudrate to upsteam devices
#define TEST_MODE_PERIOD 100 // period in milliseconds between testmode packets being sent
#define FIRST_LOCAL_COMMAND 2 // value of hdr->cmd that is the first command local to the board
#define NUM_LOCAL_CONTROLS 8 // how many commands total are local to the board
#define MAX519 32 // testing DAC
#define MCP_4728_CATHODE 96  // 7-bit slave address for DACs
#define MCP_4728_POTENTIAL 96  // 7-bit slave address for DACs
#define SENSOR_UPDATE_PERIOD 1000// how often to check/write sensors
#define  WIRE_INTERFACES_COUNT 4   // needed for multiple instances of TwoWire
/* Declare instances of PacketSerial to set up the serial lines */



uint32_t ADCValues[1];

#define PRESSURE_UPDATE_PERIOD 2000
unsigned long pressureUpdateTime=0; // keeping time for the sensor check/write

// FOR HV (channel D(3) of MCP4728 is VPGM (for both) and channel A(0) is IPGM (for both).
uint8_t cat_LDAC=14; // pin 2 of launchpad is the LDAC pin we use here. 
uint8_t pot_LDAC=17; // pin 2 of launchpad is the LDAC pin we use here. 
unsigned long hvmonUpdateTime=0; // keeping time for the sensor check/write
#define HVMON_UPDATE_PERIOD 1000
uint16_t voltage_potential=0;
uint16_t voltage_cathode=0;
uint16_t current_potential=0;
uint16_t current_cathode=0;
int OT_POT = 19;
int EN_POT=18;
int OT_CAT=38;
int EN_CAT=15;
bool is_cathode_disabled=true;
bool is_potential_disabled=true;
int ot_cathode_read=0;
int ot_potential_read=0;
uint32_t hv_read;
uint8_t which_adc = 0;

// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
float pres_val=0;
float cat_v=0;
float conversion_factor=3.3*1406.0/(1000.0*4096.0);
#define PACKET_UPDATE_PERIOD 1000
unsigned long PACKETUpdateTime=0; // send reads as packets every period defined above
char receive[20]; // big anough to store things
int write_val=0;
int char_iter;
bool new_write=false;
int length_sent=0;
#define CATHODE_VOLTAGE_CUTOFF 3800
#define CATHODE_VOLTAGE_STEP_PERIOD 2000 // ten seconds
#define CATHODE_VOLTAGE_STEP_VALUE 10 // in DAC counts
#define POTENTIAL_VOLTAGE_CUTOFF 4090
#define POTENTIAL_VOLTAGE_STEP_PERIOD 2000 // ten seconds
#define POTENTIAL_VOLTAGE_STEP_VALUE 10 // in DAC counts
bool step_HV_C=false;
bool step_HV_P=false;
unsigned long step_HV_C_time=0;
unsigned long step_HV_P_time=0;
struct DCTHVMON{
  uint16_t PotVmon;
  uint16_t PotImon;
  uint16_t CatVmon;
  uint16_t CatImon;
};
DCTHVMON hvmon;
struct DCTPRESSURE{
  uint16_t Pressure_vessel;
};
DCTPRESSURE dct_pressure;
int PULSE_PIN=3;
/*******************************************************************************
* Main program
*******************************************************************************/
void setup()
{
  // to DEBUG this device when connecting directly to computer, use this serial port instead of Serial.1
  Serial.begin(DOWNBAUD);
  //downStream1.setStream(&Serial);
  //downStream1.setPacketHandler(&checkHdr);
  Serial.print("Hello Mike \n");
  // Serial port for downstream to Main HSK
  //Serial1.begin(DOWNBAUD);
  //downStream1.setStream(&Serial1);
  //downStream1.setPacketHandler(&checkHdr);
  // LED on launchpad
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  Serial3.begin(UPBAUD); //Pot A serial port
  Serial3.flush();
  Serial2.begin(UPBAUD); //Pot B serial port
  Serial2.flush();
  pinMode(PULSE_PIN, OUTPUT);

//
//  InitADC();
  analogReadResolution(12);
// 4 ADCs used for the HV monitoring: they are pins 26 (Potential field V Mon),27 (Potential field I Mon),28 (Cathode field V Mon),29 (Cathode field I Mon)
// These are, in order, A4,A2,A1,A0 (ADC labels from TM4C launchpad). 
// HV System has Binary pins also: OverTemp Fault, and Enable Input: 19 (OT Potential Field), 18 (EN Potential Field), 16 (OT Cathode Field), 15 (EN Cathode Field)
// like this: pinMode(15,INPUT);
  pinMode(OT_POT,INPUT);
  pinMode(EN_POT,OUTPUT);
  pinMode(OT_CAT,INPUT);
  pinMode(EN_CAT,OUTPUT);
  // by default start disabled:
  digitalWrite(EN_CAT,LOW);
  digitalWrite(EN_POT,LOW);
// The DACs on I2C on ports 1 and 3 have an LDAC pin that goes to 17 (LDAC Potential DAC) and 14 (LDAC Cathode DAC) 
// which can be used if programming the DACs address and updating output registers.
  pinMode(pot_LDAC,OUTPUT);
  pinMode(cat_LDAC,OUTPUT);
  delay(100);
  // Setup thermistors reads
  // chip selects setup here and set to all high
 digitalWrite(LED,LOW);

}

/*******************************************************************************
 * Main program
 ******************************************************************************/
void loop()
{
  // Read in the values until a carriage return (13)
  if(Serial.available()){
    do{
      receive[char_iter]=Serial.read();
      //Serial.print(receive[char_iter]);
      char_iter++;
      if(receive[char_iter-1]==13){
        length_sent=char_iter-1;
        char_iter=0;
        new_write=true;
        break;
      }
      else {
        if(char_iter>=10) {
          length_sent=char_iter;
          char_iter=0;
          // reset the characters
          for(int j=0;j<20;j++){
            receive[j]=0;
          }
          new_write=false;
          break;
        }
     } 
    } while(Serial.available());
  }

  // read in HV monitoring
  if((long) (millis() - hvmonUpdateTime) > 0){
    digitalWrite(PULSE_PIN,HIGH);
    hvmonUpdateTime+= HVMON_UPDATE_PERIOD;
    switch(which_adc){
      case 0: hvmon.PotVmon=(uint16_t) analogRead(A4); // VMON potential
      case 1: hvmon.PotImon=(uint16_t) analogRead(A2); // IMON potential
      case 2: hvmon.CatVmon=(uint16_t) analogRead(A1); // VMON cathode
      case 3: hvmon.CatImon=(uint16_t) analogRead(A0); // IMON cathode
      //print_HV_mon();
    }
    which_adc++;
    if(which_adc>=4) which_adc=0;
    // check the OT pins
    ot_potential_read=digitalRead(OT_POT);
    ot_cathode_read=digitalRead(OT_CAT);
   digitalWrite(PULSE_PIN,LOW);
  }
  // read in pressure
  if((long) (millis() - pressureUpdateTime) > 0){
    pressureUpdateTime+= PRESSURE_UPDATE_PERIOD;
    //dct_pressure.Pressure_vessel=PressureRead();
  }

  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    //switch_LED();
    
  }
  if((long) (millis() - PACKETUpdateTime) > 0){
    PACKETUpdateTime+= PACKET_UPDATE_PERIOD;
    //Serial.println("INCOMING DATA");
    //Serial.println();
    Serial.print("P: ");
    print_PHV_mon();
    Serial.print("C: ");
    print_CHV_mon();
    //print_pressure();
    //print_thermistors();
    //Serial.println();
    
  }

  // for debugging just one channel at a time
//  if((long) (millis() - thermsUpdateTime) > 0){
 //   thermsUpdateTime+= THERMS_UPDATE_PERIOD;
  //  thermistors.Therms[counter_all] = return_temperature_2((uint8_t)CHIP_SELECT_E, temp_channels[counter]);
 // }
}

void print_CHV_mon(){
    // Cathode HV
  Serial.print(voltage_cathode);
  Serial.print(",");
  Serial.print(hvmon.CatVmon);
  Serial.print(",");
  cat_v=hvmon.CatVmon*conversion_factor;
  //Serial.print("In Volts out of 4.64 it is: ");
  Serial.print(cat_v,4);
  Serial.print(",");
  Serial.print(cat_v/4.64*10000.0,4);
  Serial.print(",");
  // Cathode Current
  Serial.print(hvmon.CatImon);
  cat_v=hvmon.CatImon*conversion_factor;
  Serial.print(",");
  //Serial.print("In Volts out of 4.64 it is: ");
  Serial.print(cat_v,4);
  Serial.print(",");
  //Serial.print("SO Current line reads (mA): ");
  Serial.println(cat_v/4.64*1.5,4);
}
void print_PHV_mon(){
    // Potential HV
  Serial.print(voltage_potential);
  Serial.print(",");
  Serial.print(hvmon.PotVmon);
  Serial.print(",");
  cat_v=hvmon.PotVmon*conversion_factor;
  //Serial.print("In Volts out of 4.64 it is: ");
  Serial.print(cat_v,4);
  Serial.print(",");
  Serial.print(cat_v/4.64*10000.0,4);
  Serial.print(",");
  // Potential Current
  Serial.print(hvmon.PotImon);
  cat_v=hvmon.PotImon*conversion_factor;
  Serial.print(",");
  //Serial.print("In Volts out of 4.64 it is: ");
  Serial.print(cat_v,4);
  Serial.print(",");
  //Serial.print("SO Current line reads (mA): ");
  Serial.println(cat_v/4.64*1.5,4);
}

void switch_LED(){
  if(is_high){
    is_high=false;
    digitalWrite(LED,LOW);
  }
  else{    
    is_high=true;
    digitalWrite(LED,HIGH);
  }
}
