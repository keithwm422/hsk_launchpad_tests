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
/* These are device specific */
#include "src/DCTHSK_lib/DCTHSK_protocol.h"
using namespace DCTHSK_cmd;
// from magnet hsk for LTC2983

#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
#include "src/DCTHSK_lib/configuration_constants.h"
#include "src/DCTHSK_lib/DCT_SPI_support_functions.h"
#include "src/DCTHSK_lib/Wire.h"
#include "src/DCTHSK_lib/DCTHSK_support_functions.h"
#include "src/DCTHSK_lib/config_functions.h"

// these files above need to be changed based on the thermistor or rtd, etc. So if we borrow the examples from magnet hsk then we can change the channels and types in just these files but keep the function the same. 
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
PacketSerial downStream1; // mainHSK

/* Name of this device */
housekeeping_id myID = eDCTHsk;

/* Outgoing buffer, for up or downstream. Only gets used once a complete packet
 * is received -- a command or forward is executed before anything else happens,
 * so there shouldn't be any over-writing here. */
uint8_t outgoingPacket [MAX_PACKET_LENGTH] ={0}; 

/* Use pointers for all device's housekeeping headers and the autopriorityperiods*/
housekeeping_hdr_t * hdr_in;     housekeeping_hdr_t * hdr_out;
housekeeping_err_t * hdr_err;   housekeeping_prio_t * hdr_prio;
/* Memory buffers for housekeeping system functions */
uint8_t numDevices = 0;           // Keep track of how many devices are upstream
uint8_t commandPriority[NUM_LOCAL_CONTROLS] = {1,0,0,3,2,0,0,3};     // Each command's priority takes up one byte
PacketSerial *serialDevices = &downStream1;
uint8_t addressList = 0; // List of all downstream devices

/* Utility variables for internal use */
size_t hdr_size = sizeof(housekeeping_hdr_t)/sizeof(hdr_out->src); // size of the header
uint8_t numSends = 0; // Used to keep track of number of priority commands executed
uint16_t currentPacketCount=0;
static_assert(sizeof(float) == 4);
unsigned long timelastpacket; //for TestMode
/*Structs for storing most current reads*/
sDCTThermistors thermistors;

uint32_t ADCValues[1];

/* SENSOR STATES AND VARS */
// I2C port map
const int port_HV=1;
const int port_pressure=2;
//TwoWire * wire_pressure= new TwoWire(port_pressure);
TwoWire * wire_pressure=&Wire2;
//TwoWire * wire_HV= new TwoWire(port_HV);
TwoWire * wire_HV = &Wire1;
// 10 bit ADC read from i2c.
sDCTPressure dct_pressure;
#define PRESSURE_UPDATE_PERIOD 2000
unsigned long pressureUpdateTime=0; // keeping time for the sensor check/write
// for new Heaters
TwoWire * wire_Heater = &Wire3;

// FOR HV (channel D(3) of MCP4728 is VPGM (for both) and channel A(0) is IPGM (for both).
//uint8_t cat_LDAC=14; // pin 2 of launchpad is the LDAC pin we use here. 
//uint8_t pot_LDAC=17; // pin 2 of launchpad is the LDAC pin we use here. 
unsigned long hvmonUpdateTime=0; // keeping time for the sensor check/write
#define HVMON_UPDATE_PERIOD 10000
uint16_t voltage_potential=0;
uint16_t voltage_cathode=0;
uint16_t current_potential=0;
uint16_t current_cathode=0;
int EN_HV=18;
bool is_HV_disabled=true;
uint32_t hv_read;
uint8_t which_adc = 0;
sDCTHV hvmon;
uint8_t address_hvdac=0x10;
// For thermistors
#define NUM_THERMS 25
//float thermistor_temps[NUM_THERMS]={0};
// there are five thermistors per chip_select, so need to cycle five times on the chip then go on to the next chip (or mod 5 it?)
#define THERMS_UPDATE_PERIOD 1000
unsigned long thermsUpdateTime=0; // keeping time for the sensor check/write
int counter=0; // counter 0,4 are CSA, 5,9 are CSB, 10,14 C, 15,19 D 20,24 E. 
int chip_to_read=0;
uint8_t temp_channels[5]={4,8,12,16,20};
int counter_all=0;
// So switch chip select when counter+1 %5==0?? (and then if counter==24 set counter back to 0).
#define THERMPACKET_UPDATE_PERIOD 10000
unsigned long ThermPacketUpdateTime=0; // keeping time for the sensor check/write

// for heater sheets
uint8_t change_all[3]={254,171,0}; // default to zero
uint8_t change_one[4]={254,170,0,0};

// for Launchpad LED
#define LED GREEN_LED
//int LED=15;
#define LED_UPDATE_PERIOD 5050
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
float pres_val=0;
float cat_v=0;
float conversion_factor=3.3*1406.0/(1000.0*4096.0);
double delta_V_DAC=4.096/4096.0;
#define PACKET_UPDATE_PERIOD 2000
unsigned long PACKETUpdateTime=0; // send reads as packets every period defined above
char receive[20]; // big anough to store things
int write_val=0;
int char_iter;
bool new_write=false;
int length_sent=0;
#define CATHODE_VOLTAGE_CUTOFF 4095
#define CATHODE_VOLTAGE_STEP_PERIOD 1000 // one second
#define CATHODE_VOLTAGE_STEP_VALUE 17 // in DAC counts
#define POTENTIAL_VOLTAGE_CUTOFF 4095
#define POTENTIAL_VOLTAGE_STEP_PERIOD 1000 // one second
#define POTENTIAL_VOLTAGE_STEP_VALUE 17 // in DAC counts
bool step_HV_C=false;
bool step_HV_P=false;
unsigned long step_HV_C_time=0;
unsigned long step_HV_P_time=0;
int clear_buffers_with_this=0;

double cathode_scale_factor=1.13;
double potential_voltage_scale_factor=1.0/2.21;
double potential_current_scale_factor=1.0/13.0;
double max_pin=4.64;
double highvolt_FS=10000.0;  // in volts
double highcurrent_FS=1500.0; // in microamps
double cathode_volt_FS=9975.0; // in volts
double cathode_current_FS=1496.0; // in microamps
double potential_volt_FS=3994.0; // in volts
double potential_current_FS=102.0; // in microamps

unsigned int vref = 5000;             // Voltage reference value for calculations (set to 5000 for internal reference)
unsigned int Heater_read_value = 0;          // Value read from DAC
float heater_voltage = 0;                    // Read voltage

uint16_t _value=2200;
int status_heaters=0;
int voltPin = 33;   
int involtPin=32;
int isErr=0;
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
  clear_buffers_with_this=0;
  while(clear_buffers_with_this!=-1){
    clear_buffers_with_this=Serial3.read();
  }
  //Serial2.begin(UPBAUD); //Pot B serial port
  //Serial2.flush();

  // Point to data in a way that it can be read as a header
  hdr_out = (housekeeping_hdr_t *) outgoingPacket;
  hdr_err = (housekeeping_err_t *) (outgoingPacket + hdr_size);
  currentPacketCount=0;
  // functions begin SPI connections, should only need one SPI thingy
//
//  InitADC();
  analogReadResolution(12);
// 4 ADCs used for the HV monitoring: they are pins 26 (Potential field V Mon),27 (Potential field I Mon),28 (Cathode field V Mon),29 (Cathode field I Mon)
// These are, in order, A4,A2,A1,A0 (ADC labels from TM4C launchpad). 
// HV System has Binary pins also: OverTemp Fault, and Enable Input: 19 (OT Potential Field), 18 (EN Potential Field), 16 (OT Cathode Field), 15 (EN Cathode Field)
// like this: pinMode(15,INPUT);
  pinMode(EN_HV,OUTPUT);
  // by default start disabled:
  digitalWrite(EN_HV,LOW);
// The DACs on I2C on ports 1 and 3 have an LDAC pin that goes to 17 (LDAC Potential DAC) and 14 (LDAC Cathode DAC) 
// which can be used if programming the DACs address and updating output registers.
  wire_pressure->begin();
  Serial.print("wire pressure begin \n");
  pinMode(voltPin, OUTPUT);
  digitalWrite(voltPin, HIGH); // turn on the heater driver PCB
  delay(1); // give it a msec to turn on before trying I2C connection
  pinMode(involtPin,INPUT);
  isErr=digitalRead(involtPin);
  if(isErr==0){
    Serial.print("Error circuit low");
  }
  wire_Heater->begin();
  Serial.print("wire heater begin \n");
  HeaterSetup(*wire_Heater);
  for(int i=0; i <8; i++){
    HeaterExecute(i,0);
  }
  delay(100);
  isErr=digitalRead(involtPin);
  if(isErr==0){
    Serial.print("Error circuit low");
    delay(100000);
  }
  Serial.print("heater setup \n");
  //wire_HV->begin();
  // run setup of I2C stuff from support_functions
  //HVSetup(*wire_HV,address_hvdac);
  PressureSetup(*wire_pressure);
  Serial.print("pressure setup \n");
  delay(100);
  // Setup thermistors reads
  // chip selects setup here and set to all high
  pinMode(CHIP_SELECT_A, OUTPUT);
  pinMode(CHIP_SELECT_B, OUTPUT);
  pinMode(CHIP_SELECT_C, OUTPUT);
  pinMode(CHIP_SELECT_D, OUTPUT);
  pinMode(CHIP_SELECT_E, OUTPUT);
  // set all CS to high
  set_CS_all_high();
  delay (1000);
  Initialize_TM4C123();
  Serial.print("Initialized tm4c123 \n");
  //delay(200);
  // Setup Thermistors after Initiliazing the SPI and chip selects
  configure_channels((uint8_t)CHIP_SELECT_A);
  configure_global_parameters((uint8_t)CHIP_SELECT_A);
  configure_channels((uint8_t)CHIP_SELECT_B);
  configure_global_parameters((uint8_t)CHIP_SELECT_B);
  configure_channels((uint8_t)CHIP_SELECT_C);
  configure_global_parameters((uint8_t)CHIP_SELECT_C);
  configure_channels((uint8_t)CHIP_SELECT_D);
  configure_global_parameters((uint8_t)CHIP_SELECT_D);
  configure_channels((uint8_t)CHIP_SELECT_E);
  configure_global_parameters((uint8_t)CHIP_SELECT_E);
  ThermPacketUpdateTime=millis()+THERMPACKET_UPDATE_PERIOD;
  Serial.print("configured temps\n");

  digitalWrite(LED,LOW);
  // set the reading and controlling off by 500ms
  hvmonUpdateTime= millis() + HVMON_UPDATE_PERIOD;
  PACKETUpdateTime+= PACKET_UPDATE_PERIOD/2;

 //raise_PHV();
  //raise_CHV();
}

/*******************************************************************************
 * Main program
 ******************************************************************************/
void loop()
{
  isErr=digitalRead(involtPin);
  if(isErr==0){
    Serial.print("Error circuit low");
    Serial.println(_value,DEC);
  }
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
  // once we have a value, write it out to the DAC
  if(new_write){
    // if "C" for Cathode HV
    if(receive[0]=='C') write_HV_d(1);
    //if(receive[0]=='C') write_HV(1);
    // if "P" for Potential HV 
    else if(receive[0]=='E') ENABLE_HV(); // for CATHODE ENABLE PIN
    else if(receive[0]=='D') DIS_HV(); // for DISABLING CATHODE
    /*
    else if(receive[0]=='P') write_HV(2); // for VPGM of Potential
    else if(receive[0]=='I') write_HV(3); // for Ilim of potnetial
    else if(receive[0]=='J') write_HV(4); // for Ilim of Potential
    */
    else if(receive[0]=='P') write_HV_d(2); // for VPGM of Potential
    else if(receive[0]=='I') write_HV_d(3); // for Ilim of potnetial
    else if(receive[0]=='J') write_HV_d(4); // for Ilim of Potential
    else if(receive[0]=='R') raise_CHV(); // for ramping Cathode
    else if(receive[0]=='Q') raise_PHV(); // for ramping Potential
    else if(receive[0]=='S') pause_all_HV(); // for sleeping/pausing ramping procedures
    else if(receive[0]=='W') continue_all_HV(); // for waking/resuming ramping procedures
    else if(receive[0]=='X') cancel_all_HV(); // for Ilim of Potential
    // if A for writing to potentiometer
    else if(receive[0]=='A') write_Potentiometer(1);
    else if(receive[0]=='B') write_Potentiometer_channel();
    else {
      Serial.println("UNKNOWN COMMAND");
      new_write=false;
    }
    new_write=false;
    for(int j=0;j<20;j++){
      receive[j]=0;
    }
    
  }
  if(Serial3.available()){
        uint8_t returns=Serial3.read();
        Serial.print(returns,DEC);
        Serial.print(",");
  }
  //if(Serial2.available()){
  //      uint8_t returns_2=Serial2.read();
  //      Serial.print(returns_2,DEC);
  //      Serial.print(",");
  //}
  // secondary code is simply blocking version with wire.
  // read in thermistors
  // do a read based on timer and for the counter and chip_to_read
  // channel number is the pin number from the ltc2983 reading method. 4,8,12,16,20.
  // if counter is 0,5,10,15,20 then read 4, if 1,6,11,16,21 then 8, etc.

   if((long) (millis() - thermsUpdateTime) > 0){
    thermsUpdateTime+= THERMS_UPDATE_PERIOD;
    switch(chip_to_read){
      case 0:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_A, temp_channels[counter],TEMPERATURE);
        break;
      case 1:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_B, temp_channels[counter],TEMPERATURE);
        break;
      case 2:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_C, temp_channels[counter],TEMPERATURE);
        break;
      case 3:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_D, temp_channels[counter],TEMPERATURE);
        break;
      case 4:
        thermistors.Therms[counter_all] = measure_channel((uint8_t)CHIP_SELECT_E, temp_channels[counter],TEMPERATURE);
        break;
    }
    counter++;
    counter_all++;
    if(counter>=5){
      chip_to_read++;
      counter=0;
    }
    if(chip_to_read>=5) chip_to_read=0;
    if(counter_all>=25) counter_all=0;    
    //thermistors.Therms[0] = measure_channel((uint8_t)CHIP_SELECT_E, temp_channels[4],TEMPERATURE);
  }
  // read in HV monitoring
  if((long) (millis() - hvmonUpdateTime) > 0){
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
  }
  // read in pressure
  if((long) (millis() - pressureUpdateTime) > 0){
    pressureUpdateTime+= PRESSURE_UPDATE_PERIOD;
    dct_pressure.Pressure_vessel=PressureRead();
  }
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
    /*if(status_heaters==0){
      _value=2048;
      status_heaters+=1;
    }
    else{
      _value=2866;
      status_heaters=0;
    }*/
    _value+=1;
    if(_value>=4096) _value=0;
    HeaterExecute(6,_value);
  }
  if((long) (millis() - PACKETUpdateTime) > 0){
    PACKETUpdateTime+= PACKET_UPDATE_PERIOD;
    //Serial.println("INCOMING DATA");
    //Serial.println();
    /*Serial.print("P: ");
    print_PHV_mon();
    Serial.print(" C: ");
    print_CHV_mon();
    //print_pressure();
    //Serial.println();
    */
  }
  if((long) (millis() - ThermPacketUpdateTime) > 0){
    ThermPacketUpdateTime+= THERMPACKET_UPDATE_PERIOD;
    //print_thermistors();
  }
  if(step_HV_C){
    if((int) voltage_cathode >= CATHODE_VOLTAGE_CUTOFF){
      step_HV_C=false;
      Serial.println("Ramping of Cathode wires complete, cutoff reached");
      // should just stay at this value and then can be overwritten by the Cxxx command
     // do i need some more stuff?
    }
    else{
      if((long) (millis() - step_HV_C_time) > 0){
        step_HV_C_time+= CATHODE_VOLTAGE_STEP_PERIOD;
        voltage_cathode+=CATHODE_VOLTAGE_STEP_VALUE;
        if((int) voltage_cathode <= CATHODE_VOLTAGE_CUTOFF) CATChannelProgram(voltage_cathode, 0);
        else{
          step_HV_C=false;
          Serial.println("Ramping of Cathode wires complete, cutoff reached");
        }
      }
    }
  }
  if(step_HV_P){
    if((int) voltage_potential >= POTENTIAL_VOLTAGE_CUTOFF){
      step_HV_P=false;
      Serial.println("Ramping of Potential wires complete, cutoff reached");
      // should just stay at this value and then can be overwritten by the Cxxx command
     // do i need some more stuff?
    }
    else{
      if((long) (millis() - step_HV_P_time) > 0){
        step_HV_P_time+= POTENTIAL_VOLTAGE_STEP_PERIOD;
        voltage_potential+=POTENTIAL_VOLTAGE_STEP_VALUE;
        if((int) voltage_potential <= POTENTIAL_VOLTAGE_CUTOFF) POTChannelProgram(voltage_potential, 2);
        else {
          step_HV_P=false;
          Serial.println("Ramping of Potential wires complete, cutoff reached");
        }
      }
    }
  }
  // for debugging just one channel at a time
//  if((long) (millis() - thermsUpdateTime) > 0){
 //   thermsUpdateTime+= THERMS_UPDATE_PERIOD;
  //  thermistors.Therms[counter_all] = return_temperature_2((uint8_t)CHIP_SELECT_E, temp_channels[counter]);
 // }
  /* Continuously read in one byte at a time until a packet is received */
  //if (downStream1.update() != 0) badPacketReceived(&downStream1);
}

/*
void print_HV_mon(){
  // Cathode HV
  if(is_cathode_disabled) Serial.println("Cathode HV is disabled");
  else Serial.println("Cathode HV is enabled");
  Serial.print("The Cathode HV is: ");
  Serial.print(hvmon.CatVmon);
  cat_v=hvmon.CatVmon*conversion_factor;
  // Serial.print("In Volts out of 4.64 it is: ");
  Serial.print(" , ");
  Serial.print(cat_v,4);
  //Serial.print("SO HV line is at (Volts): ");
  Serial.print(" , ");
  Serial.println(cat_v/4.64*10000.0,4);
  // Cathode Current
  Serial.print("The Cathode Current is: ");
  Serial.print(hvmon.CatImon);
  cat_v=hvmon.CatImon*conversion_factor;
  //Serial.print("In Volts out of 4.64 it is: ");
  Serial.print(" , ");
  Serial.print(cat_v,4);
  //Serial.print("SO Current line reads (mA): ");
  Serial.print(" , ");
  Serial.println(cat_v/4.64*1.5,4);
    // Potential HV
  if(is_potential_disabled) Serial.println("Potential HV is disabled");
  else Serial.println("Potential HV is enabled");
  Serial.print("The Potential HV is: ");
  Serial.print(hvmon.PotVmon);
  Serial.print(" , ");
  cat_v=hvmon.PotVmon*conversion_factor;
  //Serial.print("In Volts out of 4.64 it is: ");
  Serial.print(cat_v,4);
  Serial.print(" , ");
  //Serial.print("SO HV line is at (Volts): ");
  Serial.println(cat_v/4.64*10000.0,4);
  // Potential Current
  Serial.print("The Potential Current is: ");
  Serial.print(hvmon.PotImon);
  cat_v=hvmon.PotImon*conversion_factor;
  Serial.print(" , ");
  //Serial.print("In Volts out of 4.64 it is: ");
  Serial.print(cat_v,4);
  Serial.print(" , ");
  //Serial.print("SO Current line reads (mA): ");
  Serial.println(cat_v/4.64*1.5,4);
  Serial.println("The OverTemp Pins read (C,P): ");
  Serial.print(ot_cathode_read);
  Serial.print(" , ");
  Serial.println(ot_potential_read);
}
*/
void print_CHV_mon(){
    // Cathode HV
  Serial.print(voltage_cathode);
  Serial.print(",");
  Serial.print(current_cathode);
  Serial.print(",");  
  Serial.print(hvmon.CatVmon);
  Serial.print(",");
  Serial.print(hvmon.CatImon);
  Serial.print("\n");
  //Serial.print(",");
/*  cat_v=hvmon.CatVmon*conversion_factor;
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
  */
}
void print_PHV_mon(){
    // Potential HV
  Serial.print(voltage_potential);
  Serial.print(",");
  Serial.print(current_potential);
  Serial.print(",");
  Serial.print(hvmon.PotVmon);
  Serial.print(",");
  // Potential Current
  Serial.print(hvmon.PotImon);

  //Serial.print(",");
  /*cat_v=hvmon.PotVmon*conversion_factor;
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
  */
}

void print_pressure(){
  Serial.print("The Pressure Transducer read: ");
  Serial.println(dct_pressure.Pressure_vessel);
  pres_val=(dct_pressure.Pressure_vessel*30.0/(1024.0));
  Serial.print("In psi: ");
  Serial.println(pres_val,4);
}

void print_thermistors(){
  Serial.print("The Thermistor reads are: ");
  for(int i=0;i<25;i++){
    Serial.print(i);
    Serial.print(":");
    Serial.print(thermistors.Therms[i],4);
    Serial.print(" , ");
  }
  Serial.println("...done");
}
void ENABLE_HV(){
  Serial.println("HV enabled");
  digitalWrite(EN_HV,HIGH);
  is_HV_disabled=false;
}
void DIS_HV(){
  Serial.println("HV disabled");
  digitalWrite(EN_HV,LOW);
  is_HV_disabled=true;
}

// as integers or percentages
void write_the_value_to_HV_C(){
  int number = ceil(write_val*4096.0/100.0);
  //int number = write_val;
  if(number <=4095 && number>=0){
    memcpy((uint8_t * ) &voltage_cathode, (uint8_t *)&number, sizeof(voltage_cathode));
    CATChannelProgram(voltage_cathode, 0);
    //write_some_valC(voltage_cathode);
/*
    uint8_t first= (uint8_t)(voltage_cathode >> 8);  // higher bits?
    uint8_t second= (uint8_t) (voltage_cathode & 0x00FF); // lower bits?
    wire_HV->beginTransmission(0x10);
    wire_HV->write(0x30); // 0x30 is ch 0 0x31 is ch1
    wire_HV->write(100);
    wire_HV->write(100);
    wire_HV->endTransmission();*/
    Serial.println("Cathode Wire Voltage changed");
  }
}
void write_the_value_to_Cat_Ilim(){
  int number = ceil(write_val*4096.0/100.0);
  //int number = write_val;
  if(number <=4095 && number>=0){
    memcpy((uint8_t * ) &current_cathode, (uint8_t *)&number, sizeof(current_cathode));
    //CATChannelProgram(current_cathode, (uint8_t)(1));
    //uint8_t first = 10;
    //uint8_t second = 250;
    uint8_t first= (uint8_t)((current_cathode >> 4) & 0xFF);  // higher bits?
    uint8_t second= (uint8_t) ((current_cathode & 0x0F) << 4); // lower bits?
    wire_HV->beginTransmission(0x10);
    wire_HV->write(0x31); // 0x30 is ch 0 0x31 is ch1
    wire_HV->write(first);
    wire_HV->write(second);
    wire_HV->endTransmission();
    Serial.println("Cathode Wire Ilim changed");
  }
  else Serial.println("no cigar");
}

void write_the_value_to_HV_P(){
  int number = ceil(write_val*4096.0/100.0);
  //int number = write_val;
  if(number <=4095 && number>=0){
    memcpy((uint8_t * ) &voltage_potential, (uint8_t *)&number, sizeof(voltage_potential));
    //write_some_valP(voltage_potential);
    //POTChannelProgram(voltage_potential, 2);
    Serial.println("Potential Wire Voltage changed");
  }
}
void write_the_value_to_Pot_Ilim(){
  int number = write_val;
  if(number <=4095 && number>=0){
    memcpy((uint8_t * ) &current_potential, (uint8_t *)&number, sizeof(current_potential));
    //write_some_valPI(current_potential);
    //POTChannelProgram(current_potential, 3);
    Serial.println("Potential Wire Ilim changed");
  }
}

void write_HV(int which_HV){
  if(length_sent<=5 && length_sent>=2){
      Serial.print("you sent: ");
      Serial.println(receive);
      //Serial.println("AHHH");
      char temp_chars[10];
      // as an int
      memcpy((uint8_t *) temp_chars,(uint8_t *) &receive +1, length_sent--);
      write_val=atoi(temp_chars);
      // can also do atof()
      Serial.print("converted to integer: ");
      Serial.println(write_val);
      new_write=false;
      if(which_HV==1) write_the_value_to_HV_C();
      else if(which_HV==2) write_the_value_to_HV_P();
      else if(which_HV==3) write_the_value_to_Cat_Ilim();
      else if(which_HV==4) write_the_value_to_Pot_Ilim();
      else Serial.println("choose 1 for Cat HV, 2 for Pot HV, 3 for Cat Ilim, 4 for Pot Ilim");
  }
}
// required conversions
double convert_daccode_to_vout(uint16_t input){
  return (input*4.096/4096.0);
}

double convert_vout_to_vprog(double input,double fs){
  return (input*fs);
}

double convert_vprog_to_supply(double input,double FS){
  return (input*FS/max_pin);
}
// required inversions
int convert_vout_to_daccode(double input){
  return lround(input*4096.0/4.096);
}

double convert_vprog_to_vout(double input,double fs){
  return (input/fs);
}

double convert_supply_to_vprog(double input,double FS){
  return (input*4.64/FS);
}

uint16_t full_conversion(double write_val_d, double FS_X, double fs_x, double max_in){
  if(write_val_d<=max_in){
    double voltage_on_programming_pin=convert_supply_to_vprog(write_val_d,FS_X);
    Serial.println(voltage_on_programming_pin,4);
    double vout_DAC=convert_vprog_to_vout(voltage_on_programming_pin,fs_x);
    Serial.println(vout_DAC,4);
    long int to_write_code=convert_vout_to_daccode(vout_DAC);
    Serial.println(to_write_code);
    if(to_write_code<=4096) return ((uint16_t)(to_write_code));
  }
  else return 0;
}

// as doubles of microamps or Volts
void write_the_value_to_HV_C_d(double write_val_d){
  uint16_t to_write_code = full_conversion(write_val_d, highvolt_FS, cathode_scale_factor,cathode_volt_FS);
  voltage_cathode=(uint16_t) to_write_code;
  CATChannelProgram(voltage_cathode, 0);
  //Serial_print_header(voltage_on_programming_pin, DAC_value_d, voltage_cathode);
  Serial.println("Cathode Wire Voltage changed");
}
void write_the_value_to_Cat_Ilim_d(double write_val_d){
  uint16_t to_write_code = full_conversion(write_val_d, highcurrent_FS, cathode_scale_factor,cathode_current_FS);  
  current_cathode = (uint16_t) to_write_code;
  CATChannelProgram(current_cathode, 1);
  //Serial_print_header(voltage_on_programming_pin, DAC_value_d, current_cathode);
  Serial.println("Cathode Wire Ilim changed");
}

void write_the_value_to_HV_P_d(double write_val_d){
  uint16_t to_write_code = full_conversion(write_val_d, highvolt_FS, potential_voltage_scale_factor,potential_volt_FS);
  voltage_potential=(uint16_t) to_write_code;
  POTChannelProgram(voltage_potential, 2);
  //Serial_print_header(voltage_on_programming_pin, DAC_value_d, voltage_potential);
  Serial.println("Potential Wire Voltage changed");
}
void write_the_value_to_Pot_Ilim_d(double write_val_d){
  uint16_t to_write_code = full_conversion(write_val_d, highcurrent_FS, potential_current_scale_factor,potential_current_FS);
  current_potential=(uint16_t) to_write_code;
  POTChannelProgram(current_potential, 3);
  //Serial_print_header(voltage_on_programming_pin, DAC_value_d, current_potential);
  Serial.println("Potential Wire Ilim changed");
}
void write_HV_d(int which_HV){
  if(length_sent<=8 && length_sent>=2){ // minimum number of bytes sent for something like ".1" is 2 bytes right?
      Serial.print("you sent: ");
      Serial.println(receive);
      //Serial.println("AHHH");
      char temp_chars[10];
      // as an int
      memcpy((uint8_t *) temp_chars,(uint8_t *) &receive +1, length_sent--);
      // can also do atof()
      double write_val_d=atof(temp_chars);
      //Serial.print("converted to double: ");
      //Serial.println(write_val_d);
      new_write=false;
      if(which_HV==1) write_the_value_to_HV_C_d(write_val_d);
      else if(which_HV==2) write_the_value_to_HV_P_d(write_val_d);
      else if(which_HV==3) write_the_value_to_Cat_Ilim_d(write_val_d);
      else if(which_HV==4) write_the_value_to_Pot_Ilim_d(write_val_d);
      else Serial.println("choose 1 for Cat HV, 2 for Pot HV, 3 for Cat Ilim, 4 for Pot Ilim");
  }
}
void Serial_print_header(double voltage_pgm, double DAC_d, uint16_t DAC_bits){
  Serial.print("voltage on pgm pin: ");
  Serial.println(voltage_pgm);
  Serial.print("DAC double value: ");
  Serial.println(DAC_d);
  Serial.print("DAC int value: ");
  Serial.println(DAC_bits);

}
void write_the_value_to_PotA(){
  // need to store it as a uint8_t then write the uint8_t heater sheet array to the serial port
  if(write_val<256 && write_val>=0){
      uint8_t number=(uint8_t) write_val;
      uint8_t to_write_array[4]={0};
      to_write_array[0]=254;
      to_write_array[1]=171;
      to_write_array[2]=number;
//      to_write_array[3]=13; // carriage return duh
      //Serial3.write(to_write_array,4);
      Serial3.write(to_write_array,3);
      Serial.println("POTA");
  }
  
}

void write_the_value_to_PotB(int channel_sent){
  // need to store it as a uint8_t then write the uint8_t heater sheet array to the serial port
  if(write_val<256 && write_val>=0){
      uint8_t number=(uint8_t) write_val;
      if(channel_sent<=23 && channel_sent>=0){
        uint8_t channel=(uint8_t) channel_sent;
        uint8_t to_write_array[4]={0};
        to_write_array[0]=254;
        to_write_array[1]=170;
        to_write_array[2]=channel;
        to_write_array[3]=number;
        //to_write_array[3]=13;
        //Serial2.write(to_write_array,4);
        Serial3.write(to_write_array,4);
        Serial.println("POTB");
      }
  }
  
}

void write_Potentiometer_channel(){
  if(length_sent<=8 && length_sent>=4){
      Serial.print("you sent: ");
      Serial.println(receive);
      //Serial.println("AHHH");
      char temp_chars[10];
      // as an int
      memcpy((uint8_t *) temp_chars,(uint8_t *) &receive +1, length_sent-1);
      //write_val=atoi(temp_chars);
      // do I have to find the comma?
      int loc_comma=0;
      char before_comma[10];
      char after_comma[10];
      for(int i=0;i<9;i++){
        if(temp_chars[i]==',') loc_comma=i;
      }
      if(loc_comma>0){
        int j_after=0;
        for(int i=0;i<9;i++){
          if(i<loc_comma) before_comma[i]=temp_chars[i];
          else if(i>loc_comma) after_comma[j_after++]=temp_chars[i];
        }
        write_val=atoi(after_comma);
        int channel_to_send=atoi(before_comma);
        Serial.print("writeval integer: ");
        Serial.println(write_val);
        Serial.print("channel integer: ");
        Serial.println(channel_to_send);        
        // convert to integer
        write_the_value_to_PotB(channel_to_send);
      }
      else{
        Serial.println("could not write channel to potentiometer");
      }
      new_write=false;
  }
}

void write_Potentiometer(uint8_t which_pot){
  if(length_sent<=5 && length_sent>=2){
      Serial.print("you sent: ");
      Serial.println(receive);
      //Serial.println("AHHH");
      char temp_chars[10];
      // as an int
      memcpy((uint8_t *) temp_chars,(uint8_t *) &receive +1, length_sent--);
      write_val=atoi(temp_chars);
      Serial.print("converted to integer: ");
      Serial.println(write_val);
      new_write=false;
      //write_the_value_to_HV();
      if(which_pot==1) write_the_value_to_PotA();
  }   
}

void raise_CHV(){
  Serial.println("Raising HV on Cathode wires");
  step_HV_C=true;
  step_HV_C_time=millis()+CATHODE_VOLTAGE_STEP_PERIOD;
  voltage_cathode=0;
}


void raise_PHV(){
  Serial.println("Raising HV on Potential wires");
  step_HV_P=true;
  step_HV_P_time=millis()+POTENTIAL_VOLTAGE_STEP_PERIOD;
  voltage_potential=0;
}
void cancel_all_HV(){
    step_HV_P=false;
    step_HV_C=false;
    POTChannelProgram(0, 2);
    CATChannelProgram(0, 0);
    Serial.println("All HV lines set to zero and ramping cancelled");
}
void pause_all_HV(){
    step_HV_P=false;
    step_HV_C=false;
    Serial.println("All HV lines paused");
}
void continue_all_HV(){
    step_HV_P=true;
    step_HV_C=true;
    step_HV_P_time=millis()+POTENTIAL_VOLTAGE_STEP_PERIOD;
    step_HV_C_time=millis()+CATHODE_VOLTAGE_STEP_PERIOD;

    Serial.println("All HV lines resumed");
}
/*******************************************************************************
 * Packet handling functions
 *******************************************************************************/
void checkHdr(const void *sender, const uint8_t *buffer, size_t len) {
  // Default header & error data values
  hdr_out->src = myID;          // Source of data packet
  hdr_in = (housekeeping_hdr_t *)buffer;
  hdr_prio = (housekeeping_prio_t *) (buffer + hdr_size);
    // If an error occurs at this device from a message
  if (hdr_in->dst == eBroadcast || hdr_in->dst==myID) hdr_err->dst = myID;
  else hdr_err->dst = hdr_in->dst;
  // If the checksum didn't match, throw a bad args error
  // Check for data corruption
  if (!(verifyChecksum((uint8_t *) buffer))) {
      //error_badArgs(hdr_in, hdr_out, hdr_err);  
      buildError(hdr_err, hdr_out, hdr_in, EBADARGS);
      fillChecksum((uint8_t *) outgoingPacket);
      downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);
      currentPacketCount++;
  }
  else {
  // Check if the message is a broadcast or local command and only then execute it. 
    if (hdr_in->dst == eBroadcast || hdr_in->dst==myID) {
      if(hdr_in->cmd==eTestMode) handleTestMode(hdr_in, (uint8_t *) hdr_in + hdr_size, (uint8_t *) outgoingPacket);
      else if ((int)(hdr_in->cmd < 254) && (int)(hdr_in->cmd > 249)) handlePriority(hdr_in->cmd, (uint8_t *) outgoingPacket); // for doing a send of priority type.
      else handleLocalCommand(hdr_in, (uint8_t *) hdr_in + hdr_size, (uint8_t *) outgoingPacket); // this constructs the outgoingpacket when its a localcommand and sends the packet.
    } 
  // If the message wasn't meant for this device pass it along (up is away from SFC and down and is to SFC
    else forwardDown(buffer, len, sender);
  }
}
// forward downstream to the SFC
void forwardDown(const uint8_t * buffer, size_t len, const void * sender) {
  downStream1.send(buffer, len);
  checkDownBoundDst(sender);
  currentPacketCount++;
}

/* checkDownBoundDst Function flow:
 * --Checks to see if the downstream device that sent the message is known
 *    --If not, add it to the list of known devices
 *    --If yes, just carry on
 * --Executed every time a packet is received from downStream
 * 
 * Function params:
 * sender:    PacketSerial instance (serial line) where the message was received
 * 
 */
void checkDownBoundDst(const void * sender) {
  if (serialDevices == (PacketSerial *) sender){
    if (addressList == 0) {
      addressList = (uint8_t) hdr_in->src;
      numDevices++;
      return;
    }
  }
}
/* Function flow:
 * --Find the device address that produced the error
 * --Execute the bad length function & send the error to the SFC
 * Function params:
 * sender:    PacketSerial instance which triggered the error protocol
 * Send an error if a packet is unreadable in some way */
void badPacketReceived(PacketSerial * sender){
  if (sender == serialDevices){
    hdr_in->src = addressList;
  }
  hdr_out->src = myID;
  buildError(hdr_err, hdr_out, hdr_in, EBADLEN);
  fillChecksum((uint8_t *) outgoingPacket);
  downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);
  currentPacketCount++;
}

// Function for building the error packets to send back when an error is found (see the Core_Protocol.h for the defs of the errors and the error typdefs).
void buildError(housekeeping_err_t *err, housekeeping_hdr_t *respHdr, housekeeping_hdr_t * hdr, int error){
  respHdr->cmd = eError;
  respHdr->len = 4;
  err->src = hdr->src;
  err->dst = hdr->dst;
  err->cmd = hdr->cmd;
  err->error = error;
}
/***********************
/*******************************************************************************
 * END OF Packet handling functions
 *******************************************************************************/

// sending priority command function
// probably can be cleaned up
// Note: SendAll is 253 and SendLow is 250 so we made SendLow-> int priority=1 for checking the device's list of command's priorities.
// got a priority request from destination dst
void handlePriority(uint8_t prio_in, uint8_t * responsePacketBuffer){
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + hdr_size;
  int priority=0;
  int retval = 0;
  uint8_t sum = 0; // hdr length of data atatched from all those commands data
//  respHdr->cmd = hdr_in->cmd;
  // priority == 4 when this function is called is code for "eSendAll"
  // otherwise priority=1,2,3 and that maps to eSendLowPriority+priority
  if(prio_in==eSendAll) priority=4;
  else priority = prio_in - 249;
//  int retval;
  respHdr->src = myID;
  respHdr->dst = eSFC;
  respHdr->cmd =  prio_in;
  // go through every priority
  for (int i=0;i<NUM_LOCAL_CONTROLS;i++) {
    if (commandPriority[i] == (uint8_t)priority || priority==4) {
      retval=handleLocalRead((uint8_t) i + FIRST_LOCAL_COMMAND, respData+sum);
      // if that read overflowed the data???? fix later?
      sum+= (uint8_t) retval;
    }
    else sum+=0;
  }
  respHdr->len=sum;
  fillChecksum(responsePacketBuffer);
  downStream1.send(responsePacketBuffer, respHdr->len + hdr_size + 1);
  currentPacketCount++;
}

// function for when a "SetPriority" command is received by this device, adding that commands priority value to the array/list
void setCommandPriority(housekeeping_prio_t * prio, uint8_t * respData, uint8_t len) {
//  housekeeping_prio_t * set_prio = (housekeeping_prio_t *) prio;
  commandPriority[prio->command-FIRST_LOCAL_COMMAND] = (uint8_t) prio->prio_type;
  memcpy(respData, (uint8_t*)prio, len);
}
// Fn to handle a local command write.
// This gets called when a local command is received
// with data (len != 0).
int handleLocalWrite(uint8_t localCommand, uint8_t * data, uint8_t len, uint8_t * respData) {
  int retval = 0;
  switch(localCommand) {
  case eSetPriority:
    setCommandPriority((housekeeping_prio_t *)data,respData,len);
    retval=len;
    break;
  case eHeaterControlAll:{
    //uint8_t pot_array[3]={254,171,*data};
    //Serial3.write(254);
    //Serial3.write(171);
    //Serial3.write(*data);
    //Serial5.write(pot_array,3);
    uint8_t readval=0;
    //readval=Serial3.read();
    memcpy(respData,&readval,sizeof(readval));
    retval = sizeof(readval);
    break;
  }
  case eHeaterControlChannel:{
    //Serial3.write(254);
    //Serial3.write(170);
    //Serial3.write(*data);
    //Serial3.write(*data+1);
     uint8_t readval=0;
    //readval=Serial3.read();
    memcpy(respData,&readval,sizeof(readval));
    retval = sizeof(readval);
    break;
  }
  case eVPGMPotential:{
    // write to I2C? should be 12-bit DAC, MCP4728, aka two bytes (not more than 4096 in total value).
    memcpy((uint8_t * ) &voltage_potential, data, len);
    if(((int) voltage_potential <= 4095) && ((int) voltage_potential >= 0)){
      POTChannelProgram(voltage_potential, 3);
      memcpy(respData,(uint8_t *) &voltage_potential, len);
    }
    else {
      voltage_potential=65535;  // set to all ones if the value was too high for the 12bit DAC
      memcpy(respData,(uint8_t *) &voltage_potential, len);
    }
    retval=len;
    break;
  }
  case eIPGMPotential:{
    // write to I2C? should be 12-bit DAC, MCP4728, aka two bytes (not more than 4096 in total value).
    memcpy((uint8_t * ) &current_potential, data, len);
    if(((int) current_potential <= 4095) && ((int) current_potential >= 0)){
      POTChannelProgram(current_potential, 0);
      memcpy(respData,(uint8_t *) &current_potential, len);
    }
    else {
      current_potential=65535;  // set to all ones if the value was too high for the 12bit DAC
      memcpy(respData,(uint8_t *) &current_potential, len);
    }
    retval=len;
    break;
  }
  case eVPGMCathode:{
    // write to I2C? should be 12-bit DAC, MCP4728, aka two bytes (not more than 4096 in total value).
    memcpy((uint8_t * ) &voltage_cathode, data, len);
    if(((int) voltage_cathode <= 4095) && ((int) voltage_cathode >= 0)){
      CATChannelProgram(voltage_cathode, 3);
      memcpy(respData,(uint8_t *) &voltage_cathode, len);
    }
    else {
      voltage_cathode=65535;  // set to all ones if the value was too high for the 12bit DAC
      memcpy(respData,(uint8_t *) &voltage_cathode, len);
    }
    retval=len;
    break;
  }
  case eIPGMCathode:{
    // write to I2C? should be 12-bit DAC, MCP4728, aka two bytes (not more than 4096 in total value).
    memcpy((uint8_t * ) &current_cathode, data, len);
    if(((int) current_cathode <= 4095) && ((int) current_cathode >= 0)){
      CATChannelProgram(current_cathode, 0);
      memcpy(respData,(uint8_t *) &current_cathode, len);
    }
    else {
      current_cathode=65535;  // set to all ones if the value was too high for the 12bit DAC
      memcpy(respData,(uint8_t *) &current_cathode, len);
    }
    retval=len;
    break;
  }
  case ePacketCount:{
    retval = EBADLEN;
    break;
  }
  default:
    retval=EBADCOMMAND;    
    break;
  }
  return retval;
}

// Fn to handle a local command read.
// This gets called when a local command is received
// with no data (len == 0)
// buffer contains the pointer to where the data
// will be written.
// int returns the number of bytes that were copied into
// the buffer, or EBADCOMMAND if there's no command there
int handleLocalRead(uint8_t localCommand, uint8_t *buffer) {
  int retval = 0;
  switch(localCommand) {
  case ePingPong:
    retval=0;
    break;
  case eSetPriority:
    retval = EBADLEN;
    break;
  case eDCTIntSensorRead: {
    uint32_t TempRead=analogRead(TEMPSENSOR);
    float TempC = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
    memcpy(buffer,(uint8_t *) &TempC,sizeof(TempC));
    retval=sizeof(TempC);
    break;
  }
  case eThermistorsTest:{
    int len_therms = 4; // length of the thermistor channels to send back?
// Have just plan Serial.read() 
//    retval = Serial2.readBytes(buffer,len_therms);
//    whatToDoIfThermistors(buffer);
    break;}
  case ePacketCount:
    memcpy(buffer, (uint8_t *) &currentPacketCount, sizeof(currentPacketCount));
    retval = sizeof(currentPacketCount);
    break;
  case eThermistors:{
    retval = sizeof(sDCTThermistors); // length of the thermistor channels to send back?
//    float res= 10;
//    float res = return_resistance(CHIP_SELECT_A, 10);
//    float res = measure_channel_2(CHIP_SELECT, 10); // test functions to do the read correctly
//    float res = return_resistance_2(CHIP_SELECT, 10); // read the channel 10 resistance from RTD
    memcpy(buffer,(uint8_t *) &thermistors, sizeof(sDCTThermistors));
    break;}
  case eHVmon:{
    memcpy(buffer, (uint8_t *) &hvmon,sizeof(sDCTHV));
    retval=sizeof(sDCTHV);
    break;
  }
  case eVPGMPotential:{
    memcpy(buffer, (uint8_t *) &voltage_potential,sizeof(voltage_potential));
    retval=sizeof(voltage_potential);
    break;
  }
  case eIPGMPotential:{
    memcpy(buffer, (uint8_t *) &current_potential,sizeof(current_potential));
    retval=sizeof(current_potential);
    break;
  }
  case eVPGMCathode:{
    memcpy(buffer, (uint8_t *) &voltage_cathode,sizeof(voltage_cathode));
    retval=sizeof(voltage_cathode);
    break;
  }
  case eIPGMCathode:{
    memcpy(buffer, (uint8_t *) &current_cathode,sizeof(current_cathode));
    retval=sizeof(current_cathode);
    break;
  }

  case eVMONCathode:{
    memcpy(buffer, (uint8_t *) hvmon.CatVmon,sizeof(hvmon.CatVmon));
    retval=sizeof(hvmon.CatVmon);
    break;
  }
  case eIMONCathode:{
    memcpy(buffer, (uint8_t *) hvmon.CatImon,sizeof(hvmon.CatImon));
    retval=sizeof(hvmon.CatImon);
    break;
  }
  case eVMONPotential:{
    memcpy(buffer, (uint8_t *) hvmon.PotVmon,sizeof(hvmon.PotVmon));
    retval=sizeof(hvmon.PotVmon);
    break;
  }
  case eIMONPotential:{
    memcpy(buffer, (uint8_t *) hvmon.PotImon,sizeof(hvmon.PotImon));
    retval=sizeof(hvmon.PotImon);
    break;
  }
  case ePressure:{
    memcpy(buffer,(uint8_t *)&dct_pressure,sizeof(sDCTPressure));
    retval=sizeof(sDCTPressure);
    break;
  }
  case eDCTISR: {
    uint32_t TempRead=analogRead(TEMPSENSOR);
    float TempC = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
    memcpy(buffer,(uint8_t *) &TempC,sizeof(TempC));
    retval=sizeof(TempC);
    break;
  }
  case eReset: {
    SysCtlReset();
    retval = 0;
  }
  default:
    retval=EBADCOMMAND;
  }  
  return retval;
}

// Function to call first when localcommand sent. 
// Store the "result" as retval (which is the bytes read or written, hopefully)
void handleLocalCommand(housekeeping_hdr_t *hdr, uint8_t * data, uint8_t * responsePacketBuffer) {
  int retval=0;
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + sizeof(housekeeping_hdr_t);
  respHdr->src = myID;
  respHdr->dst = eSFC;
  if (hdr->len) {
    retval = handleLocalWrite(hdr->cmd, data, hdr->len, respData); // retval is negative construct the baderror hdr and send that instead. 
    if(retval>=0) {
//      *respData= 5;
      respHdr->cmd = hdr->cmd;
      respHdr->len = retval; // response bytes of the write.
    }
    else{
      housekeeping_err_t *err = (housekeeping_err_t *) respData;
      buildError(err, respHdr, hdr, retval);
    }  
  } 
  else {
    // local read. by definition these always go downstream.
    retval = handleLocalRead(hdr->cmd, respData);
    if (retval>=0) {
      respHdr->cmd = hdr->cmd;
      respHdr->len = retval; //bytes read
    }
    else {
      housekeeping_err_t *err = (housekeeping_err_t *) respData;
      buildError(err, respHdr, hdr, retval); // the err pointer is pointing to the data of the response packet based on the line above so this fn fills that packet. 
    }
  }
  fillChecksum(responsePacketBuffer);
  // send to SFC
  downStream1.send(responsePacketBuffer, respHdr->len + hdr_size + 1 );
  currentPacketCount++;
}

void handleTestMode(housekeeping_hdr_t *hdr, uint8_t *data, uint8_t * responsePacketBuffer) {
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + sizeof(housekeeping_hdr_t);
  respHdr->src = myID;
  respHdr->dst = hdr->src;
// if length was actually placed then go into testmode, else build badlength error.
  if (hdr->len) {
   //construct data incoming to be the num testpackets and send the data packet in a while loop and decrement numtestpackets?
    uint16_t numTestPackets = ((uint16_t) (*(data+1) << 8)) | *(data) ; // figure out the correct way to get 2 bytes into a 16_t
    timelastpacket = millis();
    while(numTestPackets){
      if(long (millis()-timelastpacket)>0) { // only send every 50 milliseconds?
        *(respData) = numTestPackets;    
        *(respData+1) = numTestPackets >> 8;
        respHdr->cmd = hdr->cmd;
        respHdr->len = 0x02; // response bytes of the write.
        fillChecksum(responsePacketBuffer);
        // send to SFC
        downStream1.send(responsePacketBuffer, respHdr->len + sizeof(housekeeping_hdr_t) + 1 );  
        numTestPackets--;
        timelastpacket = timelastpacket+TEST_MODE_PERIOD;
        currentPacketCount++;
      }
    }
  }
  else{
    housekeeping_err_t *err = (housekeeping_err_t *) respData;
    buildError(err, respHdr, hdr, EBADLEN); 
    fillChecksum(responsePacketBuffer);
    // send to SFC
    downStream1.send(responsePacketBuffer, respHdr->len + sizeof(housekeeping_hdr_t) + 1 );  
    currentPacketCount++;
  }  
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
