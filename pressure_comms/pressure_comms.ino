// Use to test the pressure gauge at scott's crib
#include "Pressure.h"
#include "Wire_nonblocking.h"
#include "DS2482_nonblocking_v2.h"
Pressure pressure;
char response[200]={0};
int baud_pressure =19200;
int new_baud =19200;
char one_byte;
char helix[8]="HELIX";
uint8_t unit=2;
char pressureBuffer;
#define PRESSURE_UPDATE_PERIOD 1000  // period between reads/writes
unsigned long PressureUpdateTime=0;  // for transmit recording time
uint8_t matched;
uint8_t pressureNumSend=0;
typedef enum {
  Pressure_Idle=0,
  Pressure_Request=1,
  Pressure_Read_First=2,
  Pressure_Read=3,
  Pressure_Finish=4,
} Pressure_State_t;
Pressure_State_t Pressure_State=Pressure_Idle;
// two options, run as state machine or run as requests and wait. Current is state machine.
float pres;
bool pres_request=true;  // to alternate between pressure and temperature reads of the transducer with the same switch case. 

int avail_test;
// I2C 1 wire bridge states
typedef enum {
  SENSOR_STATE_I2C_IDLE = 0, // checks for time delay 
  SENSOR_STATE_I2C_RESET = 1, // request a read
  SENSOR_STATE_I2C_SELECTCHANNEL_Setup = 2, // request a read
  SENSOR_STATE_I2C_SELECTCHANNEL_Request = 3, // checking rx 
  SENSOR_STATE_I2C_SELECTCHANNEL_Bool = 4, // checking tx
  SENSOR_STATE_I2C_RESETWIRE_Setup = 5,
  SENSOR_STATE_I2C_RESETWIRE_Finish = 6, // checking tx
  SENSOR_STATE_I2C_WIRE_SELECT = 7,
  SENSOR_STATE_I2C_WIRE_WRITE = 8, // checking tx
  SENSOR_STATE_I2C_READ_Setup = 9,
  SENSOR_STATE_I2C_READ_Next = 10, // checking tx
  SENSOR_STATE_I2C_READ_Request = 11, // checking tx
  SENSOR_STATE_I2C_READ_Read = 12, // checking tx
  SENSOR_STATE_I2C_STARTUP_RESET = 13,
  SENSOR_STATE_I2C_STARTUP = 14,
  SENSOR_STATE_I2C_STARTUP_REQUEST = 15,
  SENSOR_STATE_I2C_STARTUP_CHECK = 16,
} SensorState;

typedef enum {
  INTERNAL_STATE_Setup=0,
  INTERNAL_STATE_Request=1,
  INTERNAL_STATE_Read=2,
  INTERNAL_STATE_Finish=3,
} Internal_state;
Internal_state wirestatus=INTERNAL_STATE_Setup;
SensorState Onewire_state=SENSOR_STATE_I2C_STARTUP_RESET;
// I2C 1Wire Bridge vars
int wirestatus_tries=0;
float celsius[2]={0};
#define SENSOR_UPDATE_PERIOD 1500  // period between reads/writes
unsigned long sensorUpdateTime=0;  // for transmit recording time
#define INTERNAL_DELAY 1
unsigned long InternalStatusTime=0;  // for 1wire devices busy wait
#define SENSOR_DELAY_PERIOD 50  // period between telling probe to write temp and reading temp
unsigned long sensorDelayTime=0;  // for transmit recording time
int temp_tracker=0; // to record which temp1wire probe we are using. 
unsigned long i2c_onewire=1;
bool Onewire_conn;
uint8_t chan_1_wire=4;
TwoWire *wire_onewirebus_obj= new TwoWire(i2c_onewire); // i2C object for the i2c port on the launchpad
DS2482 ds_sm(0,*wire_onewirebus_obj); // OneWire bridge object for sending commands to the OneWire bridge
uint8_t channel_rtn;
uint8_t address1[8]={0x28,0x7B,0xAA,0x79,0x97,0x06,0x03,0xD1};
//uint8_t address2[8]={0x28,0x6A,0xC6,0x79,0x97,0x06,0x03,0x8D};
uint8_t address2[8]={0x28,0x0E,0xA4,0x79,0x97,0x06,0x03,0x00};
uint8_t address_1wire[8];
bool status_bool=true; // for the 1wire bridge comms to 1wire devices
uint8_t write_temp_cmd; 
bool second_write=false;
int select_step=0; //used to go into same state 9 times woah
int read_step=0; //incrementer for reading the temp sensor data bytes
byte data_1wire[12]={0};  // data from a 1wire device
uint8_t rx_state=I2C_BUSY;
uint8_t tx_state=I2C_BUSY;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  pressure.begin(Serial1,baud_pressure);
  Serial.print("starting...");
//  pressure.read_port_again();
//  print_pressure();
  pressure.reset_string();
    // new i2c 1wire bridge setup
  wire_onewirebus_obj->begin();
  wire_onewirebus_obj->flush();
  //while(wire_bridge_broken());
}
// state machine loop
void loop(){
  // idle should check if available, and go to read it if there is something available. otherwise should check the timer to request a new read.
  switch (Pressure_State){
    case Pressure_Idle:{
      if(pressure.is_available()){ // something was ready to read so go read it
        Pressure_State=Pressure_Read_First;
        //print_status();
        break;
      }
      else {  // nothing to read so check to see if time to request a read
        if ((long) (millis() - PressureUpdateTime) > 0) {
          //print_status();
          PressureUpdateTime = millis() + PRESSURE_UPDATE_PERIOD;
          Pressure_State=(Pressure_State_t) ((unsigned char) Pressure_State + 1);
          pressureNumSend=0;
          break;
        }
        else break;
      }
    }
    case Pressure_Request:{
      // make sure no Serial1.available before requesting,either way go to read state.
      //print_status();
      if(!pressure.is_available()){
        if(pres_request) pressure.poll_pressure4(pressureNumSend);
        else pressure.poll_temp(pressureNumSend);
        pressureNumSend++;
      }
      if(pressureNumSend>=4) Pressure_State=Pressure_Idle;
      break;
    }
    case Pressure_Read_First:{
      //print_status();
      if (pressure.is_available()){
        pressureBuffer=pressure.get_one_byte();
        //Serial.print(pressureBuffer);
        if(pressureBuffer='@'){ // if it is an @ go to more reads, if not check again.
          Pressure_State=(Pressure_State_t) ((unsigned char) Pressure_State + 1);        
          break;
        }
        else {
          pressure.reset_string();
          break;
        }
      }
      else break;
    }
    case Pressure_Read:{
      // read and check each var
      //print_status();
      if(pressure.is_available()) {
        pressureBuffer = pressure.get_one_byte();
        if (matched == 1 || matched == 2) {
          if (pressureBuffer == 'F') matched++;
          else matched = 0;
        }
        if (matched == 0) {
          if (pressureBuffer == ';') matched++;
        }
        if (matched == 3) { // done, move to next state
          Pressure_State=(Pressure_State_t) ((unsigned char) Pressure_State + 1);
          matched=0;       
        }
        break;
      }
      else break;
    }
    case Pressure_Finish:{
      // fill in the pressure variable from the response, this also clears the object's char array of the response to reset.
      if(pres_request){
        pres=pressure.give_me_the_float();
        pres_request=false;
      }
      else {
        pres=pressure.give_me_the_float();
        pres_request=true;        
      }
      Serial.print("\n");
      Serial.print(pres,3);
      //if(!pres_request) Serial.print("pressure");
      //else Serial.print("temp");
      Serial.print("\n");
      // go back to idle
      Pressure_State=Pressure_Idle;
      //delay(1000);
      break;
    }
  }
  // add in wire SM stuff and see if we can get them all to print or why not?
//  Serial.print("state var is: ");
//  Serial.print(Pressure_State);
//  Serial.print("\n");
  //  i2c one wire bridge conn
  // new states for 1wire bridge
  if(ds_sm.RXStatus()==0 && ds_sm.TXStatus()==0){
    switch (Onewire_state){
      case SENSOR_STATE_I2C_IDLE:{
        if ((long) (millis() - sensorUpdateTime) > 0) {
          sensorUpdateTime = millis() + SENSOR_UPDATE_PERIOD;
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
          status_bool=true;
          break;
        }
        else break;
      }
      case SENSOR_STATE_I2C_RESET:{
        ds_sm.reset();
        Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
        wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
        status_bool=false;  // do we need to first set the bridge register pointer? no because after a reset the status register points to 
        break;
      }
      case SENSOR_STATE_I2C_SELECTCHANNEL_Setup:{
        if(readstatus_stuff()){  // once the readstatus is done, do stuff
          channel_rtn=ds_sm.selectChannelSetup(chan_1_wire);
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
          wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
          status_bool=false;
          break;
        }
        else break; // by default keep trying to readstatus of 1wire device
      }
      case SENSOR_STATE_I2C_SELECTCHANNEL_Request:{
        if(readstatus_stuff()){  // once the readstatus is done, do stuff
          ds_sm.selectChannelRequest();
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
          break;
        }
        else break; // by default keep trying to readstatus of 1wire device        
      }
      case SENSOR_STATE_I2C_SELECTCHANNEL_Bool:{
        ds_sm.selectChannelBool(channel_rtn);        
        Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
        wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
        status_bool=false;
        break;
      }
      case SENSOR_STATE_I2C_RESETWIRE_Setup:{
        if(readstatus_stuff()){  // once the wire status is not busy, then do stuff
          // if its time for a second write then delay a little to give the probe time to convert?
          if(second_write) { // delay
             if ((long) (millis() - sensorDelayTime) > 0) {
               ds_sm.wireReset();
               Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
               wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
               status_bool=false;
               break;
             }
             else break;
          }
          else{
            ds_sm.wireReset();
            Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
            wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
            status_bool=false;
            break;
          }
        }
        else break; // by default keep trying to readstatus of 1wire device
      }
      case SENSOR_STATE_I2C_RESETWIRE_Finish:{
        if(readstatus_stuff()){  // once the readstatus is done, do stuff
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
          wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
          status_bool=false;
          break;
        }
        else break;
      }
      case SENSOR_STATE_I2C_WIRE_SELECT:{
        // need to be in here 9 times, with a status check(true) before each one
        if(select_step<=8){
          if(readstatus_stuff()){
            if(select_step==0) ds_sm.wireSelectBegin(address_1wire);
            else ds_sm.wireSelectStep();
            wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire device read status check 
            status_bool=false;
            select_step++;
            break;
          }
          else break;
        }
        else { // finally done selecting the 1wire device
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);             
          select_step=0;
          wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire device read status check 
          status_bool=false;          
          break;
        }
      }
      case SENSOR_STATE_I2C_WIRE_WRITE:{
        if(readstatus_stuff()){  // once the readstatus is done, do stuff
          if(second_write){ // if this is the second time the temp cmd was written, its a different value and move to next state, otherwise go back to wire reset state
            write_temp_cmd=0xBE;
            ds_sm.wireWriteByte(write_temp_cmd);
            Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
            wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
            status_bool=false;
            second_write=false;
            break;
          }
          else{  // go back to the wirereset if this is the first write
            write_temp_cmd=0x44;// need a delay in here?
            // another state then????
            sensorDelayTime = millis() + SENSOR_DELAY_PERIOD;
            ds_sm.wireWriteByte(write_temp_cmd);
            Onewire_state=SENSOR_STATE_I2C_RESETWIRE_Setup;
            wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
            status_bool=false;            
            second_write=true;
            break;
          }
        }
        else break;        
      }
      case SENSOR_STATE_I2C_READ_Setup:{
        if(readstatus_stuff()){  // once the readstatus is done, do stuff
          ds_sm.wireReadByteSetup();
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
          wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
          status_bool=false;
          break;
        }
        else break;
      }
      case SENSOR_STATE_I2C_READ_Next:{
        if(readstatus_stuff()){  // once the readstatus is done, do stuff
          ds_sm.wireReadByteNext();
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
          wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
          status_bool=false;  // we need to point back to the status register so we pass true.
          break;
        }
        else break;
      }
      case SENSOR_STATE_I2C_READ_Request:{
        ds_sm.wireReadByteRequest();
        Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
        break;
      }
      case SENSOR_STATE_I2C_READ_Read:{
        // read first, check later
        data_1wire[read_step]=ds_sm.wireReadByteRead();
        // incrememnt stepper
        read_step++;
        // now check to see how many times a read was done? looks like we only need to do 9 total. jump back to readByteSetup until 9 times. 
        //if we have read 12/9 bytes, then end it!
        if(read_step==9){
          read_step=0;
          write_celsius();
          temp_tracker++;
          if(temp_tracker >=2) temp_tracker=0;
          Onewire_state=SENSOR_STATE_I2C_IDLE;   // go back to wait till sensor time is up.
          break;
        }
        else {// we aint done reading so request some more.
          Onewire_state=SENSOR_STATE_I2C_READ_Setup;
          break;
        }
      }
      case SENSOR_STATE_I2C_STARTUP_RESET:{
          ds_sm.reset();
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
          wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
          status_bool=true;
          break;
      }
      case SENSOR_STATE_I2C_STARTUP:{
        if(readstatus_stuff()){  // once the readstatus is done, do stuff
          ds_sm.configureSetup(DS2482_CONFIG_APU); //we dont want active pullup because ?
          Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
          wirestatus=INTERNAL_STATE_Setup; // initialize the things for 1wire devices 
          status_bool=true;
          break;
        }
        else break; // by default keep trying to readstatus of 1wire device
      }
      case SENSOR_STATE_I2C_STARTUP_REQUEST:{        
        ds_sm.configureRequest();
        Onewire_state=(SensorState) ((unsigned char) Onewire_state + 1);
        break;
      }
      case SENSOR_STATE_I2C_STARTUP_CHECK:{
        uint8_t config_status=ds_sm.configureBool();
        if(config_status!=1){ // bad need to reset the bridge and go to startUp.
          Onewire_state=SENSOR_STATE_I2C_STARTUP_RESET;
          break;
        }
        else { // yay we can go on to do the readings. 
          Onewire_state=SENSOR_STATE_I2C_IDLE;
          break;
        }
      }
    } // end sm switch
  } // end if idles

 if(temp_tracker==0){
    for(int l=0; l<8;l++) address_1wire[l]=address1[l];  
  }
  else if(temp_tracker==1){
    for(int l=0; l<8;l++) address_1wire[l]=address2[l];  
  }
  // end of i2C 1wire bridge states
 // print_1wire();
 // print_internal();

}
// new I2C 1Wire bridge states
void check_states(){
  do{
    rx_state=ds_sm.RXStatus();
    tx_state=ds_sm.TXStatus();
  } while(rx_state!=0 || tx_state!=0);
}
void write_celsius(){
  int16_t raw = (data_1wire[1] << 8) | data_1wire[0];
  byte cfg = (data_1wire[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7; // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time
  celsius[temp_tracker] = (float)raw / 16.0;
//  if(celsius <125 && celsius>-55){
    // write out the celsius to the array of temp probes values
//    celsius;
// }
 //else Serial.print("BAD READ\n");
  Serial.print("\n");
  Serial.print(celsius[temp_tracker],4);
  Serial.print("\n");
}
void wirestatus_setup(bool value){
  ds_sm.wireReadStatusSetup(value);
}
bool readstatus_stuff(){
  switch(wirestatus){
    case INTERNAL_STATE_Setup:{
      if ((long) (millis() - InternalStatusTime) > 0) {
        InternalStatusTime = millis() + INTERNAL_DELAY;
        wirestatus_setup(status_bool); // if status is false then this just checks the TX of I2C since its about to read, if its true it writes to set the read ptr to status reg.
        wirestatus=(Internal_state) ((unsigned char) wirestatus + 1);
        return false;
      }
      else return false;
    }
    case INTERNAL_STATE_Request:{
      ds_sm.wireReadStatusRequest();
      wirestatus=(Internal_state) ((unsigned char) wirestatus + 1);
      return false;
    }
    case INTERNAL_STATE_Read:{
      uint8_t status_read=ds_sm.wireReadStatusRead();
      //Serial.print(status_read);
      if(status_read & 0x01){
        // it was busy so add one the delay to the time to wait longer
        InternalStatusTime = millis() + (1+wirestatus_tries)*INTERNAL_DELAY;
        wirestatus = INTERNAL_STATE_Setup;
        wirestatus_tries++;
        if(wirestatus_tries>20) {
          wirestatus_tries=0;
          InternalStatusTime = millis() + 10*INTERNAL_DELAY;
          // depending on the OneWire state we need to do something extra? but for sure if its during startup then go back to STARTUP RESET
          Onewire_state=SENSOR_STATE_I2C_STARTUP_RESET;
          // that temp is too busy so either go to another probe or reset that device?
          //Onewire_state=SENSOR_STATE_I2C_RESETWIRE_Setup;
          //status_bool=true;

        }
        return false;
      }
      else{
        wirestatus_tries=0;
        wirestatus=(Internal_state) ((unsigned char) wirestatus + 1);
        return false;
      }
    }
    case INTERNAL_STATE_Finish:{
      if(ds_sm.RXStatus()==0 && ds_sm.TXStatus()==0) return true;
      else return false;
    }
  }
}
/* request-receive loop
void loop() {
//  pressure.poll();
//  pressure.just_give_me_the_response(response);
//  print_pressure();
//  delay(10000);

  // put your main code here, to run repeatedly: 
  if (Serial.available() > 0) one_byte = Serial.read();
  switch(one_byte){
    case 49: {
      Serial.print("1\n");
      pressure.poll_pressure4();
//      pressure.just_give_me_the_response(response);
      float pres=pressure.give_me_the_float();
      Serial.print("\n");
      Serial.print(pres,3);
      Serial.print("\n");
      //print_pressure();
      one_byte=0;
      break;
    }
    case 50: {
      Serial.print("2\n");
      pressure.poll_temp();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 51: {
      Serial.print("3\n");
      pressure.poll_baud();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 52: {
      Serial.print("4\n");
      pressure.poll_address();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 53: {
      Serial.print("5\n");
      pressure.poll_model();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 54: {
      Serial.print("6\n");
      pressure.poll_name();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 55: {
      Serial.print("7\n");
      pressure.poll_manu();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 56: {
      Serial.print("8\n");
      pressure.poll_hv();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 57: {
      Serial.print("9\n");
      pressure.poll_fv();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 65: {
      Serial.print("A\n");
      pressure.poll_sn();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 66: {
      Serial.print("B\n");
      pressure.poll_time();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 67: {
      Serial.print("C\n");
      pressure.poll_string();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 68: {
      Serial.print("D\n");
      pressure.poll_status();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 69: {
      Serial.print("E\n");
      pressure.poll_units();
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 70: {
      Serial.print("F\n");
      pressure.unlock();
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      pressure.setstring(helix);
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      pressure.lock();
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      one_byte=0;
      break;
    }
    case 71: {
      Serial.print("G\n");
      pressure.setunits(unit);
      pressure.just_give_me_the_response(response);
      print_pressure();
      one_byte=0;
      break;
    }
    case 72: {
      Serial.print("H\n");
      pressure.unlock();
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      pressure.setbaud(new_baud);
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      pressure.lock();
      pressure.just_give_me_the_response(response);
      print_pressure();
      delay(1000);
      one_byte=0;
      break;
    }
    default: {
      Serial.print("null, latest response was: \n");
      pressure.read_port_again();
//      pressure.just_give_me_the_response(response);
      float pres=pressure.give_me_the_float();
      Serial.print("\n");
      Serial.print(pres,3);
      Serial.print("\n");
//      print_pressure();
    }
  }
  delay(3000);
}
*/
bool wire_busy(bool status_for_wire){
  check_states();
  ds_sm.wireReadStatusSetup(status_for_wire);
  check_states();
  ds_sm.wireReadStatusRequest();
  check_states();
//  Serial.print(ds_sm.wireReadStatusRead(),BIN);
  uint8_t busy_status=ds_sm.wireReadStatusRead();
  check_states();
  if(busy_status & 0x01) return true;
  else return false;
}
bool wire_bridge_broken(){
  // check rx and tx
  check_states();
  ds_sm.reset();
  check_states();
  while(wire_busy(true));
  check_states();
  // print off the configuration bits?
//  ds_sm.configureSetup(DS2482_CONFIG_APU); we dont want active pullup because ?
  check_states();
  ds_sm.configureRequest();
  check_states();
  uint8_t config_status=ds_sm.configureBool();
  check_states();
  if(config_status!=0) return true;
  else return false;
  //if configure does not return correctly, reset and try again.


}
void print_status(){
  Serial.print("state var is: ");
  Serial.print(Pressure_State);
  Serial.print("\n");
}

void print_1wire(){
  Serial.print("state var is: ");
  Serial.print(Onewire_state);
  Serial.print("\n");
}
void print_internal(){
  Serial.print("internal is: ");
  Serial.print(wirestatus);
  Serial.print("\n");
}
void print_pressure(){
  for(int i=0;i<200;i++){
    Serial.print(response[i]);
    response[i]=0;
  }
  Serial.print("\n");  
}
/*  pressure.poll();
  pressure.just_give_me_the_response(response);
  print_pressure();
  delay(10000);
*/
void print_response_parse(char response[20]){
  for(int i=0;i<20;i++){
    Serial.print(response[i]);
  }
  Serial.print("\n");  
}
/*float give_me_the_float(){
    char response_parse[20]={0};
    int K_loc=0;
    int semi_loc=0;
    for(int j=0;j<40;j++){
      if(response[j]==75) K_loc=j; // if K then put a space $
      if(response[j]==59) semi_loc=j; // if a semicolon put a space$
    }
    Serial.print(K_loc);
    Serial.print(semi_loc);    
    // now copy just that in between into response_parse
    for(int j=0;j<semi_loc-K_loc-1;j++){
      response_parse[j]=response[K_loc+1+j];
    }
    print_pressure();
    print_response_parse(response_parse);
    float pressure_new=atof(response_parse);
//    Serial.print("scanning... : ");
//    Serial.print(sscanf(response,"%s %f FF",response_parse,&pressure_new));
//    Serial.print("\n");
    return pressure_new;
  }
*/
