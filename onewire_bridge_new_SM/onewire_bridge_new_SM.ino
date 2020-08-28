#include "Pressure.h"
#include "Wire_nonblocking.h"
#include "DS2482_nonblocking_v2.h"

// I2C 1Wire Bridge vars
int wirestatus_tries=0;
float celsius[2]={0};
#define SENSOR_UPDATE_PERIOD 1000  // period between reads/writes
unsigned long sensorUpdateTime=0;  // for transmit recording time
#define INTERNAL_DELAY 3
unsigned long InternalStatusTime=0;  // for 1wire devices busy wait
#define NUM_TRIES 500
#define SENSOR_DELAY_PERIOD 100  // period between telling probe to write temp and reading temp
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
bool set_the_pointer=true; // for the 1wire bridge comms to 1wire devices
uint8_t write_temp_cmd; 
bool second_write=false;
int select_step=0; //used to go into same state 9 times woah
int read_step=0; //incrementer for reading the temp sensor data bytes
byte data_1wire[12]={0};  // data from a 1wire device
uint8_t rx_state=I2C_BUSY;
uint8_t tx_state=I2C_BUSY;
bool select_channel_bool;
uint8_t ppd_detect_state=1;
uint8_t ppd_tries=0;
unsigned long timer;
uint8_t mAddress = 0x18 | 0;
int state=0;
bool OneWB_ready=false; // false means the 1WB is busy and it timed out, true means it was not busy and so next command can be processed.
bool i2c_ready=false; // false means the i2c is busy and it timed out, true means it was not busy and so next command can be processed.
bool ppd_detected=false; // false means the ppd wasn't detected and it timed out, true means it was detected and so next cmd can be executed.
int timeouts=0;
int i2ctimeouts=0;
int counter=0;
uint8_t val=0; // use for reading registers and checking its value throughout code
// states to be used:
typedef enum {
  SENSOR_STATE_RESET = 0, // RESET BRIDGE
  SENSOR_STATE_CONFIG=1, // CONFIGURE BRIDGE APU
  SENSOR_STATE_CONFIG_CHECK=2, // CHECK CONFIG REGISTER
  SENSOR_STATE_SELECTCHANNEL = 3, // SELECT BRIDGE CHANNEL
  SENSOR_STATE_SELECTCHANNEL_CHECK = 4, // CHECK SELECT CHANNEL REGISTER
  SENSOR_STATE_SETPTR1=5, // SETPTR TO SR FOR 1WB
  SENSOR_STATE_WIRERESET1=6, // FIRST WIRE RESET
  SENSOR_STATE_WIREADDR_BEGIN1=7, // FIRST WIRE BEGIN ADDRESSING
  SENSOR_STATE_WIREADDR1=8, //REST OF WIRE ADDRESS
  SENSOR_STATE_WIRECONVERT=9,// WIRE CONVERT THE TEMP
  SENSOR_STATE_WIRERESET2=10, // SECOND WIRE RESET
  SENSOR_STATE_WIREADDR_BEGIN2=11, // SECOND WIRE BEGIN ADDRESSING
  SENSOR_STATE_WIREADDR2=12, //REST OF WIRE ADDRESS
  SENSOR_STATE_WIREREAD=13, // WIRE READ THE TEMP
  SENSOR_STATE_READTEMP=14, // READ TEMP FROM READ REGISTER
  SENSOR_STATE_WRITETEMP=15, // WRITE THE TEMP TO ARRAY OR SERIAL PORT
} SensorState;
SensorState ONEWIRE_STATE=SENSOR_STATE_RESET;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  //pressure.begin(Serial1,baud_pressure);
  Serial.print("starting...");
//  pressure.read_port_again();
//  print_pressure();
  //pressure.reset_string();
    // new i2c 1wire bridge setup
  wire_onewirebus_obj->begin();
  //wire_onewirebus_obj->flush();
  //while(wire_bridge_broken());
  timer=millis();
  timeouts=0;
  counter=0;
  //delay(10);

}
// state machine loop
// check_states is checking only the I2C RX/TX busies
// check1WB_again is checking the 1Wire Busy bit which invovles check_states and so only need to use check1WB_again
void loop(){
  ONEWIRE_STATE=SENSOR_STATE_RESET;
  //ONEWIRE_STATE=SENSOR_STATE_RESET; // top of loop anyways
  i2c_ready=check_states();
  //RESET BRIDGE
  if(i2c_ready){
    wire_onewirebus_obj->beginTransmission(mAddress);
    wire_onewirebus_obj->write(0xF0);
    wire_onewirebus_obj->endTransmission_nonblocking(1);
    ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
  }
  else {
    Serial.print("0");
  }
  // END RESET BRIDGE
  delayMicroseconds(100); // this seems to be necessary to give the 1WB time to go low??? needed a delay after the bridge reset and before the select channel this entire time. 
  // CONFIGURE APU
  OneWB_ready=check1WB_again(); //
  if(OneWB_ready && ONEWIRE_STATE==SENSOR_STATE_CONFIG){
    uint8_t config_byte=0x01; // 1<<0
    wire_onewirebus_obj->beginTransmission(mAddress);
    wire_onewirebus_obj->write(0xD2);
    wire_onewirebus_obj->write(config_byte | (~config_byte)<<4); // APU ONLY
    wire_onewirebus_obj->endTransmission_nonblocking(1);
    ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
  }
  else{
    // go back to reset
    ONEWIRE_STATE=SENSOR_STATE_RESET;
    timeouts++;
  }
  // check states and then Read Configure register
  if(ONEWIRE_STATE==SENSOR_STATE_CONFIG_CHECK){
    i2c_ready=check_states();
    if(i2c_ready){
      wire_onewirebus_obj->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
      if(check_states()){
        val=wire_onewirebus_obj->read();
        if(val==1){  // should be 1 because thats what we wrote to the config (APU)
          //Serial.print("156-");
          //Serial.print(val);
          ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
        }
      }
      else {
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        i2ctimeouts++;
      }
    }
    else{
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      i2ctimeouts++;
    }
  }
  // SELECT CHANNEL
  OneWB_ready=check1WB_again(); // state=
  if(OneWB_ready && ONEWIRE_STATE==SENSOR_STATE_SELECTCHANNEL) {// Select a channel
    wire_onewirebus_obj->beginTransmission(mAddress);
    wire_onewirebus_obj->write(0xC3);
    wire_onewirebus_obj->write(0xB4); // channel 4
    wire_onewirebus_obj->endTransmission_nonblocking(1);
    ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
  }
  else{
    // somehow go back to the top of loop resetting the bridge?, for now just print to Serial port
    ONEWIRE_STATE=SENSOR_STATE_RESET;
    timeouts++;
  }
  // check states and then Read channel register
  if(ONEWIRE_STATE==SENSOR_STATE_SELECTCHANNEL_CHECK){
    i2c_ready=check_states();
    if(i2c_ready){
      wire_onewirebus_obj->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
      if(check_states()){
        val=wire_onewirebus_obj->read();
        if(val==156){
          //Serial.print("156-");
          //Serial.print(val);
          ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
        }
      }
      else {
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        i2ctimeouts++;
      }
    }
    else{
      // somehow go back to the top of loop resetting the bridge?, for now just print to Serial port
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      i2ctimeouts++;
    }
  }
  // if the channel was selected properly above only then set the ptr correctly to read out the SR for 1WB
  if(ONEWIRE_STATE==SENSOR_STATE_SETPTR1 && (val==156)){
    if(check_states()){
      // set the ptr to the Status register
      wire_onewirebus_obj->beginTransmission(mAddress);
      wire_onewirebus_obj->write(0xE1); // bridge set ptr 
      wire_onewirebus_obj->write(0xF0); // where to set ptr (F0=SR)
      wire_onewirebus_obj->endTransmission_nonblocking(1);
      ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
    }
    else{
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      timeouts++;
    }
  }
  if(ONEWIRE_STATE==SENSOR_STATE_WIRERESET1){
    if(check_states()){
      OneWB_ready=check1WB_again(); //1wire busy must be 0 or next command cannot be executed by bridge
      if(OneWB_ready){
        wire_onewirebus_obj->beginTransmission(mAddress);
        wire_onewirebus_obj->write(0xB4); // 1wire device reset 
        wire_onewirebus_obj->endTransmission_nonblocking(1);
        ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
        if(check_states()){
          delayMicroseconds(500); 
        }
      }
      else{
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        timeouts++;
      }
    }
    else{
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      i2ctimeouts++;
    }
  }
  // check for ppd and 1WB
  if(ONEWIRE_STATE==SENSOR_STATE_WIREADDR_BEGIN1){
    ppd_detected=checkppd_again();
    OneWB_ready=check1WB_again(); //1wire busy must be 0 or next command cannot be executed by bridge
    if(ppd_detected && OneWB_ready){
      // start writing all of the address
      if(check_states()){
        write_byte_to_1wire(0x55); // write 0x55 to the 1wire device, telling 1 wire device to listen to the next 8 bytes for its address. 
        ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
      }
      else{
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        i2ctimeouts++;
      }
    }
    else{
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      timeouts++;
    }
  }
  if(ONEWIRE_STATE==SENSOR_STATE_WIREADDR1){
    for(int i=0; i<8; i++){
      if(check_states()){
        delayMicroseconds(600);
        OneWB_ready=check1WB_again();
        if(OneWB_ready) {
          counter++;
          if(check_states()){
            write_byte_to_1wire(address_1wire[i]);
            if(i==7) ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
          }
          else{
            i=10;
            ONEWIRE_STATE=SENSOR_STATE_RESET;
            i2ctimeouts++;
          }
        }
        else{
          i=10;
          ONEWIRE_STATE=SENSOR_STATE_RESET;
          timeouts++;
        }
      }
      else{
        i=10;
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        i2ctimeouts++;
      }
    }
  }
  if(ONEWIRE_STATE==SENSOR_STATE_WIRECONVERT){
    if(check_states()){
      OneWB_ready=check1WB_again();
      if(OneWB_ready){
        if(check_states()){
          write_byte_to_1wire(0x44);  // convert the temp broo
          ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
          if(check_states()){
            delay(10);
          }
          else{
            ONEWIRE_STATE=SENSOR_STATE_RESET;
            i2ctimeouts++;
          }
        }
        else{
          ONEWIRE_STATE=SENSOR_STATE_RESET;
          i2ctimeouts++;
        }
      }
      else{
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        timeouts++;
      }
    }
    else{
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      i2ctimeouts++;
    }
  }
  if(ONEWIRE_STATE==SENSOR_STATE_WIRERESET2){
    if(check_states()){
      OneWB_ready=check1WB_again(); //1wire busy must be 0 or next command cannot be executed by bridge
      if(OneWB_ready){
        wire_onewirebus_obj->beginTransmission(mAddress);
        wire_onewirebus_obj->write(0xB4); // 1wire device reset 
        wire_onewirebus_obj->endTransmission_nonblocking(1);
        ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
        if(check_states()){
          delayMicroseconds(500); 
        }
      }
      else{
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        timeouts++;
      }
    }
    else{
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      i2ctimeouts++;
    }
  }
  // check for ppd and 1WB
  if(ONEWIRE_STATE==SENSOR_STATE_WIREADDR_BEGIN2){
    ppd_detected=checkppd_again();
    OneWB_ready=check1WB_again(); //1wire busy must be 0 or next command cannot be executed by bridge
    if(ppd_detected && OneWB_ready){
      // start writing all of the address
      if(check_states()){
        write_byte_to_1wire(0x55); // write 0x55 to the 1wire device, telling 1 wire device to listen to the next 8 bytes for its address. 
        ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
      }
      else{
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        i2ctimeouts++;
      }
    }
    else{
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      timeouts++;
    }
  }
  if(ONEWIRE_STATE==SENSOR_STATE_WIREADDR2){
    for(int i=0; i<8; i++){
      if(check_states()){
        delayMicroseconds(600);
        OneWB_ready=check1WB_again();
        if(OneWB_ready) {
          //Serial.print("Y\n");
          counter++;
          if(check_states()){
            write_byte_to_1wire(address_1wire[i]);
            if(i==7) ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
          }
          else{
            i=10;
            ONEWIRE_STATE=SENSOR_STATE_RESET;
            i2ctimeouts++;
          }
        }
        else{
          i=10;
          ONEWIRE_STATE=SENSOR_STATE_RESET;
          timeouts++;
        }
      }
      else{
        i=10;
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        i2ctimeouts++;
      }
    }
  }
  if(ONEWIRE_STATE==SENSOR_STATE_WIREREAD){
    if(check_states()){
      OneWB_ready=check1WB_again();
      if(OneWB_ready){
        if(check_states()){
          write_byte_to_1wire(0xBE);  // Read the temp broo
          ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
          if(check_states()) Serial.print("E\n");
        }
        else{
          ONEWIRE_STATE=SENSOR_STATE_RESET;
          i2ctimeouts++;
        }
      }
      else{
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        timeouts++;
      }
    }
    else{
      ONEWIRE_STATE=SENSOR_STATE_RESET;
      i2ctimeouts++;
    }
  }
  if(ONEWIRE_STATE==SENSOR_STATE_READTEMP){ // we should check if it got here first then try to do the thing 9 times?
    for(int i=0; i<1; i++){
      if(check_states()){
        //delayMicroseconds(10);
        OneWB_ready=check1WB_again();
        if(OneWB_ready) {
          Serial.print("Y\n");
          counter++;
          if(check_states()){
            write_byte_to_1wire(0x96); // write 0x96 to generate a read on the read register of the bridge
            // this can take up to ~600 us to complete?
            if(check_states()){
              delayMicroseconds(650);
              // make sure the 1WB is 0
              OneWB_ready=check1WB_again();
              if(OneWB_ready){
                // set the read ptr
                Serial.print("Z\n");
                wire_onewirebus_obj->beginTransmission(mAddress);
                wire_onewirebus_obj->write(0xE1); // bridge set ptr 
                wire_onewirebus_obj->write(0xE1); // where to set ptr (E1=RR)
                wire_onewirebus_obj->endTransmission_nonblocking(1);  
                if(check_states()){
                  wire_onewirebus_obj->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
                  if(check_states()){
                    //data_1wire[i]=wire_onewirebus_obj->read();
                    Serial.print(wire_onewirebus_obj->read());
                    if(i==8) ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
                  }
                  else{
                    i=10;
                    ONEWIRE_STATE=SENSOR_STATE_RESET;
                    i2ctimeouts++;
                  }
                }
                else{
                  i=10;
                  ONEWIRE_STATE=SENSOR_STATE_RESET;
                  i2ctimeouts++;
                }
              }
              else{
                i=10;
                ONEWIRE_STATE=SENSOR_STATE_RESET;
                timeouts++;
              }
            }
            else{
              i=10;
              ONEWIRE_STATE=SENSOR_STATE_RESET;
              i2ctimeouts++;
            }
          }
          else{
            i=10;
            ONEWIRE_STATE=SENSOR_STATE_RESET;
            i2ctimeouts++;
          }
        }
        else{
          i=10;
          ONEWIRE_STATE=SENSOR_STATE_RESET;
          timeouts++;
        }
      }
      else{
        i=10;
        ONEWIRE_STATE=SENSOR_STATE_RESET;
        i2ctimeouts++;
      }
    }
  }
  if(ONEWIRE_STATE==SENSOR_STATE_WRITETEMP){
    write_celsius();
    delay(1234);
  }
  /*
  check_states();
  Serial.print("4");
  select_channel_bool=ds_sm.selectChannelBool(channel_rtn);
  check_states();
  check1WB_again();
  Serial.print("5");
  ds_sm.wireReset();
  // check for PPD
  while(checkppd_again());
  Serial.print("6");
  check1WB_again();
  ds_sm.wireSelectBegin(address_1wire);
  Serial.print("7");
  check1WB_again();
  Serial.print("8");
  for(int i=0; i<8; i++){
    ds_sm.wireSelectStep();
    check1WB_again();
  }
  ds_sm.wireWriteByte(0x44);
  check1WB_again();
  delay(100);
  ds_sm.wireReset();
 // while(checkppd_again());
  check1WB_again();
  ds_sm.wireSelectBegin(address_1wire);
  check1WB_again();
  for(int i=0; i<8; i++){
    ds_sm.wireSelectStep();
    check1WB_again();
  }
  ds_sm.wireWriteByte(0xBE);
  for(int i=0; i<9; i++){
    check1WB_again();
    ds_sm.wireReadByteSetup(); 
    check1WB_again();
    ds_sm.wireReadByteNext();
    check_states();
    ds_sm.wireReadByteRequest();
    check_states();
    data_1wire[i]=ds_sm.wireReadByteRead();
    check_states();
  }
  write_celsius();
  temp_tracker++;
  if(temp_tracker >=2) temp_tracker=0;
  delay(1000);
  if(temp_tracker==0){
    for(int l=0; l<8;l++) address_1wire[l]=address1[l];  
  }
  else if(temp_tracker==1){
    for(int l=0; l<8;l++) address_1wire[l]=address2[l];  
  }
  */
  //temp_tracker++;
  if(temp_tracker >=2) temp_tracker=0;
  if(temp_tracker==0){
    for(int l=0; l<8;l++) address_1wire[l]=address1[l];  
  }
  else if(temp_tracker==1){
    for(int l=0; l<8;l++) address_1wire[l]=address2[l];  
  }
//
  if(timeouts%100==0 && timeouts!=0) {
    Serial.print(timeouts);
  }
  if(i2ctimeouts!=0) {
    Serial.print("a");
    Serial.print(i2ctimeouts);
  }
 // if(counter%10==0 && counter!=0) Serial.print(counter);

}// end loop
// new I2C 1Wire bridge states
bool check_states(){
  //state++;
  int count =0;
  do{
    //delayMicroseconds(10);
    if(count>=1000) return false;
    rx_state=ds_sm.RXStatus();
    tx_state=ds_sm.TXStatus();
    count++;
  } while(rx_state!=0 || tx_state!=0);
  return true;
/*  if(count >1){ 
    Serial.print(count);
    Serial.print("\n");
    Serial.print(state);
    Serial.print("\n");
  }
  */
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
  for (int j=0; j<9; j++){
    Serial.println(data_1wire[j]);
  }
  Serial.print("crc is: ");
  Serial.println(ds_sm.crc8(data_1wire,8));
}
// this was the way in which i stupidly tried to set the readptr to the SR or not. 
void wirestatus_setup(bool value){
  ds_sm.wireReadStatusSetup(value);
}
// to check the 1wire busy bit in the status regsiter of the DS2482. 

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
  ds_sm.configureSetup(DS2482_CONFIG_APU); //we dont want active pullup because ?
  check_states();
  ds_sm.configureRequest();
  check_states();
  uint8_t config_status=ds_sm.configureBool();
  check_states();
  if(config_status!=1) return true;
  else return false;
  //if configure does not return correctly, reset and try again.
}
void debug(){
  Serial.print("debug ");
//  Serial.print(Onewire_state);
  Serial.print("\n");
}
void debug_wirestatus(){
  Serial.print("wirestatus ");
//  Serial.print(wirestatus);
  Serial.print("\n");
}


bool check1WB_again(){
  int loopCount = 1000;
  uint8_t status_read;
  do{
    delayMicroseconds(20);
    if(check_states()) wire_onewirebus_obj->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
    else {
      i2ctimeouts++;
      return false;
    }
    if(check_states()) status_read=wire_onewirebus_obj->read();
    else {
      i2ctimeouts++;
      return false;
    }
    if(check_states()){
      if (--loopCount <= 0){
        val=status_read;
        return false;
      }
    }
    else {
      i2ctimeouts++;
      return false;
    }
  }while(status_read & 0x01);
  return true;
}

bool check1WB_4eva(){
  int loopCount = 10000000;
  uint8_t status_read;
  do{
    delayMicroseconds(20);
    if(check_states()) wire_onewirebus_obj->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
    else return false;
    if(check_states()) status_read=wire_onewirebus_obj->read();
    else return false;
    if(check_states()){
      if (--loopCount <= 0){
        val=status_read;
        return false;
      }
    }
    else return false;
  }while(status_read & 0x01);
  return true;
}

bool checkppd_again(){
  int loopCount = 1000;
  uint8_t status_read;
  do{
    delayMicroseconds(20);
    if(check_states()) wire_onewirebus_obj->requestFrom_nonblocking(mAddress,(uint8_t)1,1);
    else return false;
    if(check_states()) status_read=wire_onewirebus_obj->read();
    else return false;
    if(check_states()){
      if (--loopCount <= 0){
        val=status_read;
        return false;
      }
    }
    else return false;
  }while(status_read & 0x02);
  return true;
}

void print_SR(){
  Serial.print("SR: ");
  check_states();
  ds_sm.wireReadStatusSetup(false); // if status is false then this just checks the TX of I2C since its about to read, if its true it writes to set the read ptr to status reg.
  check_states();
  ds_sm.wireReadStatusRequest();
  check_states();
  Serial.print(ds_sm.wireReadStatusRead());
}
void write_byte_to_1wire(uint8_t b){
  wire_onewirebus_obj->beginTransmission(mAddress);
  wire_onewirebus_obj->write(0xA5); // to the 1wire device write the next byte to it.
  wire_onewirebus_obj->write(b); // to the 1wire device write the next byte to it.
  wire_onewirebus_obj->endTransmission_nonblocking(1);
}
