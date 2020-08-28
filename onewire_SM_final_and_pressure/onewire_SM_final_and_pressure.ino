#include "Pressure.h"
#include "Wire_nonblocking.h"
#include "DS2482_nonblocking_v3.h"

TwoWire *wire_onewirebus_obj= new TwoWire(1);
// i2C object for the i2c port on the launchpad
DS2482 ds(0,*wire_onewirebus_obj); // OneWire bridge object for sending commands to the OneWire bridge 
//DS2482 ds(0,);
bool Onewire_conn;
byte addr[8];
byte addr1[8]={0x28,0x7B,0xAA,0x79,0x97,0x06,0x03,0xD1};
byte addr2[8]={0x28,0x0E,0xA4,0x79,0x97,0x06,0x03,0x00};
uint8_t data[9];
int num_reads;
#define SENSOR_DELAY_PERIOD 100
unsigned long sensorDelayTime;
#define SENSOR_IDLE_PERIOD 3000
unsigned long sensorIdleTime;
#define SENSOR_RESETDELAY_PERIOD 100
unsigned long sensorResetDelayTime;
uint8_t address1[8]={0x28,0x7B,0xAA,0x79,0x97,0x06,0x03,0xD1};
//uint8_t address2[8]={0x28,0x6A,0xC6,0x79,0x97,0x06,0x03,0x8D};
uint8_t address2[8]={0x28,0x0E,0xA4,0x79,0x97,0x06,0x03,0x00};
typedef enum {
  SENSOR_STATE_RESET=0,
  SENSOR_STATE_CONFIG=1,
  SENSOR_STATE_SELECTCHANNEL=2,
  SENSOR_STATE_WIRERESET1=3,
  SENSOR_STATE_WIRESELECT1=4,
  SENSOR_STATE_WIREWRITE1=5,
  SENSOR_STATE_DELAY=6,
  SENSOR_STATE_WIRERESET2=7,
  SENSOR_STATE_WIRESELECT2=8,
  SENSOR_STATE_WIREWRITE2=9,
  SENSOR_STATE_READ=10,
  SENSOR_STATE_ASSESS=11,
  SENSOR_STATE_IDLE=12
/*  SENSOR_STATE_RESETDELAY=1,
  SENSOR_STATE_CONFIG=2,
  SENSOR_STATE_SELECTCHANNEL=3,
  SENSOR_STATE_WIRERESET1=4,
  SENSOR_STATE_WIRESELECT1=5,
  SENSOR_STATE_WIREWRITE1=6,
  SENSOR_STATE_DELAY=7,
  SENSOR_STATE_WIRERESET2=8,
  SENSOR_STATE_WIRESELECT2=9,
  SENSOR_STATE_WIREWRITE2=10,
  SENSOR_STATE_READ=11,
  SENSOR_STATE_ASSESS=12,
  SENSOR_STATE_IDLE=13
*/
} SensorState;
SensorState ONEWIRE_STATE=SENSOR_STATE_RESET;
int tracker=0;

////PRESURE/////
Pressure pressure;
char response[200]={0};
int baud_pressure =19200;

uint8_t unit=2;
char pressureBuffer;
#define PRESSURE_UPDATE_PERIOD 3000  // period between reads/writes
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


void setup(){
  Serial.begin(1000000);
  Serial.print("Starting...");
  wire_onewirebus_obj->begin();
  pressure.begin(Serial1,baud_pressure);
  pressure.reset_string();
  PressureUpdateTime=millis() + PRESSURE_UPDATE_PERIOD;

}

void loop(){
 /* switch (Pressure_State){
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
*/

  if ((long) (millis() - PressureUpdateTime) > 0) {  // time to request
    if(pressure.is_available()){
      // read the pressure duh
      pressure.read_port_again();
      pres=pressure.give_me_the_float();
    }
    else {
      // request the pressure duh
      pressure.poll();
      pressure.read_port_again();
      pres=pressure.give_me_the_float();
    }
    //
    //pressure.just_give_me_the_response(response);
    //Serial.print(response);
    Serial.print(pres);
    PressureUpdateTime=millis() + PRESSURE_UPDATE_PERIOD;
  }
  ds.CheckStates();
  switch(ONEWIRE_STATE){
    case SENSOR_STATE_RESET: {
      if(ds.reset()) {
        ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
        //sensorResetDelayTime= millis() + SENSOR_RESETDELAY_PERIOD;
      }
      else { 
        ONEWIRE_STATE=SENSOR_STATE_IDLE;
        sensorIdleTime = millis() + SENSOR_IDLE_PERIOD;
      }
      break;
    }
/*    case SENSOR_STATE_RESETDELAY: {
      if ((long) (millis() - sensorResetDelayTime) > 0) {
        ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
        break;
      }
      else break;
    }
*/
    case SENSOR_STATE_CONFIG: {
      if(ds.configure(DS2482_CONFIG_APU)) ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      else ONEWIRE_STATE=SENSOR_STATE_IDLE;
      break;
    }
    case SENSOR_STATE_SELECTCHANNEL: {
      if(ds.selectChannel(4)) ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      else ONEWIRE_STATE=SENSOR_STATE_IDLE;
      break;
    }
    case SENSOR_STATE_WIRERESET1: {
      if(ds.wireReset()) ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      else ONEWIRE_STATE=SENSOR_STATE_IDLE;
      break;
    }
    case SENSOR_STATE_WIRESELECT1: {
//      byte addr[8];
      if(tracker==0) memcpy(&addr,(uint8_t *) &addr1, sizeof(addr1));
      else memcpy(&addr,(uint8_t *) &addr2, sizeof(addr2));
      ds.wireSelect(addr);
      ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      break;
    }
    case SENSOR_STATE_WIREWRITE1: {
      ds.wireWriteByte(0x44);
      ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      sensorDelayTime = millis() + SENSOR_DELAY_PERIOD; 
      break;
    }
    case SENSOR_STATE_DELAY: {
      if ((long) (millis() - sensorDelayTime) > 0) {
        ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      }
      else break;
    }
    case SENSOR_STATE_WIRERESET2: {
      if(ds.wireReset()) ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      else ONEWIRE_STATE=SENSOR_STATE_IDLE;
      break;
    }
    case SENSOR_STATE_WIRESELECT2: {
//      byte addr[8];
      if(tracker==0) memcpy(&addr,(uint8_t *) &addr1, sizeof(addr1));
      else memcpy(&addr,(uint8_t *) &addr2, sizeof(addr2));
      ds.wireSelect(addr);
      ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      break;
    }
    case SENSOR_STATE_WIREWRITE2: {
      ds.wireWriteByte(0xBE);
      ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1); 
      break;
    }
    case SENSOR_STATE_READ:{
      for (int i=0;i<9;i++){
      //if(!ds.hasTimeout()) data[i]=ds.wireReadByte();// if no timeout read a byte
      //else return 0;
        data[i]=ds.wireReadByte();
      }
      ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      break;
    }
    case SENSOR_STATE_ASSESS:{
      assess();
      ONEWIRE_STATE=(SensorState) ((unsigned char) ONEWIRE_STATE + 1);
      sensorIdleTime = millis() + SENSOR_IDLE_PERIOD;
      tracker++;
      if(tracker>=2) tracker=0;
      break;
    }
    case SENSOR_STATE_IDLE:{
      if ((long) (millis() - sensorIdleTime) > 0) {
        ONEWIRE_STATE=SENSOR_STATE_RESET;
      }
      else break;
    }
  }

  //if(ONEWIRE_STATE!=SENSOR_STATE_RESETDELAY) Serial.println("p");
  //Serial.println(ONEWIRE_STATE);
  //Serial.println(OneWireReadOneChannel());
  //delay(2000);
  //if(ds.hasTimeout()){
  //  OneWireSetup();    
  //}
}

void assess(){
  if(ds.crc8(data,8)==data[8]){
    float celsius=0;
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    celsius = (float)raw / 16.0;
    for (int j=0; j<9; j++){
      Serial.print(data[j]);
      Serial.print(",");
    }
    Serial.print("crc is: ");
    Serial.println(ds.crc8(data,8));
    Serial.println(celsius);
    Serial.println(tracker);
  }
  else Serial.print("X\n");
}

bool OneWireSetup(){
  ds.reset();  // returns true if it i2c communicated successfully.
    if (!ds.configure(DS2482_CONFIG_APU)) {
      return false; 
    } 
    else {
      return true;
    }
  //configure DS2482 to use active pull-up instead of pull-up resistor 
  //configure returns 0 if it cannot find DS2482 connected 
}

float OneWireReadOneChannel(){
    float celsius;
  // need to select the channel of the onewirebridge device before doing any reads.
  ds.selectChannel(4); // returns true if select channel was executed successfully.
  //  byte addr[8]={0x28,0x7B,0xAA,0x79,0x97,0x06,0x03,0xD1};
  if (ds.wireSearch(addr)){
    memcpy(&addr,(uint8_t *) &addr1, sizeof(addr));
  }

    ds.wireReset();
    ds.wireSelect(addr);
    ds.wireWriteByte(0x44);
//  delay(100);       // maybe 750ms is enough, maybe not
    ds.wireReset();
    ds.wireSelect(addr);
    ds.wireWriteByte(0xBE);
    // once we get here, start assessing the integrity, after each readbyte check for timeout?
    for (int i=0;i<9;i++){
      //if(!ds.hasTimeout()) data[i]=ds.wireReadByte();// if no timeout read a byte
      //else return 0;
      data[i]=ds.wireReadByte();
    }
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    celsius = (float)raw / 16.0;
  //  for (int j=0; j<9; j++){
    //  Serial.println(data[j]);
   // }
    Serial.println(data[8]);
    Serial.print("crc is: ");
    Serial.println(ds.crc8(data,8));    

  return celsius;
}

float ReadOneWireTemp(){
    float celsius;

  // need to select the channel of the onewirebridge device before doing any reads.
  ds.selectChannel(4); // returns true if select channel was executed successfully.
    byte addr[8]={0x28,0x7B,0xAA,0x79,0x97,0x06,0x03,0xD1};
//  if (ds.wireSearch(addr)){
    memcpy(&addr,(uint8_t *) &addr1, sizeof(addr1));
//  }
    ds.wireReset();
    ds.wireSelect(addr);
    ds.wireWriteByte(0x44);
//  delay(100);       // maybe 750ms is enough, maybe not
    ds.wireReset();
    ds.wireSelect(addr);
    ds.wireWriteByte(0xBE);  
    for (int i=0;i<9;i++){
      data[i]=ds.wireReadByte();
    }
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    celsius = (float)raw / 16.0;
    for (int j=0; j<9; j++){
      Serial.println(data[j]);
    }
    Serial.print("crc is: ");
    Serial.println(ds.crc8(data,8));    

  return celsius;
}
