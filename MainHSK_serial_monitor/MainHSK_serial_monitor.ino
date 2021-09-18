
#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>
//#include <Hsk_all_data_types.h>
#include <file_test_head.h>
using namespace MainHSK_cmd;
#define DOWNBAUD 115200 // Baudrate for serial monitor

#include "src/MainHsk_IU_tests_lib/Flowmeters.h"
/* structs for storing data from this devices sensors */
Flowmeters flow_1;
double gasdata[6]={0};
char gastype[100];
char errorcode[100];
int baud_flows =19200;
const char * flow_ID_1= "C";
sMainFlows mainflows[3];
sMainFlowsGasString mainflowsgasstring;
char gas1[100];
typedef enum {
  Flowmeters_Idle=0,
  Flowmeters_Request=1,
  Flowmeters_Available=2,
  Flowmeters_Read=3,
  Flowmeters_Read_Complete=4,
} Flowmeters_State_t;
Flowmeters_State_t Flowmeters_State=Flowmeters_Idle;
int avail_wait_tries=0;
bool parse_worked;
#define FLOWMETERS_UPDATE_PERIOD 10000  // period between reads/writes
unsigned long FlowmetersUpdateTime=0;  // for transmit recording time
char flowmetersBuffer;
char flowmetersReadArray[100]={0};
char id;
float set2{0};
char  gas[32] = "";
char  stat1[32] = "";
char  stat2[32] = "";
int flowmeters_read_index=0;
const char err_msg[20]="NO READ AVAIL.";   
//double mainflows_a[6]={0}; // to store flowmeter 1 values in mainflows structs
bool is_high=true;
int LED=38;
unsigned long LED_time=0;
#define LED_PERIOD 1000
void setup() {
  Serial.begin(DOWNBAUD);

  //Flowmeters initializations, flow_1 object instance (should be one instance per flowmeter)
  // Serial connections: 4,6, and 7. 
  flow_1.begin(Serial4,baud_flows, flow_ID_1);
  FlowmetersUpdateTime = millis() + FLOWMETERS_UPDATE_PERIOD;
  Serial.println("Starting...");
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
  LED_time=millis();
  while(flow_1.available()){ // this will just clear the buffer really.
    flowmetersBuffer=flow_1.read_byte();
    Serial.print(flowmetersBuffer);
  }
  delay(1000);
}

void loop() {
  //  i2c one wire bridge conn
  if ((long) (millis() - LED_time) > 0) {
    LED_time = millis() + LED_PERIOD;
    switch_LED();

  }  
  // read in the flows
  switch(Flowmeters_State){
    case Flowmeters_Idle:{
      if(flow_1.available()){ // this will just clear the buffer really.
        flowmetersBuffer=(char)flow_1.read_byte();
        break;
      }
      else {  // nothing to read so check to see if time to request a read
        if ((long) (millis() - FlowmetersUpdateTime) > 0) {
          FlowmetersUpdateTime = millis() + FLOWMETERS_UPDATE_PERIOD;
          Flowmeters_State=(Flowmeters_State_t) ((unsigned char) Flowmeters_State + 1);
          break;
        }
        else break;
      }
    }
    case Flowmeters_Request:{
      flow_1.poll();
      Flowmeters_State=(Flowmeters_State_t) ((unsigned char) Flowmeters_State + 1);
      flowmeters_read_index=0;
      avail_wait_tries=0;
      parse_worked=false;
      break;
    }
    case Flowmeters_Available:{
      // setup the chars and the arrays for this flowmeter (probably need to clear something)
      flow_1.read_setup();
      Flowmeters_State=(Flowmeters_State_t) ((unsigned char) Flowmeters_State + 1);
      avail_wait_tries=0;
      break;
    }
    case Flowmeters_Read:{
      int read_long_val=flow_1.read_byte();
      if(read_long_val!=-1){
        flowmetersReadArray[flowmeters_read_index]=(char) read_long_val;
        flowmeters_read_index++;
        avail_wait_tries=0; //reset the wait times before checking again?
        break;
      }
      else{
        avail_wait_tries++;
        if(avail_wait_tries>=1000) Flowmeters_State=(Flowmeters_State_t) ((unsigned char) Flowmeters_State + 1);
        break;
      }
    }
    case Flowmeters_Read_Complete:{
      parse_worked=flow_1.getGasData(gasdata,gastype);
      if(parse_worked){
        mainflows[0].pressure=gasdata[0];
        mainflows[0].temperature=gasdata[1];
        mainflows[0].volume=gasdata[2];
        mainflows[0].mass=gasdata[3];
        mainflows[0].setpoint=gasdata[4];
        set2=gasdata[5];
        memcpy((uint8_t*) gas,(uint8_t *) gastype,sizeof(gas));
        print_packet();
      }
      Flowmeters_State=Flowmeters_Idle;
      break;
    }
  }  
}

void print_packet(){
  Serial.println("Flowmeter data: ");
  Serial.print("Pressure ");
  Serial.print(mainflows[0].pressure,4);
  Serial.print(", Temperature ");
  Serial.print(mainflows[0].temperature,4);
  Serial.print(", Vol Flow ");
  Serial.print(mainflows[0].volume,4);
  Serial.print(", Mass Flow ");
  Serial.print(mainflows[0].mass,4);
  Serial.print(", Setpoint ");
  Serial.print(mainflows[0].setpoint,4);
  Serial.println("  done  ");
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
