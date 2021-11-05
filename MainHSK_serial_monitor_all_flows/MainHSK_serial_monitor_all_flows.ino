
#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>
//#include <Hsk_all_data_types.h>
#include <file_test_head.h>
using namespace MainHSK_cmd;
#define DOWNBAUD 115200 // Baudrate for serial monitor

#include "src/MainHsk_IU_tests_lib/Flowmeters.h"
/* structs for storing data from this devices sensors */
Flowmeters flows[3];
int which_flow=0; // 0 is C, 1 is M, 2 is A
double gasdata[6]={0};
char gastype[100];
char errorcode[100];
int baud_flows =19200;
const char * flow_ID_1= "C"; // order is mix c argon, where mix and c are on top of eachother (mix is top of the double din connector so is serial 7, carbon is serial 4, bottom of dual connector). Argon is single one (serial6). 
const char * flow_ID_2= "M";
const char * flow_ID_3= "A";
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
  flows[0].begin(Serial4,baud_flows, flow_ID_1);
  flows[1].begin(Serial7,baud_flows, flow_ID_2);
  flows[2].begin(Serial6,baud_flows, flow_ID_3);
  FlowmetersUpdateTime = millis() + FLOWMETERS_UPDATE_PERIOD;
  Serial.println("Starting...");
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
  LED_time=millis();
  for(int i=0;i<3;i++){
    while(flows[i].available()){ // this will just clear the buffer really.
      flowmetersBuffer=flows[i].read_byte();
      Serial.print(flowmetersBuffer);
    }
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
      if(flows[which_flow].available()){ // this will just clear the buffer really.
        flowmetersBuffer=(char)flows[which_flow].read_byte();
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
      flows[which_flow].poll();
      Flowmeters_State=(Flowmeters_State_t) ((unsigned char) Flowmeters_State + 1);
      flowmeters_read_index=0;
      avail_wait_tries=0;
      parse_worked=false;
      break;
    }
    case Flowmeters_Available:{
      // setup the chars and the arrays for this flowmeter (probably need to clear something)
      flows[which_flow].read_setup();
      Flowmeters_State=(Flowmeters_State_t) ((unsigned char) Flowmeters_State + 1);
      avail_wait_tries=0;
      break;
    }
    case Flowmeters_Read:{
      int read_long_val=flows[which_flow].read_byte();
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
      parse_worked=flows[which_flow].getGasData(gasdata,gastype);
      if(parse_worked){
        mainflows[which_flow].pressure=gasdata[0];
        mainflows[which_flow].temperature=gasdata[1];
        mainflows[which_flow].volume=gasdata[2];
        mainflows[which_flow].mass=gasdata[3];
        mainflows[which_flow].setpoint=gasdata[4];
        set2=gasdata[5];
        memcpy((uint8_t*) gas,(uint8_t *) gastype,sizeof(gas));
        print_packet();
      }
      Flowmeters_State=Flowmeters_Idle;
      which_flow++;
      if(which_flow>=3) which_flow=0;
      break;
    }
  }  
}

void print_packet(){
  Serial.println("Flowmeter data: ");
  Serial.print("Pressure ");
  Serial.print(mainflows[which_flow].pressure,4);
  Serial.print(", Temperature ");
  Serial.print(mainflows[which_flow].temperature,4);
  Serial.print(", Vol Flow ");
  Serial.print(mainflows[which_flow].volume,4);
  Serial.print(", Mass Flow ");
  Serial.print(mainflows[which_flow].mass,4);
  Serial.print(", Setpoint ");
  Serial.print(mainflows[which_flow].setpoint,4);
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
