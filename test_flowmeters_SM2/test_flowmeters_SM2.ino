
#include <Hsk_all_data_types.h>

#include "Flowmeters.h"

/* structs for storing data from this devices sensors */
Flowmeters flow_1;
double gasdata[6]={0};
char gastype[100];
char errorcode[100];
int baud_flows =19200;
const char * flow_ID_1= "A";
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
bool is_high=true;
bool is_high_2=true;
int LED_PIN=38;
int LED_PIN_2=39;
unsigned long LED_time=0;
#define LED_PERIOD 1000
#define SENSOR_IDLE_PERIOD 4000
unsigned long sensorIdleTime;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting");
  flow_1.begin(Serial7,baud_flows, flow_ID_1);
  FlowmetersUpdateTime = millis() + FLOWMETERS_UPDATE_PERIOD;
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,HIGH);
  pinMode(LED_PIN_2,OUTPUT);
  digitalWrite(LED_PIN_2,HIGH);
  while(flow_1.available()){ // this will just clear the buffer really.
    flowmetersBuffer=flow_1.read_byte();
    Serial.print(flowmetersBuffer);
  }
}

void loop() {
  // put your main code here, to run repeatedly: 
    //  i2c one wire bridge conn
  if ((long) (millis() - sensorIdleTime) > 0) {
    sensorIdleTime = millis() + SENSOR_IDLE_PERIOD;
    //Serial.print(flowmetersBuffer);
    //Serial.print(flow_1.available(),DEC);
    switch_LED_2();
  }  
  // read in the flows
  switch(Flowmeters_State){
    case Flowmeters_Idle:{
      if(flow_1.available()){ // this will just clear the buffer really.
        flowmetersBuffer=(char)flow_1.read_byte();
        //Serial.print(flowmetersBuffer);
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
      Serial.println("Polling");
      Flowmeters_State=(Flowmeters_State_t) ((unsigned char) Flowmeters_State + 1);
      flowmeters_read_index=0;
      avail_wait_tries=0;
      break;
    }
    case Flowmeters_Available:{
        // setup the chars and the arrays for this flowmeter
        flow_1.read_setup();
        Flowmeters_State=(Flowmeters_State_t) ((unsigned char) Flowmeters_State + 1);
        avail_wait_tries++;
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
      Flowmeters_State=Flowmeters_Idle;
      print_packet();
      break;
    }
  }
}
void print_err(){
    Serial.println("Here's an error");
  Serial.print(flowmetersReadArray);
  Serial.print(Flowmeters_State);
}
void print_packet(){
  //parse it
  flow_1.getGasData(gasdata,gastype);
  mainflows[0].pressure=gasdata[0];
  mainflows[0].temperature=gasdata[1];
  mainflows[0].volume=gasdata[2];
  mainflows[0].mass=gasdata[3];
  mainflows[0].setpoint=gasdata[4];
  set2=gasdata[5];
  memcpy((uint8_t*) gas,(uint8_t *) gastype,sizeof(gas));
  Serial.print("Here's a packet ");
  Serial.println(flowmeters_read_index);
  Serial.print(flowmetersReadArray);
  Serial.println(Flowmeters_State,DEC);
//&id,&mainflows[0].pressure , &mainflows[0].temperature, &mainflows[0].volume, &mainflows[0].mass, &mainflows[0].setpoint, &set2, gas, stat1,stat2
  Serial.print(id);
  Serial.print(",");
  Serial.print(mainflows[0].pressure,4);
  Serial.print(",");
  Serial.print(mainflows[0].temperature,4);
  Serial.print(",");
  Serial.print(mainflows[0].volume,4);
  Serial.print(",");
  Serial.print(mainflows[0].mass,4);
  Serial.print(",");
  Serial.print(mainflows[0].setpoint,4);
  Serial.print(",");
  Serial.print(set2,4);
  Serial.print(",");
  Serial.print(gas);
  Serial.print(",");
  Serial.print(stat1);
  Serial.print(",");
  Serial.print(stat2);
  Serial.println("Done");
  
 
}
void switch_LED(){
  if(is_high){
    is_high=false;
    digitalWrite(LED_PIN,LOW);
  }
  else{    
    is_high=true;
    digitalWrite(LED_PIN,HIGH);
  }
}
void switch_LED_2(){
  if(is_high_2){
    is_high_2=false;
    digitalWrite(LED_PIN_2,LOW);
  }
  else{    
    is_high_2=true;
    digitalWrite(LED_PIN_2,HIGH);
  }
}
