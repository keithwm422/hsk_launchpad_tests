// Use to test the pressure gauge at scott's crib
#include "Pressure.h"
Pressure pressure;
char response[200]={0};
int baud_pressure =115200;

char one_byte;
char helix[8]="HELIX";
uint8_t unit=2;
char pressureBuffer;
#define PRESSURE_UPDATE_PERIOD 5000  // period between reads/writes
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

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("starting...");
  Serial3.begin(baud_pressure);

  //Serial3.begin(baud_pressure);
  //pressure.begin(Serial3,baud_pressure);
  Serial.print("starting...");
  //delay(1000);
  //pressure.poll_baud();
//  pressure.unlock();
//  while(pressure.is_available()){
//    Serial.print(pressure.get_one_byte());
//  }
  
  //pressure.setbaud(new_baud);
  //Serial.println("changed baud");
  //while(pressure.is_available()){
  //  Serial.print(pressure.get_one_byte());
 // }

 // pressure.lock();
//  pressure.read_port_again();
//  print_pressure();
  PressureUpdateTime=millis()+PRESSURE_UPDATE_PERIOD;
}

void loop() {
  // put your main code here, to run repeatedly: 
  if((long) (millis() - PressureUpdateTime) > 0){
    PressureUpdateTime+= PRESSURE_UPDATE_PERIOD;
    Serial.println("x");
    Serial.print("polling");  
    Serial3.print("@253PR4?;FF");
    Serial.println("x");

  }
  uint32_t read_val=Serial3.read();
  //Serial.print((char)read_val);
  if(read_val!=-1)Serial.print((char)read_val);
  //while(pressure.is_available()){
  //  Serial.print(pressure.get_one_byte());
  //}
  

}
