// Use to test the pressure gauge at scott's crib
#include "Pressure.h"
Pressure pressure;
char response[200]={0};
int baud_pressure =9600;
int new_baud =115200;

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

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(115200);
  pressure.begin(Serial3,baud_pressure);
  Serial.print("starting...");
  delay(1000);
  pressure.poll_baud();
  while(pressure.is_available()){
    Serial.print(pressure.get_one_byte());
  }
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
}

void loop() {
  // put your main code here, to run repeatedly: 
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
}
