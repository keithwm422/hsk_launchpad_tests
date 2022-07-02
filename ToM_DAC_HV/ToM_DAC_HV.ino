
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
#include "src/libs/LTC2655.h"  // for the src/libs/DAC_code.h

#define DOWNBAUD 115200 // Baudrate to the SFC

const int port_HV=1;
TwoWire * wire_HV= new TwoWire(port_HV);


// for Launchpad LED
#define LED GREEN_LED
// int LED = 15;
#define LED_UPDATE_PERIOD 4050
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;

int clear_buffers_with_this=0;
LTC2655 HVDAC;
uint8_t val1=0;
uint8_t val2=0;
uint16_t step_voltage;

uint16_t voltage_potential=0;
uint16_t voltage_cathode=0;
uint16_t current_potential=0;
uint16_t current_cathode=0;
int EN_HV=18;
bool is_cathode_disabled=true;
bool is_potential_disabled=true;

char receive[20]; // big anough to store things
int write_val=0;
int char_iter;
bool new_write=false;
int length_sent=0;

float conversion_factor=3.3*1406.0/(1000.0*4096.0);
double delta_V_DAC=5.0/4096.0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(DOWNBAUD);
  while(clear_buffers_with_this!=-1){
    clear_buffers_with_this=Serial.read();
  }
  Serial.print("start");
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);

  pinMode(EN_HV,OUTPUT);
  // by default start disabled:
  digitalWrite(EN_HV,LOW);

  wire_HV->begin();
  HVDAC.attach(*wire_HV,0x10);
  //write_initial();
}

void loop() {
  // put your main code here, to run repeatedly: 
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
    // if "P" for Potential HV 
    else if(receive[0]=='E') EN_CHV(); // for CATHODE ENABLE PIN
    else if(receive[0]=='D') DIS_CHV(); // for DISABLING CATHODE
    else if(receive[0]=='Y') EN_PHV(); // for Potential ENABLE PIN
    else if(receive[0]=='Z') DIS_PHV(); // for DISABLING Potential
    else if(receive[0]=='P') write_HV_d(2); // for VPGM of Potential
    else if(receive[0]=='I') write_HV_d(3); // for Ilim of potnetial
    else if(receive[0]=='J') write_HV_d(4); // for Ilim of Potential
    else {
      Serial.println("UNKNOWN COMMAND");
      new_write=false;
    }
    new_write=false;
    for(int j=0;j<20;j++){
      receive[j]=0;
    }    
  }

  
  
  
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
    val1+=1;
    //step_voltage+=100;
    if(val1>=255){
      val2++;
      val1=0;
    }
    if(val2>=255) val2=0;
    write_step(val1,val2);
    if(step_voltage>=4096){
      step_voltage=0;
    }
    //HVDAC.analogWrite(step_voltage,0);
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

void write_initial(){
  wire_HV->beginTransmission(0x10);
  Serial.println((0x06 << 4) | 0x0F,HEX); // first the command with the address
  wire_HV->write(0x6F);
  wire_HV->write(0);
  wire_HV->write(0);
  wire_HV->endTransmission();  
}

void write_step(uint8_t first, uint8_t second){
  wire_HV->beginTransmission(0x10);
  wire_HV->write(0x31); // 0x30 is ch 0 0x31 is ch1
  wire_HV->write(first);
  wire_HV->write(second);
  wire_HV->endTransmission();  
}

void EN_CHV(){
  Serial.println("Cathode HV enabled");
  digitalWrite(EN_HV,HIGH);
  is_cathode_disabled=false;
}
void EN_PHV(){
  Serial.println("Potential HV enabled");
  digitalWrite(EN_HV,HIGH);
  is_potential_disabled=false;
}
void DIS_CHV(){
  Serial.println("Cathode HV disabled");
  digitalWrite(EN_HV,LOW);
  is_cathode_disabled=true;
}
void DIS_PHV(){
  Serial.println("Potential HV disabled");
  digitalWrite(EN_HV,LOW);
  is_potential_disabled=true;
}


// as doubles of microamps or Volts
void write_the_value_to_HV_C_d(double write_val_d){
  double voltage_on_programming_pin=write_val_d*4.64/10000.0; // this is for HV, different for current limit. Now this is in volts
  // get the DAC to set to that value;
  if(voltage_on_programming_pin<=4.65){
    double DAC_value_d=voltage_on_programming_pin/delta_V_DAC;
    voltage_cathode=lround(DAC_value_d);
    //CATChannelProgram(voltage_cathode, 1);
    HVDAC.analogWrite(voltage_cathode,0);
    Serial.println("Cathode Wire Voltage changed");
  }
}
void write_the_value_to_Cat_Ilim_d(double write_val_d){
  double voltage_on_programming_pin=write_val_d*4.64/1500.0; // this is for current limit, different for HV. Now this is in volts
  // get the DAC to set to that value;
  if(voltage_on_programming_pin<=4.65){
    double DAC_value_d=voltage_on_programming_pin/delta_V_DAC;
    current_cathode=lround(DAC_value_d);
    //CATChannelProgram(current_cathode, 0);
    // we put the simple shit instead
    HVDAC.analogWrite(current_cathode,1);
    Serial.println("Cathode Wire Ilim changed");
  }
}

void write_the_value_to_HV_P_d(double write_val_d){
  double voltage_on_programming_pin=write_val_d*4.64/10000.0; // this is for HV, different for current limit. Now this is in volts
  // get the DAC to set to that value;
  if(voltage_on_programming_pin<=4.65){
    double DAC_value_d=voltage_on_programming_pin/delta_V_DAC;
    voltage_potential=lround(DAC_value_d);
    //POTChannelProgram(voltage_potential, 2);
    HVDAC.analogWrite(voltage_potential,2);
    Serial.println("Potential Wire Voltage changed");
  }
}
void write_the_value_to_Pot_Ilim_d(double write_val_d){
  double voltage_on_programming_pin=write_val_d*4.64/1500.0; // this is for current limit, different for HV. Now this is in volts
  // get the DAC to set to that value;
  if(voltage_on_programming_pin<=4.65){
    double DAC_value_d=voltage_on_programming_pin/delta_V_DAC;
    current_potential=lround(DAC_value_d);    
    //POTChannelProgram(current_potential, 3);
    HVDAC.analogWrite(current_potential,3);
    Serial.println("Potential Wire Ilim changed");
  }
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
