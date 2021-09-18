#include "Wire.h"
#include "MCP4728_cathode.h"
MCP4728_CAT cat_dac;
const int port_cathode=3;
TwoWire * wire_cathode= new TwoWire(port_cathode);
uint8_t cat_LDAC=2; // pin 2 of launchpad is the LDAC pin we use here. 

uint8_t default_address=96;
uint8_t offset_ID=1;
// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");
  pinMode(cat_LDAC,OUTPUT);
  wire_cathode->begin();
  Cathode_setup();
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);

}

void loop() {
  // put your main code here, to run repeatedly: 
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
    cat_dac.readRegisters();
    read_everything();
  }
}

void Cathode_setup(){
  cat_dac.attach(*wire_cathode, cat_LDAC); // set the LDAC pin of launchpad and the i2c comms
  // Setup all DAC channels to have VDD as reference.
  //There are 8 chips each with their own addresses and 4 channels on each chip
  cat_dac.setID(default_address+offset_ID);
  cat_dac.selectVref(MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD);
  cat_dac.analogWrite(4095,4095,4095,4095);
//  cat_dac.readRegisters();
//  for(int j=0;j<4;j++) dac_val=dac_val | cat_dac.getDACData(j);
}

void read_everything(){
  Serial.println("here comes all the bytes");
  for(int i=0;i<4; i++){
    read_channel(i);
  }
}
void read_channel(int channel){
  Serial.print("channel: ");
  Serial.println(channel,DEC);
  Serial.print("getvref: ");
  Serial.print(cat_dac.getVref(channel,false));
  Serial.print(",");
  Serial.print(cat_dac.getVref(channel,true));
  Serial.print("getgain: ");
  Serial.print(cat_dac.getGain(channel,false));
  Serial.print(",");
  Serial.print(cat_dac.getGain(channel,true));
  Serial.print("getDACdata: ");
  Serial.print(cat_dac.getDACData(channel,false));
  Serial.print(",");
  Serial.print(cat_dac.getDACData(channel,true));
  Serial.println("    DONE");

}
void switch_LED(){
  if(is_high){
    is_high=false;
    digitalWrite(LED,LOW);
    digitalWrite(cat_LDAC,HIGH);

  }
  else{    
    is_high=true;
    digitalWrite(LED,HIGH);
    digitalWrite(cat_LDAC,LOW);
  }
}
