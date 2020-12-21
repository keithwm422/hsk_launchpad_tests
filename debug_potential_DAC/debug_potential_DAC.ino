#include "Wire.h"
#include "MCP4728_cathode.h"
MCP4728_CAT cat_dac;
const int port_cathode=3;
TwoWire * wire_cathode= new TwoWire(port_cathode);
//uint8_t cat_LDAC=14; // pin 2 of launchpad is the LDAC pin we use here. 
uint8_t cat_LDAC=17; // pin 2 of launchpad is the LDAC pin we use here. 

uint8_t default_address=96;

// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 10350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");
  pinMode(cat_LDAC,OUTPUT);
  wire_cathode->begin();
  wire_cathode->flush();
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
    custom_reads();
  }
}

void Cathode_setup(){
  cat_dac.attach(*wire_cathode, cat_LDAC); // set the LDAC pin of launchpad and the i2c comms
  // Setup all DAC channels to have VDD as reference.
  //There are 8 chips each with their own addresses and 4 channels on each chip
  cat_dac.setID(default_address);
  cat_dac.selectVref(MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD);
  cat_dac.analogWrite(0,0,0,0);
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
  Serial.print(cat_dac.getVref(channel,false),DEC);
  Serial.print(",");
  Serial.print(cat_dac.getVref(channel,true),DEC);
  Serial.print("getgain: ");
  Serial.print(cat_dac.getGain(channel,false),DEC);
  Serial.print(",");
  Serial.print(cat_dac.getGain(channel,true),DEC);
  Serial.print("getDACdata: ");
  Serial.print(cat_dac.getDACData(channel,false),DEC);
  Serial.print(",");
  Serial.print(cat_dac.getDACData(channel,true),DEC);
  Serial.println("    DONE");

}

void custom_reads(){
  Serial.println("CUSTOM:");
  wire_cathode->requestFrom((int)default_address, 24);
  while(wire_cathode->available()){
    Serial.print(wire_cathode->read(),HEX);
    Serial.print(",");
  }
}
void switch_LED(){
  if(is_high){
    is_high=false;
    digitalWrite(LED,LOW);
    digitalWrite(cat_LDAC,HIGH);
    Serial.println("HIGH");
    cat_dac.selectVref(MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD, MCP4728_CAT::VREF::VDD);
    cat_dac.analogWrite(4095,4095,4095,4095);
  }
  else{    
    is_high=true;
    digitalWrite(LED,HIGH);
    digitalWrite(cat_LDAC,LOW);
    Serial.println("LOW");
    cat_dac.selectVref(MCP4728_CAT::VREF::INTERNAL_2_8V, MCP4728_CAT::VREF::INTERNAL_2_8V, MCP4728_CAT::VREF::INTERNAL_2_8V, MCP4728_CAT::VREF::INTERNAL_2_8V);
    cat_dac.analogWrite(4095,4095,4095,4095);

  }
}
