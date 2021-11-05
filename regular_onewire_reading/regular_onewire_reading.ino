#include "src/MainHSK_lib/MainHSK_support_functions.h"

// I2C 1 wire bridge states
unsigned long i2c_onewire=1;
TwoWire *wire_onewirebus_obj= new TwoWire(i2c_onewire); // i2C object for the i2c port on the launchpad
// i2C object for the i2c port on the launchpad
DS2482 ds_sm(0,*wire_onewirebus_obj); // OneWire bridge object for sending commands to the OneWire bridge
bool Onewire_conn;
#define NUM_TEMP_SENSORS 4
byte addr[NUM_TEMP_SENSORS][8];
byte addr1[8]={0x28,0x7B,0xAA,0x79,0x97,0x06,0x03,0xD1}; // temp addresses at OSU
byte addr2[8]={0x28,0x6A,0xC6,0x79,0x97,0x06,0x03,0x8D};
byte addr3[8]={0x28,0x0E,0xA4,0x79,0x97,0x06,0x03,0x00};
byte addr4[8]={0x28,0x75,0xBC,0x79,0x97,0x06,0x03,0x22};
/*byte addr1[8]={0x28,0xB8,0x94,0x79,0x97,0x06,0x03,0x31}; // temp addresses at UC
byte addr2[8]={0x28,0xBB,0xC3,0x79,0x97,0x06,0x03,0xAC};
byte addr3[8]={0x28,0x99,0x87,0x79,0x97,0x06,0x03,0x82}; */
int num_reads;
// for debugging 1wire temps
uint8_t addr1_found[8];
uint8_t addr2_found[8];
uint8_t addr3_found[8];
uint8_t addr4_found[8];
int search_num=0;
#define SENSOR_IDLE_PERIOD 30000
unsigned long sensorIdleTime;
int tracker=0;
uint8_t chan_1_wire=4;
float celsius[NUM_TEMP_SENSORS]={0};
// LED
bool led_is_high=true;
int LED_PIN=38;
//#define LED_PIN RED_LED

#define LED_IDLE_PERIOD 2000
unsigned long LEDIdleTime;
int clear_buffers_with_this=0;


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN,OUTPUT); //LED
  digitalWrite(LED_PIN,LOW);
  Serial.begin(115200);
  Serial.print("starting");
 // I2C(1) is one wire bridge chip and you can find that i2c setup in the DS2482.h/cpp files and MainHSK_support_functions.cpp
  // original way of doing 1 wire device read
  wire_onewirebus_obj->setModule(i2c_onewire);  
  Onewire_conn=OneWireSetup(chan_1_wire, ds_sm);
  copy_temp_addresses(); // copy temp probe address over
  
}

void loop() {
  // put your main code here, to run repeatedly: 
    if ((long) (millis() - LEDIdleTime) > 0) {
    LEDIdleTime = millis() + LED_IDLE_PERIOD;
    switch_LED();
  }
  if ((long) (millis() - sensorIdleTime) > 0) {
    sensorIdleTime = millis() + SENSOR_IDLE_PERIOD;
    celsius[tracker]=OneWireReadOneChannel(chan_1_wire,tracker, ds_sm);
    Serial.print(celsius[tracker],4);
    tracker++;
    if(tracker>=NUM_TEMP_SENSORS) tracker=0;
  }
}

void switch_LED(){
  if(led_is_high){
    led_is_high=false;
    digitalWrite(LED_PIN,LOW);
  }
  else{    
    led_is_high=true;
    digitalWrite(LED_PIN,HIGH);
  }
}
// 1Wire temperature probe related code. 
void copy_temp_addresses(){
  // seems to need this type of format in order to memcpy successfully?
  memcpy(&addr[0],(uint8_t *) &addr1, sizeof(addr1));
  memcpy(&addr[1],(uint8_t *) &addr2, sizeof(addr2));
  memcpy(&addr[2],(uint8_t *) &addr3, sizeof(addr3));
  memcpy(&addr[3],(uint8_t *) &addr4, sizeof(addr4));
}
