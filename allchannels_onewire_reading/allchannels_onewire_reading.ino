// Note: May not be compatible with regular_onewire_reading.ino in the same folder

#include "src/MainHSK_lib/MainHSK_support_functions.h"

// I2C 1 wire bridge states
unsigned long i2c_onewire = 2;
// i2C object for the i2c port on the launchpad
TwoWire *wire_onewirebus_obj= new TwoWire(i2c_onewire);
// OneWire bridge object for sending commands to the OneWire bridge
DS2482 ds_sm(0,*wire_onewirebus_obj);
bool Onewire_conn;
bool onewireSwitch;

// number of sensors per channel
#define numMaxTempSensors 30
#define numChannels 8
#define numAddrBytes 8
byte allFoundAddr[numChannels][numMaxTempSensors][numAddrBytes];
int numFoundTempSensors[numChannels];

// temp reading storage
// float celsiusTempArray[numChannels][numMaxTempSensors]={0};
float celsiusTemp;
// float internal_temperature=0;

#define SENSOR_IDLE_PERIOD 100
unsigned long sensorIdleTime;
#define CHANNEL_IDLE_PERIOD 1000

// LED
bool led_is_high=true;
#define LED_PIN RED_LED
#define LED_IDLE_PERIOD 100
unsigned long LEDIdleTime;

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


// setup code; this runs once
void setup() {

  // LED set
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

  // open comm
  Serial.begin(115200);
  Serial.println("Starting...");
  
  // I2C(1) is one wire bridge chip and you can find that i2c setup in the DS2482.h/cpp files and MainHSK_support_functions.cpp
  // original way of doing 1 wire device read
  wire_onewirebus_obj->setModule(i2c_onewire);
  Onewire_conn=OneWireSetup(0, ds_sm);

  Serial.println("All found addresses:");

  // print all found addresses per channel
  for (int activeChan=0; activeChan<numChannels; activeChan++){
  
    // find all device addresses
    Serial.print("Channel ");
    Serial.println(activeChan);
    
    numFoundTempSensors[activeChan] = numMaxTempSensors;
    int activeSensor = 0;
    while(activeSensor<numFoundTempSensors[activeChan]){
      
      OneWireCopyAddress(allFoundAddr[activeChan][activeSensor], activeChan, activeSensor);
      
      // print found address and determine if we found all sensors
      int noMoreSensors = 0; // stays 0 if a 0 address is found aka no more sensors detected
      for(int addrByte=0;addrByte<8;addrByte++){
        Serial.print(allFoundAddr[activeChan][activeSensor][addrByte],HEX);
        Serial.print(",");
        noMoreSensors = noMoreSensors || allFoundAddr[activeChan][activeSensor][addrByte];
      }
      Serial.println();
      if (noMoreSensors == 0) {
        numFoundTempSensors[activeChan] = activeSensor;
        Serial.print(numFoundTempSensors[activeChan]);
        Serial.print(" sensors found for channel ");
        Serial.println(activeChan);
      }
      activeSensor++;
    }
  }
  
  Serial.println("Finished. Beginning temperature readings...");
  
}


// main code; runs repeatedly
void loop() {

  // LED on feedback
  switch_LED();
  delay(LED_IDLE_PERIOD/2);
  switch_LED();
  delay(LED_IDLE_PERIOD/2);
  switch_LED();
  delay(LED_IDLE_PERIOD/2);
  switch_LED();

  // go through each channel
  // int channels[] = {0,2,3,...};
  // for (int activeChan : channels) {   // use for specific channel reading instead of all
  for (int activeChan=0; activeChan<numChannels; activeChan++) {
    delay(CHANNEL_IDLE_PERIOD);

    // An additional LED blink feedback
    switch_LED();
    delay(LED_IDLE_PERIOD);
    switch_LED();

    Serial.print("Channel ");
    Serial.print(activeChan);
    Serial.println(":");

    // read temperatures
    for (int activeSensor=0; activeSensor<numFoundTempSensors[activeChan]; activeSensor++){
      delay(SENSOR_IDLE_PERIOD);
  
      // print address
      for(int addrByte=0; addrByte<8; addrByte++){
        Serial.print(allFoundAddr[activeChan][activeSensor][addrByte],HEX);
        Serial.print(",");
      }
      Serial.print(": ");

      // print temperature
      celsiusTemp = OneWireReadOneChannel(activeChan, activeSensor, ds_sm);
      Serial.print(celsiusTemp,4);
      Serial.println(";");
      
    }
  }
  
  Serial.println("--------- End ---------");
  
}
