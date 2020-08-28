#include "Wire_nonblocking.h"
#include "DS2482_nonblocking_v3.h"

TwoWire *wire_onewirebus_obj= new TwoWire(1);
// i2C object for the i2c port on the launchpad
DS2482 ds(0,*wire_onewirebus_obj); // OneWire bridge object for sending commands to the OneWire bridge 
//DS2482 ds(0,);
bool Onewire_conn;
byte addr_1[8];
uint8_t data[9];
int num_reads;
void setup() 
{ 
  Serial.begin(1000000);
  Serial.print("Starting...");
//  Wire.begin(); 
  wire_onewirebus_obj->begin();
  //ds.reset(); 
  //i2c_onewire=1;
  //Serial.print("P");
  //wire.setModule(i2c_onewire);
  Serial.println(OneWireSetup());
  //Serial.println("Q");
  //configure DS2482 to use active pull-up instead of pull-up resistor 
  //configure returns 0 if it cannot find DS2482 connected 
  //if (!ds.configure(DS2482_CONFIG_APU)) 
  //{ 
  //   Serial.print("DS2482 not found\n"); 
  //} 
  num_reads=0;
}

void loop() 
{ 
  //num_reads+=wire_onewirebus_obj->available();
  //Serial.println("avail: ");
  //Serial.print(num_reads);
  //Serial.print("\n");
  Serial.println(OneWireReadOneChannel());
  delay(2000);
  if(ds.hasTimeout()){
    OneWireSetup();    
  }
}


bool OneWireSetup(){
  ds.reset();  // returns true if it i2c communicated successfully.
    if (!ds.configure(DS2482_CONFIG_APU)) {
      return false; 
    } 
    else {
      return true;
    }
  //configure DS2482 to use active pull-up instead of pull-up resistor 
  //configure returns 0 if it cannot find DS2482 connected 
}

float OneWireReadOneChannel(){
    float celsius;
  // need to select the channel of the onewirebridge device before doing any reads.
  ds.selectChannel(4); // returns true if select channel was executed successfully.
  //  byte addr[8]={0x28,0x7B,0xAA,0x79,0x97,0x06,0x03,0xD1};
  byte addr[8];
  if (ds.wireSearch(addr)){
    memcpy(&addr_1,(uint8_t *) &addr, sizeof(addr));
  }

    ds.wireReset();
    ds.wireSelect(addr_1);
    ds.wireWriteByte(0x44);
//  delay(100);       // maybe 750ms is enough, maybe not
    ds.wireReset();
    ds.wireSelect(addr_1);
    ds.wireWriteByte(0xBE);
    // once we get here, start assessing the integrity, after each readbyte check for timeout?
    for (int i=0;i<9;i++){
      //if(!ds.hasTimeout()) data[i]=ds.wireReadByte();// if no timeout read a byte
      //else return 0;
      data[i]=ds.wireReadByte();
    }
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    celsius = (float)raw / 16.0;
  //  for (int j=0; j<9; j++){
    //  Serial.println(data[j]);
   // }
    Serial.println(data[8]);
    Serial.print("crc is: ");
    Serial.println(ds.crc8(data,8));    

  return celsius;
}

float ReadOneWireTemp(){
    float celsius;

  // need to select the channel of the onewirebridge device before doing any reads.
  ds.selectChannel(4); // returns true if select channel was executed successfully.
    byte addr[8]={0x28,0x7B,0xAA,0x79,0x97,0x06,0x03,0xD1};
//  if (ds.wireSearch(addr)){
    memcpy(&addr_1,(uint8_t *) &addr, sizeof(addr));
//  }
    ds.wireReset();
    ds.wireSelect(addr_1);
    ds.wireWriteByte(0x44);
//  delay(100);       // maybe 750ms is enough, maybe not
    ds.wireReset();
    ds.wireSelect(addr_1);
    ds.wireWriteByte(0xBE);  
    for (int i=0;i<9;i++){
      data[i]=ds.wireReadByte();
    }
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    celsius = (float)raw / 16.0;
    for (int j=0; j<9; j++){
      Serial.println(data[j]);
    }
    Serial.print("crc is: ");
    Serial.println(ds.crc8(data,8));    

  return celsius;
}
