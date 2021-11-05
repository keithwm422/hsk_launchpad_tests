#include "HardwareSerial_v2.h"
char packet[20]="ABCDEFGHIJKLMNOP";
char buffer[64];
int buff;
bool is_high=true;
bool is_high_2=true;
int LED_PIN=40;
int LED_PIN_2=39;
unsigned long LED_time=0;
#define LED_PERIOD 1000
#define SENSOR_IDLE_PERIOD 10000
unsigned long sensorIdleTime;
unsigned long start_time;
unsigned long end_time; 
bool new_poll=false;
int num_negs=0;
unsigned long time_to_enable_SFCUART=0;
#define ENABLE_SFCUART_PERIOD 100000
bool SFCUART_disabled=false;
#define PACKET_UPDATE_PERIOD 10000
unsigned long PacketUpdateTime=0; // unprompted packet timer
void setup() {
  vSerial.begin(115200);
  vSerial.write("S");
  //vSerial7.begin(19200);
  //buff=Serial7.read();
  //vUARTIntHandler();
  //vSerial.print(buff);
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,HIGH);
  //pinMode(LED_PIN_2,OUTPUT);
  //digitalWrite(LED_PIN_2,HIGH);
  sensorIdleTime = millis() + SENSOR_IDLE_PERIOD;
  LED_time = millis() + LED_PERIOD;
  SFCUART_disabled=false;
  PacketUpdateTime= millis() + PACKET_UPDATE_PERIOD;

}

void loop() {
 /* if(new_poll){
    buff=vSerial7.read();
    if(buff!=-1) {
      vSerial.print(" num negative returns are: ");
      vSerial.println(num_negs);
      num_negs=0;
      vSerial.print(" non neg returns are: ");
      vSerial.print((char) buff);
      if(buff=='A') start_time=millis();
      else if(buff=='O') end_time=millis();
    }
    else{ 
      num_negs++;
      if(num_negs>=10000) new_poll=false;
    }
  }*/
  //vUARTIntHandler();
  if(vSerial.turn_off_TX){
    time_to_enable_SFCUART=millis()+ENABLE_SFCUART_PERIOD;
    SFCUART_disabled=true;
  }
  if(SFCUART_disabled){
    if((long)(millis()-time_to_enable_SFCUART)>0){
      SFCUART_disabled=false;
      //vSerial.turn_back_on_UART();
      //vSerial.begin(115200);
    }
  }
  if ((long) (millis() - LED_time) > 0) {
    LED_time = millis() + LED_PERIOD;
    switch_LED();
    //vSerial.print("blah");
  }

    // example send packet unprompted every PACKET_PERIOD
  if(((long)(millis() -PacketUpdateTime) > 0) && !SFCUART_disabled){
    PacketUpdateTime= millis() +PACKET_UPDATE_PERIOD;
    for(int i=0; i<10;i++){
      vSerial.write(packet[i]);
    }
  }
/*    
  // put your main code here, to run repeatedly: 
  if ((long) (millis() - sensorIdleTime) > 0) {
    sensorIdleTime = millis() + SENSOR_IDLE_PERIOD;
    //vSerial.print(flowmetersBuffer);
    //vSerial.print(flow_1.available(),DEC);
    //vSerial7.print("A\r");
    new_poll=true;
    num_negs=0;
    //vSerial.print(start_time);
    //vSerial.print(",");
    //vSerial.print(end_time);    
    switch_LED_2();
  }  
*/
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
