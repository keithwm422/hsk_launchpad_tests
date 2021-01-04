char buffer[64];
int buff;
bool is_high=true;
bool is_high_2=true;
int LED_PIN=38;
int LED_PIN_2=39;
unsigned long LED_time=0;
#define LED_PERIOD 1000
#define SENSOR_IDLE_PERIOD 10000
unsigned long sensorIdleTime;
unsigned long start_time;
unsigned long end_time; 
bool new_poll=false;
int num_negs=0;
void setup() {
  Serial.begin(115200);
  Serial.println("Starting");
  Serial7.begin(19200);
  buff=Serial7.read();
  Serial.print(buff);
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,HIGH);
  pinMode(LED_PIN_2,OUTPUT);
  digitalWrite(LED_PIN_2,HIGH);
}

void loop() {
  if(new_poll){
    buff=Serial7.read();
    if(buff!=-1) {
      Serial.print(" num negative returns are: ");
      Serial.println(num_negs);
      num_negs=0;
      Serial.print(" non neg returns are: ");
      Serial.print((char) buff);
      if(buff=='A') start_time=millis();
      else if(buff=='O') end_time=millis();
    }
    else{ 
      num_negs++;
      if(num_negs>=10000) new_poll=false;
    }
  }
  // put your main code here, to run repeatedly: 
  if ((long) (millis() - sensorIdleTime) > 0) {
    sensorIdleTime = millis() + SENSOR_IDLE_PERIOD;
    //Serial.print(flowmetersBuffer);
    //Serial.print(flow_1.available(),DEC);
    Serial7.print("A\r");
    new_poll=true;
    num_negs=0;
    Serial.print(start_time);
    Serial.print(",");
    Serial.print(end_time);    
    switch_LED_2();
  }  

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
