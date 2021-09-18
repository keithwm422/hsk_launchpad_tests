
uint8_t to_write_array[4]={0};
uint8_t number=0;
unsigned long PotUpdateTime=0;
#define POT_UPDATE_PERIOD 1000
// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=false;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Hello Mike \n");
  Serial3.begin(115200);
  Serial.println(Serial3.read());
  PotUpdateTime=millis() +POT_UPDATE_PERIOD;
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  is_high=true;
}

void loop() {
  // put your main code here, to run repeatedly: 
  if(Serial3.available()){
    Serial.println(Serial3.read());
  }
  if((long) (millis() - PotUpdateTime) > 0){
    PotUpdateTime+= POT_UPDATE_PERIOD;
    switch_LED();
    number++;
    to_write_array[0]=254;
    to_write_array[1]=171;
    to_write_array[2]=number;
    //      to_write_array[3]=13; // carriage return duh
    //Serial3.write(to_write_array,4);
    Serial3.write(to_write_array,3);
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
