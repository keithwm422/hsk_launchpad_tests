// Flowmeato emulator
char send_out[80]="A +14.599 +021.71 -000.09 -000.09 +000.00 +000000.0    CO2";

char receive[3];
int i=0;
// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
bool new_write=false;
void setup() {
  Serial2.begin(19200);
  // don't do anything else because this should only be sending messages when polled
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);
}

void loop() {
  // put your main code here, to run repeatedly
  while(Serial2.available()){
    receive[i]=Serial2.read();
    if(receive[i]==13){
      i=0;
      new_write=true;
      break;
    }
    else {
      if(i>=2) {
        i=0;
        // reset the characters
        for(int j=0;j<10;j++){
          receive[j]=0;
        }
        break;
      }
      else i++;
    }
  }
  if(new_write){
    if(receive[0]==65){
      Serial2.print(send_out);
      for(int j=0;j<10;j++){
        receive[j]=0;
      }
    }
    else{
      Serial2.print("Wrong ID");
      delay(1000);
    }
    new_write=false;
  }
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
  }
// Serial1.print("this is the me");
// delay(1000); 
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
