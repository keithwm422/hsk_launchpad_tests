// Flowmeato emulator
char send_out[80]="85\r";

char receive[20];
int i=0;
// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
bool new_write=false;
void setup() {
  Serial1.begin(19200);
  Serial.begin(115200);
  Serial.println("STARTING");
  // don't do anything else because this should only be sending messages when polled
  pinMode(LED, OUTPUT);
  Serial1.flush();
  digitalWrite(LED,LOW);

}

void loop() {
  // put your main code here, to run repeatedly
  Serial.print("i");
  if(!Serial){
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(BLUE_LED,HIGH);
  }
  while(Serial1.available()){
    receive[i]=Serial1.read();
    i++;
    if(i>=19) {
        i=0;
        // reset the characters
        for(int j=0;j<20;j++){
          receive[j]=0;
        }
        break;
    }
    else if(receive[i-1]==13){
      new_write=true;
      i=0;
      break;
    }
  }
  if(new_write){
    if(receive[0]==254 && receive[1]==171){
      Serial1.print(receive[0]);
      Serial1.print(receive[1]);
      Serial1.print(receive[2]);
      for(int j=0;j<20;j++){
        receive[j]=0;
      }
    }
    else if(receive[0]==254 && receive[1]==170){
      Serial1.print(receive[0]);
      Serial1.print(receive[1]);
      Serial1.print(receive[2]);
      Serial1.print(receive[3]);
      for(int j=0;j<20;j++){
        receive[j]=0;
      }
    }
    else{
      Serial1.print("Wrong ID");
      for(int j=0;j<20;j++){
        receive[j]=0;
      }
    }
    new_write=false; 
  }
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
    Serial.println("LED");
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
