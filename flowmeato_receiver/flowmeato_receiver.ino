// Flowmeato receiver
char receive[80];
// either serial1, 7 or 6 (6 should be solo conn)
char send_out[10];
uint8_t send_out_real[4]={0,0,0,0};
int i=0;
const int buttonPin = PUSH1;    
int state = LOW;      
int reading;           
int previous = HIGH;   
void setup() {
  Serial.begin(115200);
  Serial4.begin(19200);
  delay(1000);
  Serial.print("hello");
  Serial4.print("E\r");
  pinMode(buttonPin, INPUT_PULLUP);

  // don't do anything else because this should only be sending messages when polled
}

void loop() {
  // put your main code here, to run repeatedly
  //if(Serial3.available()){
//    receive[i]=Serial1.read();
  int receive_new=Serial4.read();
  if(receive_new!=-1){
    Serial.print((char) receive_new);
  }
  int receive_0=Serial.read();
  if(receive_0!=-1){
    if(receive_0==65){
      Serial.println("pushed");
      //Serial6.print("E\r");
      // try reading the register 20. 
      Serial4.print("ER20\r");// read register 20 and it will be in the form ID REG = VALUE like B 016 = 199
      //Serial4.print("EW20=9303\r"); //fill in __ with value to write subtract 768 8535
      
    }
  }
}
