// Flowmeato receiver
char receive[80];
// either serial1, 7 or 6 (6 should be solo conn)
char send_out[10];
uint8_t send_out_real[4]={0,0,0,0};
int i=0;
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(1000);
    Serial.print("hello");
    Serial1.print("A\r");

  // don't do anything else because this should only be sending messages when polled
}

void loop() {
  // put your main code here, to run repeatedly
  //if(Serial3.available()){
//    receive[i]=Serial1.read();
    int receive_new=Serial1.read();
    if(receive_new!=-1){
      Serial.print((char) receive_new);
    }
    

/*    else {
      if(i>=79) {
        i=0;
        break;
      }
      else i++;
    }
  }
  for(int j=0;j<80;j++){
    Serial.print(receive[i]);
  }
*/  // reset the characters
//  for(int j=0;j<10;j++){
//    receive[j]=0;
//  }
    //Serial.print(receive_new);
  //}
  /*else {
    Serial3.print("A\r");
    //Serial3.write(send_out_real,4);
    //delay(1000);
  }*/
}
