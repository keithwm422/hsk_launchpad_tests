// Flowmeato receiver
char receive[80];
// either serial1, 7 or 6 (6 should be solo conn)
char send_out[10];
int i=0;
void setup() {
  Serial.begin(115200);
  Serial.print("hello");
  Serial7.begin(19200);
  // don't do anything else because this should only be sending messages when polled
}

void loop() {
  // put your main code here, to run repeatedly
  if(Serial7.available()){
//    receive[i]=Serial1.read();
    char receive_new=Serial7.read();
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
    Serial.print(receive_new);
  }
  else {
    Serial7.print("A\r");
    delay(1000);
  }
}
