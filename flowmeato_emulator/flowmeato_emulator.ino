// Flowmeato emulator
char send_out[80]="A +14.599 +021.71 -000.09 -000.09 +000.00 +000000.0    CO2";

char receive[3];
int i=0;
void setup() {
  Serial1.begin(19200);
  // don't do anything else because this should only be sending messages when polled
}

void loop() {
  // put your main code here, to run repeatedly
  while(Serial1.available()){
    receive[i]=Serial1.read();
    if(receive[i]==13){
      i=0;
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
  if(receive[0]==65){
    Serial1.print(send_out);
  }
  else{
    Serial1.print("Wrong ID");
    delay(1000);
  }
// Serial1.print("this is the me");
// delay(1000); 
}
