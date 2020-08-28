// Potentiometer emulator
char send_out=85;

char receive[10];
int i=0;
void setup() {
  Serial1.begin(38400);
  // don't do anything else because this should only be sending messages when polled
}

void loop() {
  // put your main code here, to run repeatedly
  i=0;
  while(Serial1.available()){
    receive[i]=Serial1.read();
    i++;
    if(i==3) break;  // received three bytes
  }
  // if it was correct send back send_out
  if(receive[0]==254){
    if(receive[1]==171){
      Serial1.print(send_out);
    }
    else clear_receive();
  }
  else clear_receive();  // wrong first byte so just throw away the entire read
}

void clear_receive(){
  for(int j=0;j<10;j++){
    receive[j]=0;
  }
}
