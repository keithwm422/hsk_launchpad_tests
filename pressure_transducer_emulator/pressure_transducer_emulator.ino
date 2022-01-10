// pressure transducer for the mainhsk emulator
// for pressure for temperature @253TEM?;FF for pressure @253PR4?;FF
// Query reply: @253ACK1.23E-4;FF or @253ACK2.50E+1;FF
char send_out_pressure[80]="@253ACK1.23E-4;FF";
char send_out_temperature[80]="@253ACK2.50E+1;FF";
char receive[20];
int i=0;
// for Launchpad LED
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
bool new_write=false;
int number_bytes_read=0;
int num_fs=0;
int bad_reqs=0;
int good_reqs=0;
void setup() {
  Serial2.begin(19200);
  // don't do anything else because this should only be sending messages when polled
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);
}

void loop() {
  // new code just read always
  int value_read=Serial2.read();
  if(value_read!=-1){
    // then we had a read, but now we want to check which read we have...
    // check how many reads we have had, should be a total of 11 bytes read to then decode the message
    receive[number_bytes_read]=(char) value_read;
    number_bytes_read++;
    if(value_read=='F') num_fs++;
    if(num_fs>=2){
      if(number_bytes_read==11){
        decode_message(number_bytes_read);
        good_reqs++;
        number_bytes_read=0;
        num_fs=0;
      }
      else{
        // we got FFs in the wrong spot, so restart reading arrays completely
        number_bytes_read=0;
        num_fs=0;
        bad_reqs++;
      }
    }
  }
  if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
  }
// Serial1.print("this is the me");
// delay(1000); 
}
void decode_message(int bytes_read){
  // check the first 4 to be "@253"
  if(check_first_4_chars() && check_last_3_chars(bytes_read)){
    // check if pressure or temp
    bool is_pressure=check_command();
    if(is_pressure) Serial2.print(send_out_pressure);
    else Serial2.print(send_out_temperature);
    number_bytes_read=0;
  }
  else {
    Serial2.print("Wrong ID");
    Serial2.println(bytes_read,DEC);
    Serial2.println(receive);
    Serial2.println(return_last_3_chars(bytes_read),DEC);
    number_bytes_read=0;
  }
}
bool check_char(int i, char to_check){
  if(receive[i]==to_check) return true;
  else return false;
}
bool check_first_4_chars(){
  char to_check[5]="@253";
  int check_num_bytes=4;
  int matches=0;
  for(int i =0;i<check_num_bytes;i++){
    if(check_char(i,to_check[i])) matches++;
  }
  if(matches==4) return true;
  else return false;
}
bool check_last_3_chars(int bytes_read){
  char to_check[5]=";FF";
  int check_num_bytes=3;
  int matches=0;
  for(int i=0;i<3;i++){
    if(check_char(bytes_read-3+i,to_check[i])) matches++;
  }
  if(matches==3) return true;
  else return false;
}
bool check_command(){
  char to_check[2]="P";
  if(receive[4]==to_check[0]) return true;
  else return false;
}
int return_last_3_chars(int bytes_read){
  char to_check[5]=";FF";
  int check_num_bytes=3;
  int matches=0;
  for(int i=0;i<3;i++){
    Serial2.println(receive[bytes_read-3+i]);
    if(check_char(bytes_read-3+i,to_check[i])) matches++;
  }
  return matches;
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
