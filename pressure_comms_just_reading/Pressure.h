// Library authored by Keith McBride
// 04/16/20
#pragma once
#ifndef Pressure_h
#define Pressure_h

#include "Arduino.h"
#include <string.h>

class Pressure
{
public:

  void begin(HardwareSerial &refSer, int BAUD_rate){
    Serial_num = &refSer;
    Serial_num->begin(BAUD_rate);
  }
  void read_port_again(){
    read_port();
  }
  void poll(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("PR4");  //query pressure request, which is here 4 digits
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void setbaud(int BAUD_rate){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("BR");  // to set baud rate
    Serial_num->print("!");    // because it is a config command
    Serial_num->print(BAUD_rate);  // the baud rate to set, can only be standards up to 230400
    Serial_num->print(";FF");  // command termination
  }
  void setaddress(int address){
    if(address>=1 && address<=253){
      Serial_num->print("@");
      Serial_num->print("253");  // address here which 254 or 255 are universal
      Serial_num->print("AD");  // to set address (001-253)
      Serial_num->print("!");    // because it is a config command
      Serial_num->print(address);  // the address
      Serial_num->print(";FF");  // command termination
    }
  }
  void setstring(char tag[8]){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("UT");  // to set string tag of transducer
    Serial_num->print("!");    // because it is a config command
    Serial_num->print(tag);  // the 'string'
    Serial_num->print(";FF");  // command termination
  }
  void setunits(uint8_t which_unit){
    if(which_unit>=1 && which_unit<=3){
      Serial_num->print("@");
      Serial_num->print("253");  // address here which 254 or 255 are universal
      Serial_num->print("U");  // to set string tag of transducer
      Serial_num->print("!");    // because it is a config command
      switch(which_unit){
        case 1: Serial_num->print("TORR");  // the 'string'
        case 2: Serial_num->print("MBAR");  // the 'string'
        case 3: Serial_num->print("PASCAL");  // the 'string'
      }
      Serial_num->print(";FF");  // command termination
    }
  }
  void poll_units(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("U");  //query current units of pressure
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_pressure1(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("PR1");  //query pressure request, which is here 4 digits
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_pressure2(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("PR2");  //query pressure request, which is here 4 digits
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_pressure3(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("PR3");  //query pressure request, which is here 4 digits
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_pressure4(uint8_t w_index){
    switch(w_index){
      case 0: Serial_num->print("@");
      case 1: Serial_num->print("253");  // address here which 254 or 255 are universal
      case 2: Serial_num->print("PR4");  //query pressure request, which is here 4 digits
      case 3: Serial_num->print("?");    // because it is a query
      case 4: Serial_num->print(";FF");  // command termination
      default: break;
    }
  }
  void poll_pressure5(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("PR5");  //query pressure request, which is here 4 digits
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_temp(uint8_t w_index){
    switch(w_index){
      case 0: Serial_num->print("@");
      case 1: Serial_num->print("253");  // address here which 254 or 255 are universal
      case 2: Serial_num->print("TEM");  //query temperature
      case 3: Serial_num->print("?");    // because it is a query
      case 4: Serial_num->print(";FF");  // command termination      
      default: break;
    }
  }
  void poll_baud(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("BR");  //query baud rate
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_address(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("AD");  //query device address
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_model(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("MD");  //query device model number
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_name(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("DT");  //query type name
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_manu(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("MF");  //query manufacturer name
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_hv(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("HV");  //query hardware version
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_fv(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("FV");  //query firmware version
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_sn(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("SN");  //query serial number
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_time(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("TIM");  //query time on
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_string(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("UT");  //query some string
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void poll_status(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("T");  //query status
    Serial_num->print("?");    // because it is a query
    Serial_num->print(";FF");  // command termination
  }
  void unlock(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("FD");  // lock/unlock call
    Serial_num->print("!");    // because it is a cmd
    Serial_num->print("UNLOCK;FF");  // command termination
    read_port();
  }
  void lock(){
    Serial_num->print("@");
    Serial_num->print("253");  // address here which 254 or 255 are universal
    Serial_num->print("FD");  // lock/unlock call
    Serial_num->print("!");    // because it is a cmd
    Serial_num->print("LOCK;FF");  // command termination
    read_port();
  }
  void just_give_me_the_response(char RESP[40]){
    for(int i=0;i<40;i++){
      RESP[i]=responseString[i];
      responseString[i]='\0';
    }
  }
  void reset_string(){
    internal_reset_string();
  }
  float give_me_the_float(){
    char response_parse[20]={0};
    int K_loc=0;
    int semi_loc=0;
    for(int j=0;j<40;j++){
      if(responseString[j]==75) K_loc=j; // if K then stroe pos $
      if(responseString[j]==59) semi_loc=j; // if a semicolon store pos$
    }
    for(int j=0;j<semi_loc-K_loc-1;j++){ // dont want K or semicolon so subtract 2 from end pos
      response_parse[j]=responseString[K_loc+1+j];
    }
    internal_reset_string();
    return (atof(response_parse));
  }
  int is_available(){
    return (Serial_num->available());
  }
  char get_one_byte(){  // use internal vars like from read_port to keep track of how many have been read
    chunk = Serial_num->read();
    if(chunk>-1){
      responseString[rindex_i] = chunk;
      responseString[rindex_i + 1] = '\0';
      rindex_i++;
    }
    return chunk;
  }
private:
  void read_port(){
    if(Serial_num->available()){
      responseString[0] = '\0';
      rindex_i = 0;
      while (Serial_num->available()) {
        chunk = Serial_num->read();
        responseString[rindex_i] = chunk;
        responseString[rindex_i + 1] = '\0';
        rindex_i++;
        if (strlen(responseString) > 39) {
          strcpy(error, "Bad read: Too long");
          break;
        }
      }
    }
  }
  int charIndex(char *string, int start, char c) {
    for (int i = start; i < strlen(string); i++) {
      if (string[i] == c) {
        return i;
      }
    }
    return -1;
  }
  void internal_reset_string(){
        // reset rindex_i and the responseString
    for(int i=0;i<40;i++){
      responseString[i]='\0';
    }
    rindex_i=0;
  }
private:

  char responseString[40]; // to store the response of the pressure transducer on a read
  int rindex_i = 0;
  int timeout = 0;
  char chunk;
  char error[100];  // to store the error returns if any
  HardwareSerial * Serial_num;
};

#endif // Pressure_H
