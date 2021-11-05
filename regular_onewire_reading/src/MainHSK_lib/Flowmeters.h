// Library authored by Keith McBride
// adapted from github code by Rostom and Lucas
#pragma once
#ifndef Flowmeters_h
#define Flowmeters_h

#include "Arduino.h"
#include <string.h>

class Flowmeters
{
public:

  void begin(HardwareSerial &refSer, int BAUD_rate, const char * ID){
    Serial_num = &refSer;
    Serial_num->begin(BAUD_rate);
    flow_ID=ID;
  }
  int available(){
    return Serial_num->available();
  }
  void read_setup(){
    responseString[0] = '\0';
    rindex_i = 0;
  }
  int read_byte(){
    int read_long=Serial_num->read();
    if(read_long!=-1){
      chunk = (char) read_long;
      responseString[rindex_i] = chunk;
      responseString[rindex_i + 1] = '\0';
      rindex_i++;
    }
    return read_long;
  }
  void setpoint(double setpoint){  
    Serial_num->print(flow_ID);
    Serial_num->print("S");
    Serial_num->print(setpoint,2);
    Serial_num->print("\r");
  }
  void poll(){
    Serial_num->print(flow_ID);
    Serial_num->print("\r");
    //read_port();
  }
  void openthevalve(){
    Serial_num->print(flow_ID);
    Serial_num->print("C\r"); 
  }
  void closethevalve(){
    Serial_num->print(flow_ID);
    Serial_num->print("HC\r");
  }
  void holdthevalve(){
    Serial_num->print(flow_ID);
    Serial_num->print("HP\r"); 
  }
  void lockdisplay(){
    Serial_num->print(flow_ID);
    Serial_num->print("1\r"); 
  }
  void unlockdisplay(){
    Serial_num->print(flow_ID);
    Serial_num->print("u\r"); 
  }
  void getmanufacturerinfo(){
    Serial_num->print(flow_ID);
    Serial_num->print("??m*\r"); 
    //read_port();
  }
  void getfirmwareversion(){
    Serial_num->print(flow_ID);
    Serial_num->print("ve\r"); 
    //read_port();
  }
  void getGasList(){
    Serial_num->print(flow_ID);
    Serial_num->print("??g*\r"); 
    //read_port();
  }
  // gas number must be within range [0,255] or else this function returns false.
  bool choosegas(int gas_num){
    if(gas_num>=0 && gas_num <=255){
      Serial_num->print(flow_ID);
      Serial_num->print("G");
      Serial_num->print(gas_num);
      Serial_num->print("\r");
      return true;
    }
    else return false;
  }

  bool mix_two_gases(char gas_name[6], int mix_num, int gas_num_1, float gas_percent_1, int gas_num_2, float gas_percent_2){
    bool status;
    if(mix_num>=236 && mix_num<=255) status=true;
    else if (mix_num==0) status=true; // 0 means next mix number not specified is used
    else return false; 
    Serial_num->print(flow_ID);
    Serial_num->print("GM");
    Serial_num->print(" ");
    Serial_num->print(gas_name);
    Serial_num->print(" ");
    Serial_num->print(mix_num);
    Serial_num->print(" ");
    Serial_num->print(gas_percent_1,2);
    Serial_num->print(gas_num_1);
    Serial_num->print(gas_percent_2,2);
    Serial_num->print(gas_num_2);  
    Serial_num->print("\r");
    //agm [Mix Name] [Mix Number] [Gas1 %]   [Gas1 #] [Gas2 %] [Gas2 #]...);
    //read_port();
    return status;
  }
  bool delete_mix_num(int mix_num){
    //agd [Mix #]
    if(mix_num<236 || mix_num>255) return false;
    else{
      Serial_num->print(flow_ID);
      Serial_num->print("GD");
      Serial_num->print(mix_num);
      Serial_num->print("\r");
      return true;
    }
  }

  void tare_flow(){
    Serial_num->print(flow_ID);
    Serial_num->print("V\r");
  }
  bool getGasData(double Data[6],char gas_stuff[100]){
    bool read_good = parse_read();
    if(!read_good) return false;
    else{
      for(int i=0;i<6;i++){
        Data[i]=gasdata[i];
      }
      for(int j=0;j<100;j++) gas_stuff[j]=gas[j];
      return true;
    }
  }
  void getMoreData(char Data[100]) {
    for(int i=0;i<100;i++){
      Data[i]=gas[i];
    }
  }
  void getErrors(char error_msg[100]){
    for(int i=0;i<100;i++){
        error_msg[i]=error[i];
      }
  }
  void just_give_me_the_response(char RESP[200]){
    for(int i=0;i<200;i++){
      RESP[i]=responseString[i];
    }
  }

private:
  bool parse_read(){
    // Start Parsing
    int prior_index;
    int index = -1;
    char selection[100];
    bool retval;
    index = charIndex(responseString, 0, ' ');
    for (int i = 0; i < 6; i++) {
      prior_index = index;
      index = charIndex(responseString, index + 1, ' ');
      if (index == -1) {
        strcpy(error, "Bad Response: Unable to parse");
        return false;
      }
      else {
        for (int j = prior_index + 1; j < index; j++) {
          selection[j - (prior_index + 1)] = responseString[j];
        }
        selection[index - (prior_index)] = '\0';
        gasdata[i] = atof(selection);
      }
    }
    while (responseString[index + 1] == ' ') {
      index++;
    }
    for (int j = index + 1; j < strlen(responseString); j++) {
      gas[j - (index + 1)] = responseString[j];
    }
    gas[strlen(responseString) - (index + 1)] = '\0';
    strcpy(error,"None");
    return true;
  }
  void read_port(){
    if(Serial_num->available()){
      responseString[0] = '\0';
      rindex_i = 0;
      while (Serial_num->available()) {
        chunk = Serial_num->read();
        responseString[rindex_i] = chunk;
        responseString[rindex_i + 1] = '\0';
        rindex_i++;
        if (strlen(responseString) > 198) {
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

private:

/*    struct DACInputData
    {
        VREF vref;
        PWR_DOWN pd;
        GAIN gain;
        uint16_t data;
    };
*/
  char gas[100]; // gas info parts or the response string
  double gasdata[6]; // actual values to send back because they are reads by the flowmeter
  char responseString[200]; // to store the response of the flowmeter on a read
  int rindex_i = 0;
  int timeout = 0;
  char chunk;
  char error[100];  // to store the error returns if any
  HardwareSerial * Serial_num;
  const char * flow_ID;
};

#endif // Flowmeters_H
