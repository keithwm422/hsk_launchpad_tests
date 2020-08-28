#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(){

//  char response[80]="@253ACK7.460E+2;FF";
  char response[80]="@253ACK7.460E+2;FF";
  printf("%s\n",response);
  char response_parse[20];
  for(int j=0;j<80;j++){
    if(response[j]==75) response[j]=32; // if K then put a space instead
    if(response[j]==59) response[j]=32; // if a semicolon put a space instead
  }
  float pressure_new{0};
  sscanf(response,"%s %f FF",response_parse,&pressure_new);
  printf("%s\n",response);
  printf("%f\n",pressure_new);
/*  char * resp;
  int i=0;
//  for(int i=0; i<80;i++){
//    printf("%c",response[i]);
//  }
  resp = strtok(response,";FF");  // here resp points to array that doesn't have ;FF after the value
  char * ack = strchr(resp,'K'); //ack should point to the end of ACK
  printf ("%s\n",resp);
  printf ("%s\n",ack);
  char new_array[10];
  char new_array_new[10];
  strcpy(new_array,ack);
  for(int j=0;j<10;j++) {
    printf("%c\n",new_array[j]);
    if(new_array[j]==69) new_array[j]=101;
    if(j==0) new_array_new[j]=32;
    else new_array_new[j]=new_array[j];
  }
  printf("%s\n",new_array);
  printf("%s\n",new_array_new);
  float pressure= atof(new_array_new);
  printf("%d\n",pressure);
  float pressure2= strtof(new_array,&ack);
  printf("%d\n",pressure2);

//    strcpy(response_parse,resp);
//    resp = strtok (response_parse, "ACK");
//    resp = strtok (resp_r, "ACK");
//    i++;
*/
  return 0;
}
