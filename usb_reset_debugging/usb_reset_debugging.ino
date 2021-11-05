#include "uart.h" // to check if comp-launchpad is okay
#include "inc/hw_uart.h"
//#include "inc/hw_usb.h"

#include "driverlib/usb.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
unsigned long LEDUpdateTime=0; // keeping LED to visualize no hanging
bool is_high=true;
uint32_t uart_error_reg;
uint8_t lower_nibble_error_reg;
uint32_t usbdeviceaddress;
uint32_t usbmode_status;
void setup() {
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
  ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);
  USBPHYPowerOn(USB0_BASE);
  SysCtlUSBPLLEnable();
  //USBModeConfig(USB0_BASE,USB_MODE_DEV);
  USBDevMode(USB0_BASE);
  USBIntEnableControl(USB0_BASE,USB_INTCTRL_SUSPEND);
  USBDevConnect(USB0_BASE);
  //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  usbmode_status=USBModeGet(USB0_BASE);
  // put your setup code here, to run once:
  // LED on launchpad
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  //usbdeviceaddress=USBDevAddrGet(USB0_BASE);
  Serial.begin(19200);
  Serial.println(usbmode_status,HEX);

  usbmode_status=SysCtlClockGet();
  Serial.println(usbmode_status,DEC);

  //Serial.println(UARTRxErrorGet(0x4000C000),HEX);
  //Serial.println(usbdeviceaddress,HEX);
  pinMode(BLUE_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
  LEDUpdateTime= millis()+LED_UPDATE_PERIOD;
    //
    // Set the USB stack mode to Device mode with VBUS monitoring.
    //

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
  //USBDCDCInit(0, &g_sCDCDevice);
  usbmode_status=USBDevSpeedGet(USB0_BASE);
  Serial.println(usbmode_status,HEX);
  usbmode_status=USBIntStatusControl(USB0_BASE);
  Serial.println(usbmode_status,HEX);
  //usbdeviceaddress=USBDevAddrGet(USB0_BASE);

}

void loop() {
  // put your main code here, to run repeatedly: 
    if((long) (millis() - LEDUpdateTime) > 0){
    LEDUpdateTime+= LED_UPDATE_PERIOD;
    switch_LED();
    //uart_error_reg|=0x01;
    if(usbmode_status & 0x01) digitalWrite(RED_LED,HIGH);
  }
  usbmode_status=USBIntStatusControl(USB0_BASE);

  //uart_error_reg=UARTRxErrorGet(0x4000C000);
  //lower_nibble_error_reg=uart_error_reg & 0x0F;
  
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
