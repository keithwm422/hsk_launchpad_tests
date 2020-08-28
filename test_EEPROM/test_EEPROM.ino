// test of the EEPROM functions for priorities
#include <EEPROM.h>
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
#define NUM_LOCAL_CONTROLS 243
#define EEPROM_CMD_SIZE NUM_LOCAL_CONTROLS+1 // this needs to be a # divisible by 4 (since eeprom does things in 4 byte words).
uint8_t arr[10]={0};
// version number (incremented after a change)
uint32_t lock= 0xAA; // lock was dumbly set here. should be 0x02090989 or 34146697 in dec. 
uint32_t version;
// has to be 32 bits in eeprom
uint8_t test_prio_array[EEPROM_CMD_SIZE];
uint8_t * arg; 
uint32_t one_byte=0;
uint32_t i;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Hello, starting...");
  Serial.print("\n");
  ROM_EEPROMInit(); //necessary//
  unlock_eeprom();
  read_eeprom();
  lock_eeprom();
}

void loop() {
  while(Serial.available()>0){
    one_byte=Serial.read();
    Serial.print("\n");
//    Serial.print("try writing i to test_prio_array? \n");
//    memcpy(test_prio_array,&i, sizeof(i));
    if(one_byte==49) program_eeprom();
    else if(one_byte==50){ //2
      for(int j=0;j<NUM_LOCAL_CONTROLS;j++){
        test_prio_array[j]=0;
      }
      program_eeprom();
      Serial.print("array zeroed, prios off\n");
    }
    else if(one_byte==51){  //3
      lock=0xAA;
      // set the password to be 0xAA.
      ROM_EEPROMBlockProtectSet(0,0);  // the 0th block is the lock.EEPROM_PROT_RW_LRO_URW = 0x0000000 so that read access permitted when locked and write/read acess permitted when unlocked. 
      ROM_EEPROMBlockPasswordSet(0, &lock, 1);
    }
    else if(one_byte == 52){  //4
      // unlock the epprom to write to it.
      unlock_eeprom();
    }
    else if(one_byte==53){ // 5
      read_eeprom();
    }
    else if(one_byte==54){  //6
      Serial.print(ROM_EEPROMMassErase());
    }
    else {
      lock_eeprom();
    }
  }
//  delay(3000);
}

//! \return Returns a logical OR combination of \b EEPROM_RC_WRBUSY, \b
//! EEPROM_RC_NOPERM, \b EEPROM_RC_WKCOPY, \b EEPROM_RC_WKERASE, and \b
//! EEPROM_RC_WORKING to indicate status and error conditions.
//
//*****************************************************************************
//uint32_t
//EEPROMBlockProtectSet(uint32_t ui32Block, uint32_t ui32Protect)


//! \return Returns a logical OR combination of \b EEPROM_RC_WRBUSY, \b
//! EEPROM_RC_NOPERM, \b EEPROM_RC_WKCOPY, \b EEPROM_RC_WKERASE, and \b
//! EEPROM_RC_WORKING to indicate status and error conditions.
//ui32_tBlock is which block the password starts on. we use a size 1 lock, so uin32_t lock is the password and block 0 to lock the entire eeprom. 
// call as ROM_EEPROMBlockPasswordSet(0, &lock, 1) // count has to be 1, 2, or 3. 
//*****************************************************************************
//uint32_t EEPROMBlockPasswordSet(uint32_t ui32Block, uint32_t *pui32Password, uint32_t ui32Count)

//! \return Returns the lock state for the block on exit, 1 if unlocked (as
//! would be the case if no password was set) or 0 if locked.
//!
//*****************************************************************************
//uint32_t EEPROMBlockLock(uint32_t ui32Block)


//! \return Returns the lock state for the block on exit, 1 if unlocked or 0 if
//! locked.
//!
//*****************************************************************************
//uint32_t EEPROMBlockUnlock(uint32_t ui32Block, uint32_t *pui32Password, uint32_t ui32Count)


void print_array(){
  Serial.print("test_prio_array is : [");
  for(int j=0; j<NUM_LOCAL_CONTROLS; j++) {
    Serial.print(test_prio_array[j]);
    if (j<NUM_LOCAL_CONTROLS-1) Serial.print(", ");    
  }
  Serial.print(" ]");
  Serial.print("\n");    
}

void read_eeprom(){
  Serial.print("reading eeprom\n");
  ROM_EEPROMRead(&version, 32*sizeof(lock), sizeof(version)); // read is in the form (pointer to the place to put the read, address_start (block 0 should be set to password so this should be sizeof(version)), how many bytes? (or words?)).
  ROM_EEPROMRead((uint32_t *)test_prio_array, (32*sizeof(lock)) + sizeof(version), sizeof(test_prio_array));
  Serial.print("version is " );
  Serial.print(version);
  Serial.print("\n");
  print_array();
}
void program_eeprom(){
  Serial.print("programming eeprom\n");  
  ROM_EEPROMProgram((uint32_t *) test_prio_array, (32*sizeof(lock)) + sizeof(version), sizeof(test_prio_array));
  version+=1;
  ROM_EEPROMProgram(&version, 32*sizeof(lock), sizeof(version));
  Serial.print("version incremented! : ");
  Serial.print(version);
  Serial.print("\n");
  print_array();
}

void unlock_eeprom(){
  // unlock the epprom to write to it.
  lock =0xAA; // this is now the password!
  Serial.print("should be unlocked: ");
  Serial.print(ROM_EEPROMBlockUnlock(0, &lock, 1));
  Serial.print("\n");
}

void lock_eeprom(){
  Serial.print("eeprom locked: ");
  Serial.print(ROM_EEPROMBlockLock(0));
  Serial.print("\n");
}
