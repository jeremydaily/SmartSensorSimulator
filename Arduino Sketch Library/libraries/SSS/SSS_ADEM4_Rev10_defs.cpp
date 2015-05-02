/*
Smart Sensor Simulator Specific Header File for the Cat ADEM 4
Adjust these settings to make the module fault free
For use with a SSS Rev 10 board and SSS Cable Assembly
*/

#ifndef _SSS_ADEM4_DEFS_H_
#define _SSS_ADEM4_DEFS_H_
#include <Arduino.h>
#include <SPI.h>
#include <inttypes.h>

  const int numCommands = 83;
  unsigned int settings[numCommands];//   
  //unsigned int &settingsPtr = settings;
  
  const char headings[numCommands][5]={'U1-P0','U1-P1','U1-P2','U1-P3','U2-P0','U2-P1','U2-P2','U2-P3','U3-P0','U3-P1','U3-P2','U3-P3','U4-P0','U4-P1','U4-P2','U4-P3','U5-P0','U5-P1','U5-P2','U5-P3','VoutA','VoutB','VoutC','VoutD','PWM1','PWM2','19391','19392','CAN21','CAN22','CAN31','CAN32','V2WS1','V2VS2','E2WS1','E2WS2','E2WS3','E2WS4','U1-0V','U1-1V','U1-2V','U1-3V','U2-0V','U2-1V','U2-2V','U2-3V','U3-0V','U3-1V','U3-2V','U3-3V','U4-0V','U4-1V','U4-2V','U4-3V','U5-0V','U5-1V','U5-2V','U5-3V','U1-0G','U1-1G','U1-2G','U1-3G','U2-0G','U2-1G','U2-2G','U2-3G','U3-0G','U3-1G','U3-2G','U3-3G','U4-0G','U4-1G','U4-2G','U4-3G','U5-0G','U5-1G','U5-2G','U5-3G','CAN2P','Coil1','Coil2','Coil3','Coil4'};

  
  
#endif