/*
Smart Sensor Simulator Specific Header File for the Cat ADEM 4
Adjust these settings to make the module fault free
For use with a SSS Rev 10 board and SSS Cable Assembly
*/
#ifndef _SSS_ADEM4_DEFS_H_
#define _SSS_ADEM4_DEFS_H_
#include <Arduino.h>

  const int numCommands = 83;
  unsigned int settings[numCommands];//   
  const char headings[numCommands][5]={'U1-P0','U1-P1','U1-P2','U1-P3','U2-P0','U2-P1','U2-P2','U2-P3','U3-P0','U3-P1','U3-P2','U3-P3','U4-P0','U4-P1','U4-P2','U4-P3','U5-P0','U5-P1','U5-P2','U5-P3','VoutA','VoutB','VoutC','VoutD','PWM1','PWM2','19391','19392','CAN21','CAN22','CAN31','CAN32','V2WS1','V2VS2','E2WS1','E2WS2','E2WS3','E2WS4','U1-0V','U1-1V','U1-2V','U1-3V','U2-0V','U2-1V','U2-2V','U2-3V','U3-0V','U3-1V','U3-2V','U3-3V','U4-0V','U4-1V','U4-2V','U4-3V','U5-0V','U5-1V','U5-2V','U5-3V','U1-0G','U1-1G','U1-2G','U1-3G','U2-0G','U2-1G','U2-2G','U2-3G','U3-0G','U3-1G','U3-2G','U3-3G','U4-0G','U4-1G','U4-2G','U4-3G','U5-0G','U5-1G','U5-2G','U5-3G','CAN2P','Coil1','Coil2','Coil3','Coil4'};
//setting[??] = value //Schematic Port Name, PCB Pin Number, Connector Pin Number, Wire Application, Wire Color, Fault Cleared  
  settings[0] = 50; //U1-P0W, J16-9, P2-38, Air Inlet Temp, Green/White
  settings[1] = 50; //U1-P1W, J16-10, P2-84, Coolant Temp, Red/Blue
  settings[2] = 50; //U1-P2W, J16-11, P2-95, Inlet Manifold Air Temp, Brown/Blue
  settings[3] = 50; //U1-P3W, J16-13, P2-96, Fuel Temp, Yellow/White
  settings[4] = 50; //U2-P0W, J18-2, P1-49, Coolant level Norm (4 wire), Grey/Black
  settings[5] = 50; //U2-P1W, J18-3, P1-26, Coolant level normal (2 wire), White/Black
  settings[6] = 50; //U2-P2W, J18-6, P1-19, Output # 6, Yellow/Black
  settings[7] = 50; //U2-P3W, J18-7, P1-20, Output # 7, Green/Black
  settings[8] = 50; //U3-P0W, J10-4, P2-5, Coolant Diverter Solenoid/ADR Valve, Tan
  settings[9] = 50; //U3-P1W, J10-9, P2-41, ARD Pilot Fuel Pressure, Pink/Black
  settings[10] = 50; //U3-P2W, J10-5, P2-40, Fuel Pressure, Tan/White
  settings[11] = 50; //U3-P3W, J10-10, P2-89, ARD Fuel Pressure, White/Black
  settings[12] = 50; //U4-P0W, J24-15, P2-13, DPF inlet temperature (Double), Blue
  settings[13] = 50; //U4-P1W, J24-16, P2-62, DPF outlet temperature (Double), Brown/White
  settings[14] = 50; //U4-P2W, J24-17, P2-56, DPF inlet temperature (Single), Orange/White
  settings[15] = 50; //U4-P3W, J24-18, P2-94, DPF outlet temperature (Single), Purple/Black
  settings[16] = 50; //U5-P0W, J10-7,  P2-27, ARD MAF Delta Pressure, Brown
  settings[17] = 50; //U5-P1W, J24-9,  P2-66, DPF Pressure Sensor (Single), Green/Black
  settings[18] = 50; //U5-P2W, J24-10, P2-85, DPF pressure Sensor (Double), Blue/Black
  settings[19] = 50; //U5-P3W, J24-14, P2-57, CGI Absolute pressure, Purple/White
  
  //The following settings are in milliVolts
  settings[20] = 1000; //VoutA, J24-19, P2-28, Oil Press, Red/White
  settings[21] = 2000; //VoutB, J24-20, P2-33, Crank Case Pressure, Black/White
  settings[22] = 3000; //VoutC, J24-21, P2-15, Inlet Manifold Air Pressure, Orange
  settings[23] = 4000; //VoutD, J24-22, P2-74, Intake Valve actuation oil  Press, Tan/Black
  
  //The following settings are in duty cycle, so 50 = 2.5V average
  settings[24] = 50; //PWM1, J24-23, P2-16, CGI Delta Pressure, Purple
  settings[25] = 50; //PWM2,    (Not Connected on Rev 10. May need to move to E2WS1)
    
  // for the following switch settings 0 = connected (closed) and 1 = disconnected (open).
  // These are normally closed switches. Uses 120 ohm resistor
  settings[26] = 0; //J1939 Terminating Resistor #1
  settings[27] = 0; //J1939 Terminating Resistor #2
  settings[28] = 1; //CAN2 Terminating Resistor #1
  settings[29] = 1; //CAN2 Terminating Resistor #2
  settings[30] = 0; //CAN3 Terminating Resistor #1
  settings[31] = 0; //CAN3 Terminating Resistor #2
  
  // For the following switches 0 = pull high to +12V through 10 ohms and 1 = Pulled Down.
  settings[32] = 0; //V2WS1, J18-12
  settings[33] = 0; //V2WS2, J18-8, P1-54, Coolant level Low (4 Wire), Grey
  settings[34] = 0; //E2WS1, J24-5
  settings[35] = 0; //E2WS2, J24-6, P2-91, ARD outlet Temperature sensor (EG #2), Orange/Black
  settings[36] = 0; //E2WS3, J24-7, P2-63, ARD Flame Boundary Temp (EG #3), Red/Black
  settings[37] = 0; //E2WS4, J24-8, P2-55, ARD Turbo Outlet Temp (EG #1), Blue/White
  
  //For the following switches on the high side of the potentiometer, 0 = connected to +5V, 1 = disconnected.
  settings[38] = 0; //VccSelectU1-0, J16
  settings[39] = 0; //VccSelectU1-1
  settings[40] = 0; //VccSelectU1-2
  settings[41] = 0; //VccSelectU1-3
  settings[42] = 0; //VccSelectU2-0
  settings[43] = 0; //VccSelectU2-1
  settings[44] = 0; //VccSelectU2-2
  settings[45] = 0; //VccSelectU2-3
  settings[46] = 0; //VccSelectU3-0
  settings[47] = 0; //VccSelectU3-1
  settings[48] = 0; //VccSelectU3-2
  settings[49] = 0; //VccSelectU3-3
  settings[50] = 0; //VccSelectU4-0
  settings[51] = 0; //VccSelectU4-1
  settings[52] = 0; //VccSelectU4-2
  settings[53] = 0; //VccSelectU4-3
  settings[54] = 0; //VccSelectU5-0
  settings[55] = 0; //VccSelectU5-1
  settings[56] = 0; //VccSelectU5-2
  settings[57] = 0; //VccSelectU5-3
 
  //For the following switches on the low side of the potentiometer, 0 = connected to ground, 1 = disconnected.
  settings[58] = 0; //GroundSelectU1-0
  settings[59] = 0; //GroundSelectU1-1
  settings[60] = 0; //GroundSelectU1-2
  settings[61] = 0; //GroundSelectU1-3
  settings[62] = 0; //GroundSelectU2-0
  settings[63] = 0; //GroundSelectU2-1
  settings[64] = 0; //GroundSelectU2-2
  settings[65] = 0; //GroundSelectU2-3
  settings[66] = 0; //GroundSelectU3-0
  settings[67] = 0; //GroundSelectU3-1
  settings[68] = 0; //GroundSelectU3-2
  settings[69] = 0; //GroundSelectU3-3
  settings[70] = 0; //GroundSelectU4-0
  settings[71] = 0; //GroundSelectU4-1
  settings[72] = 0; //GroundSelectU4-2
  settings[73] = 0; //GroundSelectU4-3
  settings[74] = 0; //GroundSelectU5-0
  settings[75] = 0; //GroundSelectU5-1
  settings[76] = 0; //GroundSelectU5-2
  settings[77] = 0; //GroundSelectU5-3
  
  //CAN2 Panel Select - 0 = CAN2 on 9-pin is connected to E-CAN, 1 = disconnected
  settings[78] = 0; //CAN2 
  
  // For the following switches 0 = pull high to +12V through 10 ohms and 1 = Pulled Down.
  settings[79] = 0; //Coil 1
  settings[80] = 0; //Coil 2
  settings[81] = 0; //Coil 3
  settings[82] = 0; //Coil 4, P2-70, CGI Temperature, Yellow/Black
  
  
#endif