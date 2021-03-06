/*
Title: Smart Sensor Simulator Firmware for the ATMega2560 Processor on an SSS Rev 10 board
Comments are for the wiring of an SSS for a CAT ADEM4.
*/

//Load these libraries
#include <Wire.h> // Digital to Analog converter
#include <SPI.h> // CAN and Digital Pots
#include <mcp_can.h> // J1939 communications
#include <Mcp4261.h> // Digital Potentiometers
#include <SSS.h> // Pin definitions and helper functions


void setup()
{ 
  sss.begin();
  sss.IDstring = "SYNER*SSS-ADEM4*1R90023     "; //Change this to match the case of the SSS you are programming. Be sure to include trailing spaces.
  sss.IDstring.toCharArray(sss.compID,29); // Convert the component ID to a character array for CAN

  //Adjust the settings below to match the particular Smart Sensor Simulator that is being programmed. These settings are interpreted by the adjustSetting(i) function in SSS.cpp
  //setting[??] = value //Schematic Port Name (PCB Pin Number): Connector Pin Number, Wire Application, Wire Color, Fault Cleared  
  sss.settings[0] = 25; //U1-P0W (J16-9): P2-38, Air Inlet Temp, Green/White
  sss.settings[1] = 50; //U1-P1W (J16-10): P2-84, Coolant Temp, Red/Blue
  sss.settings[2] = 50; //U1-P2W (J16-11): P2-95, Inlet Manifold Air Temp, Brown/Blue
  sss.settings[3] = 10; //U1-P3W (J16-13): P2-96, Fuel Temp, Yellow/White
  sss.settings[4] = 50; //U2-P0W (J18-2): P1-49, Coolant level Norm (4 wire), Grey/Black
  sss.settings[5] = 00; //U2-P1W (J18-3): P1-26, Coolant level normal (2 wire), White/Black
  sss.settings[6] = 00; //U2-P2W (J18-6): P1-19, Output # 6, Yellow/Black
  sss.settings[7] = 00; //U2-P3W (J18-7): P1-20, Output # 7, Green/Black
  sss.settings[8] = 05; //U3-P0W (J10-4): P2-5, Coolant Diverter Solenoid/ADR Valve, Tan
  sss.settings[9] = 30; //U3-P1W (J10-9): P2-41, ARD Pilot Fuel Pressure, Pink/Black
  sss.settings[10] = 55; //U3-P2W (J10-5): P2-40, Fuel Pressure, Tan/White
  sss.settings[11] = 50; //U3-P3W (J10-10): P2-89, ARD Fuel Pressure, White/Black
  sss.settings[12] = 50; //U4-P0W (J24-15): P2-13, DPF inlet temperature (Double), Blue
  sss.settings[13] = 50; //U4-P1W (J24-16): P2-62, DPF outlet temperature (Double), Brown/White
  sss.settings[14] = 50; //U4-P2W (J24-17): P2-56, DPF inlet temperature (Single), Orange/White
  sss.settings[15] = 50; //U4-P3W (J24-18): P2-94, DPF outlet temperature (Single), Purple/Black
  sss.settings[16] = 50; //U5-P0W (J10-7):  P2-27, ARD MAF Delta Pressure, Brown
  sss.settings[17] = 07; //U5-P1W (J24-9):  P2-66, DPF Pressure Sensor (Single), Green/Black
  sss.settings[18] = 10; //U5-P2W (J24-10): P2-85, DPF pressure Sensor (Double), Blue/Black
  sss.settings[19] = 50; //U5-P3W (J24-14): P2-57, CGI Absolute pressure, Purple/White

  //The following settings are in milliVolts
   sss.settings[20] = 2500; //VoutA (J24-19): P2-28, Oil Press, Red/White, 
  sss.settings[21] = 2000; //VoutB (J24-20): P2-33, Engine Crank Case Pressure, Black/White 
  sss.settings[22] = 3000; //VoutC (J24-21): P2-15, Inlet Manifold Air Pressure (Boost Pressure), Orange
  sss.settings[23] = 2000; //VoutD (J24-22): P2-74, Intake Valve actuation oil  Press, Tan/Black
  
  //The following settings are in duty cycle, so 50 = 2.5V average
  sss.settings[24] = 50; //PWM1 (J24-23) P2-16, CGI Delta Pressure, Purple
  sss.settings[25] = 35; //PWM2(Not Connected on Rev 10. May need to move to E2WS1)
    
  // for the following switch settings 0 = connected (closed) and 1 = disconnected (open).
  // These are normally closed switches. Uses 120 ohm resistor
  sss.settings[26] = 0; //J1939 Terminating Resistor #1
  sss.settings[27] = 0; //J1939 Terminating Resistor #2
  sss.settings[28] = 0; //CAN2 Terminating Resistor #1
  sss.settings[29] = 0; //CAN2 Terminating Resistor #2
  sss.settings[30] = 0; //CAN3 Terminating Resistor #1
  sss.settings[31] = 0; //CAN3 Terminating Resistor #2
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
  sss.settings[32] = 0; //V2WS1 (J18-12):
  sss.settings[33] = 0; //V2WS2 (J18-8): P1-54, Coolant level Low (4 Wire), Grey
  sss.settings[34] = 0; //E2WS1 (J24-5):
  sss.settings[35] = 0; //E2WS2 (J24-6): P2-91, ARD outlet Temperature sensor (EG #2), Orange/Black
  sss.settings[36] = 0; //E2WS3 (J24-7): P2-63, ARD Flame Boundary Temp (EG #3), Red/Black
  sss.settings[37] = 0; //E2WS4 (J24-8): P2-55, ARD Turbo Outlet Temp (EG #1), Blue/White
  
  //For the following switches on the high side of the potentiometer, 0 = connected to +5V, 1 = disconnected.
  sss.settings[38] = 0; //VccSelectU1-0 (J16-9): 
  sss.settings[39] = 0; //VccSelectU1-1 (J16-10): 
  sss.settings[40] = 0; //VccSelectU1-2 (J16-11): 
  sss.settings[41] = 0; //VccSelectU1-3 (J16-13): 
  sss.settings[42] = 0; //VccSelectU2-0 (J18-2):  
  sss.settings[43] = 0; //VccSelectU2-1 (J18-3):  
  sss.settings[44] = 0; //VccSelectU2-2 (J18-6): 
  sss.settings[45] = 0; //VccSelectU2-3 (J18-7): 
  sss.settings[46] = 1; //VccSelectU3-0 (J10-4): 
  sss.settings[47] = 1; //VccSelectU3-1 (J10-9): 
  sss.settings[48] = 0; //VccSelectU3-2 (J10-5):  
  sss.settings[49] = 1; //VccSelectU3-3 (J10-10):
  sss.settings[50] = 0; //VccSelectU4-0 (J24-15): 
  sss.settings[51] = 0; //VccSelectU4-1 (J24-16): 
  sss.settings[52] = 0; //VccSelectU4-2 (J24-17): 
  sss.settings[53] = 0; //VccSelectU4-3 (J24-18): 
  sss.settings[54] = 0; //VccSelectU5-0 (J10-7):  
  sss.settings[55] = 0; //VccSelectU5-1 (J24-9):   
  sss.settings[56] = 1; //VccSelectU5-2 (J24-10):  
  sss.settings[57] = 1; //VccSelectU5-3 (J24-14): 
 
  //For the following switches on the low side of the potentiometer, 0 = connected to ground, 1 = disconnected.
  sss.settings[58] = 0; //GroundSelectU1-0 (J16-9): 
  sss.settings[59] = 0; //GroundSelectU1-1 (J16-10): 
  sss.settings[60] = 0; //GroundSelectU1-2 (J16-11): 
  sss.settings[61] = 0; //GroundSelectU1-3 (J16-13): 
  sss.settings[62] = 0; //GroundSelectU2-0 (J18-2):  
  sss.settings[63] = 0; //GroundSelectU2-1 (J18-3):  
  sss.settings[64] = 0; //GroundSelectU2-2 (J18-6): 
  sss.settings[65] = 0; //GroundSelectU2-3 (J18-7): 
  sss.settings[66] = 0; //GroundSelectU3-0 (J10-4): 
  sss.settings[67] = 0; //GroundSelectU3-1 (J10-9): 
  sss.settings[68] = 0; //GroundSelectU3-2 (J10-5): 
  sss.settings[69] = 0; //GroundSelectU3-3 (J10-10): 
  sss.settings[70] = 0; //GroundSelectU4-0 (J24-15): 
  sss.settings[71] = 0; //GroundSelectU4-1 (J24-16): 
  sss.settings[72] = 0; //GroundSelectU4-2 (J24-17): 
  sss.settings[73] = 0; //GroundSelectU4-3 (J24-18): 
  sss.settings[74] = 0; //GroundSelectU5-0 (J10-7):  
  sss.settings[75] = 0; //GroundSelectU5-1 (J24-9):   
  sss.settings[76] = 1; //GroundSelectU5-2 (J24-10):  
  sss.settings[77] = 0; //GroundSelectU5-3 (J24-14): 
  
  //CAN2 Panel Select - 0 = CAN2 on 9-pin is connected to E-CAN, 1 = disconnected
  sss.settings[78] = 0; //CAN2 pn 9 pin connector
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
 sss.settings[79] = 0; //Coil 1 (J16-16):
  sss.settings[80] = 0; //Coil 2 (J16-15):
  sss.settings[81] = 0; //Coil 3 (J16-14):
  sss.settings[82] = 0; //Coil 4 (J24-24): P2-70, CGI Temperature, Yellow/Black
  
  
  //Sample Commands:
  //
  //CAN first ID byte
  //
  //1XXX XXXX = Reset messages 
  //X1XX XXXX = Sends Command to primary (0x40)
  //XX1X XXXX = Use second channel
  //XX0X XXXX = Use first channel (J1939)
  //
  //so F8 will send an id that starts with x18 to the primary processor after resetting the messages and use the seconf processor.
  //
  //CAN18FEF1230200aabbccddeeff1122 creates a CAN message with ID x18FEF123 that is broadcast with a period of 200 ms 
  //  with data  aabbccddeeff1122
  //
  //If the leading id nibble is a 4, then the CAN message is sent over i2c to the other processor to be transmitted.
  //eg. CAN413EF1230150AABBCCDDEEFF1122
  //
  //This command has all zeros for the time period. This is caught and set to a minimum of 10 ms.
  //CAN413EF1220000AABBCCDDEEFF1122
  //
  //Type can8 to reset the can messages.
  //
  //re resets the CAN and wipers to default
  //
  //setC45 sets U5-3 wiper to 45%
  //
  //^r Toggles the connection to VCC 
  //
  //the first nibble of the CAN message tells the SSS where to put the message.  
  //Construct new CAN messages using the format below. Each CAN message needs to have all 3 lines.
 
  String commandString = "CAN18FEF52110000000000000000000"; // PGN 65269 Ambient Conditions 1000 = 1 second period
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  
  for (int h = 0; h < sss.numCommands; h++) adjustSetting(h);
  Serial.println("Main Board Program for ATmega2560 Processor.");
  Serial.println(sss.IDstring);
  Serial.println("Finished Starting Up... Type a command:");
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
void loop(){
  //Check for new commands on the serial bus
  if (Serial.available()>0) 
  {
    int nDataBytes = Serial.readBytesUntil(sss.separatorChar,sss.command,99);
    sss.processCommand(nDataBytes);
  }
  
  //Transmit periodic CAN messages that are stored in memory.
  sss.sendCANmessages();
  
  //reads to see if component ID is available:
  sss.processCAN1message();
  
  
} //end loop()



