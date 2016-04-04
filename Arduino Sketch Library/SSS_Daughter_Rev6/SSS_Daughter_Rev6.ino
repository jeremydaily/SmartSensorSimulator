/*
Title: Smart Sensor Simulator Firmware for the ATMega2560 Processor on an SSS Daughter Board - Rev 6

*/

//Load these libraries

#include <Wire.h> // Digital to Analog converter
#include <mcp_can.h> // J1939 communications
#include <SPI.h> // CAN and Digital Pots
#include <Mcp4261.h> // Digital Potentiometers
#include <SSSdaughter6.h> // Pin definitions and helper functions

void setup()
{ 
  sss.begin();
  //Adjust the settings below to match the particular Smart Sensor Simulator that is being programmed. These settings are interpreted by the adjustSetting(i) function in SSS.cpp
 
  //setting[??] = value //Schematic Port Name (PCB Pin Number): Connector Pin Number, Wire Application, Wire Color, SPN  
  sss.settings[0] = 26;  //q U1-P0W (J20-13): Not Used (MCM120-43, White/Blue)
  sss.settings[1] = 20;  //w U1-P1W (J20-14): MCM120-108, Engine Oil Temperature, Green/White, SPN 175, 20 = 76deg f
  sss.settings[2] = 15;  //e U1-P2W (J20-15): MCM120-90, Intake Air Throttle Circuit, Brown/White, SPN 51
  sss.settings[3] = 10;  //r U1-P3W (J20-16): MCM120-113, Low Side Fuel Pressure (LPPO), Black/Blue, 
  sss.settings[4] = 50;  //t U2-P0W (J20-17): ACM120-84, DEF Pressure Sensor, Brown Blue 
  sss.settings[5] = 55;  //y U2-P1W (J20-18): MCM120-86, Turbo Inlet Temperature, Grey/Blue,
  sss.settings[6] = 1;   //u U2-P2W (J20-19): MCM120-77, Fuel Temperature, Grey/White, SPN 174^u
  sss.settings[7] = 20;  //i U2-P3W (J20-20): ACM120-103, DEF Tank Temperature, Orange Black
  sss.settings[8] = 1;   //a U3-P0W (J22-15): ACM120-27, DOC Outlet Temperature, Purple/White, SPN 3250 
  sss.settings[9] = 30;  //s U3-P1W (J22-16): ACM120-85, DEF Metering Unit temperature, Pink/Black 
  sss.settings[10] = 1;  //d U3-P2W (J22-17): ACM120-97, DPF Outlet Temperature, Purple, SPN 3246 
  sss.settings[11] = 1;  //f U3-P3W (J22-18): ACM120-107, DOC Inlet Temperature, White, 
  sss.settings[12] = 50; //g U4-P0W (J22-19): ACM120-87, DOC Inlet Pressure, Blue/Black,
  sss.settings[13] = 1;  //h U4-P1W (J22-20): ACM120-78, SCR Temp Signal Out, Yellow/Black
  sss.settings[14] = 1;  //j U4-P2W (J22-21): ACM120-76, SCR Inlet Temperature, Green/Black, SPN 4360 
  sss.settings[15] = 50; //k U4-P3W (J22-22): ACM120-74, Metering/Dosing Unit Air Pressure, White/Black
  sss.settings[16] = 0;  //z U5-P0W (J12-12): Not Used 
  sss.settings[17] = 50; //x U5-P1W (J12-11): Not Used 
  sss.settings[18] = 10; //c U5-P2W (J12-10): ACM120-14 Purge Process, Pink/Blue, 
  sss.settings[19] = 20; //v U5-P3W (J12-9):  Not Used

   //The following settings are in milliVolts
  sss.settings[20] = 1500; //b Daughter VoutA (J22-12): MCM120-47, Fan Speed, Tan/White 
  sss.settings[21] = 1600; //n Daughter VoutB (J22-13): ACM120-109, DEF Tank Level, Pink
  sss.settings[22] = 1700; //m Daughter VoutC (J22-1): ACM120-72, DPF pressure out signal, Orange/White
  sss.settings[23] = 1800; //l Daughter VoutD (J22-2): ACM120-100, DEF Pressure signal, Purple/Black
  sss.settings[24] = 2300;  //VoutE:
  sss.settings[25] = 2400; //VoutF:
  sss.settings[26] = 2500;  //VoutG:
  sss.settings[27] = 2600;  //VoutH:
  sss.settings[28] = 300;  //VoutI:
  sss.settings[29] = 400;  //VoutJ:
  sss.settings[30] = 5000;  //VoutK:
  sss.settings[31] = 5000;  //VoutL:
  
  // For the following are switches.
  sss.settings[32] = 0; //J1939 Select (Connects CAN4 to J1939).
  sss.settings[33] = 0; //CAN4Term1, Connects Termination resistor across CAN4 channels  
  sss.settings[34] = 1; //CAN4Term2
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
  sss.settings[79] = 1; //Coil 1 (J20-12): ACM120-8, DEF Tank Valve for Coolant, Grey
  sss.settings[80] = 1; //Coil 2 (J20-11): ACM120-34, DEF Tank Coolant Valve Signal, Blue/white
  sss.settings[81] = 1; //Coil 3 (J22-6): ACM120-16, Heater 5 Metering Unit, Yellow/white
  sss.settings[82] = 1; //Coil 4 (J22-10): ACM120-20, DEF Pump Signal, Blue 
  sss.settings[35] = 1; //Coil 5 (J12-3): ACM120-22, DEF Diffuser Heater, Brown
  sss.settings[36] = 1; //Coil 6 (J12-4): ACM120-26, DEF pressure limiting unit / Air Control valve, Green/White
  sss.settings[37] = 1; //Coil 7 (J12-5): ACM120-28, Dosing Valve low side, Pink/White
  sss.settings[78] = 1; //Coil 8 (J12-6): Not Used
  
  //For the following switches on the high side of the potentiometer, 0 = connected to +5V, 1 = disconnected.
  sss.settings[38] = 0; //VccSelectU1-0
  sss.settings[39] = 1; //VccSelectU1-1 (J20-14): MCM120-108, Engine Oil Temperature, Green/White, SPN 175
  sss.settings[40] = 0; //VccSelectU1-2 (J20-15): MCM120-90, Intake Air Throttle Circuit, Brown/White, SPN 51 
  sss.settings[41] = 0; //VccSelectU1-3 (J20-16): MCM120-113, Low Side Fuel Pressure (LPPO), Black/Blue, 
  sss.settings[42] = 0; //VccSelectU2-0 (J20-17): ACM120-84, DEF Pressure Sensor, Brown Blue 
  sss.settings[43] = 1; //VccSelectU2-1 (J20-18): MCM120-86, Turbo Inlet Temperature, Grey/Blue,
  sss.settings[44] = 0; //VccSelectU2-2 (J20-19): MCM120-77, Fuel Temperature, Grey/White, SPN 174^u
  sss.settings[45] = 0; //VccSelectU2-3 (J20-20): ACM120-103, DEF Tank Temperature, Orange Black
  sss.settings[46] = 1; //VccSelectU3-0 (J22-15): ACM120-27, DOC Outlet Temperature, Purple/White, SPN 3250 
  sss.settings[47] = 1; //VccSelectU3-1 (J22-16): ACM120-85, DEF Metering Unit temperature, Pink/Black
  sss.settings[48] = 1; //VccSelectU3-2 (J22-17): ACM120-97, DPF Outlet Temperature, Purple, SPN 3246
  sss.settings[49] = 1; //VccSelectU3-3 (J22-18): ACM120-107, DOC Inlet Temperature, White
  sss.settings[50] = 0; //VccSelectU4-0 (J22-19): ACM120-87, DOC Inlet Pressure, Blue/Black
  sss.settings[51] = 1; //VccSelectU4-1 (J22-20): ACM120-78, SCR Temp Signal Out, Yellow/Black
  sss.settings[52] = 1; //VccSelectU4-2 (J22-21): ACM120-76, SCR Inlet Temperature, Green/Black, SPN 4360
  sss.settings[53] = 0; //VccSelectU4-3 (J22-22): ACM120-74, Metering/Dosing Unit Air Pressure, White/Black
  sss.settings[54] = 0; //VccSelectU5-0 (J12-12): Not Used 
  sss.settings[55] = 1; //VccSelectU5-1 (J12-11): Not Used 
  sss.settings[56] = 1; //VccSelectU5-2 (J12-10): ACM120-14 Purge Process, Pink/Blue, 
  sss.settings[57] = 1; //VccSelectU5-3 (J12-9):  Not Used
 
  //For the following switches on the low side of the potentiometer, 0 = connected to ground, 1 = disconnected.
  sss.settings[58] = 0; //GroundSelectU1-0
  sss.settings[59] = 0; //GroundSelectU1-1 (J20-14): MCM120-108, Engine Oil Temperature, Green/White, SPN 175
  sss.settings[60] = 0; //GroundSelectU1-2 (J20-15): MCM120-90, Intake Air Throttle Circuit, Brown/White, SPN 51
  sss.settings[61] = 0; //GroundSelectU1-3 (J20-16): MCM120-113, Low Side Fuel Pressure (LPPO), Black/Blue, 
  sss.settings[62] = 0; //GroundSelectU2-0 (J20-17): ACM120-84, DEF Pressure Sensor, Brown Blue 
  sss.settings[63] = 0; //GroundSelectU2-1 (J20-18): MCM120-86, Turbo Inlet Temperature, Grey/Blue,
  sss.settings[64] = 0; //GroundSelectU2-2 (J20-19): MCM120-77, Fuel Temperature, Grey/White, SPN 174^u
  sss.settings[65] = 0; //GroundSelectU2-3 (J20-20): ACM120-103, DEF Tank Temperature, Orange Black
  sss.settings[66] = 0; //GroundSelectU3-0 (J22-15): ACM120-27, DOC Outlet Temperature, Purple/White, SPN 3250 
  sss.settings[67] = 0; //GroundSelectU3-1 (J22-16): ACM120-85, DEF Metering Unit temperature, Pink/Black
  sss.settings[68] = 0; //GroundSelectU3-2 (J22-17): ACM120-97, DPF Outlet Temperature, Purple, SPN 3246
  sss.settings[69] = 0; //GroundSelectU3-3 (J22-18): ACM120-107, DOC Inlet Temperature, White
  sss.settings[70] = 0; //GroundSelectU4-0 (J22-19): ACM120-87, DOC Inlet Pressure, Blue/Black
  sss.settings[71] = 0; //GroundSelectU4-1 (J22-20): ACM120-78, SCR Temp Signal Out, Yellow/Black
  sss.settings[72] = 0; //GroundSelectU4-2 (J22-21): ACM120-76, SCR Inlet Temperature, Green/Black, SPN 4360
  sss.settings[73] = 0; //GroundSelectU4-3 (J22-22): ACM120-74, Metering/Dosing Unit Air Pressure, White/Black
  sss.settings[74] = 0; //GroundSelectU5-0 (J12-12): Not Used 
  sss.settings[75] = 0; //GroundSelectU5-1 (J12-11): Not Used 
  sss.settings[76] = 0; //GroundSelectU5-2 (J12-10): ACM120-14 Purge Process, Pink/Blue, 
  sss.settings[77] = 0; //GroundSelectU5-3 (J12-9):  Not Used

  
    
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
  setDAC();
  
  String commandString = "CAN0CFF00520020FFFFFFFFFFFFFFFF"; // SCR Outlet Nox Sensor Signal
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN0CFF00510020FFFFFFFFFFFFFFFF"; // SCR Inlet Nox Sensor Signal
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
 
  for (int h = 0; h<83; h++) adjustSetting(h);
  Serial.println("Daughter Board Program for ATmega2560 Processor.");
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
  
  
} //end loop()



