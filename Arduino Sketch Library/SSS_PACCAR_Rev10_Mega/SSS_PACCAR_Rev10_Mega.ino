/*
Title: Smart Sensor Simulator Firmware for the ATMega2560 Processor on an SSS Rev 10 board
Comments are for the wiring of an SSS for a DDEC 10.
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
  sss.IDstring = "SYNER*SSS-PACCAR-MX*1RX0001     "; //Change this to match the case of the SSS you are programming. Be sure to include trailing spaces.
  sss.IDstring.toCharArray(sss.compID,29); // Convert the component ID to a character array for CAN

  //Adjust the settings below to match the particular Smart Sensor Simulator that is being programmed. These settings are interpreted by the adjustSetting(i) function in SSS.cpp
  //setting[??] = value //Schematic Port Name (PCB Pin Number): Connector Pin Number, Wire Application, Wire Color, Fault Cleared  
  sss.settings[0] = 26;  //q U1-P0W (J16-9): 
  sss.settings[1] = 15;  //w U1-P1W (J16-10): 
  sss.settings[2] = 1;  //e U1-P2W (J16-11): C9, EGR Temperature, Yellow/Black 
  sss.settings[3] = 1;  //r U1-P3W (J16-13):  C11, TEMP Sensor After BPV Signal, Grey/Black
  sss.settings[4] = 75;  //t U2-P0W (J18-2): 
  sss.settings[5] = 70;  //y U2-P1W (J18-3): B41, PThrottle Sense #2, White
  sss.settings[6] = 30;   //u U2-P2W (J18-6): 
  sss.settings[7] = 50;  //i U2-P3W (J18-7): 
  sss.settings[8] = 50;  //a U3-P0W (J10-4): 
  sss.settings[9] = 50;  //s U3-P1W (J10-9): B41, PThrottle Sense #2, White
  sss.settings[10] = 70; //d U3-P2W (J10-5): 
  sss.settings[11] = 50;  //f U3-P3W (J10-10): B45, PThrottle Sense #1, Pink/Black
  sss.settings[12] = 10; //g U4-P0W (J24-15): 
  sss.settings[13] = 50; //h U4-P1W (J24-16): C81, Exhaust Pressure Sensor After BPV Signal, Brown/Black
  sss.settings[14] = 50; //j U4-P2W (J24-17): C83, Pressure Before Turbo Signal, Red/Black 
  sss.settings[15] = 20; //k U4-P3W (J24-18): C84, Oil Pressure Signal, Green/White
  sss.settings[16] = 0;  //z U5-P0W (J10-7):  
  sss.settings[17] = 5; //x U5-P1W (J24-9):  
  sss.settings[18] = 10; //c U5-P2W (J24-10): 
  sss.settings[19] = 20; //v U5-P3W (J24-14): 

  //The following settings are in milliVolts
  sss.settings[20] = 400; //b VoutA (J24-19): 
  sss.settings[21] = 1800; //n VoutB (J24-20): 
  sss.settings[22] = 1000; //m VoutC (J24-21): 
  sss.settings[23] = 2000; //l VoutD (J24-22): 
  
  //The following settings are in duty cycle, so 50 = 2.5V average
  sss.settings[24] = 0; //PWM1 (J24-23): 
  sss.settings[25] = 35; //PWM2(Not Connected on Rev 10. May need to move to E2WS1)
    
  // for the following switch settings 0 = connected (closed) and 1 = disconnected (open).
  // These are normally closed switches. Uses 120 ohm resistor
  sss.settings[26] = 0; //J1939 Terminating Resistor #1
  sss.settings[27] = 1; //J1939 Terminating Resistor #2
  sss.settings[28] = 0; //CAN2 Terminating Resistor #1
  sss.settings[29] = 1; //CAN2 Terminating Resistor #2
  sss.settings[30] = 1; //CAN3 Terminating Resistor #1
  sss.settings[31] = 0; //CAN3 Terminating Resistor #2
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
  sss.settings[32] = 0; //V2WS1 (J18-12):
  sss.settings[33] = 0; //V2WS2 (J18-8): 
  sss.settings[34] = 1; //E2WS1 (J24-5): 
  sss.settings[35] = 1; //E2WS2 (J24-6): 
  sss.settings[36] = 1; //E2WS3 (J24-7):
  sss.settings[37] = 1; //E2WS4 (J24-8):
  
  //For the following switches on the high side of the potentiometer, 0 = connected to +5V, 1 = disconnected.
  sss.settings[38] = 0; //VccSelectU1-0 (J16-9): 
  sss.settings[39] = 0; //VccSelectU1-1 (J16-10):
  sss.settings[40] = 1; //VccSelectU1-2 (J16-11): C9, EGR Temperature, Yellow/Black 
  sss.settings[41] = 1; //VccSelectU1-3 (J16-13): C11, TEMP Sensor After BPV Signal, Grey/Black
  sss.settings[42] = 0; //VccSelectU2-0 (J18-2): 
  sss.settings[43] = 0; //VccSelectU2-1 (J18-3): 
  sss.settings[44] = 0; //VccSelectU2-2 (J18-6): 
  sss.settings[45] = 1; //VccSelectU2-3 (J18-7): 
  sss.settings[46] = 1; //VccSelectU3-0 (J10-4): 
  sss.settings[47] = 0; //VccSelectU3-1 (J10-9): 
  sss.settings[48] = 1; //VccSelectU3-2 (J10-5): 
  sss.settings[49] = 0; //VccSelectU3-3 (J10-10): 
  sss.settings[50] = 0; //VccSelectU4-0 (J24-15):
  sss.settings[51] = 0; //VccSelectU4-1 (J24-16): C81, Exhaust Pressure Sensor After BPV Signal, Brown/Black
  sss.settings[52] = 0; //VccSelectU4-2 (J24-17): C83, Pressure Before Turbo Signal, Red/Black 
  sss.settings[53] = 0; //VccSelectU4-3 (J24-18): C84, Oil Pressure Signal, Green/White
  sss.settings[54] = 1; //VccSelectU5-0 (J10-7):
  sss.settings[55] = 0; //VccSelectU5-1 (J24-9):
  sss.settings[56] = 0; //VccSelectU5-2 (J24-10):
  sss.settings[57] = 1; //VccSelectU5-3 (J24-14):
 
  //For the following switches on the low side of the potentiometer, 0 = connected to ground, 1 = disconnected.
  sss.settings[58] = 0; //GroundSelectU1-0 (J16-9): 
  sss.settings[59] = 0; //GroundSelectU1-1 (J16-10):
  sss.settings[60] = 0; //GroundSelectU1-2 (J16-11): C9, EGR Temperature, Yellow/Black 
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
  sss.settings[76] = 0; //GroundSelectU5-2 (J24-10):
  sss.settings[77] = 0; //GroundSelectU5-3 (J24-14):
  
  //CAN2 Panel Select - 0 = CAN2 on 9-pin is connected to E-CAN, 1 = disconnected
  sss.settings[78] = 0; //CAN2 pin 9 pin connector
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
  sss.settings[79] = 1; //Coil 1 (J16-16): MCM120-37, Variable Speed Water Pump, Tan/Black 
  sss.settings[80] = 1; //Coil 2 (J16-15): MCM120-35, Turbocharger control, Purple/Blue
  sss.settings[81] = 1; //Coil 3 (J16-14): MCM120-33, Two-speed Fan or Variable-speed Fan, Tan
  sss.settings[82] = 1; //Coil 4 (J24-24): MCM120-61, EGR Valve, Pink/Blue
  
  
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
  
  commandString = "CAN38FEF13101000000000000000000";
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN38F0010b01000000000000000000"; //EBC1
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
 
  commandString = "CAN38FEBF0b01000000000000000000"; //EBC2
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18FE4F0b01000000000000000000"; //VDC1
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN38F0233D005000000000000000000"; //A1SCRDSI1
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN38F0253D00500000000000000000"; //A1SCRA1
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN38F0245500500000000000000000"; //A1SCRDSR1
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN38FD413D05000000000000000000"; //A1SCRDSI2
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN38FD405505000000000000000000"; //A1SCRDSR2
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN38FD3C3D10000000000000000000"; //A1SCRRSI1
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18F0090B00100000000000000000"; //VDC2
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



