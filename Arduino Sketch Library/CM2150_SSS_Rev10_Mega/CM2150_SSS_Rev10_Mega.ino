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
  sss.IDstring ="SYNER*SSS-CM2150*1R90001*     "; //Change this to match the case of the SSS you are programming. Be sure to include trailing spaces.
  sss.IDstring.toCharArray(sss.compID,29); // Convert the component ID to a character array for CAN

  //Adjust the settings below to match the particular Smart Sensor Simulator that is being programmed. These settings are interpreted by the adjustSetting(i) function in SSS.cpp
  //setting[??] = value //Schematic Port Name (PCB Pin Number): Connector Pin Number, Wire Application, Wire Color, Fault Cleared  
  sss.settings[0] = 40;  //q U1-P0W (J16-9): K5-25, EGR position A, Tan
  sss.settings[1] = 40;  //w U1-P1W (J16-10): K5-5	EGR position B	Tan/Black
  sss.settings[2] = 40;  //e U1-P2W (J16-11): K5-35	EGR position C	Tan/White
  sss.settings[3] = 50;  //r U1-P3W (J16-13): K5-15	Exhaust gas pressure signal	Brown/White
  sss.settings[4] = 30;  //t U2-P0W (J18-2):  K6-35, Throttle Position Sensor 1 signal, Grey
  sss.settings[5] = 15;  //y U2-P1W (J18-3):  K6-25, Throttle position  sensor 2 signal, Grey/White
  sss.settings[6] = 0;   //u U2-P2W (J18-6): 
  sss.settings[7] = 0;  //i U2-P3W (J18-7): 
  sss.settings[8] = 50;  //a U3-P0W (J10-4): 
  sss.settings[9] = 50;  //s U3-P1W (J10-9): 
  sss.settings[10] = 80; //d U3-P2W (J10-5): K6-34	ADPF diff press sig	Pink/White
  sss.settings[11] = 50;  //f U3-P3W (J10-10): 
  sss.settings[12] = 39; //g U4-P0W (J24-15): 
  sss.settings[13] = 30; //h U4-P1W (J24-16): K5-19	Intake manifold pressure signal	Black/White
  sss.settings[14] = 82; //j U4-P2W (J24-17): K5-34, DPF Pressure sensor, Purple/White
  sss.settings[15] = 19; //k U4-P3W (J24-18): 
  sss.settings[16] = 50;  //z U5-P0W (J10-7):  
  sss.settings[17] = 80; //x U5-P1W (J24-9): K5-20, Barometric pressure signal, Yellow
  sss.settings[18] = 10; //c U5-P2W (J24-10):  K5-27	Engine oil pressure signal	Purple
  sss.settings[19] = 50; //v U5-P3W (J24-14): 

  //The following settings are in milliVolts
  sss.settings[20] = 500; //b VoutA (J24-19): K5-50	Crankcase pressure signal	Orange/White
  sss.settings[21] = 2000; //n VoutB (J24-20): 
  sss.settings[22] = 3000; //m VoutC (J24-21): 
  sss.settings[23] = 4000; //l VoutD (J24-22): 
  
  //The following settings are in duty cycle, so 50 = 2.5V average
  sss.settings[24] = 50; //PWM1 (J24-23): 
  sss.settings[25] = 70; //PWM2
    
  // for the following switch settings 0 = connected (closed) and 1 = disconnected (open).
  // These are normally closed switches. Uses 120 ohm resistor
  sss.settings[26] = 0; //J1939 Terminating Resistor #1
  sss.settings[27] = 1; //J1939 Terminating Resistor #2
  sss.settings[28] = 1; //CAN2 Terminating Resistor #1
  sss.settings[29] = 0; //CAN2 Terminating Resistor #2
  sss.settings[30] = 1; //CAN3 Terminating Resistor #1
  sss.settings[31] = 0; //CAN3 Terminating Resistor #2
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
  sss.settings[32] = 0; //V2WS1 (J18-12):
  sss.settings[33] = 0; //V2WS2 (J18-8): 
  sss.settings[34] = 0; //E2WS1 (J24-5): 
  sss.settings[35] = 0; //E2WS2 (J24-6): 
  sss.settings[36] = 0; //E2WS3 (J24-7):   
  sss.settings[37] = 0; //E2WS4 (J24-8): 
  
  //For the following switches on the high side of the potentiometer, 0 = connected to +5V, 1 = disconnected.
  sss.settings[38] = 0; //VccSelectU1-0 (J16-9): 
  sss.settings[39] = 0; //VccSelectU1-1 (J16-10): 
  sss.settings[40] = 0; //VccSelectU1-2 (J16-11):  
  sss.settings[41] = 0; //VccSelectU1-3 (J16-13): 
  sss.settings[42] = 0; //VccSelectU2-0 (J18-2):  
  sss.settings[43] = 0; //VccSelectU2-1 (J18-3):  
  sss.settings[44] = 0; //VccSelectU2-2 (J18-6): 
  sss.settings[45] = 0; //VccSelectU2-3 (J18-7): 
  sss.settings[46] = 0; //VccSelectU3-0 (J10-4): 
  sss.settings[47] = 0; //VccSelectU3-1 (J10-9): 
  sss.settings[48] = 0; //VccSelectU3-2 (J10-5):  
  sss.settings[49] = 0; //VccSelectU3-3 (J10-10): 
  sss.settings[50] = 0; //VccSelectU4-0 (J24-15): 
  sss.settings[51] = 0; //VccSelectU4-1 (J24-16): 
  sss.settings[52] = 0; //VccSelectU4-2 (J24-17): 
  sss.settings[53] = 0; //VccSelectU4-3 (J24-18): 
  sss.settings[54] = 0; //VccSelectU5-0 (J10-7):  
  sss.settings[55] = 0; //VccSelectU5-1 (J24-9):   
  sss.settings[56] = 0; //VccSelectU5-2 (J24-10):  
  sss.settings[57] = 0; //VccSelectU5-3 (J24-14): 
 
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
  sss.settings[76] = 0; //GroundSelectU5-2 (J24-10): 
  sss.settings[77] = 0; //GroundSelectU5-3 (J24-14): 
  
  //CAN2 Panel Select - 0 = CAN2 on 9-pin is connected to E-CAN, 1 = disconnected
  sss.settings[78] = 0; //CAN2 pn 9 pin connector
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
  sss.settings[79] = 0; //Coil 1 (J16-16): K5-14	Fuel pump actuator signal	Pink
  sss.settings[80] = 0; //Coil 2 (J16-15): 
  sss.settings[81] = 0; //Coil 3 (J16-14): 
  sss.settings[82] = 0; //Coil 4 (J24-24): 
  
  String commandString = "CAN28FF008400250100005300000081";// VGT
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



