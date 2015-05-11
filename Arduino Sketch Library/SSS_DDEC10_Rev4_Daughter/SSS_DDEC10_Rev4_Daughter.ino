/*
Title: Smart Sensor Simulator Firmware for the ATMega2560 Processor on an SSS Rev 10 board

*/

//Load these libraries

#include <Wire.h> // Digital to Analog converter
#include <mcp_can.h> // J1939 communications
#include <SPI.h> // CAN and Digital Pots
#include <Mcp4261.h> // Digital Potentiometers
#include <SSSdaughter.h> // Pin definitions and helper functions

void setup()
{ 
  
  //Adjust the settings below to match the particular Smart Sensor Simulator that is being programmed. These settings are interpreted by the adjustSetting(i) function in SSS.cpp
 
  //setting[??] = value //Schematic Port Name (PCB Pin Number): Connector Pin Number, Wire Application, Wire Color, SPN  
  sss.settings[0] = 26;  //q U1-P0W (): 
  sss.settings[1] = 20;  //w U1-P1W (J20-14): MCM120-108, Engine Oil Temperature, Green/White, SPN 175, 20 = 76deg f
  sss.settings[2] = 15;  //e U1-P2W (J20-15): MCM120-90, Intake Air Throttle Circuit, Brown/White, SPN 51
  sss.settings[3] = 10;  //r U1-P3W (J20-16): MCM120-113, Low Side Fuel Pressure (LPPO), Black/Blue, 
  sss.settings[4] = 50;  //t U2-P0W (): 
  sss.settings[5] = 55;  //y U2-P1W (J20:18): MCM120-86, Turbo Inlet Temperature, Grey/Blue,
  sss.settings[6] = 20;   //u U2-P2W (J20:19): MCM120-77, Fuel Temperature, Grey/White, SPN 174^u
  sss.settings[7] = 0;   //i U2-P3W (J18-7): 
  sss.settings[8] = 1;  //a U3-P0W (J22-15): ACM120-27, DOC Outlet Temperature, Purple/White, SPN 3250 
  sss.settings[9] = 30;  //s U3-P1W (J10-9): 
  sss.settings[10] = 1; //d U3-P2W (J22-17): ACM120-97, DPF Outlet Temperature, Purple, SPN 3246 
  sss.settings[11] = 0;  //f U3-P3W (J10-10): 
  sss.settings[12] = 95; //g U4-P0W (J24-15): 
  sss.settings[13] = 0; //h U4-P1W (J24-16): 
  sss.settings[14] = 1; //j U4-P2W (J22:21): ACM120-76, SCR Inlet Temperature, Green/Black, SPN 4360 
  sss.settings[15] = 50; //k U4-P3W (J24-18): 
  sss.settings[16] = 0;  //z U5-P0W (J10-7):  CPC1-3, Idle Validation Switch 2 (throttle active), White/Black
  sss.settings[17] = 50; //x U5-P1W (J24-9):  MCM120-54, Engine Oil Pressure Sensor, Brown/Blue, 
  sss.settings[18] = 10; //c U5-P2W (J24-10): MCM120-60, EGR Valve Position, Yellow/Blue, 
  sss.settings[19] = 20; //v U5-P3W (J24-14): MCM120-106,Intake manifold temperature, Red/Black

  
  //The following settings are in milliVolts
  sss.settings[20] = 400; //b VoutA (J24-19): MCM120-78, Rail Pressure, Pink/White, Set to 400mV to prevent QCV valve from driving.
  sss.settings[21] = 1800; //n VoutB (J24-20): MCM120-108, Engine oil Temperature, Green/White, Set to 1800mV gives 85deg F
  sss.settings[22] = 1000; //m VoutC (J24-21): MCM120-90, Intake Air Throttle Actual Position, Brown/White, 1 volt = 0%
  sss.settings[23] = 2000; //l VoutD (J24-22): MCM120-113, Low Side Fuel Pressure Sensor, Black Yellow, No effect.
  
  //The following settings are in duty cycle, so 50 = 2.5V average
  sss.settings[24] = 0; //PWM1 (J24-23): MCM120-47, FAN Speed, Tan/White, set to 0 gives 0 speed and any other gives 5000 rpm
  sss.settings[25] = 35; //PWM2(Not Connected on Rev 10. May need to move to E2WS1)
    
  // for the following switch settings 0 = connected (closed) and 1 = disconnected (open).
  // These are normally closed switches. Uses 120 ohm resistor
  sss.settings[26] = 0; //J1939 Terminating Resistor #1
  sss.settings[27] = 0; //J1939 Terminating Resistor #2
  sss.settings[28] = 1; //CAN2 Terminating Resistor #1
  sss.settings[29] = 1; //CAN2 Terminating Resistor #2
  sss.settings[30] = 0; //CAN3 Terminating Resistor #1
  sss.settings[31] = 0; //CAN3 Terminating Resistor #2
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
  sss.settings[32] = 0; //V2WS1 (J18-12):
  sss.settings[33] = 0; //V2WS2 (J18-8): 
  sss.settings[34] = 1; //E2WS1 (J24-5): MCM120-69, Fuel Cutoff Valve, Purple
  sss.settings[35] = 1; //E2WS2 (J24-6): MCM120-65, HC Dosing Valve, Blue
  sss.settings[36] = 1; //E2WS3 (J24-7): MCM120-66, Jake Brake High, Brown  
  sss.settings[37] = 1; //E2WS4 (J24-8): MCM120-32, Jake Brake Low, White
  
  //For the following switches on the high side of the potentiometer, 0 = connected to +5V, 1 = disconnected.
  sss.settings[38] = 0; //VccSelectU1-0
  sss.settings[39] = 1; //VccSelectU1-1 (J20-14): MCM120-108, Engine Oil Temperature, Green/White, SPN 175
  sss.settings[40] = 0; //VccSelectU1-2 (J20-15): MCM120-90, Intake Air Throttle Circuit, Brown/White, SPN 51 
  sss.settings[41] = 0; //VccSelectU1-3 
  sss.settings[42] = 0; //VccSelectU2-0
  sss.settings[43] = 0; //VccSelectU2-1
  sss.settings[44] = 0; //VccSelectU2-2
  sss.settings[45] = 0; //VccSelectU2-3
  sss.settings[46] = 1; //VccSelectU3-0 (J22-15): ACM120-27, DOC Outlet Temperature, Purple/White, SPN 3250 
  sss.settings[47] = 1; //VccSelectU3-1
  sss.settings[48] = 1; //VccSelectU3-2 (J22-17): ACM120-97, DPF Outlet Temperature, Purple, SPN 3246
  sss.settings[49] = 0; //VccSelectU3-3
  sss.settings[50] = 0; //VccSelectU4-0
  sss.settings[51] = 0; //VccSelectU4-1
  sss.settings[52] = 1; //VccSelectU4-2 (J22:21): ACM120-76, SCR Inlet Temperature, Green/Black, SPN 4360
  sss.settings[53] = 0; //VccSelectU4-3
  sss.settings[54] = 0; //VccSelectU5-0
  sss.settings[55] = 1; //VccSelectU5-1
  sss.settings[56] = 1; //VccSelectU5-2
  sss.settings[57] = 1; //VccSelectU5-3
 
  //For the following switches on the low side of the potentiometer, 0 = connected to ground, 1 = disconnected.
  sss.settings[58] = 0; //GroundSelectU1-0
  sss.settings[59] = 0; //GroundSelectU1-1 (J20-14): MCM120-108, Engine Oil Temperature, Green/White, SPN 175
  sss.settings[60] = 0; //GroundSelectU1-2 (J20-15): MCM120-90, Intake Air Throttle Circuit, Brown/White, SPN 51
  sss.settings[61] = 0; //GroundSelectU1-3 (J16-13): MCM120-57, Water level for fuel water separator, Grey
  sss.settings[62] = 0; //GroundSelectU2-0
  sss.settings[63] = 0; //GroundSelectU2-1
  sss.settings[64] = 0; //GroundSelectU2-2
  sss.settings[65] = 0; //GroundSelectU2-3
  sss.settings[66] = 0; //GroundSelectU3-0 (J22-15): ACM120-27, DOC Outlet Temperature, Purple/White, SPN 3250 
  sss.settings[67] = 0; //GroundSelectU3-1
  sss.settings[68] = 0; //GroundSelectU3-2 (J22-17): ACM120-97, DPF Outlet Temperature, Purple, SPN 3246
  sss.settings[69] = 0; //GroundSelectU3-3
  sss.settings[70] = 1; //GroundSelectU4-0
  sss.settings[71] = 0; //GroundSelectU4-1
  sss.settings[72] = 0; //GroundSelectU4-2 (J22-21): ACM120-76, SCR Inlet Temperature, Green/Black, SPN 4360
  sss.settings[73] = 0; //GroundSelectU4-3
  sss.settings[74] = 0; //GroundSelectU5-0
  sss.settings[75] = 1; //GroundSelectU5-1
  sss.settings[76] = 0; //GroundSelectU5-2
  sss.settings[77] = 0; //GroundSelectU5-3
  
  //CAN2 Panel Select - 0 = CAN2 on 9-pin is connected to E-CAN, 1 = disconnected
  sss.settings[78] = 0; //CAN2 pn 9 pin connector
  
  // For the following switches 1 = pull high to +12V through 10 ohms and 0 = Pulled Down.
  sss.settings[79] = 1; //Coil 1 (J16-16): 
  sss.settings[80] = 1; //Coil 2 (J16-15): MCM120-35, Turbocharger control, Purple/Blue
  sss.settings[81] = 1; //Coil 3 (J16-14): MCM120-33, Two-speed Fan or Variable-speed Fan, Tan
  sss.settings[82] = 1; //Coil 4 (J24-24): MCM120-61, EGR Valve, Pink/Blue
  
  Serial.begin(115200); // Serial to the USB to UART bridge
  Serial.println("Starting Up...");
  Serial.println("Synercon Technologies Smart Sensor Simulator");
  Serial.println("Program running for the SSS Daughterboard.");
 
  
  Serial.println("Setting up CAN4..."); 
  if(sss.CAN4.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN1 init ok!!");
  else Serial.println("CAN0 init fail!!");
 
  
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



