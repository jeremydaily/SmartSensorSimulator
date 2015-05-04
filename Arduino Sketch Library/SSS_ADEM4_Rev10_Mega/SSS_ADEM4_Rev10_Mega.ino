

/*
Title: Smart Sensor Simulator Firmware for the ATMega2560 Processor

Sample Commands:

CAN first ID byte

1XXX XXXX = Reset messages 
X1XX XXXX = Sends Command to primary (0x40)
XX1X XXXX = Use second channel
XX0X XXXX = Use first channel (J1939)

so F8 will send an id that starts with x18 to the primary processor after resetting the messages and use the seconf processor.

CAN18FEF1230200aabbccddeeff1122 creates a CAN message with ID x18FEF123 that is broadcast with a period of 200 ms 
  with data  aabbccddeeff1122

If the leading id nibble is a 4, then the CAN message is sent over i2c to the other processor to be transmitted.
eg. CAN413EF1230150AABBCCDDEEFF1122

This command has all zeros for the time period. This is caught and set to a minimum of 10 ms.
CAN413EF1220000AABBCCDDEEFF1122

Type can8 to reset the can messages.

re resets the CAN and wipers to default

setC45 sets U5-3 wiper to 45%

^r Toggles the connection to VCC 

the first nibble of the CAN message tells the SSS where to put the message.
*/
#include <SSS.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Mcp4261.h>
#include <I2C.h>


MCP4261 Mcp4261_U1 = MCP4261( CSU1Pin, rAB_ohms );
MCP4261 Mcp4261_U2 = MCP4261( CSU2Pin, rAB_ohms );
MCP4261 Mcp4261_U3 = MCP4261( CSU3Pin, rAB_ohms );
MCP4261 Mcp4261_U4 = MCP4261( CSU4Pin, rAB_ohms );
MCP4261 Mcp4261_U5 = MCP4261( CSU5Pin, rAB_ohms );

int pwm_1;
int pwm_2; 

char command[100]; //declare the array to store serial command characters.

String IDstring = "SYNER*SSS-ADEM4*1R90003     ";

void setup()
{ 
  // Set up Serial Connections
  Serial.begin(115200); // Serial to the USB to UART bridge
   
  
  //Adjust the settings below to match the particular Smart Sensor Simulator that is being programmed.
 
  //setting[??] = value //Schematic Port Name, PCB Pin Number, Connector Pin Number, Wire Application, Wire Color, Fault Cleared  
  sss.settings[0] = 50; //U1-P0W, J16-9, P2-38, Air Inlet Temp, Green/White
  sss.settings[1] = 50; //U1-P1W, J16-10, P2-84, Coolant Temp, Red/Blue
  sss.settings[2] = 50; //U1-P2W, J16-11, P2-95, Inlet Manifold Air Temp, Brown/Blue
  sss.settings[3] = 50; //U1-P3W, J16-13, P2-96, Fuel Temp, Yellow/White
  sss.settings[4] = 50; //U2-P0W, J18-2, P1-49, Coolant level Norm (4 wire), Grey/Black
  sss.settings[5] = 50; //U2-P1W, J18-3, P1-26, Coolant level normal (2 wire), White/Black
  sss.settings[6] = 50; //U2-P2W, J18-6, P1-19, Output # 6, Yellow/Black
  sss.settings[7] = 50; //U2-P3W, J18-7, P1-20, Output # 7, Green/Black
  sss.settings[8] = 50; //U3-P0W, J10-4, P2-5, Coolant Diverter Solenoid/ADR Valve, Tan
  sss.settings[9] = 50; //U3-P1W, J10-9, P2-41, ARD Pilot Fuel Pressure, Pink/Black
  sss.settings[10] = 50; //U3-P2W, J10-5, P2-40, Fuel Pressure, Tan/White
  sss.settings[11] = 50; //U3-P3W, J10-10, P2-89, ARD Fuel Pressure, White/Black
  sss.settings[12] = 50; //U4-P0W, J24-15, P2-13, DPF inlet temperature (Double), Blue
  sss.settings[13] = 50; //U4-P1W, J24-16, P2-62, DPF outlet temperature (Double), Brown/White
  sss.settings[14] = 50; //U4-P2W, J24-17, P2-56, DPF inlet temperature (Single), Orange/White
  sss.settings[15] = 50; //U4-P3W, J24-18, P2-94, DPF outlet temperature (Single), Purple/Black
  sss.settings[16] = 50; //U5-P0W, J10-7,  P2-27, ARD MAF Delta Pressure, Brown
  sss.settings[17] = 50; //U5-P1W, J24-9,  P2-66, DPF Pressure Sensor (Single), Green/Black
  sss.settings[18] = 50; //U5-P2W, J24-10, P2-85, DPF pressure Sensor (Double), Blue/Black
  sss.settings[19] = 50; //U5-P3W, J24-14, P2-57, CGI Absolute pressure, Purple/White
  
  //The following settings are in milliVolts
  sss.settings[20] = 1500; //VoutA, J24-20, P2-33, Engine Crank Case Pressure, Black/White 
  sss.settings[21] = 1950; //VoutB, J24-21, P2-15, Inlet Manifold Air Pressure (Boost Pressure), Orange
  sss.settings[22] = 1200; //VoutC, J24-22, P2-74, Intake Valve actuation oil  Press, Tan/Black
  sss.settings[23] = 600; //VoutD, J24-19, P2-28, Oil Press, Red/White, 
  
  //The following settings are in duty cycle, so 50 = 2.5V average
  sss.settings[24] = 50; //PWM1, J24-23, P2-16, CGI Delta Pressure, Purple
  sss.settings[25] = 50; //PWM2,    (Not Connected on Rev 10. May need to move to E2WS1)
    
  // for the following switch settings 0 = connected (closed) and 1 = disconnected (open).
  // These are normally closed switches. Uses 120 ohm resistor
  sss.settings[26] = 0; //J1939 Terminating Resistor #1
  sss.settings[27] = 0; //J1939 Terminating Resistor #2
  sss.settings[28] = 1; //CAN2 Terminating Resistor #1
  sss.settings[29] = 1; //CAN2 Terminating Resistor #2
  sss.settings[30] = 0; //CAN3 Terminating Resistor #1
  sss.settings[31] = 0; //CAN3 Terminating Resistor #2
  
  // For the following switches 0 = pull high to +12V through 10 ohms and 1 = Pulled Down.
  sss.settings[32] = 0; //V2WS1, J18-12
  sss.settings[33] = 0; //V2WS2, J18-8, P1-54, Coolant level Low (4 Wire), Grey
  sss.settings[34] = 0; //E2WS1, J24-5
  sss.settings[35] = 0; //E2WS2, J24-6, P2-91, ARD outlet Temperature sensor (EG #2), Orange/Black
  sss.settings[36] = 0; //E2WS3, J24-7, P2-63, ARD Flame Boundary Temp (EG #3), Red/Black
  sss.settings[37] = 0; //E2WS4, J24-8, P2-55, ARD Turbo Outlet Temp (EG #1), Blue/White
  
  //For the following switches on the high side of the potentiometer, 0 = connected to +5V, 1 = disconnected.
  sss.settings[38] = 0; //VccSelectU1-0, J16
  sss.settings[39] = 0; //VccSelectU1-1
  sss.settings[40] = 0; //VccSelectU1-2
  sss.settings[41] = 0; //VccSelectU1-3
  sss.settings[42] = 0; //VccSelectU2-0
  sss.settings[43] = 0; //VccSelectU2-1
  sss.settings[44] = 0; //VccSelectU2-2
  sss.settings[45] = 0; //VccSelectU2-3
  sss.settings[46] = 0; //VccSelectU3-0
  sss.settings[47] = 0; //VccSelectU3-1
  sss.settings[48] = 0; //VccSelectU3-2
  sss.settings[49] = 0; //VccSelectU3-3
  sss.settings[50] = 0; //VccSelectU4-0
  sss.settings[51] = 0; //VccSelectU4-1
  sss.settings[52] = 0; //VccSelectU4-2
  sss.settings[53] = 0; //VccSelectU4-3
  sss.settings[54] = 0; //VccSelectU5-0
  sss.settings[55] = 0; //VccSelectU5-1
  sss.settings[56] = 0; //VccSelectU5-2
  sss.settings[57] = 0; //VccSelectU5-3
 
  //For the following switches on the low side of the potentiometer, 0 = connected to ground, 1 = disconnected.
  sss.settings[58] = 0; //GroundSelectU1-0
  sss.settings[59] = 0; //GroundSelectU1-1
  sss.settings[60] = 0; //GroundSelectU1-2
  sss.settings[61] = 0; //GroundSelectU1-3
  sss.settings[62] = 0; //GroundSelectU2-0
  sss.settings[63] = 0; //GroundSelectU2-1
  sss.settings[64] = 0; //GroundSelectU2-2
  sss.settings[65] = 0; //GroundSelectU2-3
  sss.settings[66] = 0; //GroundSelectU3-0
  sss.settings[67] = 0; //GroundSelectU3-1
  sss.settings[68] = 0; //GroundSelectU3-2
  sss.settings[69] = 0; //GroundSelectU3-3
  sss.settings[70] = 0; //GroundSelectU4-0
  sss.settings[71] = 0; //GroundSelectU4-1
  sss.settings[72] = 0; //GroundSelectU4-2
  sss.settings[73] = 0; //GroundSelectU4-3
  sss.settings[74] = 0; //GroundSelectU5-0
  sss.settings[75] = 0; //GroundSelectU5-1
  sss.settings[76] = 0; //GroundSelectU5-2
  sss.settings[77] = 0; //GroundSelectU5-3
  
  //CAN2 Panel Select - 0 = CAN2 on 9-pin is connected to E-CAN, 1 = disconnected
  sss.settings[78] = 0; //CAN2 
  
  // For the following switches 0 = pull high to +12V through 10 ohms and 1 = Pulled Down.
  sss.settings[79] = 0; //Coil 1
  sss.settings[80] = 0; //Coil 2
  sss.settings[81] = 0; //Coil 3
  sss.settings[82] = 0; //Coil 4, P2-70, CGI Temperature, Yellow/Black


  Serial.println("Starting Up...");
  Serial.println("Synercon Technologies Smart Sensor Simulator");
  Serial.println(IDstring);
  Serial.println("Originally written by: Jeremy Daily on 6 Feb 2015");
  Serial.println("Last edited by:_____");
  
  Serial.println("Setting up CAN1..."); //J1939
  if(sss.CAN1.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN1 init ok!!");
  else Serial.println("CAN0 init fail!!");
  Serial.println("Setting up CAN3..."); //Used CAN3 in reference to the circuit in the schematics
  if(sss.CAN3.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN3 init ok!!");
  else Serial.println("CAN3 init fail!!");
  
  String commandString = "CAN18FEF521100000000000000000000"; // PGN 65269 Ambient Conditions 1000 = 1 second period
  commandString.toCharArray(command,32);
  sss.processCommand(31);
  
  IDstring.toCharArray(sss.compID,29);

  for (int h = 0; h<83; h++) adjustSetting(h);
  Serial.println("Finished Starting Up... Type a command:");
  
}


  
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


void adjustSetting(int i)
{
 
  Serial.print(i);
  Serial.print(" ");
  Serial.print(sss.decrementCommands[i]);
  Serial.print(" ");
 
  switch (i){
    case 0:
      Mcp4261_U1.wiper0(sss.settings[i]);
      Serial.print("U1-P0 (J16-9): ");
      Serial.println(sss.settings[i]);
      break;
    case 1:
      Mcp4261_U1.wiper1(sss.settings[i]);
      Serial.print("U1-P1 (J16-10): ");
      Serial.println(sss.settings[i]);
      break;
    case 2:
      Mcp4261_U1.wiper2(sss.settings[i]);
      Serial.print("U1-P2 (J16-11): ");
      Serial.println(sss.settings[i]);
      break;
    case 3:
      Mcp4261_U1.wiper3(sss.settings[i]);
      Serial.print("U1-P3 (J16-13)");
      Serial.println(sss.settings[i]);
      break;
    case 4:
      Mcp4261_U2.wiper0(sss.settings[i]);
      Serial.print("U2-P0 (J18-2): ");
      Serial.println(sss.settings[i]);
      break;
    case 5:
      Mcp4261_U2.wiper1(sss.settings[i]);
      Serial.print("U2-P1 (J18-3): ");
      Serial.println(sss.settings[i]);
      break;
    case 6:
      Mcp4261_U2.wiper2(sss.settings[i]);
      Serial.print("U2-P2 (J18-6): ");
      Serial.println(sss.settings[i]);
      break;
    case 7:
      Mcp4261_U2.wiper3(sss.settings[i]);
      Serial.print("U2-P3 (J18-7): ");
      Serial.println(sss.settings[i]);
      break;
    case 8:
      Mcp4261_U3.wiper0(sss.settings[i]);
      Serial.print("U3-P0 (J10-4): ");
      Serial.println(sss.settings[i]);
      break;
    case 9:
      Mcp4261_U3.wiper1(sss.settings[i]);
      Serial.print("U3-P1 (J10-9): ");
      Serial.println(sss.settings[i]);
      break;
    case 10:
      Mcp4261_U3.wiper2(sss.settings[i]);
      Serial.print("U3-P2 (J10-5): ");
      Serial.println(sss.settings[i]);
      break;
    case 11:
      Mcp4261_U3.wiper3(sss.settings[i]);
      Serial.print("U3-P3 (J10-10): ");
      Serial.println(sss.settings[i]);
      break;
    case 12:
      Mcp4261_U4.wiper0(sss.settings[i]);
      Serial.print("U4-P0 (J24-15): ");
      Serial.println(sss.settings[i]);
      break;
    case 13:
      Mcp4261_U4.wiper1(sss.settings[i]);
      Serial.print("U4-P1 (J24-16): ");
      Serial.println(sss.settings[i]);
      break;
    case 14:
      Mcp4261_U4.wiper2(sss.settings[i]);
      Serial.print("U4-P2 (J24-17): ");
      Serial.println(sss.settings[i]);
      break;
    case 15:
      Mcp4261_U4.wiper3(sss.settings[i]);
      Serial.print("U4-P3 (J24-18): ");
      Serial.println(sss.settings[i]);
      break;
    case 16:
      Mcp4261_U5.wiper0(sss.settings[i]);
      Serial.print("U5-P0 (J10-7): ");
      Serial.println(sss.settings[i]);
      break;
    case 17:
      Mcp4261_U5.wiper1(sss.settings[i]);
      Serial.print("U5-P1 (J24-9): ");
      Serial.println(sss.settings[i]);
      break;
    case 18:
      Mcp4261_U5.wiper2(sss.settings[i]);
      Serial.print("U5-P2 (J24-10): ");
      Serial.println(sss.settings[i]);
      break;
    case 19:
      Mcp4261_U5.wiper3(sss.settings[i]);
      Serial.print("U5-P3 (J24-14): ");
      Serial.println(sss.settings[i]);
      break;
    case 20:
      setDAC();
      Serial.print("VoutA (J24-20): ");
      Serial.println(sss.settings[i]);
      break;
    case 21:
      setDAC();
      Serial.print("VoutB (J24-21): ");
      Serial.println(sss.settings[i]);
      break;
    case 22:
      setDAC();
      Serial.print("VoutC (J24-22): ");
      Serial.println(sss.settings[i]);
      break;
    case 23:
      setDAC();
      Serial.print("VoutD (J24-19): ");
      Serial.println(sss.settings[i]);
      break;
    case 24:
      pwm_1 = map(sss.settings[i],0,100,0,255);
      analogWrite(PWMPin1,pwm_1); //192 = 3.70 volts on PWM1
      Serial.print("PWM1 (J24-23): ");
      Serial.println(sss.settings[i]);
      break;
    case 25:
      pwm_2 = map(sss.settings[i],0,100,0,255);
      analogWrite(PWMPin2,pwm_2); //192 = 3.70 volts on PWM1
      Serial.print("PWM2 (J24-24): ");
      Serial.println(sss.settings[i]);
      break;
    case 26:
      digitalWrite(J1939Term1Pin,sss.settings[i]);
      Serial.print("J1939 Term 1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 27:
      digitalWrite(J1939Term2Pin,sss.settings[i]);
      Serial.print("J1939 Term 2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 28:
      digitalWrite(CAN2Term1Pin,sss.settings[i]);
      Serial.print("CAN2 Term 1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 29:
      Serial.print("CAN2 Term 2: ");
      Serial.print(sss.settings[i]);
      digitalWrite(CAN2Term2Pin,sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 30:
      digitalWrite(CAN3Term1Pin,sss.settings[i]);
      Serial.print("CAN3 Term 1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 31:
      digitalWrite(CAN3Term2Pin,sss.settings[i]);
      Serial.print("CAN3 Term 2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 32:
      digitalWrite(V2WS1SelectPin,sss.settings[i]);
      Serial.print("V2WS1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 7 ohms to Vehicle Sense Return (GND)");
      else if (sss.settings[i] == 1) Serial.println(" = 7 ohms to Vehicle Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 33:
      digitalWrite(V2WS2SelectPin,sss.settings[i]);
      Serial.print("V2WS2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 7 ohms to Vehicle Sense Return (GND)");
      else if (sss.settings[i] == 1) Serial.println(" = 7 ohms to Vehicle Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 34:
      digitalWrite(E2WS1SelectPin,sss.settings[i]);
      Serial.print("E2WS1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (sss.settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 35:
      digitalWrite(E2WS2SelectPin,sss.settings[i]);
      Serial.print("E2WS2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (sss.settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 36:
      digitalWrite(E2WS3SelectPin,sss.settings[i]);
      Serial.print("E2WS3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (sss.settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 37:
      digitalWrite(E2WS4SelectPin,sss.settings[i]);
      Serial.print("E2WS4: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (sss.settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 38:
      digitalWrite(VccSelectU1_0,sss.settings[i]);
      Serial.print("VccSelectU1_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 39:
      digitalWrite(VccSelectU1_1,sss.settings[i]);
      Serial.print("VccSelectU1_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 40:
      digitalWrite(VccSelectU1_2,sss.settings[i]);
      Serial.print("VccSelectU1_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 41:
      digitalWrite(VccSelectU1_3,sss.settings[i]);
      Serial.print("VccSelectU1_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 42:
      digitalWrite(VccSelectU2_0,sss.settings[i]);
      Serial.print("VccSelectU2_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 43:
      digitalWrite(VccSelectU2_1,sss.settings[i]);
      Serial.print("VccSelectU2_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 44:
      digitalWrite(VccSelectU2_2,sss.settings[i]);
      Serial.print("VccSelectU2_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 45:
      digitalWrite(VccSelectU2_3,sss.settings[i]);
      Serial.print("VccSelectU2_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 46:
      digitalWrite(VccSelectU3_0,sss.settings[i]);
      Serial.print("VccSelectU3_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 47:
      digitalWrite(VccSelectU3_1,sss.settings[i]);
      Serial.print("VccSelectU3_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 48:
      digitalWrite(VccSelectU3_2,sss.settings[i]);
      Serial.print("VccSelectU3_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 49:
      digitalWrite(VccSelectU3_3,sss.settings[i]);
      Serial.print("VccSelectU3_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 50:
      digitalWrite(VccSelectU4_0,sss.settings[i]);
      Serial.print("VccSelectU4_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 51:
      digitalWrite(VccSelectU4_1,sss.settings[i]);
      Serial.print("VccSelectU4_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 52:
      digitalWrite(VccSelectU4_2,sss.settings[i]);
      Serial.print("VccSelectU4_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 53:
      digitalWrite(VccSelectU4_3,sss.settings[i]);
      Serial.print("VccSelectU4_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 54:
      digitalWrite(VccSelectU5_0,sss.settings[i]);
      Serial.print("VccSelectU5_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 55:
      digitalWrite(VccSelectU5_1,sss.settings[i]);
      Serial.print("VccSelectU5_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 56:
      digitalWrite(VccSelectU5_2,sss.settings[i]);
      Serial.print("VccSelectU5_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 57:
      digitalWrite(VccSelectU5_3,sss.settings[i]);
      Serial.print("VccSelectU5_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 58:
      digitalWrite(GroundSelectU1_0,sss.settings[i]);
      Serial.print("GroundSelectU1_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 59:
      digitalWrite(GroundSelectU1_1,sss.settings[i]);
      Serial.print("GroundSelectU1_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 60:
      digitalWrite(GroundSelectU1_2,sss.settings[i]);
      Serial.print("GroundSelectU1_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 61:
      digitalWrite(GroundSelectU1_3,sss.settings[i]);
      Serial.print("GroundSelectU1_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 62:
      digitalWrite(GroundSelectU2_0,sss.settings[i]);
      Serial.print("GroundSelectU2_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 63:
      digitalWrite(GroundSelectU2_1,sss.settings[i]);
      Serial.print("GroundSelectU2_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 64:
      digitalWrite(GroundSelectU2_2,sss.settings[i]);
      Serial.print("GroundSelectU2_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 65:
      digitalWrite(GroundSelectU2_3,sss.settings[i]);
      Serial.print("GroundSelectU2_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 66:
      digitalWrite(GroundSelectU3_0,sss.settings[i]);
      Serial.print("GroundSelectU3_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 67:
      digitalWrite(GroundSelectU3_1,sss.settings[i]);
      Serial.print("GroundSelectU3_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 68:
      digitalWrite(GroundSelectU3_2,sss.settings[i]);
      Serial.print("GroundSelectU3_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 69:
      digitalWrite(GroundSelectU3_3,sss.settings[i]);
      Serial.print("GroundSelectU3_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 70:
      digitalWrite(GroundSelectU4_0,sss.settings[i]);
      Serial.print("GroundSelectU4_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 71:
      digitalWrite(GroundSelectU4_1,sss.settings[i]);
      Serial.print("GroundSelectU4_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 72:
      digitalWrite(GroundSelectU4_2,sss.settings[i]);
      Serial.print("GroundSelectU4_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 73:
      digitalWrite(GroundSelectU4_3,sss.settings[i]);
      Serial.print("GroundSelectU4_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 74:
      digitalWrite(GroundSelectU5_0,sss.settings[i]);
      Serial.print("GroundSelectU5_0: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 75:
      digitalWrite(GroundSelectU5_1,sss.settings[i]);
      Serial.print("GroundSelectU5_1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 76:
      digitalWrite(GroundSelectU5_2,sss.settings[i]);
      Serial.print("GroundSelectU5_2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 77:
      digitalWrite(GroundSelectU5_3,sss.settings[i]);
      Serial.print("GroundSelectU5_3: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 78:
      digitalWrite(CAN2FrontEnablePin,sss.settings[i]);
      Serial.print("CAN2 Panel Enable: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Not Connected");
      else if (sss.settings[i] == 1) Serial.println(" = CAN2 on Front Panel Connected");
      else Serial.println(" Value out of bounds.");
      break;
    case 79:
      digitalWrite(CAN2FrontEnablePin,sss.settings[i]);
      Serial.print("CAN2 Panel Enable: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Not Connected");
      else if (sss.settings[i] == 1) Serial.println(" = CAN2 on Front Panel Connected");
      else Serial.println(" Value out of bounds.");
      break;
  }
}  
