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

#define LINcsPIN 4
#define LINwakePIN 85

unsigned long currentMicros;
unsigned long previousMicros;
unsigned long serialSend0Micros;
unsigned long serialSend1Micros;
unsigned long serialSend2Micros;
unsigned long serialSend3Micros;
unsigned long serialSend4Micros;

byte LINDataBytes[3];
int i;
byte k;
byte outByte[5];
boolean firstLINtime=true;
boolean LIN0send = false;
boolean LIN1send = false;
boolean LIN2send = false;
boolean LIN3send = false;
boolean LIN4send = false;

void setup()
{ 
  sss.begin();
  sss.IDstring = "SYNER*SSS-DDEC13*1R90004     "; //Change this to match the case of the SSS you are programming. Be sure to include trailing spaces.
  sss.IDstring.toCharArray(sss.compID,29); // Convert the component ID to a character array for CAN

  //Adjust the settings below to match the particular Smart Sensor Simulator that is being programmed. These settings are interpreted by the adjustSetting(i) function in SSS.cpp
  //setting[??] = value //Schematic Port Name (PCB Pin Number): Connector Pin Number, Wire Application, Wire Color, Fault Cleared  
  sss.settings[0] = 26;  //q U1-P0W (J16-9): MCM120-87, Intake Manifold Charge air pressure, Purple/White
  sss.settings[1] = 15;  //w U1-P1W (J16-10): MCM120-109, Differential pressure EGR Yellow/White
  sss.settings[2] = 20;  //e U1-P2W (J16-11): MCM120-110, Engine Coolant Outlet Temperature, Grey/Black, Fault 110
  sss.settings[3] = 10;  //r U1-P3W (J16-13): MCM120-57, Water level for fuel water separator, Grey
  sss.settings[4] = 20;  //t U2-P0W (J18-2): CPC1-4, Accelerator Pedal Position, Tan/Black, 
  sss.settings[5] = 55;  //y U2-P1W (J18-3): CPC3-5, Multifunction, Grey/White, Open 
  sss.settings[6] = 10;   //u U2-P2W (J18-6): CPC4-14, Throttle Position Sensor 2, Purple/White
  sss.settings[7] = 50;  //i U2-P3W (J18-7): CPC1-1, Multifunction, Brown, Open circuit
  sss.settings[8] = 50;  //a U3-P0W (J10-4): CPC4-15, Multiple application, Tan/White
  sss.settings[9] = 20;  //s U3-P1W (J10-9): CPC3-15, Ambient Air Temperature Sensor, Pink/White
  sss.settings[10] = 70; //d U3-P2W (J10-5): CPC4-16, Multiple applications, Tan
  sss.settings[11] = 1;  //f U3-P3W (J10-10): CPC3-11, Low Coolant Level Sensor, Orange/White
  sss.settings[12] = 10; //g U4-P0W (J24-15): MCM120-84, HC doser fuel compensation pressure in, Orange/White
  sss.settings[13] = 20; //h U4-P1W (J24-16): MCM120-80, Coolant inlet temperature, Blue/White
  sss.settings[14] = 20; //j U4-P2W (J24-17): MCM120-111, HC doser fuel line pressure out, Pink/Black
  sss.settings[15] = 20; //k U4-P3W (J24-18): MCM120-119, Charge air temperature, Blue/Black
  sss.settings[16] = 0;  //z U5-P0W (J10-7):  CPC1-3, Idle Validation Switch 2 (throttle active), White/Black
  sss.settings[17] = 5; //x U5-P1W (J24-9):  MCM120-54, Engine Oil Pressure Sensor, Brown/Blue, 
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
  sss.settings[28] = 0; //CAN2 Terminating Resistor #1
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
  sss.settings[38] = 0; //VccSelectU1-0 (J16-9): MCM120-87, Intake Manifold Charge air pressure, Purple/White
  sss.settings[39] = 0; //VccSelectU1-1 (J16-10): MCM120-109, Differential pressure EGR Yellow/White
  sss.settings[40] = 1; //VccSelectU1-2 (J16-11): MCM120-110, Engine Coolant Outlet Temperature, Grey/Black, Fault 110/3 
  sss.settings[41] = 0; //VccSelectU1-3 (J16-13): MCM120-57, Water level for fuel water separator, Grey
  sss.settings[42] = 0; //VccSelectU2-0 (J18-2): CPC1-4, Accelerator Pedal Position, Tan/Black, 
  sss.settings[43] = 1; //VccSelectU2-1 (J18-3): CPC3-5, Multifunction, Grey/White, Open 
  sss.settings[44] = 0; //VccSelectU2-2 (J18-6): CPC4-14, Throttle Position Sensor 2, Purple/White
  sss.settings[45] = 1; //VccSelectU2-3 (J18-7): CPC1-1, Multifunction, Brown, Open circuit
  sss.settings[46] = 1; //VccSelectU3-0 (J10-4): CPC4-15, Multiple application, Tan/White
  sss.settings[47] = 1; //VccSelectU3-1 (J10-9): CPC3-15, Ambient Air Temperature Sensor, Pink/White
  sss.settings[48] = 1; //VccSelectU3-2 (J10-5):  CPC4-16, Multiple applications, Tan
  sss.settings[49] = 1; //VccSelectU3-3 (J10-10): CPC3-11, Low Coolant Level Sensor, Orange/White
  sss.settings[50] = 0; //VccSelectU4-0 (J24-15): MCM120-84, HC doser fuel compensation pressure in, Orange/White
  sss.settings[51] = 1; //VccSelectU4-1 (J24-16): MCM120-80, Coolant inlet temperature, Blue/White
  sss.settings[52] = 0; //VccSelectU4-2 (J24-17): MCM120-111, HC doser fuel line pressure out, Pink/Black
  sss.settings[53] = 1; //VccSelectU4-3 (J24-18): MCM120-119, Charge air temperature, Blue/Black
  sss.settings[54] = 1; //VccSelectU5-0 (J10-7):  CPC1-3, Idle Validation Switch 2 (throttle active), White/Black
  sss.settings[55] = 0; //VccSelectU5-1 (J24-9):  MCM120-54, Engine Oil Pressure Sensor, Brown/Blue, 
  sss.settings[56] = 0; //VccSelectU5-2 (J24-10): MCM120-60, EGR Valve Position, Yellow/Blue, 
  sss.settings[57] = 1; //VccSelectU5-3 (J24-14): MCM120-106, Intake manifold temperature, Red/Black
 
  //For the following switches on the low side of the potentiometer, 0 = connected to ground, 1 = disconnected.
  sss.settings[58] = 0; //GroundSelectU1-0 (J16-9): MCM120-87, Intake Manifold Charge air pressure, Purple/White
  sss.settings[59] = 0; //GroundSelectU1-1 (J16-10): MCM120-109, Differential pressure EGR Yellow/White
  sss.settings[60] = 0; //GroundSelectU1-2 (J16-11): MCM120-110, Engine Coolant Outlet Temperature, Grey/Black, Fault 110/3
  sss.settings[61] = 0; //GroundSelectU1-3 (J16-13): MCM120-57, Water level for fuel water separator, Grey
  sss.settings[62] = 0; //GroundSelectU2-0 (J18-2): CPC1-4, Accelerator Pedal Position, Tan/Black, 
  sss.settings[63] = 0; //GroundSelectU2-1 (J18-3): CPC3-5, Multifunction, Grey/White, Open 
  sss.settings[64] = 0; //GroundSelectU2-2 (J18-6): CPC4-14, Throttle Position Sensor 2, Purple/White
  sss.settings[65] = 0; //GroundSelectU2-3 (J18-7): CPC1-1, Multifunction, Brown, Open circuit
  sss.settings[66] = 0; //GroundSelectU3-0 (J10-4): CPC4-15, Multiple application, Tan/White
  sss.settings[67] = 0; //GroundSelectU3-1 (J10-9): CPC3-15, Ambient Air Temperature Sensor, Pink/White
  sss.settings[68] = 0; //GroundSelectU3-2 (J10-5): CPC4-16, Multiple applications, Tan
  sss.settings[69] = 0; //GroundSelectU3-3 (J10-10): CPC3-11, Low Coolant Level Sensor, Orange/White
  sss.settings[70] = 0; //GroundSelectU4-0 (J24-15): MCM120-84, HC doser fuel compensation pressure in, Orange/White
  sss.settings[71] = 0; //GroundSelectU4-1 (J24-16): MCM120-80, Coolant inlet temperature, Blue/White
  sss.settings[72] = 0; //GroundSelectU4-2 (J24-17): MCM120-111, HC doser fuel line pressure out, Pink/Black
  sss.settings[73] = 0; //GroundSelectU4-3 (J24-18): MCM120-119, Charge air temperature, Blue/Black
  sss.settings[74] = 0; //GroundSelectU5-0 (J10-7):  CPC1-3, Idle Validation Switch 2 (throttle active), White/Black
  sss.settings[75] = 0; //GroundSelectU5-1 (J24-9):  MCM120-54, Engine Oil Pressure Sensor, Brown/Blue, 
  sss.settings[76] = 0; //GroundSelectU5-2 (J24-10): MCM120-60, EGR Valve Position, Yellow/Blue, 
  sss.settings[77] = 0; //GroundSelectU5-3 (J24-14): MCM120-106, Intake manifold temperature, Red/Black
  
  //CAN2 Panel Select - 0 = CAN2 on 9-pin is connected to E-CAN, 1 = disconnected
  sss.settings[78] = 0; //CAN2 pn 9 pin connector
  
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
  
  commandString = "CAN18F0013101000000000000000000"; // Electronic Brake Controller from SA=49 100 = .1 second period
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18F0010B01000000000000000000"; // Electronic Brake Controller from SA=11 100 = .1 second period
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18FEF11701000000000000000000"; //CCVS from SA=23
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
 
  commandString = "CAN18FEF12801000000000000000000"; //CCVS from SA=40
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18FEF12101000000000000000000"; //CCVS from SA=33
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18FEF13101000000000000000000"; //CCVS from SA=49
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18E0001701000000000000000000"; //Cab Message 1 CM1  from SA=23
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18E0001901000000000000000000"; //Cab Message 1 CM1  from SA=25
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18E0002101000000000000000000"; //Cab Message 1 CM1  from SA=33
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18E0002801000000000000000000"; //Cab Message 1 CM1  from SA=40
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN18E0003101000000000000000000"; //Cab Message 1 CM1  from SA=49
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  commandString = "CAN0CFE6E0B00200000000000000000";// High Resolution Wheel Speed  message from Brake Controller
  commandString.toCharArray(sss.command,32);
  sss.processCommand(31);
  
  for (int h = 0; h < sss.numCommands; h++) adjustSetting(h);
  Serial.println("Main Board Program for ATmega2560 Processor.");
  Serial.println(sss.IDstring);
  
  pinMode(LINcsPIN,OUTPUT);
  pinMode(LINwakePIN,INPUT);
  digitalWrite(LINcsPIN,HIGH);
  //digitalWrite(LINwakePIN,HIGH);
  Serial1.begin(19200);
  Serial.println("Started LIN at 19200.");
  
  Serial.println("Finished Starting Up... Type a command:");
 
  k=0;
  firstLINtime =true;
  Serial1.flush();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
void loop(){
  //Check for new commands on the serial bus
  if (Serial.available()>0) 
  {
    int nDataBytes = Serial.readBytesUntil(sss.separatorChar,sss.command,99);
    sss.processCommand(nDataBytes);
  }
  
  
  currentMicros = micros();

//  if (Serial1.available()>=1){
//        byte dataByte = Serial1.read();
//        Serial.print(currentMicros-previousMicros);
//        previousMicros = currentMicros;
//        Serial.print(" 0x");
//        Serial.println(dataByte,HEX);
//      }
//      
//  else {
  if (Serial1.available()>=3) 
  {
    byte firstChar = Serial1.read();
    byte secondChar = Serial1.read() ;
    byte thirdChar = Serial1.read() ;
    if (firstChar == 0x00 && secondChar == 0xF0){
      Serial.println("LIN Start");
      Serial1.write(0xF0);
    }
    else if (firstChar == 0x00 && secondChar == 0x55 && thirdChar == 0x20 && firstLINtime){
        firstLINtime =false;
         Serial1.write(0xFF);
         Serial1.write(0xF);
         Serial1.write(0xFF);
         Serial1.write(0x3F);
         Serial1.write(0x91);
         Serial.println("first LIN Pass");
    }
    else if (firstChar == 0x00 && secondChar == 0x55 && thirdChar == 0x20 && !firstLINtime)//Serial.print(micros());
    {
      outByte[0] = 0x14;
      outByte[1] = (k << 4) + 0x01;
      k+=1;
      outByte[2] = 0x02;
      outByte[3] = 0x0D;
      
      if      (outByte[1]==0x01) outByte[4] =0xBB;
      else if (outByte[1]==0x11) outByte[4] =0xAB;
      else if (outByte[1]==0x21) outByte[4] =0x9B;
      else if (outByte[1]==0x31) outByte[4] =0x8B;
      else if (outByte[1]==0x41) outByte[4] =0x7B;
      else if (outByte[1]==0x51) outByte[4] =0x6B;
      else if (outByte[1]==0x61) outByte[4] =0x5B;
      else if (outByte[1]==0x71) outByte[4] =0x4B;
      else if (outByte[1]==0x81) outByte[4] =0x3B;
      else if (outByte[1]==0x91) outByte[4] =0x2B;
      else if (outByte[1]==0xA1) outByte[4] =0x1B;
      else if (outByte[1]==0xB1) outByte[4] =0x0B;
      else if (outByte[1]==0xC1) outByte[4] =0xFA;
      else if (outByte[1]==0xD1) outByte[4] =0xEA;
      else if (outByte[1]==0xE1) outByte[4] =0xDA;
      else if (outByte[1]==0xF1) outByte[4] =0xCA;
      //else outByte[4] = -outByte[1] + 188;

//      Serial1.write(outByte[0]);
      serialSend0Micros = currentMicros+500;
      LIN0send = true;
      serialSend1Micros = currentMicros+1000;
      LIN1send = true;
      serialSend2Micros = currentMicros+1500;
      LIN2send = true;
      serialSend3Micros = currentMicros+2000;
      LIN3send = true;
      serialSend4Micros = currentMicros+2500;
      LIN4send = true;
      
//      Serial.print("Sent ");
//      for (int q = 0;q < 5;q++){
//        
//        Serial.print(" 0x");
//        Serial.print(outByte[q],HEX);
//      }
//      Serial.println();
    }
  }    

  if (currentMicros - serialSend0Micros >=0 && LIN0send) {
    Serial1.write(outByte[0]);
    LIN0send=false;
  }
  if (currentMicros - serialSend1Micros >=0 && LIN1send) {
    Serial1.write(outByte[1]);
    LIN1send=false;
  }
  if (currentMicros - serialSend2Micros >=0 && LIN2send) {
    Serial1.write(outByte[2]);
    LIN2send=false;
  }
  if (currentMicros - serialSend3Micros >=0 && LIN3send){
    Serial1.write(outByte[3]);
    LIN3send=false;
  }
  if (currentMicros - serialSend4Micros >=0 && LIN4send) {
    Serial1.write(outByte[4]);
    LIN4send=false;
  }
  //}
  
  
  //Transmit periodic CAN messages that are stored in memory.
  sss.sendCANmessages();
  
  //reads to see if component ID is available:
  sss.processCAN1message();
  
  
} //end loop()



