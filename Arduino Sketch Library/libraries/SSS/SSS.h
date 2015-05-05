/*
  SSS.h - Library for operating the Synercon Technologies
  Smart Sensor Simulator.
  
  Copyright 2015 Jeremy S. Daily 
  The University of Tulsa

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
 
    http://www.apache.org/licenses/LICENSE-2.0
 
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/
#ifndef SSS_h
#define SSS_h

#include "Arduino.h"
#include "Mcp4261.h"
#include "mcp_can.h"

//See PinMappings.xlxs and the Smart Sensor Simulator Schematics for details.
//Define slave select pins for SPI
const int CSU1Pin = 81;
const int CSU2Pin = 82;
const int CSU3Pin = 83;
const int CSU4Pin = 38;
const int CSU5Pin = 40;
 
const int LDACPin = 41;
const int DACAddress = 0x61;
const int daughterDACAddress = 0x62;


//Define slave select pins for SPI
//Set up digital potentiometer pins according to the schematic and pin mapping table
const int VccSelectU1_0 = 22;
const int VccSelectU1_1 = 23;
const int VccSelectU1_2 = 24;
const int VccSelectU1_3 = 25;
const int GroundSelectU1_0 = 26;
const int GroundSelectU1_1 = 27;
const int GroundSelectU1_2 = 28;
const int GroundSelectU1_3 = 29;

const int VccSelectU2_0 = 15;
const int VccSelectU2_1 = 14;
const int VccSelectU2_2 = 70;
const int VccSelectU2_3 = 71;
const int GroundSelectU2_0 = 72;
const int GroundSelectU2_1 = 73;
const int GroundSelectU2_2 = 74;
const int GroundSelectU2_3 = 75;

const int VccSelectU3_0 = 37;
const int VccSelectU3_1 = 36;
const int VccSelectU3_2 = 35;
const int VccSelectU3_3 = 34;
const int GroundSelectU3_0 = 33;
const int GroundSelectU3_1 = 32;
const int GroundSelectU3_2 = 31;
const int GroundSelectU3_3 = 30;

const int VccSelectU4_0 = 49;
const int VccSelectU4_1 = 48;
const int VccSelectU4_2 = 47;
const int VccSelectU4_3 = 46;
const int GroundSelectU4_0 = 45;
const int GroundSelectU4_1 = 44;
const int GroundSelectU4_2 = 43;
const int GroundSelectU4_3 = 42;

const int VccSelectU5_0 = 62;
const int VccSelectU5_1 = 63;
const int VccSelectU5_2 = 64;
const int VccSelectU5_3 = 65;
const int GroundSelectU5_0 = 66;
const int GroundSelectU5_1 = 67;
const int GroundSelectU5_2 = 68;
const int GroundSelectU5_3 = 39;

const int Coil1Control = 76;
const int Coil2Control = 5;
const int Coil3Control = 79;
const int Coil4Control = 80;

const int LINwake = 85;
const int ADC15 = 69;
const int LINcs = 4;


//Resistor Network Selections (pull Up or Pull down)
const int E2WS1SelectPin = 10;
const int E2WS2SelectPin = 11;
const int E2WS3SelectPin = 12;
const int E2WS4SelectPin = 13;
const int V2WS1SelectPin = 60;
const int V2WS2SelectPin = 61;

//CAN Termination Resistor Selections
const int J1939Term1Pin = 54;
const int J1939Term2Pin = 55;
const int CAN2Term1Pin = 56;
const int CAN2Term2Pin = 57;
const int CAN3Term1Pin = 58;
const int CAN3Term2Pin = 59;
const int CAN2FrontEnablePin = 84;
 
 
const int PWMPin1 = 8; //PH5
const int PWMPin2 = 9; //PH6
//const int PWMPin4 = 4; //PG5 in Rev 9b

const int potFullScale = 100;
const float rAB_ohms = 10000.00; // 10k Ohm


void setDAC();
void setPinModes();
void adjustSetting(int i);


class SSS
{
  public:
    SSS();
    
    //declare functions
    void processCommand(int numDataBytes);
    boolean isIgnitionOn();
    void printHelp();
    void buildCANmessage();
    int lookupIndex(char c);
    void sendComponentInfo(char compID[29]);
    void sendCANmessages();
    void processCAN1message();    
    void processCAN3message();  
    
    
    
    char compID[29];
    
    
    char command[100];
    const char separatorChar = ',';
    unsigned int settings[83]; 

    //declare instances of the CAN class
    MCP_CAN CAN1 = MCP_CAN(6); // Set CS to PH3
    MCP_CAN CAN3 = MCP_CAN(7); // Set CS to PH4
    
      
    
    
  
  
  private:    
    
    //declare character or byte arrays (bytes are unsigned chars)
    char value[6];
    byte CANchannel[100];
    byte CANmessages[50][8];
    int numCANmsgs = 0;
    int CANtxPeriod[100];
    int periodNumber;
    
    //The following are characters to be converted into numbers. These come from serial commands.
    char ID[9] ={0,0,0,0,0,0,0,0,0};
    char period[5] ={0,0,0,0,0};
    char data1[9]={0,0,0,0,0,0,0,0,0};
    char data2[9]={0,0,0,0,0,0,0,0,0};
    char CANmessage[14] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    byte buf[8];
    byte _rxBuf[8];
    unsigned long CANIDs[100];
    unsigned long previousCANmillis[100];
    unsigned long IDnumber = 0;
    unsigned long rxId;
    byte len = 0;
 
    
    //declare constants
    const char decrementChar = '_';
    const char incrementChar = '^';
    const char commandChar = '*';
    const int numCommands = 83;
    
    boolean validHex;
    boolean displayCAN = false;
    
    const int _ignitionPin = 77;


};


extern SSS sss;


#endif
   