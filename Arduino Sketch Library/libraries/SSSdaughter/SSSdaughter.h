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
const int CSU5Pin = 80;
 
const int LDACPin = 41;
const int DACAddress = 0x61;
const int daughterDACAddress = 0x60;

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

const int VccSelectU2_0 = 72;
const int VccSelectU2_1 = 73;
const int VccSelectU2_2 = 74;
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

const int VccSelectU5_0 = 69;
const int VccSelectU5_1 = 68;
const int VccSelectU5_2 = 67;
const int VccSelectU5_3 = 66;
const int GroundSelectU5_0 = 65;
const int GroundSelectU5_1 = 64;
const int GroundSelectU5_2 = 63;
const int GroundSelectU5_3 = 62;

const int Coil1Control = 84;
const int Coil2Control = 39;
const int Coil3Control = 40;
const int Coil4Control = 41;
const int Coil5Control = 58;
const int Coil6Control = 59;
const int Coil7Control = 60;
const int Coil8Control = 61;

const int ADC0 = 54;
const int ADC1 = 55;
const int ADC2 = 56;
const int ADC3 = 57;


const int J1939Select = 79;
const int LEDpin = 77;

//CAN Termination Resistor Selections
const int CAN4Term1Pin = 85;
const int CAN4Term2Pin = 4;
 
 
const int PWM1Pin = 10; 
const int PWM2Pin = 11; 
const int PWM3Pin = 12; 
const int PWM4Pin = 13; 
const int PWM5Pin = 6; 
const int PWM6Pin = 7; 
const int PWM7Pin = 8; 
const int PWM8Pin = 9;

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
    void processCAN4message();    
   
    
    
    char compID[29];
    
    
    char command[100];
    const char separatorChar = ',';
    int settings[83]; 

    //declare instances of the CAN class
    MCP_CAN CAN4 = MCP_CAN(53); // 
    
      
    
    
  
  
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
    
    const int _ignitionPin = 5;


};


extern SSS sss;


#endif
   