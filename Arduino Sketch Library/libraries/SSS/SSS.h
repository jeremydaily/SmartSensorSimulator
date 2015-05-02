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






class SSS
{
  public:
    SSS(int potFullScale);
    int setDAC(int _mVA, int _mVB, int _mVC, int _mVD);
    void processCommand(int numDataBytes);
    void adjustSetting(int i);
    int potFullScale;
    boolean isIgnitionOn();
    void printHelp();
    void buildCANmessage();
    int lookupIndex(char c);
     char value[6];
    char command[100];
    char *commandPointer = command;
    String commandString;
    
    const char decrementChar = '_';
    const char incrementChar = '^';
    const char separatorChar = ',';
    
    const float _rAB_ohms = 10000.00; // 10k Ohm
    MCP4261 Mcp4261_U1 = MCP4261( _CSU1Pin, _rAB_ohms );
    MCP4261 Mcp4261_U2 = MCP4261( _CSU2Pin, _rAB_ohms );
    MCP4261 Mcp4261_U3 = MCP4261( _CSU3Pin, _rAB_ohms );
    MCP4261 Mcp4261_U4 = MCP4261( _CSU4Pin, _rAB_ohms );
    MCP4261 Mcp4261_U5 = MCP4261( _CSU5Pin, _rAB_ohms );
    
    MCP_CAN CAN1 = MCP_CAN(6); // Set CS to PH3
    MCP_CAN CAN3 = MCP_CAN(7); // Set CS to PH4
    
    const int numCommands = 83;
    unsigned int settings[83]; 
    const String decrementCommands[83];
    int numCANmsgs = 0;
byte CANchannel[50];
unsigned long CANIDs[50];
int CANtxPeriod[50];
byte CANmessages[50][8];
    char ID[9] ={0,0,0,0,0,0,0,0,0};
char period[5] ={0,0,0,0,0};
char data1[9]={0,0,0,0,0,0,0,0,0};
char data2[9]={0,0,0,0,0,0,0,0,0};
unsigned long IDnumber = 0;
int periodNumber;
char CANmessage[14] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    boolean validHex;
    boolean displayCAN = false;

  private:
    int _pwm1;
    int _pwm2;
    int _pwm4;
    int _PWMPin1; 
    int _PWMPin2; 
    int _PWMPin4; 
    
    //Set up DAC values for the MCP4728
    int _LDACPin;
    int _DACAddress;
    int _daughterDACAddress;
    int _mVA;
    int _mVB;
    int _mVC;
    int _mVD;
    
    int _ignitionPin;

    //Set up digital potentiometer pins according to the schematic and pin mapping table
    int _VccSelectU1_0;
    int _VccSelectU1_1;
    int _VccSelectU1_2;
    int _VccSelectU1_3;
    int _GroundSelectU1_0;
    int _GroundSelectU1_1;
    int _GroundSelectU1_2;
    int _GroundSelectU1_3;

    int _VccSelectU2_0;
    int _VccSelectU2_1;
    int _VccSelectU2_2;
    int _VccSelectU2_3;
    int _GroundSelectU2_0;
    int _GroundSelectU2_1;
    int _GroundSelectU2_2;
    int _GroundSelectU2_3;

    int _VccSelectU3_0;
    int _VccSelectU3_1;
    int _VccSelectU3_2;
    int _VccSelectU3_3;
    int _GroundSelectU3_0;
    int _GroundSelectU3_1;
    int _GroundSelectU3_2;
    int _GroundSelectU3_3;

    int _VccSelectU4_0;
    int _VccSelectU4_1;
    int _VccSelectU4_2;
    int _VccSelectU4_3;
    int _GroundSelectU4_0;
    int _GroundSelectU4_1;
    int _GroundSelectU4_2;
    int _GroundSelectU4_3;

    int _VccSelectU5_0;
    int _VccSelectU5_1;
    int _VccSelectU5_2;
    int _VccSelectU5_3;
    int _GroundSelectU5_0;
    int _GroundSelectU5_1;
    int _GroundSelectU5_2;
    int _GroundSelectU5_3;

    //Define slave select pins for SPI
    int _CSU1Pin;
    int _CSU2Pin;
    int _CSU3Pin;
    int _CSU4Pin;
    int _CSU5Pin;

    //Resistor Network Selections (pull Up or Pull down)
    int _E2WS1SelectPin;
    int _E2WS2SelectPin;
    int _E2WS3SelectPin;
    int _E2WS4SelectPin;
    int _V2WS1SelectPin;
    int _V2WS2SelectPin;

    //CAN Termination Resistor Selections
    int _J1939Term1Pin;
    int _J1939Term2Pin;
    int _CAN2Term1Pin;
    int _CAN2Term2Pin;
    int _CAN3Term1Pin;
    int _CAN3Term2Pin;
    int _CAN2FrontEnablePin;
};

#endif
   
