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
#include "I2C.h"


//Define slave select pins for SPI
const int CSU1Pin = 81;
const int CSU2Pin = 82;
const int CSU3Pin = 83;
const int CSU4Pin = 38;
const int CSU5Pin = 40;

    
const int LDACPin = 41;
const int DACAddress = 0x61;
void setDAC();
void setPinModes();
void adjustSetting(int i);

const float rAB_ohms = 10000.00; // 10k Ohm
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
const int PWMPin4 = 4; //PG5 in Rev 9b



class SSS
{
  public:
    SSS();
    
    //declare functions
    //int setDAC1(int _mVA, int _mVB, int _mVC, int _mVD);
    //void setDAC();
    void processCommand(int numDataBytes);
    //void adjustSetting(int i);
    //int potFullScale = 100;
    boolean isIgnitionOn();
    void printHelp();
    void buildCANmessage();
    int lookupIndex(char c);
    void sendComponentInfo(char compID[29]);
    
    boolean CAN1Received = false;
    boolean CAN3Received = false;
    void processCAN1message();    
    void processCAN3message();  
    void MCP2515RX1Int();
    void MCP2515RX3Int();
    
    char compID[29];
    
    int potFullScale = 100;
    void sendCANmessages();

    char command[100];
    String modelNumber = "SSS-XXXX";
    String serialNumber = "1R90000";
    const char separatorChar = ',';
    
     //declare instances of the CAN class
    MCP_CAN CAN1 = MCP_CAN(6); // Set CS to PH3
    MCP_CAN CAN3 = MCP_CAN(7); // Set CS to PH4
    I2C I2c = I2C();
    
    // declare instances of digital potentiometer class
    // MCP4261 Mcp4261_U1 = MCP4261( _CSU1Pin, _rAB_ohms );
    // MCP4261 Mcp4261_U2 = MCP4261( _CSU2Pin, _rAB_ohms );
    // MCP4261 Mcp4261_U3 = MCP4261( _CSU3Pin, _rAB_ohms );
    // MCP4261 Mcp4261_U4 = MCP4261( _CSU4Pin, _rAB_ohms );
    // MCP4261 Mcp4261_U5 = MCP4261( _CSU5Pin, _rAB_ohms );
     //declare instances of digital potentiometer class
    MCP4261 Mcp4261_U1 = MCP4261(  CSU1Pin,  rAB_ohms );
    MCP4261 Mcp4261_U2 = MCP4261(  CSU2Pin,  rAB_ohms );
    MCP4261 Mcp4261_U3 = MCP4261(  CSU3Pin,  rAB_ohms );
    MCP4261 Mcp4261_U4 = MCP4261(  CSU4Pin,  rAB_ohms );
    MCP4261 Mcp4261_U5 = MCP4261(  CSU5Pin,  rAB_ohms );
    
    unsigned int settings[83]; 
    const float _rAB_ohms = 10000.0;
    const String decrementCommands[83] = {"q","w","e","r","t","y","u","i","a","s","d","f","g","h","j","k","z","x","c","v","b","n","m","o","p","l","`","<",".",">","/","?","[","{","]","}","\\","|","^q","^w","^e","^r","^t","^y","^u","^i","^a","^s","^d","^f","^g","^h","^j","^k","^z","^x","^c","^v","_q","_w","_e","_r","_t","_y","_u","_i","_a","_s","_d","_f","_g","_h","_j","_k","_z","_x","_c","_v","=","1","2","3","4"};
    char headings[83][5]= {};//{'U1-P0','U1-P1','U1-P2','U1-P3','U2-P0','U2-P1','U2-P2','U2-P3','U3-P0','U3-P1','U3-P2','U3-P3','U4-P0','U4-P1','U4-P2','U4-P3','U5-P0','U5-P1','U5-P2','U5-P3','VoutA','VoutB','VoutC','VoutD','PWM1','PWM2','1939A','1939B','CAN2A','CAN2B','CAN3A','CAN3B','V2WS1','V2VS2','E2WS1','E2WS2','E2WS3','E2WS4','U1-0V','U1-1V','U1-2V','U1-3V','U2-0V','U2-1V','U2-2V','U2-3V','U3-0V','U3-1V','U3-2V','U3-3V','U4-0V','U4-1V','U4-2V','U4-3V','U5-0V','U5-1V','U5-2V','U5-3V','U1-0G','U1-1G','U1-2G','U1-3G','U2-0G','U2-1G','U2-2G','U2-3G','U3-0G','U3-1G','U3-2G','U3-3G','U4-0G','U4-1G','U4-2G','U4-3G','U5-0G','U5-1G','U5-2G','U5-3G','CAN2P','Coil1','Coil2','Coil3','Coil4'};
    const unsigned int defaultSettings[83] = {50,50,50,10,50,00,00,00,50,30,55,50,50,50,50,50,50,07,10,65,2500,2000,3000,2000,50,35,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0};
    
  private:    
    
    //declare character or byte arrays (bytes are unsigned chars)
    char value[6];
    byte CANchannel[100];
    byte CANmessages[50][8];
    
    //The following are characters to be converted into numbers. These come from serial commands.
    char ID[9] ={0,0,0,0,0,0,0,0,0};
    char period[5] ={0,0,0,0,0};
    char data1[9]={0,0,0,0,0,0,0,0,0};
    char data2[9]={0,0,0,0,0,0,0,0,0};
    byte buf[8];
    char CANmessage[14] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    byte _rxBuf[8];
    
    byte len = 0;
 
    
    //declare constants
    const char decrementChar = '_';
    const char incrementChar = '^';
    const char commandChar = '*';
    const int numCommands = 83;
    const int CAN1Int = 0; // Interrupt 0 is on pin PE4,INT4,Pin D2. See http){//arduino.cc/en/Reference/attachInterrupt
    const int CAN3Int = 1; // Interrupt 1 is on pin PE5,INT5,Pin D3. See http){//arduino.cc/en/Reference/attachInterrupt
    
    
    //declare long integer arrays
    unsigned long CANIDs[100];
    unsigned long previousCANmillis[100];
    unsigned long IDnumber = 0;
    unsigned long rxId;
    
    //declare booleans
    boolean validHex;
    boolean displayCAN = false;
    
    //declare integers
    int numCANmsgs = 0;
    int CANtxPeriod[100];
    int periodNumber;
    
    
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


extern SSS sss;


#endif
   
