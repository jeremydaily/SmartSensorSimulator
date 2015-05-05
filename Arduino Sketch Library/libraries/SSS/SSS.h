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

MCP_CAN CAN1(6); // Set CS to PH3
MCP_CAN CAN3(7); // Set CS to PH4

void setDAC();
void setPinModes();
void adjustSetting(int i);
void processCommand(int numDataBytes);
void processCAN1message();
void processCAN3message();
void sendCANmessages();
boolean isIgnitionOn();
int lookupIndex(char c);
void buildCANmessage();
void sendComponentInfo(char id[29]);
void printHelp();

char compID[29];

//Define slave select pins for SPI
const int CSU1Pin = 81;
const int CSU2Pin = 82;
const int CSU3Pin = 83;
const int CSU4Pin = 38;
const int CSU5Pin = 40;

const int LDACPin = 41;
const int DACAddress = 0x61;
int daughterDACAddress = 0x62;

const float rAB_ohms = 10000.00; // 10k Ohm
const int potFullScale = 100;

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
 
const int ignitionPin = 77;
 
const int PWMPin1 = 8; //PH5
const int PWMPin2 = 9; //PH6
const int PWMPin4 = 4; //PG5 in Rev 9b

//CAN Message Structures
char ID[9]; 
char period[5];
char data1[9];
char data2[9];


const char headings[83][5]= {'U1-P0','U1-P1','U1-P2','U1-P3','U2-P0','U2-P1','U2-P2','U2-P3','U3-P0','U3-P1','U3-P2','U3-P3','U4-P0','U4-P1','U4-P2','U4-P3','U5-P0','U5-P1','U5-P2','U5-P3','VoutA','VoutB','VoutC','VoutD','PWM1','PWM2','1939A','1939B','CAN2A','CAN2B','CAN3A','CAN3B','V2WS1','V2VS2','E2WS1','E2WS2','E2WS3','E2WS4','U1-0V','U1-1V','U1-2V','U1-3V','U2-0V','U2-1V','U2-2V','U2-3V','U3-0V','U3-1V','U3-2V','U3-3V','U4-0V','U4-1V','U4-2V','U4-3V','U5-0V','U5-1V','U5-2V','U5-3V','U1-0G','U1-1G','U1-2G','U1-3G','U2-0G','U2-1G','U2-2G','U2-3G','U3-0G','U3-1G','U3-2G','U3-3G','U4-0G','U4-1G','U4-2G','U4-3G','U5-0G','U5-1G','U5-2G','U5-3G','CAN2P','Coil1','Coil2','Coil3','Coil4'};

const int numCommands = 83;
const char decrementChar = '_';
const char incrementChar = '^';
const char separatorChar = ',';

const char commandChar = '*';
String decrementCommands[83] = {"q","w","e","r","t","y","u","i","a","s","d","f","g","h","j","k","z","x","c","v","b","n","m","o","p","l","`","<",".",">","/","?","[","{","]","}","\\","|","^q","^w","^e","^r","^t","^y","^u","^i","^a","^s","^d","^f","^g","^h","^j","^k","^z","^x","^c","^v","_q","_w","_e","_r","_t","_y","_u","_i","_a","_s","_d","_f","_g","_h","_j","_k","_z","_x","_c","_v","=","+","-","(",")"};

unsigned int defaultSettings[83] = {50,50,50,10,50,00,00,00,50,30,55,50,50,50,50,50,50,07,10,65,3500,2000,3000,2000,50,35,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0};
unsigned int settings[83];//       
int i ;

boolean CAN1Received = false;
boolean CAN3Received = false;

//initialize storage for 50 CAN messages to be transmitted.
int numCANmsgs = 0;
byte CANchannel[50];
unsigned long CANIDs[50];
int CANtxPeriod[50];
byte CANmessages[50][8];
unsigned long int previousCANmillis[50];
unsigned long int currentMillis;
unsigned long int previousMillis100;
unsigned long int previousMillis10;
unsigned long int startTime;

byte len = 0;
byte buf[8];
long unsigned int rxId;
byte rxBuf[8];

boolean validHex;
char command[100];
char *commandPointer = command;
String commandString;
unsigned long IDnumber = 0;
char value[6];

// 4 bytes for ID, 2 bytes for time period, and 8 bytes for data
char CANmessage[14] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

int periodNumber;
boolean displayCAN;

MCP4261 Mcp4261_U1 = MCP4261( CSU1Pin, rAB_ohms );
MCP4261 Mcp4261_U2 = MCP4261( CSU2Pin, rAB_ohms );
MCP4261 Mcp4261_U3 = MCP4261( CSU3Pin, rAB_ohms );
MCP4261 Mcp4261_U4 = MCP4261( CSU4Pin, rAB_ohms );
MCP4261 Mcp4261_U5 = MCP4261( CSU5Pin, rAB_ohms );

int pwm1;
int pwm2;

#endif
   
