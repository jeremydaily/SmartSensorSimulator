/*
  SSS.cpp - Library for programming the Smart Sensor Simulator.
*/

#include "Arduino.h"
#include "SSS.h"
#include "Wire.h"
#include "mcp_can.h"
#include "SPI.h"
#include "Mcp4261.h"

SSS::SSS(int potFullScale)
{
    const long accelCalibrationDelay = 30000;

    const int numCommands = 79;
    const char decrementChar = '_';
    const char incrementChar = '^';
    const char separatorChar = ',';

    const char commandChar = '*';
    const String decrementCommands[numCommands] = {"q","w","e","r","t","y","u","i","a","s","d","f","g","h","j","k","z","x","c","v","b","n","m","o","p","l","`","<",".",">","/","?","[","{","]","}","\\","|","^q","^w","^e","^r","^t","^y","^u","^i","^a","^s","^d","^f","^g","^h","^j","^k","^z","^x","^c","^v","_q","_w","_e","_r","_t","_y","_u","_i","_a","_s","_d","_f","_g","_h","_j","_k","_z","_x","_c","_v","="};
    const char incrementCommands[numCommands][2] = {'Q','W','E','R','T','Y','U','I','A','S','D','F','G','H','J','K','Z','X','C','V','B','N','M','O','P','L','`','<','.','>','/','?','[','{',']','}','\\','|','^Q','^W','^E','^R','^T','^Y','^U','^I','^A','^S','^D','^F','^G','^H','^J','^K','^Z','^X','^C','^V','_Q','_W','_E','_R','_T','_Y','_U','_I','_A','_S','_D','_F','_G','_H','_J','_K','_Z','_X','_C','_V','='};
    const char headings[numCommands][5]={'U1-P0','U1-P1','U1-P2','U1-P3','U2-P0','U2-P1','U2-P2','U2-P3','U3-P0','U3-P1','U3-P2','U3-P3','U4-P0','U4-P1','U4-P2','U4-P3','U5-P0','U5-P1','U5-P2','U5-P3','VoutA','VoutB','VoutC','VoutD','PWM1','PWM2','19391','19392','CAN21','CAN22','CAN31','CAN32','V2WS1','V2VS2','E2WS1','E2WS2','E2WS3','E2WS4','U1-0V','U1-1V','U1-2V','U1-3V','U2-0V','U2-1V','U2-2V','U2-3V','U3-0V','U3-1V','U3-2V','U3-3V','U4-0V','U4-1V','U4-2V','U4-3V','U5-0V','U5-1V','U5-2V','U5-3V','U1-0G','U1-1G','U1-2G','U1-3G','U2-0G','U2-1G','U2-2G','U2-3G','U3-0G','U3-1G','U3-2G','U3-3G','U4-0G','U4-1G','U4-2G','U4-3G','U5-0G','U5-1G','U5-2G','U5-3G','CAN2P'};

    //These are the default settings. Change these according to the research for the specific ECM.
    unsigned int defaultSettings[numCommands] = {50,50,50,10,50,00,00,00,50,30,55,50,50,50,50,50,50,07,10,65,2500,2000,3000,2000,50,35,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0};
    unsigned int settings[numCommands];//       {00,01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18,19,0020,0021,0022,0023,24,25,6,7,8,9,3,1,2,3,4,5,6,7,8,9,4,1,2,3,4,5,6,7,8,9,5,1,2,3,4,5,6,7,8,9,6,1,2,3,4,5,6,7,8,9,7,1,2,3,4,5,6,7}

    
    
    //set Up PWM Pins
    const int _PWMPin1 = 8; //PH5
    const int _PWMPin2 = 9; //PH6
    const int _PWMPin4 = 4; //PG5 in Rev 9b

    //set up PWM values
    const int _pwm1 = 0;
    const int _pwm2 = 0;
    const int _pwm4 = 0;


    //Set up DAC values for the MCP4728
    const int _LDACPin = 41;
    const int _DACAddress = 0x61;
    const int _daughterDACAddress = 0x62;
    const int _mVA = 1000;
    const int _mVB = 2000;
    const int _mVC = 3000;
    const int _mVD = 4000;
    
    const int _ignitionPin = 77;

    //Set up digital potentiometer pins according to the schematic and pin mapping table
    const int _VccSelectU1_0 = 22;
    const int _VccSelectU1_1 = 23;
    const int _VccSelectU1_2 = 24;
    const int _VccSelectU1_3 = 25;
    const int _GroundSelectU1_0 = 26;
    const int _GroundSelectU1_1 = 27;
    const int _GroundSelectU1_2 = 28;
    const int _GroundSelectU1_3 = 29;

    const int _VccSelectU2_0 = 15;
    const int _VccSelectU2_1 = 14;
    const int _VccSelectU2_2 = 70;
    const int _VccSelectU2_3 = 71;
    const int _GroundSelectU2_0 = 72;
    const int _GroundSelectU2_1 = 73;
    const int _GroundSelectU2_2 = 74;
    const int _GroundSelectU2_3 = 75;

    const int _VccSelectU3_0 = 37;
    const int _VccSelectU3_1 = 36;
    const int _VccSelectU3_2 = 35;
    const int _VccSelectU3_3 = 34;
    const int _GroundSelectU3_0 = 33;
    const int _GroundSelectU3_1 = 32;
    const int _GroundSelectU3_2 = 31;
    const int _GroundSelectU3_3 = 30;

    const int _VccSelectU4_0 = 49;
    const int _VccSelectU4_1 = 48;
    const int _VccSelectU4_2 = 47;
    const int _VccSelectU4_3 = 46;
    const int _GroundSelectU4_0 = 45;
    const int _GroundSelectU4_1 = 44;
    const int _GroundSelectU4_2 = 43;
    const int _GroundSelectU4_3 = 42;

    const int _VccSelectU5_0 = 62;
    const int _VccSelectU5_1 = 63;
    const int _VccSelectU5_2 = 64;
    const int _VccSelectU5_3 = 65;
    const int _GroundSelectU5_0 = 66;
    const int _GroundSelectU5_1 = 67;
    const int _GroundSelectU5_2 = 68;
    const int _GroundSelectU5_3 = 39;

    //Define slave select pins for SPI
    const int _CSU1Pin = 81;
    const int _CSU2Pin = 82;
    const int _CSU3Pin = 83;
    const int _CSU4Pin = 38;
    const int _CSU5Pin = 40;

    //Resistor Network Selections (pull Up or Pull down)
    const int _E2WS1SelectPin = 10;
    const int _E2WS2SelectPin = 11;
    const int _E2WS3SelectPin = 12;
    const int _E2WS4SelectPin = 13;
    const int _V2WS1SelectPin = 60;
    const int _V2WS2SelectPin = 61;

    //CAN Termination Resistor Selections
    const int _J1939Term1Pin = 54;
    const int _J1939Term2Pin = 55;
    const int _CAN2Term1Pin = 56;
    const int _CAN2Term2Pin = 57;
    const int _CAN3Term1Pin = 58;
    const int _CAN3Term2Pin = 59;
    const int _CAN2FrontEnablePin = 84;
    
    const float _rAB_ohms = 10000.00; // 10k Ohm
    MCP4261 Mcp4261_U1 = MCP4261( _CSU1Pin, _rAB_ohms );
    MCP4261 Mcp4261_U2 = MCP4261( _CSU2Pin, _rAB_ohms );
    MCP4261 Mcp4261_U3 = MCP4261( _CSU3Pin, _rAB_ohms );
    MCP4261 Mcp4261_U4 = MCP4261( _CSU4Pin, _rAB_ohms );
    MCP4261 Mcp4261_U5 = MCP4261( _CSU5Pin, _rAB_ohms );
    
    Mcp4261_U1.scale = potFullScale;
    Mcp4261_U2.scale = potFullScale;
    Mcp4261_U3.scale = potFullScale;
    Mcp4261_U4.scale = potFullScale;
    Mcp4261_U5.scale = potFullScale;

    boolean ignition = false;
    boolean runOnce = true;
    boolean ssState = true;

    char command[100];
    char *commandPointer = command;
    String commandString;
    
    
    pinMode(_ignitionPin, INPUT);
   
    //Initialize the SPI chip pins
    pinMode(_CSU1Pin, OUTPUT);
    pinMode(_CSU2Pin, OUTPUT);
    pinMode(_CSU3Pin, OUTPUT);
    pinMode(_CSU4Pin, OUTPUT);
    pinMode(_CSU5Pin, OUTPUT);

    digitalWrite(_CSU1Pin,HIGH);
    digitalWrite(_CSU2Pin,HIGH);
    digitalWrite(_CSU3Pin,HIGH);
    digitalWrite(_CSU4Pin,HIGH);
    digitalWrite(_CSU5Pin,HIGH);

    // initialize the digital pins.
    pinMode(_PWMPin1, OUTPUT);
    pinMode(_PWMPin2, OUTPUT);
    pinMode(_PWMPin4, OUTPUT);
    pinMode(_LDACPin, OUTPUT);

    digitalWrite(_LDACPin,HIGH);

    //Resistor Network Modes
    pinMode(_E2WS1SelectPin, OUTPUT);
    pinMode(_E2WS2SelectPin, OUTPUT);
    pinMode(_E2WS3SelectPin, OUTPUT);
    pinMode(_E2WS4SelectPin, OUTPUT);
    pinMode(_V2WS1SelectPin, OUTPUT);
    pinMode(_V2WS2SelectPin, OUTPUT);

    //CAN Termination Resistor Modes
    pinMode(_J1939Term1Pin, OUTPUT);
    pinMode(_J1939Term2Pin, OUTPUT);
    pinMode(_CAN2Term1Pin, OUTPUT);
    pinMode(_CAN2Term2Pin, OUTPUT);
    pinMode(_CAN3Term1Pin, OUTPUT);
    pinMode(_CAN3Term2Pin, OUTPUT);

    //Initialize Resistor Network Switches
    //LOW ties all these devices to the Return lines.
    digitalWrite(_E2WS1SelectPin, LOW);
    digitalWrite(_E2WS2SelectPin, LOW);
    digitalWrite(_E2WS3SelectPin, LOW);
    digitalWrite(_E2WS4SelectPin, LOW);
    digitalWrite(_V2WS1SelectPin, LOW);
    digitalWrite(_V2WS2SelectPin, LOW);

    //Initialize CAN Termination Switches
    digitalWrite(_J1939Term1Pin, LOW);
    digitalWrite(_J1939Term2Pin, LOW);
    digitalWrite(_CAN2Term1Pin, LOW);
    digitalWrite(_CAN2Term2Pin, LOW);
    digitalWrite(_CAN3Term1Pin, LOW);
    digitalWrite(_CAN3Term2Pin, LOW);

    pinMode(_GroundSelectU1_0,OUTPUT);
    pinMode(_GroundSelectU1_1,OUTPUT);
    pinMode(_GroundSelectU1_2,OUTPUT);
    pinMode(_GroundSelectU1_3,OUTPUT);
    pinMode(_GroundSelectU2_0,OUTPUT);
    pinMode(_GroundSelectU2_1,OUTPUT);
    pinMode(_GroundSelectU2_2,OUTPUT);
    pinMode(_GroundSelectU2_3,OUTPUT);
    pinMode(_GroundSelectU3_0,OUTPUT);
    pinMode(_GroundSelectU3_1,OUTPUT);
    pinMode(_GroundSelectU3_2,OUTPUT);
    pinMode(_GroundSelectU3_3,OUTPUT);
    pinMode(_GroundSelectU4_0,OUTPUT);
    pinMode(_GroundSelectU4_1,OUTPUT);
    pinMode(_GroundSelectU4_2,OUTPUT);
    pinMode(_GroundSelectU4_3,OUTPUT);
    pinMode(_GroundSelectU5_0,OUTPUT);
    pinMode(_GroundSelectU5_1,OUTPUT);
    pinMode(_GroundSelectU5_2,OUTPUT);
    pinMode(_GroundSelectU5_3,OUTPUT);

    pinMode(_VccSelectU1_0,OUTPUT);
    pinMode(_VccSelectU1_1,OUTPUT);
    pinMode(_VccSelectU1_2,OUTPUT);
    pinMode(_VccSelectU1_3,OUTPUT);
    pinMode(_VccSelectU2_0,OUTPUT);
    pinMode(_VccSelectU2_1,OUTPUT);
    pinMode(_VccSelectU2_2,OUTPUT);
    pinMode(_VccSelectU2_3,OUTPUT);
    pinMode(_VccSelectU3_0,OUTPUT);
    pinMode(_VccSelectU3_1,OUTPUT);
    pinMode(_VccSelectU3_2,OUTPUT);
    pinMode(_VccSelectU3_3,OUTPUT);
    pinMode(_VccSelectU4_0,OUTPUT);
    pinMode(_VccSelectU4_1,OUTPUT);
    pinMode(_VccSelectU4_2,OUTPUT);
    pinMode(_VccSelectU4_3,OUTPUT);
    pinMode(_VccSelectU5_0,OUTPUT);
    pinMode(_VccSelectU5_1,OUTPUT);
    pinMode(_VccSelectU5_2,OUTPUT); 
    pinMode(_VccSelectU5_3,OUTPUT);
  
} 

int SSS::setDAC(int _mVA, int _mVB, int _mVC, int _mVD)
{  //Settings are in millivolts.
  digitalWrite(_LDACPin,LOW);
  if (_mVA>5000) _mVA=5000;
  if (_mVA<0) _mVA=0;
  if (_mVB>5000) _mVB=5000;
  if (_mVB<0) _mVB=0;
  if (_mVC>5000) _mVC=5000;
  if (_mVC<0) _mVC=0;
  if (_mVD>5000) _mVD=5000;
  if (_mVD<0) _mVD=0;
    
  int VoutA = map(_mVA,0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  int VoutB = map(_mVB,0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  int VoutC = map(_mVC,0,3606,0,3000); //0x08BB = 2.682V on VoutC
  int VoutD = map(_mVD,0,3606,0,3000); //0x0ABB = 3.298V on VoutD

  VoutA = constrain(VoutA,0,4095);
  VoutB = constrain(VoutB,0,4095);
  VoutC = constrain(VoutC,0,4095);
  VoutD = constrain(VoutD,0,4095);
  
  Wire.beginTransmission(_DACAddress);
  Wire.write(byte(0x50));             
  Wire.write(highByte(VoutA));             
  Wire.write(lowByte(VoutA));             
  Wire.write(highByte(VoutB));             
  Wire.write(lowByte(VoutB));             
  Wire.write(highByte(VoutC));             
  Wire.write(lowByte(VoutC));             
  Wire.write(highByte(VoutD));             
  Wire.write(lowByte(VoutD));             
  int flag = Wire.endTransmission(); 

  digitalWrite(_LDACPin,HIGH);
  return flag;
}