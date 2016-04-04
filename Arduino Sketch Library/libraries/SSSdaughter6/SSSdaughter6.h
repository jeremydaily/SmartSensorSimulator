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
#define CSCAN4Pin 17
#define CSCAN5Pin 53

#define CSU1Pin 62
#define CSU2Pin 63
#define CSU3Pin 64
#define CSU4Pin 65
#define CSU5Pin 66
#define CSU6Pin 67
#define CSU7Pin 68
#define CSU8Pin 69
#define CSU9Pin 22
#define CSU10Pin 23
#define CSU11Pin 24
#define CSU12Pin 25
#define CSU13Pin 26
#define CSU14Pin 27
#define CSU15Pin 28
#define CSU16Pin 29
#define CSU17Pin 75
#define CSU18Pin 74
#define CSU19Pin 73
#define CSU20Pin 72
#define CSU21Pin 71
#define CSU22Pin 70
#define CSU23Pin 83
#define CSU24Pin 38

#define Reset1	18

#define Select12VPort1Pin 49
#define Select12VPort2Pin 48
#define Select12VPort3Pin 47
#define Select12VPort4Pin 46
#define Select12VPort7Pin 45
#define Select12VPort8Pin 44
#define Select12VPort9Pin 43
#define Select12VPort10Pin 42

#define ADC0Pin 54
#define Sense12VPin 56
#define IgnitionSensePin 55
#define IgnitionSelectPin 19


#define CAN4Term1Pin 85
#define CAN4Term2Pin 4
#define CAN5Term1Pin 39
#define CAN5Term2Pin 84
#define J1939toCAN4Pin 40
#define J1939toCAN5Pin 41

#define CAN4IntPin 3
#define CAN5IntPin 77

#define LDAC0Pin 79
#define LDAC1Pin 6
#define LDAC2Pin 7
#define LINCSPin	81
#define LINWakePin	82

#define LowRSelectU1Pin 37
#define LowRSelectU2Pin 36
#define LowRSelectU3Pin 35
#define LowRSelectU4Pin 34
#define LowRSelectU5Pin 33
#define LowRSelectU6Pin 32
#define LowRSelectU7Pin 9
#define LowRSelectU8Pin 80
#define GroundSelectU11 16
#define GroundSelectU12 13
#define GroundSelectU10 31
#define GroundSelectU9  30

#define PWM1Pin 5
#define PWM2Pin 2
#define PWM3Pin 8

#define U1_U2_12VSelect 76
#define U11_U12_12VSelect 61
#define U13_U14_12VSelect 10
#define U15_U16_12VSelect 11
#define U17_U18_12VSelect 12
#define U3_U4_12VSelect 57
#define U5_U6_12VSelect 58
#define U7_U8_12VSelect 59
#define U9_U10_12VSelect 60


void setDAC();
void setPinModes();
void setupSerial();
void adjustSetting(int i);

//declare constants
const char decrementChar = '_';
const char incrementChar = '^';
const char commandChar = '*';
const int numCommands = 83;

class SSS
{
  public:
    SSS();
    void begin();
    //declare functions
    void processCommand(int numDataBytes);
    boolean isIgnitionOn();
    void printHelp();
    void buildCANmessage();
    int lookupIndex(char c);
    void sendComponentInfo(char compID[29]);
    void sendCANmessages();
    void processCAN4message();    
   
    
    String IDstring;
     
    char compID[29];
    
    
    char command[100];
    char separatorChar;
    int settings[83]; 

    int numCommands;
   
    
    //declare character or byte arrays (bytes are unsigned chars)
    char value[6];
    byte CANchannel[100];
    byte CANmessages[50][8];
    int numCANmsgs;
    int CANtxPeriod[100];
    int periodNumber;
    
    //The following are characters to be converted into numbers. These come from serial commands.
    char ID[9];
    char period[5];
    char data1[9];
    char data2[9];
    char CANmessage[14];
    byte buf[8];
    byte _rxBuf[8];
    unsigned long CANIDs[100];
    unsigned long previousCANmillis[100];
    unsigned long IDnumber;
    unsigned long rxId;
    byte len;
 
    
    
    
    boolean validHex;
    boolean displayCAN;
    
    int _ignitionPin;


};


extern SSS sss;


#endif
   