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


class SSS
{
  public:
    SSS(int potFullScale);
    int setDAC(int _mVA, int _mVB, int _mVC, int _mVD);
    int potFullScale;
  private:
    //Set up DAC values for the MCP4728
    int _LDACPin;
    int _DACAddress;
    int _daughterDACAddress;
    int _mVA;
    int _mVB;
    int _mVC;
    int _mVD;
    
    int ignitionPin = 77;

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
   
