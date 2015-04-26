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
    SSS();
    void processCommand();
  private:
    //Set up DAC values for the MCP4728
    _int LDACPin = 41;
    _int DACAddress = 0x61;
    _int daughterDACAddress = 0x62;
};

#endif
   
