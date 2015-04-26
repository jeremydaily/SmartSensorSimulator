/*
  SSS.cpp - Library for programming the Smart Sensor Simulator.
*/

#include "Ardunio.h"
#include "SSS.h"
#include "Wire.h"
#include "mcp_can.h"
#include "SPI.h"
#include "Mcp4261.h"

SSS::SSS()
{
 
  //Set up DAC values for the MCP4728
  int _LDACPin = 41;
  int _DACAddress = 0x61;
  int _daughterDACAddress = 0x62;
}
void SSS::setDAC(){  //Settings are in millivolts.
  digitalWrite(_LDACPin,LOW);
  for (int j=20; j<24; j++)
  {
    if (settings[j]>5000) settings[j]=5000;
    if (settings[j]<0) settings[j]=0;
  }
    
  int VoutA = map(settings[20],0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  int VoutB = map(settings[21],0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  int VoutC = map(settings[22],0,3606,0,3000); //0x08BB = 2.682V on VoutC
  int VoutD = map(settings[23],0,3606,0,3000); //0x0ABB = 3.298V on VoutD

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

  if (flag != 0) 
  {
    Serial.print("Setting DAC over I2C returned error flag of ");
    Serial.println(flag); 
  }
  delay(10);
  digitalWrite(_LDACPin,HIGH);
}
