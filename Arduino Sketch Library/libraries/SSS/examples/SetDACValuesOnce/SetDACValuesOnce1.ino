/*

*/


#include <SSS.h>
#include <Wire.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Mcp4261.h>


SSS sss = SSS(100);

void setup(){

  sss.setDAC(1000,2000,3000,4000);
}

void loop(){

}

