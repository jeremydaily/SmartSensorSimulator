
/********************************************************
**  Downloaded from:                                   **
**  http://www.arduino-projekte.de                     **
********************************************************/

#include "AH_MCP41xxx.h"
//#include <SPI.h>

//#define DATAOUT  11   //uno MOSI   //IC SI
//#define DATAIN   12   //uno MISO   //IC not used
//#define SPICLOCK 13   //uno SCK    //IC SCK
#define CHIPSELECT 10   //uno ss     //IC CS

byte resistance = 0;

AH_MCP41xxx mcp41010;
 
void setup()
{  
 Serial.begin(9600);
 Serial.println("Setup ready");
 mcp41010.init_MCP41xxx(CHIPSELECT);  //spi pins initialisation
 mcp41010.reset();
}

void loop()
{
 Serial.print(resistance);
 Serial.print(": "); 
 mcp41010.setValue(resistance);  //value range 0-255 (8-bit)
 delay(100);
 resistance++;
 if (resistance==255) {resistance=0;}
 int signal = analogRead(A0);
 Serial.print(signal);
 Serial.print(" ~");
 Serial.print((float)signal/1024*5);
 Serial.println("V"); 
}
