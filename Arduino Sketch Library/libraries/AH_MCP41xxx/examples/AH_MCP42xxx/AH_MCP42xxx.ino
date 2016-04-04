
/********************************************************
**  Downloaded from:                                   **
**  http://www.arduino-projekte.de                     **
********************************************************/

#include "AH_MCP41xxx.h"
//#include <SPI.h>

//#define DATAOUT  11   //uno MOSI , IC SI
//#define SPICLOCK 13   //uno SCK  , IC SCK
#define CS   10   //chipselect pin
#define SHDN 9   //shutdown pin
#define RS   8   //reset pin

#define POTIOMETER_0 0
#define POTIOMETER_1 1

byte resistance = 0;
int testIN1 = A0;
int testIN2 = A1;
 

AH_MCP41xxx mcp42010;
 
void setup()
{  
 Serial.begin(9600);
 Serial.println("Setup ready");
 mcp42010.init_MCP42xxx (CS, SHDN, RS);  //initialisation
 
}

void loop()
{
 mcp42010.setValue(resistance, POTIOMETER_0);       
 // value range 0-255 (8-bit)
 delay(50);
 Serial.print("A0: "); 
 Serial.print(resistance);
 Serial.print(": "); 
 if (resistance==255) {resistance=0;}
 int signal = analogRead(testIN1);
 Serial.print(signal);

 mcp42010.setValue(resistance, POTIOMETER_1);  //value range 0-255 (8-bit)
 delay(100);
 Serial.print("A1: "); 
 Serial.print(resistance);
 Serial.print(": "); 
 if (resistance==255) {resistance=0;}
 signal = analogRead(testIN2);
 Serial.print(signal);
 
 //delay(50);
 resistance++;
}
