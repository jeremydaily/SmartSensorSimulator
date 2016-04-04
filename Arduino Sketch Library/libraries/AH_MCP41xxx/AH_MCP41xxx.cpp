
/*************************************************************************
**  Device: MCP41010/MCP41050/MCP42010/MCP42050                       	**
**  File:   AH_MCP41xxx.h 					    	**
**			  		     				**
**  Created by A. Hinkel 2012-09-30                                 	**
**  downloaded from http://www.arduino-projekte.de			**
**									**
**									**
**  Released into the public domain.  		                    	**
**                                                                  	**
*************************************************************************/


#include "Arduino.h"

#include "AH_MCP41xxx.h"
//#include "SPI.h"
//#include "pins_arduino.h"


//************************************************************************
/*
AH_MCP41xxx::AH_MCP41xxx(int CS)
{

}
*/
//************************************************************************

void AH_MCP41xxx::init_MCP41xxx(int CS)
{
  _CS = CS; 
  pinMode(_CS, OUTPUT); 
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  
  digitalWrite(SCK, LOW);
  digitalWrite(MOSI, LOW);
  digitalWrite(_CS,HIGH); //disable device to start

  //SPCR = 01010000
  //interrupt disabled, spi enabled, msb 1st, master, clk low when idel,
  //sample on leading edge of clk, system clock/4
  SPCR = (1<<SPE)|(1<<MSTR);   //SPI configuration
  delay(10);
}


//************************************************************************

void AH_MCP41xxx::init_MCP42xxx(int CS, int SHDN, int RS)
{
  _CS = CS; 
  _SHDN = SHDN;
  _RS = RS;

  pinMode(_CS, OUTPUT); 
  pinMode(_SHDN, OUTPUT); 
  pinMode(_RS, OUTPUT); 

  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  
  digitalWrite(SCK, LOW);
  digitalWrite(MOSI, LOW);

  digitalWrite(_CS,HIGH); //disable device to start
  digitalWrite(_SHDN,HIGH); 
  digitalWrite(_RS,HIGH); 

  //SPCR = 01010000
  //interrupt disabled, spi enabled, msb 1st, master, clk low when idel,
  //sample on leading edge of clk, system clock/4
  SPCR = (1<<SPE)|(1<<MSTR);   //SPI configuration
  delay(10);
}

//************************************************************************

void AH_MCP41xxx::setValue(byte value)  //0-255
{
  digitalWrite(_CS,LOW);
    //b00010001 = 18d  MCP41xxx
    //b00010011 = 19d  MCP42xxx
    //b00cc00pp   //cc comand bytes, pp potentiometer selection

    spi_transfer(19);     //write config 
    spi_transfer(value);  //write value

  digitalWrite(_CS,HIGH);
  // _lastValue = value;

} 

//************************************************************************
void AH_MCP41xxx::setValue(byte value, int potentiometer)
{
  byte settings;

  digitalWrite(_CS,LOW);

    if (potentiometer == 0) 
       {settings = B00010001;}
    else 
     {
           if (potentiometer == 1) 
              {settings = B00010010;}
           else 
              {settings = B00010011;}
     }
    
    spi_transfer(settings);     //write config 
    spi_transfer(value);        //write value

  digitalWrite(_CS,HIGH);
  // _lastValue = value;

} 

//************************************************************************
byte AH_MCP41xxx::spi_transfer(volatile byte data)
{
   SPDR = data;
   while (!(SPSR & (1<<SPIF))) 
   { 
   };
   return SPSR;
}

//************************************************************************

void AH_MCP41xxx::shutdown(boolean data)
{
   digitalWrite(_SHDN,data); 
}

//************************************************************************

void AH_MCP41xxx::reset()
{
   digitalWrite(_RS,LOW); 
   delay(1);
   digitalWrite(_RS,HIGH); 
}
//************************************************************************

