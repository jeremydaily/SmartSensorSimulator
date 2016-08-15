/*
  Digital Pot Control
  
  This example controls an Analog Devices AD5206 digital potentiometer.
  The AD5206 has 6 potentiometer channels. Each channel's pins are labeled
  A - connect this to voltage
  W - this is the pot's wiper, which changes when you set it
  B - connect this to ground.
 
 The AD5206 is SPI-compatible,and to command it, you send two bytes, 
 one with the channel number (0 - 5) and one with the resistance value for the
 channel (0 - 255).  
 
 The circuit:
  * All A pins  of AD5206 connected to +5V
  * All B pins of AD5206 connected to ground
  * An LED and a 220-ohm resisor in series connected from each W pin to ground
  * CS - to digital pin 10  (SS pin)
  * SDI - to digital pin 11 (MOSI pin)
  * CLK - to digital pin 13 (SCK pin)
 
 created 10 Aug 2010 
 by Tom Igoe
 
 Thanks to Heather Dewey-Hagborg for the original tutorial, 2005
 
*/


// include the SPI library:
#include <SPI.h>


// set pin 10 as the slave select for the digital pot:
const int port1_selectPin = 62;
const int port1_lowRselectPin = 37;
const int port1_2_12VcommonSelectPin = 72;
const int port1_12VselectPin = 49;
byte wiper;
byte address = 0;
byte TCONaddress = 0x40; // Used to control the internal connections
byte TCONvalue = 0x0F;


boolean port1_lowRstate = false;
boolean port1_2_12VcommonState = 0;

void setup() {
  // set the slaveSelectPin as an output:
  Serial.begin(115200);
  pinMode (port1_selectPin, OUTPUT);
  
  pinMode (port1_lowRselectPin, OUTPUT);
  pinMode (port1_2_12VcommonSelectPin, OUTPUT);
  pinMode (port1_12VselectPin, OUTPUT);
  
  digitalWrite(port1_lowRselectPin, port1_lowRstate);
  digitalWrite(port1_selectPin, HIGH);
  digitalWrite(port1_2_12VcommonSelectPin, port1_2_12VcommonState);
  digitalWrite(port1_12VselectPin, LOW);
  
  // initialize SPI:
  SPI.begin(); 
}

void loop() {
 port1_lowRstate = 0;
 port1_2_12VcommonState = 1;
 TCONvalue = 0b00001111; // All internal Connections made
 printState(); 
 incrementWiper();

 port1_lowRstate = 1;
 port1_2_12VcommonState = 1;
 printState(); 
 incrementWiper();

 port1_lowRstate = 0;
 TCONvalue = 0b00001011; //Disconnects P0A (or 12V)
 printState(); 
 incrementWiper();
 
 TCONvalue = 0b00001101; ////Disconnects P0W (or wiper). 
 printState(); 
 incrementWiper();
 
 TCONvalue = 0b00001110; ////Disconnects POB (or Ground). 
 printState(); 
 incrementWiper();
  

}

void printState(){
  Serial.print("Testing poteniometer with a Chip Select of ");
  Serial.println(port1_selectPin);
  
  Serial.print("Setting Low R Select pin ");
  Serial.print(port1_lowRselectPin);
  Serial.print(" to ");
  Serial.println(port1_lowRstate);
  digitalWrite(port1_lowRselectPin, port1_lowRstate);
  
  Serial.print("Setting port1_2_12VcommonSelectPin ");
  Serial.print(port1_2_12VcommonSelectPin);
  Serial.print(" to ");
  Serial.println(port1_2_12VcommonState);
  digitalWrite(port1_2_12VcommonSelectPin, port1_2_12VcommonState);

  Serial.print("Setting TCON register to ");
  Serial.println(TCONvalue,BIN);
  digitalWrite(port1_2_12VcommonSelectPin, port1_2_12VcommonState);

  digitalPotWrite(port1_selectPin, TCONaddress, TCONvalue);
  
}

void incrementWiper(){
Serial.println("Incrementing wiper position");
  for (int wiper = 0; wiper<256;wiper++){
    Serial.print("+");
    delay(40);
    digitalPotWrite(port1_selectPin, address, wiper);
  }
  Serial.println("\nDone.");
  delay(1000);
}
  
void digitalPotWrite(int slaveSelectPin,byte address, byte value) {
  // gain control of the SPI port
  // and configure settings
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin,HIGH);
  // release control of the SPI port
  SPI.endTransaction();
}
