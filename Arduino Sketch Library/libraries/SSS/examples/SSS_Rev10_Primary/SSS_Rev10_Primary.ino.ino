/*
Copyright 2014
Synercon Technologies, LLC
This code is designed to test the primary processor (U11) of an SSS Rev 9 board.

The bootloader was programmed as the Ardunio pro mini
Last update by J. Daily on 3/11 2015

Features:
This program can broadcast up to 50 different periodic CAN messages that it receives over i2C.
The processor is programmed to listen to the i2C bus on address 2.
It listens for a 14-byte CAN bus message structure with the following format:
The first 4 bytes are the ID that needs to be used for broadcast on J1939
ID0 ID1 ID2 ID3 T0 T1 B0 B1 B3 B3 B4 B5 B6 B7

The ID0 byte is either 0b000XYYYY for J1939 or  0b001XYYYY for the second CAN at 125kbps,
where X is the first bit of the ID and Y is the first nibble.
Example:
  0x38 0xFE 0xF1 0x21 would put a message with an id of 0x18FEF121 on the second CAN channel.
Data follow the motorola format with MSB first.

*/

#include <mcp_can.h>
#include <SPI.h>

//J1939
MCP_CAN CAN0(14); // Set CS to pin A0

//CAN2
MCP_CAN CAN1(15);  // Set CS to pin A1

const int vSensePin   = A6;
const int buttonPin = 7;
const int relayPin = 8;
const int RL2Pin = A2;
const int RL3Pin = A3;
const int greenLEDpin = 5;
const int PWM3Pin = 9;


int vSense = 0;
int safe12mV = 0;

int greenLEDstate = LOW;
int relayState = LOW;

unsigned long buttonDelay = 2000;

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;
unsigned long previousMillis10 = 0;
unsigned long previousMillis100 = 0;
unsigned long previousMillis200 = 0;
unsigned long previousMillis1000 = 0;
unsigned long previousMillis5000 = 0;

int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

unsigned long loopCount = 0;
int i=0;

byte stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //Default CAN message payload
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
//String IDstring = "SYNER*SSS-ECMmake*ABCDEFG    ";
char ID[28];
boolean sendID = false;




void setup()
{ 
  //IDstring.toCharArray(ID,29);
  // initialize the digital pins.
  pinMode(vSensePin,   INPUT);
  pinMode(buttonPin,   INPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(relayPin,    OUTPUT);
  pinMode(RL2Pin,      OUTPUT);
  pinMode(RL3Pin,      OUTPUT);
  pinMode(PWM3Pin,     OUTPUT);
  pinMode(2, INPUT); 
  // Analog write gives outputs with the duty cycle of 50% with a value of 127
  analogWrite(PWM3Pin,127);
  
  digitalWrite(greenLEDpin,greenLEDstate);
  digitalWrite(relayPin, relayState);
  
  Serial.begin(115200);
  Serial.println("Synercon Technologies, LLC");
  Serial.println("Smart Sensor Simulator");
  Serial.println("Programmed to control the SSS board");
  Serial.println("Due to the various programming options of ECMs, this system is not guaranteed to be fault free.");
  
  
  // init can bus
  Serial.println("Setting up CAN0 for J1939..."); //J1939
  if(CAN0.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN0 init ok!!");
  else Serial.print("CAN0 init fail!!\r\n");

//
//  Serial.println("Setting up CAN1..."); //Engine CAN or CAN2 on the SSS
//  if(CAN1.begin(CAN_666KBPS) == CAN_OK) Serial.println("CAN1 init ok!!");
//  else Serial.print("CAN1 init fail!!\r\n");
//  
//  
}

void loop()
{
  currentMillis = millis();
  int reading = digitalRead(buttonPin);
  
  vSense = analogRead(vSensePin);
  safe12mV=map(vSense,0,632,0,12356); 
  
  if (safe12mV < 9000) { //Blink if there is not enough power.
    if ((currentMillis - previousMillis200) > 200) 
      { // Do this every 200 milliseconds
        //Serial.println(safe12mV);
        previousMillis200 = currentMillis;
        greenLEDstate=!greenLEDstate;
      }
  }
  else
  {
     if (reading != lastButtonState) 
      { // reset the debouncing timer
      lastDebounceTime = millis();
      }
   
    if ((millis() - lastDebounceTime) > buttonDelay) 
      {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:
  
      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;
  
        // only toggle the LED if the new button state is HIGH
        if (buttonState == HIGH) {
           relayState= !relayState;
           greenLEDstate = relayState;
           buttonState == LOW;
           reading = LOW;
           buttonDelay = 100; //Enables a short off period before cycling the 
        }
        else
        {
           buttonDelay = 2000;   
        }
      }
    }
  }
      
if (relayState){
    currentMillis = millis();  
    
    if(currentMillis - previousMillis1000 > 1000) { // Do this every second
        previousMillis1000 = currentMillis; // resets the loop timer  
    
        CAN0.sendMsgBuf(0x18FEBD23, 1, 8, stmp);  // Fan Drive 
        loopCount++;
        Serial.println(loopCount);
      } //end 10 ms loop
    if ((currentMillis - previousMillis100) > 100) { // Do this every 100 milliseconds
        previousMillis100 = currentMillis;
/*
Some useful information for J1939:
*----------*----------*--------------------------------*
| SA (Hex) | SA (Dec) | SA Description                 |
*==========*==========*================================*
|  00      |  0       | Engine Controller 1
*----------*----------*--------------------------------*
| 03       | 3        * Transmission Controller
*----------*----------*--------------------------------*
| 0B       | 11       | Electronic Brake Controller 
*----------*----------*--------------------------------*
| 17       | 23       | Instrument Cluster
*----------*----------*--------------------------------*
| 19       | 25       | Passenger Climate Control
*----------*----------*--------------------------------*
| 21       | 33       | Body Controller
*----------*----------*--------------------------------*
| 31       | 49       | Cab Controller
*----------*----------*--------------------------------*

*/

        
        CAN0.sendMsgBuf(0x18F00131, 1, 8, stmp); //Electronic Brake Controller from SA=49
        CAN0.sendMsgBuf(0x18F0010B, 1, 8, stmp); //Electronic Brake Controller from SA=11
        CAN0.sendMsgBuf(0x18FEF117, 1, 8, stmp); //CCVS from SA=23
        CAN0.sendMsgBuf(0x18FEF121, 1, 8, stmp); //CCVS from SA=33
        CAN0.sendMsgBuf(0x18FEF131, 1, 8, stmp); //CCVS from SA=49
        CAN0.sendMsgBuf(0x18E00019, 1, 8, stmp); //Cab Message 1 CM1  from SA=25
        CAN0.sendMsgBuf(0x18E00031, 1, 8, stmp); //Cab Message 1 CM1  from SA=49
        CAN0.sendMsgBuf(0x18F00503, 1, 8, stmp); //Electronic Transmission Controller 2 
        
        
      } //end 100 ms loop
      
    if(currentMillis - previousMillis10 > 10) { // Do this every 10 milliseconds
        previousMillis10 = currentMillis; // resets the loop timer  
    
        CAN0.sendMsgBuf(0x0CF00203, 1, 8, stmp);  // Electronic Transmission Controller 1 (ETC1)
      } //end 10 ms loop

    if(currentMillis - previousMillis5000 > 5000) { // Do this every 5seconds 
        previousMillis5000 = currentMillis; // resets the loop timer  

      } //end 5s loop


  } //fi relayState
 
  digitalWrite(greenLEDpin,relayState); 
  digitalWrite(relayPin, relayState); 
  
  lastButtonState = reading;
} // end loop

