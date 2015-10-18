/*
Copyright 2014
Synercon Technologies, LLC
This code is designed to test the primary processor (U11) of an SSS Rev 10 board.

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
#include <Wire.h>

//J1939
MCP_CAN CAN0(14); // Set CS to pin A0

//CAN2
MCP_CAN CAN1(15);  // Set CS to pin A1

const int vSensePin   = A6;
int vSense = 0;
int safe12mV = 0;
int buttonPin = 7;
int relayPin = 8;
int relay1 = A2;
int relay2 = A3;

int PWMPin3 = 9;
int PWMPin4 = 10;

int greenLEDpin = 5;
int greenLEDstate = LOW;
int relayState = LOW;

//initialize storage for 50 CAN messages to be transmitted.
int numCANmsgs = 0;
byte CANchannel[50];
unsigned long CANIDs[50];
int CANtxPeriod[50];
byte CANmessages[50][8];
unsigned long previousCANmillis[50];

unsigned long buttonDelay = 2000;
unsigned long currentMillis = millis();
unsigned long previousMillis = 0;
unsigned long previousMillis10 = 0;
unsigned long previousMillis50 = 0;
unsigned long previousMillis100 = 0;
unsigned long previousMillis200 = 0;
unsigned long previousMillis1000 = 0;

int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

//boolean newI2C = false;
boolean newI2C = true;

int i=0;

byte buffer[14] = {0x00,0xFF,0xFF,0xFF,0x00,0x2F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
long count = 0;


void setup()
{ 
  pinMode(relayPin,INPUT_PULLUP);
 
 
//  Wire.begin(4); //Join the i2C Bus with address 4
//  Wire.onReceive(buildCANMessage); //see request Event at the bottom of the code
  
  
  
  // initialize the digital pins.
  pinMode(vSensePin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(PWMPin3, OUTPUT);
  //pinMode(PWMPin4, OUTPUT); //This pin may be overpowered by SPI.
  pinMode(greenLEDpin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  
  // Analog write gives outputs with the duty cycle of 50% with a value of 127
  analogWrite(PWMPin3,127);
  //analogWrite(PWMPin4,0);
  
  digitalWrite(greenLEDpin,greenLEDstate);
  digitalWrite(relayPin, LOW);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  
  Serial.begin(115200);
  delay(100);
  Serial.println("Synercon Technologies, LLC");
  Serial.println("Smart Sensor Simulator");
  Serial.println("Programmed to control the Rev 10 SSS board");
  Serial.println("Due to the various programming options of ECMs, this system is not guaranteed to be fault free.");
  Serial.println("Software revision date: 9 March 2015");
  Serial.println("Copyright 2014, all rights reserved.");
  Serial.println(); 

  
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
////Build a blank message as a holder
  numCANmsgs=0;
  CANIDs[numCANmsgs] = 0x18F0010B;
  CANtxPeriod[numCANmsgs] = 100;
  CANchannel[numCANmsgs]=0x00;
  for (int j = 0; j<8; j++)
    {
      CANmessages[numCANmsgs][j] =  0x00;
    }
//  
}



void loop()
{
  currentMillis = millis();
  int reading = digitalRead(buttonPin);
  
  vSense = analogRead(vSensePin);
  safe12mV=map(vSense,0,632,0,12356); 
  
  if (safe12mV < 9000)
    { //Blink if there is not enough power.
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
    if ((currentMillis - previousMillis100) >= 100) { // Do this every 100 milliseconds
        previousMillis100 = currentMillis;
        
        //The following J1939 Sources are found in the Parameters screen of DDDL software
        
        CAN0.sendMsgBuf(0x18F00131, 1, 8, stmp); //Electronic Brake Controller from SA=49
        CAN0.sendMsgBuf(0x18F0010B, 1, 8, stmp); //Electronic Brake Controller from SA=11
        CAN0.sendMsgBuf(0x18FEF111, 1, 8, stmp); //CCVS from SA=17
        CAN0.sendMsgBuf(0x18FEF121, 1, 8, stmp); //CCVS from SA=33
        CAN0.sendMsgBuf(0x18FEF131, 1, 8, stmp); //CCVS from SA=49
        CAN0.sendMsgBuf(0x18E00011, 1, 8, stmp); //Cab Message 1 CM1  from SA=17
        CAN0.sendMsgBuf(0x18E00031, 1, 8, stmp); //Cab Message 1 CM1  from SA=49
        
      }
      
    if(currentMillis - previousMillis10 >= 10) {
        previousMillis10 = currentMillis; // resets the loop timer  
    
        CAN0.sendMsgBuf(0x0CF00203, 1, 8, stmp);  // Electronic Transmission Controller 1 (ETC1)
      }
      if(currentMillis - previousMillis50 >= 50) {
        previousMillis50 = currentMillis; // resets the loop timer  
    
        CAN0.sendMsgBuf(0x0CF00311, 1, 8, stmp);  // EEC2 Electronic controller 2
      }
      if(currentMillis - previousMillis1000 >= 1000) {
        previousMillis1000 = currentMillis; // resets the loop timer  
    
        CAN0.sendMsgBuf(0x18FEE411, 1, 8, stmp);  // Shutdown PGN 65252
        
      }
  } //fi relayState

//  if (relayState){
//   for (int i = 0; i < numCANmsgs; i++){
//      currentMillis = millis();
//      unsigned long spacing = (unsigned long) CANtxPeriod[i];
//      if ((currentMillis - previousCANmillis[i]) > CANtxPeriod[i]) { // Do this on time.
//        previousCANmillis[i] = currentMillis;
//        if (CANchannel[i] == 0) CAN0.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]); 
//        else if (CANchannel[i] == 0x20) CAN1.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]); 
//      }
//     }
//  } //fi relayState
//  
  
  digitalWrite(greenLEDpin,relayState); 
  digitalWrite(relayPin, relayState); 
  
  lastButtonState = reading;
  }



//// function that executes whenever data is requested by master
//// this function is registered as an event, see setup()
//// These may need to be arguments: int numCANmsgs, int CANchannel[50],long CANIDs[50],int CANtxPeriod[50],byte CANmessages[50][8]
void buildCANMessage(int numBytes){
  //Serial.write("Receved i2c Message.");
  
  
  i=0;
  while ( Wire.available() )
  {
    buffer[i] = Wire.read();
    i++;
   // delay(1);
  }
  if ((buffer[0] & 0x80) == 0x80) numCANmsgs=0;
  else {
    byte ID0 = buffer[0] & 0x1F;
    byte ID1 = buffer[1];
    byte ID2 = buffer[2];
    byte ID3 = buffer[3];
    
    if (numCANmsgs ==50) numCANmsgs = 0;
    
    
    CANchannel[numCANmsgs] = buffer[0] & 0x20;
    
    CANIDs[numCANmsgs] = (unsigned long)ID0 << 24 | (unsigned long)ID1 <<16 | (unsigned long)ID2 << 8 | (unsigned long)ID3;
    
    CANtxPeriod[numCANmsgs] = (int)buffer[4] <<8 | (int)buffer[5];
    
    for (int j = 6; j<14; j++){
      CANmessages[numCANmsgs][j-6] =  buffer[j];
    }
    numCANmsgs++;
  }    
  
  
}
