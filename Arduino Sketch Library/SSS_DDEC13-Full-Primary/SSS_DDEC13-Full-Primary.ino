/*
Title: Smart Sensor Simulator Firmware for the ATMega328p processor on an SSS Rev 10 board.
Provided by Synercon Technologies, LLC as an example

*/

#include <mcp_can.h>
#include <SPI.h>
#include <Wire.h>

//J1939
MCP_CAN CAN(14); // Set CS to pin A0

//CAN2
MCP_CAN CAN0(15);  // Set CS to pin A1

const int vSensePin   = A6;
int vSense = 0;
int safe12mV = 0;
int buttonPin = 7;
int relayPin = 8;

int PWMPin3 = 9;
int PWMPin4 = 10;

int greenLEDpin = 5;
int greenLEDstate = LOW;
int relayState = LOW;

unsigned long buttonDelay = 2000;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis10 = 0;  
unsigned long previousMillis20 = 0;  
unsigned long previousMillis50 = 0;  
unsigned long previousMillis100 = 0;  
unsigned long previousMillis1000 = 0; 
unsigned long previousMillis1000delayed = 0;
unsigned long c59timer =0;
unsigned long c5Btimer =0;

byte blankCANdata[8] = {255,255,255,255,255,255,255,255};
byte sessionControlMessage[8] = {0x20,0x0E,0x00,0x01,0xFF,0xCA,0xFE,0x00};
//byte sessionTransportMessage[8] = {0x01,0x00,0xFF,0x00,0x00,0x00,0x00,0xFF};
byte sessionTransportMessage[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // This eliminates an unknown code in DDEC Reports

byte feca03Data[8] ={0x00,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF};
byte ff0903DataList[2][8] ={
{0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x58,0x00,0x00,0x00,0x00}
};
byte ff0903Data[8] ={0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,0xAA};
byte ff0803Data[8] ={0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,0xAA};
byte fef803Data[8] ={0xFF,0xFF,0xFF,0xFF,0x59,0x29,0xFF,0xFF};
byte _8ff0203Data[8] ={0xD2,0x04,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF};
byte _4ff0203Data[8] ={0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0xFF,0xFF};
byte ff0503Data[8] ={0xFF,0x2D,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte ff8003Data[8] ={0xFC,0x03,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF};

byte ff0403Data[8] ={0xFC,0x03,0x64,0xE0,0x00,0xC3,0x00,0x00};
byte ff0403sixthByte[16]={0x0C,0x1C,0x2C,0x3C,0x4C,0x5C,0x6C,0x7C,0x8C,0x9C,0xAC,0xBC,0xCC,0xDC,0xEC,0xFC};
byte ff0403sevenByte[16]={0x25,0x17,0x41,0x73,0xED,0xDF,0x89,0xBB,0x2E,0x1C,0x4A,0x78,0xE6,0xD4,0x82,0xB0};
int index_ff0403 =0;

byte ff0003Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
byte ff0003sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
// This works:
//byte ff0003sevenByte[16]={0x70,0x42,0x14,0x26,0xB8,0x8A,0xDC,0xEE,0x7B,0x49,0x1F,0x2D,0xB3,0x81,0xD7,0xE5};
 byte ff0003sevenByte[16]={0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int index_ff0003 =0;

//This block Works for J1939 Transmission Output shaft speed
//byte ff0103Data[8] ={0xFF,0xFF,0x00,0x00,0x00,0x5C,0x00,0x00};
//byte ff0103sixthByte[16]={0x0C,0x1C,0x2C,0x3C,0x4C,0x5C,0x6C,0x7C,0x8C,0x9C,0xAC,0xBC,0xCC,0xDC,0xEC,0xFC};
//byte ff0103sevenByte[16]={0xD6,0xE4,0xB2,0x80,0x1E,0x2C,0x7A,0x48,0xDD,0xEF,0xB9,0x8B,0x15,0x27,0x94,0x43};

//This block also works for J1939 Transmission Output Shaft Speed
//byte ff0103Data[8] ={0xFF,0xFF,0x00,0x00,0xFF,0xDF,0x00,0x00};
  byte ff0103Data[8] ={0xFF,0xFF,0x00,0x00,0xFF,0xDF,0x00,0x00};
byte ff0103sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
byte ff0103sevenByte[16]={0x33,0x01,0x57,0x65,0xFB,0xC9,0x9F,0xAD,0x38,0x0A,0x5C,0x6E,0xF0,0xC2,0x94,0xA6};
// byte ff0103sevenByte[16]={0x00,0x00,0x00,0x65,0xFB,0xC9,0x9F,0xAD,0x38,0x0A,0x5C,0x6E,0xF0,0xC2,0x94,0xA6};
//byte ff0103sevenByte[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5C,0x00,0x00,0x00,0x00,0x00};
int index_ff0103 =0;

//Changes J1939 FE4A some.
byte ff0203Data[8] ={0xFF,0xFF,0xFF,0xFF,0xD2,0xF1,0x00,0x00};
byte ff0203sixthByte[16]={0x0C,0x1C,0x2C,0x3C,0x4C,0x5C,0x6C,0x7C,0x8C,0x9C,0xAC,0xBC,0xCC,0xDC,0xEC,0xFC};
byte ff0203sevenByte[16]={0x8D,0xBF,0xE9,0xD8,0x45,0x77,0x21,0x13,0x86,0x84,0xE2,0xD0,0x4E,0x7C,0x2A,0x18};
int index_ff0203 =0;

byte ff0603Data[8] ={0xFF,0xFF,0xFF,0xFF,0xF3,0x7F,0x00,0x00};
byte ff0603sixthByte[16]={0x0C,0x1C,0x2C,0x3C,0x4C,0x5C,0x6C,0x7C,0x8C,0x9C,0xAC,0xBC,0xCC,0xDC,0xEC,0xFC};
byte ff0603sevenByte[16]={0xD7,0xE5,0xB3,0x81,0x1F,0x2D,0x7B,0x49,0xDC,0xEE,0xB8,0x8A,0x14,0x26,0x70,0x42};
int index_ff0603 =0;

byte ff0703Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
byte ff0703sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
//byte ff0703sevenByte[16]={0x70,0x42,0x14,0x26,0xB8,0x8A,0xDC,0xEE,0x7B,0x49,0x1F,0x2D,0xB3,0x81,0xD7,0xE5};
byte ff0703sevenByte[16]={0x02,0x30,0x66,0x54,0xCA,0xF8,0xAE,0x9C,0x09,0x3B,0x6D,0x5F,0xC1,0xF3,0xA5,0x97};
int index_ff0703 =0;

byte ff0903sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
byte ff0903sevenByte[16]={0x02,0x30,0x66,0x54,0xCA,0xF8,0xAE,0x9C,0x09,0x3B,0x6D,0x5F,0xC1,0xF3,0xA5,0x97};
int index_ff0903 =0;


byte Cff0703Data[8] ={0xD0,0x20,0xC3,0x00,0xFF,0x5A,0x00,0x00};
//byte Cff0703fifthByte[16]={0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82};
byte Cff0703sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
// This works: 
//byte Cff0703sevenByte[16]={0x02,0x30,0x66,0x54,0xCA,0xF8,0xAE,0x9C,0x09,0x3B,0x6D,0x5F,0xC1,0xF3,0xA5,0x97};
  byte Cff0703sevenByte[16]={0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int index_Cff0703 =0;

//byte ff0303Data[8] ={0x7D,0x8C,0xFF,0xFF,0xA8,0x61,0xA8,0x61};
byte ff0303Data[8] ={0xFF,0x8C,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

byte Cff0203Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte Cff0303Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
//byte Cff0303Data[8] ={0xE2,0xE2,0xE2,0xE2,0xE2,0xE2,0xE2,0xE2};
byte Cff0403Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

int i=0;

unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
long count = 0;


void setup()
{ 
  
  // initialize the digital pins.
  pinMode(vSensePin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(PWMPin3, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  
  // Analog write gives outputs with the duty cycle of 50% with a value of 127
  analogWrite(PWMPin3,127);
 
  digitalWrite(greenLEDpin,greenLEDstate);
  digitalWrite(relayPin, relayState);
  
  Serial.begin(115200);
  delay(100);
  Serial.println("Synercon Technologies, LLC");
  Serial.println("Smart Sensor Simulator");
  Serial.println("Programmed to control the Rev 9 and 10 SSS board");
  Serial.println("Due to the various programming options of ECMs, this system is not guaranteed to be fault free.");
  Serial.println("This software is provide AS IS with no warranty.");
  Serial.println(); 

  
  // init can bus
  Serial.println("Setting up CAN0 for J1939..."); //J1939
  if(CAN.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN0 init ok!!");
  else Serial.print("CAN0 init fail!!\r\n");


  
  Serial.println("Setting up CAN0 for 666k for DDEC 13..."); 
  if(CAN0.begin(CAN_666KBPS) == CAN_OK) Serial.println("CAN0 init ok!!");
  else Serial.print("CAN0 init fail!!\r\n");
  Serial.println("Done Starting Up");
}

void loop()
{
  currentMillis = millis();
  int reading = digitalRead(buttonPin);
  
  vSense = analogRead(vSensePin);
  safe12mV=map(vSense,0,632,0,12356); 
  //if ((currentMillis - previousMillis1000) > 1000) {
    //Serial.print("MilliVolts = ");
    //Serial.println(safe12mV);
   // previousMillis1000 = currentMillis;
  //}
  
  if (safe12mV > 9000){
    
    
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
      
//if (relayState){
     currentMillis = millis();
  
  if(currentMillis - previousMillis1000 >= 1000) {
    previousMillis1000 = currentMillis; // resets the loop timer  
    previousMillis1000delayed = currentMillis+5; // resets the loop timer  
  //  CAN0.sendMsgBuf(0x10ECFF01, 1, 8, sessionControlMessage); //MCM DM1
    
    //CAN0.sendMsgBuf(0x10FECA03, 1, 8, feca03Data); //TCM DM1
    CAN0.sendMsgBuf(0x18FEF803, 1, 8, fef803Data); // Transmission Oil Temperature
    //CAN0.sendMsgBuf(0x18FF8003, 1, 8, ff8003Data);
    //CAN0.sendMsgBuf(0x18FF0503, 1, 8, ff0503Data);
    
  //  CAN0.sendMsgBuf(0x10ECFF3D, 1, 8, sessionControlMessage); //ACM DM1
    
  }

//  if(currentMillis - previousMillis1000delayed >= 1000) {
//   CAN0.sendMsgBuf(0x10EBFF01, 1, 8, sessionTransportMessage); //MCM DM1
//   CAN0.sendMsgBuf(0x10EBFF3D, 1, 8, sessionTransportMessage); //ACM DM1
//  } 
    
  if(currentMillis - previousMillis100 >= 100) {
    previousMillis100 = currentMillis;     
    
    ff0403Data[6]=ff0403sixthByte[index_ff0403];
    ff0403Data[7]=ff0403sevenByte[index_ff0403];
    //CAN0.sendMsgBuf(0x14FF0403, 1, 8, ff0403Data);
    index_ff0403 += 1;
    if (index_ff0403 >= 16) index_ff0403=0;
    
    //CAN0.sendMsgBuf(0x14FF0203, 1, 8, _4ff0203Data);
    //CAN0.sendMsgBuf(0x14FF0803, 1, 8, ff0803Data);
    
  }
  
  if(currentMillis - previousMillis50 >= 50) {
    previousMillis50 = currentMillis;     
//    ff0903Data[3]=byte(0x5F); 
    //ff0903Data[3]=byte(random(0,255)>>4);
    //ff0903Data[3]=byte(0x50 + byte(random(0,255) & 0xF0));
    ff0903Data[0]=byte(random(0,256));
    ff0903Data[1]=byte(random(0,256));
    ff0903Data[2]=byte(random(0,256));
    ff0903Data[3]=byte(random(0,256));
    ff0903Data[4]=byte(random(0,256));
    ff0903Data[5]=byte(random(0,256));
    ff0903Data[6]=ff0903sixthByte[index_ff0903];
    ff0903Data[7]=ff0903sevenByte[index_ff0903];
    CAN0.sendMsgBuf(0x10FF0903, 1, 8, ff0903Data);
    index_ff0903 += 1;
    if (index_ff0903 >= 2) index_ff0903=0;
    
  }
  
  if(currentMillis - previousMillis20 >= 20) {
    previousMillis20 = currentMillis;     
    
    if (currentMillis - c59timer > 2000) {
      c59timer = currentMillis;
      Cff0703Data[5] = 0x5A;
      //ff0003sevenByte[0]+=1; // look for valid ids in TCM System IDs
    }
    else
      Cff0703Data[5] = 0x59;
    
    if (currentMillis - c5Btimer > 3000) {
      c5Btimer = currentMillis;
      Cff0703Data[5] = 0x5B;
    }
    else
      Cff0703Data[5] = 0x59;
    
    //TCM transport layer Message with TCM System ID
    Cff0703Data[6]=Cff0703sixthByte[index_Cff0703];
    Cff0703Data[7]=Cff0703sevenByte[index_Cff0703];
    CAN0.sendMsgBuf(0xCFF0703, 1, 8, Cff0703Data);
    index_Cff0703 += 1;
    if (index_Cff0703 >= 16) index_Cff0703=0;
    
    //No Changing data for TCM Message
    //CAN0.sendMsgBuf(0x18FF0203, 1, 8, _8ff0203Data);
    //CAN0.sendMsgBuf(0xCFF0203, 1, 8, Cff0203Data);
    //CAN0.sendMsgBuf(0xCFF0303, 1, 8, Cff0303Data);
    //CAN0.sendMsgBuf(0xCFF0403, 1, 8, Cff0403Data);
  }
    
  if(currentMillis - previousMillis10 >= 10) {
    previousMillis10 = currentMillis;     
 //   CAN0.sendMsgBuf(0x08FF0001, 1, 8, blankCANdata); //MCM ID
    
    //TCM System ID Message ID 
    ff0003Data[6]=ff0003sixthByte[index_ff0003];
    ff0003Data[7]=ff0003sevenByte[index_ff0003];
    CAN0.sendMsgBuf(0x8FF0003, 1, 8, ff0003Data);
    index_ff0003 += 1;
    if (index_ff0003 >= 16) index_ff0003=0;
    
    
    //J1939 Transmission Output Shaft Speed Signal which may corrospond to PGN 0x00F002
    ff0103Data[6]=ff0103sixthByte[index_ff0103];
    ff0103Data[7]=ff0103sevenByte[index_ff0103];
    CAN0.sendMsgBuf(0x8FF0103, 1, 8, ff0103Data);
    index_ff0103 += 1;
    if (index_ff0103 >= 16) index_ff0103=0;
    
    ff0203Data[6]=ff0203sixthByte[index_ff0203];
    ff0203Data[7]=ff0203sevenByte[index_ff0203];
    //CAN0.sendMsgBuf(0x8FF0203, 1, 8, ff0203Data);
    index_ff0203 += 1;
    if (index_ff0203 >= 16) index_ff0203=0;
    
    //No Changing data for TCM message
    CAN0.sendMsgBuf(0x8FF0303, 1, 8, ff0303Data);
   
    //TCM transport layer Message
    ff0603Data[6]=ff0603sixthByte[index_ff0603];
    ff0603Data[7]=ff0603sevenByte[index_ff0603];
  //  CAN0.sendMsgBuf(0x8FF0603, 1, 8, ff0603Data);
    index_ff0603 += 1;
    if (index_ff0603 >= 16) index_ff0603=0;
   
    //TCM transport layer Message
    ff0703Data[6]=ff0703sixthByte[index_ff0703];
    ff0703Data[7]=ff0703sevenByte[index_ff0703];
   // CAN0.sendMsgBuf(0x8FF0703, 1, 8, ff0703Data);
    index_ff0703 += 1;
    if (index_ff0703 >= 16) index_ff0703=0;
  }
  
//  } //fi relayState

  digitalWrite(greenLEDpin,relayState); 
  digitalWrite(relayPin, relayState); 
  
  lastButtonState = reading;
}
