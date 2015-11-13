/*
Title: Smart Sensor Simulator Firmware for the ATMega328p processor on an SSS Rev 10 board.
Provided by Synercon Technologies, LLC as an example

Need 11.5 Amp Power Supply for initial inrush current.
*/

#include <mcp_can.h>
#include <SPI.h>
//#include <Wire.h>

//J1939
MCP_CAN CAN0(14); // Set CS to pin A0

//CAN2
MCP_CAN CAN1(15);  // Set CS to pin A1

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
unsigned long currentMillis = millis();
unsigned long previousMillis = 0;
unsigned long previousMillis20 = 0;
unsigned long previousMillis50 = 0;
unsigned long previousMillis100 = 0;
unsigned long previousMillis200 = 0;
unsigned long previousMillis250 = 0;
unsigned long previousMillis500 = 0;
unsigned long previousMillis1000 = 0;

int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

int i=0;

unsigned char DM1[8] = {0x00,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF};

unsigned char x18FEBF0BMessage[8] = {0x00,0x00,0x7D,0x7D,0x7D,0x7D,0xFF,0xFF};
unsigned char x18F0010BMessage[8] = {0xCF,0xFF,0xF0,0xFF,0xFF,0xDC,0xFF,0x3F};
unsigned char x18F00E51Message[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
unsigned char x18F00F52Message[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x1F,0x1F};
unsigned char x18F0233DMessage[8] = {0x00,0x00,0x08,0x00,0x00,0xFF,0xFF,0xFF};
unsigned char x18F02455Message[8] = {0x00,0x00,0xF0,0xFF,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FD2055Message[8] = {0xC5,0x24,0xDA,0x24,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FD3C3DMessage[8] = {0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FD3E55Message[8] = {0xE4,0x24,0xFF,0xC9,0x24,0xFF,0xFF,0xFF};
unsigned char x18FD4055Message[8] = {0x8F,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FD413DMessage[8] = {0xFF,0xFF,0xFF,0x1F,0x7C,0x7C,0x7C,0x7C};
unsigned char x18FD6E55Message[8] = {0xFF,0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xFF};
unsigned char x18FD8C55Message[8] = {0xFF,0xFF,0x00,0x00,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FD9F55Message[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xCF};
unsigned char x18FDA155Message[8] = {0xA8,0x03,0x00,0x00,0xFF,0xFF,0x3F,0xFF};
unsigned char x18FDB255Message[8] = {0xFF,0xFF,0xFF,0xFF,0x00,0x00,0xFF,0xFF};
unsigned char x18FDB355Message[8] = {0xFF,0xFF,0xD0,0x24,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FDB455Message[8] = {0xC5,0x24,0xDC,0x24,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FE5655Message[8] = {0x49,0x40,0xFF,0xFF,0x1F,0xFF,0x00,0xFF};
unsigned char x18FEDF55Message[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF0};
unsigned char x18FEF755Message[8] = {0xFF,0xFF,0xFF,0xFF,0xC7,0x00,0xFF,0xFF};
unsigned char x18FF4855Message[8] = {0x19,0x00,0x43,0x4D,0x49,0x43,0x45,0x53};
unsigned char x18FF4955Message[8] = {0x1F,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FF4D3DMessage[8] = {0xFF,0xE1,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
unsigned char x18FF4E3DMessage[8] = {0x08,0x00,0x00,0x00,0x00,0x0A,0x1E,0x23};
unsigned char x18FF4F3DMessage[8] = {0x00,0x00,0x00,0xFD,0x00,0x00,0x19,0xFF};
unsigned char x18FF503DMessage[8] = {0x49,0x41,0xFF,0xFF,0xFF,0xFF,0x00,0xFF};
unsigned char x18FFA155Message[8] = {0xFF,0xFE,0xFE,0x00,0xFF,0xFE,0xFF,0xFF};
unsigned char x18FFA355Message[8] = {0x20,0x6D,0xE0,0x34,0xC0,0x80,0xD0,0x82};
unsigned char x18FFA255Message[8] = {0x06,0x02,0xFF,0xFF,0xFF,0x00,0x75,0xAE};
unsigned char x18FFA955Message[8] = {0x76,0x0D,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

unsigned char x0CFD1151Message[8] = {0x0D,0x04,0x00,0x04,0x04,0xCE,0x0F,0xFF};
unsigned char x0CFD0F52Message[8] = {0x1C,0x04,0xDE,0x03,0x03,0xFF,0xFF,0xFF};
unsigned char x1CEBFF3DMessage[3][8]= {{0x01,0xEE,0xFF,0x8F,0x80,0x3B,0xD3,0x6F},
                                       {0x02,0xFB,0xD4,0xFF,0x7F,0x30,0xAF,0xFF},
                                       {0x03,0xFF,0xF7,0xFF,0x7B,0xFF,0xFF,0x00}};

unsigned char x18ECFF55Message[2][8]= {{0x20,0x20,0x00,0x05,0xFF,0x98,0xFD,0x00},
                                       {0x20,0x3F,0x00,0x09,0xFF,0xA4,0xFF,0x00}};

int x1CEBFF3DIndex = 0;
int x18ECFF55Index = 0;



unsigned char stmp[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
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
  
  Serial.begin(57600);
  delay(100);
  Serial.println("Synercon Technologies, LLC");
  Serial.println("Smart Sensor Simulator");
  Serial.println("Programmed to control the Rev 9 and 10 SSS board");
  Serial.println("Due to the various programming options of ECMs, this system is not guaranteed to be fault free.");
  Serial.println("This software is provide AS IS with no warranty.");
  Serial.println(); 

  
  // init can bus
  Serial.println("Setting up CAN0 for J1939..."); //J1939
  if(CAN0.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN0 init ok!!");
  else Serial.print("CAN0 init fail!!\r\n");


  Serial.println("Setting up CAN1..."); //Engine CAN or CAN2 on the SSS
  if(CAN1.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN1 init ok!!");
  else Serial.print("CAN1 init fail!!\r\n");
}

void loop()
{
  currentMillis = millis();
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) lastDebounceTime = currentMillis;
  if ((currentMillis - lastDebounceTime) > buttonDelay) 
  {
    if (reading != buttonState) {
      buttonState = reading;
      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
         relayState = !relayState;
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
//  relayState = HIGH;
  digitalWrite(greenLEDpin,relayState); 
  digitalWrite(relayPin, relayState); 
  lastButtonState = reading; 


      
if (relayState){
    if ((currentMillis - previousMillis100) >= 100) { // Do this every 100 milliseconds
        previousMillis100 = currentMillis;
        
              
        //CAN0.sendMsgBuf(0x18F00131, 1, 8, stmp); //Electronic Brake Controller from SA=49
        CAN0.sendMsgBuf(0x18F0010B, 1, 8, x18F0010BMessage); //Electronic Brake Controller from SA=11
        CAN0.sendMsgBuf(0x18F00131, 1, 8, x18F0010BMessage); //Electronic Brake Controller from SA=49
        CAN0.sendMsgBuf(0x18FEBF0B, 1, 8, x18FEBF0BMessage); //Electronic Brake Controller from SA=11

        CAN0.sendMsgBuf(0x18FEF131, 1, 8, stmp); //CCVS from SA=49
        CAN0.sendMsgBuf(0x18E00031, 1, 8, stmp); //Cab Message 1 CM1  from SA=49
        
        CAN1.sendMsgBuf(0x18FF4F3D, 1, 8, x18FF4F3DMessage); 
        CAN1.sendMsgBuf(0x18FF4E3D, 1, 8, x18FF4E3DMessage); 
        CAN1.sendMsgBuf(0x18FF4955, 1, 8, x18FF4955Message);
        
      }
    if(currentMillis - previousMillis200 >= 200) {
      previousMillis200 = currentMillis; // resets the loop timer  
      CAN1.sendMsgBuf(0x18FFA355, 1, 8, x18FFA355Message);
      CAN1.sendMsgBuf(0x18FFA255, 1, 8, x18FFA255Message);
      CAN1.sendMsgBuf(0x18FD6E55, 1, 8, x18FD6E55Message);

    }  
    
    if(currentMillis - previousMillis250 >= 250) {
      previousMillis250 = currentMillis; // resets the loop timer  
      CAN1.sendMsgBuf(0x18FEDF55, 1, 8, x18FEDF55Message);
    }  
   
    if(currentMillis - previousMillis500 >= 500) {
      previousMillis500 = currentMillis; // resets the loop timer  
      CAN1.sendMsgBuf(0x18FDB455, 1, 8, x18FDB455Message);
      CAN1.sendMsgBuf(0x18FDB355, 1, 8, x18FDB355Message);
      CAN1.sendMsgBuf(0x18FDB255, 1, 8, x18FDB255Message);
      CAN1.sendMsgBuf(0x18FDA155, 1, 8, x18FDA155Message);
      CAN1.sendMsgBuf(0x18FD9F55, 1, 8, x18FD9F55Message);
      CAN1.sendMsgBuf(0x18FD8C55, 1, 8, x18FD8C55Message);
      CAN1.sendMsgBuf(0x18FD413D, 1, 8, x18FD413DMessage);
      CAN1.sendMsgBuf(0x18FD4055, 1, 8, x18FD4055Message);
      CAN1.sendMsgBuf(0x18FD3E55, 1, 8, x18FD3E55Message);
      CAN1.sendMsgBuf(0x18FD2055, 1, 8, x18FD2055Message);
      
      CAN1.sendMsgBuf(0x18ECFF55, 1, 8, x18ECFF55Message[x18ECFF55Index]); 
      x18ECFF55Index +=1;
      if (x18ECFF55Index >= 2 ) x18ECFF55Index=0;
    }  
   
    if(currentMillis - previousMillis50 >= 50) {
        previousMillis50 = currentMillis; // resets the loop timer  
    
        CAN1.sendMsgBuf(0x1CECFF3D, 1, 8, stmp);  
        CAN1.sendMsgBuf(0x18FFA155, 1, 8, x18FFA155Message); 
        CAN1.sendMsgBuf(0x18F02455, 1, 8, x18F02455Message); 
        CAN1.sendMsgBuf(0x18F0233D, 1, 8, x18F0233DMessage); 
        CAN1.sendMsgBuf(0x18F00F52, 1, 8, x18F00F52Message); 
        CAN1.sendMsgBuf(0x18F00E51, 1, 8, x18F00E51Message); 
        
        CAN0.sendMsgBuf(0x0CF00331, 1, 8, stmp); 
        
        
//        CAN1.sendMsgBuf(0x1CEBFF3D, 1, 8, x1CEBFF3DMessage[x1CEBFF3DIndex]); 
//        x1CEBFF3DIndex +=1;
//        if (x1CEBFF3DIndex=>3) x1CEBFF3DIndex=0;
      }
      
      if(currentMillis - previousMillis20 >= 20) {
        previousMillis20 = currentMillis; // resets the loop timer  
    
        CAN0.sendMsgBuf(0x08FE6E0B, 1, 8, stmp);  // High Resolution Wheel Speed  message from Brake Controller
       
      }
      
     if (currentMillis - previousMillis1000 >= 1000) {
       previousMillis1000 = currentMillis;
       CAN1.sendMsgBuf(0x0CFD1151, 1, 8, x0CFD1151Message);
       CAN1.sendMsgBuf(0x0CFD0F52, 1, 8, x0CFD0F52Message);
       CAN1.sendMsgBuf(0x18FFA955, 1, 8, x18FFA955Message);
       CAN1.sendMsgBuf(0x18FF503D, 1, 8, x18FF503DMessage);
       CAN1.sendMsgBuf(0x18FF4D3D, 1, 8, x18FF4D3DMessage); 
       CAN1.sendMsgBuf(0x18FF4855, 1, 8, x18FF4855Message); 
       CAN1.sendMsgBuf(0x18FEF755, 1, 8, x18FEF755Message);  
       CAN1.sendMsgBuf(0x18FE5655, 1, 8, x18FE5655Message); 
       CAN1.sendMsgBuf(0x18FD3C3D, 1, 8, x18FD3C3DMessage); 
       
       CAN0.sendMsgBuf(0x18ECFF0B, 1, 8, stmp); 
       CAN0.sendMsgBuf(0x18FECA0B, 1, 8, DM1); //DM1 from brakes
       CAN0.sendMsgBuf(0x18FECA31, 1, 8, DM1); //DM1 from cab controller
       CAN0.sendMsgBuf(0x18FEA331, 1, 8, stmp); //DM1 from cab controller
      
     }


  } //fi relayState
} //end loop
