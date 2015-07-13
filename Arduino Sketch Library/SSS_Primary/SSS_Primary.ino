/*
Title: Smart Sensor Simulator Firmware for the ATMega328p processor on an SSS Rev 10 board.
Provided by Synercon Technologies, LLC as an example

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

int PWMPin3 = 9;
int PWMPin4 = 10;

int greenLEDpin = 5;
int greenLEDstate = LOW;
int relayState = LOW;

unsigned long buttonDelay = 2000;
unsigned long currentMillis = millis();
unsigned long previousMillis = 0;
unsigned long previousMillis20 = 0;
unsigned long previousMillis100 = 0;
unsigned long previousMillis200 = 0;
unsigned long previousMillis1000 = 0;

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
  if(CAN1.begin(CAN_666KBPS) == CAN_OK) Serial.println("CAN1 init ok!!");
  else Serial.print("CAN1 init fail!!\r\n");
}

void loop()
{
  currentMillis = millis();
  int reading = digitalRead(buttonPin);
  
  vSense = analogRead(vSensePin);
  safe12mV=map(vSense,0,632,0,12356); 
  if ((currentMillis - previousMillis1000) > 1000) {
    Serial.print("MilliVolts = ");
    Serial.println(safe12mV);
    previousMillis1000 = currentMillis;
  }
  
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
      
if (relayState){
     currentMillis = millis();  
    if ((currentMillis - previousMillis100) > 99) { // Do this every 100 milliseconds
        previousMillis100 = currentMillis;
        
        //The following J1939 Sources are found in the Parameters screen of DDDL software
        
        CAN0.sendMsgBuf(0x18F00131, 1, 8, stmp); //Electronic Brake Controller from SA=49
        CAN0.sendMsgBuf(0x18F0010B, 1, 8, stmp); //Electronic Brake Controller from SA=11
        CAN0.sendMsgBuf(0x18FEF117, 1, 8, stmp); //CCVS from SA=23
        CAN0.sendMsgBuf(0x18FEF128, 1, 8, stmp); //CCVS from SA=40
        CAN0.sendMsgBuf(0x18FEF121, 1, 8, stmp); //CCVS from SA=33
        CAN0.sendMsgBuf(0x18FEF131, 1, 8, stmp); //CCVS from SA=49
        CAN0.sendMsgBuf(0x18E00017, 1, 8, stmp); //Cab Message 1 CM1  from SA=23
        CAN0.sendMsgBuf(0x18E00019, 1, 8, stmp); //Cab Message 1 CM1  from SA=25
        CAN0.sendMsgBuf(0x18E00021, 1, 8, stmp); //Cab Message 1 CM1  from SA=33
        CAN0.sendMsgBuf(0x18E00028, 1, 8, stmp); //Cab Message 1 CM1  from SA=40
        CAN0.sendMsgBuf(0x18E00031, 1, 8, stmp); //Cab Message 1 CM1  from SA=49
        
      }
      
    if(currentMillis - previousMillis20 > 19) {
        previousMillis20 = currentMillis; // resets the loop timer  
    
        CAN0.sendMsgBuf(0x0CFE6E0B, 1, 8, stmp);  // High Resolution Wheel Speed  message from Brake Controller
        
      }
  } //fi relayState

  digitalWrite(greenLEDpin,relayState); 
  digitalWrite(relayPin, relayState); 
  
  lastButtonState = reading;
}
