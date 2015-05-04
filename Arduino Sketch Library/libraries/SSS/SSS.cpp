/*
  SSS.cpp - Library for programming the Smart Sensor Simulator.
*/
#include "Arduino.h"
#include "mcp_can.h"
#include "SPI.h"
#include "Mcp4261.h"
#include "SSS.h"
#include "I2C.h"


MCP_CAN CAN1(6); // Set CS to PH3
MCP_CAN CAN3(7); // Set CS to PH4

void setPinModes()
{   
    //Initialize the SPI chip pins
    pinMode(CSU1Pin, OUTPUT);
    pinMode(CSU2Pin, OUTPUT);
    pinMode(CSU3Pin, OUTPUT);
    pinMode(CSU4Pin, OUTPUT);
    pinMode(CSU5Pin, OUTPUT);

    digitalWrite(CSU1Pin,HIGH);
    digitalWrite(CSU2Pin,HIGH);
    digitalWrite(CSU3Pin,HIGH);
    digitalWrite(CSU4Pin,HIGH);
    digitalWrite(CSU5Pin,HIGH);

   
    // initialize the digital pins.
    pinMode(PWMPin1, OUTPUT);
    pinMode(PWMPin2, OUTPUT);
    pinMode(PWMPin4, OUTPUT);

    //Resistor Network Modes
    pinMode(E2WS1SelectPin, OUTPUT);
    pinMode(E2WS2SelectPin, OUTPUT);
    pinMode(E2WS3SelectPin, OUTPUT);
    pinMode(E2WS4SelectPin, OUTPUT);
    pinMode(V2WS1SelectPin, OUTPUT);
    pinMode(V2WS2SelectPin, OUTPUT);

    //CAN Termination Resistor Modes
    pinMode(J1939Term1Pin, OUTPUT);
    pinMode(J1939Term2Pin, OUTPUT);
    pinMode(CAN2Term1Pin, OUTPUT);
    pinMode(CAN2Term2Pin, OUTPUT);
    pinMode(CAN3Term1Pin, OUTPUT);
    pinMode(CAN3Term2Pin, OUTPUT);


    pinMode(GroundSelectU1_0,OUTPUT);
    pinMode(GroundSelectU1_1,OUTPUT);
    pinMode(GroundSelectU1_2,OUTPUT);
    pinMode(GroundSelectU1_3,OUTPUT);
    pinMode(GroundSelectU2_0,OUTPUT);
    pinMode(GroundSelectU2_1,OUTPUT);
    pinMode(GroundSelectU2_2,OUTPUT);
    pinMode(GroundSelectU2_3,OUTPUT);
    pinMode(GroundSelectU3_0,OUTPUT);
    pinMode(GroundSelectU3_1,OUTPUT);
    pinMode(GroundSelectU3_2,OUTPUT);
    pinMode(GroundSelectU3_3,OUTPUT);
    pinMode(GroundSelectU4_0,OUTPUT);
    pinMode(GroundSelectU4_1,OUTPUT);
    pinMode(GroundSelectU4_2,OUTPUT);
    pinMode(GroundSelectU4_3,OUTPUT);
    pinMode(GroundSelectU5_0,OUTPUT);
    pinMode(GroundSelectU5_1,OUTPUT);
    pinMode(GroundSelectU5_2,OUTPUT);
    pinMode(GroundSelectU5_3,OUTPUT);

    pinMode(VccSelectU1_0,OUTPUT);
    pinMode(VccSelectU1_1,OUTPUT);
    pinMode(VccSelectU1_2,OUTPUT);
    pinMode(VccSelectU1_3,OUTPUT);
    pinMode(VccSelectU2_0,OUTPUT);
    pinMode(VccSelectU2_1,OUTPUT);
    pinMode(VccSelectU2_2,OUTPUT);
    pinMode(VccSelectU2_3,OUTPUT);
    pinMode(VccSelectU3_0,OUTPUT);
    pinMode(VccSelectU3_1,OUTPUT);
    pinMode(VccSelectU3_2,OUTPUT);
    pinMode(VccSelectU3_3,OUTPUT);
    pinMode(VccSelectU4_0,OUTPUT);
    pinMode(VccSelectU4_1,OUTPUT);
    pinMode(VccSelectU4_2,OUTPUT);
    pinMode(VccSelectU4_3,OUTPUT);
    pinMode(VccSelectU5_0,OUTPUT);
    pinMode(VccSelectU5_1,OUTPUT);
    pinMode(VccSelectU5_2,OUTPUT); 
    pinMode(VccSelectU5_3,OUTPUT);


    // Set up Serial Connections
    Serial2.begin(9600); //J1708 
}




SSS::SSS()
{
    setPinModes();
    displayCAN = false;
    
    boolean CAN1Received = false;
    boolean CAN3Received = false;
    char headings[83][5]= {'U1-P0','U1-P1','U1-P2','U1-P3','U2-P0','U2-P1','U2-P2','U2-P3','U3-P0','U3-P1','U3-P2','U3-P3','U4-P0','U4-P1','U4-P2','U4-P3','U5-P0','U5-P1','U5-P2','U5-P3','VoutA','VoutB','VoutC','VoutD','PWM1','PWM2','1939A','1939B','CAN2A','CAN2B','CAN3A','CAN3B','V2WS1','V2VS2','E2WS1','E2WS2','E2WS3','E2WS4','U1-0V','U1-1V','U1-2V','U1-3V','U2-0V','U2-1V','U2-2V','U2-3V','U3-0V','U3-1V','U3-2V','U3-3V','U4-0V','U4-1V','U4-2V','U4-3V','U5-0V','U5-1V','U5-2V','U5-3V','U1-0G','U1-1G','U1-2G','U1-3G','U2-0G','U2-1G','U2-2G','U2-3G','U3-0G','U3-1G','U3-2G','U3-3G','U4-0G','U4-1G','U4-2G','U4-3G','U5-0G','U5-1G','U5-2G','U5-3G','CAN2P','Coil1','Coil2','Coil3','Coil4'};
    
    //These are the default settings. Change these according to the research for the specific ECM.
    for (int i=1;i<83;i++) settings[i] = defaultSettings[i];

    //CAN Message Structures
    char ID[9] ={0,0,0,0,0,0,0,0,0};
    char period[5] ={0,0,0,0,0};
    char data1[9]={0,0,0,0,0,0,0,0,0};
    char data2[9]={0,0,0,0,0,0,0,0,0};

    unsigned long IDnumber = 0;
    int periodNumber;

    boolean validHex;
    boolean displayCAN = false;
    
    //set Up PWM Pins
    const int _PWMPin1 = 8; //PH5
    const int _PWMPin2 = 9; //PH6
    const int _PWMPin4 = 4; //PG5 in Rev 9b

    //set up PWM values
    const int _pwm1 = 0;
    const int _pwm2 = 0;
    const int _pwm4 = 0;

    I2C I2c = I2C();
    I2c.begin();
    //Set up DAC values for the MCP4728
    const int _LDACPin = 41;
    const int _DACAddress = 0x61;
    const int _daughterDACAddress = 0x62;
    const int _mVA = 1000;
    const int _mVB = 2000;
    const int _mVC = 3000;
    const int _mVD = 4000;
    
    const int _ignitionPin = 77;

    //Set up digital potentiometer pins according to the schematic and pin mapping table
    const int _VccSelectU1_0 = 22;
    const int _VccSelectU1_1 = 23;
    const int _VccSelectU1_2 = 24;
    const int _VccSelectU1_3 = 25;
    const int _GroundSelectU1_0 = 26;
    const int _GroundSelectU1_1 = 27;
    const int _GroundSelectU1_2 = 28;
    const int _GroundSelectU1_3 = 29;

    const int _VccSelectU2_0 = 15;
    const int _VccSelectU2_1 = 14;
    const int _VccSelectU2_2 = 70;
    const int _VccSelectU2_3 = 71;
    const int _GroundSelectU2_0 = 72;
    const int _GroundSelectU2_1 = 73;
    const int _GroundSelectU2_2 = 74;
    const int _GroundSelectU2_3 = 75;

    const int _VccSelectU3_0 = 37;
    const int _VccSelectU3_1 = 36;
    const int _VccSelectU3_2 = 35;
    const int _VccSelectU3_3 = 34;
    const int _GroundSelectU3_0 = 33;
    const int _GroundSelectU3_1 = 32;
    const int _GroundSelectU3_2 = 31;
    const int _GroundSelectU3_3 = 30;

    const int _VccSelectU4_0 = 49;
    const int _VccSelectU4_1 = 48;
    const int _VccSelectU4_2 = 47;
    const int _VccSelectU4_3 = 46;
    const int _GroundSelectU4_0 = 45;
    const int _GroundSelectU4_1 = 44;
    const int _GroundSelectU4_2 = 43;
    const int _GroundSelectU4_3 = 42;

    const int _VccSelectU5_0 = 62;
    const int _VccSelectU5_1 = 63;
    const int _VccSelectU5_2 = 64;
    const int _VccSelectU5_3 = 65;
    const int _GroundSelectU5_0 = 66;
    const int _GroundSelectU5_1 = 67;
    const int _GroundSelectU5_2 = 68;
    const int _GroundSelectU5_3 = 39;

    

    //Resistor Network Selections (pull Up or Pull down)
    const int _E2WS1SelectPin = 10;
    const int _E2WS2SelectPin = 11;
    const int _E2WS3SelectPin = 12;
    const int _E2WS4SelectPin = 13;
    const int _V2WS1SelectPin = 60;
    const int _V2WS2SelectPin = 61;

    //CAN Termination Resistor Selections
    const int _J1939Term1Pin = 54;
    const int _J1939Term2Pin = 55;
    const int _CAN2Term1Pin = 56;
    const int _CAN2Term2Pin = 57;
    const int _CAN3Term1Pin = 58;
    const int _CAN3Term2Pin = 59;
    const int _CAN2FrontEnablePin = 84;
    
    MCP4261 Mcp4261_U1 = MCP4261( CSU1Pin, rAB_ohms );
    MCP4261 Mcp4261_U2 = MCP4261( CSU2Pin, rAB_ohms );
    MCP4261 Mcp4261_U3 = MCP4261( CSU3Pin, rAB_ohms );
    MCP4261 Mcp4261_U4 = MCP4261( CSU4Pin, rAB_ohms );
    MCP4261 Mcp4261_U5 = MCP4261( CSU5Pin, rAB_ohms );

    Mcp4261_U1.scale = potFullScale;
    Mcp4261_U2.scale = potFullScale;
    Mcp4261_U3.scale = potFullScale;
    Mcp4261_U4.scale = potFullScale;
    Mcp4261_U5.scale = potFullScale;

   
    pinMode(CAN1Int,INPUT_PULLUP);
    pinMode(CAN3Int,INPUT_PULLUP);
    
    pinMode(_ignitionPin, INPUT);
   
    //Initialize the SPI chip pins
    pinMode(_CSU1Pin, OUTPUT);
    pinMode(_CSU2Pin, OUTPUT);
    pinMode(_CSU3Pin, OUTPUT);
    pinMode(_CSU4Pin, OUTPUT);
    pinMode(_CSU5Pin, OUTPUT);

    digitalWrite(_CSU1Pin,HIGH);
    digitalWrite(_CSU2Pin,HIGH);
    digitalWrite(_CSU3Pin,HIGH);
    digitalWrite(_CSU4Pin,HIGH);
    digitalWrite(_CSU5Pin,HIGH);

    // initialize the digital pins.
    pinMode(_PWMPin1, OUTPUT);
    pinMode(_PWMPin2, OUTPUT);
    pinMode(_PWMPin4, OUTPUT);
    pinMode(_LDACPin, OUTPUT);
    
    
    digitalWrite(_LDACPin,HIGH);

    //Resistor Network Modes
    pinMode(_E2WS1SelectPin, OUTPUT);
    pinMode(_E2WS2SelectPin, OUTPUT);
    pinMode(_E2WS3SelectPin, OUTPUT);
    pinMode(_E2WS4SelectPin, OUTPUT);
    pinMode(_V2WS1SelectPin, OUTPUT);
    pinMode(_V2WS2SelectPin, OUTPUT);

    //CAN Termination Resistor Modes
    pinMode(_J1939Term1Pin, OUTPUT);
    pinMode(_J1939Term2Pin, OUTPUT);
    pinMode(_CAN2Term1Pin, OUTPUT);
    pinMode(_CAN2Term2Pin, OUTPUT);
    pinMode(_CAN3Term1Pin, OUTPUT);
    pinMode(_CAN3Term2Pin, OUTPUT);

    //Initialize Resistor Network Switches
    //LOW ties all these devices to the Return lines.
    digitalWrite(_E2WS1SelectPin, LOW);
    digitalWrite(_E2WS2SelectPin, LOW);
    digitalWrite(_E2WS3SelectPin, LOW);
    digitalWrite(_E2WS4SelectPin, LOW);
    digitalWrite(_V2WS1SelectPin, LOW);
    digitalWrite(_V2WS2SelectPin, LOW);

    //Initialize CAN Termination Switches
    digitalWrite(_J1939Term1Pin, LOW);
    digitalWrite(_J1939Term2Pin, LOW);
    digitalWrite(_CAN2Term1Pin, LOW);
    digitalWrite(_CAN2Term2Pin, LOW);
    digitalWrite(_CAN3Term1Pin, LOW);
    digitalWrite(_CAN3Term2Pin, LOW);

    pinMode(_GroundSelectU1_0,OUTPUT);
    pinMode(_GroundSelectU1_1,OUTPUT);
    pinMode(_GroundSelectU1_2,OUTPUT);
    pinMode(_GroundSelectU1_3,OUTPUT);
    pinMode(_GroundSelectU2_0,OUTPUT);
    pinMode(_GroundSelectU2_1,OUTPUT);
    pinMode(_GroundSelectU2_2,OUTPUT);
    pinMode(_GroundSelectU2_3,OUTPUT);
    pinMode(_GroundSelectU3_0,OUTPUT);
    pinMode(_GroundSelectU3_1,OUTPUT);
    pinMode(_GroundSelectU3_2,OUTPUT);
    pinMode(_GroundSelectU3_3,OUTPUT);
    pinMode(_GroundSelectU4_0,OUTPUT);
    pinMode(_GroundSelectU4_1,OUTPUT);
    pinMode(_GroundSelectU4_2,OUTPUT);
    pinMode(_GroundSelectU4_3,OUTPUT);
    pinMode(_GroundSelectU5_0,OUTPUT);
    pinMode(_GroundSelectU5_1,OUTPUT);
    pinMode(_GroundSelectU5_2,OUTPUT);
    pinMode(_GroundSelectU5_3,OUTPUT);

    pinMode(_VccSelectU1_0,OUTPUT);
    pinMode(_VccSelectU1_1,OUTPUT);
    pinMode(_VccSelectU1_2,OUTPUT);
    pinMode(_VccSelectU1_3,OUTPUT);
    pinMode(_VccSelectU2_0,OUTPUT);
    pinMode(_VccSelectU2_1,OUTPUT);
    pinMode(_VccSelectU2_2,OUTPUT);
    pinMode(_VccSelectU2_3,OUTPUT);
    pinMode(_VccSelectU3_0,OUTPUT);
    pinMode(_VccSelectU3_1,OUTPUT);
    pinMode(_VccSelectU3_2,OUTPUT);
    pinMode(_VccSelectU3_3,OUTPUT);
    pinMode(_VccSelectU4_0,OUTPUT);
    pinMode(_VccSelectU4_1,OUTPUT);
    pinMode(_VccSelectU4_2,OUTPUT);
    pinMode(_VccSelectU4_3,OUTPUT);
    pinMode(_VccSelectU5_0,OUTPUT);
    pinMode(_VccSelectU5_1,OUTPUT);
    pinMode(_VccSelectU5_2,OUTPUT); 
    pinMode(_VccSelectU5_3,OUTPUT);
    
    
   
} 

boolean SSS::isIgnitionOn()
{
    return digitalRead(_ignitionPin);
    
}

void SSS::sendCANmessages()
{
  if (isIgnitionOn() && numCANmsgs >0 ){
    for (int i = 0; i < numCANmsgs; i++){
      unsigned long currentMillis = millis();
      if ((currentMillis - previousCANmillis[i]) > CANtxPeriod[i]) { // Do this on time.
        previousCANmillis[i] = currentMillis;
        if (CANchannel[i] == 0) CAN1.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]); 
        else if (CANchannel[i] == 32) CAN3.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]); 
      } //end if
    } // end for
  } // end if
}


void SSS::processCAN1message(){
    CAN1.readMsgBuf(&len, _rxBuf);              // Read data: len = data length, buf = data byte(s)
    rxId = CAN1.getCanId();                    // Get message ID
    if ((rxId & 0x00FF0000) == 0x00EA0000){
        //Serial.print("Request ");
        if (_rxBuf[0] == 0xEB && _rxBuf[1] == 0xFE) sendComponentInfo(compID);
    }
}
void SSS::processCAN3message(){
 Serial.print("CAN3 RX, ");
 rxId = CAN1.getCanId();                    // Get message ID
 Serial.print("ID: ");
 Serial.print(rxId, HEX);
 CAN1.readMsgBuf(&len, buf);
 Serial.print(", DLC: ");
 Serial.print(len, HEX);
 Serial.print(", CAN message = ");
 for (int i = 0; i < len; i++)
 {
   Serial.print(buf[i], HEX);
   Serial.print(" ");
 }
 Serial.println();
}

void SSS::MCP2515RX1Int(){
  CAN1Received = true;
}

void SSS::MCP2515RX3Int(){
  CAN3Received = true;
}


void SSS::processCommand(int numDataBytes)
{
  Serial.print("Command: ");
  for (int t=0;t<numDataBytes;t++) Serial.print(command[t]);
  Serial.println();
  
  //displayCAN
  if (command[0] == '#' ) displayCAN = !displayCAN;
  
  else if (command[0] > 13 && command[0] < 39){
    for (int q = 0; q<numCommands; q++) adjustSetting(q);
  }
  
  //help text
  else if (command[0] == 'h' && command[1] == 'e') printHelp();
  
  
  //reset can message count
  else if (command[0] == 'r' && command[1] == 'e' ){
    numCANmsgs=0;
    for (int h = 0; h<numCommands; h++){
      
      adjustSetting(h);
    } 
  }
  
  //Process CAN Instructions
  else if ( (command[0] == 'c' || command[0]=='C') && (command[1] == 'a' || command[1] == 'A') && (command[2] =='N' || command[2] == 'n')){
    Serial.println("CAN Command. Make first byte on ID = 0xF0 to reset CAN messages.");
    // if (command[3]=='8'){
      // Serial.print("Sending CAN Reset command.");
      // I2C.beginTransmission(4); //Address the primary processor that is listening for 14 bytes
      // for (int i=0; i<14;i++) I2C.write(0x80);
      // int flag = I2C.endTransmission(); 
      // numCANmsgs=0;
    // } 
    if (numDataBytes >= 31){
      for (int q=3;q<31;q++){
        validHex=isxdigit(command[q]);
        if (!validHex) break;
      }
      if (validHex){
        
        for (int q= 3;q<11;q++) ID[q-3] = command[q];
        for (int q=11;q<15;q++) period[q-11] = command[q];
        for (int q=15;q<23;q++) data1[q-15] = command[q];
        for (int q=23;q<31;q++) data2[q-23] = command[q];
       
 
        IDnumber = strtoul(ID,0,16);
        Serial.print("ID: ");
        Serial.print(IDnumber,HEX);
        
        CANmessage[0] = (byte)((IDnumber & 0xFF000000) >> 24);
        CANmessage[1] = (byte)((IDnumber & 0xFF0000) >> 16);
        CANmessage[2] = (byte)((IDnumber & 0xFF00) >> 8);
        CANmessage[3] = (byte)((IDnumber & 0xFF));
        
        
        
        periodNumber = (int)(strtoul(period,0,10));
        if (periodNumber < 10) periodNumber = 10; //Don't allow the SSS to flood the bus
        Serial.print(" Period: ");
        Serial.print(periodNumber,DEC);
       

        CANmessage[4]=(byte)((periodNumber & 0x0000FF00) >> 8);
        CANmessage[5]=(byte)((periodNumber & 0x000000FF) >> 0);
        
        unsigned long  firstData = strtoul(data1,0,16);
        unsigned long  secondData = strtoul(data2,0,16);
        Serial.print(" Data: ");
        Serial.print(firstData,HEX);
        Serial.print(secondData,HEX);
        Serial.println();
         
        CANmessage[6]=(byte)((firstData & 0xFF000000) >> 24) ;
        CANmessage[7]=(byte)((firstData & 0x00FF0000) >> 16);
        CANmessage[8]=(byte)((firstData & 0x0000FF00) >> 8);
        CANmessage[9]=(byte)((firstData & 0x000000FF) >> 0);
        CANmessage[10]=(byte)((secondData & 0xFF000000) >> 24);
        CANmessage[11]=(byte)((secondData & 0x00FF0000) >> 16);
        CANmessage[12]=(byte)((secondData & 0x0000FF00) >> 8);
        CANmessage[13]=(byte)((secondData & 0x000000FF) >> 0);
        
        Serial.print("Built CAN message structure: ");
        for (int g=0;g<14;g++) Serial.print((byte)CANmessage[g],HEX);
        Serial.println();
 
        
        buildCANmessage();
        
       }
       else{
         Serial.println("INPUT ERROR: invalid Hex Data.");
         Serial.println(command);
       }
    }
       else {
         Serial.println("INPUT ERROR: Not enough data for creating a CAN message.");
         Serial.println(command);
       }
     
      }
      else if ((command[0] == 's' || command[0]=='S') && (command[1] == 'e' || command[1] == 'E') && (command[2] =='t' || command[2] == 'T')){ 
        int g = lookupIndex(command[3]);
        for (int w=4;w<numDataBytes;w++) value[w-4] = command[w];
        settings[g]=atoi(value);
        adjustSetting(g);
      }

      else if (command[0] =='^'){ 
        int g = lookupIndex(command[1]) + 38;
        settings[g]= !settings[g];
        adjustSetting(g);
      }
      else if (command[0] =='_'){ 
        int g = lookupIndex(command[1]) + 38 + 20;
        settings[g]= !settings[g];
        adjustSetting(g);
      }
      else if (command[0] < 91 && command[0] > 64 ){ //upper case letters
        int b = lookupIndex(command[0]);
        if (b>19 && b<24){
           settings[b]+=100;
           if (settings[b] > 5000) settings[b]=5000;
        } 
        else {
           settings[b]+=10;
           if (settings[b] > 100) settings[b]=100;
        }
        adjustSetting(b);
      }
      else if (command[0] < 123 && command[0] > 96 ){ //Lower case letters
        int d = lookupIndex(command[0]);
        if (d>19 && d<24){
               settings[d]-=100;
               if (settings[d] < 0) settings[d]=0;
        } 
        else {
           settings[d]-=10;
           if (settings[d] < 0 ) settings[d]=0;
        }
        adjustSetting(d);
      }
      else if (lookupIndex(command[0]) > 25){
        Serial.print("Command for lookupIndex = ");
        Serial.println(command[0]);
        int ind = lookupIndex(command[0]);
        Serial.print("Result from lookupIndex = ");
        Serial.println(ind);
        settings[ind] = !settings[ind];  
        adjustSetting(ind);
      }
      
      else 
      {
       Serial.println("INPUT ERROR: Command not recognized.");
      }
      
}
// void SSS::setDAC(){  //Settings are in millivolts.
   
  // digitalWrite(_LDACPin,LOW);
  // for (int j=20; j<24; j++)
  // {
    // if (settings[j]>5000) settings[j]=5000;
    // if (settings[j]<0) settings[j]=0;
  // }
    
  // int VoutA = map(settings[20],0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  // int VoutB = map(settings[21],0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  // int VoutC = map(settings[22],0,3606,0,3000); //0x08BB = 2.682V on VoutC
  // int VoutD = map(settings[23],0,3606,0,3000); //0x0ABB = 3.298V on VoutD

  // VoutA = constrain(VoutA,0,4095);
  // VoutB = constrain(VoutB,0,4095);
  // VoutC = constrain(VoutC,0,4095);
  // VoutD = constrain(VoutD,0,4095);
  
  // byte dataOut[8] = {highByte(VoutA),lowByte(VoutA),highByte(VoutB),lowByte(VoutB),highByte(VoutC),lowByte(VoutC),highByte(VoutD),lowByte(VoutD)};
  
  
  // int flag = I2c.write(_DACAddress,0,dataOut,8);             
  // delay(10);
  // digitalWrite(_LDACPin,HIGH);
  // if (flag != 0) 
  // {
    // Serial.print("Setting DAC over I2C returned error flag of ");
    // Serial.println(flag); 
  // }
  // delay(10);
 
 
// }

/* int SSS::setDAC1(int _mVA, int _mVB, int _mVC, int _mVD)
{  //Settings are in millivolts.
  digitalWrite(_LDACPin,LOW);
  if (_mVA>5000) _mVA=5000;
  if (_mVA<0) _mVA=0;
  if (_mVB>5000) _mVB=5000;
  if (_mVB<0) _mVB=0;
  if (_mVC>5000) _mVC=5000;
  if (_mVC<0) _mVC=0;
  if (_mVD>5000) _mVD=5000;
  if (_mVD<0) _mVD=0;
    
  int VoutA = map(_mVA,0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  int VoutB = map(_mVB,0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  int VoutC = map(_mVC,0,3606,0,3000); //0x08BB = 2.682V on VoutC
  int VoutD = map(_mVD,0,3606,0,3000); //0x0ABB = 3.298V on VoutD

  VoutA = constrain(VoutA,0,4095);
  VoutB = constrain(VoutB,0,4095);
  VoutC = constrain(VoutC,0,4095);
  VoutD = constrain(VoutD,0,4095);
  
  I2C.beginTransmission(_DACAddress);
  I2C.write(byte(0x50));             
  I2C.write(highByte(VoutA));             
  I2C.write(lowByte(VoutA));             
  I2C.write(highByte(VoutB));             
  I2C.write(lowByte(VoutB));             
  I2C.write(highByte(VoutC));             
  I2C.write(lowByte(VoutC));             
  I2C.write(highByte(VoutD));             
  I2C.write(lowByte(VoutD));             
  int flag = I2C.endTransmission(); 

  if (flag != 0) 
  {
    Serial.print("Setting DAC over I2C returned error flag of ");
    Serial.println(flag); 
  }
  delay(10);
  digitalWrite(_LDACPin,HIGH);
  
  return flag;
} */



int SSS::lookupIndex(char c){

    if      (c =='q') return 0;
    else if (c =='w') return 1;
    else if (c =='e') return 2;
    else if (c =='r') return 3;
    else if (c =='t') return 4;
    else if (c =='y') return 5;  
    else if (c =='u') return 6;
    else if (c =='i') return 7;
    else if (c =='a') return 8;
    else if (c =='s') return 9;
    else if (c =='d') return 10;
    else if (c =='f') return 11;
    else if (c =='g') return 12;
    else if (c =='h') return 13;
    else if (c =='j') return 14;
    else if (c =='k') return 15;
    else if (c =='z') return 16;
    else if (c =='x') return 17;
    else if (c =='c') return 18;
    else if (c =='v') return 19;
    else if (c =='b') return 20;
    else if (c =='n') return 21;
    else if (c =='m') return 22;
    else if (c =='o') return 23;
    else if (c =='p') return 24;
    else if (c =='l') return 25;
    else if (c =='Q') return 0;
    else if (c =='W') return 1;
    else if (c =='E') return 2;
    else if (c =='R') return 3;
    else if (c =='T') return 4;
    else if (c =='Y') return 5;  
    else if (c =='U') return 6;
    else if (c =='I') return 7;
    else if (c =='A') return 8;
    else if (c =='S') return 9;
    else if (c =='D') return 10;
    else if (c =='F') return 11;
    else if (c =='G') return 12;
    else if (c =='H') return 13;
    else if (c =='J') return 14;
    else if (c =='K') return 15;
    else if (c =='Z') return 16;
    else if (c =='X') return 17;
    else if (c =='C') return 18;
    else if (c =='V') return 19;
    else if (c =='B') return 20;
    else if (c =='N') return 21;
    else if (c =='M') return 22;
    else if (c =='O') return 23;
    else if (c =='P') return 24;
    else if (c =='L') return 25;
    else if (c == '`') return 26; 
    else if (c == '<') return 27;
    else if (c == '.') return 28;
    else if (c == '>') return 29;
    else if (c == '/') return 30;
    else if (c == '?') return 31;
    else if (c == '[') return 32;
    else if (c == '{') return 33;
    else if (c == ']') return 34;
    else if (c == '}') return 35;
    else if (c == '\\') return 36;
    else if (c == '|') return 37;
    else if (c == '=') return 78;

    else
      return -1;

}

void SSS::buildCANmessage()
{
  
  Serial.print("Building CAN Message number ");
  Serial.print(numCANmsgs);

  CANIDs[numCANmsgs]= IDnumber & 0x1fffffff;
  
  Serial.print(", ID: ");
  Serial.print(CANIDs[numCANmsgs],HEX);
  
  CANtxPeriod[numCANmsgs] = periodNumber;
  Serial.print(", Period: ");
  Serial.print((unsigned int)CANtxPeriod[numCANmsgs]);
  
  CANchannel[numCANmsgs]=CANmessage[0] & 0x20; // either 32 or 0
  
  Serial.print(", Channel: ");
  Serial.print(CANchannel[numCANmsgs]);
  Serial.print(", Data: ");
  for (int j = 0; j<8; j++)
    {
      CANmessages[numCANmsgs][j] =  CANmessage[6+j];
      Serial.print(int(CANmessages[numCANmsgs][j]),HEX);
      Serial.print(' ');
    }
  Serial.println();
  numCANmsgs++;
}

void SSS::sendComponentInfo(char id[29])
{
       Serial.print("Received Request for Component ID. Sending  ");
       for (int i = 0; i<28;i++) Serial.print(id[i]);
       Serial.println();
       byte transport0[8] = {32,28,0,4,0xFF,0xEB,0xFE,0};
       byte transport1[8] = {1,id[0],id[1],id[2],id[3],id[4],id[5],id[6]};
       byte transport2[8] = {2,id[7],id[8],id[9],id[10],id[11],id[12],id[13]};
       byte transport3[8] = {3,id[14],id[15],id[16],id[17],id[18],id[19],id[20]};
       byte transport4[8] = {4,id[21],id[22],id[23],id[24],id[25],id[26],id[27]};
       CAN1.sendMsgBuf(0x1CECFF0B, 1, 8, transport0);
       delay(3);
       CAN1.sendMsgBuf(0x1CEBFF0B, 1, 8, transport1);
       delay(3);
       CAN1.sendMsgBuf(0x1CEBFF0B, 1, 8, transport2);
       delay(2);
       CAN1.sendMsgBuf(0x1CEBFF0B, 1, 8, transport3);
       delay(2);
       CAN1.sendMsgBuf(0x1CEBFF0B, 1, 8, transport4);
       
}
void SSS::printHelp(){
  Serial.println("This is a helper message:");
}

SSS sss = SSS(); // Call the Smart Sensor Simulator library. 

    
void setDAC(){  //Settings are in millivolts.
   
  digitalWrite(LDACPin,LOW);
  for (int j=20; j<24; j++)
  {
    if (sss.settings[j]>5000) sss.settings[j]=5000;
    if (sss.settings[j]<0) sss.settings[j]=0;
  }
    
  int VoutA = map(sss.settings[20],0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  int VoutB = map(sss.settings[21],0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  int VoutC = map(sss.settings[22],0,3606,0,3000); //0x08BB = 2.682V on VoutC
  int VoutD = map(sss.settings[23],0,3606,0,3000); //0x0ABB = 3.298V on VoutD

  VoutA = constrain(VoutA,0,4095);
  VoutB = constrain(VoutB,0,4095);
  VoutC = constrain(VoutC,0,4095);
  VoutD = constrain(VoutD,0,4095);
  
  byte dataOut[9] = {0x50,highByte(VoutA),lowByte(VoutA),highByte(VoutB),lowByte(VoutB),highByte(VoutC),lowByte(VoutC),highByte(VoutD),lowByte(VoutD)};
  
  
  int flag = I2c.write(DACAddress,0,dataOut,9);             
  delay(10);
  digitalWrite(LDACPin,HIGH);
  if (flag != 0) 
  {
    Serial.print("Setting DAC over I2C returned error flag of ");
    Serial.println(flag); 
  }
  delay(10);
 
 
}