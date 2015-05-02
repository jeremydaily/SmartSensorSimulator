/*
  SSS.cpp - Library for programming the Smart Sensor Simulator.
*/

#include "Arduino.h"
#include "Wire.h"
#include "mcp_can.h"
#include "SPI.h"
#include "Mcp4261.h"
#include "SSS.h"

MCP_CAN CAN1(6); // Set CS to PH3
MCP_CAN CAN3(7); // Set CS to PH4

SSS::SSS(int potFullScale)
{
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

    //Define slave select pins for SPI
    const int _CSU1Pin = 81;
    const int _CSU2Pin = 82;
    const int _CSU3Pin = 83;
    const int _CSU4Pin = 38;
    const int _CSU5Pin = 40;

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
  

    const float _rAB_ohms = 10000.00; // 10k Ohm
    MCP4261 Mcp4261_U1 = MCP4261( _CSU1Pin, _rAB_ohms );
    MCP4261 Mcp4261_U2 = MCP4261( _CSU2Pin, _rAB_ohms );
    MCP4261 Mcp4261_U3 = MCP4261( _CSU3Pin, _rAB_ohms );
    MCP4261 Mcp4261_U4 = MCP4261( _CSU4Pin, _rAB_ohms );
    MCP4261 Mcp4261_U5 = MCP4261( _CSU5Pin, _rAB_ohms );

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
 Serial.print("CAN1 RX, ");
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
    if (command[3]=='8'){
      Serial.print("Sending CAN Reset command.");
      Wire.beginTransmission(4); //Address the primary processor that is listening for 14 bytes
      for (int i=0; i<14;i++) Wire.write(0x80);
      int flag = Wire.endTransmission(); 
      numCANmsgs=0;
    } 
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
               if (settings[d] > 5000) settings[d]=0;
        } 
        else {
           settings[d]-=10;
           if (settings[d] > 100) settings[d]=0;
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

int SSS::setDAC(int _mVA, int _mVB, int _mVC, int _mVD)
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
  
  Wire.beginTransmission(_DACAddress);
  Wire.write(byte(0x50));             
  Wire.write(highByte(VoutA));             
  Wire.write(lowByte(VoutA));             
  Wire.write(highByte(VoutB));             
  Wire.write(lowByte(VoutB));             
  Wire.write(highByte(VoutC));             
  Wire.write(lowByte(VoutC));             
  Wire.write(highByte(VoutD));             
  Wire.write(lowByte(VoutD));             
  int flag = Wire.endTransmission(); 

  digitalWrite(_LDACPin,HIGH);
  return flag;
}

void SSS::adjustSetting(int i){
  _mVA = settings[20];
  _mVB = settings[21];
  _mVC = settings[22];
  _mVD = settings[23];
  
  Serial.print(i);
  Serial.print(" ");
  Serial.print(decrementCommands[i]);
  Serial.print(" ");
 
  switch (i){
    case 0:
      Mcp4261_U1.wiper0(settings[i]);
      Serial.print("U1-P0 (J16-9): ");
      Serial.println(settings[i]);
      break;
    case 1:
      Mcp4261_U1.wiper1(settings[i]);
      Serial.print("U1-P1 (J16-10): ");
      Serial.println(settings[i]);
      break;
    case 2:
      Mcp4261_U1.wiper2(settings[i]);
      Serial.print("U1-P2 (J16-11): ");
      Serial.println(settings[i]);
      break;
    case 3:
      Mcp4261_U1.wiper3(settings[i]);
      Serial.print("U1-P3 (J16-13)");
      Serial.println(settings[i]);
      break;
    case 4:
      Mcp4261_U2.wiper0(settings[i]);
      Serial.print("U2-P0 (J18-2): ");
      Serial.println(settings[i]);
      break;
    case 5:
      Mcp4261_U2.wiper1(settings[i]);
      Serial.print("U2-P1 (J18-3): ");
      Serial.println(settings[i]);
      break;
    case 6:
      Mcp4261_U2.wiper2(settings[i]);
      Serial.print("U2-P2 (J18-6): ");
      Serial.println(settings[i]);
      break;
    case 7:
      Mcp4261_U2.wiper3(settings[i]);
      Serial.print("U2-P3 (J18-7): ");
      Serial.println(settings[i]);
      break;
    case 8:
      Mcp4261_U3.wiper0(settings[i]);
      Serial.print("U3-P0 (J10-4): ");
      Serial.println(settings[i]);
      break;
    case 9:
      Mcp4261_U3.wiper1(settings[i]);
      Serial.print("U3-P1 (J10-9): ");
      Serial.println(settings[i]);
      break;
    case 10:
      Mcp4261_U3.wiper2(settings[i]);
      Serial.print("U3-P2 (J10-5): ");
      Serial.println(settings[i]);
      break;
    case 11:
      Mcp4261_U3.wiper3(settings[i]);
      Serial.print("U3-P3 (J10-10): ");
      Serial.println(settings[i]);
      break;
    case 12:
      Mcp4261_U4.wiper0(settings[i]);
      Serial.print("U4-P0 (J24-15): ");
      Serial.println(settings[i]);
      break;
    case 13:
      Mcp4261_U4.wiper1(settings[i]);
      Serial.print("U4-P1 (J24-16): ");
      Serial.println(settings[i]);
      break;
    case 14:
      Mcp4261_U4.wiper2(settings[i]);
      Serial.print("U4-P2 (J24-17): ");
      Serial.println(settings[i]);
      break;
    case 15:
      Mcp4261_U4.wiper3(settings[i]);
      Serial.print("U4-P3 (J24-18): ");
      Serial.println(settings[i]);
      break;
    case 16:
      Mcp4261_U5.wiper0(settings[i]);
      Serial.print("U5-P0 (J10-7): ");
      Serial.println(settings[i]);
      break;
    case 17:
      Mcp4261_U5.wiper1(settings[i]);
      Serial.print("U5-P1 (J24-9): ");
      Serial.println(settings[i]);
      break;
    case 18:
      Mcp4261_U5.wiper2(settings[i]);
      Serial.print("U5-P2 (J24-10): ");
      Serial.println(settings[i]);
      break;
    case 19:
      Mcp4261_U5.wiper3(settings[i]);
      Serial.print("U5-P3 (J24-14): ");
      Serial.println(settings[i]);
      break;
    case 20:
      setDAC(_mVA, _mVB, _mVC, _mVD);
      Serial.print("VoutA (J24-19): ");
      Serial.println(settings[i]);
      break;
    case 21:
      setDAC(_mVA, _mVB, _mVC, _mVD);
      Serial.print("VoutB (J24-20): ");
      Serial.println(settings[i]);
      break;
    case 22:
      setDAC(_mVA, _mVB, _mVC, _mVD);
      Serial.print("VoutC (J24-21): ");
      Serial.println(settings[i]);
      break;
    case 23:
      setDAC(_mVA, _mVB, _mVC, _mVD);
      Serial.print("VoutD (J24-22): ");
      Serial.println(settings[i]);
      break;
    case 24:
      _pwm1 = map(settings[i],0,100,0,255);
      analogWrite(_PWMPin1,_pwm1); //192 = 3.70 volts on PWM1
      Serial.print("PWM1 (J24-23): ");
      Serial.println(settings[i]);
      break;
    case 25:
      _pwm2 = map(settings[i],0,100,0,255);
      analogWrite(_PWMPin2,_pwm2); //192 = 3.70 volts on PWM1
      Serial.print("PWM2 (J24-24): ");
      Serial.println(settings[i]);
      break;
    case 26:
      digitalWrite(_J1939Term1Pin,settings[i]);
      Serial.print("J1939 Term 1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 27:
      digitalWrite(_J1939Term2Pin,settings[i]);
      Serial.print("J1939 Term 2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 28:
      digitalWrite(_CAN2Term1Pin,settings[i]);
      Serial.print("CAN2 Term 1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 29:
      Serial.print("CAN2 Term 2: ");
      Serial.print(settings[i]);
      digitalWrite(_CAN2Term2Pin,settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 30:
      digitalWrite(_CAN3Term1Pin,settings[i]);
      Serial.print("CAN3 Term 1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 31:
      digitalWrite(_CAN3Term2Pin,settings[i]);
      Serial.print("CAN3 Term 2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 32:
      digitalWrite(_V2WS1SelectPin,settings[i]);
      Serial.print("V2WS1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Vehicle Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Vehicle Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 33:
      digitalWrite(_V2WS2SelectPin,settings[i]);
      Serial.print("V2WS2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Vehicle Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Vehicle Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 34:
      digitalWrite(_E2WS1SelectPin,settings[i]);
      Serial.print("E2WS1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 35:
      digitalWrite(_E2WS2SelectPin,settings[i]);
      Serial.print("E2WS2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 36:
      digitalWrite(_E2WS3SelectPin,settings[i]);
      Serial.print("E2WS3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 37:
      digitalWrite(_E2WS4SelectPin,settings[i]);
      Serial.print("E2WS4: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = 7 ohms to Engine Sense Return (GND)");
      else if (settings[i] == 1) Serial.println(" = 7 ohms to Engine Supply (+5V)");
      else Serial.println(" Value out of bounds.");
      break;
    case 38:
      digitalWrite(_VccSelectU1_0,settings[i]);
      Serial.print("VccSelectU1_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 39:
      digitalWrite(_VccSelectU1_1,settings[i]);
      Serial.print("VccSelectU1_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 40:
      digitalWrite(_VccSelectU1_2,settings[i]);
      Serial.print("VccSelectU1_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 41:
      digitalWrite(_VccSelectU1_3,settings[i]);
      Serial.print("VccSelectU1_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 42:
      digitalWrite(_VccSelectU2_0,settings[i]);
      Serial.print("VccSelectU2_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 43:
      digitalWrite(_VccSelectU2_1,settings[i]);
      Serial.print("VccSelectU2_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 44:
      digitalWrite(_VccSelectU2_2,settings[i]);
      Serial.print("VccSelectU2_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 45:
      digitalWrite(_VccSelectU2_3,settings[i]);
      Serial.print("VccSelectU2_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 46:
      digitalWrite(_VccSelectU3_0,settings[i]);
      Serial.print("VccSelectU3_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 47:
      digitalWrite(_VccSelectU3_1,settings[i]);
      Serial.print("VccSelectU3_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 48:
      digitalWrite(_VccSelectU3_2,settings[i]);
      Serial.print("VccSelectU3_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 49:
      digitalWrite(_VccSelectU3_3,settings[i]);
      Serial.print("VccSelectU3_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 50:
      digitalWrite(_VccSelectU4_0,settings[i]);
      Serial.print("VccSelectU4_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 51:
      digitalWrite(_VccSelectU4_1,settings[i]);
      Serial.print("VccSelectU4_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 52:
      digitalWrite(_VccSelectU4_2,settings[i]);
      Serial.print("VccSelectU4_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 53:
      digitalWrite(_VccSelectU4_3,settings[i]);
      Serial.print("VccSelectU4_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to Engine Supply (+5V)");
      else if (settings[i] == 1) Serial.println(" = Engine Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 54:
      digitalWrite(_VccSelectU5_0,settings[i]);
      Serial.print("VccSelectU5_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 55:
      digitalWrite(_VccSelectU5_1,settings[i]);
      Serial.print("VccSelectU5_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 56:
      digitalWrite(_VccSelectU5_2,settings[i]);
      Serial.print("VccSelectU5_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 57:
      digitalWrite(_VccSelectU5_3,settings[i]);
      Serial.print("VccSelectU5_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 58:
      digitalWrite(_GroundSelectU1_0,settings[i]);
      Serial.print("GroundSelectU1_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 59:
      digitalWrite(_GroundSelectU1_1,settings[i]);
      Serial.print("GroundSelectU1_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 60:
      digitalWrite(_GroundSelectU1_2,settings[i]);
      Serial.print("GroundSelectU1_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 61:
      digitalWrite(_GroundSelectU1_3,settings[i]);
      Serial.print("GroundSelectU1_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 62:
      digitalWrite(_GroundSelectU2_0,settings[i]);
      Serial.print("GroundSelectU2_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 63:
      digitalWrite(_GroundSelectU2_1,settings[i]);
      Serial.print("GroundSelectU2_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 64:
      digitalWrite(_GroundSelectU2_2,settings[i]);
      Serial.print("GroundSelectU2_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 65:
      digitalWrite(_GroundSelectU2_3,settings[i]);
      Serial.print("GroundSelectU2_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 66:
      digitalWrite(_GroundSelectU3_0,settings[i]);
      Serial.print("GroundSelectU3_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 67:
      digitalWrite(_GroundSelectU3_1,settings[i]);
      Serial.print("GroundSelectU3_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 68:
      digitalWrite(_GroundSelectU3_2,settings[i]);
      Serial.print("GroundSelectU3_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 69:
      digitalWrite(_GroundSelectU3_3,settings[i]);
      Serial.print("GroundSelectU3_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 70:
      digitalWrite(_GroundSelectU4_0,settings[i]);
      Serial.print("GroundSelectU4_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 71:
      digitalWrite(_GroundSelectU4_1,settings[i]);
      Serial.print("GroundSelectU4_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 72:
      digitalWrite(_GroundSelectU4_2,settings[i]);
      Serial.print("GroundSelectU4_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 73:
      digitalWrite(_GroundSelectU4_3,settings[i]);
      Serial.print("GroundSelectU4_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to Engine Sense Return");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 74:
      digitalWrite(_GroundSelectU5_0,settings[i]);
      Serial.print("GroundSelectU5_0: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 75:
      digitalWrite(_GroundSelectU5_1,settings[i]);
      Serial.print("GroundSelectU5_1: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 76:
      digitalWrite(_GroundSelectU5_2,settings[i]);
      Serial.print("GroundSelectU5_2: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 77:
      digitalWrite(_GroundSelectU5_3,settings[i]);
      Serial.print("GroundSelectU5_3: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 78:
      digitalWrite(_CAN2FrontEnablePin,settings[i]);
      Serial.print("CAN2 Panel Enable: ");
      Serial.print(settings[i]);
      if (settings[i] == 0) Serial.println(" = Not Connected");
      else if (settings[i] == 1) Serial.println(" = CAN2 on Front Panel Connected");
      else Serial.println(" Value out of bounds.");
      break;
  }
    
}

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


void SSS::printHelp(){
  Serial.println("This is a helper message:");
}