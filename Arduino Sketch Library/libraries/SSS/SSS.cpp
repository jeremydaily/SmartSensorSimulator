/*
  SSS.cpp - Library for programming the Smart Sensor Simulator.
*/
#include "Arduino.h"
#include "SPI.h"
#include "mcp_can.h"
#include "Mcp4261.h"
#include "SSS.h"
#include "Wire.h"

MCP4261 Mcp4261_U1 = MCP4261(CSU1Pin, rAB_ohms );
MCP4261 Mcp4261_U2 = MCP4261(CSU2Pin, rAB_ohms );
MCP4261 Mcp4261_U3 = MCP4261(CSU3Pin, rAB_ohms );
MCP4261 Mcp4261_U4 = MCP4261(CSU4Pin, rAB_ohms );
MCP4261 Mcp4261_U5 = MCP4261(CSU5Pin, rAB_ohms );

MCP_CAN CAN1 = MCP_CAN(6); // Set CS to PH3
MCP_CAN CAN3 = MCP_CAN(7); // Set CS to PH4


int pwm_1;
int pwm_2; 

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

    pinMode(ADC15,INPUT);
   
    // initialize the digital pins.
    pinMode(PWMPin1, OUTPUT);
    pinMode(LINwake, OUTPUT);
    pinMode(LINcs, OUTPUT);
    pinMode(LDACPin, OUTPUT);
    
    digitalWrite(PWMPin1, LOW);
    digitalWrite(LINwake, LOW);
    digitalWrite(LINcs, LOW);
    digitalWrite(LDACPin, HIGH);
    
    //Resistor Network Modes
    pinMode(E2WS1SelectPin, OUTPUT);
    pinMode(E2WS2SelectPin, OUTPUT);
    pinMode(E2WS3SelectPin, OUTPUT);
    pinMode(E2WS4SelectPin, OUTPUT);
    pinMode(V2WS1SelectPin, OUTPUT);
    pinMode(V2WS2SelectPin, OUTPUT);
    pinMode(Coil1Control, OUTPUT);
    pinMode(Coil2Control, OUTPUT);
    pinMode(Coil3Control, OUTPUT);
    pinMode(Coil4Control, OUTPUT);

    digitalWrite(E2WS1SelectPin, LOW);
    digitalWrite(E2WS2SelectPin, LOW);
    digitalWrite(E2WS3SelectPin, LOW);
    digitalWrite(E2WS4SelectPin, LOW);
    digitalWrite(V2WS1SelectPin, LOW);
    digitalWrite(V2WS2SelectPin, LOW);
    digitalWrite(Coil1Control, LOW);
    digitalWrite(Coil2Control, LOW);
    digitalWrite(Coil3Control, LOW);
    digitalWrite(Coil4Control, LOW);
    
    //CAN Termination Resistor Modes
    pinMode(J1939Term1Pin, OUTPUT);
    pinMode(J1939Term2Pin, OUTPUT);
    pinMode(CAN2Term1Pin, OUTPUT);
    pinMode(CAN2Term2Pin, OUTPUT);
    pinMode(CAN3Term1Pin, OUTPUT);
    pinMode(CAN3Term2Pin, OUTPUT);
    pinMode(CAN2FrontEnablePin,OUTPUT);
    
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
    
    

    Mcp4261_U1.scale = potFullScale;
    Mcp4261_U2.scale = potFullScale;
    Mcp4261_U3.scale = potFullScale;
    Mcp4261_U4.scale = potFullScale;
    Mcp4261_U5.scale = potFullScale;
 
}

void setupSerial(){
    // Set up Serial Connections
    Serial.begin(115200); // Serial to the USB to UART bridge
    Serial.println("Starting Up...");
    Serial.println("Synercon Technologies Smart Sensor Simulator");
    Serial.println("Program running for the ATmega2560 Processor.");

    

    Serial.println("Setting up CAN1..."); //J1939
    if(CAN1.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN1 init ok!!");
    else Serial.println("CAN0 init fail!!");
    Serial.println("Setting up CAN3..."); //Used CAN3 in reference to the circuit in the schematics
    if(CAN3.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN3 init ok!!");
    else Serial.println("CAN3 init fail!!");

    Serial2.begin(9600); //J1708 
}
  

SSS::SSS(){
    //Constructor
}

void SSS::begin(){
    Wire.begin();
    setupSerial();
    setPinModes();
    displayCAN = false;
 
    separatorChar = ',';
    

    numCommands = 83;
    numCANmsgs = 0;
    displayCAN = false;
    len = 0;
    _ignitionPin = 77;
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
        for (int f=0;f<6;f++) value[f] = 0x00;
        for (int w=4;w<numDataBytes;w++) value[w-4] = command[w];
        settings[g]=atoi(value);
        
        if (g>19 && g<24){
          if (settings[g] > 5000) settings[g]=5000;
          if (settings[g] < 0) settings[g]=0;
        } 
        else {
            if (settings[g] > 100) settings[g]=100;
            if (settings[g] < 0 ) settings[g]=0;
        }
        
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
    else if (c == '1') return 79;
    else if (c == '2') return 80;
    else if (c == '3') return 81;
    else if (c == '4') return 82;

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
       //Serial.print("Received Request for Component ID. Sending  ");
       //for (int i = 0; i<28;i++) Serial.print(id[i]);
       //Serial.println();
       byte transport0[8] = {32,28,0,4,0xFF,0xEB,0xFE,0};
       byte transport1[8] = {1,id[0],id[1],id[2],id[3],id[4],id[5],id[6]};
       byte transport2[8] = {2,id[7],id[8],id[9],id[10],id[11],id[12],id[13]};
       byte transport3[8] = {3,id[14],id[15],id[16],id[17],id[18],id[19],id[20]};
       byte transport4[8] = {4,id[21],id[22],id[23],id[24],id[25],id[26],id[27]};
       CAN1.sendMsgBuf(0x1CECFFFE, 1, 8, transport0);
       delay(3);
       CAN1.sendMsgBuf(0x1CEBFFFE, 1, 8, transport1);
       delay(3);
       CAN1.sendMsgBuf(0x1CEBFFFE, 1, 8, transport2);
       delay(2);
       CAN1.sendMsgBuf(0x1CEBFFFE, 1, 8, transport3);
       delay(2);
       CAN1.sendMsgBuf(0x1CEBFFFE, 1, 8, transport4);
       
}
void SSS::printHelp(){
  Serial.println("This is a helper message:");
}

SSS sss = SSS(); // Call the Smart Sensor Simulator library. 

    



void adjustSetting(int i)
{
  const String decrementCommands[83] = {"q","w","e","r","t","y","u","i","a","s","d","f","g","h","j","k","z","x","c","v","b","n","m","o","p","l","`","<",".",">","/","?","[","{","]","}","\\","|","^q","^w","^e","^r","^t","^y","^u","^i","^a","^s","^d","^f","^g","^h","^j","^k","^z","^x","^c","^v","_q","_w","_e","_r","_t","_y","_u","_i","_a","_s","_d","_f","_g","_h","_j","_k","_z","_x","_c","_v","=","1","2","3","4"};
  
  Serial.print(i);
  Serial.print(" ");
  Serial.print(decrementCommands[i]);
  Serial.print(" ");
 
  switch (i){
    case 0:
      Mcp4261_U1.wiper0(sss.settings[i]);
      Serial.print("U1-P0 (J16-9): ");
      Serial.println(sss.settings[i]);
      break;
    case 1:
      Mcp4261_U1.wiper1(sss.settings[i]);
      Serial.print("U1-P1 (J16-10): ");
      Serial.println(sss.settings[i]);
      break;
    case 2:
      Mcp4261_U1.wiper2(sss.settings[i]);
      Serial.print("U1-P2 (J16-11): ");
      Serial.println(sss.settings[i]);
      break;
    case 3:
      Mcp4261_U1.wiper3(sss.settings[i]);
      Serial.print("U1-P3 (J16-13): ");
      Serial.println(sss.settings[i]);
      break;
    case 4:
      Mcp4261_U2.wiper0(sss.settings[i]);
      Serial.print("U2-P0 (J18-2): ");
      Serial.println(sss.settings[i]);
      break;
    case 5:
      Mcp4261_U2.wiper1(sss.settings[i]);
      Serial.print("U2-P1 (J18-3): ");
      Serial.println(sss.settings[i]);
      break;
    case 6:
      Mcp4261_U2.wiper2(sss.settings[i]);
      Serial.print("U2-P2 (J18-6): ");
      Serial.println(sss.settings[i]);
      break;
    case 7:
      Mcp4261_U2.wiper3(sss.settings[i]);
      Serial.print("U2-P3 (J18-7): ");
      Serial.println(sss.settings[i]);
      break;
    case 8:
      Mcp4261_U3.wiper0(sss.settings[i]);
      Serial.print("U3-P0 (J10-4): ");
      Serial.println(sss.settings[i]);
      break;
    case 9:
      Mcp4261_U3.wiper1(sss.settings[i]);
      Serial.print("U3-P1 (J10-9): ");
      Serial.println(sss.settings[i]);
      break;
    case 10:
      Mcp4261_U3.wiper2(sss.settings[i]);
      Serial.print("U3-P2 (J10-5): ");
      Serial.println(sss.settings[i]);
      break;
    case 11:
      Mcp4261_U3.wiper3(sss.settings[i]);
      Serial.print("U3-P3 (J10-10): ");
      Serial.println(sss.settings[i]);
      break;
    case 12:
      Mcp4261_U4.wiper0(sss.settings[i]);
      Serial.print("U4-P0 (J24-15): ");
      Serial.println(sss.settings[i]);
      break;
    case 13:
      Mcp4261_U4.wiper1(sss.settings[i]);
      Serial.print("U4-P1 (J24-16): ");
      Serial.println(sss.settings[i]);
      break;
    case 14:
      Mcp4261_U4.wiper2(sss.settings[i]);
      Serial.print("U4-P2 (J24-17): ");
      Serial.println(sss.settings[i]);
      break;
    case 15:
      Mcp4261_U4.wiper3(sss.settings[i]);
      Serial.print("U4-P3 (J24-18): ");
      Serial.println(sss.settings[i]);
      break;
    case 16:
      Mcp4261_U5.wiper0(sss.settings[i]);
      Serial.print("U5-P0 (J10-7): ");
      Serial.println(sss.settings[i]);
      break;
    case 17:
      Mcp4261_U5.wiper1(sss.settings[i]);
      Serial.print("U5-P1 (J24-9): ");
      Serial.println(sss.settings[i]);
      break;
    case 18:
      Mcp4261_U5.wiper2(sss.settings[i]);
      Serial.print("U5-P2 (J24-10): ");
      Serial.println(sss.settings[i]);
      break;
    case 19:
      Mcp4261_U5.wiper3(sss.settings[i]);
      Serial.print("U5-P3 (J24-14): ");
      Serial.println(sss.settings[i]);
      break;
    case 20:
      setDAC();
      Serial.print("VoutA (J24-19): ");
      Serial.println(sss.settings[i]);
      break;
    case 21:
      setDAC();
      Serial.print("VoutB (J24-20): ");
      Serial.println(sss.settings[i]);
      break;
    case 22:
      setDAC();
      Serial.print("VoutC (J24-21): ");
      Serial.println(sss.settings[i]);
      break;
    case 23:
      setDAC();
      Serial.print("VoutD (J24-22): ");
      Serial.println(sss.settings[i]);
      break;
    case 24:// Only on rev 9 and 8 boards
      pwm_1 = map(sss.settings[i],0,100,0,255);
      analogWrite(PWMPin1,pwm_1); //192 = 3.70 volts on PWM1
      Serial.print("PWM1 (J24-23): ");
      Serial.println(sss.settings[i]);
      break;
    case 25:
      //pwm_2 = map(sss.settings[i],0,100,0,255);
      //analogWrite(PWMPin2,pwm_2); //192 = 3.70 volts on PWM1
      Serial.println("PWM2 (Not Connected)");
      //Serial.println(sss.settings[i]);
      break;
    case 26:
      digitalWrite(J1939Term1Pin,sss.settings[i]);
      Serial.print("J1939 Term 1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 27:
      digitalWrite(J1939Term2Pin,sss.settings[i]);
      Serial.print("J1939 Term 2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 28:
      digitalWrite(CAN2Term1Pin,sss.settings[i]);
      Serial.print("CAN2 Term 1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 29:
      Serial.print("CAN2 Term 2: ");
      Serial.print(sss.settings[i]);
      digitalWrite(CAN2Term2Pin,sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 30:
      digitalWrite(CAN3Term1Pin,sss.settings[i]);
      Serial.print("CAN3 Term 1: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #1 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 31:
      digitalWrite(CAN3Term2Pin,sss.settings[i]);
      Serial.print("CAN3 Term 2: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Terminating Resistor #2 Present");
      else if (sss.settings[i] == 1) Serial.println(" = No Terminating Resistor");
      else Serial.println(" Value out of bounds.");
      break;
    case 32:
      digitalWrite(V2WS1SelectPin,sss.settings[i]);
      Serial.print("V2WS1 (J18-12): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 210 ohms to Vehicle Sense Return (J18-13)");
      else if (sss.settings[i] == 1) Serial.println(" = 8 ohms to Vehicle Supply (J18-18)");
      else Serial.println(" Value out of bounds.");
      break;
    case 33:
      digitalWrite(V2WS2SelectPin,sss.settings[i]);
      Serial.print("V2WS2 (J18-8): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 210 ohms to Vehicle Sense Return (J18-13)");
      else if (sss.settings[i] == 1) Serial.println(" = 8 ohms to Vehicle Supply (J18-18)");
      else Serial.println(" Value out of bounds.");
      break;
    case 34:
      digitalWrite(E2WS1SelectPin,sss.settings[i]);
      Serial.print("E2WS1 (J24-5): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 210 ohms to Ground");
      else if (sss.settings[i] == 1 ) Serial.println(" = 10 ohms to +12V Engine Supply (J24-13)");
      else Serial.println(" Value out of bounds.");
      break;
    case 35:
      digitalWrite(E2WS2SelectPin,sss.settings[i]);
      Serial.print("E2WS2 (J24-6): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 210 ohms to Ground");
      else if (sss.settings[i] == 1) Serial.println(" = 10 ohms to +12V Engine Supply (J24-13)");
      else Serial.println(" Value out of bounds.");
      break;
    case 36:
      digitalWrite(E2WS3SelectPin,sss.settings[i]);
      Serial.print("E2WS3 (J24-7): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 210 ohms to Ground");
      else if (sss.settings[i] == 1) Serial.println(" = 10 ohms to +12V Engine Supply (J24-13)");
      else Serial.println(" Value out of bounds.");
      break;
    case 37:
      digitalWrite(E2WS4SelectPin,sss.settings[i]);
      Serial.print("E2WS4 (J24-8): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = 10 ohms to Ground");
      else if (sss.settings[i] == 1) Serial.println(" = 10 ohms to +12V Engine Supply");
      else Serial.println(" Value out of bounds.");
      break;
    case 38:
      digitalWrite(VccSelectU1_0,sss.settings[i]);
      Serial.print("VccSelectU1_0 (J16-9): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 39:
      digitalWrite(VccSelectU1_1,sss.settings[i]);
      Serial.print("VccSelectU1_1 (J16-10): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 40:
      digitalWrite(VccSelectU1_2,sss.settings[i]);
      Serial.print("VccSelectU1_2 (J16-11):");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 41:
      digitalWrite(VccSelectU1_3,sss.settings[i]);
      Serial.print("VccSelectU1_3 (J16-13): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 42:
      digitalWrite(VccSelectU2_0,sss.settings[i]);
      Serial.print("VccSelectU2_0 (J18-2): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 43:
      digitalWrite(VccSelectU2_1,sss.settings[i]);
      Serial.print("VccSelectU2_1 (J18-3): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 44:
      digitalWrite(VccSelectU2_2,sss.settings[i]);
      Serial.print("VccSelectU2_2 (J18-6): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 45:
      digitalWrite(VccSelectU2_3,sss.settings[i]);
      Serial.print("VccSelectU2_3 (J18-7): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to Vehicle Supply (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = Vehicle Supply is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 46:
      digitalWrite(VccSelectU3_0,sss.settings[i]);
      Serial.print("VccSelectU3_0 (J10-4): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 47:
      digitalWrite(VccSelectU3_1,sss.settings[i]);
      Serial.print("VccSelectU3_1 (J10-9): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 48:
      digitalWrite(VccSelectU3_2,sss.settings[i]);
      Serial.print("VccSelectU3_2 (J10-5): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 49:
      digitalWrite(VccSelectU3_3,sss.settings[i]);
      Serial.print("VccSelectU3_3 (J10-10: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 50:
      digitalWrite(VccSelectU4_0,sss.settings[i]);
      Serial.print("VccSelectU4_0 (J24-15): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 51:
      digitalWrite(VccSelectU4_1,sss.settings[i]);
      Serial.print("VccSelectU4_1 (J24-16): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 52:
      digitalWrite(VccSelectU4_2,sss.settings[i]);
      Serial.print("VccSelectU4_2 (J24-17): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 53:
      digitalWrite(VccSelectU4_3,sss.settings[i]);
      Serial.print("VccSelectU4_3 (J24-18): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 54:
      digitalWrite(VccSelectU5_0,sss.settings[i]);
      Serial.print("VccSelectU5_0 (J10-7): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 55:
      digitalWrite(VccSelectU5_1,sss.settings[i]);
      Serial.print("VccSelectU5_1 (J24-9): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 56:
      digitalWrite(VccSelectU5_2,sss.settings[i]);
      Serial.print("VccSelectU5_2 (J24-10): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 57:
      digitalWrite(VccSelectU5_3,sss.settings[i]);
      Serial.print("VccSelectU5_3 (J24-14): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Pulled Up to VCC (+5V)");
      else if (sss.settings[i] == 1) Serial.println(" = VCC is Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 58:
      digitalWrite(GroundSelectU1_0,sss.settings[i]);
      Serial.print("GroundSelectU1_0 (J16-9): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 59:
      digitalWrite(GroundSelectU1_1,sss.settings[i]);
      Serial.print("GroundSelectU1_1 (J16-10): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 60:
      digitalWrite(GroundSelectU1_2,sss.settings[i]);
      Serial.print("GroundSelectU1_2 (J16-11): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 61:
      digitalWrite(GroundSelectU1_3,sss.settings[i]);
      Serial.print("GroundSelectU1_3 (J16-13): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 62:
      digitalWrite(GroundSelectU2_0,sss.settings[i]);
      Serial.print("GroundSelectU2_0 (J18-2): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 63:
      digitalWrite(GroundSelectU2_1,sss.settings[i]);
      Serial.print("GroundSelectU2_1 (J18-3): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 64:
      digitalWrite(GroundSelectU2_2,sss.settings[i]);
      Serial.print("GroundSelectU2_2 (J18-6): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 65:
      digitalWrite(GroundSelectU2_3,sss.settings[i]);
      Serial.print("GroundSelectU2_3 (J18-7): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to Vehicle Sense Return");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 66:
      digitalWrite(GroundSelectU3_0,sss.settings[i]);
      Serial.print("GroundSelectU3_0 (J10-4): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 67:
      digitalWrite(GroundSelectU3_1,sss.settings[i]);
      Serial.print("GroundSelectU3_1 (J10-9): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 68:
      digitalWrite(GroundSelectU3_2,sss.settings[i]);
      Serial.print("GroundSelectU3_2 (J10-5): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 69:
      digitalWrite(GroundSelectU3_3,sss.settings[i]);
      Serial.print("GroundSelectU3_3 (J10-10): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 70:
      digitalWrite(GroundSelectU4_0,sss.settings[i]);
      Serial.print("GroundSelectU4_0 (J24-15): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 71:
      digitalWrite(GroundSelectU4_1,sss.settings[i]);
      Serial.print("GroundSelectU4_1 (J24-16): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 72:
      digitalWrite(GroundSelectU4_2,sss.settings[i]);
      Serial.print("GroundSelectU4_2 (J24-17): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 73:
      digitalWrite(GroundSelectU4_3,sss.settings[i]);
      Serial.print("GroundSelectU4_3 (J24-18): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 74:
      digitalWrite(GroundSelectU5_0,sss.settings[i]);
      Serial.print("GroundSelectU5_0 (J10-7): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 75:
      digitalWrite(GroundSelectU5_1,sss.settings[i]);
      Serial.print("GroundSelectU5_1 (J24-9): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 76:
      digitalWrite(GroundSelectU5_2,sss.settings[i]);
      Serial.print("GroundSelectU5_2 (J24-10): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 77:
      digitalWrite(GroundSelectU5_3,sss.settings[i]);
      Serial.print("GroundSelectU5_3 (J24-14): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 0) Serial.println(" = Grounded to GND");
      else if (sss.settings[i] == 1) Serial.println(" = Disconnected");
      else Serial.println(" Value out of bounds.");
      break;
    case 78:
      digitalWrite(CAN2FrontEnablePin,sss.settings[i]);
      Serial.print("CAN2 Panel Enable: ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 1) Serial.println(" = Not Connected");
      else if (sss.settings[i] == 0) Serial.println(" = CAN2 on Front Panel Connected");
      else Serial.println(" Value out of bounds.");
      break;
    case 79:
      digitalWrite(Coil1Control,sss.settings[i]);
      Serial.print("Coil 1 Control (J16-16): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 1) Serial.println(" = 10 ohms to 12V");
      else if (sss.settings[i] == 0) Serial.println(" = 210 ohms to Ground");
      else Serial.println(" Value out of bounds.");
      break;
    case 80:
      digitalWrite(Coil2Control,sss.settings[i]);
      Serial.print("Coil 2 Control (J16-15): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 1) Serial.println(" = 10 ohms to 12V");
      else if (sss.settings[i] == 0) Serial.println(" = 210 ohms to Ground");
      else Serial.println(" Value out of bounds.");
      break;
    case 81:
      digitalWrite(Coil3Control,sss.settings[i]);
      Serial.print("Coil 3 Control (J16-14): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 1) Serial.println(" = 10 ohms to 12V");
      else if (sss.settings[i] == 0) Serial.println(" = 210 ohms to Ground");
      else Serial.println(" Value out of bounds.");
      break;
    case 82:
      digitalWrite(Coil4Control,sss.settings[i]);
      Serial.print("Coil 4 Control (J24-24): ");
      Serial.print(sss.settings[i]);
      if (sss.settings[i] == 1) Serial.println(" = 10 ohms to 12V");
      else if (sss.settings[i] == 0) Serial.println(" = 10 ohms to Ground");
      else Serial.println(" Value out of bounds.");
      break;
  }
}  

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
  
  Wire.beginTransmission(DACAddress);
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

  if (flag != 0) 
  {
    Serial.print("Setting DAC over I2C returned error flag of ");
    Serial.println(flag); 
  }
  delay(10);
  digitalWrite(LDACPin,HIGH);
}