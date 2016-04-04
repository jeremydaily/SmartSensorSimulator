/*
  SSSdaughter6.cpp - Library for programming the Smart Sensor Simulator daughteboard rev6
*/
#include "Arduino.h"
#include "mcp_can.h"
#include "SPI.h"
#include "Mcp4261.h"
#include "SSSdaughter6.h"
#include "Wire.h"

MCP4261 Mcp4261_U1 = MCP4261(  CSU1Pin,  10000 );
MCP4261 Mcp4261_U2 = MCP4261(  CSU2Pin,  10000 );
MCP4261 Mcp4261_U3 = MCP4261(  CSU3Pin,  10000 );
MCP4261 Mcp4261_U4 = MCP4261(  CSU4Pin,  10000 );
MCP4261 Mcp4261_U5 = MCP4261(  CSU5Pin,  10000 );

//declare instances of the CAN class
MCP_CAN CAN4 = MCP_CAN(3); // 
MCP_CAN CAN5 = MCP_CAN(77); // 
    
  
void setPinModes()
{    //Initialize pin modes
   
    pinMode(CSCAN4Pin, OUTPUT);
    pinMode(CSCAN5Pin, OUTPUT);
    
    pinMode(CSU1Pin, OUTPUT);
    pinMode(CSU2Pin, OUTPUT);
    pinMode(CSU3Pin, OUTPUT);
    pinMode(CSU4Pin, OUTPUT);
    pinMode(CSU5Pin, OUTPUT);
    pinMode(CSU6Pin, OUTPUT);
    pinMode(CSU7Pin, OUTPUT);
    pinMode(CSU8Pin, OUTPUT);
    pinMode(CSU9Pin, OUTPUT);
    pinMode(CSU10Pin, OUTPUT);
    pinMode(CSU11Pin, OUTPUT);
    pinMode(CSU12Pin, OUTPUT);
    pinMode(CSU13Pin, OUTPUT);
    pinMode(CSU14Pin, OUTPUT);
    pinMode(CSU15Pin, OUTPUT);
    pinMode(CSU16Pin, OUTPUT);
    pinMode(CSU17Pin, OUTPUT);
    pinMode(CSU18Pin, OUTPUT);
    pinMode(CSU19Pin, OUTPUT);
    pinMode(CSU20Pin, OUTPUT);
    pinMode(CSU21Pin, OUTPUT);
    pinMode(CSU22Pin, OUTPUT);
    pinMode(CSU23Pin, OUTPUT);
    pinMode(CSU24Pin, OUTPUT);
    
    pinMode(Select12VPort1Pin, OUTPUT);
    pinMode(Select12VPort2Pin, OUTPUT);
    pinMode(Select12VPort3Pin, OUTPUT);
    pinMode(Select12VPort4Pin, OUTPUT);
    pinMode(Select12VPort7Pin, OUTPUT);
    pinMode(Select12VPort8Pin, OUTPUT);
    pinMode(Select12VPort9Pin, OUTPUT);
    pinMode(Select12VPort10Pin, OUTPUT);
    
    pinMode(ADC0Pin,INPUT);
    pinMode(IgnitionSensePin,INPUT);
    pinMode(Sense12VPin,INPUT);
    
    pinMode(CAN4Term1Pin, OUTPUT);
    pinMode(CAN4Term2Pin, OUTPUT);
    pinMode(CAN5Term1Pin, OUTPUT);
    pinMode(CAN5Term2Pin, OUTPUT);
    pinMode(J1939toCAN4Pin, OUTPUT);
    pinMode(J1939toCAN5Pin, OUTPUT);
    
    pinMode(LDAC0Pin, OUTPUT);
    pinMode(LDAC1Pin, OUTPUT);
    pinMode(LDAC2Pin, OUTPUT);
    
    pinMode(LINCSPin,OUTPUT);
    pinMode(LINWakePin,OUTPUT);
    
    pinMode(PWM1Pin, OUTPUT);
    pinMode(PWM2Pin, OUTPUT);
    pinMode(PWM3Pin, OUTPUT);
    
    pinMode(LowRSelectU1Pin, OUTPUT);
    pinMode(LowRSelectU2Pin, OUTPUT);
    pinMode(LowRSelectU3Pin, OUTPUT);
    pinMode(LowRSelectU4Pin, OUTPUT);
    pinMode(LowRSelectU5Pin, OUTPUT);
    pinMode(LowRSelectU6Pin, OUTPUT);
    pinMode(LowRSelectU7Pin, OUTPUT);
    pinMode(LowRSelectU8Pin, OUTPUT);
    
    pinMode(GroundSelectU9, OUTPUT);
    pinMode(GroundSelectU10, OUTPUT);
    pinMode(GroundSelectU11, OUTPUT);
    pinMode(GroundSelectU12, OUTPUT);
    
    pinMode(U1_U2_12VSelect, OUTPUT);
    pinMode(U3_U4_12VSelect, OUTPUT);
    pinMode(U5_U6_12VSelect, OUTPUT);
    pinMode(U7_U8_12VSelect, OUTPUT);
    pinMode(U9_U10_12VSelect, OUTPUT);
    pinMode(U11_U12_12VSelect, OUTPUT);
    pinMode(U13_U14_12VSelect, OUTPUT);
    pinMode(U15_U16_12VSelect, OUTPUT);
    pinMode(U17_U18_12VSelect, OUTPUT);
    
    
    //Set all pins to their initial values
    digitalWrite(CSCAN4Pin, HIGH);
    digitalWrite(CSCAN5Pin, HIGH);
    
    digitalWrite(CSU1Pin,HIGH);
    digitalWrite(CSU2Pin,HIGH);
    digitalWrite(CSU3Pin,HIGH);
    digitalWrite(CSU4Pin,HIGH);
    digitalWrite(CSU5Pin,HIGH);
    digitalWrite(CSU6Pin,HIGH);
    digitalWrite(CSU7Pin,HIGH);
    digitalWrite(CSU8Pin,HIGH);
    digitalWrite(CSU9Pin,HIGH);
    digitalWrite(CSU10Pin,HIGH);
    digitalWrite(CSU11Pin,HIGH);
    digitalWrite(CSU12Pin,HIGH);
    digitalWrite(CSU13Pin,HIGH);
    digitalWrite(CSU14Pin,HIGH);
    digitalWrite(CSU15Pin,HIGH);
    digitalWrite(CSU16Pin,HIGH);
    digitalWrite(CSU17Pin,HIGH);
    digitalWrite(CSU18Pin,HIGH);
    digitalWrite(CSU19Pin,HIGH);
    digitalWrite(CSU20Pin,HIGH);
    digitalWrite(CSU21Pin,HIGH);
    digitalWrite(CSU22Pin,HIGH);
    digitalWrite(CSU23Pin,HIGH);
    digitalWrite(CSU24Pin,HIGH);
    
    digitalWrite(Select12VPort1Pin,LOW);
    digitalWrite(Select12VPort2Pin,LOW);
    digitalWrite(Select12VPort3Pin,LOW);
    digitalWrite(Select12VPort4Pin,LOW);
    digitalWrite(Select12VPort7Pin,LOW);
    digitalWrite(Select12VPort8Pin,LOW);
    digitalWrite(Select12VPort9Pin,LOW);
    digitalWrite(Select12VPort10Pin,LOW);
    
    digitalWrite(GroundSelectU9,  LOW);
    digitalWrite(GroundSelectU10, LOW);
    digitalWrite(GroundSelectU11, LOW);
    digitalWrite(GroundSelectU12, LOW);

    digitalWrite(LowRSelectU1Pin, LOW);
    digitalWrite(LowRSelectU2Pin, LOW);
    digitalWrite(LowRSelectU3Pin, LOW);
    digitalWrite(LowRSelectU4Pin, LOW);
    digitalWrite(LowRSelectU5Pin, LOW);
    digitalWrite(LowRSelectU6Pin, LOW);
    digitalWrite(LowRSelectU7Pin, LOW);
    digitalWrite(LowRSelectU8Pin, LOW);

    digitalWrite(U1_U2_12VSelect, HIGH);
    digitalWrite(U3_U4_12VSelect, HIGH);
    digitalWrite(U5_U6_12VSelect, HIGH);
    digitalWrite(U7_U8_12VSelect, HIGH);
    digitalWrite(U9_U10_12VSelect, HIGH);
    digitalWrite(U11_U12_12VSelect, HIGH);
    digitalWrite(U13_U14_12VSelect, HIGH);
    digitalWrite(U15_U16_12VSelect, HIGH);
    digitalWrite(U17_U18_12VSelect, HIGH);

 
    digitalWrite(PWM1Pin, LOW);
    digitalWrite(PWM2Pin, LOW);
    digitalWrite(PWM3Pin, LOW);
    
    //Resistor Network Modes
    
    //CAN Termination Resistor Modes
    digitalWrite(J1939toCAN4Pin,LOW);
    digitalWrite(J1939toCAN5Pin,LOW);
    digitalWrite(CAN4Term1Pin,LOW);
    digitalWrite(CAN4Term2Pin,LOW);
    digitalWrite(CAN5Term1Pin,LOW);
    digitalWrite(CAN5Term2Pin,LOW);
    
    
    digitalWrite(LDAC0Pin, LOW);
    digitalWrite(LDAC1Pin, LOW);
    digitalWrite(LDAC2Pin, LOW);
    
    digitalWrite(LINCSPin,   LOW);
    digitalWrite(LINWakePin, LOW);
    
    
 
}


void setupSerial(){
    // Set up Serial Connections
    Serial.begin(115200); // Serial to the USB to UART bridge
    Serial.println("Starting Up...");
    Serial.println("Synercon Technologies Smart Sensor Simulator");
    Serial.println("Program running for the ATmega2560 Processor on the daughterboard.");

    

    Serial.println("Setting up CAN4..."); //J1939
    if(CAN4.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN4 init ok!!");
    else Serial.println("CAN0 init fail!!");
    
    Serial.println("Setting up CAN5..."); //J1939
    if(CAN5.begin(CAN_250KBPS) == CAN_OK) Serial.println("CAN5 init ok!!");
    else Serial.println("CAN0 init fail!!");
    
    Serial2.begin(9600); //J1708 
}


SSS::SSS(){
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

} 

boolean SSS::isIgnitionOn()
{
    return digitalRead(IgnitionSensePin);
}

void SSS::sendCANmessages()
{
  if (isIgnitionOn() && numCANmsgs >0 ){
    for (int i = 0; i < numCANmsgs; i++){
      unsigned long currentMillis = millis();
      if ((currentMillis - previousCANmillis[i]) > CANtxPeriod[i]) { // Do this on time.
        previousCANmillis[i] = currentMillis;
        CAN4.sendMsgBuf(CANIDs[i], 1, 8, CANmessages[i]); 
     
      } //end if
    } // end for
  } // end if
}



void SSS::processCAN4message(){
 Serial.print("CAN3 RX, ");
 rxId = CAN4.getCanId();                    // Get message ID
 Serial.print("ID: ");
 Serial.print(rxId, HEX);
 CAN4.readMsgBuf(&len, buf);
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

/* void SSS::sendComponentInfo(char id[29])
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
       
} */
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
      Serial.print("U1-P0 (J20-13): ");
      Serial.println(sss.settings[i]);
      break;
    case 1:
      Mcp4261_U1.wiper1(sss.settings[i]);
      Serial.print("U1-P1 (J20-14): ");
      Serial.println(sss.settings[i]);
      break;
    case 2:
      Mcp4261_U1.wiper2(sss.settings[i]);
      Serial.print("U1-P2 (J20-15): ");
      Serial.println(sss.settings[i]);
      break;
    case 3:
      Mcp4261_U1.wiper3(sss.settings[i]);
      Serial.print("U1-P3 (J20-16): ");
      Serial.println(sss.settings[i]);
      break;
    case 4:
      Mcp4261_U2.wiper0(sss.settings[i]);
      Serial.print("U2-P0 (J20-17): ");
      Serial.println(sss.settings[i]);
      break;
    case 5:
      Mcp4261_U2.wiper1(sss.settings[i]);
      Serial.print("U2-P1 (J20-18): ");
      Serial.println(sss.settings[i]);
      break;
    case 6:
      Mcp4261_U2.wiper2(sss.settings[i]);
      Serial.print("U2-P2 (J20-19): ");
      Serial.println(sss.settings[i]);
      break;
    case 7:
      Mcp4261_U2.wiper3(sss.settings[i]);
      Serial.print("U2-P3 (J20-20): ");
      Serial.println(sss.settings[i]);
      break;
    case 8:
      Mcp4261_U3.wiper0(sss.settings[i]);
      Serial.print("U3-P0 (J22-15): ");
      Serial.println(sss.settings[i]);
      break;
    case 9:
      Mcp4261_U3.wiper1(sss.settings[i]);
      Serial.print("U3-P1 (J22-16): ");
      Serial.println(sss.settings[i]);
      break;
    case 10:
      Mcp4261_U3.wiper2(sss.settings[i]);
      Serial.print("U3-P2 (J22-17): ");
      Serial.println(sss.settings[i]);
      break;
    case 11:
      Mcp4261_U3.wiper3(sss.settings[i]);
      Serial.print("U3-P3 (J22-18): ");
      Serial.println(sss.settings[i]);
      break;
    case 12:
      Mcp4261_U4.wiper0(sss.settings[i]);
      Serial.print("U4-P0 (J22-19): ");
      Serial.println(sss.settings[i]);
      break;
    case 13:
      Mcp4261_U4.wiper1(sss.settings[i]);
      Serial.print("U4-P1 (J22-20): ");
      Serial.println(sss.settings[i]);
      break;
    case 14:
      Mcp4261_U4.wiper2(sss.settings[i]);
      Serial.print("U4-P2 (J22-21): ");
      Serial.println(sss.settings[i]);
      break;
    case 15:
      Mcp4261_U4.wiper3(sss.settings[i]);
      Serial.print("U4-P3 (J22-22): ");
      Serial.println(sss.settings[i]);
      break;
    case 16:
      Mcp4261_U5.wiper0(sss.settings[i]);
      Serial.print("U5-P0 (J12-12): ");
      Serial.println(sss.settings[i]);
      break;
    case 17:
      Mcp4261_U5.wiper1(sss.settings[i]);
      Serial.print("U5-P1 (J12-11): ");
      Serial.println(sss.settings[i]);
      break;
    case 18:
      Mcp4261_U5.wiper2(sss.settings[i]);
      Serial.print("U5-P2 (J12-10): ");
      Serial.println(sss.settings[i]);
      break;
    case 19:
      Mcp4261_U5.wiper3(sss.settings[i]);
      Serial.print("U5-P3 (J12-9): ");
      Serial.println(sss.settings[i]);
      break;
    
  }
}  

void setDAC(){  //Settings are in millivolts.
  //digitalWrite(LDACPin,LOW);
  for (int j=20; j<32; j++)
  {
    if (sss.settings[j]>5000) sss.settings[j]=5000;
    if (sss.settings[j]<0) sss.settings[j]=0;
  }
    
  int VoutA = map(sss.settings[20],0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  int VoutB = map(sss.settings[21],0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  int VoutC = map(sss.settings[22],0,3606,0,3000); //0x08BB = 2.682V on VoutC
  int VoutD = map(sss.settings[23],0,3606,0,3000); //0x0ABB = 3.298V on VoutD
  int VoutE = map(sss.settings[24],0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  int VoutF = map(sss.settings[25],0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  int VoutG = map(sss.settings[26],0,3606,0,3000); //0x08BB = 2.682V on VoutC
  int VoutH = map(sss.settings[27],0,3606,0,3000); //0x0ABB = 3.298V on VoutD
  int VoutI = map(sss.settings[28],0,3606,0,3000); //0x0BBB or 3003 = 3.616V on VoutA
  int VoutJ = map(sss.settings[29],0,3606,0,3000); //0x06BB or 1721 = 2.074V on VoutB
  int VoutK = map(sss.settings[30],0,3606,0,3000); //0x08BB = 2.682V on VoutC
  int VoutL = map(sss.settings[31],0,3606,0,3000); //0x0ABB = 3.298V on VoutD

  VoutA = constrain(VoutA,0,4095);
  VoutB = constrain(VoutB,0,4095);
  VoutC = constrain(VoutC,0,4095);
  VoutD = constrain(VoutD,0,4095);
  VoutE = constrain(VoutE,0,4095);
  VoutF = constrain(VoutF,0,4095);
  VoutG = constrain(VoutG,0,4095);
  VoutH = constrain(VoutH,0,4095);
  VoutI = constrain(VoutI,0,4095);
  VoutJ = constrain(VoutJ,0,4095);
  VoutK = constrain(VoutK,0,4095);
  VoutL = constrain(VoutL,0,4095);
  
  Wire.beginTransmission(0x60);
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
    Serial.print("Setting DAC 0x60 over I2C returned error flag of ");
    Serial.println(flag); 
  }
  delay(1);

  Wire.beginTransmission(0x61);
  Wire.write(byte(0x50));             
  Wire.write(highByte(VoutE));             
  Wire.write(lowByte(VoutE));             
  Wire.write(highByte(VoutF));             
  Wire.write(lowByte(VoutF));             
  Wire.write(highByte(VoutG));             
  Wire.write(lowByte(VoutG));             
  Wire.write(highByte(VoutH));             
  Wire.write(lowByte(VoutH));             
  int flag1 = Wire.endTransmission(); 

  if (flag1 != 0) 
  {
    Serial.print("Setting DAC 0x61 over I2C returned error flag of ");
    Serial.println(flag1); 
  }
 
 delay(1);

 
  Wire.beginTransmission(0x62);
  Wire.write(byte(0x50));             
  Wire.write(highByte(VoutI));             
  Wire.write(lowByte(VoutI));             
  Wire.write(highByte(VoutJ));             
  Wire.write(lowByte(VoutJ));             
  Wire.write(highByte(VoutK));             
  Wire.write(lowByte(VoutK));             
  Wire.write(highByte(VoutL));             
  Wire.write(lowByte(VoutL));             
  int flag2 = Wire.endTransmission(); 

  if (flag2 != 0) 
  {
    Serial.print("Setting DAC 0x62 over I2C returned error flag of ");
    Serial.println(flag2); 
  }
// delay(1);
  //digitalWrite(LDACPin,HIGH);
}